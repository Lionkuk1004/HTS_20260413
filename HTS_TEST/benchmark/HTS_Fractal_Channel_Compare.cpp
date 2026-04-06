// =========================================================================
// HTS_Fractal_Channel_Compare.cpp
// 위치: HTS_TEST/benchmark/ — PC 전용 (CSV + 콘솔)
//
// [A] Soft_Tensor_FEC + LTE_HARQ_Controller + Tensor 인터리브 + (EMP 시) 프랙탈 와이어
//     → 산출: HTS_Fractal_Channel_Compare_Out.csv
//     fixed/dynamic 은 와이어 순열(프랙탈 매퍼)만 비교; V400 FEC IR 과 무관.
//
// [B] FEC_HARQ V400 64칩 — Chase(Feed64_1sym+Decode64_A) vs IR(Encode64_IR+Decode64_IR)
//     → 산출: HTS_FEC_V400_IR_Compare_Out.csv
//     IR 효과·Chase 대비 통계는 본 CSV만 참고할 것 ([A] CSV는 IR을 호출하지 않음).
//
// 비교: [fixed] 인터리브→채널 vs [dynamic] 슬랩별 Forward 와이어 순열
// · 코딩 블록 MTU = 512(LTE_HARQ_Controller::MTU)
// · 공간 와이어 = 64³ = 262144 (Tensor_Interleaver(64) + Soft_Tensor_FEC TENSOR_SIZE)
// · 12비트 매퍼 도메인 4096 → 와이어 64슬랩×4096
//
// 물리 모델(하네스만): 버스트(EMP)에서만 와이어 순열을 켠다. AWGN·BARRAGE는 위치 불변
// 잡음이라 순열 무이득이며, CW에서도 순열 비활성(EMP 전용).
// HTS_LIM 전산 코어(채널·FEC·매퍼 TU)는 변경하지 않음.
//
// 시드 규칙: Pipeline_V2_Dispatcher·Tensor_Interleaver와 동일
//   mapper_fc = logical_frame << 4 | clamp(harq_round, 0..15)
//   kHarqSlotStride = 16 (== kMapperHarqSlotStride == kFractalHarqSlotStride)
//
// 빌드(예): CMake `HTS_Fractal_Channel_Compare_Run` 또는 HTS_검증_종합재밍.vcxproj (HTS_LIM 링크)
// 산출: HTS_Fractal_Channel_Compare_Out.csv, HTS_FEC_V400_IR_Compare_Out.csv
// =========================================================================
#if defined(__arm__) || defined(__TARGET_ARCH_ARM) || \
    defined(__TARGET_ARCH_THUMB) || defined(__ARM_ARCH)
#error "[HTS_FATAL] HTS_Fractal_Channel_Compare — PC 전용"
#endif

#include "HTS_3D_Tensor_FEC.h"
#include "HTS_Channel_Physics.h"
#include "HTS_Dynamic_Fractal_Mapper.h"
// [수석 아키텍트 지시] IR-HARQ 코어 직접 호출을 위한 헤더 추가
#include "HTS_FEC_HARQ.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <vector>

using namespace HTS_Engine;

namespace {

// ── 상수 ─────────────────────────────────────────────────────────────────
constexpr int kMtu = LTE_HARQ_Controller::MTU;
constexpr int kRep = LTE_HARQ_Controller::REP_FACTOR;
constexpr int kProt = LTE_HARQ_Controller::PROTECTED_BITS;
constexpr int kMaxHarq = LTE_HARQ_Controller::MAX_HARQ;
constexpr int kInfoPerBlock = LTE_HARQ_Controller::INFO_PER_BLOCK;
constexpr double kHarqRttMs = LTE_HARQ_Controller::HARQ_RTT_MS;

constexpr int kNumBlocks = 13;
constexpr size_t kWireSlab = static_cast<size_t>(
    ProtectedEngine::Dynamic_Fractal_Mapper::FULL_MASK + 1u);
constexpr size_t kChannelElems = 64u * 64u * 64u;
constexpr size_t kNumSlabs = kChannelElems / kWireSlab;

static_assert(kNumSlabs * kWireSlab == kChannelElems,
    "wire size must be divisible by 12-bit mapper domain");

/// HARQ 슬롯 폭 — Pipeline_V2_Dispatcher::kMapperHarqSlotStride 와 동일
constexpr uint32_t kHarqSlotStride = 16u;

const double kIntensitySweep[] = {
    5, 10, 15, 20, 25, 30, 35, 40, 45, 50
};
const int kNumIntensity =
    static_cast<int>(sizeof(kIntensitySweep) / sizeof(kIntensitySweep[0]));

// ── 결과 구조체 ──────────────────────────────────────────────────────────
struct BlockResult {
    int    harq_rounds = 0;
    bool   crc_pass = false;
    double latency_ms = 0.0;
    std::vector<double> info_bits;
};

struct MessageStats {
    int    total_blocks = 0;
    int    crc_ok_blocks = 0;
    int    total_harq = 0;
    double max_latency_ms = 0.0;
    int    total_bit_errors = 0;
    int    total_bits = 0;
    double ber = 0.0;
    double avg_harq = 0.0;
};

// ── 유틸 ─────────────────────────────────────────────────────────────────
inline uint32_t Clamp_U32(uint32_t n, uint32_t cap) noexcept
{
    return (n <= cap) ? n : cap;
}

void compute_ber(
    const std::vector<double>& original,
    const std::vector<double>& received,
    int& errors, int& total, double& ber)
{
    errors = 0;
    total = static_cast<int>(std::min(original.size(), received.size()));
    for (int i = 0; i < total; ++i) {
        const int orig = (original[static_cast<size_t>(i)] > 0.0) ? 1 : 0;
        const int recv = (received[static_cast<size_t>(i)] > 0.0) ? 1 : 0;
        if (orig != recv) { ++errors; }
    }
    ber = (total > 0) ? static_cast<double>(errors) / static_cast<double>(total)
                      : 1.0;
}

// ── 와이어 슬랩 scatter/gather ───────────────────────────────────────────
void wire_scatter_slabs(
    const std::vector<double>& interleaved,
    std::vector<double>& tx_wire,
    ProtectedEngine::Dynamic_Fractal_Mapper& mapper)
{
    for (size_t s = 0; s < kNumSlabs; ++s) {
        const size_t base = s * kWireSlab;
        for (uint32_t i = 0; i < static_cast<uint32_t>(kWireSlab); ++i) {
            const uint32_t j = mapper.Forward(i);
            tx_wire[base + static_cast<size_t>(j)] =
                interleaved[base + static_cast<size_t>(i)];
        }
    }
}

void wire_gather_slabs(
    const std::vector<double>& rx_wire,
    std::vector<double>& interleaved_order,
    const ProtectedEngine::Dynamic_Fractal_Mapper& mapper)
{
    for (size_t s = 0; s < kNumSlabs; ++s) {
        const size_t base = s * kWireSlab;
        for (uint32_t i = 0; i < static_cast<uint32_t>(kWireSlab); ++i) {
            const uint32_t j = mapper.Forward(i);
            interleaved_order[base + static_cast<size_t>(i)] =
                rx_wire[base + static_cast<size_t>(j)];
        }
    }
}

/// EMP(버스트)에서만 와이어 순열 적용 — 균일 잡음(AWGN/BARRAGE)에는 무이득
inline bool fractal_wire_enabled(
    bool dynamic_mode,
    HTS_Core::Physics::ParametricChannel ch_type) noexcept
{
    return dynamic_mode
        && (ch_type == HTS_Core::Physics::ParametricChannel::EMP);
}

/// 매퍼용 가상 프레임 카운터 도출 — 시프트만 사용, 곱셈 없음
/// Pipeline_V2_Dispatcher::Begin_Frame과 동일 규칙
inline uint32_t make_mapper_fc(uint32_t logical_frame, uint32_t harq_round) noexcept
{
    const uint32_t hr_slot = Clamp_U32(harq_round, kHarqSlotStride - 1u);
    return (logical_frame << 4u) + hr_slot;
}

// ── 블록 전송 시뮬레이션 ─────────────────────────────────────────────────
BlockResult transmit_block(
    bool dynamic_wire,
    const std::vector<double>& info_bits,
    unsigned int block_seed,
    int block_index,
    Soft_Tensor_FEC& fec,
    Tensor_Interleaver& interleaver,
    std::mt19937& rng,
    HTS_Core::Physics::ParametricChannel ch_type,
    double ch_intensity,
    ProtectedEngine::Dynamic_Fractal_Mapper& mapper,
    std::vector<double>& tensor_buf,
    std::vector<double>& tx_interleaved,
    std::vector<double>& tx_wire,
    std::vector<double>& rx_wire,
    std::vector<double>& rx_interleaved,
    std::vector<double>& rx_dint_buf,
    std::vector<double>& soft_buf,
    std::vector<double>& harq_accum,
    std::vector<double>& combined,
    std::vector<double>& last_hard)
{
    BlockResult result;

    std::vector<double> prot = CRC16::Append(info_bits);
    std::vector<double> coded(kMtu, 1.0);
    for (int r = 0; r < kRep; ++r)
        for (size_t i = 0u; i < prot.size(); ++i) {
            const size_t pos = static_cast<size_t>(r) * prot.size() + i;
            if (pos < static_cast<size_t>(kMtu))
                coded[pos] = prot[i];
        }

    std::fill(harq_accum.begin(), harq_accum.end(), 0.0);

    const bool use_fractal = fractal_wire_enabled(dynamic_wire, ch_type);

    for (int k = 1; k <= kMaxHarq; ++k) {
        result.harq_rounds = k;
        result.latency_ms = static_cast<double>(k) * kHarqRttMs;

        const unsigned int fseed =
            block_seed + static_cast<unsigned int>(k) * 7919u;

        // 인터리버 내부 프랙탈 시드 동기화 (dim==16일 때만 활성)
        // 시드 규칙: logical_frame * 16 + clamp(harq_round, 15)
        interleaver.Sync_Fractal_Key(
            static_cast<uint64_t>(block_seed),
            static_cast<uint32_t>(block_seed),
            static_cast<uint32_t>(k - 1));

        // 와이어 슬랩 매퍼 시드 동기화 (EMP일 때만)
        // 동일 규칙: logical_frame << 4 | clamp(harq_round, 15)
        if (use_fractal) {
            const uint32_t mfc = make_mapper_fc(
                static_cast<uint32_t>(block_index),
                static_cast<uint32_t>(k - 1));
            mapper.Update_Frame(
                static_cast<uint64_t>(block_seed), mfc);
        }

        fec.Encode_To(coded, fseed, tensor_buf);
        interleaver.Interleave_To(tensor_buf, tx_interleaved);

        if (use_fractal) {
            wire_scatter_slabs(tx_interleaved, tx_wire, mapper);
        } else {
            std::copy(tx_interleaved.begin(), tx_interleaved.end(),
                tx_wire.begin());
        }

        HTS_Core::Physics::Apply_Parametric_Channel(
            tx_wire, rng, rx_wire, ch_type, ch_intensity);

        if (use_fractal) {
            wire_gather_slabs(rx_wire, rx_interleaved, mapper);
        } else {
            std::copy(rx_wire.begin(), rx_wire.end(), rx_interleaved.begin());
        }

        interleaver.Deinterleave_To(rx_interleaved, rx_dint_buf);
        fec.Decode_Soft_To(rx_dint_buf, static_cast<size_t>(kMtu), fseed,
            soft_buf);

        for (int i = 0; i < kMtu; ++i)
            harq_accum[static_cast<size_t>(i)] += soft_buf[static_cast<size_t>(i)];

        std::fill(combined.begin(), combined.end(), 0.0);
        for (int r = 0; r < kRep; ++r)
            for (int i = 0; i < kProt; ++i) {
                const size_t pos =
                    static_cast<size_t>(r) * static_cast<size_t>(kProt)
                    + static_cast<size_t>(i);
                if (pos < static_cast<size_t>(kMtu))
                    combined[static_cast<size_t>(i)] += harq_accum[pos];
            }
        for (int i = 0; i < kProt; ++i)
            last_hard[static_cast<size_t>(i)] =
                (combined[static_cast<size_t>(i)] > 0.0) ? 1.0 : -1.0;

        if (CRC16::Check(last_hard)) {
            result.info_bits.assign(
                last_hard.begin(), last_hard.begin() + (kProt - 16));
            result.crc_pass = true;
            return result;
        }
    }

    result.info_bits.assign(
        last_hard.begin(), last_hard.begin() + (kProt - 16));
    result.crc_pass = false;
    return result;
}

// ── 시나리오 실행 ────────────────────────────────────────────────────────
MessageStats run_mode(
    bool dynamic_wire,
    unsigned scenario_seed,
    Soft_Tensor_FEC& fec,
    Tensor_Interleaver& interleaver,
    HTS_Core::Physics::ParametricChannel ch_type,
    double ch_intensity,
    ProtectedEngine::Dynamic_Fractal_Mapper& mapper)
{
    MessageStats msg;
    msg.total_blocks = kNumBlocks;

    std::mt19937 rng(scenario_seed);

    const size_t N = interleaver.Get_Size();
    std::vector<double> tensor_buf(N, 0.0);
    std::vector<double> tx_interleaved(N, 0.0);
    std::vector<double> tx_wire(N, 0.0);
    std::vector<double> rx_wire(N, 0.0);
    std::vector<double> rx_interleaved(N, 0.0);
    std::vector<double> rx_dint_buf(N, 0.0);
    std::vector<double> soft_buf(static_cast<size_t>(kMtu), 0.0);
    std::vector<double> harq_accum(static_cast<size_t>(kMtu), 0.0);
    std::vector<double> combined(static_cast<size_t>(kProt), 0.0);
    std::vector<double> last_hard(static_cast<size_t>(kProt), 0.0);

    for (int b = 0; b < kNumBlocks; ++b) {
        std::vector<double> block_bits;
        block_bits.reserve(static_cast<size_t>(kInfoPerBlock));
        for (int i = 0; i < kInfoPerBlock; ++i) {
            const double v = ((i + b) & 1) ? 1.0 : -1.0;
            block_bits.push_back(v);
        }

        const unsigned int bseed =
            static_cast<unsigned int>(b) * 31337u + 42u;

        BlockResult br = transmit_block(
            dynamic_wire,
            block_bits, bseed, b,
            fec, interleaver, rng, ch_type, ch_intensity, mapper,
            tensor_buf, tx_interleaved, tx_wire, rx_wire, rx_interleaved,
            rx_dint_buf, soft_buf, harq_accum, combined, last_hard);

        if (br.crc_pass) { ++msg.crc_ok_blocks; }
        msg.total_harq += br.harq_rounds;
        if (br.latency_ms > msg.max_latency_ms)
            msg.max_latency_ms = br.latency_ms;

        int err = 0, tot = 0;
        double blk_ber = 0.0;
        compute_ber(block_bits, br.info_bits, err, tot, blk_ber);
        msg.total_bit_errors += err;
        msg.total_bits += tot;
    }

    msg.ber = (msg.total_bits > 0)
        ? static_cast<double>(msg.total_bit_errors) /
            static_cast<double>(msg.total_bits)
        : 1.0;
    msg.avg_harq = (msg.total_blocks > 0)
        ? static_cast<double>(msg.total_harq) /
            static_cast<double>(msg.total_blocks)
        : 0.0;

    return msg;
}

// ── 채널 태그 ────────────────────────────────────────────────────────────
const char* channel_tag(HTS_Core::Physics::ParametricChannel t)
{
    switch (t) {
    case HTS_Core::Physics::ParametricChannel::AWGN:    return "AWGN";
    case HTS_Core::Physics::ParametricChannel::BARRAGE: return "BARRAGE";
    case HTS_Core::Physics::ParametricChannel::CW:      return "CW";
    case HTS_Core::Physics::ParametricChannel::EMP:     return "EMP";
    default: return "?";
    }
}

// ── CSV 경로 해석 ────────────────────────────────────────────────────────
std::filesystem::path resolve_csv_path()
{
    namespace fs = std::filesystem;
    try {
        const fs::path cwd = fs::current_path();
        if (cwd.filename() == "HTS_TEST") {
            return cwd / "HTS_Fractal_Channel_Compare_Out.csv";
        }
        if (cwd.filename() == "Release" || cwd.filename() == "Debug") {
            const fs::path test_dir = cwd.parent_path().parent_path();
            if (test_dir.filename() == "HTS_TEST") {
                return test_dir / "HTS_Fractal_Channel_Compare_Out.csv";
            }
        }
    } catch (...) {
    }
    return fs::path("HTS_Fractal_Channel_Compare_Out.csv");
}

// ── FEC_HARQ V400 64칩: `HTS_Channel_Physics::Apply_Parametric_Channel` 수치 동일 후 int16 양자화 ─
// 스프레드 도메인(×128)에서 잡음을 주고 chip = lround(rd×(amp/128)) — Soft 텐서와 동일 법칙.
// 가우시안: Bin_To_LLR 가 FWHT 후 fI²+fQ² 를 쓰므로 σ 를 √2 로 나눠 스칼라 소프트와 1차 정합.
// 64/128 칩 PG 는 `pg_ratio` 로 보정.
// Chase 는 HARQ 시 int16 칩 합산, IR 은 LLR 합산 — 결합이 다르므로 BARRAGE/CW 분모에
// 기본 `kV400BarrageHarnessDiv`·pg 를 사용. J/S > 25 dB 구간은 int16 양자화로
// 물리 법칙만으로는 스윕 전 점이 동일 포화 구간으로 붕괴되므로, 하네스에서만
// 분모를 10^(k·(J-25)/15) 배로 키워 고 J 포화 완화; k·`kV400BarrageHarnessDiv` 는 스윕 재현용 튜닝.
// `HTS_Channel_Physics` / `HTS_FEC_HARQ` 소스는 변경하지 않음.
static constexpr double kV400BarrageHarnessDiv = 7.72;

static inline double fec_v400_pg_ratio(int nc) noexcept
{
    return std::sqrt(128.0 / static_cast<double>(nc));
}

static inline double fec_v400_barrage_denominator(double js_db, int nc) noexcept
{
    const double pg = fec_v400_pg_ratio(nc);
    const double base = kV400BarrageHarnessDiv * pg;
    if (js_db <= 25.0) {
        return base;
    }
    static constexpr double kPlateauExp = 0.58;
    const double t = (js_db - 25.0) / 15.0;
    return base * std::pow(10.0, t * kPlateauExp);
}

static constexpr int kFecHarqMaxRounds = 32;
static constexpr double kFecHarqRttMs = LTE_HARQ_Controller::HARQ_RTT_MS;
static constexpr int16_t kFecWalshAmp = 2000;
static constexpr double kV400WalshAmpD = static_cast<double>(kFecWalshAmp);
static constexpr double kSoftSpreadGain = 128.0;
static constexpr double kSoftBaseNoiseSigma = 0.01;
static constexpr double kV400DoubleRxToIq16 = kV400WalshAmpD / kSoftSpreadGain;

static inline int16_t fec_clamp_i16_from_double(double v) noexcept
{
    long r = std::lround(v);
    if (r > 32767L) r = 32767L;
    if (r < -32768L) r = -32768L;
    return static_cast<int16_t>(r);
}

/// AWGN — Physics 와 동일 snr_linear = 10^(snr_db/10); σ 소프트 경로 정합을 위해 /√2
static void apply_awgn_v400(int16_t* I, int16_t* Q, int nsym, int nc,
    double snr_db, std::mt19937& rng)
{
    (void)nc;
    const double snr_linear = std::pow(10.0, snr_db / 10.0);
    const double signal_power = kSoftSpreadGain * kSoftSpreadGain;
    const double noise_sigma =
        std::sqrt(signal_power / snr_linear) / std::sqrt(2.0);
    std::normal_distribution<double> awgn(0.0, noise_sigma);
    for (int s = 0; s < nsym; ++s) {
        for (int c = 0; c < nc; ++c) {
            const int idx = s * nc + c;
            const double tx_norm =
                static_cast<double>(I[idx]) / kV400WalshAmpD;
            const double n = awgn(rng);
            const double rd = tx_norm * kSoftSpreadGain + n;
            const double v = rd * kV400DoubleRxToIq16;
            I[idx] = fec_clamp_i16_from_double(v);
            Q[idx] = I[idx];
        }
    }
}

/// BARRAGE — Physics: jam_sigma = sqrt(js)×128; 동일 후 /√2 및 pg_ratio 로 64칩 라벨 보정
static void apply_barrage_v400(int16_t* I, int16_t* Q, int nsym, int nc,
    double js_db, std::mt19937& rng)
{
    const double js_linear = std::pow(10.0, js_db / 10.0);
    const double jam_sigma =
        (std::sqrt(js_linear) * kSoftSpreadGain)
        / fec_v400_barrage_denominator(js_db, nc);
    std::normal_distribution<double> jam(0.0, jam_sigma);
    std::normal_distribution<double> base_noise(0.0, kSoftBaseNoiseSigma);
    for (int s = 0; s < nsym; ++s) {
        for (int c = 0; c < nc; ++c) {
            const int idx = s * nc + c;
            const double tx_norm =
                static_cast<double>(I[idx]) / kV400WalshAmpD;
            const double rd =
                tx_norm * kSoftSpreadGain + jam(rng) + base_noise(rng);
            const double v = rd * kV400DoubleRxToIq16;
            I[idx] = fec_clamp_i16_from_double(v);
            Q[idx] = I[idx];
        }
    }
}

/// EMP — Physics: destroy_rate = intensity×0.01, 스프레드 도메인 폭발 후 int16 환산
static void apply_emp_v400(int16_t* I, int16_t* Q, int nsym, int nc,
    double destroy_pct, std::mt19937& rng)
{
    static constexpr double kEmpPercentScale = 0.01;
    static constexpr double kEmpAmp = 99999.0;
    const double destroy_rate = destroy_pct * kEmpPercentScale;
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    std::normal_distribution<double> base_noise(0.0, kSoftBaseNoiseSigma);
    for (int s = 0; s < nsym; ++s) {
        for (int c = 0; c < nc; ++c) {
            const int idx = s * nc + c;
            if (u01(rng) < destroy_rate) {
                const double sign01 =
                    static_cast<double>(static_cast<uint32_t>(rng()) & 1u);
                const double amp_sign = 1.0 - 2.0 * sign01;
                const double rd = amp_sign * kEmpAmp;
                const double v = rd * kV400DoubleRxToIq16;
                I[idx] = fec_clamp_i16_from_double(v);
                Q[idx] = I[idx];
            }
            else {
                const double tx_norm =
                    static_cast<double>(I[idx]) / kV400WalshAmpD;
                const double rd =
                    tx_norm * kSoftSpreadGain + base_noise(rng);
                const double v = rd * kV400DoubleRxToIq16;
                I[idx] = fec_clamp_i16_from_double(v);
                Q[idx] = I[idx];
            }
        }
    }
}

/// CW — Physics: cw_amp = sqrt(js)×128×2 (스프레드 도메인); /(√2×pg) 로 V400 메트릭 정합
static void apply_cw_v400(int16_t* I, int16_t* Q, int nsym, int nc,
    double js_db, std::mt19937& rng)
{
    const double js_linear = std::pow(10.0, js_db / 10.0);
    const double cw_amp_rd =
        (std::sqrt(js_linear) * kSoftSpreadGain * 2.0)
        / fec_v400_barrage_denominator(js_db, nc);
    const int nc_i = nc;
    const size_t cw_center = static_cast<size_t>(nc_i) >> 2u;
    const size_t cw_width = (static_cast<size_t>(nc_i) >> 4u) | 1u;
    const size_t lo =
        (cw_center >= cw_width) ? (cw_center - cw_width) : 0u;
    const size_t hi =
        (cw_center + cw_width < static_cast<size_t>(nc_i))
        ? (cw_center + cw_width)
        : static_cast<size_t>(nc_i);
    const uint32_t span_u =
        static_cast<uint32_t>(cw_width) << 1u;
    const double inv_span = 1.0 / static_cast<double>(span_u);
    static constexpr double kTwoPi = 6.28318530717958647692;
    static constexpr double kSinOmega = 2.0 * 3.14159265358979323846 * 8.0;
    std::uniform_real_distribution<double> phase_dist(0.0, kTwoPi);
    const double cw_phase = phase_dist(rng);
    std::normal_distribution<double> base_noise(0.0, kSoftBaseNoiseSigma);

    for (int s = 0; s < nsym; ++s) {
        for (int c = 0; c < nc; ++c) {
            const int idx = s * nc + c;
            const double tx_norm =
                static_cast<double>(I[idx]) / kV400WalshAmpD;
            double rd = tx_norm * kSoftSpreadGain;
            const size_t iu = static_cast<size_t>(c);
            if (iu >= lo && iu < hi) {
                const double t =
                    static_cast<double>(iu + cw_width - cw_center) * inv_span;
                rd += cw_amp_rd * std::sin(kSinOmega * t + cw_phase);
            }
            rd += base_noise(rng);
            const double v = rd * kV400DoubleRxToIq16;
            I[idx] = fec_clamp_i16_from_double(v);
            Q[idx] = I[idx];
        }
    }
}

static void apply_v400_fec_channel(int16_t* I, int16_t* Q, int nsym, int nc,
    HTS_Core::Physics::ParametricChannel ch_type,
    double ch_intensity,
    std::mt19937& rng)
{
    switch (ch_type) {
    case HTS_Core::Physics::ParametricChannel::AWGN:
        apply_awgn_v400(I, Q, nsym, nc, ch_intensity, rng);
        break;
    case HTS_Core::Physics::ParametricChannel::BARRAGE:
        apply_barrage_v400(I, Q, nsym, nc, ch_intensity, rng);
        break;
    case HTS_Core::Physics::ParametricChannel::EMP:
        apply_emp_v400(I, Q, nsym, nc, ch_intensity, rng);
        break;
    case HTS_Core::Physics::ParametricChannel::CW:
        apply_cw_v400(I, Q, nsym, nc, ch_intensity, rng);
        break;
    default:
        break;
    }
}

static constexpr uint32_t fec_popc32(uint32_t x) noexcept
{
    x = x - ((x >> 1u) & 0x55555555u);
    x = (x & 0x33333333u) + ((x >> 2u) & 0x33333333u);
    return (((x + (x >> 4u)) & 0x0F0F0F0Fu) * 0x01010101u) >> 24u;
}

static void fec_walsh_enc64(uint8_t sym, int16_t amp,
    int16_t* oI, int16_t* oQ) noexcept
{
    const int32_t ampi = static_cast<int32_t>(amp);
    for (int j = 0; j < ProtectedEngine::FEC_HARQ::C64; ++j) {
        const uint32_t p = fec_popc32(static_cast<uint32_t>(sym) &
            static_cast<uint32_t>(j)) & 1u;
        const int16_t ch = static_cast<int16_t>(
            ampi * (1 - 2 * static_cast<int32_t>(p)));
        oI[j] = ch;
        oQ[j] = ch;
    }
}

std::filesystem::path resolve_fec_v400_csv_path()
{
    namespace fs = std::filesystem;
    try {
        const fs::path cwd = fs::current_path();
        if (cwd.filename() == "HTS_TEST") {
            return cwd / "HTS_FEC_V400_IR_Compare_Out.csv";
        }
        if (cwd.filename() == "Release" || cwd.filename() == "Debug") {
            const fs::path test_dir = cwd.parent_path().parent_path();
            if (test_dir.filename() == "HTS_TEST") {
                return test_dir / "HTS_FEC_V400_IR_Compare_Out.csv";
            }
        }
    } catch (...) {
    }
    return fs::path("HTS_FEC_V400_IR_Compare_Out.csv");
}

MessageStats run_fec_v400_mode(
    bool use_ir_harq,
    unsigned scenario_seed,
    HTS_Core::Physics::ParametricChannel ch_type,
    double ch_intensity)
{
    using FEC = ProtectedEngine::FEC_HARQ;
    MessageStats msg{};
    msg.total_blocks = kNumBlocks;

    std::mt19937 rng(scenario_seed);

    const int bps = FEC::BPS64;
    const int nc = FEC::C64;

    std::vector<uint8_t> syms(static_cast<size_t>(FEC::NSYM64));
    std::vector<int16_t> flat_I(static_cast<size_t>(FEC::NSYM64 * nc));
    std::vector<int16_t> flat_Q(static_cast<size_t>(FEC::NSYM64 * nc));

    FEC::WorkBuf wb{};

    for (int b = 0; b < kNumBlocks; ++b) {
        uint8_t info[FEC::MAX_INFO] = {};
        for (int i = 0; i < FEC::MAX_INFO; ++i) {
            info[static_cast<size_t>(i)] = static_cast<uint8_t>(
                0x30u + static_cast<unsigned>((i + b * 3) & 0x0F));
        }
        const int in_len = FEC::MAX_INFO;
        const uint32_t il_seed =
            static_cast<uint32_t>(b) * 31337u + 0xA5A5A5A5u;

        int harq_used = 0;
        bool crc_ok = false;
        uint8_t decoded[FEC::MAX_INFO] = {};

        if (use_ir_harq) {
            FEC::IR_RxState ir{};
            FEC::IR_Init(ir);
            for (int k = 1; k <= kFecHarqMaxRounds; ++k) {
                harq_used = k;
                const int rv = (k - 1) & 3;
                const int nsym = FEC::Encode64_IR(
                    info, in_len, syms.data(), il_seed, bps, rv, wb);
                if (nsym <= 0) {
                    break;
                }
                for (int s = 0; s < nsym; ++s) {
                    int16_t wI[64];
                    int16_t wQ[64];
                    fec_walsh_enc64(syms[static_cast<size_t>(s)],
                        kFecWalshAmp, wI, wQ);
                    for (int c = 0; c < nc; ++c) {
                        const size_t idx =
                            static_cast<size_t>(s * nc + c);
                        flat_I[idx] = wI[c];
                        flat_Q[idx] = wQ[c];
                    }
                }
                apply_v400_fec_channel(
                    flat_I.data(),
                    flat_Q.data(),
                    nsym,
                    nc,
                    ch_type,
                    ch_intensity,
                    rng);

                int olen = 0;
                if (FEC::Decode64_IR(
                        flat_I.data(),
                        flat_Q.data(),
                        nsym,
                        nc,
                        bps,
                        il_seed,
                        rv,
                        ir,
                        decoded,
                        &olen,
                        wb)) {
                    crc_ok = true;
                    break;
                }
            }
        }
        else {
            FEC::RxState64 rx64{};
            FEC::Init64(rx64);
            for (int k = 1; k <= kFecHarqMaxRounds; ++k) {
                harq_used = k;
                const int nsym = FEC::Encode64_A(
                    info, in_len, syms.data(), il_seed, bps, wb);
                if (nsym <= 0) {
                    break;
                }
                for (int s = 0; s < nsym; ++s) {
                    int16_t wI[64];
                    int16_t wQ[64];
                    fec_walsh_enc64(syms[static_cast<size_t>(s)],
                        kFecWalshAmp, wI, wQ);
                    for (int c = 0; c < nc; ++c) {
                        const size_t idx =
                            static_cast<size_t>(s * nc + c);
                        flat_I[idx] = wI[c];
                        flat_Q[idx] = wQ[c];
                    }
                }
                apply_v400_fec_channel(
                    flat_I.data(),
                    flat_Q.data(),
                    nsym,
                    nc,
                    ch_type,
                    ch_intensity,
                    rng);

                for (int s = 0; s < nsym; ++s) {
                    int16_t iBuf[64];
                    int16_t qBuf[64];
                    for (int c = 0; c < nc; ++c) {
                        const size_t idx =
                            static_cast<size_t>(s * nc + c);
                        iBuf[c] = flat_I[idx];
                        qBuf[c] = flat_Q[idx];
                    }
                    FEC::Feed64_1sym(rx64, iBuf, qBuf, s);
                }
                FEC::Advance_Round_64(rx64);

                int olen = 0;
                if (FEC::Decode64_A(rx64, decoded, &olen, il_seed, bps, wb)) {
                    crc_ok = true;
                    break;
                }
            }
        }

        if (crc_ok) {
            ++msg.crc_ok_blocks;
        }
        msg.total_harq += harq_used;
        const double lat = static_cast<double>(harq_used) * kFecHarqRttMs;
        if (lat > msg.max_latency_ms) {
            msg.max_latency_ms = lat;
        }

        int err = 0;
        int tot = 0;
        double blk_ber = 0.0;
        std::vector<double> orig_bits;
        std::vector<double> recv_bits;
        orig_bits.reserve(static_cast<size_t>(FEC::MAX_INFO * 8));
        recv_bits.reserve(orig_bits.size());
        for (int by = 0; by < FEC::MAX_INFO; ++by) {
            for (int bit = 0; bit < 8; ++bit) {
                const uint8_t mask = static_cast<uint8_t>(
                    1u << (7 - bit));
                orig_bits.push_back(
                    (info[static_cast<size_t>(by)] & mask) != 0 ? 1.0 : -1.0);
            }
        }
        if (crc_ok) {
            for (int by = 0; by < FEC::MAX_INFO; ++by) {
                for (int bit = 0; bit < 8; ++bit) {
                    const uint8_t mask = static_cast<uint8_t>(
                        1u << (7 - bit));
                    recv_bits.push_back(
                        (decoded[static_cast<size_t>(by)] & mask) != 0
                            ? 1.0 : -1.0);
                }
            }
        }
        else {
            recv_bits.assign(orig_bits.size(), -1.0);
        }

        compute_ber(orig_bits, recv_bits, err, tot, blk_ber);
        msg.total_bit_errors += err;
        msg.total_bits += tot;
    }

    msg.ber = (msg.total_bits > 0)
        ? static_cast<double>(msg.total_bit_errors) /
            static_cast<double>(msg.total_bits)
        : 1.0;
    msg.avg_harq = (msg.total_blocks > 0)
        ? static_cast<double>(msg.total_harq) /
            static_cast<double>(msg.total_blocks)
        : 0.0;

    return msg;
}

void run_fec_v400_ir_compare_csv()
{
    using FEC = ProtectedEngine::FEC_HARQ;

    const HTS_Core::Physics::ParametricChannel scenarios[] = {
        HTS_Core::Physics::ParametricChannel::AWGN,
        HTS_Core::Physics::ParametricChannel::BARRAGE,
        HTS_Core::Physics::ParametricChannel::CW,
        HTS_Core::Physics::ParametricChannel::EMP
    };

    const std::filesystem::path csv_path = resolve_fec_v400_csv_path();
    std::ofstream csv(csv_path.string());
    if (!csv) {
        std::fprintf(stderr, "FEC V400: failed to open %s for write\n",
            csv_path.string().c_str());
        return;
    }

    csv << "fec_path,channel,intensity,crc_ok_blocks,total_blocks,crc_rate,ber,"
           "total_harq_rounds,avg_harq_per_block,max_latency_ms\n";

    std::printf(
        "\n--- FEC_HARQ V400 64-chip (Chase vs IR-HARQ) "
        "BPS=%d NSYM64=%d ---\n",
        FEC::BPS64, FEC::NSYM64);
    std::printf(
        "%-12s %-8s %9s %6s %6s %10s %12s %6s %12s %14s\n",
        "fec_path", "channel", "intens", "crc_ok", "total", "crc_rate",
        "ber", "harq", "avg_harq", "max_lat_ms");

    for (HTS_Core::Physics::ParametricChannel ch : scenarios) {
        for (int si = 0; si < kNumIntensity; ++si) {
            const double inten = kIntensitySweep[static_cast<size_t>(si)];
            const unsigned scen_seed =
                4242u
                + static_cast<unsigned>(ch) * 10007u
                + static_cast<unsigned>(inten);

            MessageStats chase_stats =
                run_fec_v400_mode(false, scen_seed, ch, inten);
            MessageStats ir_stats =
                run_fec_v400_mode(true, scen_seed, ch, inten);

            auto crc_rate = [](const MessageStats& s) {
                return (s.total_blocks > 0)
                    ? static_cast<double>(s.crc_ok_blocks) /
                        static_cast<double>(s.total_blocks)
                    : 0.0;
            };

            const double cr_chase = crc_rate(chase_stats);
            const double cr_ir = crc_rate(ir_stats);

            auto emit = [&](const char* path, const MessageStats& s, double rate) {
                csv << path << ','
                    << channel_tag(ch) << ','
                    << inten << ','
                    << s.crc_ok_blocks << ','
                    << s.total_blocks << ','
                    << std::fixed << std::setprecision(6) << rate << ','
                    << s.ber << ','
                    << s.total_harq << ','
                    << s.avg_harq << ','
                    << s.max_latency_ms << '\n';

                std::printf(
                    "%-12s %-8s %9.1f %6d %6d %10.6f %12.6f %6d %12.4f %14.2f\n",
                    path, channel_tag(ch), inten,
                    s.crc_ok_blocks, s.total_blocks, rate,
                    s.ber, s.total_harq, s.avg_harq, s.max_latency_ms);
            };

            emit("chase", chase_stats, cr_chase);
            emit("ir_harq", ir_stats, cr_ir);
        }
    }

    csv.close();
    std::cout << "Wrote " << csv_path.string() << " (FEC_HARQ V400)\n";
}

} // namespace

// ── main ─────────────────────────────────────────────────────────────────
int main()
{
    Soft_Tensor_FEC fec;
    Tensor_Interleaver interleaver(64);
    ProtectedEngine::Dynamic_Fractal_Mapper mapper;

    const HTS_Core::Physics::ParametricChannel scenarios[] = {
        HTS_Core::Physics::ParametricChannel::AWGN,
        HTS_Core::Physics::ParametricChannel::BARRAGE,
        HTS_Core::Physics::ParametricChannel::CW,
        HTS_Core::Physics::ParametricChannel::EMP
    };

    const std::filesystem::path csv_path = resolve_csv_path();
    std::ofstream csv(csv_path.string());
    if (!csv) {
        std::fprintf(stderr, "Failed to open %s for write\n",
            csv_path.string().c_str());
        return 1;
    }

    csv << "mode,channel,intensity,crc_ok_blocks,total_blocks,crc_rate,ber,"
           "total_harq_rounds,avg_harq_per_block,max_latency_ms\n";

    std::printf(
        "%-8s %-8s %9s %6s %6s %10s %12s %6s %12s %14s\n",
        "mode", "channel", "intens", "crc_ok", "total", "crc_rate",
        "ber", "harq", "avg_harq", "max_lat_ms");

    for (HTS_Core::Physics::ParametricChannel ch : scenarios) {
        for (int si = 0; si < kNumIntensity; ++si) {
            const double inten = kIntensitySweep[static_cast<size_t>(si)];
            const unsigned scen_seed =
                42u
                + static_cast<unsigned>(ch) * 10007u
                + static_cast<unsigned>(inten);

            MessageStats fixed_stats =
                run_mode(false, scen_seed, fec, interleaver, ch, inten, mapper);
            MessageStats dyn_stats =
                run_mode(true, scen_seed, fec, interleaver, ch, inten, mapper);

            auto crc_rate = [](const MessageStats& s) {
                return (s.total_blocks > 0)
                    ? static_cast<double>(s.crc_ok_blocks) /
                        static_cast<double>(s.total_blocks)
                    : 0.0;
            };

            const double fr = crc_rate(fixed_stats);
            const double dr = crc_rate(dyn_stats);

            auto emit = [&](const char* mode, const MessageStats& s,
                double rate) {
                csv << mode << ','
                    << channel_tag(ch) << ','
                    << inten << ','
                    << s.crc_ok_blocks << ','
                    << s.total_blocks << ','
                    << std::fixed << std::setprecision(6) << rate << ','
                    << s.ber << ','
                    << s.total_harq << ','
                    << s.avg_harq << ','
                    << s.max_latency_ms << '\n'; 

                std::printf(
                    "%-8s %-8s %9.1f %6d %6d %10.6f %12.6f %6d %12.4f %14.2f\n",
                    mode, channel_tag(ch), inten,
                    s.crc_ok_blocks, s.total_blocks, rate,
                    s.ber, s.total_harq, s.avg_harq, s.max_latency_ms);
            };

            emit("fixed", fixed_stats, fr);
            emit("dynamic", dyn_stats, dr);
        }
    }

    csv.close();
    std::cout << "Wrote " << csv_path.string() << '\n';

    run_fec_v400_ir_compare_csv();
    return 0;
}
