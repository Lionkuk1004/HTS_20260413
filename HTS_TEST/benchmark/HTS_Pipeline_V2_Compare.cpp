// =========================================================================
// HTS_Pipeline_V2_Compare.cpp
// 위치: HTS_TEST/benchmark/ — PC 전용
//
// 외과(동적 프랙탈 와이어, HTS_Pipeline_V2_Dispatcher) + 내과(V400 64칩 FEC)
// 통합 재밍 시뮬레이션:
//   Tx: payload → Encode64(_IR) → Walsh int16 → 바이트 직렬
//       → 슬랩별 심볼 블록 Scatter (nc*4바이트 단위, FWHT 칩 연속 유지)
//       → 와이어 도메인 물리 채널(AWGN/BARRAGE/CW/EMP)
//   Rx: 와이어 → 슬랩별 Gather → 역직렬 → Decode64_A / Decode64_IR
//
// 대조군(Battle):
//   Fixed  : 슬랩별 memcpy + Chase HARQ
//   Dynamic: 슬랩당 Begin_Frame 후 Mapper.Forward(sl) 정렬 순열로 심볼 블록 재배치 + IR-HARQ
//   (바이트 단위 Fractal_Scatter_Tx 는 LE int16·칩 경계를 깨므로 이 벤치에서는 사용하지 않음)
//
// 스케일: HTS_Fractal_Channel_Compare.cpp 의 V400 하네스와 동일
//   kV400DoubleRxToIq16 (= kFecRxDoubleToIq16), fec_v400_pg_ratio(64)=sqrt(2)
//   BPS=4, NSYM64=172, C64=64 (HTS_FEC_SIMULATE_M4_RAM_LAYOUT)
//
// 슬랩: kMapperDomain(4096)바이트/슬랩 = (4096/(nc*4)) 심볼. 순열 인덱스 sl 은 슬랩 내
//       심볼 번호(0..M-1); 칩 바이트는 nc*4 덩어리로만 이동.
//       Begin_Frame: frame_counter=(블록·슬랩), harq_round 별도 (이중 시프트 금지).
//
// 빌드: HTS_Fractal_Channel_Compare_Run 에 TU 링크 + Pipeline_V2 + Adaptive_BPS
// =========================================================================
#if defined(__arm__) || defined(__TARGET_ARCH_ARM) || \
    defined(__TARGET_ARCH_THUMB) || defined(__ARM_ARCH)
#error "[HTS_FATAL] HTS_Pipeline_V2_Compare — PC 전용"
#endif

#include "HTS_Channel_Physics.h"
#include "HTS_FEC_HARQ.hpp"
#include "HTS_Pipeline_V2_Dispatcher.h"
#include "HTS_RF_Metrics.h"
#include "HTS_Adaptive_BPS_Controller.h"
#include "HTS_V400_Dispatcher.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <vector>

using FEC = ProtectedEngine::FEC_HARQ;
using Pipeline = ProtectedEngine::HTS_Pipeline_V2_Dispatcher;

static constexpr int kNumBlocks = 13;
static constexpr int kFecHarqMaxRounds = 32;
static constexpr double kFecHarqRttMs = 8.0;
static constexpr int16_t kFecWalshAmp = 2000;
static constexpr double kV400WalshAmpD = static_cast<double>(kFecWalshAmp);
static constexpr double kSoftSpreadGain = 128.0;
static constexpr double kSoftBaseNoiseSigma = 0.01;
/// HTS_Fractal_Channel_Compare 와 동일 — 스프레드↔int16 환산
constexpr double kV400DoubleRxToIq16 = kV400WalshAmpD / kSoftSpreadGain;

static constexpr double kV400BarrageHarnessDiv = 7.72;

static inline double fec_v400_pg_ratio(int nc) noexcept
{
    return std::sqrt(static_cast<double>(nc) / 32.0);
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

static inline int16_t fec_clamp_i16_from_double(double v) noexcept
{
    long r = std::lround(v);
    if (r > 32767L) r = 32767L;
    if (r < -32768L) r = -32768L;
    return static_cast<int16_t>(r);
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
    for (int j = 0; j < FEC::C64; ++j) {
        const uint32_t p = fec_popc32(static_cast<uint32_t>(sym) &
            static_cast<uint32_t>(j)) & 1u;
        const int16_t ch = static_cast<int16_t>(
            ampi * (1 - 2 * static_cast<int32_t>(p)));
        oI[j] = ch;
        oQ[j] = ch;
    }
}

static void store_le16(uint8_t* d, int16_t v) noexcept
{
    const uint16_t u = static_cast<uint16_t>(v);
    d[0] = static_cast<uint8_t>(u & 0xFFu);
    d[1] = static_cast<uint8_t>((u >> 8u) & 0xFFu);
}

static int16_t load_le16(const uint8_t* d) noexcept
{
    const uint16_t u = static_cast<uint16_t>(d[0]) |
        (static_cast<uint16_t>(d[1]) << 8u);
    return static_cast<int16_t>(u);
}

/// 와이어 바이트 버퍼(직렬 IQ)에 물리 채널 — 슬랩 순열 후에도 에너지 스케일 유지
static void apply_awgn_wire_bytes(uint8_t* buf, size_t byte_len,
    double snr_db, std::mt19937& rng)
{
    const double snr_linear = std::pow(10.0, snr_db / 10.0);
    const double signal_power = kSoftSpreadGain * kSoftSpreadGain;
    const double noise_sigma =
        std::sqrt(signal_power / snr_linear) / std::sqrt(2.0);
    std::normal_distribution<double> awgn(0.0, noise_sigma);
    for (size_t off = 0; off + 1u < byte_len; off += 2u) {
        int16_t v = load_le16(buf + off);
        const double tx_norm =
            static_cast<double>(v) / kV400WalshAmpD;
        const double rd = tx_norm * kSoftSpreadGain + awgn(rng);
        const double out = rd * kV400DoubleRxToIq16;
        store_le16(buf + off, fec_clamp_i16_from_double(out));
    }
}

static void apply_barrage_wire_bytes(uint8_t* buf, size_t byte_len,
    double js_db, int nc, std::mt19937& rng)
{
    const double js_linear = std::pow(10.0, js_db / 10.0);
    const double jam_sigma =
        (std::sqrt(js_linear) * kSoftSpreadGain)
        / fec_v400_barrage_denominator(js_db, nc);
    std::normal_distribution<double> jam(0.0, jam_sigma);
    std::normal_distribution<double> base_noise(0.0, kSoftBaseNoiseSigma);
    for (size_t off = 0; off + 1u < byte_len; off += 2u) {
        int16_t v = load_le16(buf + off);
        const double tx_norm =
            static_cast<double>(v) / kV400WalshAmpD;
        const double rd =
            tx_norm * kSoftSpreadGain + jam(rng) + base_noise(rng);
        store_le16(buf + off, fec_clamp_i16_from_double(rd * kV400DoubleRxToIq16));
    }
}

// EMP: 4바이트(I+Q)=1칩. 파괴: int16 도메인 균일 |v|∈[5000,32767](IR erasure ~30000 경계).
// 정상: Walsh amp(≈2000) 근처 유지 + int16 환경 잡음(강도는 파괴율에 비례).
static void apply_emp_wire_bytes(uint8_t* buf, size_t byte_len,
    double destroy_pct, std::mt19937& rng)
{
    static constexpr double kEmpPercentScale = 0.01;
    static constexpr double kEmpI16Lo = 5000.0;
    static constexpr double kEmpI16Span = 27767.0;
    const double destroy_rate = destroy_pct * kEmpPercentScale;
    const double env_sigma_i16 = 8.0 + destroy_pct * 0.35;
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    std::normal_distribution<double> emp_env_i16(0.0, env_sigma_i16);

    for (size_t off = 0; off + 3u < byte_len; off += 4u) {
        if (u01(rng) < destroy_rate) {
            const double sign01 =
                static_cast<double>(static_cast<uint32_t>(rng()) & 1u);
            const double amp_sign = 1.0 - 2.0 * sign01;
            const double emp_amp_i16 =
                kEmpI16Lo + u01(rng) * kEmpI16Span;
            const int16_t v = static_cast<int16_t>(
                amp_sign > 0.0
                    ? std::min(emp_amp_i16, 32767.0)
                    : std::max(-emp_amp_i16, -32768.0));
            store_le16(buf + off, v);
            store_le16(buf + off + 2u, v);
        }
        else {
            int16_t vI = load_le16(buf + off);
            store_le16(buf + off,
                fec_clamp_i16_from_double(
                    static_cast<double>(vI) + emp_env_i16(rng)));
            int16_t vQ = load_le16(buf + off + 2u);
            store_le16(buf + off + 2u,
                fec_clamp_i16_from_double(
                    static_cast<double>(vQ) + emp_env_i16(rng)));
        }
    }
}

static void apply_cw_wire_bytes(uint8_t* buf, size_t byte_len,
    double js_db, int nsym, int nc, std::mt19937& rng)
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

    for (size_t off = 0; off + 1u < byte_len; off += 2u) {
        const int chip_ord = static_cast<int>((off >> 1u) %
            static_cast<unsigned>(nsym * nc));
        const int c = chip_ord % nc;
        const int s = chip_ord / nc;
        (void)s;
        int16_t v = load_le16(buf + off);
        double rd = static_cast<double>(v) / kV400WalshAmpD * kSoftSpreadGain;
        const size_t iu = static_cast<size_t>(c);
        if (iu >= lo && iu < hi) {
            const double t =
                static_cast<double>(iu + cw_width - cw_center) * inv_span;
            rd += cw_amp_rd * std::sin(kSinOmega * t + cw_phase);
        }
        rd += base_noise(rng);
        store_le16(buf + off,
            fec_clamp_i16_from_double(rd * kV400DoubleRxToIq16));
    }
}

static void apply_wire_channel(uint8_t* buf, size_t byte_len,
    HTS_Core::Physics::ParametricChannel ch_type,
    double ch_intensity, int nsym, int nc, std::mt19937& rng)
{
    switch (ch_type) {
    case HTS_Core::Physics::ParametricChannel::AWGN:
        apply_awgn_wire_bytes(buf, byte_len, ch_intensity, rng);
        break;
    case HTS_Core::Physics::ParametricChannel::BARRAGE:
        apply_barrage_wire_bytes(buf, byte_len, ch_intensity, nc, rng);
        break;
    case HTS_Core::Physics::ParametricChannel::EMP:
        apply_emp_wire_bytes(buf, byte_len, ch_intensity, rng);
        break;
    case HTS_Core::Physics::ParametricChannel::CW:
        apply_cw_wire_bytes(buf, byte_len, ch_intensity, nsym, nc, rng);
        break;
    default:
        break;
    }
}

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

static void compute_ber_bits(const uint8_t* info, const uint8_t* dec,
    bool crc_ok, int& err, int& tot) noexcept
{
    err = 0;
    tot = FEC::MAX_INFO * 8;
    if (!crc_ok) {
        err = tot;
        return;
    }
    for (int by = 0; by < FEC::MAX_INFO; ++by) {
        for (int bit = 0; bit < 8; ++bit) {
            const uint8_t mask = static_cast<uint8_t>(1u << (7 - bit));
            const int o =
                (info[static_cast<size_t>(by)] & mask) != 0 ? 1 : 0;
            const int r =
                (dec[static_cast<size_t>(by)] & mask) != 0 ? 1 : 0;
            if (o != r) { ++err; }
        }
    }
}

/// 논리 IQ 평면 → LE 바이트 (4 * nsym * nc)
static void iq_plane_to_bytes(const int16_t* flat_I, const int16_t* flat_Q,
    int nsym, int nc, uint8_t* out) noexcept
{
    int k = 0;
    for (int s = 0; s < nsym; ++s) {
        for (int c = 0; c < nc; ++c) {
            const int idx = s * nc + c;
            store_le16(out + static_cast<size_t>(k),
                flat_I[static_cast<size_t>(idx)]);
            k += 2;
            store_le16(out + static_cast<size_t>(k),
                flat_Q[static_cast<size_t>(idx)]);
            k += 2;
        }
    }
}

static void bytes_to_iq_plane(const uint8_t* in,
    int16_t* flat_I, int16_t* flat_Q, int nsym, int nc) noexcept
{
    int k = 0;
    for (int s = 0; s < nsym; ++s) {
        for (int c = 0; c < nc; ++c) {
            const int idx = s * nc + c;
            flat_I[static_cast<size_t>(idx)] =
                load_le16(in + static_cast<size_t>(k));
            k += 2;
            flat_Q[static_cast<size_t>(idx)] =
                load_le16(in + static_cast<size_t>(k));
            k += 2;
        }
    }
}

/// Walsh 심볼당 와이어 바이트 (I/Q 각 LE16 × nc)
static inline uint32_t pipeline_v2_symbol_stride_bytes(int nc) noexcept
{
    return static_cast<uint32_t>(nc) * 4u;
}

/// 동적 경로: 슬랩 내 심볼 인덱스 sl 에 대해 Forward(sl) 오름차순 정렬 순서 = 와이어 상 슬롯.
/// (12비트 Feistel 은 0..4095 전역 순열이나, sl<M 에서 Forward 값은 서로 다름 → 정렬로 M-순열)
static void scatter_gather_symbol_slabs_tx(
    bool use_dynamic,
    Pipeline& pipe,
    uint64_t session_id,
    uint32_t block_id,
    uint32_t harq_round,
    const uint8_t* logical_src,
    uint8_t* wire_dst,
    int nsym,
    int nc)
{
    const uint32_t stride = pipeline_v2_symbol_stride_bytes(nc);
    const uint32_t domain = Pipeline::kMapperDomain;
    if (stride == 0u || domain < stride || (domain % stride) != 0u) {
        return;
    }
    const int sym_per_slab = static_cast<int>(domain / stride);
    uint32_t slab = 0u;
    for (int sym_base = 0; sym_base < nsym; sym_base += sym_per_slab) {
        const int m = std::min(sym_per_slab, nsym - sym_base);
        const size_t base_off =
            static_cast<size_t>(sym_base) * static_cast<size_t>(stride);
        const uint32_t frame_counter =
            static_cast<uint32_t>(block_id) * 64u + slab;
        pipe.Begin_Frame(session_id, frame_counter, harq_round);
        const uint8_t* src = logical_src + base_off;
        uint8_t* dst = wire_dst + base_off;
        if (!use_dynamic) {
            std::memcpy(
                dst,
                src,
                static_cast<size_t>(m) * static_cast<size_t>(stride));
        }
        else {
            std::vector<int> ord(static_cast<size_t>(m));
            for (int i = 0; i < m; ++i) {
                ord[static_cast<size_t>(i)] = i;
            }
            std::sort(ord.begin(), ord.end(), [&](int a, int b) {
                return pipe.Mapper().Forward(static_cast<uint32_t>(a))
                    < pipe.Mapper().Forward(static_cast<uint32_t>(b));
            });
            for (int k = 0; k < m; ++k) {
                const int s = ord[static_cast<size_t>(k)];
                std::memcpy(
                    dst + static_cast<size_t>(k) * stride,
                    src + static_cast<size_t>(s) * stride,
                    static_cast<size_t>(stride));
            }
        }
        ++slab;
    }
}

static void scatter_gather_symbol_slabs_rx(
    bool use_dynamic,
    Pipeline& pipe,
    uint64_t session_id,
    uint32_t block_id,
    uint32_t harq_round,
    const uint8_t* wire_src,
    uint8_t* logical_dst,
    int nsym,
    int nc)
{
    const uint32_t stride = pipeline_v2_symbol_stride_bytes(nc);
    const uint32_t domain = Pipeline::kMapperDomain;
    if (stride == 0u || domain < stride || (domain % stride) != 0u) {
        return;
    }
    const int sym_per_slab = static_cast<int>(domain / stride);
    uint32_t slab = 0u;
    for (int sym_base = 0; sym_base < nsym; sym_base += sym_per_slab) {
        const int m = std::min(sym_per_slab, nsym - sym_base);
        const size_t base_off =
            static_cast<size_t>(sym_base) * static_cast<size_t>(stride);
        const uint32_t frame_counter =
            static_cast<uint32_t>(block_id) * 64u + slab;
        pipe.Begin_Frame(session_id, frame_counter, harq_round);
        const uint8_t* src = wire_src + base_off;
        uint8_t* dst = logical_dst + base_off;
        if (!use_dynamic) {
            std::memcpy(
                dst,
                src,
                static_cast<size_t>(m) * static_cast<size_t>(stride));
        }
        else {
            std::vector<int> ord(static_cast<size_t>(m));
            for (int i = 0; i < m; ++i) {
                ord[static_cast<size_t>(i)] = i;
            }
            std::sort(ord.begin(), ord.end(), [&](int a, int b) {
                return pipe.Mapper().Forward(static_cast<uint32_t>(a))
                    < pipe.Mapper().Forward(static_cast<uint32_t>(b));
            });
            for (int k = 0; k < m; ++k) {
                const int s = ord[static_cast<size_t>(k)];
                std::memcpy(
                    dst + static_cast<size_t>(s) * stride,
                    src + static_cast<size_t>(k) * stride,
                    static_cast<size_t>(stride));
            }
        }
        ++slab;
    }
}

static constexpr size_t kMaxFecBytes =
    static_cast<size_t>(FEC::NSYM64) * static_cast<size_t>(FEC::C64) * 4u;

static_assert(kMaxFecBytes <= 65536u,
    "FEC wire slab buffer budget");
static_assert(
    Pipeline::kMapperDomain
        % (static_cast<uint32_t>(FEC::C64) * 4u) == 0u,
    "symbol-block scatter: 4096B slab must hold whole Walsh symbols");

static const double kIntensitySweep[] = {
    5, 10, 15, 20, 25, 30, 35, 40, 45, 50
};
static constexpr int kNumIntensity =
    static_cast<int>(sizeof(kIntensitySweep) / sizeof(kIntensitySweep[0]));

static const char* channel_tag(HTS_Core::Physics::ParametricChannel t)
{
    switch (t) {
    case HTS_Core::Physics::ParametricChannel::AWGN:    return "AWGN";
    case HTS_Core::Physics::ParametricChannel::BARRAGE: return "BARRAGE";
    case HTS_Core::Physics::ParametricChannel::CW:      return "CW";
    case HTS_Core::Physics::ParametricChannel::EMP:     return "EMP";
    default: return "?";
    }
}

static MessageStats run_pipeline_v2_scenario(
    bool use_dynamic_mapper,
    bool use_ir_harq,
    unsigned scenario_seed,
    HTS_Core::Physics::ParametricChannel ch_type,
    double ch_intensity,
    ProtectedEngine::HTS_RF_Metrics& metrics,
    ProtectedEngine::HTS_Adaptive_BPS_Controller& bps_ctrl,
    Pipeline& pipe)
{
    MessageStats msg{};
    msg.total_blocks = kNumBlocks;

    std::mt19937 rng(scenario_seed);

    const int bps = FEC::BPS64;
    const int nc = FEC::C64;

    std::array<uint8_t, kMaxFecBytes> logical_tx{};
    std::array<uint8_t, kMaxFecBytes> wire_buf{};
    std::array<uint8_t, kMaxFecBytes> logical_rx{};

    std::array<uint8_t, static_cast<size_t>(FEC::NSYM64)> syms{};
    std::array<int16_t, static_cast<size_t>(FEC::NSYM64 * FEC::C64)> flat_I{};
    std::array<int16_t, static_cast<size_t>(FEC::NSYM64 * FEC::C64)> flat_Q{};

    FEC::WorkBuf wb{};

    for (int b = 0; b < kNumBlocks; ++b) {
        std::array<uint8_t, static_cast<size_t>(FEC::MAX_INFO)> info{};
        for (int i = 0; i < FEC::MAX_INFO; ++i) {
            info[static_cast<size_t>(i)] = static_cast<uint8_t>(
                0x30u + static_cast<unsigned>((i + b * 3) & 0x0F));
        }
        const int in_len = FEC::MAX_INFO;
        const uint32_t il_seed =
            static_cast<uint32_t>(b) * 31337u + 0xA5A5A5A5u;

        int harq_used = 0;
        bool crc_ok = false;
        std::array<uint8_t, static_cast<size_t>(FEC::MAX_INFO)> decoded{};

        const uint64_t session_id =
            (static_cast<uint64_t>(scenario_seed) << 16)
            | static_cast<uint64_t>(static_cast<uint32_t>(b));

        if (use_ir_harq) {
            FEC::IR_RxState ir{};
            FEC::IR_Init(ir);
            for (int k = 1; k <= kFecHarqMaxRounds; ++k) {
                harq_used = k;
                metrics.current_bps.store(
                    static_cast<uint8_t>(bps), std::memory_order_relaxed);
                const int rv = (k - 1) & 3;
                const int nsym = FEC::Encode64_IR(
                    info.data(), in_len, syms.data(), il_seed, bps, rv, wb);
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
                const uint32_t fec_byte_len =
                    static_cast<uint32_t>(nsym) * static_cast<uint32_t>(nc) * 4u;

                iq_plane_to_bytes(
                    flat_I.data(), flat_Q.data(), nsym, nc, logical_tx.data());

                scatter_gather_symbol_slabs_tx(
                    use_dynamic_mapper,
                    pipe,
                    session_id,
                    static_cast<uint32_t>(b),
                    static_cast<uint32_t>(k - 1u),
                    logical_tx.data(),
                    wire_buf.data(),
                    nsym,
                    nc);

                apply_wire_channel(
                    wire_buf.data(),
                    static_cast<size_t>(fec_byte_len),
                    ch_type,
                    ch_intensity,
                    nsym,
                    nc,
                    rng);

                scatter_gather_symbol_slabs_rx(
                    use_dynamic_mapper,
                    pipe,
                    session_id,
                    static_cast<uint32_t>(b),
                    static_cast<uint32_t>(k - 1u),
                    wire_buf.data(),
                    logical_rx.data(),
                    nsym,
                    nc);

                bytes_to_iq_plane(
                    logical_rx.data(),
                    flat_I.data(), flat_Q.data(), nsym, nc);

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
                        decoded.data(),
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
                metrics.current_bps.store(
                    static_cast<uint8_t>(bps), std::memory_order_relaxed);
                const int nsym = FEC::Encode64_A(
                    info.data(), in_len, syms.data(), il_seed, bps, wb);
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
                const uint32_t fec_byte_len =
                    static_cast<uint32_t>(nsym) * static_cast<uint32_t>(nc) * 4u;

                iq_plane_to_bytes(
                    flat_I.data(), flat_Q.data(), nsym, nc, logical_tx.data());

                scatter_gather_symbol_slabs_tx(
                    use_dynamic_mapper,
                    pipe,
                    session_id,
                    static_cast<uint32_t>(b),
                    static_cast<uint32_t>(k - 1u),
                    logical_tx.data(),
                    wire_buf.data(),
                    nsym,
                    nc);

                apply_wire_channel(
                    wire_buf.data(),
                    static_cast<size_t>(fec_byte_len),
                    ch_type,
                    ch_intensity,
                    nsym,
                    nc,
                    rng);

                scatter_gather_symbol_slabs_rx(
                    use_dynamic_mapper,
                    pipe,
                    session_id,
                    static_cast<uint32_t>(b),
                    static_cast<uint32_t>(k - 1u),
                    wire_buf.data(),
                    logical_rx.data(),
                    nsym,
                    nc);

                bytes_to_iq_plane(
                    logical_rx.data(),
                    flat_I.data(), flat_Q.data(), nsym, nc);

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
                if (FEC::Decode64_A(rx64, decoded.data(), &olen, il_seed, bps, wb)) {
                    crc_ok = true;
                    break;
                }
            }
        }

        if (crc_ok) {
            ++msg.crc_ok_blocks;
        }
        msg.total_harq += harq_used;
        const double rtt_ms = use_ir_harq
            ? ProtectedEngine::HTS_V400_Dispatcher::IR_HARQ_RTT_MS
            : kFecHarqRttMs;
        const double lat = static_cast<double>(harq_used) * rtt_ms;
        if (lat > msg.max_latency_ms) {
            msg.max_latency_ms = lat;
        }

        int err = 0;
        int tot = 0;
        compute_ber_bits(info.data(), decoded.data(), crc_ok, err, tot);
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

    (void)bps_ctrl;
    return msg;
}

static std::filesystem::path resolve_pipeline_v2_csv_path()
{
    namespace fs = std::filesystem;
    try {
        const fs::path cwd = fs::current_path();
        if (cwd.filename() == "HTS_TEST") {
            return cwd / "HTS_Pipeline_V2_Compare_Out.csv";
        }
        if (cwd.filename() == "Release" || cwd.filename() == "Debug") {
            const fs::path test_dir = cwd.parent_path().parent_path();
            if (test_dir.filename() == "HTS_TEST") {
                return test_dir / "HTS_Pipeline_V2_Compare_Out.csv";
            }
        }
    } catch (...) {
    }
    return fs::path("HTS_Pipeline_V2_Compare_Out.csv");
}

void run_pipeline_v2_compare_csv()
{
    ProtectedEngine::HTS_RF_Metrics metrics{};
    ProtectedEngine::HTS_Adaptive_BPS_Controller bps_ctrl(metrics);
    ProtectedEngine::HTS_Pipeline_V2_Dispatcher pipe(metrics, bps_ctrl);

    const HTS_Core::Physics::ParametricChannel scenarios[] = {
        HTS_Core::Physics::ParametricChannel::AWGN,
        HTS_Core::Physics::ParametricChannel::BARRAGE,
        HTS_Core::Physics::ParametricChannel::CW,
        HTS_Core::Physics::ParametricChannel::EMP
    };

    const std::filesystem::path csv_path = resolve_pipeline_v2_csv_path();
    std::ofstream csv(csv_path.string());
    if (!csv) {
        std::fprintf(stderr,
            "Pipeline V2: failed to open %s for write\n",
            csv_path.string().c_str());
        return;
    }

    csv << "# Pipeline_V2: Fixed+Chase vs Dynamic+IR; BPS=4 NSYM64="
        << FEC::NSYM64 << " kV400DoubleRxToIq16=walsh_amp/128\n";
    csv << "battle,channel,intensity,crc_ok_blocks,total_blocks,crc_rate,ber,"
           "total_harq_rounds,avg_harq_per_block,max_latency_ms\n";

    std::printf(
        "\n--- HTS_Pipeline_V2 (Fixed+Chase vs Dynamic+IR) "
        "BPS=%d NSYM64=%d ---\n",
        FEC::BPS64, FEC::NSYM64);
    std::printf(
        "%-22s %-8s %9s %6s %6s %10s %12s %6s %12s %14s\n",
        "battle", "channel", "intens", "crc_ok", "total", "crc_rate",
        "ber", "harq", "avg_harq", "max_lat_ms");

    for (HTS_Core::Physics::ParametricChannel ch : scenarios) {
        for (int si = 0; si < kNumIntensity; ++si) {
            const double inten = kIntensitySweep[static_cast<size_t>(si)];
            const unsigned scen_seed =
                0xC001D00Du
                + static_cast<unsigned>(ch) * 10009u
                + static_cast<unsigned>(inten);

            MessageStats fixed_chase = run_pipeline_v2_scenario(
                false, false, scen_seed, ch, inten, metrics, bps_ctrl, pipe);
            MessageStats dyn_ir = run_pipeline_v2_scenario(
                true, true, scen_seed, ch, inten, metrics, bps_ctrl, pipe);

            auto crc_rate = [](const MessageStats& s) {
                return (s.total_blocks > 0)
                    ? static_cast<double>(s.crc_ok_blocks) /
                        static_cast<double>(s.total_blocks)
                    : 0.0;
            };

            auto emit = [&](const char* label, const MessageStats& s) {
                const double rate = crc_rate(s);
                csv << label << ','
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
                    "%-22s %-8s %9.1f %6d %6d %10.6f %12.6f %6d %12.4f %14.2f\n",
                    label, channel_tag(ch), inten,
                    s.crc_ok_blocks, s.total_blocks, rate,
                    s.ber, s.total_harq, s.avg_harq, s.max_latency_ms);
            };

            emit("fixed_mapper_chase", fixed_chase);
            emit("dynamic_mapper_ir", dyn_ir);
        }
    }

    csv.close();
    std::cout << "Wrote " << csv_path.string()
              << " (Pipeline_V2 + V400 FEC)\n";
}
