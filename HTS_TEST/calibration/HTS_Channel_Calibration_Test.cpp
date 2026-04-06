// =========================================================================
// HTS_Channel_Calibration_Test.cpp
// 위치: HTS_TEST/calibration/ — HTS_Calibration_Run.exe 전용 (PC)
//
// CSV 출력(std::ofstream)은 본 타깃에만 존재 — ARM·hts_cs_core 미포함.
//
// 빌드: HTS_TEST/CMakeLists.txt → target HTS_Calibration_Run
// =========================================================================
#if defined(__arm__) || defined(__TARGET_ARCH_ARM) || \
    defined(__TARGET_ARCH_THUMB) || defined(__ARM_ARCH)
#error "[HTS_FATAL] HTS_Channel_Calibration_Test — PC 전용"
#endif

#ifndef HTS_PC_CALIBRATION_RUN
#error "HTS_PC_CALIBRATION_RUN must be defined for this translation unit (CMake)."
#endif

#include "HTS_3D_Tensor_FEC.h"
#include "HTS_AntiJam_Engine.h"
#include "HTS_Channel_Calibration.h"
#include "HTS_Channel_Physics.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>
#include <vector>

using namespace HTS_Engine;

namespace {

constexpr int kMtu = LTE_HARQ_Controller::MTU;
constexpr int kRep = LTE_HARQ_Controller::REP_FACTOR;
constexpr int kProt = LTE_HARQ_Controller::PROTECTED_BITS;
constexpr int kMaxHarq = LTE_HARQ_Controller::MAX_HARQ;
constexpr int kInfoPerBlock = LTE_HARQ_Controller::INFO_PER_BLOCK;
constexpr double kHarqRttMs = LTE_HARQ_Controller::HARQ_RTT_MS;

constexpr unsigned kFixedSeed = 42u;

const double kIntensitySweep[] = {
    5, 10, 15, 20, 25, 30, 35, 40, 45, 50
};
const int kNumIntensity =
    static_cast<int>(sizeof(kIntensitySweep) / sizeof(kIntensitySweep[0]));

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
    ber = (total > 0) ? static_cast<double>(errors) / total : 1.0;
}

using ChannelApplier = void(*)(
    const std::vector<double>& tx,
    std::mt19937& rng,
    std::vector<double>& rx,
    HTS_CalChannelType type,
    double intensity_db);

static void Apply_Pre(
    const std::vector<double>& tx,
    std::mt19937& rng,
    std::vector<double>& rx,
    HTS_CalChannelType type,
    double intensity_db)
{
    HTS_Core::Physics::Apply_Parametric_Channel(
        tx, rng, rx,
        static_cast<HTS_Core::Physics::ParametricChannel>(
            static_cast<std::uint8_t>(type)),
        intensity_db);
}

static void Apply_Post(
    const std::vector<double>& tx,
    std::mt19937& rng,
    std::vector<double>& rx,
    HTS_CalChannelType type,
    double intensity_db)
{
    HTS_ChannelCalibrationOptions opt = HTS_ChannelCalibrationOptions::Default();
    opt.apply_intensity_mapping = true;
    opt.cw_full_tensor = (type == HTS_CalChannelType::CW);
    HTS_Apply_Channel_Calibrated(tx, rng, rx, type, intensity_db, opt);
}

/// 텐서(double) ↔ int16 I/Q 스케일 — V400 64칩 블록 단위 AntiJamEngine::Process 경로
constexpr double kTensorToInt16Scale = 2048.0;

static int16_t double_to_i16_sample(double x) noexcept
{
    const double s = x * kTensorToInt16Scale;
    if (s >= 32767.0) { return 32767; }
    if (s <= -32768.0) { return -32768; }
    return static_cast<int16_t>(static_cast<int32_t>(s));
}

static double i16_to_double_sample(int16_t v) noexcept
{
    return static_cast<double>(v) / kTensorToInt16Scale;
}

/// 인터리브 텐서 전체를 64샘플 블록으로 스캔 — ProtectedEngine::AntiJamEngine (DATA nc=64)
static void RunAntiJamEngine_OnRxTensor(
    std::vector<double>& rx,
    HTS_CalChannelType ch_type,
    double intensity_db) noexcept
{
    using ProtectedEngine::AntiJamEngine;
    // BARRAGE: 3층 미가동이어도 int16 왕복만으로 FEC 입력이 달라지므로(pre 대비 왜곡)
    // 물리 double 텐서를 그대로 둔다. (실장은 int16 경로 + Set_AdaptiveBarrageBypass)
    if (ch_type == HTS_CalChannelType::BARRAGE) {
        return;
    }

    AntiJamEngine aj;
    aj.Reset(64);

    if (ch_type == HTS_CalChannelType::CW) {
        const double js_linear = std::pow(10.0, intensity_db / 10.0);
        const int32_t ja = static_cast<int32_t>(
            std::min(65534.0, std::sqrt(js_linear) * 500.0));
        aj.Seed_CW_Profile(ja, ja);
    }

    constexpr int nc = AntiJamEngine::MAX_NC;
    int16_t I[nc];
    int16_t Q[nc];
    const size_t N = rx.size();
    for (size_t base = 0; base + static_cast<size_t>(nc) <= N;
         base += static_cast<size_t>(nc)) {
        for (int k = 0; k < nc; ++k) {
            const double v = rx[base + static_cast<size_t>(k)];
            I[k] = double_to_i16_sample(v);
            Q[k] = I[k];
        }
        aj.Process(I, Q, nc);
        for (int k = 0; k < nc; ++k) {
            rx[base + static_cast<size_t>(k)] = i16_to_double_sample(I[k]);
        }
    }
}

/// pre(물리) 동일 후 수신 텐서에 AntiJamEngine 적용 — CSV mode `antijam`
static void Apply_Pre_AntiJam(
    const std::vector<double>& tx,
    std::mt19937& rng,
    std::vector<double>& rx,
    HTS_CalChannelType type,
    double intensity_db)
{
    Apply_Pre(tx, rng, rx, type, intensity_db);
    RunAntiJamEngine_OnRxTensor(rx, type, intensity_db);
}

BlockResult transmit_block(
    const std::vector<double>& info_bits,
    unsigned int block_seed,
    Soft_Tensor_FEC& fec,
    Tensor_Interleaver& interleaver,
    std::mt19937& rng,
    HTS_CalChannelType ch_type,
    double ch_intensity,
    ChannelApplier apply_fn,
    std::vector<double>& tensor_buf,
    std::vector<double>& tx_buf,
    std::vector<double>& rx_buf,
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
            size_t pos = static_cast<size_t>(r) * prot.size() + i;
            if (pos < static_cast<size_t>(kMtu))
                coded[pos] = prot[i];
        }

    std::fill(harq_accum.begin(), harq_accum.end(), 0.0);

    for (int k = 1; k <= kMaxHarq; ++k) {
        result.harq_rounds = k;
        result.latency_ms = static_cast<double>(k) * kHarqRttMs;

        const unsigned int fseed =
            block_seed + static_cast<unsigned int>(k) * 7919u;

        interleaver.Sync_Fractal_Key(
            static_cast<uint64_t>(block_seed),
            static_cast<uint32_t>(block_seed),
            static_cast<uint32_t>(k - 1));

        fec.Encode_To(coded, fseed, tensor_buf);
        interleaver.Interleave_To(tensor_buf, tx_buf);
        apply_fn(tx_buf, rng, rx_buf, ch_type, ch_intensity);
        interleaver.Deinterleave_To(rx_buf, rx_dint_buf);
        fec.Decode_Soft_To(rx_dint_buf, kMtu, fseed, soft_buf);

        for (int i = 0; i < kMtu; ++i)
            harq_accum[static_cast<size_t>(i)] += soft_buf[static_cast<size_t>(i)];

        std::fill(combined.begin(), combined.end(), 0.0);
        for (int r = 0; r < kRep; ++r)
            for (int i = 0; i < kProt; ++i) {
                size_t pos = static_cast<size_t>(r) * kProt + i;
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

MessageStats run_full_message(
    const std::string& message,
    Soft_Tensor_FEC& fec,
    Tensor_Interleaver& interleaver,
    std::mt19937& rng,
    HTS_CalChannelType ch_type,
    double ch_intensity,
    ChannelApplier apply_fn)
{
    MessageStats msg;

    std::vector<double> all_bits = Text_Codec::String_To_Bits(message);
    const int num_blocks =
        (static_cast<int>(all_bits.size()) + kInfoPerBlock - 1) / kInfoPerBlock;
    msg.total_blocks = num_blocks;

    const size_t tensor_size = interleaver.Get_Size();
    std::vector<double> tensor_buf(tensor_size, 0.0);
    std::vector<double> tx_buf(tensor_size, 0.0);
    std::vector<double> rx_buf(tensor_size, 0.0);
    std::vector<double> rx_dint_buf(tensor_size, 0.0);
    std::vector<double> soft_buf(kMtu, 0.0);
    std::vector<double> harq_accum(kMtu, 0.0);
    std::vector<double> combined(kProt, 0.0);
    std::vector<double> last_hard(kProt, 0.0);

    for (int b = 0; b < num_blocks; ++b) {
        const int start = b * kInfoPerBlock;
        const int end = std::min(start + kInfoPerBlock,
            static_cast<int>(all_bits.size()));
        std::vector<double> block_bits(
            all_bits.begin() + start,
            all_bits.begin() + end);

        while (static_cast<int>(block_bits.size()) < kInfoPerBlock)
            block_bits.push_back(1.0);

        const unsigned int bseed =
            static_cast<unsigned int>(b) * 31337u + 42u;

        BlockResult br = transmit_block(
            block_bits, bseed, fec, interleaver, rng,
            ch_type, ch_intensity, apply_fn,
            tensor_buf, tx_buf, rx_buf, rx_dint_buf,
            soft_buf, harq_accum, combined, last_hard);

        if (br.crc_pass) { msg.crc_ok_blocks++; }
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

const char* channel_tag(HTS_CalChannelType t) {
    switch (t) {
    case HTS_CalChannelType::AWGN: return "AWGN";
    case HTS_CalChannelType::BARRAGE: return "BARRAGE";
    case HTS_CalChannelType::CW: return "CW";
    case HTS_CalChannelType::EMP: return "EMP";
    }
    return "?";
}

} // namespace

/// CMake 기본 산출: .../HTS_TEST/build-calibration/(Release|Debug)/exe
/// → CSV 는 항상 .../HTS_TEST/HTS_Channel_Calibration_Out.csv 로 기록
static std::filesystem::path resolve_calibration_csv_path()
{
    namespace fs = std::filesystem;
    try {
        const fs::path cwd = fs::current_path();
        if (cwd.filename() == "HTS_TEST") {
            return cwd / "HTS_Channel_Calibration_Out.csv";
        }
        if (cwd.filename() == "Release" || cwd.filename() == "Debug") {
            const fs::path test_dir = cwd.parent_path().parent_path();
            if (test_dir.filename() == "HTS_TEST") {
                return test_dir / "HTS_Channel_Calibration_Out.csv";
            }
        }
    } catch (...) {
    }
    return fs::path("HTS_Channel_Calibration_Out.csv");
}

int main()
{
    const std::string payload =
        "HTS-V400 Tensor Engine Test 2026! [긴급] 고도 15,000ft에서 적대적 재밍 공격 감지. "
        "채널 교정 CSV 하네스 — 시드 42.";

    Soft_Tensor_FEC fec;
    Tensor_Interleaver interleaver(64);

    const HTS_CalChannelType scenarios[] = {
        HTS_CalChannelType::AWGN,
        HTS_CalChannelType::BARRAGE,
        HTS_CalChannelType::CW,
        HTS_CalChannelType::EMP
    };

    const std::filesystem::path csv_file = resolve_calibration_csv_path();
    std::ofstream csv(csv_file.string());
    if (!csv) {
        std::cerr << "Failed to open " << csv_file.string() << " for write\n";
        return 1;
    }

    csv << "mode,channel,intensity,crc_ok_blocks,total_blocks,crc_rate,ber,"
           "total_harq_rounds,avg_harq_per_block,max_latency_ms\n";

    for (HTS_CalChannelType ch : scenarios) {
        for (int si = 0; si < kNumIntensity; ++si) {
            const double inten = kIntensitySweep[static_cast<size_t>(si)];

            std::mt19937 rng_pre(kFixedSeed);
            MessageStats pre = run_full_message(
                payload, fec, interleaver, rng_pre, ch, inten, Apply_Pre);

            std::mt19937 rng_post(kFixedSeed);
            MessageStats post = run_full_message(
                payload, fec, interleaver, rng_post, ch, inten, Apply_Post);

            std::mt19937 rng_aj(kFixedSeed);
            MessageStats antijam = run_full_message(
                payload, fec, interleaver, rng_aj, ch, inten, Apply_Pre_AntiJam);

            const double pre_rate =
                (pre.total_blocks > 0)
                ? static_cast<double>(pre.crc_ok_blocks) /
                    static_cast<double>(pre.total_blocks)
                : 0.0;
            const double post_rate =
                (post.total_blocks > 0)
                ? static_cast<double>(post.crc_ok_blocks) /
                    static_cast<double>(post.total_blocks)
                : 0.0;
            const double aj_rate =
                (antijam.total_blocks > 0)
                ? static_cast<double>(antijam.crc_ok_blocks) /
                    static_cast<double>(antijam.total_blocks)
                : 0.0;

            auto line = [&](const char* mode, const MessageStats& s,
                double crc_rate) {
                csv << mode << ','
                    << channel_tag(ch) << ','
                    << inten << ','
                    << s.crc_ok_blocks << ','
                    << s.total_blocks << ','
                    << std::fixed << std::setprecision(6) << crc_rate << ','
                    << s.ber << ','
                    << s.total_harq << ','
                    << s.avg_harq << ','
                    << s.max_latency_ms << '\n';
            };

            line("pre", pre, pre_rate);
            line("post", post, post_rate);
            line("antijam", antijam, aj_rate);
        }
    }

    csv.close();
    std::cout << "Wrote " << csv_file.string()
        << " (seed=" << kFixedSeed << ")\n";
    return 0;
}
