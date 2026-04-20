// =============================================================================
// HTS_Harq_Matrix_Test.cpp — Phase 6 HARQ Matrix (t6_sim 베이스)
// =============================================================================
// setup / build_tx / RX feed 경로는 HTS_T6_SIM_Test.cpp 와 동일 패턴.
// 재밍은 HTS_Jammer_STD (SPEC_002). Clopper-Pearson 95% CI.
// =============================================================================
#if defined(__arm__) || defined(__TARGET_ARCH_ARM)
#error "[HTS_FATAL] PC 전용"
#endif

#include "HTS_V400_Dispatcher.hpp"
#include "HTS_Walsh_Row_Converter.hpp"
#include "HTS_Jammer_STD.hpp"
#include "HTS_Clopper_Pearson.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <random>
#include <string>
#include <vector>

using ProtectedEngine::DecodedPacket;
using ProtectedEngine::FEC_HARQ;
using ProtectedEngine::HTS_V400_Dispatcher;
using ProtectedEngine::PayloadMode;

namespace {

static constexpr int16_t kAmp = 1000;
static constexpr int kPreReps = 4;
static constexpr int kPreBoost = 1;
static constexpr int kMaxC =
    2048 + (FEC_HARQ::NSYM64 + kPreReps + 4) * 64;
static constexpr int kGuard = 256;
static constexpr int kTrials = 10;
static constexpr int kMaxHarq = 8;

static DecodedPacket g_last{};
static void on_pkt(const DecodedPacket &p) { g_last = p; }

// HTS_T6_SIM_Test.cpp 와 동일 (회귀 경로 보존)
static void setup(HTS_V400_Dispatcher &d, uint32_t seed) noexcept {
    d.Set_Seed(seed);
    d.Set_IR_Mode(true);
    d.Set_Preamble_Boost(kPreBoost);
    d.Set_Preamble_Reps(kPreReps);
    d.Set_Packet_Callback(on_pkt);
    d.Update_Adaptive_BPS(1000);
    d.Set_Lab_IQ_Mode_Jam_Harness();
}

static uint32_t mk_seed(uint32_t base, int t) noexcept {
    return base ^ static_cast<uint32_t>(t * 0x9E3779B9u);
}

static void fill_info(uint32_t seed, int t, uint8_t *info) noexcept {
    uint32_t s = seed + static_cast<uint32_t>(t) * 0x6C62272Eu + 1u;
    for (int b = 0; b < 8; ++b) {
        s += 0x9E3779B9u;
        uint32_t z = s;
        z = (z ^ (z >> 16u)) * 0x45D9F3Bu;
        z = (z ^ (z >> 16u)) * 0x45D9F3Bu;
        z = z ^ (z >> 16u);
        info[b] = static_cast<uint8_t>(z & 0xFFu);
    }
}

struct TxPkt {
    int16_t I[kMaxC];
    int16_t Q[kMaxC];
    uint8_t info[8];
    int n;
};

static TxPkt build_tx(uint32_t ds, int t, bool is_voice) noexcept {
    TxPkt pkt{};
    HTS_V400_Dispatcher tx;
    setup(tx, ds);
    fill_info(ds, t, pkt.info);
    const PayloadMode mode =
        is_voice ? PayloadMode::VOICE : PayloadMode::DATA;
    pkt.n = tx.Build_Packet(mode, pkt.info, 8, kAmp, pkt.I, pkt.Q, kMaxC);
    return pkt;
}

struct TrialResult {
    bool crc_ok;
    int harq_k;
    int bit_errors;
    bool pre_sync;
    bool hdr_ok;
    bool decode_called;
};

// RX: T6 feed_raw_ext 와 동일 (선행/후행 0칩 + Get_Phase 스모크 지표만 추가)
static TrialResult feed_raw_harq(uint32_t ds, const int16_t *rxI,
                                 const int16_t *rxQ, int n,
                                 const uint8_t *expected,
                                 int pre_guard = kGuard) noexcept {
    TrialResult r{};
    r.crc_ok = false;
    r.harq_k = kMaxHarq;
    r.bit_errors = 64;
    r.pre_sync = false;
    r.hdr_ok = false;
    r.decode_called = false;

    g_last = DecodedPacket{};
    HTS_V400_Dispatcher rx;
    setup(rx, ds);

    int max_phase = 0;
    auto bump = [&max_phase](int ph) {
        if (ph < 3) {
            if (ph > max_phase)
                max_phase = ph;
        } else if (max_phase < 2) {
            max_phase = 2;
        }
    };

    for (int i = 0; i < pre_guard; ++i) {
        rx.Feed_Chip(0, 0);
        bump(static_cast<int>(rx.Get_Phase()));
    }
    for (int i = 0; i < n; ++i) {
        rx.Feed_Chip(rxI[i], rxQ[i]);
        bump(static_cast<int>(rx.Get_Phase()));
    }
    for (int i = 0; i < kGuard; ++i) {
        rx.Feed_Chip(0, 0);
        bump(static_cast<int>(rx.Get_Phase()));
    }

    r.pre_sync = (max_phase >= 1);
    r.hdr_ok = (max_phase >= 2);

    const bool crc_pass =
        (g_last.success_mask == DecodedPacket::DECODE_MASK_OK);
    const bool len_ok = (g_last.data_len == 8);
    r.decode_called = crc_pass;

    if (crc_pass && len_ok) {
        int bit_err = 0;
        for (int b = 0; b < 8; ++b) {
            uint8_t d = static_cast<uint8_t>(
                static_cast<uint8_t>(g_last.data[b]) ^ expected[b]);
            d = static_cast<uint8_t>(d - ((d >> 1) & 0x55u));
            d = static_cast<uint8_t>((d & 0x33u) + ((d >> 2) & 0x33u));
            d = static_cast<uint8_t>((d + (d >> 4)) & 0x0Fu);
            bit_err += static_cast<int>(d);
        }
        r.bit_errors = bit_err;
        r.crc_ok = (bit_err == 0);
        r.harq_k = 1;
    }
    if (r.decode_called) {
        r.hdr_ok = true;
        r.pre_sync = true;
    }
    if (r.hdr_ok)
        r.pre_sync = true;
    return r;
}

struct ChannelSpec {
    HTS_Jammer_STD::ChannelType type;
    std::vector<double> intensities;
};

static std::vector<ChannelSpec> Build_Matrix() {
    using Ch = HTS_Jammer_STD::ChannelType;
    std::vector<ChannelSpec> m;
    m.push_back({Ch::Clean, {0.0}});
    m.push_back({Ch::AWGN, {-20, -25, -30, -35, -40, -45, -50}});
    m.push_back({Ch::Barrage, {5, 10, 15, 20, 25, 30, 35, 40, 45, 50}});
    m.push_back({Ch::CW, {5, 10, 15, 20, 25, 30, 35, 40, 45, 50}});
    m.push_back({Ch::Pulse, {10, 15, 20, 25, 30, 35, 40}});
    m.push_back({Ch::MultiTone, {5, 10, 15, 20, 25, 30, 35, 40, 45, 50}});
    m.push_back({Ch::Swept, {5, 10, 15, 20, 25, 30, 35, 40, 45, 50}});
    m.push_back({Ch::Partial_Barrage,
                 {5, 10, 15, 20, 25, 30, 35, 40, 45, 50}});
    return m;
}

struct CellResult {
    int crc_ok;
    int total;
    double ci_low;
    double ci_high;
    double ber;
    int pre_count;
    int hdr_count;
    int dec_count;
    double elapsed_ms;
    // DIAG (trial 평균, 유효 trial만 누적)
    double P_s_payload_avg;
    double P_s_preamble_avg;
    double P_after_jam_avg;
    double jsr_measured_db;
};

static uint64_t Derive_Cell_Seed(uint64_t base, bool use_ir, bool is_voice,
                                  int ch_idx, double intensity) {
    uint64_t s = base;
    s ^= static_cast<uint64_t>(use_ir) * 0x0123456789ABCDEFull;
    s ^= static_cast<uint64_t>(is_voice) * 0xFEDCBA9876543210ull;
    s += static_cast<uint64_t>(ch_idx) * 0x100000000ull;
    s += static_cast<uint64_t>(std::llround(intensity * 100.0)) * 0x10000ull;
    s = (s ^ (s >> 30)) * 0xBF58476D1CE4E5B9ull;
    s = (s ^ (s >> 27)) * 0x94D049BB133111EBull;
    s = s ^ (s >> 31);
    return s;
}

static CellResult Run_Cell(bool is_voice,
                           HTS_Jammer_STD::ChannelType channel,
                           double intensity, uint64_t base_seed) {
    CellResult r{};
    r.total = kTrials;
    std::mt19937 rng_jam(static_cast<uint32_t>(base_seed ^ 0xBEEFCAFEull));

    int ok = 0, pre_c = 0, hdr_c = 0, dec_c = 0;
    long long total_be = 0;
    int diag_trials = 0;
    const auto t0 = std::chrono::steady_clock::now();

    const int chips_per_sym = is_voice ? 16 : 64;
    using SP = HTS_Jammer_STD::StdParams;

    for (int t = 0; t < kTrials; ++t) {
        const uint32_t ds =
            mk_seed(static_cast<uint32_t>(base_seed & 0xFFFFFFFFu), t);
        auto tx = build_tx(ds, t, is_voice);
        if (tx.n <= 0) {
            total_be += 64;
            continue;
        }

        int16_t rI[kMaxC], rQ[kMaxC];
        std::memcpy(rI, tx.I, sizeof(int16_t) * static_cast<size_t>(tx.n));
        std::memcpy(rQ, tx.Q, sizeof(int16_t) * static_cast<size_t>(tx.n));

        const int pay_off = (kPreReps + 1) * 64 + 128;
        const int pay_chips = tx.n - pay_off;
        if (pay_chips <= 0) {
            total_be += 64;
            continue;
        }
        const int nsym = pay_chips / chips_per_sym;

        double P_s = HTS_Jammer_STD::Measure_Signal_Power(
            rI + pay_off, rQ + pay_off, pay_chips);
        if (P_s <= 0.0)
            P_s = 1.0;

        const double P_pre =
            HTS_Jammer_STD::Measure_Signal_Power(rI, rQ, pay_off);

        using Ch = HTS_Jammer_STD::ChannelType;
        switch (channel) {
        case Ch::Clean:
            break;
        case Ch::AWGN:
            HTS_Jammer_STD::Add_AWGN(rI + pay_off, rQ + pay_off, pay_chips,
                                     intensity, P_s, rng_jam);
            break;
        case Ch::Barrage:
            HTS_Jammer_STD::Add_Barrage(rI + pay_off, rQ + pay_off, pay_chips,
                                        intensity, P_s, rng_jam);
            break;
        case Ch::CW:
            HTS_Jammer_STD::Add_CW(rI + pay_off, rQ + pay_off, pay_chips,
                                   intensity, P_s, SP::CW_F_OFFSET_HZ,
                                   SP::F_CHIP_HZ, rng_jam);
            break;
        case Ch::Pulse:
            HTS_Jammer_STD::Add_Pulse(
                rI + pay_off, rQ + pay_off, pay_chips, intensity, P_s,
                SP::PULSE_DUTY, SP::PULSE_T_PERIOD, SP::CW_F_OFFSET_HZ,
                SP::F_CHIP_HZ, rng_jam);
            break;
        case Ch::MultiTone:
            HTS_Jammer_STD::Add_MultiTone(
                rI + pay_off, rQ + pay_off, pay_chips, intensity, P_s,
                SP::MULTI_N_TONES, SP::F_CHIP_HZ, SP::F_CHIP_HZ, rng_jam);
            break;
        case Ch::Swept:
            HTS_Jammer_STD::Add_Swept(
                rI + pay_off, rQ + pay_off, pay_chips, intensity, P_s,
                SP::SWEPT_F_START_HZ, SP::SWEPT_F_END_HZ, SP::SWEPT_RATE_HZ_S,
                SP::F_CHIP_HZ, rng_jam);
            break;
        case Ch::Partial_Barrage:
            HTS_Jammer_STD::Add_Partial_Barrage(
                rI + pay_off, rQ + pay_off, nsym, chips_per_sym, intensity,
                SP::PARTIAL_JSR_DB, P_s, rng_jam);
            break;
        }

        const double P_after = HTS_Jammer_STD::Measure_Signal_Power(
            rI + pay_off, rQ + pay_off, pay_chips);
        const double P_jam_est =
            (P_after > P_s) ? (P_after - P_s) : 0.0;
        const double jsr_db =
            (P_jam_est > 0.0 && P_s > 0.0)
                ? (10.0 * std::log10(P_jam_est / P_s))
                : -99.0;

        r.P_s_payload_avg += P_s;
        r.P_s_preamble_avg += P_pre;
        r.P_after_jam_avg += P_after;
        r.jsr_measured_db += jsr_db;
        ++diag_trials;

        TrialResult tr = feed_raw_harq(ds, rI, rQ, tx.n, tx.info);
        if (tr.crc_ok)
            ++ok;
        if (tr.pre_sync)
            ++pre_c;
        if (tr.hdr_ok)
            ++hdr_c;
        if (tr.decode_called)
            ++dec_c;
        total_be += tr.bit_errors;
    }

    const auto t1 = std::chrono::steady_clock::now();
    r.elapsed_ms =
        std::chrono::duration<double, std::milli>(t1 - t0).count();
    r.crc_ok = ok;
    r.pre_count = pre_c;
    r.hdr_count = hdr_c;
    r.dec_count = dec_c;
    r.ber = static_cast<double>(total_be) /
            (static_cast<double>(kTrials) * 64.0);

    auto ci = HTS_Clopper_Pearson::Compute(ok, kTrials, 0.05);
    r.ci_low = ci.p_lower;
    r.ci_high = ci.p_upper;
    if (diag_trials > 0) {
        const double nd = static_cast<double>(diag_trials);
        r.P_s_payload_avg /= nd;
        r.P_s_preamble_avg /= nd;
        r.P_after_jam_avg /= nd;
        r.jsr_measured_db /= nd;
    }
    return r;
}

static const char *FecName(bool use_ir) {
    return use_ir ? "ir_harq" : "chase";
}
static const char *ModeName(bool is_voice) {
    return is_voice ? "VOICE" : "DATA";
}

} // namespace

int main() {
    ProtectedEngine::WRC::reset_diag();
    const auto matrix = Build_Matrix();
    int per_mode = 0;
    for (const auto &cs : matrix)
        per_mode += static_cast<int>(cs.intensities.size());
    const int total_cells = per_mode * 4;

    std::printf("=== Phase 6 HARQ Matrix (t6_sim base) ===\n");
    std::printf("N_MC=%d, total_cells=%d, total_trials=%d\n", kTrials,
                total_cells, kTrials * total_cells);
    std::printf(
        "+------+---------+-------+-----------------+---------+-------+"
        "-------------+-------+-------+-------+-------+----------+\n");
    std::printf("| %-4s | %-7s | %-5s | %-15s | %-7s | %-5s | %-11s | "
                "%-5s | %-3s | %-3s | %-3s | %-8s |\n",
                "No.", "fec", "mode", "channel", "intens", "crc", "CI_95",
                "BER", "pre", "hdr", "dec", "time_ms");
    std::printf(
        "+------+---------+-------+-----------------+---------+-------+"
        "-------------+-------+-------+-------+-------+----------+\n");

    std::FILE *fcsv = std::fopen("HARQ_Matrix_Results.csv", "wb");
    const unsigned char bom[3] = {0xEF, 0xBB, 0xBF};
    std::fwrite(bom, 1, 3, fcsv);
    std::fprintf(fcsv,
                 "fec_path,mode,channel,intensity_unit,intensity,"
                 "crc_ok,total,crc_rate,ci_low,ci_high,ber,pre,hdr,dec,"
                 "P_s_pay,P_s_pre,P_after,jsr_meas_db,seed\n");

    int idx = 0;
    const uint64_t BASE_SEED = 0xDEADBEEFCAFEBABEull;
    const auto t_all = std::chrono::steady_clock::now();

    for (int f = 0; f < 2; ++f) {
        const bool use_ir = (f == 1);
        for (int m = 0; m < 2; ++m) {
            const bool is_voice = (m == 1);
            int ch_idx = 0;
            for (const auto &cs : matrix) {
                for (double intens : cs.intensities) {
                    ++idx;
                    const uint64_t seed =
                        Derive_Cell_Seed(BASE_SEED, use_ir, is_voice, ch_idx,
                                         intens);
                    CellResult rr =
                        Run_Cell(is_voice, cs.type, intens, seed);

                    char crc_s[16], ci_s[24], pre_s[8], hdr_s[8], dec_s[8];
                    std::snprintf(crc_s, sizeof(crc_s), "%2d/%2d", rr.crc_ok,
                                  rr.total);
                    std::snprintf(ci_s, sizeof(ci_s), "%.2f-%.2f", rr.ci_low,
                                  rr.ci_high);
                    std::snprintf(pre_s, sizeof(pre_s), "%2d/%2d",
                                  rr.pre_count, rr.total);
                    std::snprintf(hdr_s, sizeof(hdr_s), "%2d/%2d",
                                  rr.hdr_count, rr.total);
                    std::snprintf(dec_s, sizeof(dec_s), "%2d/%2d",
                                  rr.dec_count, rr.total);

                    std::printf(
                        "| %4d | %-7s | %-5s | %-15s | %7.2f | %-5s | "
                        "%-11s | %5.3f | %-3s | %-3s | %-3s | %8.0f |\n",
                        idx, FecName(use_ir), ModeName(is_voice),
                        HTS_Jammer_STD::Channel_Name(cs.type), intens, crc_s,
                        ci_s, rr.ber, pre_s, hdr_s, dec_s, rr.elapsed_ms);

                    std::fprintf(
                        fcsv,
                        "%s,%s,%s,%s,%.2f,%d,%d,%.6f,%.6f,%.6f,%.6f,%d,%d,%d,"
                        "%.6f,%.6f,%.6f,%.6f,0x%016llx\n",
                        FecName(use_ir), ModeName(is_voice),
                        HTS_Jammer_STD::Channel_Name(cs.type),
                        HTS_Jammer_STD::Channel_Unit(cs.type), intens,
                        rr.crc_ok, rr.total,
                        static_cast<double>(rr.crc_ok) /
                            static_cast<double>(rr.total),
                        rr.ci_low, rr.ci_high, rr.ber, rr.pre_count,
                        rr.hdr_count, rr.dec_count,
                        rr.P_s_payload_avg, rr.P_s_preamble_avg,
                        rr.P_after_jam_avg, rr.jsr_measured_db,
                        static_cast<unsigned long long>(seed));
                    std::fflush(fcsv);
                    std::fflush(stdout);
                }
                ++ch_idx;
            }
        }
    }

    std::printf(
        "+------+---------+-------+-----------------+---------+-------+"
        "-------------+-------+-------+-------+-------+----------+\n");
    const auto t_end = std::chrono::steady_clock::now();
    std::printf("Elapsed: %.1f s\n",
                std::chrono::duration<double>(t_end - t_all).count());
    std::fclose(fcsv);
    return 0;
}

#include "../../HTS_LIM/HTS_Secure_Memory.cpp"
#include "../../HTS_LIM/HTS_Polar_Codec.cpp"
#include "../../HTS_LIM/HTS_FEC_HARQ.cpp"
#include "../../HTS_LIM/HTS_Walsh_Row_Converter.cpp"
#include "../../HTS_LIM/HTS_Holo_LPI.cpp"
#include "../../HTS_LIM/HTS_Walsh_Row_Permuter.cpp"
#include "../../HTS_LIM/HTS_V400_Dispatcher.cpp"
#include "HTS_Session_Derive_Stub.cpp"
#include "HTS_Jammer_STD.cpp"
