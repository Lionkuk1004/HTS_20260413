// =============================================================================
// HTS_Harq_Matrix_Test.cpp — Phase C: FEC transition + 실제 HARQ 루프
// =============================================================================
// HTS_T6_SIM_Test.cpp 의 setup() 패턴. 재밍 HTS_Jammer_STD. Clopper-Pearson.
// HARQ: HTS_MC_Runner 와 동일 — round0 Build_Packet+Feed_Chip,
//       round>=1 Build_Retx+Feed_Retx_Chip.
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
static constexpr int kTrials = 100; // v8 Phase C
static constexpr int kMaxHarq = 8;

static DecodedPacket g_last{};
static void on_pkt(const DecodedPacket &p) { g_last = p; }

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

struct TrialResult {
    bool crc_ok;
    int harq_k;
    int bit_errors;
    bool pre_sync;
    bool hdr_ok;
    bool decode_called;
    int total_chips;
};

static void apply_jam_payload(int16_t *payI, int16_t *payQ, int pay_chips,
                              int nsym, int chips_per_sym,
                              HTS_Jammer_STD::ChannelType channel,
                              double intensity, double P_s,
                              std::mt19937 &rng_jam) noexcept {
    using Ch = HTS_Jammer_STD::ChannelType;
    using SP = HTS_Jammer_STD::StdParams;
    switch (channel) {
    case Ch::Clean:
        break;
    case Ch::AWGN:
        HTS_Jammer_STD::Add_AWGN(payI, payQ, pay_chips, intensity, P_s,
                                 rng_jam);
        break;
    case Ch::Barrage:
        HTS_Jammer_STD::Add_Barrage(payI, payQ, pay_chips, intensity, P_s,
                                    rng_jam);
        break;
    case Ch::CW:
        HTS_Jammer_STD::Add_CW(payI, payQ, pay_chips, intensity, P_s,
                               SP::CW_F_OFFSET_HZ, SP::F_CHIP_HZ, rng_jam);
        break;
    case Ch::Pulse:
        HTS_Jammer_STD::Add_Pulse(payI, payQ, pay_chips, intensity, P_s,
                                  SP::PULSE_DUTY, SP::PULSE_T_PERIOD,
                                  SP::CW_F_OFFSET_HZ, SP::F_CHIP_HZ, rng_jam);
        break;
    case Ch::MultiTone:
        HTS_Jammer_STD::Add_MultiTone(payI, payQ, pay_chips, intensity, P_s,
                                       SP::MULTI_N_TONES, SP::F_CHIP_HZ,
                                       SP::F_CHIP_HZ, rng_jam);
        break;
    case Ch::Swept:
        HTS_Jammer_STD::Add_Swept(payI, payQ, pay_chips, intensity, P_s,
                                  SP::SWEPT_F_START_HZ, SP::SWEPT_F_END_HZ,
                                  SP::SWEPT_RATE_HZ_S, SP::F_CHIP_HZ, rng_jam);
        break;
    case Ch::Partial_Barrage:
        HTS_Jammer_STD::Add_Partial_Barrage(payI, payQ, nsym, chips_per_sym,
                                            intensity, SP::PARTIAL_JSR_DB, P_s,
                                            rng_jam);
        break;
    }
}

// MC_Runner 패턴: R0=Build_Packet+Feed_Chip(+가드), R+=Build_Retx+Feed_Retx
static TrialResult run_harq_trial(uint32_t ds, const uint8_t *info,
                                    bool is_voice,
                                    HTS_Jammer_STD::ChannelType channel,
                                    double intensity,
                                    std::mt19937 &rng_jam) noexcept {
    TrialResult r{};
    r.crc_ok = false;
    r.harq_k = kMaxHarq;
    r.bit_errors = 64;
    r.pre_sync = false;
    r.hdr_ok = false;
    r.decode_called = false;
    r.total_chips = 0;

    HTS_V400_Dispatcher tx;
    HTS_V400_Dispatcher rx;
    setup(tx, ds);
    setup(rx, ds);

    const PayloadMode mode =
        is_voice ? PayloadMode::VOICE : PayloadMode::DATA;
    const int chips_per_sym = is_voice ? 16 : 64;
    const int pay_off = (kPreReps + 1) * 64 + 128;
    const int pre_guard = kGuard;

    int max_phase_global = 0;
    auto bump = [&max_phase_global](int ph) {
        if (ph < 3) {
            if (ph > max_phase_global)
                max_phase_global = ph;
        } else if (max_phase_global < 2) {
            max_phase_global = 2;
        }
    };

    int16_t I[kMaxC], Q[kMaxC];

    for (int round = 1; round <= kMaxHarq; ++round) {
        g_last = DecodedPacket{};

        int n = 0;
        if (round == 1) {
            n = tx.Build_Packet(mode, info, 8, kAmp, I, Q, kMaxC);
        } else {
            n = tx.Build_Retx(mode, info, 8, kAmp, I, Q, kMaxC);
        }
        if (n <= 0)
            break;

        int pay_start;
        int pay_chips;
        if (round == 1) {
            pay_start = pay_off;
            pay_chips = n - pay_off;
        } else {
            pay_start = 0;
            pay_chips = n;
        }
        if (pay_chips <= 0)
            break;

        const int nsym = pay_chips / chips_per_sym;

        double P_s = HTS_Jammer_STD::Measure_Signal_Power(
            I + pay_start, Q + pay_start, pay_chips);
        if (P_s <= 0.0)
            P_s = 1.0;

        apply_jam_payload(I + pay_start, Q + pay_start, pay_chips, nsym,
                          chips_per_sym, channel, intensity, P_s, rng_jam);

        if (round == 1) {
            for (int i = 0; i < pre_guard; ++i) {
                rx.Feed_Chip(0, 0);
                bump(static_cast<int>(rx.Get_Phase()));
            }
            for (int i = 0; i < n; ++i) {
                rx.Feed_Chip(I[i], Q[i]);
                bump(static_cast<int>(rx.Get_Phase()));
            }
            for (int i = 0; i < kGuard; ++i) {
                rx.Feed_Chip(0, 0);
                bump(static_cast<int>(rx.Get_Phase()));
            }
            r.total_chips += pre_guard + n + kGuard;
        } else {
            for (int i = 0; i < pay_chips; ++i) {
                rx.Feed_Retx_Chip(I[pay_start + i], Q[pay_start + i]);
                bump(static_cast<int>(rx.Get_Phase()));
            }
            r.total_chips += pay_chips;
        }

        const bool crc_ok =
            (g_last.success_mask == DecodedPacket::DECODE_MASK_OK);
        const bool len_ok = (g_last.data_len == 8);
        r.decode_called = r.decode_called || crc_ok;

        if (crc_ok && len_ok) {
            int bit_err = 0;
            for (int b = 0; b < 8; ++b) {
                uint8_t d = static_cast<uint8_t>(
                    static_cast<uint8_t>(g_last.data[b]) ^ info[b]);
                d = static_cast<uint8_t>(d - ((d >> 1) & 0x55u));
                d = static_cast<uint8_t>((d & 0x33u) + ((d >> 2) & 0x33u));
                d = static_cast<uint8_t>((d + (d >> 4)) & 0x0Fu);
                bit_err += static_cast<int>(d);
            }
            r.bit_errors = bit_err;
            if (bit_err == 0) {
                r.crc_ok = true;
                int hk = g_last.harq_k;
                if (hk < 1)
                    hk = 1;
                if (hk > kMaxHarq)
                    hk = kMaxHarq;
                r.harq_k = hk;
                break;
            }
        }

        if (round == 1 && !rx.Is_Retx_Ready())
            break;
    }

    r.pre_sync = (max_phase_global >= 1);
    r.hdr_ok = (max_phase_global >= 2);
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
    m.push_back({Ch::AWGN,
                 {-5, -6, -7, -8, -9, -10, -11, -12, -13, -14, -15, -16, -17,
                  -18, -20}});
    m.push_back({Ch::Barrage,
                 {10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 22, 25}});
    m.push_back({Ch::CW,
                 {10, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 27, 30}});
    m.push_back({Ch::MultiTone,
                 {5, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24,
                  25, 27, 30}});
    m.push_back({Ch::Swept,
                 {10, 15, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28}});
    m.push_back({Ch::Pulse, {10, 15, 20, 25, 30, 35, 40}});
    m.push_back({Ch::Partial_Barrage, {5, 10, 20, 30, 40, 50}});
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
    double harq_k_avg;
    int harq_k_max;
    double latency_avg_chips;
    int succ_at_k[9];
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
    for (int i = 0; i < 9; ++i)
        r.succ_at_k[i] = 0;
    r.harq_k_avg = 0.0;
    r.harq_k_max = 0;
    r.latency_avg_chips = 0.0;

    std::mt19937 rng_jam(static_cast<uint32_t>(base_seed ^ 0xBEEFCAFEull));

    int ok = 0, pre_c = 0, hdr_c = 0, dec_c = 0;
    long long total_be = 0;
    long long harq_k_sum = 0;
    int harq_k_max_obs = 0;
    long long total_chips_sum = 0;
    int success_at_k[kMaxHarq + 1] = {0};

    const auto t0 = std::chrono::steady_clock::now();

    for (int t = 0; t < kTrials; ++t) {
        const uint32_t ds =
            mk_seed(static_cast<uint32_t>(base_seed & 0xFFFFFFFFu), t);
        uint8_t info[8];
        fill_info(ds, t, info);

        TrialResult tr =
            run_harq_trial(ds, info, is_voice, channel, intensity, rng_jam);

        if (tr.crc_ok)
            ++ok;
        if (tr.pre_sync)
            ++pre_c;
        if (tr.hdr_ok)
            ++hdr_c;
        if (tr.decode_called)
            ++dec_c;
        total_be += tr.bit_errors;

        harq_k_sum += tr.harq_k;
        if (tr.harq_k > harq_k_max_obs)
            harq_k_max_obs = tr.harq_k;
        total_chips_sum += tr.total_chips;
        if (tr.crc_ok && tr.harq_k >= 1 && tr.harq_k <= kMaxHarq) {
            ++success_at_k[tr.harq_k];
        }
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

    r.harq_k_avg =
        static_cast<double>(harq_k_sum) / static_cast<double>(kTrials);
    r.harq_k_max = harq_k_max_obs;
    r.latency_avg_chips =
        static_cast<double>(total_chips_sum) / static_cast<double>(kTrials);
    for (int k = 1; k <= kMaxHarq; ++k)
        r.succ_at_k[k] = success_at_k[k];

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

    std::printf("=== Phase C HARQ Matrix (t6_sim base) ===\n");
    std::printf("N_MC=%d, total_cells=%d, total_trials=%d\n", kTrials,
                total_cells, kTrials * total_cells);
    std::printf(
        "+------+---------+-------+-----------------+---------+-------+"
        "-------------+-------+---------+----------+\n");
    std::printf("| %-4s | %-7s | %-5s | %-15s | %-7s | %-5s | %-11s | "
                "%-5s | %-7s | %8s |\n",
                "No.", "fec", "mode", "channel", "intens", "crc", "CI_95",
                "BER", "harq", "time_ms");
    std::printf(
        "+------+---------+-------+-----------------+---------+-------+"
        "-------------+-------+---------+----------+\n");

    std::FILE *fcsv = std::fopen("HARQ_Matrix_Results.csv", "wb");
    const unsigned char bom[3] = {0xEF, 0xBB, 0xBF};
    std::fwrite(bom, 1, 3, fcsv);
    std::fprintf(fcsv,
                 "fec_path,mode,channel,intensity_unit,intensity,"
                 "crc_ok,total,crc_rate,ci_low,ci_high,ber,pre,hdr,dec,"
                 "harq_k_avg,harq_k_max,latency_avg_chips,"
                 "s_k1,s_k2,s_k3,s_k4,s_k5,s_k6,s_k7,s_k8,seed\n");

    int idx = 0;
    const uint64_t BASE_SEED = 0xDEADBEEFCAFEBABEull;
    const auto t_all = std::chrono::steady_clock::now();

    for (int f = 0; f < 2; ++f) {
        const bool use_ir = (f == 1);
        (void)use_ir;
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

                    char crc_s[20], ci_s[24], harq_s[24];
                    std::snprintf(crc_s, sizeof(crc_s), "%3d/%3d", rr.crc_ok,
                                  rr.total);
                    std::snprintf(ci_s, sizeof(ci_s), "%.2f-%.2f", rr.ci_low,
                                  rr.ci_high);
                    std::snprintf(harq_s, sizeof(harq_s), "%.2f/%d",
                                  rr.harq_k_avg, rr.harq_k_max);

                    std::printf(
                        "| %4d | %-7s | %-5s | %-15s | %7.2f | %-5s | "
                        "%-11s | %5.3f | %-7s | %8.0f |\n",
                        idx, FecName(use_ir), ModeName(is_voice),
                        HTS_Jammer_STD::Channel_Name(cs.type), intens, crc_s,
                        ci_s, rr.ber, harq_s, rr.elapsed_ms);

                    std::fprintf(
                        fcsv,
                        "%s,%s,%s,%s,%.2f,%d,%d,%.6f,%.6f,%.6f,%.6f,%d,%d,%d,"
                        "%.6f,%d,%.1f,"
                        "%d,%d,%d,%d,%d,%d,%d,%d,0x%016llx\n",
                        FecName(use_ir), ModeName(is_voice),
                        HTS_Jammer_STD::Channel_Name(cs.type),
                        HTS_Jammer_STD::Channel_Unit(cs.type), intens,
                        rr.crc_ok, rr.total,
                        static_cast<double>(rr.crc_ok) /
                            static_cast<double>(rr.total),
                        rr.ci_low, rr.ci_high, rr.ber, rr.pre_count,
                        rr.hdr_count, rr.dec_count, rr.harq_k_avg,
                        rr.harq_k_max, rr.latency_avg_chips,
                        rr.succ_at_k[1], rr.succ_at_k[2], rr.succ_at_k[3],
                        rr.succ_at_k[4], rr.succ_at_k[5], rr.succ_at_k[6],
                        rr.succ_at_k[7], rr.succ_at_k[8],
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
        "-------------+-------+---------+----------+\n");
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
