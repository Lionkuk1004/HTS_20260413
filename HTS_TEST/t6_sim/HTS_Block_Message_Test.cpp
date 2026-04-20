// =============================================================================
// HTS_Block_Message_Test.cpp — Step C-3: 블록(38) × 10회 반복 × Partial Barrage
// =============================================================================
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif
// HTS_Harq_Matrix_Test.cpp 미수정. DATA 전용, Chase/IR, 강도×커버리지 매트릭스.
// 매 HARQ round 마다 독립 round_seed 로 chip-wise partial barrage (Box-Muller).
// =============================================================================
#if defined(__arm__) || defined(__TARGET_ARCH_ARM)
#error "[HTS_FATAL] PC 전용"
#endif

#include "HTS_V400_Dispatcher_Local.hpp"
#include "HTS_Walsh_Row_Converter.hpp"
#include "../../HTS_Jammer_STD/HTS_Jammer_STD.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

using ProtectedEngine::FEC_HARQ;
using ProtectedEngineLocal::DecodedPacket;
using ProtectedEngineLocal::HTS_V400_Dispatcher;
using ProtectedEngineLocal::PayloadMode;

namespace {

static constexpr int16_t kAmp = 1000;
static constexpr int kPreReps = 4;
static constexpr int kPreBoost = 1;
static constexpr int kMaxC =
    2048 + (FEC_HARQ::NSYM64 + kPreReps + 4) * 64;
static constexpr int kGuard = 256;
static constexpr int kMaxHarq = 8;
static constexpr int kBlocksPerRun = 38;
static constexpr int kRunsPerCell = 10;
static constexpr double kChipRate_Hz = 200000.0;
static constexpr int kJsrCount = 10;
static constexpr double kJsrDb[kJsrCount] = {5.0,  10.0, 15.0, 20.0, 25.0,
                                               30.0, 35.0, 40.0, 45.0, 50.0};
static constexpr double kCovPct[3] = {50.0, 70.0, 100.0};

static DecodedPacket g_last{};
static void on_pkt(const DecodedPacket &p) { g_last = p; }

static void setup(HTS_V400_Dispatcher &d, uint32_t seed, bool use_ir) noexcept {
    d.Set_Seed(seed);
    d.Set_IR_Mode(use_ir);
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

// ── SplitMix64 → xoroshiro128+ (round 독립 스트림) ───────────────────────
static uint64_t splitmix64(uint64_t &z) noexcept {
    uint64_t x = (z += 0x9E3779B97F4A7C15ULL);
    x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ULL;
    x = (x ^ (x >> 27)) * 0x94D049BB133111EBULL;
    return x ^ (x >> 31);
}

struct Xoroshiro128p {
    uint64_t s0_;
    uint64_t s1_;
    explicit Xoroshiro128p(uint64_t seed) noexcept {
        uint64_t z = seed ^ 0xA0761D6478BD642FULL;
        s0_ = splitmix64(z);
        s1_ = splitmix64(z);
        if ((s0_ | s1_) == 0ULL) {
            s0_ = 0xD15A2A55C33A4321ULL;
            s1_ = 0xEDC3BAFE98104653ULL;
        }
    }
    [[nodiscard]] uint32_t next_u32() noexcept {
        const uint64_t s0 = s0_;
        uint64_t s1 = s1_;
        const uint64_t rotl = (s0 << 24) | (s0 >> (64 - 24));
        const uint64_t result = s0 + s1;
        s1 ^= s0;
        s0_ = rotl ^ s1 ^ (s1 << 16);
        s1_ = (s1 << 37) | (s1 >> (64 - 37));
        s1_ = s1;
        return static_cast<uint32_t>(result & 0xFFFFFFFFu);
    }
};

// Partial barrage: coverage% 칩에만 JSR dB 가우시안 (round_seed 독립 스트림)
static void apply_barrage_partial(int16_t *chip_I, int16_t *chip_Q, int n_chips,
                                  double jsr_db, double coverage_pct,
                                  uint64_t round_seed) noexcept {
    if (chip_I == nullptr || chip_Q == nullptr || n_chips <= 0) {
        return;
    }
    double P_s =
        HTS_Jammer_STD::Measure_Signal_Power(chip_I, chip_Q, n_chips);
    if (P_s <= 0.0) {
        P_s = 1.0;
    }
    const double jam_lin = std::pow(10.0, jsr_db / 10.0);
    const double P_j = P_s * jam_lin;
    const double sigma = std::sqrt(P_j / 2.0);

    const double cov = std::max(0.0, std::min(100.0, coverage_pct));
    const uint32_t threshold =
        static_cast<uint32_t>(cov * 4294967295.0 / 100.0);

    Xoroshiro128p rng(round_seed);

    for (int i = 0; i < n_chips; ++i) {
        const uint32_t u_pick = rng.next_u32();
        if (u_pick > threshold) {
            continue;
        }
        const uint32_t ua = rng.next_u32();
        const uint32_t ub = rng.next_u32();
        const double u1 =
            (static_cast<double>(ua) + 1.0) / 4294967296.0;
        const double u2 = static_cast<double>(ub) / 4294967296.0;
        const double mag =
            std::sqrt(-2.0 * std::log(u1)) * static_cast<double>(sigma);
        const double ang = 2.0 * 3.14159265358979323846 * u2;
        const double n_I = mag * std::cos(ang);
        const double n_Q = mag * std::sin(ang);
        int32_t new_I =
            static_cast<int32_t>(chip_I[i]) + static_cast<int32_t>(std::lround(n_I));
        int32_t new_Q =
            static_cast<int32_t>(chip_Q[i]) + static_cast<int32_t>(std::lround(n_Q));
        if (new_I > 32767) {
            new_I = 32767;
        }
        if (new_I < -32768) {
            new_I = -32768;
        }
        if (new_Q > 32767) {
            new_Q = 32767;
        }
        if (new_Q < -32768) {
            new_Q = -32768;
        }
        chip_I[i] = static_cast<int16_t>(new_I);
        chip_Q[i] = static_cast<int16_t>(new_Q);
    }
}

struct TrialMetrics {
    bool crc_ok;
    int harq_k;
    int bit_errors;
    int total_chips;
};

[[nodiscard]] static TrialMetrics run_harq_one_payload_ir(
    uint32_t ds, const uint8_t *info, double jsr_db, double coverage_pct,
    uint64_t round_mix, bool use_ir) noexcept {
    TrialMetrics m{};
    m.crc_ok = false;
    m.harq_k = kMaxHarq;
    m.bit_errors = 64;
    m.total_chips = 0;

    HTS_V400_Dispatcher tx;
    HTS_V400_Dispatcher rx;
    setup(tx, ds, use_ir);
    setup(rx, ds, use_ir);

    const PayloadMode mode = PayloadMode::DATA;
    const int chips_per_sym = 64;
    const int pay_off = (kPreReps + 1) * 64 + 128;
    const int pre_guard = kGuard;

    int16_t I[kMaxC], Q[kMaxC];

    for (int round = 1; round <= kMaxHarq; ++round) {
        g_last = DecodedPacket{};

        int n = 0;
        if (round == 1) {
            n = tx.Build_Packet(mode, info, 8, kAmp, I, Q, kMaxC);
        } else {
            n = tx.Build_Retx(mode, info, 8, kAmp, I, Q, kMaxC);
        }
        if (n <= 0) {
            break;
        }

        int pay_start;
        int pay_chips;
        if (round == 1) {
            pay_start = pay_off;
            pay_chips = n - pay_off;
        } else {
            pay_start = 0;
            pay_chips = n;
        }
        if (pay_chips <= 0) {
            break;
        }

        const int nsym = pay_chips / chips_per_sym;
        (void)nsym;

        const uint64_t round_seed =
            round_mix ^ (static_cast<uint64_t>(static_cast<uint32_t>(round))
                         << 32) ^
            static_cast<uint64_t>(ds) ^
            (static_cast<uint64_t>(static_cast<uint32_t>(pay_chips)) << 16) ^
            (use_ir ? 0xC0DEC0DEC0DEULL : 0xB16B16B16B16ULL);

        apply_barrage_partial(I + pay_start, Q + pay_start, pay_chips, jsr_db,
                              coverage_pct, round_seed);

        if (round == 1) {
            for (int i = 0; i < pre_guard; ++i) {
                rx.Feed_Chip(0, 0);
            }
            for (int i = 0; i < n; ++i) {
                rx.Feed_Chip(I[i], Q[i]);
            }
            for (int i = 0; i < kGuard; ++i) {
                rx.Feed_Chip(0, 0);
            }
            m.total_chips += pre_guard + n + kGuard;
        } else {
            for (int i = 0; i < pay_chips; ++i) {
                rx.Feed_Retx_Chip(I[pay_start + i], Q[pay_start + i]);
            }
            m.total_chips += pay_chips;
        }

        const bool crc_ok =
            (g_last.success_mask == DecodedPacket::DECODE_MASK_OK);
        const bool len_ok = (g_last.data_len == 8);

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
            m.bit_errors = bit_err;
            if (bit_err == 0) {
                m.crc_ok = true;
                int hk = g_last.harq_k;
                if (hk < 1) {
                    hk = 1;
                }
                if (hk > kMaxHarq) {
                    hk = kMaxHarq;
                }
                m.harq_k = hk;
                break;
            }
        }

        if (round == 1 && !rx.Is_Retx_Ready()) {
            break;
        }
    }

    return m;
}

struct RunSummary {
    int blocks_ok;
    int harq_total;
    int64_t total_chips;
    int bit_errors_sum;
};

[[nodiscard]] static RunSummary run_single_experiment(
    uint64_t run_seed, double jsr_db, double coverage_pct, bool use_ir) noexcept {
    RunSummary s{};
    s.blocks_ok = 0;
    s.harq_total = 0;
    s.total_chips = 0;
    s.bit_errors_sum = 0;

    for (int blk = 0; blk < kBlocksPerRun; ++blk) {
        const uint32_t ds = mk_seed(
            static_cast<uint32_t>(run_seed & 0xFFFFFFFFu) ^
                static_cast<uint32_t>(blk * 0x27D4EB2Du),
            blk);
        uint8_t info[8];
        fill_info(ds, blk, info);

        const uint64_t mix =
            run_seed ^ (static_cast<uint64_t>(blk) << 16) ^
            (static_cast<uint64_t>(static_cast<uint32_t>(jsr_db * 1000.0))
             << 20) ^
            (static_cast<uint64_t>(static_cast<uint32_t>(coverage_pct * 100.0))
             << 8);

        const TrialMetrics tm =
            run_harq_one_payload_ir(ds, info, jsr_db, coverage_pct, mix, use_ir);
        if (tm.crc_ok && tm.bit_errors == 0) {
            ++s.blocks_ok;
        }
        s.harq_total += tm.harq_k;
        s.total_chips += tm.total_chips;
        s.bit_errors_sum += tm.bit_errors;
    }
    return s;
}

struct CellAgg {
    int pass_count;
    int partial_count;
    int fail_count;
    double avg_blocks_ok;
    double avg_harq;
    double avg_latency_ms;
    double max_ber;
};

[[nodiscard]] static CellAgg run_cell(double jsr_db, double coverage_pct,
                                      bool use_ir, uint64_t cell_seed,
                                      std::FILE *f_per, std::FILE *f_sum,
                                      const char *fec_name) noexcept {
    CellAgg a{};
    a.pass_count = 0;
    a.partial_count = 0;
    a.fail_count = 0;
    double sum_b = 0.0;
    double sum_h = 0.0;
    double sum_lat = 0.0;
    a.max_ber = 0.0;

    for (int run = 0; run < kRunsPerCell; ++run) {
        const uint64_t run_seed =
            cell_seed ^ (static_cast<uint64_t>(run) << 32) ^
            (static_cast<uint64_t>(static_cast<uint32_t>(jsr_db * 1000.0)) << 4) ^
            (static_cast<uint64_t>(static_cast<uint32_t>(coverage_pct * 100.0))
             << 8) ^
            (use_ir ? 0xA5A5A5A5A5A5A5A5ULL : 0x5A5A5A5A5A5A5A5AULL);

        const RunSummary rs = run_single_experiment(run_seed, jsr_db,
                                                    coverage_pct, use_ir);

        const double ber =
            static_cast<double>(rs.bit_errors_sum) /
            static_cast<double>(kBlocksPerRun * 64);
        const double lat_ms =
            (static_cast<double>(rs.total_chips) / kChipRate_Hz) * 1000.0;

        std::fprintf(
            f_per,
            "%s,%.0f,%.1f,%d,%d,%d,%.4f,%.6e,%s\n", fec_name, coverage_pct,
            jsr_db, run, rs.blocks_ok, rs.harq_total, lat_ms, ber,
            (rs.blocks_ok == kBlocksPerRun)
                ? "PASS"
                : ((rs.blocks_ok >= 19) ? "PARTIAL" : "FAIL"));

        if (rs.blocks_ok == kBlocksPerRun) {
            ++a.pass_count;
        } else if (rs.blocks_ok >= 19) {
            ++a.partial_count;
        } else {
            ++a.fail_count;
        }
        sum_b += static_cast<double>(rs.blocks_ok);
        sum_h += static_cast<double>(rs.harq_total);
        sum_lat += lat_ms;
        if (ber > a.max_ber) {
            a.max_ber = ber;
        }
    }

    a.avg_blocks_ok = sum_b / static_cast<double>(kRunsPerCell);
    a.avg_harq = sum_h / static_cast<double>(kRunsPerCell);
    a.avg_latency_ms = sum_lat / static_cast<double>(kRunsPerCell);

    std::fprintf(f_sum,
                 "%s,%.0f,%.1f,%d,%d,%d,%.3f,%.3f,%.4f,%.6e\n", fec_name,
                 coverage_pct, jsr_db, a.pass_count, a.partial_count,
                 a.fail_count, a.avg_blocks_ok, a.avg_harq, a.avg_latency_ms,
                 a.max_ber);
    std::fflush(f_per);
    std::fflush(f_sum);
    return a;
}

static uint64_t cell_base_seed(double cov, double jsr, bool ir,
                                int cov_idx, int jsr_idx) noexcept {
    uint64_t s = 0xC35C300000000003ULL;
    s ^= static_cast<uint64_t>(cov_idx) * 0x100000001B3ULL;
    s ^= static_cast<uint64_t>(jsr_idx) * 0xFEDCBA9876543210ULL;
    s ^= static_cast<uint64_t>(std::llround(cov * 100.0)) << 24;
    s ^= static_cast<uint64_t>(std::llround(jsr * 1000.0)) << 36;
    s ^= ir ? 0x1111111111111111ULL : 0x2222222222222222ULL;
    return s;
}

} // namespace

int main() {
    ProtectedEngine::WRC::reset_diag();

    std::FILE *f_log = std::fopen("stepC3_block_message_results.log", "wb");
    std::FILE *f_con = std::fopen("stepC3_block_message_console.txt", "wb");
    std::FILE *f_per = std::fopen("stepC3_block_message_per_run.csv", "wb");
    std::FILE *f_sum = std::fopen("stepC3_block_message_summary.csv", "wb");
    if (f_log == nullptr || f_con == nullptr || f_per == nullptr ||
        f_sum == nullptr) {
        std::fprintf(stderr, "[C3] fopen output failed\n");
        return 1;
    }

    std::fprintf(f_per,
                 "fec_path,coverage_pct,jsr_db,run_idx,blocks_ok,harq_total,"
                 "latency_ms,ber_avg,result\n");
    std::fprintf(f_sum,
                 "fec_path,coverage_pct,jsr_db,pass_count,partial_count,"
                 "fail_count,avg_blocks,avg_harq,avg_latency_ms,max_ber\n");

    const auto t_all = std::chrono::steady_clock::now();

    for (int ci = 0; ci < 3; ++ci) {
        const double cov = kCovPct[ci];
        for (int irf = 0; irf < 2; ++irf) {
            const bool use_ir = (irf != 0);
            const char *fecn = use_ir ? "ir_harq" : "chase";
            std::fprintf(
                f_con,
                "\n[시나리오] 바라지(부분) 재밍 (커버리지 %.0f%%, FEC=%s, %d회 "
                "반복)\n",
                cov, use_ir ? "IR" : "Chase", kRunsPerCell);
            std::fprintf(f_con,
                         "-----------------------------------------------------------------"
                         "----\n");
            std::fprintf(f_con,
                         "강도    단위    블록수  성공평균  HARQ평균  지연평균(ms)  "
                         "판정\n");
            std::fprintf(f_con,
                         "-----------------------------------------------------------------"
                         "----\n");
            std::fprintf(f_log,
                         "\n=== Scenario cov=%.0f fec=%s ===\n", cov, fecn);

            for (int ji = 0; ji < kJsrCount; ++ji) {
                const double jsr = kJsrDb[ji];
                const uint64_t cseed =
                    cell_base_seed(cov, jsr, use_ir, ci, ji);
                const CellAgg ag =
                    run_cell(jsr, cov, use_ir, cseed, f_per, f_sum, fecn);

                const char *verdict =
                    (ag.pass_count == kRunsPerCell)
                        ? "PASS"
                        : ((ag.fail_count == kRunsPerCell) ? "FAIL" : "MIX");

                std::fprintf(
                    f_con,
                    " %5.0f   J/S      %d     %5.2f    %6.1f     %7.2f     "
                    "%d/%d %s\n",
                    jsr, kBlocksPerRun, ag.avg_blocks_ok, ag.avg_harq,
                    ag.avg_latency_ms, ag.pass_count, kRunsPerCell, verdict);
                std::fprintf(
                    f_log,
                    "jsr=%.0f cov=%.0f fec=%s pass=%d partial=%d fail=%d "
                    "avg_b=%.2f avg_h=%.2f\n",
                    jsr, cov, fecn, ag.pass_count, ag.partial_count,
                    ag.fail_count, ag.avg_blocks_ok, ag.avg_harq);
                std::fflush(f_con);
                std::fflush(f_log);
            }
            std::fprintf(f_con,
                         "-----------------------------------------------------------------"
                         "----\n");
        }
    }

    const auto t_end = std::chrono::steady_clock::now();
    const double sec = std::chrono::duration<double>(t_end - t_all).count();
    std::fprintf(f_log, "\nTotal elapsed: %.1f s\n", sec);
    std::fprintf(f_con, "\nTotal elapsed: %.1f s\n", sec);

    std::fclose(f_log);
    std::fclose(f_con);
    std::fclose(f_per);
    std::fclose(f_sum);
    return 0;
}

#include "../../HTS_LIM/HTS_Secure_Memory.cpp"
#include "../../HTS_LIM/HTS_Polar_Codec.cpp"
#include "../../HTS_LIM/HTS_FEC_HARQ.cpp"
#include "../../HTS_LIM/HTS_Walsh_Row_Converter.cpp"
#include "../../HTS_LIM/HTS_Holo_LPI.cpp"
#include "../../HTS_LIM/HTS_Walsh_Row_Permuter.cpp"
#include "HTS_Session_Derive_Stub.cpp"
#include "../../HTS_Jammer_STD/HTS_Jammer_STD.cpp"
#include "HTS_V400_Dispatcher_Local.cpp"
