// =============================================================================
/// @file HTS_Phase3_Alpha_Harness.cpp
/// @brief Phase 3-α — 432×1,000 한계 스크리닝 (Conv / HTS_FEC_POLAR_DISABLE)
///
/// Dispatcher: `HTS_V400_Dispatcher_Local` + `HTS_Harq_Matrix_Test.cpp` 와 동일
/// preamble 가드·HARQ 루프. 재밍: `HTS_Jammer_STD` (SPEC_002). 통계:
/// Clopper-Pearson exact (`HTS_Clopper_Pearson.hpp`).
///
/// **Coverage (시간 점유) — 모델 (A)**: 페이로드 칩열 길이 `pay_chips` 중
/// `jam_chips = ceil(pay_chips*cov%/100)` 칸을 **연속 구간 1개**로 덮되,
/// 구간 시작 오프셋은 **매 trial** `trial_seed`에서 유도한 RNG 로
/// `[0, pay_chips - jam_chips]` 균등 (전 구간 덮이면 시작 0). 프리앰블·헤더는
/// `Build_Packet` 오프셋 `pay_off` 이전이며 재밍은 `I+pay_start` 구간에만 적용.
///
/// 빌드: `HTS_TEST\t6_sim` 에서 `build_phase3_alpha.cmd` (HTS_LIM 소스 링크).
// =============================================================================
#if defined(__arm__) || defined(__TARGET_ARCH_ARM)
#error "[HTS_FATAL] PC 전용"
#endif

#ifndef HTS_PHASE3_ALPHA_TX_CFO
#define HTS_PHASE3_ALPHA_TX_CFO 0
#endif

#include "HTS_Clopper_Pearson.hpp"
#include "HTS_FEC_HARQ.hpp"
#include "HTS_Jammer_STD.hpp"
#include "../t6_sim/HTS_V400_Dispatcher_Local.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <random>
#include <string>
#include <vector>

using ProtectedEngineLocal::DecodedPacket;
using ProtectedEngineLocal::HTS_V400_Dispatcher;
using ProtectedEngineLocal::PayloadMode;

namespace {

constexpr int kTrialsAlpha = 1000;
constexpr int kJamCount = 6;
constexpr int kCovCount = 6;
constexpr int kJsrCount = 6;
constexpr int kLpiCount = 2;
constexpr int kCovPct[kCovCount] = {20, 30, 35, 40, 45, 100};
constexpr int kJsrDb[kJsrCount] = {-5, 0, 10, 20, 30, 40};

constexpr int16_t kAmp = 1000;
constexpr int kPreReps = 4;
constexpr int kPreBoost = 1;
constexpr int kMaxHarq = 8;
constexpr int kGuard = 256;
constexpr int kMaxC =
    2048 + (ProtectedEngine::FEC_HARQ::NSYM64 + kPreReps + 4) * 64;

struct ScenarioResult {
    const char *jam_name{};
    int cov_pct{};
    int jsr_db{};
    const char *lpi_mode{};
    int trials_run{};
    int trials_pass{};
    double success_rate{};
    double cp_lower{};
    double cp_upper{};
    int avg_harq_rounds{};
};

static DecodedPacket g_last{};

static void on_pkt(const DecodedPacket &p) { g_last = p; }

static void setup_disp(HTS_V400_Dispatcher &d, uint32_t seed,
                       bool use_ir) noexcept {
    d.Set_Seed(seed);
    d.Set_IR_Mode(use_ir);
    d.Set_Preamble_Boost(kPreBoost);
    d.Set_Preamble_Reps(kPreReps);
    d.Set_Packet_Callback(on_pkt);
    d.Update_Adaptive_BPS(1000);
    d.Set_Lab_IQ_Mode_Jam_Harness();
}

/// Option 3 (v2): TX 측 연속 위상 회전 (CFO Hz). chip_rate = Jammer SPEC F_CHIP_HZ.
/// TX CFO: `chip_index_offset` 칩부터 연속 위상(전체 버스트와 동일 ω, φ0+offset·Δθ).
static void apply_hw_cfo_tx_rotate_from(int16_t *I, int16_t *Q, int n_chips,
                                        double cfo_hz, double chip_rate_hz,
                                        uint64_t phi_seed,
                                        int chip_index_offset) noexcept {
    if (n_chips <= 0 || I == nullptr || Q == nullptr || chip_rate_hz <= 0.0) {
        return;
    }
    constexpr double kTwoPi =
        6.283185307179586476925286766559005768394338798750211;
    const double phase_step = kTwoPi * (cfo_hz / chip_rate_hz);
    std::mt19937_64 rng(phi_seed ^ 0x3C0FFEE0A11CEu);
    std::uniform_real_distribution<double> phi0(0.0, kTwoPi);
    double phase =
        phi0(rng) + static_cast<double>(chip_index_offset) * phase_step;
    for (int c = 0; c < n_chips; ++c) {
        const double cs = std::cos(phase);
        const double sn = std::sin(phase);
        const double di = static_cast<double>(I[c]);
        const double dq = static_cast<double>(Q[c]);
        const double inew = di * cs - dq * sn;
        const double qnew = di * sn + dq * cs;
        I[c] = static_cast<int16_t>(std::clamp(inew, -32768.0, 32767.0));
        Q[c] = static_cast<int16_t>(std::clamp(qnew, -32768.0, 32767.0));
        phase += phase_step;
    }
}

static int jam_payload_chips(int pay_chips, int cov_pct) noexcept {
    if (pay_chips <= 0) {
        return 0;
    }
    if (cov_pct >= 100) {
        return pay_chips;
    }
    if (cov_pct <= 0) {
        return 0;
    }
    int j = (pay_chips * cov_pct + 99) / 100;
    if (j < 1) {
        j = 1;
    }
    if (j > pay_chips) {
        j = pay_chips;
    }
    return j;
}

/// trial_seed(라운드별 pos_seed)로 페이로드 내 버스트 시작 균등 샘플.
static int jam_burst_start_payload(int pay_chips, int jam_chips,
                                   uint64_t pos_seed) noexcept {
    if (jam_chips <= 0 || pay_chips <= 0) {
        return 0;
    }
    if (jam_chips >= pay_chips) {
        return 0;
    }
    const int span = pay_chips - jam_chips + 1;
    const uint32_t s0 = static_cast<uint32_t>(pos_seed ^ 0xC0FFEEC0u);
    const uint32_t s1 = static_cast<uint32_t>((pos_seed >> 32) ^ 0xA17C5EEDu);
    std::mt19937 rng_pos(s0 ^ s1);
    std::uniform_int_distribution<int> dist(0, span - 1);
    return dist(rng_pos);
}

/// 페이로드 버퍼 `payI/payQ` 상에서 길이 `jam_chips` 연속 구간에만 재밍.
/// 호출부에서 `payI += burst_start` 로 이미 오프셋 적용.
static void apply_jam_burst_slice(int16_t *sliceI, int16_t *sliceQ, int jam_chips,
                                  int chips_per_sym,
                                  HTS_Jammer_STD::ChannelType ch,
                                  double intensity, double P_s,
                                  std::mt19937 &rng_jam) noexcept {
    using Ch = HTS_Jammer_STD::ChannelType;
    using SP = HTS_Jammer_STD::StdParams;
    if (jam_chips <= 0) {
        return;
    }
    const int N = jam_chips;
    const int nsym = N / chips_per_sym;
    (void)nsym;
    switch (ch) {
    case Ch::Clean:
        return;
    case Ch::AWGN:
        HTS_Jammer_STD::Add_AWGN(sliceI, sliceQ, N, intensity, P_s, rng_jam);
        return;
    case Ch::Barrage:
        HTS_Jammer_STD::Add_Barrage(sliceI, sliceQ, N, intensity, P_s, rng_jam);
        return;
    case Ch::CW:
        HTS_Jammer_STD::Add_CW(sliceI, sliceQ, N, intensity, P_s,
                               SP::CW_F_OFFSET_HZ, SP::F_CHIP_HZ, rng_jam);
        return;
    case Ch::Pulse:
        HTS_Jammer_STD::Add_Pulse(sliceI, sliceQ, N, intensity, P_s,
                                  SP::PULSE_DUTY, SP::PULSE_T_PERIOD,
                                  SP::CW_F_OFFSET_HZ, SP::F_CHIP_HZ, rng_jam);
        return;
    case Ch::MultiTone:
        HTS_Jammer_STD::Add_MultiTone(sliceI, sliceQ, N, intensity, P_s,
                                      SP::MULTI_N_TONES, SP::F_CHIP_HZ,
                                      SP::F_CHIP_HZ, rng_jam);
        return;
    case Ch::Swept:
        HTS_Jammer_STD::Add_Swept(sliceI, sliceQ, N, intensity, P_s,
                                  SP::SWEPT_F_START_HZ, SP::SWEPT_F_END_HZ,
                                  SP::SWEPT_RATE_HZ_S, SP::F_CHIP_HZ, rng_jam);
        return;
    case Ch::Partial_Barrage:
        return;
    }
}

struct TrialResult {
    bool crc_ok{};
    int harq_k{};
    bool decode_called{};
    int max_phase{};
    int jam_burst_start{};
    int jam_burst_len{};
    int payload_chips{};
};

static TrialResult run_trial(uint64_t trial_seed, uint32_t ds,
                             const uint8_t *info, bool is_voice,
                             HTS_Jammer_STD::ChannelType ch, double jsr_axis,
                             int cov_pct, bool lpi_tim, bool use_ir) noexcept {
    TrialResult r{};
    r.crc_ok = false;
    r.harq_k = kMaxHarq;
    r.decode_called = false;
    r.max_phase = 0;

    HTS_V400_Dispatcher tx;
    HTS_V400_Dispatcher rx;
    setup_disp(tx, ds, use_ir);
    setup_disp(rx, ds, use_ir);
    if (lpi_tim) {
        uint32_t lpi_seed[4];
        uint64_t x = static_cast<uint64_t>(ds) ^ 0xA5A5A5A5A5A5A5A5ull;
        for (int i = 0; i < 4; ++i) {
            x = x * 6364136223846793005ull + 1ull;
            lpi_seed[i] = static_cast<uint32_t>(x >> 32);
        }
        tx.Enable_Holo_LPI(lpi_seed);
        rx.Enable_Holo_LPI(lpi_seed);
    }

    const PayloadMode mode =
        is_voice ? PayloadMode::VOICE : PayloadMode::DATA;
    const int chips_per_sym = is_voice ? 16 : 64;
    const int pay_off = (kPreReps + 1) * 64 + 128;
    const int pre_guard = kGuard;

    auto bump = [&r](int ph) {
        if (ph < 3) {
            if (ph > r.max_phase) {
                r.max_phase = ph;
            }
        } else if (r.max_phase < 2) {
            r.max_phase = 2;
        }
    };

    int16_t I[kMaxC], Q[kMaxC];
    const double mc_intensity =
        (ch == HTS_Jammer_STD::ChannelType::AWGN) ? (-jsr_axis) : jsr_axis;
    std::mt19937 rng_jam(static_cast<uint32_t>((trial_seed ^ 0xBEEFCAFEull) &
                                             0xFFFFFFFFu));

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
#if defined(HTS_PHASE3_ALPHA_TX_CFO) && HTS_PHASE3_ALPHA_TX_CFO
        {
            using SP = HTS_Jammer_STD::StdParams;
            std::mt19937_64 rng_hz(
                trial_seed ^ (0xD00DF00Dull * static_cast<uint64_t>(round)));
            std::uniform_real_distribution<double> hz_dist(-1000.0, 1000.0);
            const double cfo_hz = hz_dist(rng_hz);
            apply_hw_cfo_tx_rotate_from(
                I, Q, n, cfo_hz, SP::F_CHIP_HZ,
                trial_seed ^ (static_cast<uint64_t>(round) * 0x9E3779B97F4A7C15ull),
                0);
        }
#endif
        const int nsym = pay_chips / chips_per_sym;
        double P_s = HTS_Jammer_STD::Measure_Signal_Power(I + pay_start,
                                                          Q + pay_start, pay_chips);
        if (P_s <= 0.0) {
            P_s = 1.0;
        }
        const int jam_chips = jam_payload_chips(pay_chips, cov_pct);
        const uint64_t pos_seed =
            trial_seed ^ (static_cast<uint64_t>(round) * 0x100000001B3ull);
        const int burst_start =
            jam_burst_start_payload(pay_chips, jam_chips, pos_seed);
        r.jam_burst_len = jam_chips;
        r.payload_chips = pay_chips;
        r.jam_burst_start = burst_start;
        apply_jam_burst_slice(I + pay_start + burst_start, Q + pay_start + burst_start,
                              jam_chips, chips_per_sym, ch, mc_intensity, P_s,
                              rng_jam);

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
        } else {
            for (int i = 0; i < pay_chips; ++i) {
                rx.Feed_Retx_Chip(I[pay_start + i], Q[pay_start + i]);
                bump(static_cast<int>(rx.Get_Phase()));
            }
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
            if (bit_err == 0) {
                r.crc_ok = true;
                int hk = g_last.harq_k;
                if (hk < 1) {
                    hk = 1;
                }
                if (hk > kMaxHarq) {
                    hk = kMaxHarq;
                }
                r.harq_k = hk;
                break;
            }
        }
        if (round == 1 && !rx.Is_Retx_Ready()) {
            break;
        }
    }
    return r;
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

const char *jam_name(int j) noexcept {
    static const char *kN[kJamCount] = {"J1_AWGN", "J2_CW", "J3_Pulse",
                                        "J4_Barrage", "J5_Multi", "J6_Swept"};
    return (j >= 0 && j < kJamCount) ? kN[j] : "J?";
}

HTS_Jammer_STD::ChannelType jam_ch(int j) noexcept {
    using Ch = HTS_Jammer_STD::ChannelType;
    static const Ch kC[kJamCount] = {Ch::AWGN, Ch::CW, Ch::Pulse,
                                     Ch::Barrage, Ch::MultiTone, Ch::Swept};
    return (j >= 0 && j < kJamCount) ? kC[j] : Ch::Clean;
}

uint64_t scen_seed(int jam_idx, int cov_idx, int jsr_idx, int lpi_idx) noexcept {
    uint64_t s = 0x5A5A5A5A00000000ull;
    s ^= static_cast<uint64_t>(jam_idx) * 0x9E3779B97F4A7C15ull;
    s ^= static_cast<uint64_t>(cov_idx) << 20;
    s ^= static_cast<uint64_t>(jsr_idx) << 28;
    s ^= static_cast<uint64_t>(lpi_idx) << 36;
    s = (s ^ (s >> 30)) * 0xBF58476D1CE4E5B9ull;
    s = (s ^ (s >> 27)) * 0x94D049BB133111EBull;
    return s ^ (s >> 31);
}

ScenarioResult run_scen(int jam_idx, int cov_idx, int jsr_idx, int lpi_idx,
                        int trials) {
    ScenarioResult r{};
    r.jam_name = jam_name(jam_idx);
    r.cov_pct = kCovPct[cov_idx];
    r.jsr_db = kJsrDb[jsr_idx];
    r.lpi_mode = (lpi_idx == 0) ? "NONE" : "TIM";
    const auto ch = jam_ch(jam_idx);
    const double jsr = static_cast<double>(r.jsr_db);
    const bool lpi = (lpi_idx == 1);
    int pass = 0;
    long long harq_sum = 0;
    const uint64_t base = scen_seed(jam_idx, cov_idx, jsr_idx, lpi_idx);
    for (int t = 0; t < trials; ++t) {
        const uint64_t trial_seed =
            base + static_cast<uint64_t>(t) * 0x100000001B3ull;
        uint8_t info[8];
        fill_info(static_cast<uint32_t>(trial_seed & 0xFFFFFFFFu), t, info);
        const TrialResult tr = run_trial(
            trial_seed, static_cast<uint32_t>(trial_seed & 0xFFFFFFFFu), info,
            false, ch, jsr, r.cov_pct, lpi, true);
        if (tr.crc_ok) {
            ++pass;
        }
        harq_sum += tr.harq_k;
    }
    r.trials_run = trials;
    r.trials_pass = pass;
    r.success_rate =
        trials > 0 ? static_cast<double>(pass) / static_cast<double>(trials)
                   : 0.0;
    const auto ci = HTS_Clopper_Pearson::Compute(pass, trials, 0.05);
    r.cp_lower = ci.p_lower;
    r.cp_upper = ci.p_upper;
    r.avg_harq_rounds =
        trials > 0 ? static_cast<int>((harq_sum + trials / 2) / trials) : 0;
    return r;
}

const char *zone_label(double p) noexcept {
    if (p >= 0.90) {
        return "PASS";
    }
    if (p >= 0.50) {
        return "RISK";
    }
    return "FAIL";
}

} // namespace

int main(int argc, char **argv) {
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--smoke-position-check") {
            const int pay_off = (kPreReps + 1) * 64 + 128;
            std::FILE *fout = std::fopen("smoke_pos.txt", "wb");
            if (fout == nullptr) {
                std::fprintf(stderr, "ERROR: cannot open smoke_pos.txt\n");
                return 1;
            }
            std::fprintf(fout,
                         "# Phase3-alpha --smoke-position-check\n"
                         "# Model (A): random burst start within payload only\n"
                         "# chips_before_payload pay_off=%d (preamble+header)\n"
                         "# Scenario: J1_AWGN cov=20%% JSR=0dB NONE IR=DATA\n\n",
                         pay_off);
            const uint64_t base = scen_seed(0, 0, 1, 0);
            int passc = 0;
            for (int t = 0; t < 10; ++t) {
                const uint64_t trial_seed =
                    base + static_cast<uint64_t>(t) * 0x100000001B3ull;
                uint8_t info[8];
                fill_info(static_cast<uint32_t>(trial_seed & 0xFFFFFFFFu), t,
                          info);
                const TrialResult tr = run_trial(
                    trial_seed, static_cast<uint32_t>(trial_seed & 0xFFFFFFFFu),
                    info, false, jam_ch(0), 0.0, 20, false, true);
                if (tr.crc_ok) {
                    ++passc;
                }
                std::fprintf(
                    fout,
                    "trial=%d trial_seed=0x%016llx burst_start=%d burst_len=%d "
                    "payload_chips=%d crc_ok=%d\n",
                    t, static_cast<unsigned long long>(trial_seed),
                    tr.jam_burst_start, tr.jam_burst_len, tr.payload_chips,
                    static_cast<int>(tr.crc_ok));
            }
            std::fprintf(fout, "\npass_crc=%d / 10 (expect >=9 for sanity)\n",
                         passc);
            std::fclose(fout);
            std::fprintf(stderr,
                         "[Phase3-Alpha] wrote smoke_pos.txt pass_crc=%d/10\n",
                         passc);
            return 0;
        }
    }

    bool smoke = false;
    bool mini = false;
    bool out_fixed = false;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--smoke") {
            smoke = true;
        }
        if (std::string(argv[i]) == "--mini") {
            mini = true;
        }
        if (std::string(argv[i]) == "--out-fixed") {
            out_fixed = true;
        }
    }
    const int trials = smoke ? 10 : kTrialsAlpha;
    const int j0 = 0;
    const int j1 = mini ? 1 : kJamCount;
    const int c0 = 0;
    const int c1 = mini ? 1 : kCovCount;
    const int s0 = 0;
    const int s1 = mini ? 1 : kJsrCount;
    const int l0 = 0;
    const int l1 = mini ? 1 : kLpiCount;

    const char *const sum_name =
        out_fixed ? "phase3_alpha_fixed_summary.txt" : "phase3_alpha_summary.txt";
    const char *const heat_name = out_fixed ? "phase3_alpha_fixed_heatmap.csv"
                                          : "phase3_alpha_heatmap.csv";

    std::fprintf(stderr,
                 "[Phase3-Alpha] trials=%d smoke=%d mini=%d out_fixed=%d "
                 "LocalDispatcher\n",
                 trials, smoke ? 1 : 0, mini ? 1 : 0, out_fixed ? 1 : 0);
    const auto t0 = std::chrono::steady_clock::now();

    std::printf("jam,cov_pct,jsr_db,lpi,trials,pass,success_rate,cp_lower,"
                "cp_upper,avg_harq,zone\n");
    std::fflush(stdout);

    std::vector<ScenarioResult> all;
    int n_pass = 0, n_risk = 0, n_fail = 0;
    long long jam_pass[kJamCount]{};
    int lpi_p[kLpiCount]{}, lpi_r[kLpiCount]{}, lpi_f[kLpiCount]{};
    double cov_worst[kCovCount];
    bool cov_seen[kCovCount];
    for (int i = 0; i < kCovCount; ++i) {
        cov_worst[i] = 1.0;
        cov_seen[i] = false;
    }
    double jsr_mean[kJsrCount]{};

    for (int j = j0; j < j1; ++j) {
        for (int c = c0; c < c1; ++c) {
            for (int s = s0; s < s1; ++s) {
                for (int l = l0; l < l1; ++l) {
                    ScenarioResult r = run_scen(j, c, s, l, trials);
                    const char *z = zone_label(r.success_rate);
                    std::printf("%s,%d,%d,%s,%d,%d,%.4f,%.4f,%.4f,%d,%s\n",
                                r.jam_name, r.cov_pct, r.jsr_db, r.lpi_mode,
                                r.trials_run, r.trials_pass, r.success_rate,
                                r.cp_lower, r.cp_upper, r.avg_harq_rounds, z);
                    std::fflush(stdout);
                    all.push_back(r);
                    if (r.success_rate >= 0.90) {
                        ++n_pass;
                        ++jam_pass[j];
                        ++lpi_p[l];
                    } else if (r.success_rate >= 0.50) {
                        ++n_risk;
                        ++lpi_r[l];
                    } else {
                        ++n_fail;
                        ++lpi_f[l];
                    }
                    if (!cov_seen[c] || r.success_rate < cov_worst[c]) {
                        cov_worst[c] = r.success_rate;
                        cov_seen[c] = true;
                    }
                }
            }
        }
    }
    const int ncells = static_cast<int>(all.size());
    const int cells_per_jam = (c1 - c0) * (s1 - s0) * (l1 - l0);
    (void)cells_per_jam;
    for (int si = 0; si < kJsrCount; ++si) {
        double sm = 0.0;
        int cnt = 0;
        for (const auto &r : all) {
            if (r.jsr_db == kJsrDb[si]) {
                sm += r.success_rate;
                ++cnt;
            }
        }
        jsr_mean[si] = (cnt > 0) ? sm / static_cast<double>(cnt) : 0.0;
    }

    const auto t1 = std::chrono::steady_clock::now();
    const double wall_min =
        std::chrono::duration<double>(t1 - t0).count() / 60.0;

    std::FILE *fs = std::fopen(sum_name, "wb");
    if (fs != nullptr) {
        std::fprintf(fs, "Phase 3-alpha (LocalDispatcher)\n");
        std::fprintf(fs, "trials=%d n_cells=%d wall_min=%.4f\n\n", trials,
                     ncells, wall_min);
        std::fprintf(fs, "PASS>=90%%: %d  RISK: %d  FAIL: %d\n\n", n_pass,
                     n_risk, n_fail);
        std::fprintf(fs, "per_jam PASS_cells:\n");
        for (int j = 0; j < kJamCount; ++j) {
            std::fprintf(fs, "  %s: %lld\n", jam_name(j),
                         static_cast<long long>(jam_pass[j]));
        }
        std::fprintf(fs, "\nLPI NONE: pass=%d risk=%d fail=%d\n", lpi_p[0],
                     lpi_r[0], lpi_f[0]);
        std::fprintf(fs, "LPI TIM:  pass=%d risk=%d fail=%d\n", lpi_p[1],
                     lpi_r[1], lpi_f[1]);
        std::fprintf(fs, "\nCoverage min success_rate:\n");
        for (int c = 0; c < kCovCount; ++c) {
            if (cov_seen[c]) {
                std::fprintf(fs, "  %d%%: %.4f\n", kCovPct[c], cov_worst[c]);
            }
        }
        std::fprintf(fs, "\nJSR mean success_rate:\n");
        for (int si = 0; si < kJsrCount; ++si) {
            std::fprintf(fs, "  %+d dB: %.4f\n", kJsrDb[si], jsr_mean[si]);
        }
        int t1p = 0, t1r = 0, t1f = 0, t2p = 0, t2r = 0, t2f = 0, t3p = 0,
            t3r = 0, t3f = 0;
        for (const auto &r : all) {
            const int jd = r.jsr_db;
            const char *z = zone_label(r.success_rate);
            const int zp = (std::strcmp(z, "PASS") == 0) ? 1 : 0;
            const int zr = (std::strcmp(z, "RISK") == 0) ? 1 : 0;
            const int zf = (std::strcmp(z, "FAIL") == 0) ? 1 : 0;
            if (jd >= 20 && jd <= 30) {
                t1p += zp;
                t1r += zr;
                t1f += zf;
            }
            if (jd >= 5 && jd <= 15) {
                t2p += zp;
                t2r += zr;
                t2f += zf;
            }
            if (jd >= 10 && jd <= 20) {
                t3p += zp;
                t3r += zr;
                t3f += zf;
            }
        }
        std::fprintf(fs, "\nMIL coarse (overlapping JSR bins vs grid):\n");
        std::fprintf(fs, "  I   +20..+30: pass=%d risk=%d fail=%d\n", t1p,
                     t1r, t1f);
        std::fprintf(fs, "  II  +5..+15:  pass=%d risk=%d fail=%d\n", t2p,
                     t2r, t2f);
        std::fprintf(fs, "  III +10..+20: pass=%d risk=%d fail=%d\n", t3p,
                     t3r, t3f);
        std::fprintf(fs, "\nBeta candidates (RISK), max 64:\n");
        int nb = 0;
        for (const auto &r : all) {
            if (r.success_rate >= 0.50 && r.success_rate < 0.90) {
                std::fprintf(fs, "  %s cov=%d jsr=%d %s %.4f\n", r.jam_name,
                             r.cov_pct, r.jsr_db, r.lpi_mode, r.success_rate);
                ++nb;
                if (nb >= 64) {
                    break;
                }
            }
        }
        std::fclose(fs);
    }
    std::FILE *fh = std::fopen(heat_name, "wb");
    if (fh != nullptr) {
        std::fprintf(fh, "jam,lpi,jsr_db,cov_pct,success_rate,zone\n");
        for (const auto &r : all) {
            std::fprintf(fh, "%s,%s,%d,%d,%.4f,%s\n", r.jam_name, r.lpi_mode,
                         r.jsr_db, r.cov_pct, r.success_rate,
                         zone_label(r.success_rate));
        }
        std::fclose(fh);
    }
    std::fprintf(stderr, "[Phase3-Alpha] done cells=%d wall=%.3f min\n", ncells,
                 wall_min);
    return 0;
}

// Harq_Matrix_Test 와 동일: t6_sim 복사본 Jammer 구현 단일 TU 포함
#include "../t6_sim/HTS_Jammer_STD.cpp"
