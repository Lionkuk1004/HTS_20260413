// ============================================================================
// HTS_Noise_Detector_4D_Test_Brutal.cpp — v2 (2026-04-28)
// ============================================================================
#include "HTS_Noise_Detector_4D.hpp"
#include <cstdint>
#include <cstdio>
#include <cstring>
using namespace ProtectedEngine::NoiseDetect4D;
static GridCell g_tensor_4d[kSeedDim][kPhaseDim][kTimeDim];
static uint32_t g_rng_state = 1u;
static void rng_seed(uint32_t s) noexcept { g_rng_state = s ? s : 1u; }
static uint32_t rng_next() noexcept {
    g_rng_state = g_rng_state * 1664525u + 1013904223u;
    return g_rng_state;
}
static int32_t rng_range(int32_t lo, int32_t hi) noexcept {
    const uint32_t span = static_cast<uint32_t>(hi - lo + 1);
    return lo + static_cast<int32_t>(rng_next() % span);
}
static int32_t rng_gauss_q15(int32_t sigma_q15) noexcept {
    int32_t sum = 0;
    for (int i = 0; i < 12; ++i) {
        sum += static_cast<int32_t>(rng_next() & 0xFFFF) - 0x8000;
    }
    return (sum * sigma_q15) >> 18;
}
static void tensor_clear() noexcept {
    std::memset(g_tensor_4d, 0, sizeof(g_tensor_4d));
}
static void tensor_recompute_mag() noexcept {
    for (uint32_t s = 0; s < kSeedDim; ++s)
        for (uint32_t p = 0; p < kPhaseDim; ++p)
            for (uint32_t t = 0; t < kTimeDim; ++t) {
                g_tensor_4d[s][p][t].mag_q15 = mag_v15_q15(
                    g_tensor_4d[s][p][t].I_q15, g_tensor_4d[s][p][t].Q_q15);
            }
}
static void fill_noise(int32_t sigma_q15) noexcept {
    for (uint32_t s = 0; s < kSeedDim; ++s)
        for (uint32_t p = 0; p < kPhaseDim; ++p)
            for (uint32_t t = 0; t < kTimeDim; ++t) {
                g_tensor_4d[s][p][t].I_q15 = rng_gauss_q15(sigma_q15);
                g_tensor_4d[s][p][t].Q_q15 = rng_gauss_q15(sigma_q15);
            }
}
static void add_signal(uint32_t s, uint32_t p, uint32_t t,
                       int32_t amp_q15) noexcept {
    g_tensor_4d[s][p][t].I_q15 += amp_q15;
}
struct BrutalConfig {
    int32_t snr_db_lo;
    int32_t snr_db_hi;
    int32_t signal_amp_lo;
    int32_t signal_amp_hi;
    int32_t cw_count_lo;
    int32_t cw_count_hi;
    int32_t cw_amp_ratio_q8;
    int32_t alias_count_lo;
    int32_t alias_count_hi;
    int32_t alias_amp_ratio_q8;
    bool barrage_on;
    int32_t barrage_sigma_ratio_q8;
    bool edge_position;
};
struct BrutalResult {
    uint32_t total;
    uint32_t pass;
    uint32_t fail_to_alias;
    uint32_t fail_to_cw;
    uint32_t fail_other;
    int64_t noise_sum_avg;
};
static BrutalResult run_brutal(const BrutalConfig &cfg, uint32_t trials,
                               uint32_t seed_base) noexcept {
    BrutalResult res = {0u, 0u, 0u, 0u, 0u, 0};
    int64_t noise_acc = 0;
    for (uint32_t trial = 0; trial < trials; ++trial) {
        rng_seed(seed_base + trial);
        const int32_t sig_amp = rng_range(cfg.signal_amp_lo, cfg.signal_amp_hi);
        const int32_t snr_db = rng_range(cfg.snr_db_lo, cfg.snr_db_hi);
        int32_t noise_sigma;
        if (snr_db >= 20)
            noise_sigma = sig_amp / 10;
        else if (snr_db >= 10)
            noise_sigma = sig_amp / 3;
        else if (snr_db >= 0)
            noise_sigma = sig_amp;
        else if (snr_db >= -5)
            noise_sigma = sig_amp * 18 / 10;
        else if (snr_db >= -10)
            noise_sigma = sig_amp * 32 / 10;
        else
            noise_sigma = sig_amp * 56 / 10;
        uint32_t sig_s, sig_p, sig_t;
        if (cfg.edge_position) {
            sig_s = (rng_next() & 1u) ? 1u : (kSeedDim - 2);
            sig_p = (rng_next() & 1u) ? 1u : (kPhaseDim - 2);
            sig_t = (rng_next() & 1u) ? 1u : (kTimeDim - 2);
        } else {
            sig_s = static_cast<uint32_t>(
                rng_range(2, static_cast<int32_t>(kSeedDim) - 3));
            sig_p = static_cast<uint32_t>(
                rng_range(2, static_cast<int32_t>(kPhaseDim) - 3));
            sig_t = static_cast<uint32_t>(
                rng_range(2, static_cast<int32_t>(kTimeDim) - 3));
        }
        tensor_clear();
        const int32_t actual_sigma =
            (cfg.barrage_on)
                ? noise_sigma + (sig_amp * cfg.barrage_sigma_ratio_q8 / 256)
                : noise_sigma;
        fill_noise(actual_sigma);
        add_signal(sig_s, sig_p, sig_t, sig_amp);
        const int32_t cw_count = rng_range(cfg.cw_count_lo, cfg.cw_count_hi);
        for (int32_t c = 0; c < cw_count; ++c) {
            uint32_t cw_s, cw_p, cw_t;
            do {
                cw_s = static_cast<uint32_t>(
                    rng_range(0, static_cast<int32_t>(kSeedDim) - 1));
                cw_p = static_cast<uint32_t>(
                    rng_range(0, static_cast<int32_t>(kPhaseDim) - 1));
                cw_t = static_cast<uint32_t>(
                    rng_range(0, static_cast<int32_t>(kTimeDim) - 1));
            } while (cw_s == sig_s && cw_p == sig_p && cw_t == sig_t);
            const int32_t cw_amp = sig_amp * cfg.cw_amp_ratio_q8 / 256;
            add_signal(cw_s, cw_p, cw_t, cw_amp);
        }
        const int32_t alias_count =
            rng_range(cfg.alias_count_lo, cfg.alias_count_hi);
        uint32_t first_alias_s = sig_s, first_alias_p = sig_p,
                 first_alias_t = sig_t;
        for (int32_t a = 0; a < alias_count; ++a) {
            const int32_t off_s = rng_range(-4, 4);
            const int32_t off_p = rng_range(-2, 2);
            const int32_t off_t = rng_range(-2, 2);
            int32_t as = static_cast<int32_t>(sig_s) + off_s;
            int32_t ap = static_cast<int32_t>(sig_p) + off_p;
            int32_t at = static_cast<int32_t>(sig_t) + off_t;
            if (as < 0)
                as = 0;
            if (as >= static_cast<int32_t>(kSeedDim))
                as = static_cast<int32_t>(kSeedDim) - 1;
            if (ap < 0)
                ap = 0;
            if (ap >= static_cast<int32_t>(kPhaseDim))
                ap = static_cast<int32_t>(kPhaseDim) - 1;
            if (at < 0)
                at = 0;
            if (at >= static_cast<int32_t>(kTimeDim))
                at = static_cast<int32_t>(kTimeDim) - 1;
            if (static_cast<uint32_t>(as) == sig_s &&
                static_cast<uint32_t>(ap) == sig_p &&
                static_cast<uint32_t>(at) == sig_t)
                continue;
            const int32_t alias_amp = sig_amp * cfg.alias_amp_ratio_q8 / 256;
            add_signal(static_cast<uint32_t>(as), static_cast<uint32_t>(ap),
                       static_cast<uint32_t>(at), alias_amp);
            if (a == 0) {
                first_alias_s = static_cast<uint32_t>(as);
                first_alias_p = static_cast<uint32_t>(ap);
                first_alias_t = static_cast<uint32_t>(at);
            }
        }
        tensor_recompute_mag();
        DetectResult4D r = detect_noise_4d(g_tensor_4d, nullptr, 200, 150);
        noise_acc += r.noise_total_q15;
        ++res.total;
        const bool ok =
            (r.signal_seed_idx == sig_s && r.signal_phase_idx == sig_p &&
             r.signal_time_idx == sig_t);
        if (ok) {
            ++res.pass;
        } else {
            const bool got_alias =
                (alias_count > 0 && r.signal_seed_idx == first_alias_s &&
                 r.signal_phase_idx == first_alias_p &&
                 r.signal_time_idx == first_alias_t);
            if (got_alias)
                ++res.fail_to_alias;
            else if (cw_count > 0)
                ++res.fail_to_cw;
            else
                ++res.fail_other;
        }
    }
    res.noise_sum_avg = (res.total > 0) ? (noise_acc / res.total) : 0;
    return res;
}
static void print_result(const char *name, const BrutalResult &r) noexcept {
    const double rate = (r.total > 0) ? (100.0 * r.pass / r.total) : 0.0;
    std::printf("[%-30s] PASS %4u/%4u (%5.1f%%)  fail_alias=%-4u  fail_cw=%-4u "
                " fail_other=%-4u  noise_avg=%lld\n",
                name, r.pass, r.total, rate, r.fail_to_alias, r.fail_to_cw,
                r.fail_other, static_cast<long long>(r.noise_sum_avg));
}
int main() {
    std::printf(
        "============================================================\n");
    std::printf("  4D Noise Detector — BRUTAL Test v2 (n=1000/scenario)\n");
    std::printf(
        "============================================================\n\n");
    const uint32_t N = 1000u;
    // ADV1: SNR random sweep
    {
        BrutalConfig cfg = {-15, 20, 8000, 12000, 0, 0,    0,
                            0,   0,  0,    false, 0, false};
        BrutalResult r = run_brutal(cfg, N, 1000);
        print_result("ADV1_snr_random_-15to+20dB", r);
    }
    // ADV1b: 고정 SNR cliff (배열 사용)
    const int32_t snr_levels[] = {20, 10, 0, -5, -10, -15};
    const int snr_count = sizeof(snr_levels) / sizeof(snr_levels[0]);
    for (int k = 0; k < snr_count; ++k) {
        const int32_t snr = snr_levels[k];
        BrutalConfig cfg = {snr, snr, 8000, 12000, 0, 0,    0,
                            0,   0,   0,    false, 0, false};
        BrutalResult r =
            run_brutal(cfg, N, 2000u + static_cast<uint32_t>(snr + 100));
        char name[64];
        std::snprintf(name, sizeof(name), "ADV1b_snr_%+ddB_fixed", snr);
        print_result(name, r);
    }
    // ADV2: 다중 CW
    {
        BrutalConfig cfg = {0, 20, 8000, 12000, 1, 5,    192,
                            0, 0,  0,    false, 0, false};
        BrutalResult r = run_brutal(cfg, N, 3000);
        print_result("ADV2_multi_cw_1to5", r);
    }
    // ADV3: saddle alias
    {
        BrutalConfig cfg = {0, 20, 8000, 12000, 0, 0,    0,
                            1, 3,  230,  false, 0, false};
        BrutalResult r = run_brutal(cfg, N, 4000);
        print_result("ADV3_saddle_alias_1to3", r);
    }
    // ADV4: CW > signal
    {
        BrutalConfig cfg = {5, 20, 8000, 12000, 1, 3,    320,
                            0, 0,  0,    false, 0, false};
        BrutalResult r = run_brutal(cfg, N, 5000);
        print_result("ADV4_cw_stronger_than_signal", r);
    }
    // ADV5: 모든 jam 동시
    {
        BrutalConfig cfg = {0, 15, 8000, 12000, 1,   3,    200,
                            1, 2,  200,  true,  128, false};
        BrutalResult r = run_brutal(cfg, N, 6000);
        print_result("ADV5_all_jam_combined", r);
    }
    // ADV6: 가장자리
    {
        BrutalConfig cfg = {-5, 15, 8000, 12000, 0, 2,   192,
                            0,  1,  200,  false, 0, true};
        BrutalResult r = run_brutal(cfg, N, 7000);
        print_result("ADV6_edge_position", r);
    }
    // ADV7: 모든 random combination
    {
        BrutalConfig cfg = {-10, 20, 5000, 15000, 0,  5,    256,
                            0,   3,  220,  true,  64, false};
        BrutalResult r = run_brutal(cfg, N, 8000);
        print_result("ADV7_random_combination_all", r);
    }
    std::printf("\n=== BRUTAL 테스트 완료 ===\n");
    return 0;
}
