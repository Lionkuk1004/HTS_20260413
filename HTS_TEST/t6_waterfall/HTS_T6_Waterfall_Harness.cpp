// =============================================================================
// HTS_T6_Waterfall_Harness.cpp — Waterfall matrix v2 (meta 50 × inner 50)
// Trial chain mirrors HTS_TEST/t6_mc/HTS_T6_MC_Harness.cpp (MC harness unchanged).
// =============================================================================
#if defined(__arm__) || defined(__TARGET_ARCH_ARM)
#error "[HTS_FATAL] PC 전용"
#endif

#include "HTS_V400_Dispatcher.hpp"
#include "HTS_Walsh_Row_Converter.hpp"
#include "../t6_sim/HTS_BER_PER_Measure.hpp"
#include "WaterfallSpec.hpp"
#if defined(HTS_LLR_DIAG)
#include "HTS_LLR_Diag.hpp"
#endif
#if defined(HTS_ROW_CONSISTENCY_DIAG)
#include "HTS_Row_Consistency_Diag.hpp"
#endif

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <vector>

using ProtectedEngine::DecodedPacket;
using ProtectedEngine::FEC_HARQ;
using ProtectedEngine::HTS_V400_Dispatcher;
using ProtectedEngine::PayloadMode;

namespace WaterfallHarness {

constexpr int16_t kAmp = 1000;
constexpr int kPreReps = 4;
constexpr int kPreBoost = 1;
constexpr int kMaxC = 2048 + (FEC_HARQ::NSYM64 + kPreReps + 4) * 64;
constexpr int kGuard = 256;
constexpr double kPi = 3.14159265358979323846;
constexpr double kChipRate = 200000.0;

[[nodiscard]] std::uint32_t splitmix32(std::uint32_t z) noexcept {
    z += 0x9E3779B9u;
    z = (z ^ (z >> 16u)) * 0x85EBCA6Bu;
    z = (z ^ (z >> 13u)) * 0xC2B2AE35u;
    return z ^ (z >> 16u);
}

[[nodiscard]] std::uint32_t hash_cstr(const char* p) noexcept {
    std::uint32_t h = 2166136261u;
    if (p == nullptr) {
        return h;
    }
    while (*p != '\0') {
        h ^= static_cast<std::uint32_t>(static_cast<unsigned char>(*p++));
        h *= 16777619u;
    }
    return h;
}

struct TrialMetrics {
    bool pass{};
    int bit_errors{64};
};

struct TxPkt {
    int16_t I[kMaxC]{};
    int16_t Q[kMaxC]{};
    std::uint8_t info[8]{};
    int n{};
};

struct TrialParams {
    std::uint32_t tx_ds{};
    std::uint32_t barrage_rng_seed{};
    std::uint32_t awgn_rng_seed{};
    double snr_db{};
    double jsr_barr_db{};
    double jsr_cw_db{};
    double period_chips{};
    double cfo_hz{};
    int timing_extra{};
};

struct TrialResult {
    bool pass{};
    int bit_errors{64};
    int total_bits{64};
    double llr_reliability_bin{0.0};
    double row_entropy{0.0};
    double row_concentration{0.0};
};

struct MetaResult {
    int meta_idx{};
    int pass_count{};
    double pass_rate{};
    double llr_bin_mean{};
    double row_H_mean{};
    double row_conc_mean{};
};

struct SweepPointAggregate {
    const char* scenario_name{};
    SweepAxis axis{};
    double sweep_value_db{};

    int grand_N{};
    int grand_pass{};
    double grand_pass_rate{};
    double grand_ci_lo{};
    double grand_ci_hi{};

    double meta_mean_rate{};
    double meta_std_rate{};
    double meta_min_rate{};
    double meta_max_rate{};

    double llr_bin_grand_mean{};
    double row_H_grand_mean{};
    double row_conc_grand_mean{};
};

static DecodedPacket g_last{};
static void on_pkt(const DecodedPacket& p) noexcept { g_last = p; }

static void setup(HTS_V400_Dispatcher& d, std::uint32_t seed) noexcept {
    d.Set_Seed(seed);
    d.Set_IR_Mode(true);
    d.Set_Preamble_Boost(kPreBoost);
    d.Set_Preamble_Reps(kPreReps);
    d.Set_Packet_Callback(on_pkt);
    d.Update_Adaptive_BPS(1000);
    d.Set_Lab_IQ_Mode_Jam_Harness();
}

static void fill_info(std::uint32_t seed, int t, std::uint8_t* info) noexcept {
    std::uint32_t s = seed + static_cast<std::uint32_t>(t) * 0x6C62272Eu + 1u;
    for (int b = 0; b < 8; ++b) {
        s += 0x9E3779B9u;
        std::uint32_t z = s;
        z = (z ^ (z >> 16u)) * 0x45D9F3Bu;
        z = (z ^ (z >> 16u)) * 0x45D9F3Bu;
        z = z ^ (z >> 16u);
        info[b] = static_cast<std::uint8_t>(z & 0xFFu);
    }
}

[[nodiscard]] static int16_t sat16(double v) noexcept {
    if (v > 32767.0) return 32767;
    if (v < -32768.0) return -32768;
    return static_cast<int16_t>(std::lround(v));
}

static TxPkt build_tx(std::uint32_t ds, int t) noexcept {
    TxPkt pkt{};
    HTS_V400_Dispatcher tx;
    setup(tx, ds);
    fill_info(ds, t, pkt.info);
    pkt.n = tx.Build_Packet(PayloadMode::DATA, pkt.info, 8, kAmp, pkt.I, pkt.Q, kMaxC);
    return pkt;
}

static TrialMetrics feed_raw_ext(std::uint32_t ds, const int16_t* rxI, const int16_t* rxQ, int n,
                                 const std::uint8_t* expected, int pre_guard = kGuard) noexcept {
    TrialMetrics m{};
    g_last = DecodedPacket{};
    HTS_V400_Dispatcher rx;
    setup(rx, ds);
    for (int i = 0; i < pre_guard; ++i) rx.Feed_Chip(0, 0);
    for (int i = 0; i < n; ++i) rx.Feed_Chip(rxI[i], rxQ[i]);
    for (int i = 0; i < kGuard; ++i) rx.Feed_Chip(0, 0);

    const bool crc_ok = (g_last.success_mask == DecodedPacket::DECODE_MASK_OK);
    const bool len_ok = (g_last.data_len == 8);
    if (crc_ok && len_ok) {
        int bit_err = 0;
        for (int b = 0; b < 8; ++b) {
            std::uint8_t d = static_cast<std::uint8_t>(
                static_cast<std::uint8_t>(g_last.data[b]) ^ expected[b]);
            if (d != 0u) {
                d = static_cast<std::uint8_t>(d - ((d >> 1) & 0x55u));
                d = static_cast<std::uint8_t>((d & 0x33u) + ((d >> 2) & 0x33u));
                d = static_cast<std::uint8_t>((d + (d >> 4)) & 0x0Fu);
                bit_err += static_cast<int>(d);
            }
        }
        m.bit_errors = bit_err;
        m.pass = (bit_err == 0);
    }
    return m;
}

static void ch_rotate(int16_t* I, int16_t* Q, int n, int deg) noexcept {
    const double rad = deg * kPi / 180.0;
    const double c = std::cos(rad), s = std::sin(rad);
    for (int i = 0; i < n; ++i) {
        const double di = I[i], dq = Q[i];
        I[i] = sat16(di * c - dq * s);
        Q[i] = sat16(di * s + dq * c);
    }
}

static void ch_awgn(int16_t* I, int16_t* Q, int n, double snr_db, std::mt19937& rng) noexcept {
    double ps = 0;
    for (int i = 0; i < n; ++i)
        ps += static_cast<double>(I[i]) * I[i] + static_cast<double>(Q[i]) * Q[i];
    ps /= (n > 0) ? static_cast<double>(n) : 1.0;
    const double sigma = std::sqrt(ps / (2.0 * std::pow(10.0, snr_db / 10.0)));
    std::normal_distribution<double> nd(0.0, sigma);
    for (int i = 0; i < n; ++i) {
        I[i] = sat16(I[i] + nd(rng));
        Q[i] = sat16(Q[i] + nd(rng));
    }
}

static void ch_cfo(int16_t* I, int16_t* Q, int n, double cfo_hz) noexcept {
    for (int i = 0; i < n; ++i) {
        const double ph = 2.0 * kPi * cfo_hz * static_cast<double>(i) / kChipRate;
        const double c = std::cos(ph), s = std::sin(ph);
        const double di = I[i], dq = Q[i];
        I[i] = sat16(di * c - dq * s);
        Q[i] = sat16(di * s + dq * c);
    }
}

static void ch_multipath_3tap(int16_t* I, int16_t* Q, int n, int d1, double a1, int d2,
                              double a2) noexcept {
    int dmax = (d1 > d2) ? d1 : d2;
    if (dmax >= n) return;
    for (int i = n - 1; i >= dmax; --i) {
        double vi = I[i], vq = Q[i];
        if (i >= d1) {
            vi += a1 * I[i - d1];
            vq += a1 * Q[i - d1];
        }
        if (i >= d2) {
            vi += a2 * I[i - d2];
            vq += a2 * Q[i - d2];
        }
        I[i] = sat16(vi);
        Q[i] = sat16(vq);
    }
}

static void ch_barrage(int16_t* I, int16_t* Q, int n, double jsr_db, std::mt19937& rng) noexcept {
    const double sigma =
        static_cast<double>(kAmp) * std::sqrt(std::pow(10.0, jsr_db / 10.0));
    std::normal_distribution<double> nd(0.0, sigma);
    for (int i = 0; i < n; ++i) {
        I[i] = sat16(I[i] + nd(rng));
        Q[i] = sat16(Q[i] + nd(rng));
    }
}

static void ch_cw(int16_t* I, int16_t* Q, int n, double jsr_db, double period_chips) noexcept {
    const double a = static_cast<double>(kAmp) * std::sqrt(std::pow(10.0, jsr_db / 10.0));
    const double ph_scale = 2.0 * kPi / std::max(1e-6, period_chips);
    for (int i = 0; i < n; ++i) {
        const double ph = ph_scale * static_cast<double>(i);
        I[i] = sat16(I[i] + a * std::cos(ph));
        Q[i] = sat16(Q[i] + a * std::sin(ph));
    }
}

[[nodiscard]] TrialParams sample_waterfall_trial(const WaterfallSpec& w, double sweep_value_db,
                                                 std::uint32_t trial_seed,
                                                 std::uint32_t wf_tag) noexcept {
    std::uint32_t s = trial_seed ^ wf_tag;
    s = splitmix32(s);
    std::mt19937 rng(s);

    TrialParams p{};
    p.tx_ds = splitmix32(s + 1u);
    p.barrage_rng_seed = splitmix32(s + 2u);
    p.awgn_rng_seed = splitmix32(s + 3u);

    std::normal_distribution<double> cfo_n(0.0, w.fixed_cfo_hz_std);
    p.cfo_hz = cfo_n(rng);
    p.timing_extra = 0;
    p.period_chips = 8.0;
    p.jsr_barr_db = p.jsr_cw_db = 0.0;
    p.snr_db = 0.0;

    switch (w.kind) {
    case MonteCarloHarness::McScenarioKind::Clean: {
        p.snr_db = sweep_value_db;
        p.jsr_barr_db = p.jsr_cw_db = 0.0;
        p.period_chips = 8.0;
        break;
    }
    case MonteCarloHarness::McScenarioKind::LowSnr: {
        p.snr_db = sweep_value_db;
        p.jsr_barr_db = p.jsr_cw_db = 0.0;
        p.period_chips = 8.0;
        break;
    }
    case MonteCarloHarness::McScenarioKind::Barrage: {
        p.jsr_barr_db = sweep_value_db;
        p.snr_db = w.fixed_snr_db;
        p.jsr_cw_db = 0.0;
        p.period_chips = 8.0;
        break;
    }
    case MonteCarloHarness::McScenarioKind::CwFreqSweep: {
        p.jsr_cw_db = sweep_value_db;
        std::uniform_real_distribution<double> fu(w.fixed_cw_freq_min, w.fixed_cw_freq_max);
        const double f_hz = std::max(1000.0, fu(rng));
        p.period_chips = std::clamp(kChipRate / f_hz, 2.0, 512.0);
        p.jsr_barr_db = 0.0;
        p.snr_db = w.fixed_snr_db;
        break;
    }
    case MonteCarloHarness::McScenarioKind::CwPeriodMulti: {
        p.jsr_cw_db = sweep_value_db;
        std::uniform_real_distribution<double> pu(w.fixed_period_min, w.fixed_period_max);
        p.period_chips = pu(rng);
        p.jsr_barr_db = 0.0;
        p.snr_db = w.fixed_snr_db;
        break;
    }
    case MonteCarloHarness::McScenarioKind::CombinedStress: {
        p.jsr_barr_db = sweep_value_db;
        p.jsr_cw_db = sweep_value_db;
        p.snr_db = w.fixed_snr_db;
        std::uniform_real_distribution<double> fu(w.fixed_cw_freq_min, w.fixed_cw_freq_max);
        const double f_hz = std::max(1000.0, fu(rng));
        p.period_chips = std::clamp(kChipRate / f_hz, 2.0, 512.0);
        std::uniform_int_distribution<int> tu(0, w.fixed_timing_jitter);
        p.timing_extra = tu(rng);
        break;
    }
    }
    return p;
}

[[nodiscard]] TrialResult run_single_trial(const WaterfallSpec& w, const TrialParams& tp,
                                           int trial_index) noexcept {
#if defined(HTS_LLR_DIAG)
    ProtectedEngine::LLRDiag::reset_llr_stats();
#endif
#if defined(HTS_ROW_CONSISTENCY_DIAG)
    ProtectedEngine::RowConsistencyDiag::reset_stats();
#endif

    TrialResult r{};
    TxPkt tx = build_tx(tp.tx_ds, trial_index);
    if (tx.n <= 0) {
        return r;
    }

    int16_t rI[kMaxC]{};
    int16_t rQ[kMaxC]{};
    std::memcpy(rI, tx.I, sizeof(int16_t) * static_cast<std::size_t>(tx.n));
    std::memcpy(rQ, tx.Q, sizeof(int16_t) * static_cast<std::size_t>(tx.n));

    if (w.fixed_rotate_deg != 0) {
        ch_rotate(rI, rQ, tx.n, w.fixed_rotate_deg);
    }
    ch_cfo(rI, rQ, tx.n, tp.cfo_hz);

    if (w.has_multipath) {
        ch_multipath_3tap(rI, rQ, tx.n, 3, 0.2, 10, 0.1);
    }

    if (w.kind == MonteCarloHarness::McScenarioKind::Barrage ||
        w.kind == MonteCarloHarness::McScenarioKind::CombinedStress) {
        std::mt19937 rng_b(tp.barrage_rng_seed);
        ch_barrage(rI, rQ, tx.n, tp.jsr_barr_db, rng_b);
    }
    if (w.kind == MonteCarloHarness::McScenarioKind::CwFreqSweep ||
        w.kind == MonteCarloHarness::McScenarioKind::CwPeriodMulti ||
        w.kind == MonteCarloHarness::McScenarioKind::CombinedStress) {
        ch_cw(rI, rQ, tx.n, tp.jsr_cw_db, tp.period_chips);
    }

    if (w.apply_awgn) {
        std::mt19937 rng_n(tp.awgn_rng_seed);
        ch_awgn(rI, rQ, tx.n, tp.snr_db, rng_n);
    }

    const int pre_guard = kGuard + tp.timing_extra;
    const TrialMetrics m = feed_raw_ext(tp.tx_ds, rI, rQ, tx.n, tx.info, pre_guard);
    r.pass = m.pass;
    r.bit_errors = m.bit_errors;
    r.total_bits = 64;

#if defined(HTS_LLR_DIAG)
    r.llr_reliability_bin = ProtectedEngine::LLRDiag::compute_reliability(
        ProtectedEngine::LLRDiag::ObservePoint::BIN_TO_LLR);
#endif
#if defined(HTS_ROW_CONSISTENCY_DIAG)
    r.row_entropy = ProtectedEngine::RowConsistencyDiag::compute_entropy();
    r.row_concentration = ProtectedEngine::RowConsistencyDiag::compute_concentration();
#endif
    return r;
}

[[nodiscard]] double mean_of(const std::vector<double>& v) noexcept {
    if (v.empty()) return 0.0;
    double s = 0.0;
    for (double x : v) s += x;
    return s / static_cast<double>(v.size());
}

[[nodiscard]] double stddev_of(const std::vector<double>& v) noexcept {
    if (v.size() < 2u) return 0.0;
    const double m = mean_of(v);
    double acc = 0.0;
    for (double x : v) {
        const double d = x - m;
        acc += d * d;
    }
    return std::sqrt(acc / static_cast<double>(v.size() - 1u));
}

[[nodiscard]] SweepPointAggregate run_sweep_point(const WaterfallSpec& spec, double sweep_value,
                                                  std::uint32_t base_seed, std::uint32_t scenario_tag,
                                                  std::FILE* meta_csv) noexcept {
    SweepPointAggregate agg{};
    agg.scenario_name = spec.scenario_name;
    agg.axis = spec.axis;
    agg.sweep_value_db = sweep_value;
    agg.grand_N = kMetaLoops * kInnerTrials;

    const std::uint32_t sweep_tag =
        static_cast<std::uint32_t>(std::lround(sweep_value * 100.0)) * 131u;

    std::vector<double> meta_rates;
    meta_rates.reserve(static_cast<std::size_t>(kMetaLoops));

    double llr_grand_sum = 0.0;
    double H_grand_sum = 0.0;
    double conc_grand_sum = 0.0;
    int llr_diag_trials = 0;
    int H_diag_trials = 0;
    int conc_diag_trials = 0;

    for (int meta = 0; meta < kMetaLoops; ++meta) {
        const std::uint32_t outer_seed =
            base_seed ^ (static_cast<std::uint32_t>(meta) * kMetaSeedPrime) ^ scenario_tag ^
            sweep_tag ^ hash_cstr(spec.scenario_name);

        MetaResult mr{};
        mr.meta_idx = meta;

        double llr_sum_meta = 0.0;
        double H_sum_meta = 0.0;
        double conc_sum_meta = 0.0;
        int m_llr = 0;
        int m_h = 0;
        int m_c = 0;

        for (int trial = 0; trial < kInnerTrials; ++trial) {
            const std::uint32_t trial_seed = outer_seed ^ (static_cast<std::uint32_t>(trial) * 0x85EBCA77u);
            const TrialParams tp =
                sample_waterfall_trial(spec, sweep_value, trial_seed, scenario_tag);
            const TrialResult tr = run_single_trial(spec, tp, trial);

            if (tr.pass) {
                ++mr.pass_count;
            }
            agg.grand_pass += tr.pass ? 1 : 0;

            if (std::isfinite(tr.llr_reliability_bin)) {
                llr_sum_meta += tr.llr_reliability_bin;
                llr_grand_sum += tr.llr_reliability_bin;
                ++m_llr;
                ++llr_diag_trials;
            }
            if (std::isfinite(tr.row_entropy) && tr.row_entropy > 0.0) {
                H_sum_meta += tr.row_entropy;
                H_grand_sum += tr.row_entropy;
                ++m_h;
                ++H_diag_trials;
            }
            if (std::isfinite(tr.row_concentration) && tr.row_concentration > 0.0) {
                conc_sum_meta += tr.row_concentration;
                conc_grand_sum += tr.row_concentration;
                ++m_c;
                ++conc_diag_trials;
            }
        }

        mr.pass_rate = static_cast<double>(mr.pass_count) / static_cast<double>(kInnerTrials);
        if (m_llr > 0) {
            mr.llr_bin_mean = llr_sum_meta / static_cast<double>(m_llr);
        }
        if (m_h > 0) {
            mr.row_H_mean = H_sum_meta / static_cast<double>(m_h);
        }
        if (m_c > 0) {
            mr.row_conc_mean = conc_sum_meta / static_cast<double>(m_c);
        }
        meta_rates.push_back(mr.pass_rate);

        if (meta_csv != nullptr) {
            std::fprintf(meta_csv,
                         "%s,%s,%.2f,%d,%d,%d,%.6f,%.6f,%.6f,%.6f\n", spec.scenario_name,
                         (spec.axis == SweepAxis::SNR_dB) ? "SNR_dB" : "JSR_dB", sweep_value, meta,
                         mr.pass_count, kInnerTrials, mr.pass_rate, mr.llr_bin_mean, mr.row_H_mean,
                         mr.row_conc_mean);
        }
    }

    agg.grand_pass_rate = (agg.grand_N > 0)
        ? static_cast<double>(agg.grand_pass) / static_cast<double>(agg.grand_N)
        : 0.0;

    const auto ci = HTS_Phase2::clopper_pearson_ci(static_cast<std::int64_t>(agg.grand_N),
                                                    static_cast<std::int64_t>(agg.grand_pass), 0.05);
    agg.grand_ci_lo = ci.first;
    agg.grand_ci_hi = ci.second;

    agg.meta_mean_rate = mean_of(meta_rates);
    agg.meta_std_rate = stddev_of(meta_rates);
    agg.meta_min_rate =
        meta_rates.empty() ? 0.0 : *std::min_element(meta_rates.begin(), meta_rates.end());
    agg.meta_max_rate =
        meta_rates.empty() ? 0.0 : *std::max_element(meta_rates.begin(), meta_rates.end());

    if (llr_diag_trials > 0) {
        agg.llr_bin_grand_mean = llr_grand_sum / static_cast<double>(llr_diag_trials);
    }
    if (H_diag_trials > 0) {
        agg.row_H_grand_mean = H_grand_sum / static_cast<double>(H_diag_trials);
    }
    if (conc_diag_trials > 0) {
        agg.row_conc_grand_mean = conc_grand_sum / static_cast<double>(conc_diag_trials);
    }
    return agg;
}

void run_waterfall_double(std::uint32_t base_seed, std::FILE* meta_csv,
                          std::FILE* results_csv) noexcept {
    if (results_csv != nullptr) {
        std::fprintf(results_csv,
                     "Scenario,SweepAxis,SweepValue_dB,GrandN,GrandPass,GrandPassRate,GrandCI_lo,"
                     "GrandCI_hi,MetaMeanRate,MetaStdRate,MetaMinRate,MetaMaxRate,LLR_BIN_grand,"
                     "Row_H_grand,Row_conc_grand\n");
    }

    int total_conditions = 0;
    for (std::size_t s = 0; s < kWaterfallSpecCount; ++s) {
        const WaterfallSpec& w = kWaterfallSpecs[s];
        const int n_steps =
            static_cast<int>((w.sweep_end_db - w.sweep_start_db) / w.sweep_step_db + 1e-9);
        total_conditions += n_steps + 1;
    }

    std::printf("=== HTS Waterfall Matrix (Double Statistics) ===\n");
    std::printf("Meta loops: %d, inner trials: %d\n", kMetaLoops, kInnerTrials);
    std::printf("Per point: %d trials, sweep point count: %d\n", kTotalPerPoint,
                total_conditions);
    std::printf("Grand total trials: %d\n\n", total_conditions * kTotalPerPoint);

    if (meta_csv != nullptr) {
        std::fprintf(meta_csv,
                     "Scenario,Axis,Value_dB,MetaIdx,Pass,N,Rate,LLR_mean,H_mean,conc_mean\n");
    }

    int cond_done = 0;
    for (std::size_t si = 0; si < kWaterfallSpecCount; ++si) {
        const WaterfallSpec& spec = kWaterfallSpecs[si];
        const char* axis_name = (spec.axis == SweepAxis::SNR_dB) ? "SNR_dB" : "JSR_dB";
        const std::uint32_t scenario_tag =
            splitmix32(static_cast<std::uint32_t>(si) * 0x1A2B3C4Du);

        std::printf("\n=== %s (%s sweep %.1f..%.1f dB, step %.1f) ===\n", spec.scenario_name,
                    axis_name, spec.sweep_start_db, spec.sweep_end_db, spec.sweep_step_db);

        const int n_steps =
            static_cast<int>((spec.sweep_end_db - spec.sweep_start_db) / spec.sweep_step_db + 1e-9);
        for (int step_i = 0; step_i <= n_steps; ++step_i) {
            const double val = spec.sweep_start_db +
                               static_cast<double>(step_i) * spec.sweep_step_db;
            const auto t0 = std::chrono::steady_clock::now();
            const SweepPointAggregate agg =
                run_sweep_point(spec, val, base_seed, scenario_tag, meta_csv);
            const auto t1 = std::chrono::steady_clock::now();
            const double elapsed_sec = std::chrono::duration<double>(t1 - t0).count();

            ++cond_done;
            std::printf(
                "  [%d/%d] %s=%+6.1f  grand %d/%d (%.4f)  meta[mean=%.4f std=%.4f min=%.4f "
                "max=%.4f]  CI[%.4f,%.4f]  (%.2fs)\n",
                cond_done, total_conditions, axis_name, val, agg.grand_pass, agg.grand_N,
                agg.grand_pass_rate, agg.meta_mean_rate, agg.meta_std_rate, agg.meta_min_rate,
                agg.meta_max_rate, agg.grand_ci_lo, agg.grand_ci_hi, elapsed_sec);

            if (results_csv != nullptr) {
                std::fprintf(results_csv,
                             "%s,%s,%.2f,%d,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                             agg.scenario_name, axis_name, agg.sweep_value_db, agg.grand_N,
                             agg.grand_pass, agg.grand_pass_rate, agg.grand_ci_lo, agg.grand_ci_hi,
                             agg.meta_mean_rate, agg.meta_std_rate, agg.meta_min_rate,
                             agg.meta_max_rate, agg.llr_bin_grand_mean, agg.row_H_grand_mean,
                             agg.row_conc_grand_mean);
                std::fflush(results_csv);
            }
        }
    }
}

} // namespace WaterfallHarness

int main(int argc, char** argv) {
    ProtectedEngine::WRC::reset_diag();

    std::uint32_t seed = 0xABCDEF01u;
    if (argc >= 2) {
        seed = static_cast<std::uint32_t>(std::strtoul(argv[1], nullptr, 0));
    }

    std::FILE* meta_csv = std::fopen("waterfall_per_meta.csv", "w");
    std::FILE* results_csv = std::fopen("waterfall_results.csv", "w");

    const auto t0 = std::chrono::steady_clock::now();
    WaterfallHarness::run_waterfall_double(seed, meta_csv, results_csv);
    const auto t1 = std::chrono::steady_clock::now();

    if (meta_csv != nullptr) {
        std::fclose(meta_csv);
    }
    if (results_csv != nullptr) {
        std::fclose(results_csv);
    }

    const double total_hours = std::chrono::duration<double>(t1 - t0).count() / 3600.0;
    std::printf("\n=== COMPLETE ===\n");
    std::printf("Total time: %.3f h (%.1f s)\n", total_hours,
                std::chrono::duration<double>(t1 - t0).count());
    std::printf("Artifacts: waterfall_stdout.log (stdout), waterfall_results.csv, "
                "waterfall_per_meta.csv\n");

    ProtectedEngine::WRC::print_diag();
    return 0;
}

#include "../../HTS_LIM/HTS_Secure_Memory.cpp"
#include "../../HTS_LIM/HTS_Polar_Codec.cpp"
#include "../../HTS_LIM/HTS_FEC_HARQ.cpp"
#include "../../HTS_LIM/HTS_Holo_LPI.cpp"
#include "../../HTS_LIM/HTS_Walsh_Row_Permuter.cpp"
#include "../../HTS_LIM/HTS_V400_Dispatcher.cpp"
