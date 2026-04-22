// =============================================================================
// HTS_T6_MC_Harness.cpp — Monte Carlo PC harness (separate from T6 regression)
// Per-trial randomized channel realizations; Clopper–Pearson CI on pass rate.
// Build: see build_mc_harness.bat
// =============================================================================
#if defined(__arm__) || defined(__TARGET_ARCH_ARM)
#error "[HTS_FATAL] PC 전용"
#endif

#include "HTS_V400_Dispatcher.hpp"
#include "HTS_Walsh_Row_Converter.hpp"
#include "../t6_sim/HTS_BER_PER_Measure.hpp"
#include "MC_Scenario_Specs.hpp"
#if defined(HTS_LLR_DIAG)
#include "HTS_LLR_Diag.hpp"
#endif
#if defined(HTS_ROW_CONSISTENCY_DIAG)
#include "HTS_Row_Consistency_Diag.hpp"
#endif
#if defined(HTS_CW_EX_DIAG)
#include "HTS_CW_Excision.hpp"
#endif
#if defined(HTS_CW_W_DIAG) && defined(HTS_CW_LLR_WEIGHT_V1)
#include "HTS_CW_LLR_Weight.hpp"
#endif

#include <algorithm>
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

namespace MonteCarloHarness {

namespace {

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

} // namespace

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

[[nodiscard]] TrialParams sample_realization(const ScenarioSpec& spec, std::uint32_t scenario_tag,
                                             int trial_idx, std::uint32_t base_seed) noexcept {
    std::uint32_t s = base_seed ^ scenario_tag ^
                      static_cast<std::uint32_t>(static_cast<std::uint32_t>(trial_idx) * 0x9E3779B9u);
    s = splitmix32(s);
    std::mt19937 rng(s);

    TrialParams p{};
    p.tx_ds = splitmix32(s + 1u);
    p.barrage_rng_seed = splitmix32(s + 2u);
    p.awgn_rng_seed = splitmix32(s + 3u);

    std::normal_distribution<double> cfo_n(spec.cfo_hz_mean, spec.cfo_hz_std);
    p.cfo_hz = cfo_n(rng);

    switch (spec.kind) {
    case McScenarioKind::Clean: {
        std::uniform_real_distribution<double> snu(spec.snr_db_min, spec.snr_db_max);
        p.snr_db = snu(rng);
        p.jsr_barr_db = p.jsr_cw_db = 0.0;
        p.period_chips = 8.0;
        p.timing_extra = 0;
        break;
    }
    case McScenarioKind::LowSnr: {
        std::uniform_real_distribution<double> snu(spec.snr_db_min, spec.snr_db_max);
        p.snr_db = snu(rng);
        p.jsr_barr_db = p.jsr_cw_db = 0.0;
        p.period_chips = 8.0;
        p.timing_extra = 0;
        break;
    }
    case McScenarioKind::Barrage: {
        std::uniform_real_distribution<double> ju(spec.jsr_barr_min, spec.jsr_barr_max);
        p.jsr_barr_db = ju(rng);
        p.snr_db = 0.0;
        p.jsr_cw_db = 0.0;
        p.period_chips = 8.0;
        p.timing_extra = 0;
        break;
    }
    case McScenarioKind::CwFreqSweep: {
        std::uniform_real_distribution<double> ju(spec.jsr_cw_min, spec.jsr_cw_max);
        p.jsr_cw_db = ju(rng);
        std::uniform_real_distribution<double> fu(spec.cw_freq_hz_min, spec.cw_freq_hz_max);
        const double f_hz = std::max(1000.0, fu(rng));
        p.period_chips = std::clamp(kChipRate / f_hz, 2.0, 512.0);
        p.jsr_barr_db = 0.0;
        p.snr_db = 0.0;
        p.timing_extra = 0;
        break;
    }
    case McScenarioKind::CwPeriodMulti: {
        std::uniform_real_distribution<double> ju(spec.jsr_cw_min, spec.jsr_cw_max);
        p.jsr_cw_db = ju(rng);
        std::uniform_real_distribution<double> pu(spec.period_chips_min, spec.period_chips_max);
        p.period_chips = pu(rng);
        p.jsr_barr_db = 0.0;
        p.snr_db = 0.0;
        p.timing_extra = 0;
        break;
    }
    case McScenarioKind::CombinedStress: {
        std::uniform_real_distribution<double> sb(spec.jsr_barr_min, spec.jsr_barr_max);
        std::uniform_real_distribution<double> sc(spec.jsr_cw_min, spec.jsr_cw_max);
        p.jsr_barr_db = sb(rng);
        p.jsr_cw_db = sc(rng);
        std::uniform_real_distribution<double> fu(spec.cw_freq_hz_min, spec.cw_freq_hz_max);
        const double f_hz = std::max(1000.0, fu(rng));
        p.period_chips = std::clamp(kChipRate / f_hz, 2.0, 512.0);
        std::uniform_real_distribution<double> snu(spec.snr_db_min, spec.snr_db_max);
        p.snr_db = snu(rng);
        std::uniform_int_distribution<int> tu(0, spec.timing_jitter_max);
        p.timing_extra = tu(rng);
        break;
    }
    }
    return p;
}

struct TrialResult {
    bool pass{};
    int bit_errors{64};
    int total_bits{64};
    double llr_reliability_bin{0.0};
    double llr_reliability_ir{0.0};
    double row_entropy{0.0};
    double row_concentration{0.0};
};

[[nodiscard]] static TrialResult run_trial(const ScenarioSpec& spec, const TrialParams& tp,
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

    if (spec.rotate_deg != 0) ch_rotate(rI, rQ, tx.n, spec.rotate_deg);
    ch_cfo(rI, rQ, tx.n, tp.cfo_hz);

    if (spec.has_multipath) {
        ch_multipath_3tap(rI, rQ, tx.n, 3, 0.2, 10, 0.1);
    }

    if (spec.kind == McScenarioKind::Barrage || spec.kind == McScenarioKind::CombinedStress) {
        std::mt19937 rng_b(tp.barrage_rng_seed);
        ch_barrage(rI, rQ, tx.n, tp.jsr_barr_db, rng_b);
    }
    if (spec.kind == McScenarioKind::CwFreqSweep || spec.kind == McScenarioKind::CwPeriodMulti ||
        spec.kind == McScenarioKind::CombinedStress) {
        ch_cw(rI, rQ, tx.n, tp.jsr_cw_db, tp.period_chips);
    }

    if (spec.apply_awgn) {
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
    r.llr_reliability_ir = ProtectedEngine::LLRDiag::compute_reliability(
        ProtectedEngine::LLRDiag::ObservePoint::IR_ACCUM);
#endif
#if defined(HTS_ROW_CONSISTENCY_DIAG)
    r.row_entropy = ProtectedEngine::RowConsistencyDiag::compute_entropy();
    r.row_concentration = ProtectedEngine::RowConsistencyDiag::compute_concentration();
#endif
    return r;
}

struct ScenarioResult {
    int N{};
    int pass_count{};
    double pass_rate{};
    double ci_low{};
    double ci_high{};
    double bit_error_rate{};
    long long bit_errors_total{};
    long long total_bits{};
    double llr_bin_mean{};
    double llr_bin_std{};
    double llr_ir_mean{};
    double llr_ir_std{};
    double entropy_mean{};
    double entropy_std{};
    double concentration_mean{};
    double concentration_std{};
};

[[nodiscard]] ScenarioResult run_scenario(const ScenarioSpec& spec, int N, std::uint32_t base_seed,
                                          int scenario_index) {
    ScenarioResult out{};
    out.N = N;
    std::vector<double> llr_bins;
    std::vector<double> llr_irs;
    std::vector<double> entropies;
    std::vector<double> concentrations;

    const std::uint32_t scen_tag =
        splitmix32(static_cast<std::uint32_t>(scenario_index) * 0x1A2B3C4Du);

    for (int i = 0; i < N; ++i) {
        const TrialParams p = sample_realization(spec, scen_tag, i, base_seed);
        const TrialResult tr = run_trial(spec, p, i);
        if (tr.pass) ++out.pass_count;
        out.bit_errors_total += tr.bit_errors;
        out.total_bits += tr.total_bits;
        if (std::isfinite(tr.llr_reliability_bin)) llr_bins.push_back(tr.llr_reliability_bin);
        if (std::isfinite(tr.llr_reliability_ir)) llr_irs.push_back(tr.llr_reliability_ir);
        if (std::isfinite(tr.row_entropy) && tr.row_entropy > 0.0)
            entropies.push_back(tr.row_entropy);
        if (std::isfinite(tr.row_concentration) && tr.row_concentration > 0.0)
            concentrations.push_back(tr.row_concentration);
    }

    out.pass_rate = (N > 0) ? static_cast<double>(out.pass_count) / static_cast<double>(N) : 0.0;
    const auto ci = HTS_Phase2::clopper_pearson_ci(static_cast<std::int64_t>(N),
                                                    static_cast<std::int64_t>(out.pass_count), 0.05);
    out.ci_low = ci.first;
    out.ci_high = ci.second;
    out.bit_error_rate = (out.total_bits > 0)
        ? static_cast<double>(out.bit_errors_total) / static_cast<double>(out.total_bits)
        : 0.0;

    out.llr_bin_mean = mean_of(llr_bins);
    out.llr_bin_std = stddev_of(llr_bins);
    out.llr_ir_mean = mean_of(llr_irs);
    out.llr_ir_std = stddev_of(llr_irs);
    out.entropy_mean = mean_of(entropies);
    out.entropy_std = stddev_of(entropies);
    out.concentration_mean = mean_of(concentrations);
    out.concentration_std = stddev_of(concentrations);
    return out;
}

} // namespace MonteCarloHarness

int main(int argc, char** argv) {
    using namespace MonteCarloHarness;

    ProtectedEngine::WRC::reset_diag();

    int N = 500;
    std::uint32_t base_seed = 0xABCDEF01u;
    if (argc >= 2) {
        N = std::max(1, std::atoi(argv[1]));
    }
    if (argc >= 3) {
        base_seed = static_cast<std::uint32_t>(std::strtoul(argv[2], nullptr, 0));
    }

    std::printf("=================================================================\n");
    std::printf(" HTS T6 Monte Carlo harness (separate from T6 regression)\n");
    std::printf(" N = %d trials per scenario | base_seed = 0x%08X\n", N,
                static_cast<unsigned>(base_seed));
    std::printf("=================================================================\n\n");

    int si = 0;
    for (const ScenarioSpec& spec : kMcSpecs) {
        const ScenarioResult r = run_scenario(spec, N, base_seed, si);
        std::printf("=== %s ===\n", spec.name);
        std::printf("  %s\n", spec.description);
        std::printf("  Pass: %d / %d  (rate %.6f)\n", r.pass_count, r.N, r.pass_rate);
        std::printf("  95%% Clopper-Pearson CI: [%.6f, %.6f]\n", r.ci_low, r.ci_high);
        std::printf("  BER (bits): %.6f\n", r.bit_error_rate);
        std::printf("  LLR_BIN  mean=%.6f std=%.6f\n", r.llr_bin_mean, r.llr_bin_std);
        std::printf("  LLR_IR   mean=%.6f std=%.6f\n", r.llr_ir_mean, r.llr_ir_std);
        std::printf("  Row H    mean=%.6f std=%.6f\n", r.entropy_mean, r.entropy_std);
        std::printf("  Row conc mean=%.6f std=%.6f\n", r.concentration_mean, r.concentration_std);
        std::printf("\n");
        ++si;
    }

    ProtectedEngine::WRC::print_diag();
#if defined(HTS_CW_EX_DIAG)
    ProtectedEngine::CWExcision::print_stats("MC_ALL");
#endif
#if defined(HTS_CW_W_DIAG) && defined(HTS_CW_LLR_WEIGHT_V1)
    ProtectedEngine::CWLLRWeight::print_stats("MC_ALL");
#endif
    return 0;
}

// ── Single-TU link (same core as T6) ──
#include "../../HTS_LIM/HTS_Secure_Memory.cpp"
#include "../../HTS_LIM/HTS_Polar_Codec.cpp"
#include "../../HTS_LIM/HTS_FEC_HARQ.cpp"
#include "../../HTS_LIM/HTS_Holo_LPI.cpp"
#include "../../HTS_LIM/HTS_Walsh_Row_Permuter.cpp"
#include "../../HTS_LIM/HTS_V400_Dispatcher_Core.cpp"
#include "../../HTS_LIM/HTS_V400_Dispatcher_Sync_PSLTE.cpp"
#include "../../HTS_LIM/HTS_V400_Dispatcher_Payload.cpp"
#include "../../HTS_LIM/HTS_V400_Dispatcher_TX.cpp"
#include "../../HTS_LIM/HTS_V400_Dispatcher_Decode.cpp"
