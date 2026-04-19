#include "HTS_Amp_Diag.hpp"
#include <cstddef>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace ProtectedEngine {
namespace AmpDiag {

static AmpStats g_stats{};

AmpStats &get_amp_stats() noexcept {
    return g_stats;
}

void reset_amp_stats() noexcept {
    std::memset(static_cast<void *>(&g_stats), 0, sizeof(g_stats));
}

static int mag_to_bin(uint32_t a) noexcept {
    if (a == 0u) {
        return 0;
    }
    if (a < 1024u) {
        return 1;
    }
    if (a < 4096u) {
        return 2;
    }
    if (a < 8192u) {
        return 3;
    }
    if (a < 16384u) {
        return 4;
    }
    if (a < 24576u) {
        return 5;
    }
    if (a < 32767u) {
        return 6;
    }
    return 7;
}

void record_chip(int16_t I, int16_t Q) noexcept {
    const int64_t iI = static_cast<int64_t>(I);
    const int64_t iQ = static_cast<int64_t>(Q);
    const uint32_t absI = static_cast<uint32_t>(iI >= 0 ? iI : -iI);
    const uint32_t absQ = static_cast<uint32_t>(iQ >= 0 ? iQ : -iQ);

    ++g_stats.hist_I[static_cast<std::size_t>(mag_to_bin(absI))];
    ++g_stats.hist_Q[static_cast<std::size_t>(mag_to_bin(absQ))];
    ++g_stats.total_samples;

    if (absI >= 32767u || absQ >= 32767u) {
        ++g_stats.saturated_count;
    }
    if (absI >= 30000u || absQ >= 30000u) {
        ++g_stats.near_sat_count;
    }

    g_stats.sum_sq_I += static_cast<uint64_t>(absI) * absI;
    g_stats.sum_sq_Q += static_cast<uint64_t>(absQ) * absQ;

    const int32_t ai = static_cast<int32_t>(absI);
    const int32_t aq = static_cast<int32_t>(absQ);
    if (ai > g_stats.max_abs_I) {
        g_stats.max_abs_I = ai;
    }
    if (aq > g_stats.max_abs_Q) {
        g_stats.max_abs_Q = aq;
    }
}

void print_amp_stats() noexcept {
    const AmpStats &s = g_stats;
    static const char *const bin_labels[AmpStats::NUM_BINS] = {
        "       0    ", "    1- 1023 ", " 1024- 4095 ", " 4096- 8191 ",
        " 8192-16383 ", "16384-24575 ", "24576-32766 ", "32767 (SAT) "};

    std::fprintf(stderr,
                 "\n=== Chip Amplitude Distribution ===\n"
                 "  (Feed_Chip 진입, DC/AGC 이전 raw rx_I/rx_Q)\n"
                 "  total_samples    = %llu\n"
                 "  saturated        = %llu  (%.4f%%)\n"
                 "  near saturation  = %llu  (%.4f%%)\n"
                 "  max |I|          = %d\n"
                 "  max |Q|          = %d\n",
                 static_cast<unsigned long long>(s.total_samples),
                 static_cast<unsigned long long>(s.saturated_count),
                 (s.total_samples > 0u)
                     ? 100.0 * static_cast<double>(s.saturated_count) /
                           static_cast<double>(s.total_samples)
                     : 0.0,
                 static_cast<unsigned long long>(s.near_sat_count),
                 (s.total_samples > 0u)
                     ? 100.0 * static_cast<double>(s.near_sat_count) /
                           static_cast<double>(s.total_samples)
                     : 0.0,
                 static_cast<int>(s.max_abs_I), static_cast<int>(s.max_abs_Q));

    if (s.total_samples > 0u) {
        const double mean_sq_I =
            static_cast<double>(s.sum_sq_I) / static_cast<double>(s.total_samples);
        const double mean_sq_Q =
            static_cast<double>(s.sum_sq_Q) / static_cast<double>(s.total_samples);
        const double rms_I = std::sqrt(mean_sq_I);
        const double rms_Q = std::sqrt(mean_sq_Q);
        const double par_I =
            (rms_I > 0.0) ? static_cast<double>(s.max_abs_I) / rms_I : 0.0;
        const double par_Q =
            (rms_Q > 0.0) ? static_cast<double>(s.max_abs_Q) / rms_Q : 0.0;
        std::fprintf(stderr,
                     "  RMS |I|          = %.2f\n"
                     "  RMS |Q|          = %.2f\n"
                     "  PAR (max/rms) I  = %.2f\n"
                     "  PAR (max/rms) Q  = %.2f\n",
                     rms_I, rms_Q, par_I, par_Q);
    }

    std::fprintf(stderr, "\n--- Histogram (|I|, |Q|) ---\n");
    std::fprintf(stderr, "  %-14s %15s %15s\n", "Range |x|", "Count I", "Count Q");
    for (int i = 0; i < AmpStats::NUM_BINS; ++i) {
        const double pct_I =
            (s.total_samples > 0u)
                ? 100.0 * static_cast<double>(s.hist_I[static_cast<std::size_t>(i)]) /
                      static_cast<double>(s.total_samples)
                : 0.0;
        const double pct_Q =
            (s.total_samples > 0u)
                ? 100.0 * static_cast<double>(s.hist_Q[static_cast<std::size_t>(i)]) /
                      static_cast<double>(s.total_samples)
                : 0.0;
        std::fprintf(stderr, "  %s %10llu(%5.2f%%) %10llu(%5.2f%%)\n",
                     bin_labels[static_cast<std::size_t>(i)],
                     static_cast<unsigned long long>(
                         s.hist_I[static_cast<std::size_t>(i)]),
                     pct_I,
                     static_cast<unsigned long long>(
                         s.hist_Q[static_cast<std::size_t>(i)]),
                     pct_Q);
    }
    std::fprintf(stderr, "===================================\n");
}

} // namespace AmpDiag
} // namespace ProtectedEngine
