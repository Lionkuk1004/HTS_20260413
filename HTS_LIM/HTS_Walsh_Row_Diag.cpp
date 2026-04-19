// =============================================================================
// HTS_Walsh_Row_Diag.cpp — FWHT row별 에너지 분포 계측
// =============================================================================
#include "HTS_Walsh_Row_Diag.hpp"

#include <cstddef>
#include <cstdio>
#include <cstring>

namespace ProtectedEngine {
namespace WalshRowDiag {

static RowStats g_stats = {};

RowStats& get_row_stats() noexcept { return g_stats; }

void reset_row_stats() noexcept {
    std::memset(&g_stats, 0, sizeof(g_stats));
    g_stats.max_ratio = 0.0;
    g_stats.min_ratio = 1.0;
}

void record_row_energies(const int64_t energies[64]) noexcept {
    if (energies == nullptr) {
        return;
    }

    ++g_stats.total_observations;

    int64_t top1 = 0;
    int top1_idx = -1;
    int64_t top2 = 0;
    int top2_idx = -1;

    for (int r = 0; r < 64; ++r) {
        int64_t e = energies[r];
        if (e < 0) {
            e = 0;
        }

        g_stats.row_energy_sum[static_cast<std::size_t>(r)] +=
            static_cast<uint64_t>(e);

        if (e > top1) {
            top2 = top1;
            top2_idx = top1_idx;
            top1 = e;
            top1_idx = r;
        } else if (e > top2) {
            top2 = e;
            top2_idx = r;
        }
    }

    if (top1_idx >= 0 && top1_idx < 64) {
        ++g_stats.row_top1_count[static_cast<std::size_t>(top1_idx)];
    }
    if (top2_idx >= 0 && top2_idx < 64) {
        ++g_stats.row_top2_count[static_cast<std::size_t>(top2_idx)];
    }

    if (top1 > 0) {
        const double ratio =
            static_cast<double>(top2) / static_cast<double>(top1);

        int bin = static_cast<int>(ratio * 10.0);
        if (bin < 0) {
            bin = 0;
        }
        if (bin >= RowStats::RATIO_BINS) {
            bin = RowStats::RATIO_BINS - 1;
        }
        ++g_stats.ratio_hist[static_cast<std::size_t>(bin)];

        if (ratio > g_stats.max_ratio) {
            g_stats.max_ratio = ratio;
        }
        if (ratio < g_stats.min_ratio) {
            g_stats.min_ratio = ratio;
        }
    }
}

void print_row_stats(const char* label) noexcept {
    std::fprintf(stderr, "\n=== Walsh Row Energy [%s] ===\n",
                 label ? label : "ALL");
    std::fprintf(stderr, "  total_observations = %llu\n",
                 static_cast<unsigned long long>(g_stats.total_observations));

    if (g_stats.total_observations == 0) {
        std::fprintf(stderr, "  (no data)\n");
        return;
    }

    std::fprintf(stderr, "\n--- Top Rows by Avg Energy ---\n");
    struct RowAvg {
        int r;
        double avg;
    };
    RowAvg sorted[64];
    for (int r = 0; r < 64; ++r) {
        sorted[r].r = r;
        sorted[r].avg =
            static_cast<double>(g_stats.row_energy_sum[static_cast<std::size_t>(r)]) /
            static_cast<double>(g_stats.total_observations);
    }
    for (int i = 0; i < 64; ++i) {
        for (int j = i + 1; j < 64; ++j) {
            if (sorted[j].avg > sorted[i].avg) {
                const RowAvg tmp = sorted[i];
                sorted[i] = sorted[j];
                sorted[j] = tmp;
            }
        }
    }

    const double top_avg = sorted[0].avg;
    for (int i = 0; i < 10; ++i) {
        const double ratio =
            (top_avg > 0) ? sorted[i].avg / top_avg : 0.0;
        std::fprintf(stderr,
                     "  row=%2d: avg_energy=%.3e  ratio_to_top1=%.4f  "
                     "top1_count=%llu\n",
                     sorted[i].r, sorted[i].avg, ratio,
                     static_cast<unsigned long long>(
                         g_stats.row_top1_count[static_cast<std::size_t>(
                             sorted[i].r)]));
    }

    int nonzero_top1 = 0;
    for (int r = 0; r < 64; ++r) {
        if (g_stats.row_top1_count[static_cast<std::size_t>(r)] > 0) {
            ++nonzero_top1;
        }
    }
    std::fprintf(stderr, "\n  Top1 distinct rows: %d / 64\n", nonzero_top1);

    static const char* const bin_labels[RowStats::RATIO_BINS] = {
        "0.00-0.10 (excellent)", "0.10-0.20 (good)     ",
        "0.20-0.30 (fair)     ", "0.30-0.40 (fair)     ",
        "0.40-0.50 (fair)     ", "0.50-0.60 (poor)     ",
        "0.60-0.70 (poor)     ", "0.70-0.80 (bad)      ",
        "0.80-0.90 (bad)      ", "0.90-1.00 (very bad) ",
        "1.00+                "};
    std::fprintf(stderr, "\n--- top2/top1 Ratio Distribution ---\n");
    for (int b = 0; b < RowStats::RATIO_BINS; ++b) {
        if (g_stats.ratio_hist[static_cast<std::size_t>(b)] > 0) {
            const double pct =
                100.0 *
                static_cast<double>(
                    g_stats.ratio_hist[static_cast<std::size_t>(b)]) /
                static_cast<double>(g_stats.total_observations);
            std::fprintf(stderr, "  %s: %llu (%.2f%%)\n", bin_labels[b],
                         static_cast<unsigned long long>(
                             g_stats.ratio_hist[static_cast<std::size_t>(b)]),
                         pct);
        }
    }

    std::fprintf(stderr,
                 "\n  max_ratio = %.4f (가장 선명한 peak 분리)\n"
                 "  min_ratio = %.4f (가장 뭉개진 peak)\n"
                 "=================================\n",
                 g_stats.max_ratio, g_stats.min_ratio);
}

} // namespace WalshRowDiag
} // namespace ProtectedEngine
