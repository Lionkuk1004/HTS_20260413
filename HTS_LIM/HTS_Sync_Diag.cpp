// =============================================================================
// HTS_Sync_Diag.cpp — Preamble phase0_scan_ 동기 정밀 계측
// =============================================================================
#include "HTS_Sync_Diag.hpp"

#include <cstddef>
#include <cstdio>
#include <cstring>
#include <limits>

namespace ProtectedEngine {
namespace SyncDiag {

static SyncStats g_stats = {};

SyncStats& get_sync_stats() noexcept { return g_stats; }

void reset_sync_stats() noexcept {
    std::memset(&g_stats, 0, sizeof(g_stats));
    g_stats.best_off_min = 64;
    g_stats.min_e63_observed_passed =
        (std::numeric_limits<int64_t>::max)();
}

static int e63_to_bin(int64_t v) noexcept {
    if (v < 1000) {
        return 0;
    }
    if (v < 10000) {
        return 1;
    }
    if (v < 100000) {
        return 2;
    }
    if (v < 1000000) {
        return 3;
    }
    if (v < 10000000) {
        return 4;
    }
    if (v < 100000000) {
        return 5;
    }
    return 6;
}

static int ratio_to_bin(int64_t sec, int64_t best) noexcept {
    if (best <= 0) {
        return 0;
    }
    // ratio = sec / best, Q8 scale
    const int64_t r_q8 = (sec * 256) / best;
    if (r_q8 < 26) {
        return 0;
    } // < 0.1
    if (r_q8 < 64) {
        return 1;
    } // 0.1 ~ 0.25
    if (r_q8 < 128) {
        return 2;
    } // 0.25 ~ 0.5
    if (r_q8 < 192) {
        return 3;
    } // 0.5 ~ 0.75
    return 4; // >= 0.75
}

void record_scan(int best_off, int64_t best_e63, int64_t second_e63,
                 bool pass) noexcept {
    ++g_stats.total_scans;

    if (pass) {
        ++g_stats.passed_scans;

        if (best_off >= 0 && best_off < 64) {
            ++g_stats.best_off_hist[static_cast<std::size_t>(best_off)];
            g_stats.sum_best_off += best_off;
            if (static_cast<uint64_t>(best_off) < g_stats.best_off_min) {
                g_stats.best_off_min = static_cast<uint64_t>(best_off);
            }
            if (static_cast<uint64_t>(best_off) > g_stats.best_off_max) {
                g_stats.best_off_max = static_cast<uint64_t>(best_off);
            }
        }

        ++g_stats.best_e63_hist[e63_to_bin(best_e63)];
        ++g_stats.sec_ratio_hist[ratio_to_bin(second_e63, best_e63)];

        if (best_e63 > g_stats.max_e63_observed) {
            g_stats.max_e63_observed = best_e63;
        }
        if (best_e63 < g_stats.min_e63_observed_passed) {
            g_stats.min_e63_observed_passed = best_e63;
        }
        g_stats.sum_e63_passed += best_e63;
    } else {
        ++g_stats.failed_scans;
    }
}

void print_sync_stats(const char* label) noexcept {
    std::fprintf(stderr, "\n=== Sync Stats [%s] ===\n",
                 label ? label : "ALL");
    std::fprintf(stderr,
                 "  total_scans   = %llu\n"
                 "  passed        = %llu  (%.2f%%)\n"
                 "  failed        = %llu  (%.2f%%)\n",
                 static_cast<unsigned long long>(g_stats.total_scans),
                 static_cast<unsigned long long>(g_stats.passed_scans),
                 (g_stats.total_scans > 0)
                     ? 100.0 * static_cast<double>(g_stats.passed_scans) /
                           static_cast<double>(g_stats.total_scans)
                     : 0.0,
                 static_cast<unsigned long long>(g_stats.failed_scans),
                 (g_stats.total_scans > 0)
                     ? 100.0 * static_cast<double>(g_stats.failed_scans) /
                           static_cast<double>(g_stats.total_scans)
                     : 0.0);

    if (g_stats.passed_scans > 0) {
        const double avg_off =
            static_cast<double>(g_stats.sum_best_off) /
            static_cast<double>(g_stats.passed_scans);
        const double avg_e63 =
            static_cast<double>(g_stats.sum_e63_passed) /
            static_cast<double>(g_stats.passed_scans);
        std::fprintf(stderr,
                     "\n  best_off: min=%llu, max=%llu, avg=%.3f\n",
                     static_cast<unsigned long long>(g_stats.best_off_min),
                     static_cast<unsigned long long>(g_stats.best_off_max),
                     avg_off);
        std::fprintf(stderr,
                     "  best_e63: min=%lld, max=%lld, avg=%.3e\n",
                     static_cast<long long>(g_stats.min_e63_observed_passed),
                     static_cast<long long>(g_stats.max_e63_observed),
                     avg_e63);
    }

    std::fprintf(stderr, "\n--- best_off Distribution ---\n");
    for (int i = 0; i < 64; ++i) {
        if (g_stats.best_off_hist[static_cast<std::size_t>(i)] > 0 &&
            g_stats.passed_scans > 0) {
            const double pct =
                100.0 *
                static_cast<double>(
                    g_stats.best_off_hist[static_cast<std::size_t>(i)]) /
                static_cast<double>(g_stats.passed_scans);
            std::fprintf(stderr, "  off=%2d: %llu (%.2f%%)\n", i,
                         static_cast<unsigned long long>(
                             g_stats.best_off_hist[static_cast<std::size_t>(i)]),
                         pct);
        }
    }

    static const char* const e63_labels[SyncStats::E63_BINS] = {
        "      < 1K  ", "  1K ~ 10K  ", " 10K ~ 100K ",
        "100K ~ 1M   ", "  1M ~ 10M  ", " 10M ~ 100M ", "     > 100M "};
    std::fprintf(stderr, "\n--- best_e63 Distribution ---\n");
    for (int i = 0; i < SyncStats::E63_BINS; ++i) {
        if (g_stats.best_e63_hist[static_cast<std::size_t>(i)] > 0) {
            std::fprintf(stderr, "  %s: %llu\n", e63_labels[i],
                         static_cast<unsigned long long>(
                             g_stats.best_e63_hist[static_cast<std::size_t>(i)]));
        }
    }

    static const char* const ratio_labels[SyncStats::RATIO_BINS] = {
        "< 0.10 (excellent) ", "0.10 ~ 0.25 (good) ", "0.25 ~ 0.50 (fair) ",
        "0.50 ~ 0.75 (poor) ", ">= 0.75 (bad)      "};
    std::fprintf(stderr, "\n--- second/best Ratio ---\n");
    for (int i = 0; i < SyncStats::RATIO_BINS; ++i) {
        if (g_stats.sec_ratio_hist[static_cast<std::size_t>(i)] > 0) {
            std::fprintf(stderr, "  %s: %llu\n", ratio_labels[i],
                         static_cast<unsigned long long>(
                             g_stats.sec_ratio_hist[static_cast<std::size_t>(i)]));
        }
    }
    std::fprintf(stderr, "================================\n");
}

} // namespace SyncDiag
} // namespace ProtectedEngine
