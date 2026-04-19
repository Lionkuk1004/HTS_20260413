#include "HTS_Row_Consistency_Diag.hpp"

#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstring>

namespace ProtectedEngine {
namespace RowConsistencyDiag {

namespace {

ConsistencyStats g_rcs{};

static int jump_bin(int j) noexcept {
    if (j < 0) {
        return 7;
    }
    if (j == 0) {
        return 0;
    }
    if (j <= 4) {
        return 1;
    }
    if (j <= 8) {
        return 2;
    }
    if (j <= 16) {
        return 3;
    }
    if (j <= 32) {
        return 4;
    }
    if (j <= 48) {
        return 5;
    }
    if (j <= 63) {
        return 6;
    }
    return 7;
}

} // namespace

ConsistencyStats& get_stats() noexcept { return g_rcs; }

void reset_stats() noexcept {
    std::memset(&g_rcs, 0, sizeof(g_rcs));
    g_rcs.last_top_row = -1;
}

void record_symbol(int top1, int top2, int64_t top1_energy, int64_t top2_energy,
                   bool expected_in_allowed) noexcept {
    if (top1 < 0 || top1 >= 64) {
        return;
    }
    if (top2 < 0 || top2 >= 64) {
        top2 = 0;
    }
    ++g_rcs.total_symbols;
    ++g_rcs.top_row_hist[static_cast<std::size_t>(top1)];
    if (expected_in_allowed) {
        ++g_rcs.in_allowed_count;
    } else {
        ++g_rcs.out_allowed_count;
    }

    if (top1_energy > 0) {
        const double ratio =
            static_cast<double>(top2_energy) / static_cast<double>(top1_energy);
        int bin = static_cast<int>(ratio * 10.0);
        if (bin < 0) {
            bin = 0;
        }
        if (bin >= ConsistencyStats::RATIO_BINS) {
            bin = ConsistencyStats::RATIO_BINS - 1;
        }
        ++g_rcs.ratio_hist[static_cast<std::size_t>(bin)];
    } else {
        ++g_rcs.ratio_hist[0];
    }

    if (g_rcs.last_top_row >= 0) {
        const int prev = static_cast<int>(g_rcs.last_top_row);
        const int j = (top1 > prev) ? (top1 - prev) : (prev - top1);
        ++g_rcs.jump_hist[static_cast<std::size_t>(jump_bin(j))];
    }
    g_rcs.last_top_row = static_cast<int32_t>(top1);
}

double compute_entropy() noexcept {
    const double n = static_cast<double>(g_rcs.total_symbols);
    if (n <= 0.0) {
        return 0.0;
    }
    double h = 0.0;
    for (int r = 0; r < 64; ++r) {
        const double c =
            static_cast<double>(g_rcs.top_row_hist[static_cast<std::size_t>(r)]);
        if (c <= 0.0) {
            continue;
        }
        const double p = c / n;
        h -= p * (std::log(p) / std::log(2.0));
    }
    return h;
}

double compute_concentration() noexcept {
    const double n = static_cast<double>(g_rcs.total_symbols);
    if (n <= 0.0) {
        return 0.0;
    }
    uint64_t tmp[64];
    for (int r = 0; r < 64; ++r) {
        tmp[static_cast<std::size_t>(r)] =
            g_rcs.top_row_hist[static_cast<std::size_t>(r)];
    }
    for (int i = 0; i < 64; ++i) {
        for (int j = i + 1; j < 64; ++j) {
            if (tmp[static_cast<std::size_t>(j)] > tmp[static_cast<std::size_t>(i)]) {
                const uint64_t t = tmp[static_cast<std::size_t>(i)];
                tmp[static_cast<std::size_t>(i)] = tmp[static_cast<std::size_t>(j)];
                tmp[static_cast<std::size_t>(j)] = t;
            }
        }
    }
    const uint64_t s4 = tmp[0] + tmp[1] + tmp[2] + tmp[3];
    return static_cast<double>(s4) / n;
}

double compute_allowed_ratio() noexcept {
    const uint64_t tot =
        g_rcs.in_allowed_count + g_rcs.out_allowed_count;
    if (tot == 0u) {
        return 0.0;
    }
    return static_cast<double>(g_rcs.in_allowed_count) /
           static_cast<double>(tot);
}

void print_stats(const char* label) noexcept {
    std::fprintf(stderr, "\n=== Row Consistency [%s] ===\n",
                 label ? label : "ALL");
    std::fprintf(stderr, "  total_symbols=%llu\n",
                 static_cast<unsigned long long>(g_rcs.total_symbols));
    if (g_rcs.total_symbols == 0u) {
        std::fprintf(stderr, "  (no data)\n=== end Row Consistency ===\n\n");
        return;
    }
    std::fprintf(stderr,
                 "  allowed_ratio=%.6f  entropy_H=%.6f bit  "
                 "concentration_top4=%.6f\n",
                 compute_allowed_ratio(), compute_entropy(),
                 compute_concentration());
    std::fprintf(stderr,
                 "  in_allowed=%llu  out_allowed=%llu\n",
                 static_cast<unsigned long long>(g_rcs.in_allowed_count),
                 static_cast<unsigned long long>(g_rcs.out_allowed_count));

    struct RowCnt {
        int r;
        uint64_t c;
    };
    RowCnt rows[64];
    for (int r = 0; r < 64; ++r) {
        rows[static_cast<std::size_t>(r)].r = r;
        rows[static_cast<std::size_t>(r)].c =
            g_rcs.top_row_hist[static_cast<std::size_t>(r)];
    }
    for (int i = 0; i < 64; ++i) {
        for (int j = i + 1; j < 64; ++j) {
            if (rows[static_cast<std::size_t>(j)].c >
                rows[static_cast<std::size_t>(i)].c) {
                const RowCnt t = rows[static_cast<std::size_t>(i)];
                rows[static_cast<std::size_t>(i)] = rows[static_cast<std::size_t>(j)];
                rows[static_cast<std::size_t>(j)] = t;
            }
        }
    }
    std::fprintf(stderr, "  top rows (count):");
    for (int k = 0; k < 8; ++k) {
        if (rows[static_cast<std::size_t>(k)].c > 0u) {
            std::fprintf(stderr, " r%d=%llu", rows[static_cast<std::size_t>(k)].r,
                         static_cast<unsigned long long>(
                             rows[static_cast<std::size_t>(k)].c));
        }
    }
    std::fprintf(stderr, "\n");

    std::fprintf(stderr, "  jump_hist [0]=same [1]=1-4 [2]=5-8 [3]=9-16 "
                 "[4]=17-32 [5]=33-48 [6]=49-63 [7]=other:\n    ");
    for (int b = 0; b < 8; ++b) {
        std::fprintf(stderr, "[%d]=%llu ", b,
                     static_cast<unsigned long long>(
                         g_rcs.jump_hist[static_cast<std::size_t>(b)]));
    }
    std::fprintf(stderr, "\n=== end Row Consistency ===\n\n");
}

} // namespace RowConsistencyDiag
} // namespace ProtectedEngine
