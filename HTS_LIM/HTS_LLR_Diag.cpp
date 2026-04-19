#include "HTS_LLR_Diag.hpp"

#include <cstddef>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace ProtectedEngine {
namespace LLRDiag {

namespace {

LLRStats g_llr_stats{};

static int mag_bin_from_abs(uint32_t a) noexcept {
    if (a == 0u) {
        return 0;
    }
    if (a <= 10u) {
        return 1;
    }
    if (a <= 50u) {
        return 2;
    }
    if (a <= 100u) {
        return 3;
    }
    if (a <= 500u) {
        return 4;
    }
    if (a <= 1000u) {
        return 5;
    }
    if (a <= 5000u) {
        return 6;
    }
    if (a <= 10000u) {
        return 7;
    }
    if (a <= 50000u) {
        return 8;
    }
    if (a <= 100000u) {
        return 9;
    }
    if (a <= 500000u) {
        return 10;
    }
    return 11;
}

static uint32_t abs_u32(int32_t v) noexcept {
    const uint32_t u = static_cast<uint32_t>(v);
    const uint32_t m31 = static_cast<uint32_t>(static_cast<int32_t>(v) >> 31);
    return (u ^ m31) - m31;
}

static void bump_point(PointStats& ps, int32_t llr_val, int32_t clamp_limit) noexcept {
    ++ps.total_observations;
    if (llr_val > 0) {
        ++ps.pos_count;
    } else if (llr_val < 0) {
        ++ps.neg_count;
    } else {
        ++ps.zero_count;
    }

    const uint32_t a = abs_u32(llr_val);
    const int bin = mag_bin_from_abs(a);
    ++ps.mag_hist[static_cast<std::size_t>(bin)];

    const double ad = static_cast<double>(a);
    ps.sum_abs += ad;
    ps.sum_sq_abs += ad * ad;

    if (a > static_cast<uint32_t>(ps.max_abs)) {
        ps.max_abs = static_cast<int64_t>(a);
    }
    if (llr_val != 0) {
        if (ps.min_abs_nonzero == 0 ||
            static_cast<int64_t>(a) < ps.min_abs_nonzero) {
            ps.min_abs_nonzero = static_cast<int64_t>(a);
        }
    }

    if (clamp_limit != static_cast<int32_t>(0x7FFFFFFF)) {
        const uint32_t au = abs_u32(llr_val);
        const uint32_t cu = abs_u32(clamp_limit);
        if (cu != 0u && au >= cu) {
            ++ps.clamp_hits;
        }
    }
}

} // namespace

LLRStats& get_llr_stats() noexcept { return g_llr_stats; }

void reset_llr_stats() noexcept {
    std::memset(&g_llr_stats, 0, sizeof(g_llr_stats));
    for (int p = 0; p < static_cast<int>(ObservePoint::COUNT); ++p) {
        g_llr_stats.points[static_cast<std::size_t>(p)].max_abs = 0;
        g_llr_stats.points[static_cast<std::size_t>(p)].min_abs_nonzero = 0;
    }
}

void record_llr(ObservePoint pt, int32_t llr_val, int32_t clamp_limit) noexcept {
    const int pi = static_cast<int>(pt);
    if (pi < 0 || pi >= static_cast<int>(ObservePoint::COUNT)) {
        return;
    }
    bump_point(g_llr_stats.points[static_cast<std::size_t>(pi)], llr_val,
               clamp_limit);
}

void record_llr_with_truth(ObservePoint pt, int32_t llr_val, int32_t clamp_limit,
                           int tx_bit) noexcept {
    const int pi = static_cast<int>(pt);
    if (pi < 0 || pi >= static_cast<int>(ObservePoint::COUNT)) {
        return;
    }
    PointStats& ps = g_llr_stats.points[static_cast<std::size_t>(pi)];
    bump_point(ps, llr_val, clamp_limit);
    const int tb = tx_bit & 1;
    if (llr_val != 0) {
        const int hard = (llr_val < 0) ? 1 : 0;
        if (hard == tb) {
            ++ps.correct_sign_count;
        }
    }
}

double compute_reliability(ObservePoint pt) noexcept {
    const int pi = static_cast<int>(pt);
    if (pi < 0 || pi >= static_cast<int>(ObservePoint::COUNT)) {
        return 0.0;
    }
    const PointStats& ps = g_llr_stats.points[static_cast<std::size_t>(pi)];
    if (ps.total_observations == 0u) {
        return 0.0;
    }
    return ps.sum_abs / static_cast<double>(ps.total_observations);
}

double compute_consistency(ObservePoint pt) noexcept {
    const int pi = static_cast<int>(pt);
    if (pi < 0 || pi >= static_cast<int>(ObservePoint::COUNT)) {
        return 0.0;
    }
    const PointStats& ps = g_llr_stats.points[static_cast<std::size_t>(pi)];
    const double n = static_cast<double>(ps.total_observations);
    if (n <= 0.0) {
        return 0.0;
    }
    const double m1 = ps.sum_abs / n;
    const double m2 = ps.sum_sq_abs / n;
    const double v = m2 - m1 * m1;
    return (v > 0.0) ? v : 0.0;
}

void print_llr_stats(const char* label) noexcept {
    static const char* k_names[] = {"FWHT_OUT", "BIN_TO_LLR", "IR_ACCUM",
                                    "POLAR_PM"};
    std::fprintf(stderr, "\n=== LLR Diag [%s] ===\n",
                 label ? label : "ALL");
    for (int p = 0; p < static_cast<int>(ObservePoint::COUNT); ++p) {
        const PointStats& ps = g_llr_stats.points[static_cast<std::size_t>(p)];
        std::fprintf(stderr, "\n[POINT] %s  samples=%llu\n", k_names[p],
                     static_cast<unsigned long long>(ps.total_observations));
        if (ps.total_observations == 0u) {
            std::fprintf(stderr, "  (no samples)\n");
            continue;
        }
        std::fprintf(stderr,
                     "  reliability E[|LLR|]=%.6g  consistency Var[|LLR|]=%.6g\n",
                     compute_reliability(static_cast<ObservePoint>(p)),
                     compute_consistency(static_cast<ObservePoint>(p)));
        std::fprintf(stderr,
                     "  clamp_hits=%llu  max|.|=%lld  min|.|_nz=%lld  "
                     "pos/neg/zero=%llu/%llu/%llu  correct_sign=%llu\n",
                     static_cast<unsigned long long>(ps.clamp_hits),
                     static_cast<long long>(ps.max_abs),
                     static_cast<long long>(ps.min_abs_nonzero),
                     static_cast<unsigned long long>(ps.pos_count),
                     static_cast<unsigned long long>(ps.neg_count),
                     static_cast<unsigned long long>(ps.zero_count),
                     static_cast<unsigned long long>(ps.correct_sign_count));
        std::fprintf(stderr, "  mag_hist:");
        for (int b = 0; b < PointStats::HIST_BINS; ++b) {
            if (ps.mag_hist[static_cast<std::size_t>(b)] > 0u) {
                std::fprintf(stderr, " [%d]=%llu", b,
                             static_cast<unsigned long long>(
                                 ps.mag_hist[static_cast<std::size_t>(b)]));
            }
        }
        std::fprintf(stderr, "\n");
    }
    std::fprintf(stderr, "=== end LLR Diag ===\n\n");
}

} // namespace LLRDiag
} // namespace ProtectedEngine
