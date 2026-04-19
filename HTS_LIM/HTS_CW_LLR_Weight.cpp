#include "HTS_CW_LLR_Weight.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstring>

namespace ProtectedEngine {
namespace CWLLRWeight {

namespace {

struct Row {
    int idx{};
    std::int64_t energy{};
};

#if defined(HTS_CW_W_DIAG)
WeightStats g_w{};
#endif

} // namespace

DetectionResult detect_per_symbol(const std::int32_t* fI, const std::int32_t* fQ, int nc,
                                  int top_k) noexcept {
    DetectionResult r{};
#if defined(HTS_CW_W_DIAG)
    ++g_w.total_symbols;
#endif
    if (fI == nullptr || fQ == nullptr || nc <= 0 || nc > 64) {
        return r;
    }

    Row re[64];
    std::int64_t total = 0;
    for (int i = 0; i < nc; ++i) {
        const std::int64_t vfi =
            static_cast<std::int64_t>(fI[static_cast<std::size_t>(i)]);
        const std::int64_t vfq =
            static_cast<std::int64_t>(fQ[static_cast<std::size_t>(i)]);
        const std::int64_t e = vfi * vfi + vfq * vfq;
        re[static_cast<std::size_t>(i)].idx = i;
        re[static_cast<std::size_t>(i)].energy = e;
        total += e;
    }
    if (total <= 0) {
        return r;
    }

    std::sort(re, re + nc, [](const Row& a, const Row& b) {
        return a.energy > b.energy;
    });

    const int k4 = (4 < nc) ? 4 : nc;
    std::int64_t top4 = 0;
    for (int i = 0; i < k4; ++i) {
        top4 += re[static_cast<std::size_t>(i)].energy;
    }
    const double conc = static_cast<double>(top4) / static_cast<double>(total);

    double H = 0.0;
    const double den = static_cast<double>(total);
    for (int row = 0; row < nc; ++row) {
        const std::int64_t vfi =
            static_cast<std::int64_t>(fI[static_cast<std::size_t>(row)]);
        const std::int64_t vfq =
            static_cast<std::int64_t>(fQ[static_cast<std::size_t>(row)]);
        const std::int64_t e = vfi * vfi + vfq * vfq;
        if (e > 0) {
            const double p = static_cast<double>(e) / den;
            H -= p * std::log2(p);
        }
    }

    r.is_cw = (conc + 1e-15) >= CONC_TH_LO && conc <= (CONC_TH_HI + 1e-12) &&
              (H + 1e-15) >= ENT_LOW_BIT && H <= (ENT_HIGH_BIT + 1e-12);

    if (!r.is_cw) {
        return r;
    }

#if defined(HTS_CW_W_DIAG)
    ++g_w.cw_detected;
#endif

    int tk = top_k;
    if (tk < 1) {
        tk = 1;
    }
    if (tk > 8) {
        tk = 8;
    }
    if (tk > nc) {
        tk = nc;
    }
    r.cw_row_count = tk;
    for (int i = 0; i < tk; ++i) {
        r.cw_rows[i] = re[static_cast<std::size_t>(i)].idx;
#if defined(HTS_CW_W_DIAG)
        ++g_w.weighted_rows_total;
        const int rr = r.cw_rows[i];
        if (rr >= 0 && rr < 64) {
            ++g_w.weight_hit_by_row[static_cast<std::size_t>(rr)];
        }
#endif
    }

    return r;
}

#if defined(HTS_CW_W_DIAG)
WeightStats& get_stats() noexcept { return g_w; }

void reset_stats() noexcept { std::memset(&g_w, 0, sizeof(g_w)); }

void print_stats(const char* label) noexcept {
    std::fprintf(stderr, "\n=== CW LLR weight stats [%s] ===\n", label ? label : "ALL");
    std::fprintf(stderr, "  total_symbols=%llu cw_detected=%llu weighted_rows_total=%llu\n",
                 static_cast<unsigned long long>(g_w.total_symbols),
                 static_cast<unsigned long long>(g_w.cw_detected),
                 static_cast<unsigned long long>(g_w.weighted_rows_total));
    std::fprintf(stderr, "  top weighted rows (idx:count):");
    int shown = 0;
    for (int row = 0; row < 64 && shown < 8; ++row) {
        const std::uint64_t c = g_w.weight_hit_by_row[static_cast<std::size_t>(row)];
        if (c > 0u) {
            std::fprintf(stderr, " %d:%llu", row, static_cast<unsigned long long>(c));
            ++shown;
        }
    }
    std::fprintf(stderr, "\n=== end CW LLR weight stats ===\n\n");
}
#endif

} // namespace CWLLRWeight
} // namespace ProtectedEngine
