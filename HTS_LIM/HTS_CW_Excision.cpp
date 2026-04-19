#include "HTS_CW_Excision.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace ProtectedEngine {
namespace CWExcision {

namespace {

struct RowEnergy {
    int idx{};
    std::int64_t energy{};
};

[[nodiscard]] static bool sat_i32(std::int64_t v, std::int32_t* out) noexcept {
    if (v > 2147483647LL) {
        *out = 2147483647;
        return true;
    }
    if (v < static_cast<std::int64_t>(INT32_MIN)) {
        *out = INT32_MIN;
        return true;
    }
    *out = static_cast<std::int32_t>(v);
    return true;
}

/// Returns true if Phase-2 CW signature matches.
[[nodiscard]] static bool metrics_from_fwht(const std::int32_t* fI, const std::int32_t* fQ, int nc,
                                            double* out_conc, double* out_h) noexcept {
    if (fI == nullptr || fQ == nullptr || nc <= 0 || nc > 64) {
        return false;
    }
    std::int64_t energies[64];
    for (int i = 0; i < 64; ++i) {
        energies[static_cast<std::size_t>(i)] = 0;
    }
    std::int64_t total = 0;
    for (int r = 0; r < nc; ++r) {
        const std::int64_t vfi =
            static_cast<std::int64_t>(fI[static_cast<std::size_t>(r)]);
        const std::int64_t vfq =
            static_cast<std::int64_t>(fQ[static_cast<std::size_t>(r)]);
        const std::int64_t e = vfi * vfi + vfq * vfq;
        energies[static_cast<std::size_t>(r)] = e;
        total += e;
    }
    if (total <= 0) {
        return false;
    }

    std::int64_t sorted[64];
    for (int i = 0; i < nc; ++i) {
        sorted[static_cast<std::size_t>(i)] = energies[static_cast<std::size_t>(i)];
    }
    std::sort(sorted, sorted + nc, std::greater<std::int64_t>());

    const int k4 = (4 < nc) ? 4 : nc;
    std::int64_t top4 = 0;
    for (int i = 0; i < k4; ++i) {
        top4 += sorted[static_cast<std::size_t>(i)];
    }
    const double conc = static_cast<double>(top4) / static_cast<double>(total);

    double H = 0.0;
    const double den = static_cast<double>(total);
    for (int r = 0; r < nc; ++r) {
        const std::int64_t e = energies[static_cast<std::size_t>(r)];
        if (e > 0) {
            const double p = static_cast<double>(e) / den;
            H -= p * std::log2(p);
        }
    }

    if (out_conc != nullptr) {
        *out_conc = conc;
    }
    if (out_h != nullptr) {
        *out_h = H;
    }

    return (conc + 1e-15) >= CONC_TH_LO && conc <= (CONC_TH_HI + 1e-12) &&
           (H + 1e-15) >= ENT_LOW_BIT && H <= (ENT_HIGH_BIT + 1e-12);
}

#if defined(HTS_CW_EX_DIAG)
ExStats g_ex{};
#endif

} // namespace

Options default_build_options() noexcept {
    Options o{};
#if defined(HTS_CW_EX_MODE) && (HTS_CW_EX_MODE == 2)
    o.mode = Mode::SOFT_ATTEN;
#elif defined(HTS_CW_EX_MODE) && (HTS_CW_EX_MODE == 3)
    o.mode = Mode::MEDIAN_SUB;
#else
    o.mode = Mode::HARD_NULL;
#endif
    o.excise_top_k = 2;
#if defined(HTS_CW_EX_ATTEN)
    o.atten_factor = HTS_CW_EX_ATTEN;
#else
    o.atten_factor = 0.1;
#endif
    return o;
}

bool detect_only(const std::int32_t* fI, const std::int32_t* fQ, int nc) noexcept {
    return metrics_from_fwht(fI, fQ, nc, nullptr, nullptr);
}

bool detect_and_excise(std::int32_t* fI, std::int32_t* fQ, int nc, const Options& opt) noexcept {
#if defined(HTS_CW_EX_DIAG)
    ++g_ex.total_symbols;
#endif
    if (opt.mode == Mode::OFF) {
        return false;
    }
    if (fI == nullptr || fQ == nullptr || nc <= 0 || nc > 64) {
        return false;
    }

    RowEnergy re[64];
    std::int64_t total = 0;
    for (int r = 0; r < nc; ++r) {
        const std::int64_t vfi =
            static_cast<std::int64_t>(fI[static_cast<std::size_t>(r)]);
        const std::int64_t vfq =
            static_cast<std::int64_t>(fQ[static_cast<std::size_t>(r)]);
        const std::int64_t e = vfi * vfi + vfq * vfq;
        re[static_cast<std::size_t>(r)].idx = r;
        re[static_cast<std::size_t>(r)].energy = e;
        total += e;
    }
    if (total <= 0) {
        return false;
    }

    std::sort(re, re + nc, [](const RowEnergy& a, const RowEnergy& b) {
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
    for (int r = 0; r < nc; ++r) {
        const std::int64_t vfi =
            static_cast<std::int64_t>(fI[static_cast<std::size_t>(r)]);
        const std::int64_t vfq =
            static_cast<std::int64_t>(fQ[static_cast<std::size_t>(r)]);
        const std::int64_t e = vfi * vfi + vfq * vfq;
        if (e > 0) {
            const double p = static_cast<double>(e) / den;
            H -= p * std::log2(p);
        }
    }

    const bool is_cw = (conc + 1e-15) >= CONC_TH_LO && conc <= (CONC_TH_HI + 1e-12) &&
                       (H + 1e-15) >= ENT_LOW_BIT && H <= (ENT_HIGH_BIT + 1e-12);
    if (!is_cw) {
        return false;
    }

#if defined(HTS_CW_EX_DIAG)
    ++g_ex.detected_count;
#endif

    const int kex = (opt.excise_top_k < 1) ? 1 : ((opt.excise_top_k > nc) ? nc : opt.excise_top_k);

    switch (opt.mode) {
    case Mode::HARD_NULL:
        for (int i = 0; i < kex; ++i) {
            const int idx = re[static_cast<std::size_t>(i)].idx;
            fI[static_cast<std::size_t>(idx)] = 0;
            fQ[static_cast<std::size_t>(idx)] = 0;
#if defined(HTS_CW_EX_DIAG)
            if (idx >= 0 && idx < 64) {
                ++g_ex.excised_row_hits[static_cast<std::size_t>(idx)];
            }
#endif
        }
        break;
    case Mode::SOFT_ATTEN: {
        const int num = static_cast<int>(opt.atten_factor * 256.0 + 0.5);
        const int ncl = (num < 0) ? 0 : ((num > 256) ? 256 : num);
        for (int i = 0; i < kex; ++i) {
            const int idx = re[static_cast<std::size_t>(i)].idx;
            const std::int64_t ni =
                (static_cast<std::int64_t>(fI[static_cast<std::size_t>(idx)]) * ncl) >> 8;
            const std::int64_t nq =
                (static_cast<std::int64_t>(fQ[static_cast<std::size_t>(idx)]) * ncl) >> 8;
            (void)sat_i32(ni, &fI[static_cast<std::size_t>(idx)]);
            (void)sat_i32(nq, &fQ[static_cast<std::size_t>(idx)]);
#if defined(HTS_CW_EX_DIAG)
            if (idx >= 0 && idx < 64) {
                ++g_ex.excised_row_hits[static_cast<std::size_t>(idx)];
            }
#endif
        }
        break;
    }
    case Mode::MEDIAN_SUB: {
        const int med_start = kex;
        const int med_n = nc - med_start;
        if (med_n <= 0) {
            break;
        }
        const std::int64_t med_e = re[static_cast<std::size_t>(med_start + (med_n / 2))].energy;
        const double amp_d = std::sqrt(static_cast<double>(med_e) / 2.0);
        std::int64_t amp = static_cast<std::int64_t>(std::lround(amp_d));
        if (amp > 46340) {
            amp = 46340;
        }
        for (int i = 0; i < kex; ++i) {
            const int idx = re[static_cast<std::size_t>(i)].idx;
            const int sgn_i = (fI[static_cast<std::size_t>(idx)] >= 0) ? 1 : -1;
            const int sgn_q = (fQ[static_cast<std::size_t>(idx)] >= 0) ? 1 : -1;
            (void)sat_i32(static_cast<std::int64_t>(sgn_i) * amp,
                          &fI[static_cast<std::size_t>(idx)]);
            (void)sat_i32(static_cast<std::int64_t>(sgn_q) * amp,
                          &fQ[static_cast<std::size_t>(idx)]);
#if defined(HTS_CW_EX_DIAG)
            if (idx >= 0 && idx < 64) {
                ++g_ex.excised_row_hits[static_cast<std::size_t>(idx)];
            }
#endif
        }
        break;
    }
    default:
        return false;
    }

#if defined(HTS_CW_EX_DIAG)
    ++g_ex.excised_count;
#endif
    return true;
}

#if defined(HTS_CW_EX_DIAG)
ExStats& get_stats() noexcept { return g_ex; }

void reset_stats() noexcept { std::memset(&g_ex, 0, sizeof(g_ex)); }

void print_stats(const char* label) noexcept {
    std::fprintf(stderr, "\n=== CW Excision stats [%s] ===\n", label ? label : "ALL");
    std::fprintf(stderr, "  total_symbols=%llu detected=%llu excised=%llu\n",
                 static_cast<unsigned long long>(g_ex.total_symbols),
                 static_cast<unsigned long long>(g_ex.detected_count),
                 static_cast<unsigned long long>(g_ex.excised_count));
    std::fprintf(stderr, "  top excised rows (idx:count):");
    std::uint64_t best = 0;
    for (int r = 0; r < 64; ++r) {
        const std::uint64_t c = g_ex.excised_row_hits[static_cast<std::size_t>(r)];
        if (c > best) {
            best = c;
        }
    }
    int shown = 0;
    for (int r = 0; r < 64 && shown < 8; ++r) {
        const std::uint64_t c = g_ex.excised_row_hits[static_cast<std::size_t>(r)];
        if (c > 0u) {
            std::fprintf(stderr, " %d:%llu", r, static_cast<unsigned long long>(c));
            ++shown;
        }
    }
    std::fprintf(stderr, "\n=== end CW Excision stats ===\n\n");
}
#endif

} // namespace CWExcision
} // namespace ProtectedEngine
