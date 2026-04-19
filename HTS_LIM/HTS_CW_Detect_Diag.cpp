#include "HTS_CW_Detect_Diag.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstring>

namespace ProtectedEngine {
namespace CWDetectDiag {

namespace {

CWDetectStats g_cw{};
bool g_expect_non_cw = true;

double pkt_sym_conc_sum = 0.0;
double pkt_sym_ent_sum = 0.0;
int pkt_sym_n = 0;

static constexpr double kConcTh[] = {0.5, 0.7, 0.8, 0.9, 0.95};
static constexpr double kEntTh[] = {2.0, 1.5, 1.0, 0.8, 0.5};

[[nodiscard]] static double shannon_entropy_bits(const std::int64_t* e, int ncl,
                                                   std::int64_t total) noexcept {
    if (ncl <= 0 || total <= 0) {
        return 0.0;
    }
    const double den = static_cast<double>(total);
    double h = 0.0;
    for (int r = 0; r < ncl; ++r) {
        const double p = static_cast<double>(e[static_cast<std::size_t>(r)]) / den;
        if (p > 1e-12) {
            h -= p * std::log2(p);
        }
    }
    return h;
}

[[nodiscard]] static int bin_ratio_t2t1(double t2t1) noexcept {
    int b = static_cast<int>(t2t1 * static_cast<double>(CWDetectStats::RATIO_BINS_FINE));
    if (b < 0) {
        b = 0;
    }
    if (b >= CWDetectStats::RATIO_BINS_FINE) {
        b = CWDetectStats::RATIO_BINS_FINE - 1;
    }
    return b;
}

[[nodiscard]] static int bin_conc(double conc, int nbins) noexcept {
    int b = static_cast<int>(conc * static_cast<double>(nbins));
    if (b < 0) {
        b = 0;
    }
    if (b >= nbins) {
        b = nbins - 1;
    }
    return b;
}

[[nodiscard]] static int bin_entropy_win(double h) noexcept {
    int hb = static_cast<int>(h * 4.0);
    if (hb < 0) {
        hb = 0;
    }
    if (hb >= CWDetectStats::WIN_ENT_BINS) {
        hb = CWDetectStats::WIN_ENT_BINS - 1;
    }
    return hb;
}

/// Smallest bin index b such that cumulative sum in [0..b] >= frac * total.
[[nodiscard]] static int percentile_bin_from_hist(const std::uint64_t* hist, int nbins,
                                                  double frac) noexcept {
    std::uint64_t tot = 0;
    for (int i = 0; i < nbins; ++i) {
        tot += hist[static_cast<std::size_t>(i)];
    }
    if (tot == 0u) {
        return -1;
    }
    const std::uint64_t target =
        static_cast<std::uint64_t>(std::ceil(frac * static_cast<double>(tot)));
    std::uint64_t c = 0;
    for (int i = 0; i < nbins; ++i) {
        c += hist[static_cast<std::size_t>(i)];
        if (c >= target) {
            return i;
        }
    }
    return nbins - 1;
}

} // namespace

CWDetectStats& get_stats() noexcept { return g_cw; }

void reset_stats() noexcept {
    std::memset(&g_cw, 0, sizeof(g_cw));
    g_cw.buffer_idx = 0;
    g_cw.win_filled = 0;
    pkt_sym_conc_sum = 0.0;
    pkt_sym_ent_sum = 0.0;
    pkt_sym_n = 0;
}

void set_expect_non_cw(bool non_cw) noexcept { g_expect_non_cw = non_cw; }

void record_symbol_from_fwht(const std::int32_t* fi, const std::int32_t* fq,
                             int nc) noexcept {
    if (fi == nullptr || fq == nullptr || nc <= 0) {
        return;
    }
    const int ncl = (nc < 64) ? nc : 64;
    std::int64_t e[64];
    for (int i = 0; i < 64; ++i) {
        e[static_cast<std::size_t>(i)] = 0;
    }
    std::int64_t total = 0;
    for (int r = 0; r < ncl; ++r) {
        const std::int64_t vfi =
            static_cast<std::int64_t>(fi[static_cast<std::size_t>(r)]);
        const std::int64_t vfq =
            static_cast<std::int64_t>(fq[static_cast<std::size_t>(r)]);
        const std::int64_t er = vfi * vfi + vfq * vfq;
        e[static_cast<std::size_t>(r)] = er;
        total += er;
    }
    if (total <= 0) {
        return;
    }

    ++g_cw.total_symbols;

    std::int64_t sorted[64];
    std::memcpy(sorted, e, sizeof(sorted));
    std::sort(sorted, sorted + 64, std::greater<std::int64_t>());

    const double t2t1 =
        (sorted[0] > 0) ? static_cast<double>(sorted[1]) / static_cast<double>(sorted[0]) : 0.0;
    ++g_cw.per_sym_t2t1_hist[static_cast<std::size_t>(bin_ratio_t2t1(t2t1))];

    const std::int64_t top4 =
        sorted[0] + sorted[1] + sorted[2] + sorted[3];
    const double conc = static_cast<double>(top4) / static_cast<double>(total);
    ++g_cw.per_sym_conc_hist[static_cast<std::size_t>(
        bin_conc(conc, CWDetectStats::CONC_BINS))];

    pkt_sym_conc_sum += conc;
    const double Hsym = shannon_entropy_bits(e, ncl, total);
    pkt_sym_ent_sum += Hsym;
    ++pkt_sym_n;

    const int bi = g_cw.buffer_idx;
    g_cw.recent_concentration[static_cast<std::size_t>(bi)] = conc;
    g_cw.recent_entropy[static_cast<std::size_t>(bi)] = Hsym;
    g_cw.buffer_idx = (bi + 1) % CWDetectStats::WINDOW_SIZE;
    if (g_cw.win_filled < CWDetectStats::WINDOW_SIZE) {
        ++g_cw.win_filled;
    }
    if (g_cw.win_filled < CWDetectStats::WINDOW_SIZE) {
        return;
    }

    double win_avg_c = 0.0;
    double win_avg_h = 0.0;
    for (int i = 0; i < CWDetectStats::WINDOW_SIZE; ++i) {
        win_avg_c += g_cw.recent_concentration[static_cast<std::size_t>(i)];
        win_avg_h += g_cw.recent_entropy[static_cast<std::size_t>(i)];
    }
    win_avg_c /= static_cast<double>(CWDetectStats::WINDOW_SIZE);
    win_avg_h /= static_cast<double>(CWDetectStats::WINDOW_SIZE);

    ++g_cw.win_conc_hist[static_cast<std::size_t>(
        bin_conc(win_avg_c, CWDetectStats::WIN_CONC_BINS))];
    ++g_cw.win_entropy_hist[static_cast<std::size_t>(bin_entropy_win(win_avg_h))];
}

void mark_packet_boundary() noexcept {
    if (pkt_sym_n <= 0) {
        return;
    }
    const double mean_c = pkt_sym_conc_sum / static_cast<double>(pkt_sym_n);
    const double mean_h = pkt_sym_ent_sum / static_cast<double>(pkt_sym_n);

    if (g_expect_non_cw) {
        ++g_cw.pkt_total_clean;
        for (int i = 0; i < CWDetectStats::PKT_TH_N; ++i) {
            if (mean_c > kConcTh[static_cast<std::size_t>(i)]) {
                ++g_cw.pkt_conc_high_clean[static_cast<std::size_t>(i)];
            }
            if (mean_h < kEntTh[static_cast<std::size_t>(i)]) {
                ++g_cw.pkt_entropy_low_clean[static_cast<std::size_t>(i)];
            }
        }
    } else {
        ++g_cw.pkt_total_cw;
        for (int i = 0; i < CWDetectStats::PKT_TH_N; ++i) {
            if (mean_c > kConcTh[static_cast<std::size_t>(i)]) {
                ++g_cw.pkt_conc_high_cw[static_cast<std::size_t>(i)];
            }
            if (mean_h < kEntTh[static_cast<std::size_t>(i)]) {
                ++g_cw.pkt_entropy_low_cw[static_cast<std::size_t>(i)];
            }
        }
    }

    pkt_sym_conc_sum = 0.0;
    pkt_sym_ent_sum = 0.0;
    pkt_sym_n = 0;
}

void print_stats(const char* label) noexcept {
    std::fprintf(stderr, "\n=== CW Detect Phase1 [%s] ===\n", label ? label : "ALL");
    std::fprintf(stderr, "  total_symbols=%llu  pkt_clean=%llu pkt_cw=%llu\n",
                 static_cast<unsigned long long>(g_cw.total_symbols),
                 static_cast<unsigned long long>(g_cw.pkt_total_clean),
                 static_cast<unsigned long long>(g_cw.pkt_total_cw));
    if (g_cw.total_symbols == 0u) {
        std::fprintf(stderr, "  (no data)\n=== end CW Detect Phase1 ===\n\n");
        return;
    }

    auto peak_bin = [](const std::uint64_t* h, int nb) -> int {
        int mx = 0;
        for (int i = 1; i < nb; ++i) {
            if (h[static_cast<std::size_t>(i)] > h[static_cast<std::size_t>(mx)]) {
                mx = i;
            }
        }
        return mx;
    };

    const int pk_t2 = peak_bin(g_cw.per_sym_t2t1_hist, CWDetectStats::RATIO_BINS_FINE);
    std::fprintf(stderr,
                 "  per_sym_t2t1: peak_bin=%d (~%.3f-%.3f ratio top2/top1)\n", pk_t2,
                 static_cast<double>(pk_t2) / 20.0, static_cast<double>(pk_t2 + 1) / 20.0);

    const int pk_ps = peak_bin(g_cw.per_sym_conc_hist, CWDetectStats::CONC_BINS);
    const int p95_ps =
        percentile_bin_from_hist(g_cw.per_sym_conc_hist, CWDetectStats::CONC_BINS, 0.95);
    const int pk_wc = peak_bin(g_cw.win_conc_hist, CWDetectStats::WIN_CONC_BINS);
    const int p95_wc =
        percentile_bin_from_hist(g_cw.win_conc_hist, CWDetectStats::WIN_CONC_BINS, 0.95);
    const int pk_we = peak_bin(g_cw.win_entropy_hist, CWDetectStats::WIN_ENT_BINS);
    const int p05_we =
        percentile_bin_from_hist(g_cw.win_entropy_hist, CWDetectStats::WIN_ENT_BINS, 0.05);

    std::fprintf(stderr,
                 "  per_sym_conc: peak_bin=%d (~%.3f-%.3f) p95_bin=%d (~%.3f-%.3f)\n",
                 pk_ps, static_cast<double>(pk_ps) / 20.0,
                 static_cast<double>(pk_ps + 1) / 20.0, p95_ps,
                 static_cast<double>(p95_ps) / 20.0,
                 static_cast<double>(p95_ps + 1) / 20.0);
    std::fprintf(stderr,
                 "  win8_conc:    peak_bin=%d (~%.3f-%.3f) p95_bin=%d (~%.3f-%.3f)\n",
                 pk_wc, static_cast<double>(pk_wc) / 20.0,
                 static_cast<double>(pk_wc + 1) / 20.0, p95_wc,
                 static_cast<double>(p95_wc) / 20.0,
                 static_cast<double>(p95_wc + 1) / 20.0);
    std::fprintf(stderr,
                 "  win8_entropy: peak_bin=%d (~%.2f-%.2f bit) p05_bin=%d (~%.2f-%.2f bit)\n",
                 pk_we, static_cast<double>(pk_we) * 0.25,
                 static_cast<double>(pk_we + 1) * 0.25, p05_we,
                 static_cast<double>(p05_we) * 0.25,
                 static_cast<double>(p05_we + 1) * 0.25);

    std::fprintf(stderr, "  pkt mean conc/ent threshold rates (non-CW pkts=%llu CW pkts=%llu):\n",
                 static_cast<unsigned long long>(g_cw.pkt_total_clean),
                 static_cast<unsigned long long>(g_cw.pkt_total_cw));
    for (int i = 0; i < CWDetectStats::PKT_TH_N; ++i) {
        const double fp =
            (g_cw.pkt_total_clean > 0u)
                ? 100.0 * static_cast<double>(g_cw.pkt_conc_high_clean[static_cast<std::size_t>(i)]) /
                      static_cast<double>(g_cw.pkt_total_clean)
                : 0.0;
        const double tp =
            (g_cw.pkt_total_cw > 0u)
                ? 100.0 * static_cast<double>(g_cw.pkt_conc_high_cw[static_cast<std::size_t>(i)]) /
                      static_cast<double>(g_cw.pkt_total_cw)
                : 0.0;
        std::fprintf(stderr, "    conc_mean>%.2f  non_CW%%=%.3f  CW%%=%.3f\n", kConcTh[i], fp,
                     tp);
    }
    for (int i = 0; i < CWDetectStats::PKT_TH_N; ++i) {
        const double fp =
            (g_cw.pkt_total_clean > 0u)
                ? 100.0 *
                      static_cast<double>(g_cw.pkt_entropy_low_clean[static_cast<std::size_t>(i)]) /
                      static_cast<double>(g_cw.pkt_total_clean)
                : 0.0;
        const double tp =
            (g_cw.pkt_total_cw > 0u)
                ? 100.0 * static_cast<double>(g_cw.pkt_entropy_low_cw[static_cast<std::size_t>(i)]) /
                      static_cast<double>(g_cw.pkt_total_cw)
                : 0.0;
        std::fprintf(stderr, "    ent_mean<%.2f  non_CW%%=%.3f  CW%%=%.3f\n", kEntTh[i], fp, tp);
    }

    std::fprintf(stderr, "=== end CW Detect Phase1 ===\n\n");
}

#if defined(HTS_CW_DETECT_DIAG_V2)

namespace {

static constexpr BandCandidate kBandCandidatesV2[] = {
    {"CW_tight", 0.85, 1.00, 1.50, 2.25},
    {"CW_loose", 0.80, 1.00, 1.25, 2.75},
    {"CW_medium", 0.85, 1.00, 1.50, 2.50},
    {"CW_hiConc", 0.90, 1.00, 1.00, 3.00},
    {"CW_midEnt", 0.70, 1.00, 1.50, 2.50},
    {"Clean_tight", 0.90, 1.00, 0.00, 1.00},
    {"Barrage", 0.40, 0.75, 3.00, 5.00},
    {"LowSNR", 0.30, 0.70, 3.50, 5.00},
};

struct CW2DState {
    std::uint64_t joint_hist_per_sym[CW2D_CONC_BINS][CW2D_ENT_BINS];
    std::uint64_t joint_hist_win4[CW2D_CONC_BINS][CW2D_ENT_BINS];
    std::uint64_t joint_hist_win8[CW2D_CONC_BINS][CW2D_ENT_BINS];
    std::uint64_t joint_hist_win16[CW2D_CONC_BINS][CW2D_ENT_BINS];

    std::uint64_t joint_by_label[static_cast<int>(CWDetectScenarioLabel::COUNT)][CW2D_CONC_BINS]
                                  [CW2D_ENT_BINS];

    std::uint64_t band_hits[CW2D_BAND_CANDIDATES];
    std::uint64_t pkt_total;
    std::uint64_t pkt_in_band[CW2D_BAND_CANDIDATES];

    std::uint32_t cur_pkt_sym_count;
    std::uint32_t cur_pkt_band_count[CW2D_BAND_CANDIDATES];

    std::uint64_t total_symbols_v2;

    double recent_conc_4[4];
    double recent_ent_4[4];
    int idx_4{};
    int filled_4{};

    double recent_conc_8[8];
    double recent_ent_8[8];
    int idx_8{};
    int filled_8{};

    double recent_conc_16[16];
    double recent_ent_16[16];
    int idx_16{};
    int filled_16{};

    CWDetectScenarioLabel cur_label{CWDetectScenarioLabel::Clean};

    std::uint64_t label_sym_total[static_cast<int>(CWDetectScenarioLabel::COUNT)];
    std::uint64_t label_band_sym_hits[static_cast<int>(CWDetectScenarioLabel::COUNT)]
                                     [CW2D_BAND_CANDIDATES];
    std::uint64_t label_pkt_total[static_cast<int>(CWDetectScenarioLabel::COUNT)];
    std::uint64_t label_pkt_band_hits[static_cast<int>(CWDetectScenarioLabel::COUNT)]
                                     [CW2D_BAND_CANDIDATES];
};

CW2DState g2{};

[[nodiscard]] static bool in_band_v2(int bi, double conc, double H) noexcept {
    const BandCandidate& b = kBandCandidatesV2[static_cast<std::size_t>(bi)];
    return (conc + 1e-15) >= b.conc_min && conc <= (b.conc_max + 1e-12) &&
           (H + 1e-15) >= b.ent_min && H <= (b.ent_max + 1e-12);
}

[[nodiscard]] static int conc_bin_v2(double conc) noexcept {
    int cb = static_cast<int>(conc * static_cast<double>(CW2D_CONC_BINS));
    if (cb < 0) {
        cb = 0;
    }
    if (cb >= CW2D_CONC_BINS) {
        cb = CW2D_CONC_BINS - 1;
    }
    return cb;
}

[[nodiscard]] static int ent_bin_v2(double H) noexcept {
    int eb = static_cast<int>(std::floor(H * 4.0));
    if (eb < 0) {
        eb = 0;
    }
    if (eb >= CW2D_ENT_BINS) {
        eb = CW2D_ENT_BINS - 1;
    }
    return eb;
}

static void push_window_joint(std::uint64_t hist[CW2D_CONC_BINS][CW2D_ENT_BINS], int wlen,
                              double* cbuf, double* ebuf, int& idx, int& filled, double conc,
                              double H) noexcept {
    cbuf[static_cast<std::size_t>(idx)] = conc;
    ebuf[static_cast<std::size_t>(idx)] = H;
    idx = (idx + 1) % wlen;
    if (filled < wlen) {
        ++filled;
    }
    if (filled < wlen) {
        return;
    }
    double ac = 0.0;
    double ae = 0.0;
    for (int i = 0; i < wlen; ++i) {
        ac += cbuf[static_cast<std::size_t>(i)];
        ae += ebuf[static_cast<std::size_t>(i)];
    }
    ac /= static_cast<double>(wlen);
    ae /= static_cast<double>(wlen);
    const int cb = conc_bin_v2(ac);
    const int eb = ent_bin_v2(ae);
    ++hist[static_cast<std::size_t>(cb)][static_cast<std::size_t>(eb)];
}

[[nodiscard]] static std::uint64_t count_joint_region(const std::uint64_t hist[CW2D_CONC_BINS]
                                                                 [CW2D_ENT_BINS],
                                                        double c_min, double c_max, double e_min,
                                                        double e_max) noexcept {
    const int c0 = conc_bin_v2(c_min);
    const int c1 = conc_bin_v2(c_max - 1e-12);
    const int e0 = ent_bin_v2(e_min);
    const int e1 = ent_bin_v2(e_max - 1e-12);
    std::uint64_t sum = 0;
    for (int c = c0; c <= c1 && c < CW2D_CONC_BINS; ++c) {
        for (int e = e0; e <= e1 && e < CW2D_ENT_BINS; ++e) {
            sum += hist[static_cast<std::size_t>(c)][static_cast<std::size_t>(e)];
        }
    }
    return sum;
}

} // namespace

void reset_joint_label_accum() noexcept {
    std::memset(g2.joint_by_label, 0, sizeof(g2.joint_by_label));
    std::memset(g2.label_sym_total, 0, sizeof(g2.label_sym_total));
    std::memset(g2.label_band_sym_hits, 0, sizeof(g2.label_band_sym_hits));
    std::memset(g2.label_pkt_total, 0, sizeof(g2.label_pkt_total));
    std::memset(g2.label_pkt_band_hits, 0, sizeof(g2.label_pkt_band_hits));
}

void reset_stats_v2() noexcept {
    std::memset(g2.joint_hist_per_sym, 0, sizeof(g2.joint_hist_per_sym));
    std::memset(g2.joint_hist_win4, 0, sizeof(g2.joint_hist_win4));
    std::memset(g2.joint_hist_win8, 0, sizeof(g2.joint_hist_win8));
    std::memset(g2.joint_hist_win16, 0, sizeof(g2.joint_hist_win16));
    std::memset(g2.band_hits, 0, sizeof(g2.band_hits));
    g2.pkt_total = 0;
    std::memset(g2.pkt_in_band, 0, sizeof(g2.pkt_in_band));
    g2.cur_pkt_sym_count = 0;
    std::memset(g2.cur_pkt_band_count, 0, sizeof(g2.cur_pkt_band_count));
    g2.total_symbols_v2 = 0;
    g2.idx_4 = g2.filled_4 = 0;
    g2.idx_8 = g2.filled_8 = 0;
    g2.idx_16 = g2.filled_16 = 0;
}

void set_scenario_label(CWDetectScenarioLabel lab) noexcept { g2.cur_label = lab; }

void record_symbol_v2_from_fwht(const std::int32_t* fi, const std::int32_t* fq,
                                int nc) noexcept {
    if (fi == nullptr || fq == nullptr || nc <= 0) {
        return;
    }
    const int ncl = (nc < 64) ? nc : 64;
    std::int64_t energies[64];
    for (int i = 0; i < 64; ++i) {
        energies[static_cast<std::size_t>(i)] = 0;
    }
    std::int64_t total = 0;
    for (int i = 0; i < ncl; ++i) {
        const std::int64_t vfi =
            static_cast<std::int64_t>(fi[static_cast<std::size_t>(i)]);
        const std::int64_t vfq =
            static_cast<std::int64_t>(fq[static_cast<std::size_t>(i)]);
        const std::int64_t er = vfi * vfi + vfq * vfq;
        energies[static_cast<std::size_t>(i)] = er;
        total += er;
    }
    if (total <= 0) {
        return;
    }

    std::int64_t sorted[64];
    std::memcpy(sorted, energies, sizeof(sorted));
    std::sort(sorted, sorted + 64, std::greater<std::int64_t>());

    const int k = (4 < ncl) ? 4 : ncl;
    std::int64_t topk = 0;
    for (int i = 0; i < k; ++i) {
        topk += sorted[static_cast<std::size_t>(i)];
    }
    const double conc = static_cast<double>(topk) / static_cast<double>(total);
    const double H = shannon_entropy_bits(energies, ncl, total);

    ++g2.total_symbols_v2;
    const int li = static_cast<int>(g2.cur_label);
    ++g2.label_sym_total[static_cast<std::size_t>(li)];

    const int cb = conc_bin_v2(conc);
    const int eb = ent_bin_v2(H);
    ++g2.joint_hist_per_sym[static_cast<std::size_t>(cb)][static_cast<std::size_t>(eb)];
    ++g2.joint_by_label[static_cast<std::size_t>(li)][static_cast<std::size_t>(cb)]
                       [static_cast<std::size_t>(eb)];

    push_window_joint(g2.joint_hist_win4, 4, g2.recent_conc_4, g2.recent_ent_4, g2.idx_4,
                      g2.filled_4, conc, H);
    push_window_joint(g2.joint_hist_win8, 8, g2.recent_conc_8, g2.recent_ent_8, g2.idx_8,
                      g2.filled_8, conc, H);
    push_window_joint(g2.joint_hist_win16, 16, g2.recent_conc_16, g2.recent_ent_16, g2.idx_16,
                      g2.filled_16, conc, H);

    for (int b = 0; b < CW2D_BAND_CANDIDATES; ++b) {
        if (in_band_v2(b, conc, H)) {
            ++g2.band_hits[static_cast<std::size_t>(b)];
            ++g2.cur_pkt_band_count[static_cast<std::size_t>(b)];
            ++g2.label_band_sym_hits[static_cast<std::size_t>(li)][static_cast<std::size_t>(b)];
        }
    }
    ++g2.cur_pkt_sym_count;
}

void mark_packet_boundary_v2() noexcept {
    if (g2.cur_pkt_sym_count == 0u) {
        return;
    }
    ++g2.pkt_total;
    const std::uint32_t half = g2.cur_pkt_sym_count / 2u;
    const int li = static_cast<int>(g2.cur_label);
    ++g2.label_pkt_total[static_cast<std::size_t>(li)];

    for (int b = 0; b < CW2D_BAND_CANDIDATES; ++b) {
        if (g2.cur_pkt_band_count[static_cast<std::size_t>(b)] >= half) {
            ++g2.pkt_in_band[static_cast<std::size_t>(b)];
            ++g2.label_pkt_band_hits[static_cast<std::size_t>(li)][static_cast<std::size_t>(b)];
        }
        g2.cur_pkt_band_count[static_cast<std::size_t>(b)] = 0u;
    }
    g2.cur_pkt_sym_count = 0u;
}

void print_stats_2d(const char* label) noexcept {
    std::fprintf(stderr, "\n=== CW Detect 2D [%s] ===\n", label ? label : "ALL");
    std::fprintf(stderr, "  total_symbols=%llu pkt_total=%llu\n",
                 static_cast<unsigned long long>(g2.total_symbols_v2),
                 static_cast<unsigned long long>(g2.pkt_total));
    if (g2.total_symbols_v2 == 0u) {
        std::fprintf(stderr, "  (no data)\n=== end CW Detect 2D ===\n\n");
        return;
    }

    std::fprintf(stderr, "\n--- Joint Histogram Per-Symbol (CSV) ---\n");
    std::fprintf(stderr, "# conc_bin rows 0..19 (0.05 step), ent_bin cols 0..23 (0.25 bit)\n");
    std::fprintf(stderr, "conc_bin,");
    for (int e = 0; e < CW2D_ENT_BINS; ++e) {
        std::fprintf(stderr, "e%d,", e);
    }
    std::fprintf(stderr, "\n");
    for (int c = 0; c < CW2D_CONC_BINS; ++c) {
        std::fprintf(stderr, "%d,", c);
        for (int e = 0; e < CW2D_ENT_BINS; ++e) {
            std::fprintf(stderr, "%llu,",
                         static_cast<unsigned long long>(
                             g2.joint_hist_per_sym[static_cast<std::size_t>(c)]
                                                  [static_cast<std::size_t>(e)]));
        }
        std::fprintf(stderr, "\n");
    }

    const std::uint64_t w4 =
        count_joint_region(g2.joint_hist_win4, 0.85, 1.0, 1.5, 2.5);
    const std::uint64_t w8 =
        count_joint_region(g2.joint_hist_win8, 0.85, 1.0, 1.5, 2.5);
    const std::uint64_t w16 =
        count_joint_region(g2.joint_hist_win16, 0.85, 1.0, 1.5, 2.5);
    std::fprintf(stderr, "\n--- Window CW-medium region hits (conc 0.85-1, ent 1.5-2.5) ---\n");
    std::fprintf(stderr, "  win4:  %llu\n", static_cast<unsigned long long>(w4));
    std::fprintf(stderr, "  win8:  %llu\n", static_cast<unsigned long long>(w8));
    std::fprintf(stderr, "  win16: %llu\n", static_cast<unsigned long long>(w16));

    std::fprintf(stderr, "\n--- Band candidate hit rates (sym / pkt) ---\n");
    for (int b = 0; b < CW2D_BAND_CANDIDATES; ++b) {
        const double sr =
            static_cast<double>(g2.band_hits[static_cast<std::size_t>(b)]) /
            static_cast<double>((g2.total_symbols_v2 > 0u) ? g2.total_symbols_v2 : 1u);
        const double pr =
            (g2.pkt_total > 0u)
                ? static_cast<double>(g2.pkt_in_band[static_cast<std::size_t>(b)]) /
                      static_cast<double>(g2.pkt_total)
                : 0.0;
        std::fprintf(stderr, "  %-12s sym=%.6f pkt=%.6f\n", kBandCandidatesV2[b].name, sr, pr);
    }
    std::fprintf(stderr, "=== end CW Detect 2D ===\n\n");
}

void print_joint_by_label_summary() noexcept {
    std::fprintf(stderr, "\n=== CW Detect 2D Label Summary (cumulative) ===\n");
    static const char* kLabName[] = {"CLEAN", "BARRAGE", "CW", "LOWSNR", "MIXED"};
    for (int L = 0; L < static_cast<int>(CWDetectScenarioLabel::COUNT); ++L) {
        const std::uint64_t tot = g2.label_sym_total[static_cast<std::size_t>(L)];
        if (tot == 0u) {
            std::fprintf(stderr, "  [%s] (no symbols)\n", kLabName[L]);
            continue;
        }
        const std::uint64_t cw_med = count_joint_region(
            g2.joint_by_label[static_cast<std::size_t>(L)], 0.85, 1.0, 1.5, 2.5);
        const double frac_cw_med = static_cast<double>(cw_med) / static_cast<double>(tot);
        std::fprintf(stderr, "  [%s] symbols=%llu frac_in_CW_medium_2D=%.6f\n",
                     kLabName[L], static_cast<unsigned long long>(tot), frac_cw_med);
    }

    std::fprintf(stderr, "\n--- Band vs label (symbol rate) ---\n");
    std::fprintf(stderr, "band          CLEAN    BARRAGE  CW       LOWSNR   MIXED\n");
    for (int b = 0; b < CW2D_BAND_CANDIDATES; ++b) {
        std::fprintf(stderr, "%-14s", kBandCandidatesV2[b].name);
        for (int L = 0; L < static_cast<int>(CWDetectScenarioLabel::COUNT); ++L) {
            const std::uint64_t t = g2.label_sym_total[static_cast<std::size_t>(L)];
            const std::uint64_t h = g2.label_band_sym_hits[static_cast<std::size_t>(L)]
                                                         [static_cast<std::size_t>(b)];
            const double r = (t > 0u) ? 100.0 * static_cast<double>(h) / static_cast<double>(t) : 0.0;
            std::fprintf(stderr, " %7.3f%%", r);
        }
        std::fprintf(stderr, "\n");
    }

    std::fprintf(stderr, "\n--- CW-band TP vs max non-CW FP (symbol %% ) ---\n");
    for (int b = 0; b < CW2D_BAND_CANDIDATES; ++b) {
        const std::uint64_t t_cw = g2.label_sym_total[static_cast<std::size_t>(
            static_cast<int>(CWDetectScenarioLabel::Cw))];
        const std::uint64_t h_cw =
            (t_cw > 0u)
                ? g2.label_band_sym_hits[static_cast<std::size_t>(
                      static_cast<int>(CWDetectScenarioLabel::Cw))][static_cast<std::size_t>(b)]
                : 0u;
        const double tp = (t_cw > 0u) ? 100.0 * static_cast<double>(h_cw) / static_cast<double>(t_cw) : 0.0;
        double max_fp = 0.0;
        for (int L = 0; L < static_cast<int>(CWDetectScenarioLabel::COUNT); ++L) {
            if (L == static_cast<int>(CWDetectScenarioLabel::Cw)) {
                continue;
            }
            const std::uint64_t t = g2.label_sym_total[static_cast<std::size_t>(L)];
            const std::uint64_t h = g2.label_band_sym_hits[static_cast<std::size_t>(L)]
                                                         [static_cast<std::size_t>(b)];
            const double fp = (t > 0u) ? 100.0 * static_cast<double>(h) / static_cast<double>(t) : 0.0;
            if (fp > max_fp) {
                max_fp = fp;
            }
        }
        std::fprintf(stderr, "  %-14s TP=%6.3f%% max_nonCW_FP=%6.3f%% gap=%6.3f%%\n",
                     kBandCandidatesV2[b].name, tp, max_fp, tp - max_fp);
    }

    std::fprintf(stderr, "\n--- Packet >=50%% symbols in band (%% of packets) ---\n");
    for (int b = 0; b < CW2D_BAND_CANDIDATES; ++b) {
        std::fprintf(stderr, "%-14s", kBandCandidatesV2[b].name);
        for (int L = 0; L < static_cast<int>(CWDetectScenarioLabel::COUNT); ++L) {
            const std::uint64_t pt = g2.label_pkt_total[static_cast<std::size_t>(L)];
            const std::uint64_t ph = g2.label_pkt_band_hits[static_cast<std::size_t>(L)]
                                                         [static_cast<std::size_t>(b)];
            const double r = (pt > 0u) ? 100.0 * static_cast<double>(ph) / static_cast<double>(pt) : 0.0;
            std::fprintf(stderr, " %7.3f%%", r);
        }
        std::fprintf(stderr, "\n");
    }
    std::fprintf(stderr, "=== end CW Detect 2D Label Summary ===\n\n");
}

#endif // HTS_CW_DETECT_DIAG_V2

} // namespace CWDetectDiag
} // namespace ProtectedEngine
