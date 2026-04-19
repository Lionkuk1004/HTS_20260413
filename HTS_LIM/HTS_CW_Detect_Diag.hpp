#ifndef HTS_CW_DETECT_DIAG_HPP
#define HTS_CW_DETECT_DIAG_HPP

#include <cstdint>

namespace ProtectedEngine {
namespace CWDetectDiag {

/// Phase 1: FWHT row-energy distribution (per-symbol, sliding window, per-packet).
/// No decode-path decisions — observation only.
struct CWDetectStats {
    static constexpr int RATIO_BINS_FINE = 20;
    static constexpr int CONC_BINS = 20;
    static constexpr int WINDOW_SIZE = 8;
    static constexpr int WIN_CONC_BINS = 20;
    static constexpr int WIN_ENT_BINS = 20;

    std::uint64_t per_sym_t2t1_hist[RATIO_BINS_FINE];
    std::uint64_t per_sym_conc_hist[CONC_BINS];
    std::uint64_t win_conc_hist[WIN_CONC_BINS];
    std::uint64_t win_entropy_hist[WIN_ENT_BINS];

    /// Packet mean concentration vs thresholds (Clopper-style reporting in print).
    static constexpr int PKT_TH_N = 5;
    std::uint64_t pkt_conc_high_clean[PKT_TH_N];
    std::uint64_t pkt_conc_high_cw[PKT_TH_N];
    std::uint64_t pkt_entropy_low_clean[PKT_TH_N];
    std::uint64_t pkt_entropy_low_cw[PKT_TH_N];
    std::uint64_t pkt_total_clean;
    std::uint64_t pkt_total_cw;

    std::uint64_t total_symbols;

    double recent_concentration[WINDOW_SIZE];
    double recent_entropy[WINDOW_SIZE];
    int buffer_idx;
    int win_filled;
};

CWDetectStats& get_stats() noexcept;
void reset_stats() noexcept;

/// Current trial/scenario label: true = non-CW reference (S1/S3/S7), false = CW-heavy (S8/S9).
void set_expect_non_cw(bool non_cw) noexcept;

/// FWHT + unshift domain: first min(nc,64) bins carry energy; rest zero.
void record_symbol_from_fwht(const std::int32_t* fi, const std::int32_t* fq,
                             int nc) noexcept;

/// One packet decode attempt finished (T6 harness: end of feed_raw_ext).
void mark_packet_boundary() noexcept;

void print_stats(const char* label) noexcept;

#if defined(HTS_CW_DETECT_DIAG_V2)

/// Phase 2: 2D joint (concentration × entropy) + multi-window + band sweep.
enum class CWDetectScenarioLabel : std::uint8_t {
    Clean = 0,
    Barrage = 1,
    Cw = 2,
    LowSnr = 3,
    Mixed = 4,
    COUNT = 5
};

struct BandCandidate {
    const char* name;
    double conc_min;
    double conc_max;
    double ent_min;
    double ent_max;
};

static constexpr int CW2D_CONC_BINS = 20;
static constexpr int CW2D_ENT_BINS = 24;
static constexpr int CW2D_BAND_CANDIDATES = 8;

void reset_joint_label_accum() noexcept;
void reset_stats_v2() noexcept;
void set_scenario_label(CWDetectScenarioLabel lab) noexcept;

void record_symbol_v2_from_fwht(const std::int32_t* fi, const std::int32_t* fq,
                                int nc) noexcept;
void mark_packet_boundary_v2() noexcept;

void print_stats_2d(const char* label) noexcept;
void print_joint_by_label_summary() noexcept;

#endif // HTS_CW_DETECT_DIAG_V2

} // namespace CWDetectDiag
} // namespace ProtectedEngine

#endif
