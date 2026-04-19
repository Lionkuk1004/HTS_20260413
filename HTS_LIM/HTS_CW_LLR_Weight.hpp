#ifndef HTS_CW_LLR_WEIGHT_HPP
#define HTS_CW_LLR_WEIGHT_HPP

#include <cstdint>

#if defined(HTS_CW_LLR_WEIGHT_V1)
#ifndef HTS_CW_W_TOP_K
#define HTS_CW_W_TOP_K 2
#endif
#ifndef HTS_CW_W_FACTOR
#define HTS_CW_W_FACTOR 0.5
#endif
#endif

namespace ProtectedEngine {
namespace CWLLRWeight {

/// Phase 2 fixed rule (same as CW excision Phase 2 — do not change without re-measure).
constexpr double CONC_TH_LO = 0.85;
constexpr double CONC_TH_HI = 1.00;
constexpr double ENT_LOW_BIT = 1.50;
constexpr double ENT_HIGH_BIT = 2.50;

enum class Mode : int {
    OFF = 0,
    WEIGHT = 1,
};

struct Options {
    Mode mode = Mode::OFF;
    int cw_top_k = 2;
    double weight_factor = 0.5;
};

struct DetectionResult {
    bool is_cw{};
    int cw_rows[8]{};
    int cw_row_count{};
};

[[nodiscard]] DetectionResult detect_per_symbol(const std::int32_t* fI, const std::int32_t* fQ,
                                                int nc, int top_k) noexcept;

#if defined(HTS_CW_W_DIAG)
struct WeightStats {
    std::uint64_t total_symbols{};
    std::uint64_t cw_detected{};
    std::uint64_t weighted_rows_total{};
    std::uint64_t weight_hit_by_row[64]{};
};

WeightStats& get_stats() noexcept;
void reset_stats() noexcept;
void print_stats(const char* label) noexcept;
#endif

} // namespace CWLLRWeight
} // namespace ProtectedEngine

#endif
