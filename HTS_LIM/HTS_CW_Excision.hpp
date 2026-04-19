#ifndef HTS_CW_EXCISION_HPP
#define HTS_CW_EXCISION_HPP

#include <cstdint>

namespace ProtectedEngine {
namespace CWExcision {

/// Phase 2 fixed detector (do not change without re-measuring MC / T6).
constexpr double CONC_TH_LO = 0.85;
constexpr double CONC_TH_HI = 1.00;
constexpr double ENT_LOW_BIT = 1.50;
constexpr double ENT_HIGH_BIT = 2.50;

enum class Mode : int {
    OFF = 0,
    HARD_NULL = 1,
    SOFT_ATTEN = 2,
    MEDIAN_SUB = 3,
};

struct Options {
    Mode mode = Mode::OFF;
    int excise_top_k = 2;
    double atten_factor = 0.1;
};

/// Build-time defaults: `HTS_CW_EX_MODE` 1=HARD_NULL, 2=SOFT_ATTEN, 3=MEDIAN_SUB (if unset → 1).
[[nodiscard]] Options default_build_options() noexcept;

[[nodiscard]] bool detect_only(const std::int32_t* fI, const std::int32_t* fQ, int nc) noexcept;

/// In-place FWHT row excision when detector fires and `opt.mode != OFF`.
[[nodiscard]] bool detect_and_excise(std::int32_t* fI, std::int32_t* fQ, int nc,
                                       const Options& opt) noexcept;

#if defined(HTS_CW_EX_DIAG)
struct ExStats {
    std::uint64_t total_symbols{};
    std::uint64_t detected_count{};
    std::uint64_t excised_count{};
    std::uint64_t excised_row_hits[64]{};
};

ExStats& get_stats() noexcept;
void reset_stats() noexcept;
void print_stats(const char* label) noexcept;
#endif

} // namespace CWExcision
} // namespace ProtectedEngine

#endif
