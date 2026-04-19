#ifndef HTS_LLR_DIAG_HPP
#define HTS_LLR_DIAG_HPP

#include <cstdint>

namespace ProtectedEngine {
namespace LLRDiag {

enum class ObservePoint : int {
    FWHT_OUT = 0,
    BIN_TO_LLR = 1,
    IR_ACCUM = 2,
    POLAR_PM = 3,
    COUNT = 4
};

struct PointStats {
    static constexpr int HIST_BINS = 12;
    uint64_t mag_hist[HIST_BINS];

    uint64_t clamp_hits;

    uint64_t pos_count;
    uint64_t neg_count;
    uint64_t zero_count;

    uint64_t total_observations;
    double sum_abs;
    double sum_sq_abs;
    int64_t max_abs;
    int64_t min_abs_nonzero;

    uint64_t correct_sign_count;
};

struct LLRStats {
    PointStats points[static_cast<int>(ObservePoint::COUNT)];
};

LLRStats& get_llr_stats() noexcept;
void reset_llr_stats() noexcept;

void record_llr(ObservePoint pt, int32_t llr_val, int32_t clamp_limit) noexcept;

void record_llr_with_truth(ObservePoint pt, int32_t llr_val, int32_t clamp_limit,
                           int tx_bit) noexcept;

void print_llr_stats(const char* label) noexcept;

double compute_reliability(ObservePoint pt) noexcept;
double compute_consistency(ObservePoint pt) noexcept;

} // namespace LLRDiag
} // namespace ProtectedEngine

#endif
