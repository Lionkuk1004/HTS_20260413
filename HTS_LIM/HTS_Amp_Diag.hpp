#ifndef HTS_AMP_DIAG_HPP
#define HTS_AMP_DIAG_HPP

#include <cstdint>

namespace ProtectedEngine {
namespace AmpDiag {

struct AmpStats {
    static constexpr int NUM_BINS = 8;
    uint64_t hist_I[NUM_BINS]{};
    uint64_t hist_Q[NUM_BINS]{};

    uint64_t total_samples{0};
    uint64_t saturated_count{0};
    uint64_t near_sat_count{0};

    uint64_t sum_sq_I{0};
    uint64_t sum_sq_Q{0};

    int32_t max_abs_I{0};
    int32_t max_abs_Q{0};
};

AmpStats &get_amp_stats() noexcept;
void reset_amp_stats() noexcept;
void record_chip(int16_t I, int16_t Q) noexcept;
void print_amp_stats() noexcept;

} // namespace AmpDiag
} // namespace ProtectedEngine

#endif
