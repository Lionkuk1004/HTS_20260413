#ifndef HTS_ROW_CONSISTENCY_DIAG_HPP
#define HTS_ROW_CONSISTENCY_DIAG_HPP

#include <cstdint>

namespace ProtectedEngine {
namespace RowConsistencyDiag {

struct ConsistencyStats {
    uint64_t top_row_hist[64];

    uint64_t in_allowed_count;
    uint64_t out_allowed_count;

    static constexpr int RATIO_BINS = 11;
    uint64_t ratio_hist[RATIO_BINS];

    uint64_t jump_hist[8];

    int32_t last_top_row;

    uint64_t total_symbols;
};

ConsistencyStats& get_stats() noexcept;
void reset_stats() noexcept;

void record_symbol(int top1, int top2, int64_t top1_energy, int64_t top2_energy,
                   bool expected_in_allowed) noexcept;

void print_stats(const char* label) noexcept;

double compute_entropy() noexcept;
double compute_concentration() noexcept;
double compute_allowed_ratio() noexcept;

} // namespace RowConsistencyDiag
} // namespace ProtectedEngine

#endif
