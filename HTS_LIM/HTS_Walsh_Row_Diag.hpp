// =============================================================================
// HTS_Walsh_Row_Diag.hpp — FWHT row별 에너지 분포 계측 (HTS_WALSH_ROW_DIAG 전용)
// =============================================================================
#ifndef HTS_WALSH_ROW_DIAG_HPP
#define HTS_WALSH_ROW_DIAG_HPP

#include <cstdint>

namespace ProtectedEngine {
namespace WalshRowDiag {

struct RowStats {
    // 64 row 각각의 에너지 누적 (I²+Q²)
    uint64_t row_energy_sum[64];

    // 각 row 가 "top1" 으로 선정된 횟수
    uint64_t row_top1_count[64];

    // 각 row 가 "top2" 로 선정된 횟수
    uint64_t row_top2_count[64];

    // 집계
    uint64_t total_observations;

    // Top2/Top1 비율 히스토그램 (0.0, 0.1, 0.2, ..., 1.0)
    static constexpr int RATIO_BINS = 11;
    uint64_t ratio_hist[RATIO_BINS];

    // 최고/최저 비율
    double max_ratio; // 가장 선명한 peak
    double min_ratio; // 가장 뭉개진 peak
};

RowStats& get_row_stats() noexcept;
void reset_row_stats() noexcept;

// 관찰 기록 (FWHT 64 bin energy 배열 전달)
void record_row_energies(const int64_t energies[64]) noexcept;

void print_row_stats(const char* label) noexcept;

} // namespace WalshRowDiag
} // namespace ProtectedEngine

#endif
