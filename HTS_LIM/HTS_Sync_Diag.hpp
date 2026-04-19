// =============================================================================
// HTS_Sync_Diag.hpp — Preamble phase0_scan_ 동기 정밀 계측 (HTS_SYNC_DIAG 전용)
// =============================================================================
#ifndef HTS_SYNC_DIAG_HPP
#define HTS_SYNC_DIAG_HPP

#include <cstdint>

namespace ProtectedEngine {
namespace SyncDiag {

struct SyncStats {
    // best_off 히스토그램 (64 bin)
    uint64_t best_off_hist[64];

    // e63 절대값 분포 (로그 스케일 bin)
    // bin 0: 0~1K, 1: 1K~10K, 2: 10K~100K, 3: 100K~1M, 4: 1M~10M, 5: 10M~100M, 6: 100M+
    static constexpr int E63_BINS = 7;
    uint64_t best_e63_hist[E63_BINS];

    // second/best 비율 bin (낮을수록 동기 품질 좋음)
    // 0: 0~0.1, 1: 0.1~0.25, 2: 0.25~0.5, 3: 0.5~0.75, 4: 0.75~1.0
    static constexpr int RATIO_BINS = 5;
    uint64_t sec_ratio_hist[RATIO_BINS];

    // 집계
    uint64_t total_scans;
    uint64_t passed_scans; // pass == true
    uint64_t failed_scans;  // pass == false (memcpy shift)

    // best_off 통계
    int64_t sum_best_off; // 평균 계산용
    uint64_t best_off_min; // 최소값
    uint64_t best_off_max; // 최대값

    // e63 통계
    int64_t max_e63_observed;
    int64_t min_e63_observed_passed; // pass 한 scan 중 최소
    int64_t sum_e63_passed;
};

SyncStats& get_sync_stats() noexcept;
void reset_sync_stats() noexcept;

// phase0_scan_ 끝에 호출 (pass 판정 직후)
void record_scan(int best_off, int64_t best_e63, int64_t second_e63,
                 bool pass) noexcept;

void print_sync_stats(const char* label) noexcept;

} // namespace SyncDiag
} // namespace ProtectedEngine

#endif
