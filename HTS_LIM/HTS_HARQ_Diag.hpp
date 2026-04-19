#ifndef HTS_HARQ_DIAG_HPP
#define HTS_HARQ_DIAG_HPP

#include <cstdint>

namespace ProtectedEngine {
namespace HARQ_Diag {

struct RetryStats {
    static constexpr int HISTORY_SIZE = 12000;
    uint16_t history[HISTORY_SIZE]{};
    uint32_t history_idx{0};

    uint64_t total_retries{0};
    uint32_t total_trials{0};
    uint32_t max_retry_observed{0};
    uint32_t trials_hit_limit{0};
    uint32_t max_harq_limit{0};
};

RetryStats &get_retry_stats() noexcept;
void reset_retry_stats() noexcept;

/// PDU 한 건 종료 시 호출: `rounds_used` = 해당 PDU에 대해 수행된 try_decode 라운드 수(코어의 harq_round_),
/// `max_limit` = max_harq_, `decode_ok` = 성공(비0) 여부.
void note_packet_finished(uint32_t rounds_used, uint32_t max_limit,
                          uint32_t decode_ok) noexcept;

void print_retry_stats() noexcept;

} // namespace HARQ_Diag
} // namespace ProtectedEngine

#endif
