#include "HTS_HARQ_Diag.hpp"
#include <cstdio>
#include <cstring>

namespace ProtectedEngine {
namespace HARQ_Diag {

static RetryStats g_stats{};

RetryStats &get_retry_stats() noexcept {
    return g_stats;
}

void reset_retry_stats() noexcept {
    std::memset(static_cast<void *>(&g_stats), 0, sizeof(g_stats));
}

void note_packet_finished(uint32_t rounds_used, uint32_t max_limit,
                          uint32_t decode_ok) noexcept {
    g_stats.max_harq_limit = max_limit;
    const uint16_t clipped =
        (rounds_used > 65535u) ? 65535u : static_cast<uint16_t>(rounds_used);
    if (g_stats.history_idx < RetryStats::HISTORY_SIZE) {
        g_stats.history[g_stats.history_idx] = clipped;
        ++g_stats.history_idx;
    }
    g_stats.total_retries += static_cast<uint64_t>(rounds_used);
    ++g_stats.total_trials;
    if (rounds_used > g_stats.max_retry_observed) {
        g_stats.max_retry_observed = rounds_used;
    }
    if (max_limit > 0u && decode_ok == 0u &&
        rounds_used >= max_limit) {
        ++g_stats.trials_hit_limit;
    }
}

void print_retry_stats() noexcept {
    const RetryStats &s = g_stats;
    const double avg =
        (s.total_trials > 0u)
            ? static_cast<double>(s.total_retries) /
                  static_cast<double>(s.total_trials)
            : 0.0;
    const double pct =
        (s.total_trials > 0u)
            ? 100.0 * static_cast<double>(s.trials_hit_limit) /
                  static_cast<double>(s.total_trials)
            : 0.0;
    std::fprintf(stderr,
                 "\n=== HARQ RETRY STATS ===\n"
                 "  max_harq_limit         = %u\n"
                 "  total_trials           = %u\n"
                 "  total_retries          = %llu\n"
                 "  avg retries per trial  = %.2f\n"
                 "  max observed           = %u\n"
                 "  trials hit limit       = %u  (%.1f%%)\n"
                 "========================\n",
                 s.max_harq_limit, s.total_trials,
                 static_cast<unsigned long long>(s.total_retries), avg,
                 s.max_retry_observed, s.trials_hit_limit, pct);

    uint32_t bins[7] = {0, 0, 0, 0, 0, 0, 0};
    for (uint32_t i = 0; i < s.history_idx; ++i) {
        const uint32_t v = s.history[i];
        if (v == 0u) {
            ++bins[0];
        } else if (v <= 5u) {
            ++bins[1];
        } else if (v <= 10u) {
            ++bins[2];
        } else if (v <= 20u) {
            ++bins[3];
        } else if (v <= 50u) {
            ++bins[4];
        } else if (v <= 100u) {
            ++bins[5];
        } else {
            ++bins[6];
        }
    }
    std::fprintf(stderr,
                 "\n--- 분포 ---\n"
                 "  0 retry        : %u\n"
                 "  1-5 retry      : %u\n"
                 "  6-10 retry     : %u\n"
                 "  11-20 retry    : %u\n"
                 "  21-50 retry    : %u\n"
                 "  51-100 retry   : %u\n"
                 "  100+ retry     : %u\n",
                 bins[0], bins[1], bins[2], bins[3], bins[4], bins[5],
                 bins[6]);
}

} // namespace HARQ_Diag
} // namespace ProtectedEngine
