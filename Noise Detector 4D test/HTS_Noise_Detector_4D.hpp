// ============================================================================
// HTS_Noise_Detector_4D.hpp — v2 (2026-04-28)
// ============================================================================
#ifndef HTS_NOISE_DETECTOR_4D_HPP
#define HTS_NOISE_DETECTOR_4D_HPP
#include <cstdint>
namespace ProtectedEngine {
namespace NoiseDetect4D {
constexpr uint32_t kSeedDim = 64;
constexpr uint32_t kPhaseDim = 16;
constexpr uint32_t kTimeDim = 14;
constexpr uint32_t kCheckDim = 4;
struct GridCell {
    int32_t mag_q15;
    int32_t I_q15;
    int32_t Q_q15;
    uint32_t classify_tag;
};
struct DetectResult4D {
    uint32_t signal_seed_idx;
    uint32_t signal_phase_idx;
    uint32_t signal_time_idx;
    int64_t noise_total_q15;
    uint32_t converged;
    uint32_t iter_count;
};
constexpr int32_t kFaceA_NoiseFloor_Q15 = 1024;
constexpr int32_t kFaceA_SignalThr_Q15 = 8192;
constexpr int32_t kFaceA_CWSpike_Q15 = 32768;
constexpr int32_t kFaceB_PhaseStability = 256;
constexpr int32_t kFaceC_DeleteThr_Q15 = 2048;
constexpr uint32_t kFaceD_MaxIter = 4;
constexpr int32_t kFaceD_ConvergeThr_Q15 = 128;
inline int64_t pte_sumif_q15(const int32_t *data, uint32_t length,
                             int32_t threshold) noexcept {
    if (data == nullptr || length == 0)
        return 0;
    int64_t sum = 0;
    for (uint32_t i = 0; i < length; ++i) {
        const int32_t val = data[i];
        const int64_t diff =
            static_cast<int64_t>(val) - static_cast<int64_t>(threshold);
        const int32_t mask = static_cast<int32_t>(diff >> 63);
        sum += static_cast<int64_t>(val & ~mask);
    }
    return sum;
}
inline int32_t mag_v15_q15(int32_t I, int32_t Q) noexcept {
    const int32_t I_sign = I >> 31;
    const int32_t Q_sign = Q >> 31;
    const int32_t abs_I = (I ^ I_sign) - I_sign;
    const int32_t abs_Q = (Q ^ Q_sign) - Q_sign;
    const int64_t diff =
        static_cast<int64_t>(abs_I) - static_cast<int64_t>(abs_Q);
    const int32_t mask = static_cast<int32_t>(diff >> 63);
    const int32_t max_v = (abs_I & ~mask) | (abs_Q & mask);
    const int32_t min_v = (abs_I & mask) | (abs_Q & ~mask);
    return max_v + (min_v >> 1);
}
struct ArgmaxResult {
    int32_t best_val;
    uint32_t best_idx;
};
inline ArgmaxResult dpte_argmax(const int32_t *data, uint32_t length) noexcept {
    if (data == nullptr || length == 0)
        return ArgmaxResult{0, 0};
    int32_t best_val = data[0];
    int32_t best_idx = 0;
    for (uint32_t i = 1; i < length; ++i) {
        const int32_t val = data[i];
        const int64_t diff =
            static_cast<int64_t>(val) - static_cast<int64_t>(best_val);
        const int32_t mask = static_cast<int32_t>(diff >> 63);
        best_val = (best_val & mask) | (val & ~mask);
        best_idx = (best_idx & mask) | (static_cast<int32_t>(i) & ~mask);
    }
    return ArgmaxResult{best_val, static_cast<uint32_t>(best_idx)};
}
// 통합 진입점 (Cursor 최적화 cpp 의 단일 함수)
DetectResult4D
detect_noise_4d(GridCell tensor_4d[kSeedDim][kPhaseDim][kTimeDim],
                const int32_t *pn_mask, int32_t delete_threshold_q15,
                int32_t residual_threshold_q15) noexcept;
} // namespace NoiseDetect4D
} // namespace ProtectedEngine
#endif
