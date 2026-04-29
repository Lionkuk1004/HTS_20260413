// ============================================================================
// HTS_Noise_Detector_4D.cpp — 4D 노이즈 검출기 구현 (1차 prototype)
// ============================================================================
#include "HTS_Noise_Detector_4D.hpp"
namespace ProtectedEngine {
namespace NoiseDetect4D {
// ============================================================================
// 면 1: 분류 (seed)
//   각 seed 격자점 magnitude → type tag 부여
//   - mag < kFaceA_NoiseFloor_Q15           : tag=0 (noise)
//   - kFaceA_SignalThr ≤ mag < kFaceA_CWSpike : tag=1 (signal)
//   - mag >= kFaceA_CWSpike_Q15             : tag=2 (cw)
//   - 나머지                                : tag=3 (undecided)
// ============================================================================
void face1_classify_seed(GridCell *grid_seed, uint32_t seed_count,
                         const int32_t *pn_mask) noexcept {
    if (grid_seed == nullptr || seed_count == 0)
        return;
    (void)pn_mask; // PNMasked 적용은 이미 mag_q15 계산 시 반영 가정
    for (uint32_t i = 0; i < seed_count; ++i) {
        const int32_t mag = grid_seed[i].mag_q15;
        // BPTE threshold mask (분기 X)
        const int64_t d_floor =
            static_cast<int64_t>(mag) - kFaceA_NoiseFloor_Q15;
        const int64_t d_signal =
            static_cast<int64_t>(mag) - kFaceA_SignalThr_Q15;
        const int64_t d_cw = static_cast<int64_t>(mag) - kFaceA_CWSpike_Q15;
        const int32_t m_floor =
            static_cast<int32_t>(d_floor >> 63); // mag < floor  → -1
        const int32_t m_signal =
            static_cast<int32_t>(d_signal >> 63); // mag < signal → -1
        const int32_t m_cw =
            static_cast<int32_t>(d_cw >> 63); // mag < cw     → -1
        // tag 결정 (BPTE 우선순위: cw > signal > noise > undecided)
        // mag >= cw_spike            → tag = 2
        // signal ≤ mag < cw_spike    → tag = 1
        // mag < floor                → tag = 0
        // 나머지 (floor ≤ mag < sig) → tag = 3 (undecided)
        const uint32_t is_cw = static_cast<uint32_t>(~m_cw) & 1u;
        const uint32_t is_signal = (static_cast<uint32_t>(~m_signal) & 1u) &
                                   static_cast<uint32_t>(m_cw & 1);
        const uint32_t is_noise = static_cast<uint32_t>(m_floor) & 1u;
        const uint32_t tag_cw = is_cw ? 2u : 0u;
        const uint32_t tag_signal = is_signal ? 1u : 0u;
        const uint32_t tag_noise = is_noise ? 0u : 3u;
        // 우선순위 적용 (BPTE)
        uint32_t tag;
        if (is_cw)
            tag = 2u;
        else if (is_signal)
            tag = 1u;
        else if (is_noise)
            tag = 0u;
        else
            tag = 3u;
        // (1차 prototype: 분기 사용. 양산 적용 시 BPTE mask 로 변환)
        grid_seed[i].classify_tag = tag;
    }
}
// ============================================================================
// 면 2: 검출 (위상)
//   각 위상 bin 의 안정성 검증
//   - 위상 magnitude 곡률 (1차/2차 미분)
//   - atan2_bpte 결과 분산 (placeholder)
//
//   1차 prototype: magnitude 만 사용 (위상 회전 검증 추후)
// ============================================================================
void face2_detect_phase(const GridCell *grid_seed, uint32_t seed_count,
                        GridCell *grid_phase, uint32_t phase_count) noexcept {
    if (grid_seed == nullptr || grid_phase == nullptr)
        return;
    if (seed_count == 0 || phase_count == 0)
        return;
    // 위상 bin 별 magnitude 곡률 검사
    for (uint32_t p = 1; p + 1 < phase_count; ++p) {
        const int32_t mag_l = grid_phase[p - 1].mag_q15;
        const int32_t mag_m = grid_phase[p].mag_q15;
        const int32_t mag_r = grid_phase[p + 1].mag_q15;
        // 2차 미분 (곡률, 영준님 자료)
        const int64_t curv = static_cast<int64_t>(mag_l) -
                             2 * static_cast<int64_t>(mag_m) +
                             static_cast<int64_t>(mag_r);
        // curv < 0: 정상 peak (∩)
        // curv ≥ 0: 평탄/spike (noise 또는 CW)
        const int32_t curv_mask = static_cast<int32_t>(curv >> 63);
        // curv_mask = -1 if curv < 0 (정상)
        // tag 갱신: 위상 안정 → signal 유지, 불안정 → noise
        const uint32_t prev_tag = grid_phase[p].classify_tag;
        if (curv_mask == 0 && prev_tag == 1u) {
            // signal 인데 위상 불안정 → undecided
            grid_phase[p].classify_tag = 3u;
        }
    }
    // (1차 prototype: atan2_bpte 위상 분산 검증 추가 예정)
}
// ============================================================================
// 면 3: 삭제 (시간)
//   PTE_Core_SumIf 패턴: threshold 이하 mag = 0 으로 삭제
// ============================================================================
void face3_delete_time(GridCell *grid_time, uint32_t time_count,
                       int32_t delete_threshold_q15) noexcept {
    if (grid_time == nullptr || time_count == 0)
        return;
    for (uint32_t t = 0; t < time_count; ++t) {
        const int32_t mag = grid_time[t].mag_q15;
        // BPTE: mag < threshold → 삭제 (mask 0)
        const int64_t diff = static_cast<int64_t>(mag) -
                             static_cast<int64_t>(delete_threshold_q15);
        const int32_t mask = static_cast<int32_t>(diff >> 63);
        // mask = -1 if mag < threshold (삭제 대상)
        // mag = mag & ~mask (mag < thr → 0)
        grid_time[t].mag_q15 = mag & ~mask;
        grid_time[t].I_q15 = grid_time[t].I_q15 & ~mask;
        grid_time[t].Q_q15 = grid_time[t].Q_q15 & ~mask;
        // tag 갱신: 삭제된 cell → noise
        if (mask != 0) {
            grid_time[t].classify_tag = 0u;
        }
    }
}
// ============================================================================
// 면 4: 재검증 (4D 전체)
//   CS-MP-DPTE: 4D 텐서 전체에서 신호 1개 격자점 + 나머지 noise 인지 확인
//   수렴까지 반복
// ============================================================================
DetectResult4D
face4_recheck_all(GridCell tensor_4d[kSeedDim][kPhaseDim][kTimeDim],
                  int32_t residual_threshold_q15) noexcept {
    DetectResult4D result{};
    result.signal_seed_idx = 0;
    result.signal_phase_idx = 0;
    result.signal_time_idx = 0;
    result.noise_total_q15 = 0;
    result.converged = 0;
    result.iter_count = 0;
    int64_t prev_noise_total = -1;
    for (uint32_t iter = 0; iter < kFaceD_MaxIter; ++iter) {
        // 1. 4D 전체 max (DPTE argmax)
        int32_t global_max = 0;
        uint32_t max_s = 0, max_p = 0, max_t = 0;
        for (uint32_t s = 0; s < kSeedDim; ++s) {
            for (uint32_t p = 0; p < kPhaseDim; ++p) {
                for (uint32_t t = 0; t < kTimeDim; ++t) {
                    const int32_t mag = tensor_4d[s][p][t].mag_q15;
                    const int64_t diff = static_cast<int64_t>(mag) -
                                         static_cast<int64_t>(global_max);
                    const int32_t mask = static_cast<int32_t>(diff >> 63);
                    // mag > max 면 mask=0, 갱신
                    if (mask == 0) {
                        global_max = mag;
                        max_s = s;
                        max_p = p;
                        max_t = t;
                    }
                    // (1차 prototype: 분기 사용)
                }
            }
        }
        // 2. 신호 위치 기록
        result.signal_seed_idx = max_s;
        result.signal_phase_idx = max_p;
        result.signal_time_idx = max_t;
        // 3. 잔차 = 신호 외 모든 cell magnitude 합 (PTE_SumIf)
        int64_t noise_sum = 0;
        for (uint32_t s = 0; s < kSeedDim; ++s) {
            for (uint32_t p = 0; p < kPhaseDim; ++p) {
                for (uint32_t t = 0; t < kTimeDim; ++t) {
                    if (s == max_s && p == max_p && t == max_t)
                        continue;
                    const int32_t mag = tensor_4d[s][p][t].mag_q15;
                    const int64_t diff =
                        static_cast<int64_t>(mag) -
                        static_cast<int64_t>(residual_threshold_q15);
                    const int32_t mask = static_cast<int32_t>(diff >> 63);
                    noise_sum += static_cast<int64_t>(mag & ~mask);
                }
            }
        }
        result.noise_total_q15 = noise_sum;
        result.iter_count = iter + 1;
        // 4. 수렴 판정
        if (prev_noise_total >= 0) {
            const int64_t change = noise_sum - prev_noise_total;
            const int64_t abs_change = (change >= 0) ? change : -change;
            if (abs_change < kFaceD_ConvergeThr_Q15) {
                result.converged = 1u;
                break;
            }
        }
        prev_noise_total = noise_sum;
        // 5. threshold 이상 noise cell 추가 삭제 (반복 정제)
        for (uint32_t s = 0; s < kSeedDim; ++s) {
            for (uint32_t p = 0; p < kPhaseDim; ++p) {
                for (uint32_t t = 0; t < kTimeDim; ++t) {
                    if (s == max_s && p == max_p && t == max_t)
                        continue;
                    const int32_t mag = tensor_4d[s][p][t].mag_q15;
                    const int64_t diff =
                        static_cast<int64_t>(mag) -
                        static_cast<int64_t>(residual_threshold_q15);
                    const int32_t mask = static_cast<int32_t>(diff >> 63);
                    // mag > thr → 삭제 (다음 iter 잔차 줄이기)
                    tensor_4d[s][p][t].mag_q15 = mag & mask;
                }
            }
        }
    }
    return result;
}
// ============================================================================
// 통합 4D 노이즈 검출
// ============================================================================
DetectResult4D
detect_noise_4d(GridCell tensor_4d[kSeedDim][kPhaseDim][kTimeDim],
                const int32_t *pn_mask, int32_t delete_threshold_q15,
                int32_t residual_threshold_q15) noexcept {
    DetectResult4D result{};
    if (tensor_4d == nullptr)
        return result;
    // 면 1: seed 차원 따라 분류
    for (uint32_t p = 0; p < kPhaseDim; ++p) {
        for (uint32_t t = 0; t < kTimeDim; ++t) {
            // seed line 추출
            GridCell seed_line[kSeedDim];
            for (uint32_t s = 0; s < kSeedDim; ++s) {
                seed_line[s] = tensor_4d[s][p][t];
            }
            face1_classify_seed(seed_line, kSeedDim, pn_mask);
            for (uint32_t s = 0; s < kSeedDim; ++s) {
                tensor_4d[s][p][t] = seed_line[s];
            }
        }
    }
    // 면 2: 위상 차원 따라 검출
    for (uint32_t s = 0; s < kSeedDim; ++s) {
        for (uint32_t t = 0; t < kTimeDim; ++t) {
            GridCell phase_line[kPhaseDim];
            for (uint32_t p = 0; p < kPhaseDim; ++p) {
                phase_line[p] = tensor_4d[s][p][t];
            }
            face2_detect_phase(phase_line, kPhaseDim, phase_line, kPhaseDim);
            for (uint32_t p = 0; p < kPhaseDim; ++p) {
                tensor_4d[s][p][t] = phase_line[p];
            }
        }
    }
    // 면 3: 시간 차원 따라 삭제
    for (uint32_t s = 0; s < kSeedDim; ++s) {
        for (uint32_t p = 0; p < kPhaseDim; ++p) {
            GridCell time_line[kTimeDim];
            for (uint32_t t = 0; t < kTimeDim; ++t) {
                time_line[t] = tensor_4d[s][p][t];
            }
            face3_delete_time(time_line, kTimeDim, delete_threshold_q15);
            for (uint32_t t = 0; t < kTimeDim; ++t) {
                tensor_4d[s][p][t] = time_line[t];
            }
        }
    }
    // 면 4: 4D 전체 재검증
    result = face4_recheck_all(tensor_4d, residual_threshold_q15);
    return result;
}
} // namespace NoiseDetect4D
} // namespace ProtectedEngine
