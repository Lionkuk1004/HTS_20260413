// ============================================================================
// HTS_Preamble_Holographic.h
// ============================================================================
// L1+L2+L5 Holographic Sync 유틸 (Python 시뮬 Phase 6 검증 기반)
//
// L1: Zadoff-Chu scrambling    (Chu 1972, EP1105991 만료 2019)
// L2: 8-segment non-coherent    (US6005887 만료 2016)
// L5: Peak-to-median adaptive   (공지 AGC)
//
// 특허: 전 구성요소 공지/만료, FTO safe
// 정밀도: Q14 (양자화 오차 최대 6.4e-5, 이론 한계 이내)
// SRAM: k_chu_table 256B ROM + stack
// ============================================================================

#ifndef HTS_PREAMBLE_HOLOGRAPHIC_H
#define HTS_PREAMBLE_HOLOGRAPHIC_H

#include <cstdint>

namespace ProtectedEngine {
namespace Holographic {

// ─────────────────────────────────────────────────────────────
// L1: Zadoff-Chu sample (Q14 complex)
// ─────────────────────────────────────────────────────────────
struct ChuSample {
    int16_t cos_q14;  // cos(phase) × 16384
    int16_t sin_q14;  // sin(phase) × 16384
};

// ROM 상수 테이블 (64 entry, 빌드 타임 확정)
extern const ChuSample k_chu_table[64];

// ─────────────────────────────────────────────────────────────
// L1+L2: Chu descramble + k_w63 + 8-segment non-coherent
// ─────────────────────────────────────────────────────────────

/// @brief Zadoff-Chu descramble + k_w63 + 8-seg combining
/// @param chip_I [in] 64 chip int16 I samples
/// @param chip_Q [in] 64 chip int16 Q samples
/// @return 8 segment 의 non-coherent energy 합산 (int64)
///
/// 계산:
///   desc[i] = rx[i] × conj(chu[i])              (Q14 복소수 곱)
///   w[i]    = desc[i] × k_w63[i]                (±1 곱)
///   energy  = Σ over 8 seg: |Σ_{i in seg} w[i]|²
///
/// 주의:
///   - int32 accumulator per segment (overflow safe: 8 chip × int16² < int32)
///   - int64 final sum
///   - 힙 할당 없음 (stack only)
int64_t holographic_dot_segmented(const int16_t* chip_I,
                                  const int16_t* chip_Q) noexcept;

// ─────────────────────────────────────────────────────────────
// L5: Peak-to-median ratio (x10 정수 정밀)
// ─────────────────────────────────────────────────────────────

/// @brief 64 energy 배열에서 peak/median × 10 반환
/// @param energies [in] 64 개 int64 energy
/// @param n        [in] 현재 구현은 64 고정
/// @return ratio × 10 (int32, min 0, max 9999 clamp)
///
/// 용도:
///   threshold 비교: ratio_x10 >= 25 (ratio_min 2.5 × 10)
///
/// 구현:
///   partial selection sort (median 까지만)
///   stack 사용: int64[64] = 512 byte
int32_t peak_to_median_ratio_x10(const int64_t* energies, int n) noexcept;

// ─────────────────────────────────────────────────────────────
// 진단 (테스트용)
// ─────────────────────────────────────────────────────────────

/// @brief Chu table 무결성 검증 (sin²+cos² ≈ Q28 one)
/// @return 최대 상대 오차 × 1e6 (int, e.g. 64 = 6.4e-5)
///
/// 용도: 단위 테스트, 빌드 후 1회 호출하여 확인
int32_t verify_chu_table_max_err_ppm() noexcept;

}  // namespace Holographic
}  // namespace ProtectedEngine

#endif  // HTS_PREAMBLE_HOLOGRAPHIC_H
