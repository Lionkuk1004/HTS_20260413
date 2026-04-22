// ============================================================================
// HTS_Preamble_Holographic.h
// ============================================================================
// L2+L3+L5 Holographic Sync 유틸 (Python Phase 7 + Step D DIAG 기반)
//
// L2: k_w63 × 8-segment non-coherent (Chu descramble 제거, Phase 4 방안 A)
// L5: Peak-to-median adaptive (공지 AGC)
//
// ROM: k_chu_table + verify_chu_table_max_err_ppm() — 단위 테스트/레거시 호환용
//       (sync 경로의 holographic_dot_segmented 는 Chu 미사용)
//
// 특허: 전 구성요소 공지/만료, FTO safe
// 정밀도: Q14 (양자화 오차 최대 6.4e-5, 이론 한계 이내)
// SRAM: k_chu_table 256B ROM(옵션) + stack
// ============================================================================

#ifndef HTS_PREAMBLE_HOLOGRAPHIC_H
#define HTS_PREAMBLE_HOLOGRAPHIC_H

#include <cstdint>

namespace ProtectedEngine {
namespace Holographic {

// ─────────────────────────────────────────────────────────────
// Zadoff-Chu sample (Q14) — ROM 테이블 + 단위 테스트 전용
// ─────────────────────────────────────────────────────────────
struct ChuSample {
    int16_t cos_q14;  // cos(phase) × 16384
    int16_t sin_q14;  // sin(phase) × 16384
};

extern const ChuSample k_chu_table[64];

// ─────────────────────────────────────────────────────────────
// L2: k_w63 + 8-segment non-coherent (Chu 미적용)
// ─────────────────────────────────────────────────────────────

/// @brief k_w63 기반 8-seg non-coherent energy (Chu descramble 없음)
/// @param chip_I [in] 64 chip int16 I
/// @param chip_Q [in] 64 chip int16 Q
/// @return Σ_seg |Σ_{i∈seg} (chip × k_w63)_I|² + |·_Q|²
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
// Phase 4.2: per-chip CFO derotation (Q8 step × 2π/256 per chip)
// ─────────────────────────────────────────────────────────────

/// @brief src 를 per-chip phase 로 derotate → dst (Q14 sin LUT, 누적 회전)
/// @param phase_q8  칩당 위상 증분 (signed Q8; 256 ≈ 2π rad/chip 아님 — 1 unit = 2π/256)
void derotate_buffer_q8(const int16_t* src_I,
                         const int16_t* src_Q,
                         int16_t* dst_I,
                         int16_t* dst_Q,
                         int n_chip,
                         int8_t phase_q8) noexcept;

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
