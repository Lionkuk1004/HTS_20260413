// HTS_V400_Dispatcher_Gravity.hpp
// INNOViD HTS B-CDMA 양산 - 6면 PTE Gravity Cube
// V5a CFO + PaCD 후 위상 정화 / 노이즈 검출
// DPTE (BPTE + PTE) 적용, KCMVP constant-time
//
// face_D 하이브리드 설계 (G-3-2):
//   ① peak/noise 비: gravity 자체 (q10_ratio_clamp)
//   ② 곡률 (PTE):    pn_masked_pte_subchip 호출
//   ③ sharpness:     BPTE 후처리 (Q8)
//   ④ validity:      BPTE mask
//
// face_E (G-3-2): Atan2_Correlation_Dpte_Q15 (V5a 자산) 호출

#pragma once

#include <cstdint>

#if defined(HTS_USE_GRAVITY) && HTS_USE_GRAVITY

namespace HTS_LIM {
namespace detail_gravity {

// ============================================================
// 6면 Gravity Cube 자료구조 (ab19ea9^ 자산 + PTE 확장)
// ============================================================

struct GravityCube6 {
    // 6면 score (Q10)
    int32_t face_A_q10 = 0;   // 4 템플릿 에너지 균형 (BPTE)
    int32_t face_B_q10 = 0;   // 서브블록 균등성 (BPTE)
    int32_t face_C_q10 = 0;   // (0,1) vs (2,3) 묶음 (BPTE)
    int32_t face_D_q10 = 0;   // peak/noise 비 (하이브리드)
    int32_t face_E_q10 = 0;   // 위상 연속성 (atan2 V5a)
    int32_t face_F_q10 = 0;   // 약한 면 vs noise (BPTE)

    int32_t total_score = 0;
    int      best_off = 0;

    // 4 템플릿 복소 상관 (V5a + PaCD 후 chip)
    int32_t tmpl_xI[4] = {};
    int32_t tmpl_xQ[4] = {};

    // face_D PTE 확장 (G-3-2)
    int32_t pte_delta_q14 = 0;        // pn_masked_pte_subchip 결과
    int32_t pte_sharpness_q8 = 0;     // BPTE 후처리
    uint32_t pte_valid_mask = 0u;     // BPTE: 1=정상, 0=평탄/spike
};

// 임계값 (Q10, ab19ea9^ 튜닝값)
constexpr int32_t GRAVITY_THR_A_Q10 = 200;
constexpr int32_t GRAVITY_THR_B_Q10 = 45;
constexpr int32_t GRAVITY_THR_C_Q10 = 80;
constexpr int32_t GRAVITY_THR_D_Q10 = 1200;
constexpr int32_t GRAVITY_THR_E_Q10 = 400;
constexpr int32_t GRAVITY_THR_F_Q10 = 700;

// PTE 임계값 (NEW - 영준님 IP)
constexpr int32_t GRAVITY_SPIKE_THR_Q8 = 500;

// ============================================================
// API
// ============================================================

/// 6면 Gravity Cube 평가
/// 입력: V5a + PaCD 적용된 chip I/Q, 4×64 템플릿 I (`tx_pre_tmpl_I` 연속 256 int16)
/// `rx_chip_count`: 유효 RX 칩 수 — ab19 윈도우 상관에 **최소 256** 필요 (미만 시 false).
/// 출력: cube_pass (true/false), GravityCube6 (진단용)
///
/// DPTE 적용 (G-3-2):
///   - 모든 분기 BPTE 변환 (constant-time)
///   - face_D: gravity 자체 + pn_masked_pte_subchip 호출
///   - face_E: Atan2_Correlation_Dpte_Q15 (V5a 자산) 호출
[[nodiscard]] bool gravity_evaluate_cube(
    const int16_t* rx_pre_I,
    const int16_t* rx_pre_Q,
    const int16_t* tx_pre_tmpl_I,
    int32_t rx_chip_count,
    GravityCube6* out_cube) noexcept;

/// CMYK 4×64 템플릿 (연속 256 int16: A‖B‖C‖D). `pre_amp` 는 TX `generate_holo_preamble_` 와 동일 스케일(보통 1000).
void gravity_fill_cmyk_templates_i256(const uint32_t seed[4], uint32_t slot,
                                      int16_t pre_amp,
                                      int16_t* out256) noexcept;

} // namespace detail_gravity
} // namespace HTS_LIM

#endif // HTS_USE_GRAVITY
