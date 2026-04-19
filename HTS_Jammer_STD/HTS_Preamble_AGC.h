// =============================================================================
// HTS_Preamble_AGC.h — 프리앰블 기반 디지털 AGC (AND/OR/SHIFT 전용)
// =============================================================================
// P0 검출 시 수신 에너지 측정 → 디지털 gain 계산 → 이후 칩에 적용.
// gain은 shift 단계: 2^N 배율 (0~4 = ×1,×2,×4,×8,×16)
// 곱셈/나눗셈/float 0회.
//
// @warning sizeof(HTS_Preamble_AGC) ≈ 8 bytes
// =============================================================================
#ifndef HTS_PREAMBLE_AGC_H
#define HTS_PREAMBLE_AGC_H

#include <cstdint>

// 실험용: AGC 출력 추가 스케일 (시프트 이후 int16 클램프 직전).
// 기본 256/256 = 기존 동작. 예: 128/256 = 0.5배 (포화 완화 시도).
#ifndef HTS_AGC_GAIN_SCALE_NUM
#define HTS_AGC_GAIN_SCALE_NUM 256
#endif
#ifndef HTS_AGC_GAIN_SCALE_DEN
#define HTS_AGC_GAIN_SCALE_DEN 256
#endif

namespace ProtectedEngine {

class HTS_Preamble_AGC {
public:
    // 목표 진폭 (e63 = target × 63 ≈ 63,000 >> 38,000)
    static constexpr int32_t kTargetAmp = 1000;

    /// @brief 초기화 (gain = 0 = ×1)
    void Init() noexcept;

    /// @brief P0 검출 성공 시 호출: 프리앰블 에너지로 gain 결정
    /// @param peak_mag P0에서 측정한 피크 |I|+|Q| 칩당 평균
    /// @note gain은 shift 단계 (0~4), 곱셈/나눗셈 0
    void Set_From_Peak(int32_t peak_mag) noexcept;

    /// @brief 칩에 gain 적용 (Feed_Chip 내, DC/CFO 후)
    /// @param[in,out] chipI, chipQ
    /// @note shift만 사용, 곱셈 0
    void Apply(int16_t& chipI, int16_t& chipQ) const noexcept;

    /// @brief 현재 gain 시프트 값 (0~4)
    int32_t Get_Shift() const noexcept { return gain_shift_; }

    /// @brief 활성 여부
    bool Is_Active() const noexcept { return gain_shift_ > 0; }

    /// @brief 리셋
    void Reset() noexcept;

private:
    int32_t gain_shift_ = 0;  // 0=×1, 1=×2, 2=×4, 3=×8, 4=×16
};

// ── 인라인 구현 ──

inline void HTS_Preamble_AGC::Init() noexcept {
    gain_shift_ = 0;
}

inline void HTS_Preamble_AGC::Reset() noexcept {
    gain_shift_ = 0;
}

inline void HTS_Preamble_AGC::Set_From_Peak(int32_t peak_mag) noexcept {
    // peak_mag = P0에서 측정한 칩 평균 |I|+|Q|
    // 목표: peak_mag << gain_shift ≈ kTargetAmp
    //
    // peak ≥ 1000: shift=0 (×1) — 이미 충분
    // peak ≥ 500:  shift=1 (×2)
    // peak ≥ 250:  shift=2 (×4)
    // peak ≥ 125:  shift=3 (×8)
    // peak ≥ 63:   shift=4 (×16)
    // peak < 63:   shift=4 (최대, 더 올리면 잡음 증폭)
    //
    // 비교만 사용 (shift/곱셈 0)
    if (peak_mag <= 0) {
        gain_shift_ = 0;  // 신호 없음
        return;
    }
    if (peak_mag >= kTargetAmp) {
        gain_shift_ = 0;
    } else if (peak_mag >= (kTargetAmp >> 1)) {   // ≥500
        gain_shift_ = 1;
    } else if (peak_mag >= (kTargetAmp >> 2)) {   // ≥250
        gain_shift_ = 2;
    } else if (peak_mag >= (kTargetAmp >> 3)) {   // ≥125
        gain_shift_ = 3;
    } else {
        gain_shift_ = 4;  // 최대 ×16
    }
}

inline void HTS_Preamble_AGC::Apply(int16_t& chipI, int16_t& chipQ) const noexcept {
    int32_t vi = static_cast<int32_t>(chipI);
    int32_t vq = static_cast<int32_t>(chipQ);
    if (gain_shift_ > 0) {
        vi <<= gain_shift_;
        vq <<= gain_shift_;
    }
    const int64_t num = static_cast<int64_t>(HTS_AGC_GAIN_SCALE_NUM);
    const int64_t den = static_cast<int64_t>(HTS_AGC_GAIN_SCALE_DEN);
    if (den != 0) {
        vi = static_cast<int32_t>((static_cast<int64_t>(vi) * num) / den);
        vq = static_cast<int32_t>((static_cast<int64_t>(vq) * num) / den);
    }
    if (vi > 32767) {
        vi = 32767;
    }
    if (vi < -32768) {
        vi = -32768;
    }
    if (vq > 32767) {
        vq = 32767;
    }
    if (vq < -32768) {
        vq = -32768;
    }
    chipI = static_cast<int16_t>(vi);
    chipQ = static_cast<int16_t>(vq);
}

} // namespace ProtectedEngine

#endif // HTS_PREAMBLE_AGC_H
