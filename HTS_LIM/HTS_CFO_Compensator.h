// =============================================================================
// HTS_CFO_Compensator.h — 주파수 오프셋 보상 (Q14 + 선택적 float 경로)
// =============================================================================
// P0 프리앰블에서 위상 회전률 추정 → 이후 칩에 역회전 적용.
//
// - Apply: 복소 누적 위상 (Schmidl–Cox 스타일), 64 chip 마다 float 정규화.
// - Estimate_From_Preamble: 레거시 2블록 dot (호환). sin_per_chip 산출 후
//   cos_per_chip 은 Q14 단위원 제약 sqrt(1−sin²) 로 정합 (cos≈1 소각도 제거).
// - Estimate_From_Preamble_MCE: Luise–Reggiannini 1995 MCE (float atan2).
//
// @note KCMVP 심사 시 MCE·sqrt·정규화 float 는 CORDIC/정수 근사로 치환 가능.
//
// @warning sizeof(HTS_CFO_Compensator) ≈ 32 bytes
// =============================================================================
#ifndef HTS_CFO_COMPENSATOR_H
#define HTS_CFO_COMPENSATOR_H

#include <cmath>
#include <cstdint>

namespace ProtectedEngine {

class HTS_CFO_Compensator {
public:
    void Init() noexcept;

    /// @brief P0에서 연속 2블록의 Walsh-63 dot 결과로 CFO 추정 (호환 유지)
    void Estimate_From_Preamble(
        int32_t dI0, int32_t dQ0,
        int32_t dI1, int32_t dQ1,
        int32_t block_chips) noexcept;

    /**
     * @brief MCE — Luise & Reggiannini 1995 (R(m)·conj(R(m−1)) 누적)
     * @param R_I R(0..8) 실수부, 길이 9
     * @param R_Q R(0..8) 허수부
     */
    void Estimate_From_Preamble_MCE(const int64_t *R_I,
                                    const int64_t *R_Q) noexcept;

    void Apply(int16_t &chipI, int16_t &chipQ) noexcept;

    bool Is_Active() const noexcept { return active_; }

    void Reset() noexcept;

private:
    int32_t cos_per_chip_ = 16384;
    int32_t sin_per_chip_ = 0;
    int32_t cos_acc_ = 16384;
    int32_t sin_acc_ = 0;
    bool active_ = false;
    int32_t chip_counter_ = 0;

    static constexpr int32_t kQ14One = 16384;
};

inline void HTS_CFO_Compensator::Init() noexcept {
    cos_per_chip_ = kQ14One;
    sin_per_chip_ = 0;
    cos_acc_ = kQ14One;
    sin_acc_ = 0;
    chip_counter_ = 0;
    active_ = false;
}

inline void HTS_CFO_Compensator::Reset() noexcept { Init(); }

inline void HTS_CFO_Compensator::Estimate_From_Preamble(
    int32_t dI0, int32_t dQ0,
    int32_t dI1, int32_t dQ1,
    int32_t block_chips) noexcept
{
    // conj(z0)·z1 의 실수/허수: cos = dI0*dI1+dQ0*dQ1, sin = dI0*dQ1−dQ0*dI1
    const int64_t cos_delta = static_cast<int64_t>(dI0) * dI1 +
                              static_cast<int64_t>(dQ0) * dQ1;
    const int64_t sin_delta = static_cast<int64_t>(dI0) * dQ1 -
                              static_cast<int64_t>(dQ0) * dI1;

    const int64_t ac = (cos_delta < 0) ? -cos_delta : cos_delta;
    const int64_t as = (sin_delta < 0) ? -sin_delta : sin_delta;
    const int64_t mag_approx = (ac > as) ? (ac + (as >> 1)) : (as + (ac >> 1));

    if (mag_approx < 1000) {
        active_ = false;
        return;
    }

    const int64_t k14 = static_cast<int64_t>(kQ14One);
    const int32_t sin_block = static_cast<int32_t>((sin_delta * k14) / mag_approx);

    if (block_chips == 64) {
        sin_per_chip_ = sin_block >> 6;
    } else if (block_chips == 16) {
        sin_per_chip_ = sin_block >> 4;
    } else {
        sin_per_chip_ = sin_block / block_chips;
    }
    // 복소 per-chip step 과 단위원 정합: |exp(jω)|=1 → cos²+sin²=kQ14One² (Q14)
    {
        const float sf = static_cast<float>(sin_per_chip_);
        const float c2 = static_cast<float>(kQ14One) * static_cast<float>(kQ14One) -
                         sf * sf;
        cos_per_chip_ = (c2 > 0.0f)
            ? static_cast<int32_t>(std::lround(std::sqrt(c2)))
            : 0;
    }

    cos_acc_ = kQ14One;
    sin_acc_ = 0;
    chip_counter_ = 0;
    active_ = true;
}

inline void HTS_CFO_Compensator::Estimate_From_Preamble_MCE(
    const int64_t *R_I, const int64_t *R_Q) noexcept
{
    if (R_I == nullptr || R_Q == nullptr) {
        active_ = false;
        return;
    }

    float sum_V_I = 0.0f;
    float sum_V_Q = 0.0f;
    for (int m = 1; m <= 8; ++m) {
        const float rm_I = static_cast<float>(R_I[m]);
        const float rm_Q = static_cast<float>(R_Q[m]);
        const float rm1_I = static_cast<float>(R_I[m - 1]);
        const float rm1_Q = static_cast<float>(R_Q[m - 1]);
        sum_V_I += rm_I * rm1_I + rm_Q * rm1_Q;
        sum_V_Q += rm_Q * rm1_I - rm_I * rm1_Q;
    }

    if (sum_V_I == 0.0f && sum_V_Q == 0.0f) {
        active_ = false;
        return;
    }

    const float theta_8 = std::atan2(sum_V_Q, sum_V_I);
    const float angle_per_chip = theta_8 * 0.125f;
    cos_per_chip_ = static_cast<int32_t>(
        std::round(std::cos(angle_per_chip) * 16384.0f));
    sin_per_chip_ = static_cast<int32_t>(
        std::round(std::sin(angle_per_chip) * 16384.0f));

    cos_acc_ = kQ14One;
    sin_acc_ = 0;
    chip_counter_ = 0;
    active_ = true;
}

inline void HTS_CFO_Compensator::Apply(int16_t &chipI, int16_t &chipQ) noexcept
{
    // ★ [CFO 4-2 DIAG] 컴파일 타임 매크로로 Apply 강제 무효화
    //   HTS_CFO_APPLY_FORCE_OFF 정의 시 Apply 는 no-op (회귀 비교용)
#if defined(HTS_CFO_APPLY_FORCE_OFF)
    (void)chipI; (void)chipQ;
    return;
#endif
    if (!active_) {
        return;
    }

    const int32_t ci = static_cast<int32_t>(chipI);
    const int32_t cq = static_cast<int32_t>(chipQ);

    int32_t ri = (ci * cos_acc_ + cq * sin_acc_) >> 14;
    int32_t rq = (cq * cos_acc_ - ci * sin_acc_) >> 14;

    if (ri > 32767) {
        ri = 32767;
    }
    if (ri < -32768) {
        ri = -32768;
    }
    if (rq > 32767) {
        rq = 32767;
    }
    if (rq < -32768) {
        rq = -32768;
    }

    chipI = static_cast<int16_t>(ri);
    chipQ = static_cast<int16_t>(rq);

    const int32_t next_cos = static_cast<int32_t>(
        (static_cast<int64_t>(cos_acc_) * static_cast<int64_t>(cos_per_chip_) -
         static_cast<int64_t>(sin_acc_) * static_cast<int64_t>(sin_per_chip_)) >>
        14);
    const int32_t next_sin = static_cast<int32_t>(
        (static_cast<int64_t>(cos_acc_) * static_cast<int64_t>(sin_per_chip_) +
         static_cast<int64_t>(sin_acc_) * static_cast<int64_t>(cos_per_chip_)) >>
        14);

    cos_acc_ = next_cos;
    sin_acc_ = next_sin;

    ++chip_counter_;
    if ((chip_counter_ & 0x3F) == 0) {
        const float fc = static_cast<float>(cos_acc_);
        const float fs = static_cast<float>(sin_acc_);
        const float mag2 = fc * fc + fs * fs;
        if (mag2 > 0.0f) {
            const float scale = 16384.0f / std::sqrt(mag2);
            cos_acc_ = static_cast<int32_t>(std::round(fc * scale));
            sin_acc_ = static_cast<int32_t>(std::round(fs * scale));
        }
    }
}

} // namespace ProtectedEngine

#endif // HTS_CFO_COMPENSATOR_H
