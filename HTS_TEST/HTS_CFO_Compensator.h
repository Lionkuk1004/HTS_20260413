// =============================================================================
// HTS_CFO_Compensator.h — 주파수 오프셋 보상 (AND/OR/SHIFT 전용)
// =============================================================================
// P0 프리앰블에서 위상 회전률 추정 → 이후 칩에 역회전 적용.
// CORDIC/float/곱셈 배제. 1차 선형 보정.
//
// @warning sizeof ≈ 24 bytes
// =============================================================================
#ifndef HTS_CFO_COMPENSATOR_H
#define HTS_CFO_COMPENSATOR_H

#include <cstdint>

namespace ProtectedEngine {

class HTS_CFO_Compensator {
public:
    /// @brief 초기화
    void Init() noexcept;

    /// @brief P0에서 연속 2블록의 Walsh-63 dot 결과로 CFO 추정
    /// @param dI0,dQ0 블록 0의 dot(I,Q)
    /// @param dI1,dQ1 블록 1의 dot(I,Q)
    /// @param block_chips 블록 간 칩 수 (64)
    /// @note 곱셈 2회 (위상차 계산, 불가피), mag 정규화는 int64 나눗셈 2회
    void Estimate_From_Preamble(
        int32_t dI0, int32_t dQ0,
        int32_t dI1, int32_t dQ1,
        int32_t block_chips) noexcept;

    /// @brief 칩 보정 적용 (Feed_Chip 내에서 호출)
    /// @param[in,out] chipI, chipQ
    /// @note 곱셈 2회/칩 (복소 회전, 불가피), shift 정규화
    void Apply(int16_t& chipI, int16_t& chipQ) noexcept;

    /// @brief CFO 활성 여부
    bool Is_Active() const noexcept { return active_; }

    /// @brief 리셋
    void Reset() noexcept;

private:
    // 칩당 역회전 cos/sin (Q14 고정소수: 1.0 = 16384)
    int32_t cos_per_chip_ = 16384;   // Q14, 초기값 1.0
    int32_t sin_per_chip_ = 0;       // Q14, 초기값 0.0

    // 누적 위상 cos/sin (Q14)
    int32_t cos_acc_ = 16384;
    int32_t sin_acc_ = 0;

    bool active_ = false;

    // Q14 상수
    static constexpr int32_t kQ14One = 16384;  // 1 << 14
};

// ── 인라인 구현 ──

inline void HTS_CFO_Compensator::Init() noexcept {
    cos_per_chip_ = kQ14One;
    sin_per_chip_ = 0;
    cos_acc_ = kQ14One;
    sin_acc_ = 0;
    active_ = false;
}

inline void HTS_CFO_Compensator::Reset() noexcept {
    Init();
}

inline void HTS_CFO_Compensator::Estimate_From_Preamble(
    int32_t dI0, int32_t dQ0,
    int32_t dI1, int32_t dQ1,
    int32_t block_chips) noexcept
{
    // 위상차 계산: conj(d0) × d1
    //   cos_delta = dI0*dI1 + dQ0*dQ1
    //   sin_delta = dQ0*dI1 - dI0*dQ1
    // 이 곱셈은 위상 추정에 불가피
    const int64_t cos_delta = static_cast<int64_t>(dI0) * dI1 +
                               static_cast<int64_t>(dQ0) * dQ1;
    const int64_t sin_delta = static_cast<int64_t>(dQ0) * dI1 -
                               static_cast<int64_t>(dI0) * dQ1;

    // 정규화: |d0|×|d1| 로 나눠야 하지만, 부호와 비율만 필요
    // Q14로 스케일링: cos_per_block = cos_delta × 16384 / mag
    // mag = sqrt(cos_delta² + sin_delta²) ≈ max(|c|,|s|) + min(|c|,|s|)/2
    const int64_t ac = (cos_delta < 0) ? -cos_delta : cos_delta;
    const int64_t as = (sin_delta < 0) ? -sin_delta : sin_delta;
    const int64_t mag_approx = (ac > as) ? (ac + (as >> 1)) : (as + (ac >> 1));

    if (mag_approx < 1000) {
        // 신호 너무 약음 → CFO 추정 불가
        active_ = false;
        return;
    }

    // 블록 간 cos/sin (Q14) — 음수 cos_delta/sin_delta는 곱으로 처리 (<< 비음수만)
    const int64_t k14 = static_cast<int64_t>(kQ14One);
    const int32_t sin_block = static_cast<int32_t>((sin_delta * k14) / mag_approx);

    // 칩당 회전 = 블록 회전 / block_chips
    // 소각도 근사: sin(θ/N) ≈ sin(θ)/N, cos(θ/N) ≈ 1
    // sin_per_chip = sin_block / block_chips (shift: block_chips=64 → >>6)
    if (block_chips == 64) {
        sin_per_chip_ = sin_block >> 6;
    } else if (block_chips == 16) {
        sin_per_chip_ = sin_block >> 4;
    } else {
        // 범용 (나눗셈 1회, 초기화 시에만)
        sin_per_chip_ = sin_block / block_chips;
    }
    cos_per_chip_ = kQ14One;  // 소각도: cos ≈ 1

    // 누적 위상 리셋
    cos_acc_ = kQ14One;
    sin_acc_ = 0;
    active_ = true;
}

inline void HTS_CFO_Compensator::Apply(int16_t& chipI, int16_t& chipQ) noexcept {
    if (!active_) return;

    // 복소 역회전: (I',Q') = (I,Q) × (cos_acc, -sin_acc)
    //   I' = I×cos + Q×sin
    //   Q' = Q×cos - I×sin
    const int32_t ci = static_cast<int32_t>(chipI);
    const int32_t cq = static_cast<int32_t>(chipQ);

    // Q14 곱셈 (칩당 2회, 불가피)
    int32_t ri = (ci * cos_acc_ + cq * sin_acc_) >> 14;
    int32_t rq = (cq * cos_acc_ - ci * sin_acc_) >> 14;

    // 클램프
    if (ri > 32767) ri = 32767; if (ri < -32768) ri = -32768;
    if (rq > 32767) rq = 32767; if (rq < -32768) rq = -32768;

    chipI = static_cast<int16_t>(ri);
    chipQ = static_cast<int16_t>(rq);

    // 누적 위상 갱신: acc × per_chip
    // 소각도 근사: cos_acc 불변, sin_acc += sin_per_chip
    sin_acc_ += sin_per_chip_;

    // sin_acc 포화 방지 (Q14: ±16384 = ±1.0)
    if (sin_acc_ > kQ14One) sin_acc_ = kQ14One;
    if (sin_acc_ < -kQ14One) sin_acc_ = -kQ14One;
}

} // namespace ProtectedEngine

#endif // HTS_CFO_COMPENSATOR_H
