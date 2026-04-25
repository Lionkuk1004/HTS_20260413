// =============================================================================
// HTS_CFO_Compensator.h — 주파수 오프셋 보상 (Q14, ARM 경로 정수만)
// =============================================================================
// P0 프리앰블에서 위상 회전률 추정 → 이후 칩에 역회전 적용.
//
// - Apply: 복소 누적 위상 (Schmidl–Cox 스타일), 64 chip 마다 Q14 정수 정규화.
// - Estimate_From_Preamble: 레거시 2블록 dot (Walsh 호환). sin_per_chip 산출 후
//   cos_per_chip 은 Q14 단위원 제약 이진 탐색 제곱근(kQ14One²−sin²) 로 정합.
// - Estimate_From_Autocorr: Holo P0 lag 자기상관 벡터에 atan2(Q12)로 블록 위상,
//   per-chip 은 /lag 후 테일러로 sin Q14 (큰 CFO에서 small-angle 오류 완화).
// - Estimate_From_Preamble_MCE: Luise–Reggiannini 1995 MCE (PC 전용, libm).
//   HTS_ALLOW_HOST_BUILD 일 때만 선언·정의.
//
// @note KCMVP 심사 시 MCE는 비호스트 빌드에서 제외됨.
//
// @warning sizeof(HTS_CFO_Compensator) ≈ 32 bytes
// =============================================================================
#ifndef HTS_CFO_COMPENSATOR_LEGACY_H
#define HTS_CFO_COMPENSATOR_LEGACY_H

#include <cstdint>
#if defined(HTS_DIAG_PRINTF) && defined(HTS_DIAG_CFO_EST)
#include <cstdio>
#endif
#ifdef HTS_ALLOW_HOST_BUILD
#include <cmath>
#endif

namespace {

/// c2 의 정수 내림 제곱근, c2 ≤ kQ14One² (cos_per_chip 용)
inline int32_t hts_cfo_int_root_q14(int64_t c2) noexcept {
    if (c2 <= 0) return 0;
    int32_t lo = 0;
    constexpr int32_t kQ14 = 16384;
    int32_t hi = kQ14;
    while (lo < hi) {
        const int32_t mid = (lo + hi + 1) >> 1;
        if (static_cast<int64_t>(mid) * mid <= c2)
            lo = mid;
        else
            hi = mid - 1;
    }
    return lo;
}

/// mag2 내 정수 내림 제곱근 (누적 벡터 길이; mid*mid 오버플로 회피)
inline int64_t hts_cfo_int_root_u64(int64_t x) noexcept {
    if (x <= 0) return 0;
    int64_t lo = 0;
    int64_t hi = 3037000499LL;
    while (lo < hi) {
        const int64_t mid = (lo + hi + 1) >> 1;
        if (mid != 0 && mid <= x / mid)
            lo = mid;
        else
            hi = mid - 1;
    }
    return lo;
}

/// (cos_acc, sin_acc) 를 크기 ≈ kQ14One 인 Q14 벡터로 정수 정규화
inline void hts_cfo_renorm_q14_accum(int32_t &ca, int32_t &sa) noexcept {
    const int64_t fc = static_cast<int64_t>(ca);
    const int64_t fs = static_cast<int64_t>(sa);
    const int64_t mag2 = fc * fc + fs * fs;
    if (mag2 <= 0) return;
    const int64_t sm = hts_cfo_int_root_u64(mag2);
    if (sm == 0) return;
    constexpr int64_t k14 = 16384;
    ca = static_cast<int32_t>((fc * k14 + (sm >> 1)) / sm);
    sa = static_cast<int32_t>((fs * k14 + (sm >> 1)) / sm);
}

} // namespace

namespace ProtectedEngine {

class HTS_CFO_Compensator {
public:
    void Init() noexcept;

    /// @brief P0에서 연속 2블록의 Walsh-63 dot 결과로 CFO 추정 (호환 유지)
    void Estimate_From_Preamble(
        int32_t dI0, int32_t dQ0,
        int32_t dI1, int32_t dQ1,
        int32_t block_chips) noexcept;

    /// Holo P0: lag 자기상관 (ac_I + j·ac_Q) 위상, atan2 기반 (Walsh 경로 미사용).
    void Estimate_From_Autocorr(int32_t ac_I, int32_t ac_Q,
                                int32_t lag_chips) noexcept;

#ifdef HTS_ALLOW_HOST_BUILD
    /**
     * @brief MCE — Luise & Reggiannini 1995 (R(m)·conj(R(m−1)) 누적)
     * @param R_I R(0..8) 실수부, 길이 9
     * @param R_Q R(0..8) 허수부
     */
    void Estimate_From_Preamble_MCE(const int64_t *R_I,
                                    const int64_t *R_Q) noexcept;
#endif

    void Apply(int16_t &chipI, int16_t &chipQ) noexcept;

    /// @brief 칩 출력 없이 누적 위상만 n_chips 만큼 전진 (P0 스캔 구간 보정)
    ///        Apply 의 per-chip Q14 복소 곱(cos_acc_/sin_acc_ 갱신)만 반복하고,
    ///        64 chip 마다 Apply 와 동일한 Q14 정수 정규화를 수행.
    /// @param n_chips 전진할 chip 수 (Local.cpp 관례: P0 스캔 구간 192)
    /// @note  active_=false 이면 no-op. I/Q 회전·출력은 생략.
    void Advance_Phase_Only(int32_t n_chips) noexcept;

    bool Is_Active() const noexcept { return active_; }

    /// DIAG: Apply() 이 실제로 회전을 적용하는지(=active_)
    bool Is_Apply_Active() const noexcept { return active_; }

    int32_t Get_Sin_Per_Chip_Q14() const noexcept { return sin_per_chip_; }
    int32_t Get_Cos_Per_Chip_Q14() const noexcept { return cos_per_chip_; }

#ifdef HTS_ALLOW_HOST_BUILD
    /**
     * @brief sin/cos per chip (Q14) → Hz (chip_rate_hz = chips/s).
     * @note PS-LTE T6_SIM: 1e6; AMI: 2e5.
     */
    double Get_Est_Hz(double chip_rate_hz) const noexcept {
        if (!active_) {
            return 0.0;
        }
        constexpr double k14 = 16384.0;
        const double s = static_cast<double>(sin_per_chip_) / k14;
        const double c = static_cast<double>(cos_per_chip_) / k14;
        const double th = std::atan2(s, c);
        constexpr double kTwoPi =
            6.283185307179586476925286766559005768394338798750211;
        return th * chip_rate_hz / kTwoPi;
    }
#endif

    void Reset() noexcept;

private:
    int32_t cos_per_chip_ = 16384;
    int32_t sin_per_chip_ = 0;
    int32_t cos_acc_ = 16384;
    int32_t sin_acc_ = 0;
    bool active_ = false;
    int32_t chip_counter_ = 0;

    static constexpr int32_t kQ14One = 16384;
    /// atan(y/x) for 0 <= y <= x, x > 0; 결과 = atan(y/x)·4096 (Q12 각도).
    static int32_t atan_frac_q12(int32_t y, int32_t x) noexcept;
    /// atan2(y,x)·4096, 범위 약 [-12868, +12868] (±π rad).
    static int32_t atan2_q12(int32_t y, int32_t x) noexcept;
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

inline int32_t HTS_CFO_Compensator::atan_frac_q12(int32_t y,
                                                  int32_t x) noexcept {
    if (x <= 0 || y < 0 || y > x) {
        return 0;
    }
    if (y == 0) {
        return 0;
    }
    static constexpr int16_t kLut[17] = {
        0,   256,  511,  763,  1018, 1266, 1508, 1741,
        1965, 2178, 2380, 2571, 2749, 2915, 3068, 3208, 3217};
    const int64_t ratio_q14 =
        (static_cast<int64_t>(y) << 14) / static_cast<int64_t>(x);
    const int64_t scaled = ratio_q14 * 16;
    int32_t idx = static_cast<int32_t>(scaled >> 14);
    if (idx >= 16) {
        return static_cast<int32_t>(kLut[16]);
    }
    const int32_t base = static_cast<int32_t>(kLut[idx]);
    const int32_t next = static_cast<int32_t>(kLut[idx + 1]);
    const int32_t frac = static_cast<int32_t>(scaled & ((1 << 14) - 1));
    return base + static_cast<int32_t>(
               (static_cast<int64_t>(next - base) * frac) >> 14);
}

inline int32_t HTS_CFO_Compensator::atan2_q12(int32_t y,
                                              int32_t x) noexcept {
    if (x == 0 && y == 0) {
        return 0;
    }
    const int32_t ax = (x < 0) ? -x : x;
    const int32_t ay = (y < 0) ? -y : y;
    const bool swap = ay > ax;
    const int32_t u = swap ? ay : ax;
    const int32_t v = swap ? ax : ay;
    if (u == 0) {
        return (y >= 0) ? 6434 : -6434;
    }
    int32_t ang = atan_frac_q12(v, u);
    if (swap) {
        ang = 6434 - ang;
    }
    if (x < 0) {
        ang = 12868 - ang;
    }
    if (y < 0) {
        ang = -ang;
    }
    return ang;
}

inline void HTS_CFO_Compensator::Estimate_From_Preamble(
    int32_t dI0, int32_t dQ0,
    int32_t dI1, int32_t dQ1,
    int32_t block_chips) noexcept
{
#if defined(HTS_DIAG_PRINTF) && defined(HTS_DIAG_CFO_EST)
    std::printf(
        "[EST-IN] dI0=%d dQ0=%d dI1=%d dQ1=%d block=%d\n",
        static_cast<int>(dI0), static_cast<int>(dQ0), static_cast<int>(dI1),
        static_cast<int>(dQ1), static_cast<int>(block_chips));
#endif
    // conj(z0)·z1 역회전용: cos = dI0*dI1+dQ0*dQ1, sin = dQ0*dI1−dI0*dQ1
    const int64_t cos_delta = static_cast<int64_t>(dI0) * dI1 +
                              static_cast<int64_t>(dQ0) * dQ1;
    const int64_t sin_delta = static_cast<int64_t>(dQ0) * dI1 -
                              static_cast<int64_t>(dI0) * dQ1;

#if defined(HTS_DIAG_PRINTF) && defined(HTS_DIAG_CFO_EST)
    std::printf(
        "[EST-MID1] cos_delta=%lld sin_delta=%lld\n",
        static_cast<long long>(cos_delta), static_cast<long long>(sin_delta));
#endif

    const int64_t ac = (cos_delta < 0) ? -cos_delta : cos_delta;
    const int64_t as = (sin_delta < 0) ? -sin_delta : sin_delta;
    const int64_t mag_approx = (ac > as) ? (ac + (as >> 1)) : (as + (ac >> 1));

#if defined(HTS_DIAG_PRINTF) && defined(HTS_DIAG_CFO_EST)
    std::printf(
        "[EST-MID2] ac=%lld as=%lld mag_approx=%lld\n",
        static_cast<long long>(ac), static_cast<long long>(as),
        static_cast<long long>(mag_approx));
#endif

    if (mag_approx < 1000) {
#if defined(HTS_DIAG_PRINTF) && defined(HTS_DIAG_CFO_EST)
        std::printf(
            "[EST-SKIP] mag_approx=%lld < 1000 -> active=0\n",
            static_cast<long long>(mag_approx));
#endif
        active_ = false;
        return;
    }

    const int64_t k14 = static_cast<int64_t>(kQ14One);
    const int32_t sin_block = static_cast<int32_t>((sin_delta * k14) / mag_approx);

#if defined(HTS_DIAG_PRINTF) && defined(HTS_DIAG_CFO_EST)
    std::printf(
        "[EST-MID3] sin_block=%d (sin_delta*k14)/mag_approx k14=%lld\n",
        static_cast<int>(sin_block), static_cast<long long>(k14));
#endif

    if (block_chips == 64) {
        sin_per_chip_ = sin_block >> 6;
    } else if (block_chips == 16) {
        sin_per_chip_ = sin_block >> 4;
    } else {
        sin_per_chip_ = sin_block / block_chips;
    }
    // cos: 정수 제곱근(kQ14One² - sin²) (Q14), 나눗셈 없는 이진 탐색 루프
    {
        const int64_t sin_sq =
            static_cast<int64_t>(sin_per_chip_) * sin_per_chip_;
        const int64_t q14_sq =
            static_cast<int64_t>(kQ14One) * kQ14One;
        const int64_t c2 = q14_sq - sin_sq;
        if (c2 <= 0) {
            cos_per_chip_ = 0;
        } else {
            cos_per_chip_ = hts_cfo_int_root_q14(c2);
        }
    }

#if defined(HTS_DIAG_PRINTF) && defined(HTS_DIAG_CFO_EST)
#if defined(HTS_ALLOW_HOST_BUILD)
    {
        constexpr double k14d = 16384.0;
        const double th = std::atan2(
            static_cast<double>(sin_per_chip_) / k14d,
            static_cast<double>(cos_per_chip_) / k14d);
        constexpr double kTwoPi =
            6.283185307179586476925286766559005768394338798750211;
        std::printf(
            "[EST-OUT] sin14=%d cos14=%d hz_est_1Mcps=%.2f (pre-active)\n",
            static_cast<int>(sin_per_chip_), static_cast<int>(cos_per_chip_),
            th * 1000000.0 / kTwoPi);
    }
#else
    std::printf("[EST-OUT] sin14=%d cos14=%d (pre-active)\n",
                static_cast<int>(sin_per_chip_),
                static_cast<int>(cos_per_chip_));
#endif
#endif

    cos_acc_ = kQ14One;
    sin_acc_ = 0;
    chip_counter_ = 0;
    active_ = true;
}

inline void HTS_CFO_Compensator::Estimate_From_Autocorr(
    int32_t ac_I, int32_t ac_Q, int32_t lag_chips) noexcept
{
#if defined(HTS_DIAG_PRINTF) && defined(HTS_DIAG_CFO_EST)
    std::printf(
        "[EST-ATAN2-IN] acI=%d acQ=%d lag=%d\n",
        static_cast<int>(ac_I), static_cast<int>(ac_Q),
        static_cast<int>(lag_chips));
#endif
    if (lag_chips <= 0) {
        active_ = false;
        return;
    }
    const int64_t mag2 = static_cast<int64_t>(ac_I) * ac_I +
                         static_cast<int64_t>(ac_Q) * ac_Q;
    if (mag2 < 1000000LL) {
#if defined(HTS_DIAG_PRINTF) && defined(HTS_DIAG_CFO_EST)
        std::printf(
            "[EST-ATAN2-SKIP] mag2=%lld < 1e6\n",
            static_cast<long long>(mag2));
#endif
        active_ = false;
        return;
    }

    const int32_t phase_q12_block = atan2_q12(ac_Q, ac_I);
    const int32_t phase_q12_chip = phase_q12_block / lag_chips;

    const int64_t ph = static_cast<int64_t>(phase_q12_chip);
    int64_t sin_q14 =
        (ph * 4LL) -
        (ph * ph * ph * static_cast<int64_t>(kQ14One)) /
            (6LL * 4096LL * 4096LL * 4096LL);
    if (sin_q14 > 32767LL) {
        sin_q14 = 32767LL;
    }
    if (sin_q14 < -32768LL) {
        sin_q14 = -32768LL;
    }
    sin_per_chip_ = static_cast<int32_t>(sin_q14);

    {
        const int64_t sin_sq =
            static_cast<int64_t>(sin_per_chip_) * sin_per_chip_;
        const int64_t q14_sq =
            static_cast<int64_t>(kQ14One) * kQ14One;
        const int64_t c2 = q14_sq - sin_sq;
        if (c2 <= 0) {
            cos_per_chip_ = 0;
        } else {
            cos_per_chip_ = hts_cfo_int_root_q14(c2);
        }
    }

#if defined(HTS_DIAG_PRINTF) && defined(HTS_DIAG_CFO_EST)
#if defined(HTS_ALLOW_HOST_BUILD)
    {
        constexpr double k14d = 16384.0;
        const double th = std::atan2(
            static_cast<double>(sin_per_chip_) / k14d,
            static_cast<double>(cos_per_chip_) / k14d);
        constexpr double kTwoPi =
            6.283185307179586476925286766559005768394338798750211;
        std::printf(
            "[EST-ATAN2-OUT] phase_blk=%d phase_chip=%d sin14=%d cos14=%d "
            "hz_est_1Mcps=%.2f (pre-active)\n",
            static_cast<int>(phase_q12_block),
            static_cast<int>(phase_q12_chip),
            static_cast<int>(sin_per_chip_),
            static_cast<int>(cos_per_chip_),
            th * 1000000.0 / kTwoPi);
    }
#else
    std::printf(
        "[EST-ATAN2-OUT] phase_blk=%d phase_chip=%d sin14=%d cos14=%d "
        "(pre-active)\n",
        static_cast<int>(phase_q12_block), static_cast<int>(phase_q12_chip),
        static_cast<int>(sin_per_chip_), static_cast<int>(cos_per_chip_));
#endif
#endif

    cos_acc_ = kQ14One;
    sin_acc_ = 0;
    chip_counter_ = 0;
    active_ = true;
}

#ifdef HTS_ALLOW_HOST_BUILD
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
#endif

inline void HTS_CFO_Compensator::Apply(int16_t &chipI, int16_t &chipQ) noexcept
{
#ifdef HTS_CFO_APPLY_FORCE_OFF
    (void)chipI; (void)chipQ;
    return;
#endif
    if (!active_) return;

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
        hts_cfo_renorm_q14_accum(cos_acc_, sin_acc_);
    }
}

inline void HTS_CFO_Compensator::Advance_Phase_Only(int32_t n_chips) noexcept {
    if (!active_) return;
    if (n_chips <= 0) return;

    for (int32_t k = 0; k < n_chips; ++k) {
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
            hts_cfo_renorm_q14_accum(cos_acc_, sin_acc_);
        }
    }
}

} // namespace ProtectedEngine

#endif // HTS_CFO_COMPENSATOR_LEGACY_H
