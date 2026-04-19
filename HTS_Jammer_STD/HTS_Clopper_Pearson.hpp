// =============================================================================
/// @file  HTS_Clopper_Pearson.hpp
/// @brief SPEC_002 §11 준수 — Clopper-Pearson exact 95% 신뢰구간 (Boost 없이)
/// @target PC 시뮬 전용 (HTS_Jammer_STD, Win32/x64)
///
/// [SPEC_002 §11.2 대체/꼼수 금지]
///   - Normal approximation 금지
///   - Wilson score interval 금지
///   - Beta distribution 정확 CI (p_lower, p_upper) 만 허용
///
/// [수학]
///   k = 성공 수, n = 시행 수, α = 0.05 (95% CI 기본)
///   p_lower = Beta⁻¹(α/2;   k,   n-k+1)     (k=0 이면 0.0)
///   p_upper = Beta⁻¹(1-α/2; k+1, n-k)       (k=n 이면 1.0)
///
/// [구현]
///   정규화 불완전 베타 함수 I_x(a,b) 를 continued fraction 전개로 계산,
///   이후 bisection 으로 역함수 탐색.
///   (Numerical Recipes 3rd Ed. betai/betacf 공식 참조, 직접 구현)
// =============================================================================
#pragma once
#include <cmath>
#include <cstdint>
namespace HTS_Clopper_Pearson {
// ── lgamma 의존 없는 순수 구현 위해 std::lgamma 사용 (C++11) ──
namespace detail {
// continued fraction for incomplete beta — NR 공식
inline double betacf_(double a, double b, double x) noexcept {
    constexpr int MAXIT = 200;
    constexpr double EPS = 3.0e-12;
    constexpr double FPMIN = 1.0e-300;
    const double qab = a + b;
    const double qap = a + 1.0;
    const double qam = a - 1.0;
    double c = 1.0;
    double d = 1.0 - qab * x / qap;
    if (std::fabs(d) < FPMIN)
        d = FPMIN;
    d = 1.0 / d;
    double h = d;
    for (int m = 1; m <= MAXIT; ++m) {
        const int m2 = 2 * m;
        double aa =
            static_cast<double>(m) * (b - m) * x / ((qam + m2) * (a + m2));
        d = 1.0 + aa * d;
        if (std::fabs(d) < FPMIN)
            d = FPMIN;
        c = 1.0 + aa / c;
        if (std::fabs(c) < FPMIN)
            c = FPMIN;
        d = 1.0 / d;
        h *= d * c;
        aa = -(a + m) * (qab + m) * x / ((a + m2) * (qap + m2));
        d = 1.0 + aa * d;
        if (std::fabs(d) < FPMIN)
            d = FPMIN;
        c = 1.0 + aa / c;
        if (std::fabs(c) < FPMIN)
            c = FPMIN;
        d = 1.0 / d;
        const double del = d * c;
        h *= del;
        if (std::fabs(del - 1.0) < EPS)
            break;
    }
    return h;
}
// 정규화 불완전 베타 함수 I_x(a, b), 0 ≤ x ≤ 1
inline double betai_(double a, double b, double x) noexcept {
    if (x <= 0.0)
        return 0.0;
    if (x >= 1.0)
        return 1.0;
    const double lnBeta = std::lgamma(a + b) - std::lgamma(a) - std::lgamma(b);
    const double bt =
        std::exp(lnBeta + a * std::log(x) + b * std::log(1.0 - x));
    // 수렴 대역 선택
    if (x < (a + 1.0) / (a + b + 2.0)) {
        return bt * betacf_(a, b, x) / a;
    }
    return 1.0 - bt * betacf_(b, a, 1.0 - x) / b;
}
// target 을 만족하는 p 를 bisection 으로 역산
//   solve for p:  I_p(a, b) = target
inline double beta_inv_bisect_(double target, double a, double b,
                               int iters = 80) noexcept {
    double lo = 0.0;
    double hi = 1.0;
    for (int i = 0; i < iters; ++i) {
        const double mid = 0.5 * (lo + hi);
        const double I = betai_(a, b, mid);
        if (I < target)
            lo = mid;
        else
            hi = mid;
    }
    return 0.5 * (lo + hi);
}
} // namespace detail
// ── Clopper-Pearson 95% 하한 ─────────────────────────────────
/// @param k  성공 수 (0 ≤ k ≤ n)
/// @param n  총 시행 수 (n > 0)
/// @param alpha  유의수준 (기본 0.05 → 95% CI)
/// @return p_lower. k=0 이면 0.0.
inline double Lower(int k, int n, double alpha = 0.05) noexcept {
    if (n <= 0)
        return 0.0;
    if (k <= 0)
        return 0.0;
    // p_lower = Beta⁻¹(α/2; k, n-k+1)
    const double a = static_cast<double>(k);
    const double b = static_cast<double>(n - k + 1);
    return detail::beta_inv_bisect_(alpha * 0.5, a, b);
}
// ── Clopper-Pearson 95% 상한 ─────────────────────────────────
/// @return p_upper. k=n 이면 1.0.
inline double Upper(int k, int n, double alpha = 0.05) noexcept {
    if (n <= 0)
        return 1.0;
    if (k >= n)
        return 1.0;
    // p_upper = Beta⁻¹(1-α/2; k+1, n-k)
    const double a = static_cast<double>(k + 1);
    const double b = static_cast<double>(n - k);
    return detail::beta_inv_bisect_(1.0 - alpha * 0.5, a, b);
}
// ── 편의: 점추정 + CI 한번에 ──────────────────────────────────
struct Interval {
    double p_hat;   ///< k/n
    double p_lower; ///< Clopper-Pearson 하한
    double p_upper; ///< Clopper-Pearson 상한
};
inline Interval Compute(int k, int n, double alpha = 0.05) noexcept {
    Interval r{};
    if (n <= 0)
        return r;
    r.p_hat = static_cast<double>(k) / static_cast<double>(n);
    r.p_lower = Lower(k, n, alpha);
    r.p_upper = Upper(k, n, alpha);
    return r;
}
} // namespace HTS_Clopper_Pearson
