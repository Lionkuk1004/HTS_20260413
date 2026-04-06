// =========================================================================
// HTS_Channel_Physics.cpp — PC 시뮬레이션 전용 (ARM 타깃 제외)
// =========================================================================
#include "HTS_Channel_Physics.h"

#if defined(__arm__) || defined(__TARGET_ARCH_ARM) || \
    defined(__TARGET_ARCH_THUMB) || defined(__ARM_ARCH)
#error "[HTS_FATAL] HTS_Channel_Physics.cpp는 PC 시뮬레이션 전용입니다. ARM 빌드에서 제외하십시오."
#endif

#include <cmath>
#include <cstddef>

namespace HTS_Core::Physics {

void Apply_Parametric_Channel(
    const std::vector<double>& tx,
    std::mt19937& rng,
    std::vector<double>& rx,
    ParametricChannel type,
    double intensity_db)
{
    const size_t N = tx.size();
    if (N == 0u || rx.size() != N) {
        return;
    }

    static constexpr int kNumChips = 128;
    static constexpr double kSpreadGain = static_cast<double>(kNumChips);
    static constexpr double kBaseNoiseSigma = 0.01;
    static constexpr double kIntensityDbPerDecade = 0.1;
    static constexpr double kEmpPercentScale = 0.01;
    static constexpr double kEmpAmp = 99999.0;
    static constexpr double kTwoPi = 6.28318530717958647692;
    static constexpr double kSinOmega = 2.0 * 3.14159265358979323846 * 8.0;

    std::normal_distribution<double> base_noise(0.0, kBaseNoiseSigma);

    switch (type) {

    case ParametricChannel::AWGN: {
        const double snr_linear = std::pow(10.0, intensity_db * kIntensityDbPerDecade);
        const double signal_power = kSpreadGain * kSpreadGain;
        const double inv_snr = 1.0 / snr_linear;
        const double noise_sigma = std::sqrt(signal_power * inv_snr);
        std::normal_distribution<double> awgn(0.0, noise_sigma);
        for (size_t i = 0u; i < N; ++i)
            rx[i] = tx[i] * kSpreadGain + awgn(rng);
        break;
    }

    case ParametricChannel::BARRAGE: {
        const double js_linear = std::pow(10.0, intensity_db * kIntensityDbPerDecade);
        const double jam_sigma = std::sqrt(js_linear) * kSpreadGain;
        std::normal_distribution<double> jam(0.0, jam_sigma);
        for (size_t i = 0u; i < N; ++i)
            rx[i] = tx[i] * kSpreadGain + jam(rng) + base_noise(rng);
        break;
    }

    case ParametricChannel::CW: {
        const double js_linear = std::pow(10.0, intensity_db * kIntensityDbPerDecade);
        const double cw_amp = std::sqrt(js_linear) * kSpreadGain * 2.0;
        const size_t cw_center = N >> 2u;
        const size_t cw_width = (N >> 4u) | 1u;
        const size_t lo =
            (cw_center >= cw_width) ? (cw_center - cw_width) : 0u;
        const size_t hi =
            (cw_center + cw_width < N) ? (cw_center + cw_width) : N;
        const uint32_t span_u =
            static_cast<uint32_t>(cw_width) << 1u;
        const double inv_span = 1.0 / static_cast<double>(span_u);

        std::uniform_real_distribution<double> phase_dist(0.0, kTwoPi);
        const double cw_phase = phase_dist(rng);

        for (size_t i = 0u; i < N; ++i) {
            double cw = 0.0;
            if (i >= lo && i < hi) {
                const double t =
                    static_cast<double>(i + cw_width - cw_center) * inv_span;
                cw = cw_amp * std::sin(kSinOmega * t + cw_phase);
            }
            rx[i] = tx[i] * kSpreadGain + cw + base_noise(rng);
        }
        break;
    }

    case ParametricChannel::EMP: {
        const double destroy_rate = intensity_db * kEmpPercentScale;
        std::uniform_real_distribution<double> u01(0.0, 1.0);
        for (size_t i = 0u; i < N; ++i) {
            if (u01(rng) < destroy_rate) {
                const double sign01 =
                    static_cast<double>(static_cast<std::uint32_t>(rng()) & 1u);
                const double amp_sign = 1.0 - 2.0 * sign01;
                rx[i] = amp_sign * kEmpAmp;
            }
            else {
                rx[i] = tx[i] * kSpreadGain + base_noise(rng);
            }
        }
        break;
    }

    default:
        for (size_t i = 0u; i < N; ++i) {
            rx[i] = tx[i];
        }
        break;

    } // switch
}

void Apply_Cw_Full_Tensor(
    const std::vector<double>& tx,
    std::mt19937& rng,
    std::vector<double>& rx,
    double intensity_db)
{
    const size_t N = tx.size();
    if (N == 0u || rx.size() != N) {
        return;
    }

    static constexpr int kNumChips = 128;
    static constexpr double kSpreadGain = static_cast<double>(kNumChips);
    static constexpr double kBaseNoiseSigma = 0.01;
    static constexpr double kIntensityDbPerDecade = 0.1;
    static constexpr double kTwoPi = 6.28318530717958647692;
    static constexpr double kSinOmega = 2.0 * 3.14159265358979323846 * 8.0;

    std::normal_distribution<double> base_noise(0.0, kBaseNoiseSigma);

    const double js_linear = std::pow(10.0, intensity_db * kIntensityDbPerDecade);
    const double cw_amp = std::sqrt(js_linear) * kSpreadGain * 2.0;
    std::uniform_real_distribution<double> phase_dist(0.0, kTwoPi);
    const double cw_phase = phase_dist(rng);

    const double inv_denom =
        (N > 1u)
        ? (1.0 / static_cast<double>(N - 1u))
        : 1.0;

    for (size_t i = 0u; i < N; ++i) {
        const double t = static_cast<double>(i) * inv_denom;
        const double cw = cw_amp * std::sin(kSinOmega * t + cw_phase);
        rx[i] = tx[i] * kSpreadGain + cw + base_noise(rng);
    }
}

void Apply_Lte_Channel_To(
    const std::vector<double>& tensor,
    std::mt19937& rng,
    std::vector<double>& out,
    double js_db,
    int num_chips,
    double emp_rate,
    double emp_amp)
{
    const size_t N = tensor.size();
    if (N == 0u || out.size() != N) {
        return;
    }

    static constexpr double kJsDbPerDecade = 0.1;

    const double js_linear = std::pow(10.0, js_db * kJsDbPerDecade);
    const double sigma_chip = std::sqrt(js_linear);
    const double spread = static_cast<double>(num_chips);
    const double sigma_total = sigma_chip * std::sqrt(spread);
    std::normal_distribution<double> noise(0.0, sigma_total);
    std::uniform_real_distribution<double> u01(0.0, 1.0);

    for (size_t i = 0u; i < N; ++i) {
        if (u01(rng) < emp_rate) {
            const double sign01 =
                static_cast<double>(static_cast<std::uint32_t>(rng()) & 1u);
            const double amp_sign = 1.0 - 2.0 * sign01;
            out[i] = amp_sign * emp_amp;
        }
        else {
            out[i] = tensor[i] * spread + noise(rng);
        }
    }
}

} // namespace HTS_Core::Physics
