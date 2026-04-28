// 격리 미니 CFO — 양산 V5a 의존성 없음 (double L&R + Apply)
#include "HTS_Mini_CFO.hpp"

#include <cmath>
#include <cstdint>

namespace hts {
namespace mini_cfo {

namespace {
constexpr double kPi = 3.14159265358979323846;
}  // namespace

Mini_CFO::Mini_CFO() noexcept : last_cfo_hz_(0) {}

void Mini_CFO::Init() noexcept {
    last_cfo_hz_ = 0;
}

int32_t Mini_CFO::Estimate(const int16_t* rx_I, const int16_t* rx_Q) noexcept {
    if (rx_I == nullptr || rx_Q == nullptr) {
        last_cfo_hz_ = 0;
        return 0;
    }

    int64_t Z_re[kMiniNumSeg]{};
    int64_t Z_im[kMiniNumSeg]{};

    for (int k = 0; k < kMiniNumSeg; ++k) {
        int64_t sum_I = 0;
        int64_t sum_Q = 0;
        const int start = k * kMiniSegSize;
        for (int n = 0; n < kMiniSegSize; ++n) {
            sum_I += static_cast<int64_t>(rx_I[start + n]);
            sum_Q += static_cast<int64_t>(rx_Q[start + n]);
        }
        Z_re[k] = sum_I;
        Z_im[k] = sum_Q;
    }

    int64_t sum_re = 0;
    int64_t sum_im = 0;
    for (int k = 1; k < kMiniNumSeg; ++k) {
        sum_re += Z_re[k] * Z_re[k - 1] + Z_im[k] * Z_im[k - 1];
        sum_im += Z_im[k] * Z_re[k - 1] - Z_re[k] * Z_im[k - 1];
    }

    if (sum_re == 0 && sum_im == 0) {
        last_cfo_hz_ = 0;
        return 0;
    }

    const double angle =
        std::atan2(static_cast<double>(sum_im), static_cast<double>(sum_re));
    const double f_est_d =
        angle * static_cast<double>(kMiniChipRateHz) /
        (2.0 * kPi * static_cast<double>(kMiniSegSize));

    last_cfo_hz_ = static_cast<int32_t>(std::lround(f_est_d));
    return last_cfo_hz_;
}

void Mini_CFO::Apply_Per_Chip(const int16_t* in_I, const int16_t* in_Q,
                              int16_t* out_I, int16_t* out_Q, int n_chips,
                              int32_t cfo_hz) noexcept {
    if (in_I == nullptr || in_Q == nullptr || out_I == nullptr ||
        out_Q == nullptr || n_chips <= 0) {
        return;
    }

    if (cfo_hz == 0) {
        for (int n = 0; n < n_chips; ++n) {
            out_I[n] = in_I[n];
            out_Q[n] = in_Q[n];
        }
        return;
    }

    const double omega =
        2.0 * kPi * static_cast<double>(cfo_hz) /
        static_cast<double>(kMiniChipRateHz);

    for (int n = 0; n < n_chips; ++n) {
        const double phase = omega * static_cast<double>(n);
        const double c = std::cos(phase);
        const double s = std::sin(phase);

        const double i_in = static_cast<double>(in_I[n]);
        const double q_in = static_cast<double>(in_Q[n]);

        const double i_out = i_in * c + q_in * s;
        const double q_out = -i_in * s + q_in * c;

        long long li = std::llround(i_out);
        long long lq = std::llround(q_out);
        if (li > 32767) {
            li = 32767;
        }
        if (li < -32768) {
            li = -32768;
        }
        if (lq > 32767) {
            lq = 32767;
        }
        if (lq < -32768) {
            lq = -32768;
        }
        out_I[n] = static_cast<int16_t>(li);
        out_Q[n] = static_cast<int16_t>(lq);
    }
}

}  // namespace mini_cfo
}  // namespace hts
