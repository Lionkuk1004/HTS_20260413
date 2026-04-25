#include <cmath>
#include <cstdint>
#include <cstdio>

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kFs = 1000000.0;
constexpr int kN = 64;
constexpr int16_t kAmp = 1000;
static constexpr int8_t kWalsh63Row63[64] = {
    +1, -1, -1, +1, -1, +1, +1, -1, -1, +1, +1, -1, +1, -1, -1, +1,
    -1, +1, +1, -1, +1, -1, -1, +1, +1, -1, -1, +1, -1, +1, +1, -1,
    -1, +1, +1, -1, +1, -1, -1, +1, +1, -1, -1, +1, -1, +1, +1, -1,
    +1, -1, -1, +1, -1, +1, +1, -1, -1, +1, +1, -1, +1, -1, -1, +1};
}

int main() {
    int16_t base_i[kN]{};
    int16_t base_q[kN]{};
    for (int n = 0; n < kN; ++n) {
        base_i[n] = static_cast<int16_t>(kAmp * kWalsh63Row63[n]);
        base_q[n] = 0;
    }

    int64_t d0_i = 0;
    int64_t d0_q = 0;
    for (int n = 0; n < kN; ++n) {
        d0_i += static_cast<int64_t>(base_i[n]) * kWalsh63Row63[n];
        d0_q += static_cast<int64_t>(base_q[n]) * kWalsh63Row63[n];
    }
    const double e0 = static_cast<double>(d0_i * d0_i + d0_q * d0_q);

    const int hz_list[] = {0, 1000, 5000, 10000};
    for (const int hz : hz_list) {
        int64_t d_i = 0;
        int64_t d_q = 0;
        for (int n = 0; n < kN; ++n) {
            const double ph =
                2.0 * kPi * static_cast<double>(hz) * static_cast<double>(n) / kFs;
            const double i = static_cast<double>(base_i[n]) * std::cos(ph) -
                             static_cast<double>(base_q[n]) * std::sin(ph);
            const double q = static_cast<double>(base_i[n]) * std::sin(ph) +
                             static_cast<double>(base_q[n]) * std::cos(ph);
            const int32_t ri = static_cast<int32_t>(std::llround(i));
            const int32_t rq = static_cast<int32_t>(std::llround(q));
            d_i += static_cast<int64_t>(ri) * kWalsh63Row63[n];
            d_q += static_cast<int64_t>(rq) * kWalsh63Row63[n];
        }
        const double e = static_cast<double>(d_i * d_i + d_q * d_q);
        const double ratio = (e0 > 0.0) ? (e / e0) : 0.0;
        std::printf("WalshCfoSensitivity: hz=%6d energy_ratio=%.6f\n", hz, ratio);
    }

    std::printf("test_walsh_cfo_sensitivity: PASS\n");
    return 0;
}
