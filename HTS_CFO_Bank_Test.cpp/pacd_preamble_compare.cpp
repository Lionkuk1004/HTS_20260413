// C-3: v4 build_walsh128 vs PaCD static TX preamble — sign + amplitude ratio check.
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstdio>

#include "HTS_V400_Dispatcher_PaCD.hpp"

static void build_v4_preamble(int16_t* pre_I, int16_t* pre_Q, int amp) noexcept {
    for (int c = 0; c < 64; ++c) {
        const uint32_t x = 63u & static_cast<uint32_t>(c);
        const int parity = static_cast<int>(std::popcount(x) & 1u);
        const int16_t v = static_cast<int16_t>(parity ? -amp : amp);
        pre_I[c] = v;
        pre_Q[c] = v;
    }
    for (int c = 0; c < 64; ++c) {
        pre_I[64 + c] = static_cast<int16_t>(amp);
        pre_Q[64 + c] = static_cast<int16_t>(amp);
    }
}

static int sign_of(int16_t x) noexcept {
    if (x > 0) {
        return 1;
    }
    if (x < 0) {
        return -1;
    }
    return 0;
}

int main() {
    int16_t v4_I[128], v4_Q[128];
    build_v4_preamble(v4_I, v4_Q, 500);

    const int16_t* pacd_I = ::detail::pacd_tx_preamble128_I();
    const int16_t* pacd_Q = ::detail::pacd_tx_preamble128_Q();

    int sign_match_I = 0;
    int sign_diff_I = 0;
    int sign_match_Q = 0;
    int sign_diff_Q = 0;
    int both_nonzero = 0;
    double ratio_sum = 0.0;
    int ratio_count = 0;

    for (int c = 0; c < 128; ++c) {
        const int si = sign_of(v4_I[c]);
        const int pi = sign_of(pacd_I[c]);
        if (si == pi) {
            ++sign_match_I;
        } else {
            ++sign_diff_I;
        }
        const int sq = sign_of(v4_Q[c]);
        const int pq = sign_of(pacd_Q[c]);
        if (sq == pq) {
            ++sign_match_Q;
        } else {
            ++sign_diff_Q;
        }

        if (v4_I[c] != 0 && pacd_I[c] != 0) {
            ++both_nonzero;
            ratio_sum += static_cast<double>(pacd_I[c]) /
                         static_cast<double>(v4_I[c]);
            ++ratio_count;
        }
    }

    std::printf("Sign match I: %d/128, diff: %d/128\n", sign_match_I, sign_diff_I);
    std::printf("Sign match Q: %d/128, diff: %d/128\n", sign_match_Q, sign_diff_Q);
    if (ratio_count > 0) {
        std::printf("Amplitude ratio mean (pacd_I/v4_I, %d chips): %.6f\n",
                    ratio_count, ratio_sum / static_cast<double>(ratio_count));
    }
    std::printf("v4_I[0..7]:   %d %d %d %d %d %d %d %d\n", static_cast<int>(v4_I[0]),
                static_cast<int>(v4_I[1]), static_cast<int>(v4_I[2]),
                static_cast<int>(v4_I[3]), static_cast<int>(v4_I[4]),
                static_cast<int>(v4_I[5]), static_cast<int>(v4_I[6]),
                static_cast<int>(v4_I[7]));
    std::printf("pacd_I[0..7]: %d %d %d %d %d %d %d %d\n",
                static_cast<int>(pacd_I[0]), static_cast<int>(pacd_I[1]),
                static_cast<int>(pacd_I[2]), static_cast<int>(pacd_I[3]),
                static_cast<int>(pacd_I[4]), static_cast<int>(pacd_I[5]),
                static_cast<int>(pacd_I[6]), static_cast<int>(pacd_I[7]));
    const int ok = (sign_match_I == 128 && sign_match_Q == 128) ? 0 : 1;
    return ok;
}
