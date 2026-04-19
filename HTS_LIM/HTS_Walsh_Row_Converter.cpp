#include "HTS_Walsh_Row_Converter.hpp"
#include <cstdint>
#include <cstdio>

namespace ProtectedEngine {
namespace WRC {

static DiagCounters g_diag = {};

DiagCounters& get_diag() noexcept {
    return g_diag;
}

void reset_diag() noexcept {
    g_diag.call_count_16 = 0;
    g_diag.call_count_64 = 0;
    g_diag.total_delta_sum = 0;
    g_diag.method = static_cast<uint64_t>(HTS_WRC_METHOD);
}

void print_diag() noexcept {
    std::fprintf(stderr,
                 "\n=== WRC DIAG ===\n"
                 "  method         = %llu\n"
                 "  call_count_16  = %llu\n"
                 "  call_count_64  = %llu\n"
                 "  total_delta    = %llu\n"
                 "================\n",
                 static_cast<unsigned long long>(g_diag.method),
                 static_cast<unsigned long long>(g_diag.call_count_16),
                 static_cast<unsigned long long>(g_diag.call_count_64),
                 static_cast<unsigned long long>(g_diag.total_delta_sum));
}

namespace {

// FWHT N-point (in-place)
// 기존 FEC_HARQ::FWHT 대신 독립 구현 (side-effect 격리)
static void fwht_local(int32_t* d, int N) noexcept {
    for (int h = 1; h < N; h <<= 1) {
        for (int i = 0; i < N; i += (h << 1)) {
            for (int j = i; j < i + h; ++j) {
                const int32_t a = d[j];
                const int32_t b = d[j + h];
                d[j] = a + b;
                d[j + h] = a - b;
            }
        }
    }
}

// IFWHT = FWHT / N (정수는 나눗셈 주의, N 이 2^k 이므로 shift)
// N = 16 → >> 4
// N = 64 → >> 6
static void ifwht_local(int32_t* d, int N) noexcept {
    fwht_local(d, N);
    int shift = 0;
    int nn = N;
    while (nn > 1) {
        shift++;
        nn >>= 1;
    }
    for (int i = 0; i < N; ++i) {
        d[i] >>= shift;
    }
}

// integer sqrt (Newton-Raphson)
static int64_t isqrt64(int64_t v) noexcept {
    if (v <= 0) {
        return 0;
    }
    int64_t x = v;
    int64_t y = (x + 1) / 2;
    for (int it = 0; it < 10 && y < x; ++it) {
        x = y;
        y = (x + v / x) / 2;
    }
    return x;
}

// ---- 방식 A: zero-out ----
static void method_A_zero_out(int32_t* T, int N) noexcept {
    int64_t E[MAX_N];
    for (int r = 0; r < N; ++r) {
        E[r] = static_cast<int64_t>(T[r]) * T[r];
    }
    int top1 = 0;
    int64_t top1_e = E[0];
    for (int r = 1; r < N; ++r) {
        if (E[r] > top1_e) {
            top1_e = E[r];
            top1 = r;
        }
    }
    int64_t sum = 0;
    for (int r = 0; r < N; ++r) {
        if (r == top1) {
            continue;
        }
        sum += E[r];
    }
    const int64_t mean_e = sum / (N - 1);
    int64_t ssd = 0;
    for (int r = 0; r < N; ++r) {
        if (r == top1) {
            continue;
        }
        const int64_t dev = E[r] - mean_e;
        ssd += dev * dev;
    }
    const int64_t var_e = ssd / (N - 1);
    const int64_t std_e = isqrt64(var_e);
    const int64_t thresh = mean_e + std_e * SIGMA_NUM / SIGMA_DEN;
    for (int r = 0; r < N; ++r) {
        if (r == top1) {
            continue;
        }
        if (E[r] > thresh) {
            T[r] = 0;
        }
    }
}

// ---- 방식 B: attenuate ----
static void method_B_attenuate(int32_t* T, int N) noexcept {
    int64_t E[MAX_N];
    for (int r = 0; r < N; ++r) {
        E[r] = static_cast<int64_t>(T[r]) * T[r];
    }
    int top1 = 0;
    int64_t top1_e = E[0];
    for (int r = 1; r < N; ++r) {
        if (E[r] > top1_e) {
            top1_e = E[r];
            top1 = r;
        }
    }
    int64_t sum = 0;
    for (int r = 0; r < N; ++r) {
        if (r == top1) {
            continue;
        }
        sum += E[r];
    }
    const int64_t mean_e = sum / (N - 1);
    int64_t ssd = 0;
    for (int r = 0; r < N; ++r) {
        if (r == top1) {
            continue;
        }
        const int64_t dev = E[r] - mean_e;
        ssd += dev * dev;
    }
    const int64_t var_e = ssd / (N - 1);
    const int64_t std_e_raw = isqrt64(var_e);
    const int64_t std_e = (std_e_raw > 0) ? std_e_raw : 1;
    for (int r = 0; r < N; ++r) {
        if (r == top1) {
            continue;
        }
        int64_t excess_q8 = 0;
        const int64_t dev = E[r] - mean_e;
        if (dev > 0) {
            excess_q8 = (dev * 256) / std_e;
        }
        const int64_t denom = 65536 + ALPHA_Q8 * excess_q8;
        int64_t w_q8 = (static_cast<int64_t>(W_MAX_Q8) * 65536) / denom;
        if (w_q8 < W_MIN_Q8) {
            w_q8 = W_MIN_Q8;
        }
        if (w_q8 > W_MAX_Q8) {
            w_q8 = W_MAX_Q8;
        }
        T[r] = static_cast<int32_t>((static_cast<int64_t>(T[r]) * w_q8) >> 8);
    }
}

// ---- 방식 C: top1-only ----
static void method_C_top1_only(int32_t* T, int N) noexcept {
    int64_t E[MAX_N];
    for (int r = 0; r < N; ++r) {
        E[r] = static_cast<int64_t>(T[r]) * T[r];
    }
    int top1 = 0;
    int64_t top1_e = E[0];
    for (int r = 1; r < N; ++r) {
        if (E[r] > top1_e) {
            top1_e = E[r];
            top1 = r;
        }
    }
    const int32_t top1_val = T[top1];
    for (int r = 0; r < N; ++r) {
        T[r] = 0;
    }
    T[top1] = top1_val;
}

} // namespace

void clean_chips(int16_t* chips, int N) noexcept {
    if (chips == nullptr) {
        return;
    }
    if (N != 16 && N != 64) {
        return;
    }

    int16_t backup[MAX_N];
    for (int i = 0; i < N; ++i) {
        backup[i] = chips[i];
    }

#if HTS_WRC_METHOD == 0
    if (N == 16) {
        ++get_diag().call_count_16;
    } else {
        ++get_diag().call_count_64;
    }
    (void)backup;
    return;
#else
    int32_t buf[MAX_N];
    for (int i = 0; i < N; ++i) {
        buf[i] = static_cast<int32_t>(chips[i]);
    }
    fwht_local(buf, N);
#if HTS_WRC_METHOD == 1
    method_A_zero_out(buf, N);
#elif HTS_WRC_METHOD == 2
    method_B_attenuate(buf, N);
#elif HTS_WRC_METHOD == 3
    method_C_top1_only(buf, N);
#endif
    ifwht_local(buf, N);
    for (int i = 0; i < N; ++i) {
        int32_t v = buf[i];
        if (v > 32767) {
            v = 32767;
        } else if (v < -32768) {
            v = -32768;
        }
        chips[i] = static_cast<int16_t>(v);
    }
    if (N == 16) {
        ++get_diag().call_count_16;
    } else {
        ++get_diag().call_count_64;
    }
    uint64_t delta = 0;
    for (int i = 0; i < N; ++i) {
        int32_t d =
            static_cast<int32_t>(chips[i]) - static_cast<int32_t>(backup[i]);
        if (d < 0) {
            d = -d;
        }
        delta += static_cast<uint64_t>(d);
    }
    get_diag().total_delta_sum += delta;
#endif
}

} // namespace WRC
} // namespace ProtectedEngine
