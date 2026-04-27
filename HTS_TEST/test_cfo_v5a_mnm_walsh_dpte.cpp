// test_cfo_v5a_mnm_walsh_dpte.cpp — DPTE atan2 + M&M vs LR (host, /DHTS_USE_MNM_WALSH)
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

#if defined(_MSC_VER)
#include <intrin.h>
#endif

#include "HTS_CFO_V5a.hpp"
#include "HTS_Rx_CFO_SinCos_Table.hpp"

namespace hts {
namespace rx_cfo {
namespace test_export {
void Derotate_Table(const int16_t* rI, const int16_t* rQ, int16_t* oI,
                    int16_t* oQ, int chips, int32_t cfo_hz) noexcept;
int32_t LR_Estimate(const int16_t* rI, const int16_t* rQ) noexcept;
int32_t MnM_Walsh_Estimate_Dpte_Table(const int16_t* rI,
                                    const int16_t* rQ) noexcept;
}  // namespace test_export
}  // namespace rx_cfo
}  // namespace hts

#ifndef HTS_USE_MNM_WALSH
#error Build with /DHTS_USE_MNM_WALSH
#endif

namespace {

constexpr double kPi = 3.14159265358979323846;

static int32_t ref_atan2_q15(double y, double x) noexcept {
    const double a = std::atan2(y, x);
    constexpr double kInvPi = 0.31830988618379067154;
    return static_cast<int32_t>(std::llround(a * 32768.0 * kInvPi));
}

struct Acc {
    double s{ 0 };
    double s2{ 0 };
    int n{ 0 };
    void push(double x) noexcept {
        s += x;
        s2 += x * x;
        ++n;
    }
    double stddev() const noexcept {
        if (n <= 1)
            return 0.0;
        const double m = s / static_cast<double>(n);
        const double v = s2 / static_cast<double>(n) - m * m;
        return v > 0.0 ? std::sqrt(v) : 0.0;
    }
};

}  // namespace

int main() {
    using hts::rx_cfo::test_export::Atan2_Dpte_Q15_Table;
    using hts::rx_cfo::test_export::LR_Estimate;
    using hts::rx_cfo::test_export::MnM_Walsh_Estimate_Dpte_Table;

    int fail = 0;
    struct T {
        int64_t y;
        int64_t x;
        const char* name;
    };
    const T pts[] = {
        {0, 1, "0,1"},
        {1, 1, "1,1"},
        {1, 0, "1,0"},
        {1, -1, "1,-1"},
        {0, -1, "0,-1"},
        {-1, -1, "-1,-1"},
        {-1, 0, "-1,0"},
        {-1, 1, "-1,1"},
    };
    std::printf("=== Test7 atan2_dpte vs ref (tol 24 LSB) ===\n");
    for (const T& t : pts) {
        const int32_t got = Atan2_Dpte_Q15_Table(t.y, t.x);
        const int32_t refv = ref_atan2_q15(static_cast<double>(t.y),
                                           static_cast<double>(t.x));
        const int32_t d = got - refv;
        const int32_t ad = d < 0 ? -d : d;
        const int ok = ad <= 24;
        std::printf("%s got=%d ref=%d |d|=%d %s\n", t.name, static_cast<int>(got),
                    static_cast<int>(refv), static_cast<int>(ad),
                    ok ? "PASS" : "FAIL");
        fail += ok ? 0 : 1;
    }

    hts::rx_cfo::Build_SinCos_Table();

    std::printf("\n=== Test4 LR vs MnM (derotate@0Hz, signal@1000Hz, SNR10, 200 trials) ===\n");
    int16_t pre_I[128], pre_Q[128];
    for (int c = 0; c < 64; ++c) {
        const uint32_t x = 63u & static_cast<uint32_t>(c);
        const int parity = static_cast<int>(std::popcount(x) & 1u);
        const int16_t v = static_cast<int16_t>(parity ? -500 : 500);
        pre_I[c] = v;
        pre_Q[c] = v;
    }
    for (int c = 0; c < 64; ++c) {
        pre_I[64 + c] = 500;
        pre_Q[64 + c] = 500;
    }
    Acc a_lr;
    Acc a_mnm;
    int32_t lr_min = 2147483647;
    int32_t lr_max = -2147483647 - 1;
    for (int tr = 0; tr < 200; ++tr) {
        uint32_t seed = static_cast<uint32_t>(tr * 1315423911u + 17u);
        double sig = 0.0;
        for (int k = 0; k < 128; ++k) {
            sig += static_cast<double>(pre_I[k]) * pre_I[k];
            sig += static_cast<double>(pre_Q[k]) * pre_Q[k];
        }
        sig /= 128.0;
        const double ns =
            std::sqrt(sig / std::pow(10.0, 10.0 / 10.0));
        const double w = 2.0 * kPi * 1000.0 / 1e6;
        double ph = static_cast<double>(tr & 255) * 0.01;
        int16_t rx_I[128], rx_Q[128];
        for (int k = 0; k < 128; ++k) {
            const double cs = std::cos(ph);
            const double sn = std::sin(ph);
            double ri = pre_I[k] * cs - pre_Q[k] * sn;
            double rq = pre_I[k] * sn + pre_Q[k] * cs;
            const double u1 =
                static_cast<double>((seed = seed * 1664525u + 1013904223u)) /
                4294967296.0;
            const double u2 =
                static_cast<double>((seed = seed * 1664525u + 1013904223u)) /
                4294967296.0;
            double g1 = u1;
            double g2 = u2;
            if (g1 < 1e-10)
                g1 = 1e-10;
            const double g =
                std::sqrt(-2.0 * std::log(g1)) * std::cos(2.0 * kPi * g2);
            ri += g * ns;
            rq += g * ns;
            int32_t iI = static_cast<int32_t>(ri);
            int32_t iQ = static_cast<int32_t>(rq);
            if (iI > 32767)
                iI = 32767;
            if (iI < -32768)
                iI = -32768;
            if (iQ > 32767)
                iQ = 32767;
            if (iQ < -32768)
                iQ = -32768;
            rx_I[k] = static_cast<int16_t>(iI);
            rx_Q[k] = static_cast<int16_t>(iQ);
            ph += w;
        }
        int16_t wI[128], wQ[128];
        // Intentional 5 Hz derotate mismatch → nonzero LR / MnM jitter.
        // Derotate @ 0 Hz → ~1000 Hz spin remains on Walsh preamble.
        hts::rx_cfo::test_export::Derotate_Table(rx_I, rx_Q, wI, wQ, 128, 0);
        const int32_t lr = LR_Estimate(wI, wQ);
        const int32_t mnm = MnM_Walsh_Estimate_Dpte_Table(wI, wQ);
        if (tr == 0) {
            std::printf("trial0 lr=%d mnm=%d wI[0]=%d\n", static_cast<int>(lr),
                        static_cast<int>(mnm), static_cast<int>(wI[0]));
        }
        a_lr.push(static_cast<double>(lr));
        a_mnm.push(static_cast<double>(mnm));
        lr_min = lr < lr_min ? lr : lr_min;
        lr_max = lr > lr_max ? lr : lr_max;
    }
    std::printf("LR range [%d, %d] Hz (200 trials)\n", static_cast<int>(lr_min),
                static_cast<int>(lr_max));
    const double s_lr = a_lr.stddev();
    const double s_mnm = a_mnm.stddev();
    std::printf("std_lr=%.3f Hz std_mnm=%.3f Hz ratio_mnm_lr=%.3f\n", s_lr,
                s_mnm, s_mnm / (s_lr > 1e-6 ? s_lr : 1.0));
    int t4 = 0;
    if (s_lr < 0.5) {
        std::printf("Test4: SKIP (LR std < 0.5 Hz, degenerate)\n");
        t4 = 1;
    } else {
        t4 = (s_mnm < s_lr * 0.55) ? 1 : 0;
        std::printf("Test4 (MnM < 0.55*LR std): %s\n", t4 ? "PASS" : "FAIL");
    }
    fail += t4 ? 0 : 1;

    std::printf("\n=== Test6 RDTSC spread (warmup 50, measure 200) ===\n");
#if defined(_MSC_VER)
    volatile int32_t sink = 0;
    for (int w = 0; w < 50; ++w) {
        sink += Atan2_Dpte_Q15_Table(static_cast<int64_t>(w - 25),
                                       static_cast<int64_t>(100 + w));
    }
    Acc cyc;
    for (int i = 0; i < 200; ++i) {
        const int64_t yi = static_cast<int64_t>((i * 7919) % 2000001) - 1000000;
        const int64_t xi = static_cast<int64_t>((i * 65537) % 2000001) - 1000000;
        const unsigned __int64 t0 = __rdtsc();
        const int32_t v = Atan2_Dpte_Q15_Table(yi, xi);
        const unsigned __int64 t1 = __rdtsc();
        sink += v;
        cyc.push(static_cast<double>(t1 - t0));
    }
    std::printf("atan2_dpte cycles mean=%.1f std=%.2f (informational)\n",
                cyc.s / static_cast<double>(cyc.n), cyc.stddev());
#else
    std::printf("RDTSC skipped (non-MSVC)\n");
#endif

    std::printf("\nOVERALL: %s\n", fail ? "FAIL" : "PASS");
    return fail ? 1 : 0;
}
