// Mini CFO 단위 테스트 — 양산과 동일 Walsh PRE_SYM 입력 + 격리 V5a.Estimate 비교
#include "HTS_CFO_V5a.hpp"
#include "HTS_Mini_CFO.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

using hts::mini_cfo::kMiniChipRateHz;
using hts::mini_cfo::kMiniPreambleChips;
using hts::mini_cfo::Mini_CFO;
using hts::rx_cfo::CFO_Result;
using hts::rx_cfo::CFO_V5a;

namespace {
constexpr double kPi = 3.14159265358979323846;

/// Walsh row 63: chip j 부호 = (-1)^{popcount(63 & j)}
static inline int walsh63_chip(int j) noexcept {
    int p = 0;
    int x = 63 & j;
    while (x != 0) {
        p ^= 1;
        x &= (x - 1);
    }
    return (p == 0) ? 1 : -1;
}

/// 양산 PRE_SYM 패턴 (PaCD / T6 preamble과 동일 개념)
/// PRE_SYM 0 (sym=0x3F): Walsh row 63
/// PRE_SYM 1 (sym=0x00): Walsh row 0 → 전 칩 +1
/// preamble 128 chip = PRE_SYM0 (64) + PRE_SYM1 (64)
static inline int pre_sym_chip(int n) noexcept {
    if (n < 64) {
        return walsh63_chip(n);
    }
    return 1;
}
}  // namespace

/// TX 칩 = ±amp (Walsh), I=Q, 채널에 CFO e^{jωn} 적용 후 int16 포화
static void make_production_signal(int16_t* rx_I, int16_t* rx_Q, int n_chips,
                                   int32_t cfo_hz, int amp) noexcept {
    const double omega = 2.0 * kPi * static_cast<double>(cfo_hz) /
                         static_cast<double>(kMiniChipRateHz);
    for (int n = 0; n < n_chips; ++n) {
        const int chip_sign = pre_sym_chip(n);
        const double tx_i = static_cast<double>(amp * chip_sign);
        const double tx_q = static_cast<double>(amp * chip_sign);
        const double phase = omega * static_cast<double>(n);
        const double rx_i = tx_i * std::cos(phase) - tx_q * std::sin(phase);
        const double rx_q = tx_i * std::sin(phase) + tx_q * std::cos(phase);
        long long li = std::llround(rx_i);
        long long lq = std::llround(rx_q);
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
        rx_I[n] = static_cast<int16_t>(li);
        rx_Q[n] = static_cast<int16_t>(lq);
    }
}

/// Walsh dispread: 수신 칩에 PRE_SYM 부호 곱 (양산 V5a 에너지 경로의 work 버퍼 개념)
static void apply_walsh_dispread(const int16_t* rx_I, const int16_t* rx_Q,
                                 int16_t* out_I, int16_t* out_Q,
                                 int n_chips) noexcept {
    for (int n = 0; n < n_chips; ++n) {
        const int sign = pre_sym_chip(n);
        long long li = static_cast<long long>(rx_I[n]) * sign;
        long long lq = static_cast<long long>(rx_Q[n]) * sign;
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

int main() {
    Mini_CFO mini;
    mini.Init();

    int16_t rx_I[kMiniPreambleChips];
    int16_t rx_Q[kMiniPreambleChips];
    int16_t disp_I[kMiniPreambleChips];
    int16_t disp_Q[kMiniPreambleChips];
    int16_t out_I[kMiniPreambleChips];
    int16_t out_Q[kMiniPreambleChips];

    std::printf("=== Mini CFO Production Signal Test ===\n");
    std::printf("TX: Walsh PRE_SYM (row63 + row0), I=Q, amp=1000\n");
    std::printf("\n");

    std::printf("Test 1: Walsh PRE_SYM direct -> Mini.Estimate (no dispread)\n");
    std::printf("%-12s %-15s %-15s %-15s\n", "actual_Hz", "Estimate", "residual",
                "note");
    std::printf("------------------------------------------------------------\n");

    const int32_t scenarios[] = {0,     100,   200,   500,   1000,  5000,
                                 10000, 12000, 20000, 25000};
    const int n_scenarios = static_cast<int>(sizeof(scenarios) / sizeof(scenarios[0]));

    for (int i = 0; i < n_scenarios; ++i) {
        const int32_t cfo = scenarios[i];
        make_production_signal(rx_I, rx_Q, kMiniPreambleChips, cfo, 1000);
        const int32_t est = mini.Estimate(rx_I, rx_Q);
        const int32_t residual = cfo - est;
        const char* note =
            (std::abs(residual) < 100) ? "OK" : "FAIL (|res|>=100)";
        std::printf("%-12d %-15d %-15d %-15s\n", static_cast<int>(cfo),
                    static_cast<int>(est), static_cast<int>(residual), note);
    }

    std::printf("\n");

    std::printf("Test 2: Walsh dispread -> Mini.Estimate (V5a work_I/Q style)\n");
    std::printf("%-12s %-15s %-15s %-15s\n", "actual_Hz", "Estimate", "residual",
                "note");
    std::printf("------------------------------------------------------------\n");

    for (int i = 0; i < n_scenarios; ++i) {
        const int32_t cfo = scenarios[i];
        make_production_signal(rx_I, rx_Q, kMiniPreambleChips, cfo, 1000);
        apply_walsh_dispread(rx_I, rx_Q, disp_I, disp_Q, kMiniPreambleChips);
        const int32_t est = mini.Estimate(disp_I, disp_Q);
        const int32_t residual = cfo - est;
        const char* note =
            (std::abs(residual) < 100) ? "OK" : "FAIL (|res|>=100)";
        std::printf("%-12d %-15d %-15d %-15s\n", static_cast<int>(cfo),
                    static_cast<int>(est), static_cast<int>(residual), note);
    }

    std::printf("\n");

    std::printf("Test 3: dispread + Estimate + Apply(raw) + redispread + re-Estimate\n");
    std::printf("%-12s %-15s %-15s %-15s %-15s\n", "actual_Hz", "Mini_est",
                "res_after_est", "est_after", "total_res");
    std::printf("------------------------------------------------------------\n");

    for (int i = 0; i < n_scenarios; ++i) {
        const int32_t cfo = scenarios[i];
        make_production_signal(rx_I, rx_Q, kMiniPreambleChips, cfo, 1000);
        apply_walsh_dispread(rx_I, rx_Q, disp_I, disp_Q, kMiniPreambleChips);
        const int32_t est = mini.Estimate(disp_I, disp_Q);
        mini.Apply_Per_Chip(rx_I, rx_Q, out_I, out_Q, kMiniPreambleChips, est);
        apply_walsh_dispread(out_I, out_Q, disp_I, disp_Q, kMiniPreambleChips);
        const int32_t est_after = mini.Estimate(disp_I, disp_Q);
        const int32_t total_res = cfo - est - est_after;
        std::printf("%-12d %-15d %-15d %-15d %-15d\n", static_cast<int>(cfo),
                    static_cast<int>(est), static_cast<int>(cfo - est),
                    static_cast<int>(est_after), static_cast<int>(total_res));
    }

    std::printf("\n");
    std::printf("Criteria:\n");
    std::printf("  Test1: many FAIL => Mini L&R expects Walsh-despread input.\n");
    std::printf("  Test2: OK => Walsh dispread + Mini ~= production-shaped chain.\n");
    std::printf("  Test3: total_res ~ 0 => Apply + re-estimate self-consistent.\n");

    std::printf("\n");
    std::printf("=== Mini vs V5a direct compare (isolated) ===\n");
    std::printf("input: Walsh PRE_SYM (row63 + row0), I=Q, amp=1000\n");
    std::printf("PreambleChips=%d, ChipRateHz=%d\n\n", kMiniPreambleChips,
                kMiniChipRateHz);

    CFO_V5a v5a;
    v5a.Init();

    std::printf("%-12s %-12s %-12s %-12s %-12s\n", "actual_Hz", "Mini",
                  "Mini_잔여", "V5a", "V5a_잔여");
    std::printf("------------------------------------------------------------\n");

    const int32_t cmp_hz[] = {0, 100, 200, 500, 1000, 5000, 10000, 12000};
    const int n_cmp = static_cast<int>(sizeof(cmp_hz) / sizeof(cmp_hz[0]));

    for (int i = 0; i < n_cmp; ++i) {
        const int32_t cfo = cmp_hz[i];
        make_production_signal(rx_I, rx_Q, kMiniPreambleChips, cfo, 1000);
        const int32_t mini_est = mini.Estimate(rx_I, rx_Q);
        const CFO_Result v5a_res = v5a.Estimate(rx_I, rx_Q);
        const int32_t v5a_est = v5a_res.cfo_hz;
        std::printf("%-12d %-12d %-12d %-12d %-12d\n", static_cast<int>(cfo),
                    static_cast<int>(mini_est), static_cast<int>(cfo - mini_est),
                    static_cast<int>(v5a_est), static_cast<int>(cfo - v5a_est));
    }

    return 0;
}
