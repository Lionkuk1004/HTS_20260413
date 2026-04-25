// =============================================================================
// test_v5a_pte_bank.cpp — CFO V5a Bank Parabolic interpolation unit tests
// =============================================================================
#include <cmath>
#include <cstdint>
#include <cstdio>

#include "../HTS_LIM/HTS_CFO_V5a.hpp"
#include "../HTS_LIM/HTS_Rx_CFO_SinCos_Table.hpp"

#if !defined(HTS_ALLOW_HOST_BUILD)
#error "Host-only unit test (HTS_ALLOW_HOST_BUILD required)"
#endif

namespace hts {
namespace rx_cfo {
namespace test_export {
int32_t Parabolic_Offset_Q15(int64_t em1, int64_t e0, int64_t ep1) noexcept;
}  // namespace test_export
}  // namespace rx_cfo
}  // namespace hts

namespace {

int g_fail = 0;

static constexpr int8_t kW63[64] = {
    +1, -1, -1, +1, -1, +1, +1, -1, -1, +1, +1, -1, +1, -1, -1, +1,
    -1, +1, +1, -1, +1, -1, -1, +1, +1, -1, -1, +1, -1, +1, +1, -1,
    -1, +1, +1, -1, +1, -1, -1, +1, +1, -1, -1, +1, -1, +1, +1, -1,
    +1, -1, -1, +1, -1, +1, +1, -1, -1, +1, +1, -1, +1, -1, -1, +1};

void Generate_Preamble_With_CFO(int16_t* preI, int16_t* preQ, int16_t amp,
                                int32_t cfo_hz) noexcept {
    static constexpr double kPi = 3.14159265358979323846;
    const double phase_inc =
        2.0 * kPi * static_cast<double>(cfo_hz) /
        static_cast<double>(hts::rx_cfo::kChipRateHz);
    double phase = 0.0;
    for (int k = 0; k < hts::rx_cfo::kPreambleChips; ++k) {
        const int16_t base =
            static_cast<int16_t>(amp * static_cast<int32_t>(kW63[k & 63]));
        const double c = std::cos(phase);
        const double s = std::sin(phase);
        const long long lR = std::llround(static_cast<double>(base) * c);
        const long long lI = std::llround(static_cast<double>(base) * s);
        preI[k] = (lR > 32767) ? static_cast<int16_t>(32767)
                  : (lR < -32768) ? static_cast<int16_t>(-32768)
                                  : static_cast<int16_t>(lR);
        preQ[k] = (lI > 32767) ? static_cast<int16_t>(32767)
                  : (lI < -32768) ? static_cast<int16_t>(-32768)
                                  : static_cast<int16_t>(lI);
        phase += phase_inc;
    }
}

void Test_1_BinCenter_OffsetZero() noexcept {
    const int32_t off =
        hts::rx_cfo::test_export::Parabolic_Offset_Q15(9000, 10000, 9000);
    if (off != 0) {
        std::printf("FAIL: Test1 center offset=%d\n", static_cast<int>(off));
        ++g_fail;
    } else {
        std::printf("Test 1 PASS: center -> offset 0\n");
    }
}

void Test_2_HalfBin_OffsetQ15() noexcept {
    const int32_t off_p =
        hts::rx_cfo::test_export::Parabolic_Offset_Q15(97750, 99750, 99750);
    const int32_t off_n =
        hts::rx_cfo::test_export::Parabolic_Offset_Q15(99750, 99750, 97750);
    const int32_t e_p = std::abs(off_p - 16384);
    const int32_t e_n = std::abs(off_n + 16384);
    if (e_p > 256 || e_n > 256) {
        std::printf("FAIL: Test2 half-bin off+=%d off-=%d\n",
                    static_cast<int>(off_p), static_cast<int>(off_n));
        ++g_fail;
    } else {
        std::printf("Test 2 PASS: half-bin off+=%d off-=%d\n",
                    static_cast<int>(off_p), static_cast<int>(off_n));
    }
}

void Test_3_Noisy_CFO_Accuracy() noexcept {
    hts::rx_cfo::CFO_V5a cfo;
    cfo.Init();
    int16_t preI[hts::rx_cfo::kPreambleChips]{};
    int16_t preQ[hts::rx_cfo::kPreambleChips]{};
    Generate_Preamble_With_CFO(preI, preQ, 1900, 1375);
    for (int i = 0; i < hts::rx_cfo::kPreambleChips; ++i) {
        const int32_t n = ((i * 17 + 23) % 29) - 14;
        preI[i] = static_cast<int16_t>(preI[i] + n);
        preQ[i] = static_cast<int16_t>(preQ[i] - n);
    }
    const hts::rx_cfo::CFO_Result res = cfo.Estimate(preI, preQ);
    const int32_t err = std::abs(res.cfo_hz - 1375);
    if (!res.valid || err > 120) {
        std::printf("FAIL: Test3 noisy valid=%d est=%d err=%d\n",
                    res.valid ? 1 : 0, static_cast<int>(res.cfo_hz),
                    static_cast<int>(err));
        ++g_fail;
    } else {
        std::printf("Test 3 PASS: noisy est=%d err=%d\n",
                    static_cast<int>(res.cfo_hz), static_cast<int>(err));
    }
}

void Test_4_DenomZero_BPTE() noexcept {
    const int32_t off =
        hts::rx_cfo::test_export::Parabolic_Offset_Q15(12000, 12000, 12000);
    if (off != 0) {
        std::printf("FAIL: Test4 denom0 off=%d\n", static_cast<int>(off));
        ++g_fail;
    } else {
        std::printf("Test 4 PASS: denom0 protected -> offset 0\n");
    }
}

void Test_5_RMS_Regression() noexcept {
    hts::rx_cfo::CFO_V5a cfo;
    cfo.Init();
    int16_t preI[hts::rx_cfo::kPreambleChips]{};
    int16_t preQ[hts::rx_cfo::kPreambleChips]{};
    double s_est2 = 0.0;
    double s_q2 = 0.0;
    int n = 0;
    for (int32_t hz = -4950; hz <= 4950; hz += 75) {
        Generate_Preamble_With_CFO(preI, preQ, 2000, hz);
        const hts::rx_cfo::CFO_Result res = cfo.Estimate(preI, preQ);
        const int32_t q = (hz >= 0) ? ((hz + 150) / 300) * 300
                                    : ((hz - 150) / 300) * 300;
        const double e_est =
            static_cast<double>(res.cfo_hz - hz) * static_cast<double>(res.cfo_hz - hz);
        const double e_q =
            static_cast<double>(q - hz) * static_cast<double>(q - hz);
        s_est2 += e_est;
        s_q2 += e_q;
        ++n;
    }
    const double rms_est = std::sqrt(s_est2 / static_cast<double>(n));
    const double rms_quant = std::sqrt(s_q2 / static_cast<double>(n));
    if (rms_est > 60.0 || rms_est * 3.0 > rms_quant) {
        std::printf("FAIL: Test5 RMS est=%.2f quant=%.2f\n", rms_est, rms_quant);
        ++g_fail;
    } else {
        std::printf("Test 5 PASS: RMS est=%.2f quant=%.2f\n", rms_est, rms_quant);
    }
}

}  // namespace

int main() {
    hts::rx_cfo::Build_SinCos_Table();
    Test_1_BinCenter_OffsetZero();
    Test_2_HalfBin_OffsetQ15();
    Test_3_Noisy_CFO_Accuracy();
    Test_4_DenomZero_BPTE();
    Test_5_RMS_Regression();
    if (g_fail != 0) {
        std::printf("test_v5a_pte_bank: %d failure(s)\n", g_fail);
        return 1;
    }
    std::printf("test_v5a_pte_bank: PASS\n");
    return 0;
}
