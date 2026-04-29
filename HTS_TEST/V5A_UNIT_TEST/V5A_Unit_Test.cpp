// V5A_Unit_Test.cpp — [TASK-024] V5a + 양산 Walsh preamble (PRE_SYM0/1, TX 동일 walsh_enc 수식)
#include "HTS_CFO_V5a.hpp"
#include "HTS_Preamble_Holographic.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

using hts::rx_cfo::CFO_Result;
using hts::rx_cfo::CFO_V5a;
using hts::rx_cfo::kPreambleChips;

#if defined(HTS_TARGET_AMI)
constexpr const char kChipTag[] = "AMI 200 kcps";
constexpr double kFsChipHz = 200000.0;
#elif defined(HTS_TARGET_PSLTE)
constexpr const char kChipTag[] = "PSLTE 1 Mcps";
constexpr double kFsChipHz = 1000000.0;
#else
constexpr const char kChipTag[] = "Default 1 Mcps";
constexpr double kFsChipHz = 1000000.0;
#endif

namespace {
constexpr double kPi = 3.14159265358979323846;
/// T6 / RX_LINE 기본 amp; pre_boost=1 가정 (`HTS_V400_Dispatcher_TX` 와 동일 스케일).
constexpr int16_t kPreAmp = 1000;
/// `HTS_V400_Dispatcher` PRE_SYM0 / PRE_SYM1 (Walsh row 63, row 0).
constexpr uint8_t kPreSym0 = 0x3Fu;
constexpr uint8_t kPreSym1 = 0x00u;

static constexpr uint32_t popc32_u(uint32_t x) noexcept
{
    x = x - ((x >> 1u) & 0x55555555u);
    x = (x & 0x33333333u) + ((x >> 2u) & 0x33333333u);
    return (((x + (x >> 4u)) & 0x0F0F0F0Fu) * 0x01010101u) >> 24u;
}

/// TX `walsh_enc` ( `HTS_V400_Dispatcher_Internal.hpp` ) 와 동일 비트식.
static void walsh_enc_tx(uint8_t sym, int n, int16_t amp, int16_t* oI,
                         int16_t* oQ) noexcept
{
    const int32_t ampi = static_cast<int32_t>(amp);
    for (int j = 0; j < n; ++j) {
        const uint32_t p =
            popc32_u(static_cast<uint32_t>(sym) & static_cast<uint32_t>(j)) & 1u;
        const int16_t ch =
            static_cast<int16_t>(ampi * (1 - 2 * static_cast<int32_t>(p)));
        oI[j] = ch;
        oQ[j] = ch;
    }
}

static void fill_prod_preamble_128(int16_t* preI, int16_t* preQ) noexcept
{
    walsh_enc_tx(kPreSym0, 64, kPreAmp, preI, preQ);
    walsh_enc_tx(kPreSym1, 64, kPreAmp, preI + 64, preQ + 64);
}

static int16_t clamp_i16(double v) noexcept
{
    if (v > 32767.0) {
        return 32767;
    }
    if (v < -32768.0) {
        return -32768;
    }
    return static_cast<int16_t>(v);
}

static const char* pole_label(int32_t cfo_in, int32_t cfo_hz) noexcept
{
    if (cfo_in == 0) {
        return "ZERO";
    }
    if ((cfo_hz < 0) == (cfo_in < 0)) {
        return "SAME";
    }
    return "OPPOSITE";
}

static void run_sweep() noexcept
{
    std::printf("\n=== [TASK-024] V5A + prod Walsh preamble: %s fs=%.0f ===\n",
                kChipTag, kFsChipHz);
    std::printf(
        "Preamble: walsh_enc(0x3F,64)+walsh_enc(0x00,64) amp=%d; "
        "Holographic: verify_chu + dot(seg) on PRE_SYM0 block.\n",
        static_cast<int>(kPreAmp));

    alignas(8) int16_t clean_I[128];
    alignas(8) int16_t clean_Q[128];
    std::memset(clean_I, 0, sizeof(clean_I));
    std::memset(clean_Q, 0, sizeof(clean_Q));
    fill_prod_preamble_128(clean_I, clean_Q);

    const int32_t chu_ppm =
        ProtectedEngine::Holographic::verify_chu_table_max_err_ppm();
    const int64_t e0 = ProtectedEngine::Holographic::holographic_dot_segmented(
        clean_I, clean_Q);
    std::printf("[HOLO] verify_chu_table_max_err_ppm=%d holographic_dot_seg0=%lld\n",
                static_cast<int>(chu_ppm), static_cast<long long>(e0));

    std::printf("CFO_in  cfo_hz    err     pole      cb_idx fb_idx valid\n");

    CFO_V5a v5a;
    v5a.Init();
    v5a.SetEnabled(true);

    std::vector<int16_t> rotI(static_cast<size_t>(kPreambleChips));
    std::vector<int16_t> rotQ(static_cast<size_t>(kPreambleChips));

    for (int32_t cfo_in = -15000; cfo_in <= 15000; cfo_in += 500) {
        const double phase_inc =
            2.0 * kPi * static_cast<double>(cfo_in) / kFsChipHz;
        double phase = 0.0;
        for (int c = 0; c < kPreambleChips; ++c) {
            const double cosp = std::cos(phase);
            const double sinp = std::sin(phase);
            const int32_t bi = static_cast<int32_t>(clean_I[c]);
            const int32_t bq = static_cast<int32_t>(clean_Q[c]);
            rotI[static_cast<size_t>(c)] =
                clamp_i16(static_cast<double>(bi) * cosp -
                          static_cast<double>(bq) * sinp);
            rotQ[static_cast<size_t>(c)] =
                clamp_i16(static_cast<double>(bi) * sinp +
                          static_cast<double>(bq) * cosp);
            phase += phase_inc;
        }

        const CFO_Result r = v5a.Estimate(rotI.data(), rotQ.data());
        const int32_t hz = r.cfo_hz;
        const int32_t err = hz - cfo_in;
        std::printf("%+6d  %+8d  %+7d  %-8s  cb=%2d fb=%2d  %d\n",
                    static_cast<int>(cfo_in), static_cast<int>(hz),
                    static_cast<int>(err), pole_label(cfo_in, hz),
                    v5a.Get_Last_Coarse_Bin(), v5a.Get_Last_Fine_Bin(),
                    r.valid ? 1 : 0);
    }
}
} // namespace

int main()
{
    std::printf("[TASK-024] V5A_UNIT_TEST %s %s seed=0xC0FFEE (preamble fixed)\n",
#if defined(HTS_TARGET_AMI)
                "Release_AMI",
#elif defined(HTS_TARGET_PSLTE)
                "Release_PSLTE",
#else
                "generic",
#endif
                kChipTag);
    run_sweep();
    return 0;
}
