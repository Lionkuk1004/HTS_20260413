// V5A_Unit_Test.cpp — [TASK-023] V5a Estimate only (no Sync/Holo/Dispatcher)
#include "HTS_CFO_V5a.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>
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
constexpr int kAmp = 1000;

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

static const char* sign_label(int32_t cfo_in, int32_t total_hz) noexcept
{
    if (cfo_in == 0) {
        return (total_hz == 0) ? "zero_ok" : "off";
    }
    if (total_hz == -27000) {
        return "stuck_-27k";
    }
    if (total_hz == -cfo_in) {
        return "flip";
    }
    const int64_t d = static_cast<int64_t>(total_hz) - static_cast<int64_t>(cfo_in);
    const int64_t ad = (d < 0) ? -d : d;
    if (ad <= 400) {
        return "ok";
    }
    return "off";
}

static void run_sweep_for_chip_rate() noexcept
{
    std::printf("\n=== V5A isolated: %s (synth preamble fs=%.0f Hz) ===\n", kChipTag,
                kFsChipHz);
    std::printf(
        "Note: V5a internal kChipRateHz=1e6; AMI row uses fs=%.0f for phase "
        "generation only.\n",
        kFsChipHz);
    std::printf(
        "CFO_in   cb_idx cb_cfo   fb_idx fineRef  totalHz    err      sign   "
        "valid\n");

    CFO_V5a v5a;
    v5a.Init();
    v5a.SetEnabled(true);

    std::vector<int16_t> preI(static_cast<size_t>(kPreambleChips));
    std::vector<int16_t> preQ(static_cast<size_t>(kPreambleChips));

    for (int32_t cfo_in = -15000; cfo_in <= 15000; cfo_in += 500) {
        const double phase_inc =
            2.0 * kPi * static_cast<double>(cfo_in) / kFsChipHz;
        double phase = 0.0;
        for (int c = 0; c < kPreambleChips; ++c) {
            const int16_t base_I = ((c & 1) != 0) ? static_cast<int16_t>(kAmp)
                                                  : static_cast<int16_t>(-kAmp);
            const int16_t base_Q = 0;
            const double cosp = std::cos(phase);
            const double sinp = std::sin(phase);
            preI[static_cast<size_t>(c)] =
                clamp_i16(static_cast<double>(base_I) * cosp -
                          static_cast<double>(base_Q) * sinp);
            preQ[static_cast<size_t>(c)] =
                clamp_i16(static_cast<double>(base_I) * sinp +
                          static_cast<double>(base_Q) * cosp);
            phase += phase_inc;
        }

        const CFO_Result r = v5a.Estimate(preI.data(), preQ.data());
        const int32_t total_hz = r.cfo_hz;
        const int32_t err = total_hz - cfo_in;

#if defined(HTS_LR_DIAG)
        const int32_t cb_cfo = hts::rx_cfo::g_lr_diag.cb_cfo;
        const int32_t fine_ref = hts::rx_cfo::g_lr_diag.fine_refined;
#else
        const int32_t cb_cfo = 0;
        const int32_t fine_ref = 0;
#endif
        const int cb_idx = v5a.Get_Last_Coarse_Bin();
        const int fb_idx = v5a.Get_Last_Fine_Bin();

        std::printf(
            "CFO %+6d  %3d  %+7d  %2d  %+7d  %+8d  %+7d  %-10s  %d\n",
            static_cast<int>(cfo_in), cb_idx, static_cast<int>(cb_cfo), fb_idx,
            static_cast<int>(fine_ref), static_cast<int>(total_hz),
            static_cast<int>(err), sign_label(cfo_in, total_hz),
            r.valid ? 1 : 0);
    }
}
} // namespace

int main()
{
    std::printf("[TASK-023] V5A_UNIT_TEST build=%s %s\n",
#if defined(HTS_TARGET_AMI)
                "Release_AMI",
#elif defined(HTS_TARGET_PSLTE)
                "Release_PSLTE",
#else
                "generic",
#endif
                kChipTag);
    run_sweep_for_chip_rate();
    return 0;
}

