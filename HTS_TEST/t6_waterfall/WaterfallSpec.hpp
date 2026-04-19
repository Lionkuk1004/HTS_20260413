// =============================================================================
// WaterfallSpec.hpp — Waterfall matrix v2 (double statistics 50×50)
// Sweep tables match user spec; total sweep points = 61.
// =============================================================================
#pragma once

#include "../t6_mc/MC_Scenario_Specs.hpp"

#include <cstddef>
#include <cstdint>

namespace WaterfallHarness {

enum class SweepAxis : unsigned char { SNR_dB = 0, JSR_dB = 1 };

struct WaterfallSpec {
    const char* scenario_name;
    MonteCarloHarness::McScenarioKind kind;
    SweepAxis axis;
    double sweep_start_db;
    double sweep_end_db;
    double sweep_step_db;
    double fixed_snr_db;
    double fixed_cfo_hz_std;
    double fixed_cw_freq_min;
    double fixed_cw_freq_max;
    double fixed_period_min;
    double fixed_period_max;
    int fixed_rotate_deg;
    int fixed_timing_jitter;
    bool has_multipath;
    bool apply_awgn;
};

/// 61 sweep points: S1(8) + S3(5) + S7(12) + S8_Freq(12) + S8_Multi(12) + S9(12)
inline constexpr WaterfallSpec kWaterfallSpecs[] = {
    {"WF_S1_Clean", MonteCarloHarness::McScenarioKind::Clean, SweepAxis::SNR_dB, -20.0, 15.0, 5.0,
     0.0, 50.0, 0.0, 0.0, 0.0, 0.0, 0, 0, false, true},
    {"WF_S3_LowSNR", MonteCarloHarness::McScenarioKind::LowSnr, SweepAxis::SNR_dB, -30.0, -10.0,
     5.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0, 0, false, true},
    {"WF_S7_Barrage", MonteCarloHarness::McScenarioKind::Barrage, SweepAxis::JSR_dB, 5.0, 60.0,
     5.0, 0.0, 200.0, 0.0, 0.0, 0.0, 0.0, 0, 0, false, false},
    {"WF_S8_CW_Freq", MonteCarloHarness::McScenarioKind::CwFreqSweep, SweepAxis::JSR_dB, 5.0, 60.0,
     5.0, 0.0, 50.0, 100000.0, 2000000.0, 0.0, 0.0, 0, 0, false, false},
    {"WF_S8_CW_Multi", MonteCarloHarness::McScenarioKind::CwPeriodMulti, SweepAxis::JSR_dB, 5.0,
     60.0, 5.0, 0.0, 100.0, 0.0, 0.0, 8.0, 16.0, 0, 0, false, false},
    {"WF_S9_Combined", MonteCarloHarness::McScenarioKind::CombinedStress, SweepAxis::JSR_dB, 5.0,
     60.0, 5.0, -15.0, 500.0, 100000.0, 2000000.0, 0.0, 0.0, 135, 30, true, true},
};

inline constexpr std::size_t kWaterfallSpecCount =
    sizeof(kWaterfallSpecs) / sizeof(kWaterfallSpecs[0]);

inline constexpr std::uint32_t kMetaSeedPrime = 0x9E3779B9u;

#if defined(HTS_WATERFALL_FAST_SMOKE)
inline constexpr int kMetaLoops = 2;
inline constexpr int kInnerTrials = 2;
#else
inline constexpr int kMetaLoops = 50;
inline constexpr int kInnerTrials = 50;
#endif

inline constexpr int kTotalPerPoint = kMetaLoops * kInnerTrials;

/// Total sweep grid points: 8+5+12+12+12+12 = 61 (S1..S9 per user table).
inline constexpr int kWaterfallTotalSweepPoints = 61;

} // namespace WaterfallHarness
