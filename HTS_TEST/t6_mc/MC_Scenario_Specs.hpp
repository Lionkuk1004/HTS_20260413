// =============================================================================
// MC_Scenario_Specs.hpp — Monte Carlo harness scenario table (HTS_TEST/t6_mc)
// Each scenario: independent parameter ranges; per-trial values are sampled in
// HTS_T6_MC_Harness.cpp (deterministic from base_seed ^ trial_idx).
// =============================================================================
#pragma once

#include <cstddef>

namespace MonteCarloHarness {

enum class McScenarioKind : unsigned char {
    Clean = 0,
    LowSnr,
    Barrage,
    CwFreqSweep,
    CwPeriodMulti,
    CombinedStress,
};

/// Ranges used by `sample_realization` per `kind`. Unused fields are ignored.
struct ScenarioSpec {
    const char* name;
    const char* description;
    McScenarioKind kind;

    /// AWGN SNR (dB); if min==max==0 and kind needs no noise, AWGN step is skipped.
    double snr_db_min;
    double snr_db_max;

    /// Barrage / narrowband CW jammer power (JSR dB)
    double jsr_barr_min;
    double jsr_barr_max;
    double jsr_cw_min;
    double jsr_cw_max;

    /// CW tone offset frequency (Hz); period_chips = chip_rate / f_hz (clamped).
    double cw_freq_hz_min;
    double cw_freq_hz_max;

    /// Direct CW modulation period in chips (when not using freq sweep).
    double period_chips_min;
    double period_chips_max;

    /// CFO ~ Normal(cfo_hz_mean, cfo_hz_std)
    double cfo_hz_mean;
    double cfo_hz_std;

    int rotate_deg;          ///< 0 or fixed rotation before CFO (e.g. 135)
    int timing_jitter_max;   ///< extra chips on preamble guard: uniform [0, max]

    bool has_multipath;      ///< S9-style 3-tap when true

    bool apply_awgn;         ///< If false, AWGN block is skipped (e.g. barrage-only)
};

inline constexpr ScenarioSpec kMcSpecs[] = {
    {
        "MC_S1_Clean",
        "Clean AWGN + sampled SNR/CFO",
        McScenarioKind::Clean,
        -20.0,
        10.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        50.0,
        0,
        0,
        false,
        true,
    },
    {
        "MC_S3_LowSNR",
        "AWGN low SNR band (no barrage)",
        McScenarioKind::LowSnr,
        -30.0,
        -10.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        100.0,
        0,
        0,
        false,
        true,
    },
    {
        "MC_S7_Barrage",
        "Barrage JSR sweep + CFO (matches T6 S7 spirit, new RNG per trial)",
        McScenarioKind::Barrage,
        0.0,
        0.0,
        0.0,
        30.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        200.0,
        0,
        0,
        false,
        false,
    },
    {
        "MC_S8_CW_Freq",
        "CW tone: JSR sweep + offset frequency sweep (period from f_hz)",
        McScenarioKind::CwFreqSweep,
        0.0,
        0.0,
        0.0,
        0.0,
        10.0,
        25.0,
        100000.0,
        2000000.0,
        0.0,
        0.0,
        0.0,
        50.0,
        0,
        0,
        false,
        false,
    },
    {
        "MC_S8_CW_Multi",
        "CW tone: JSR + period (chips) sweep P~[8,16]",
        McScenarioKind::CwPeriodMulti,
        0.0,
        0.0,
        0.0,
        0.0,
        15.0,
        20.0,
        0.0,
        0.0,
        8.0,
        16.0,
        0.0,
        100.0,
        0,
        0,
        false,
        false,
    },
    {
        "MC_S9_Combined",
        "Barrage + CW + CFO + rotation + ISI + AWGN + random timing (S9-like)",
        McScenarioKind::CombinedStress,
        -15.0,
        -15.0,
        10.0,
        25.0,
        10.0,
        25.0,
        100000.0,
        2000000.0,
        0.0,
        0.0,
        0.0,
        500.0,
        135,
        30,
        true,
        true,
    },
};

inline constexpr std::size_t kMcSpecCount = sizeof(kMcSpecs) / sizeof(kMcSpecs[0]);

} // namespace MonteCarloHarness
