# Phase H Lab Guide

`HTS_CFO_Bank_Test.cpp` is an isolated lab scaffold for fast VS iteration.

## Goal

- Validate CFO sensitivity and sync limits without touching dispatcher mainline.
- Compare S5-like (`HOLO sync OFF`) and S5H-like (`HOLO sync ON`) behavior.
- Try `rx_soft` transforms quickly.

## Files

- `HTS_TEST/HTS_CFO_Bank_Test.cpp`
- `HTS_TEST/t6_sim/cursor_t6_build_holo_tensor_isolated.cmd`

## Build/Run

From `HTS_TEST/t6_sim`:

```bat
cursor_t6_build_holo_tensor_isolated.cmd
```

Output binary:

- `HTS_TEST/t6_sim/out_lab/holo_tensor_lab.exe`

## Main knobs (edit in `ExperimentConfig`)

- `kEnableHoloSync`:
  - `false`: S5-like path
  - `true`: S5H-like path
- `kRxSoftMode`:
  - `0`: `(I+Q)/2`
  - `1`: `I only`
  - `2`: `Q only`
  - `3`: dominant channel
  - `4`: `I+Q`
- `kChipValidThreshold`: BPTE chip-valid threshold (`0` = all valid)
- `kEnableV5aCorrection`
- `kCfoSweepList`, `kSnrLevels`, `kNumTrials`

## Notes

- This lab intentionally avoids dispatcher state machine complexity.
- Treat results as directional for rapid iteration.
- Once a promising setting is found, port to dispatcher and re-verify on T6.
