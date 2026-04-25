// =============================================================================
// HTS_CFO_V5a.hpp — INNOViD HTS Rx CFO estimator (V5a port, Phase 1+2)
// =============================================================================
#pragma once

#include <cstdint>

namespace hts {
namespace rx_cfo {

#ifndef HTS_CFO_V5A_ENABLE
#define HTS_CFO_V5A_ENABLE 0
#endif

#ifndef HTS_CFO_RANGE_HZ
#define HTS_CFO_RANGE_HZ 12000
#endif

#ifndef HTS_CFO_COARSE_STEP
#define HTS_CFO_COARSE_STEP 3000
#endif

#ifndef HTS_CFO_FINE_STEP
#define HTS_CFO_FINE_STEP 300
#endif

#ifndef HTS_CFO_FINE_BANKS
#define HTS_CFO_FINE_BANKS 11
#endif

#ifndef HTS_CFO_ITER_PASSES
#define HTS_CFO_ITER_PASSES 2
#endif

inline constexpr int kCfoRangeHz = HTS_CFO_RANGE_HZ;
inline constexpr int kCfoCoarseStep = HTS_CFO_COARSE_STEP;
inline constexpr int kCfoFineStep = HTS_CFO_FINE_STEP;
inline constexpr int kCfoFineBanks = HTS_CFO_FINE_BANKS;
inline constexpr int kCfoIterPasses = HTS_CFO_ITER_PASSES;
inline constexpr int kCfoCoarseBanks = (2 * kCfoRangeHz / kCfoCoarseStep) + 1;

inline constexpr int kChipsPerSym = 64;
inline constexpr int kPreambleChips = 128;
inline constexpr int kChipRateHz = 1000000;

inline constexpr int kLR_SegSize = 16;
inline constexpr int kLR_NumSeg = 8;
inline constexpr int kLR_MaxLag = 4;

static_assert(kCfoCoarseBanks >= 3, "Coarse bank count too small");
static_assert(kCfoCoarseBanks <= 64, "Coarse bank count too large");
static_assert(kCfoFineBanks >= 3, "Fine bank count too small");
static_assert(kCfoFineBanks <= 32, "Fine bank count too large");
static_assert(kLR_NumSeg * kLR_SegSize == kPreambleChips,
              "Segment size mismatch");

struct CFO_Result {
    int32_t cfo_hz;
    int64_t peak_energy;
    bool valid;
};

class CFO_V5a {
public:
    CFO_V5a() noexcept;

    void Init() noexcept;

    CFO_Result Estimate(const int16_t* rx_I, const int16_t* rx_Q) noexcept;

    /// Holo P0: lag 자기상관 (ac_I + j·ac_Q) 위상 → Hz → Set_Apply_Cfo.
    /// HTS_CFO_Compensator::Estimate_From_Autocorr 와 동일 Hz 관계 (Dispatcher Step 5).
    void Estimate_From_Autocorr(int32_t ac_I, int32_t ac_Q,
                                int32_t lag_chips) noexcept;

    int32_t GetLastCfoHz() const noexcept { return last_cfo_hz_; }

    int32_t Get_Apply_Sin_Per_Chip_Q14() const noexcept {
        return apply_sin_per_q14_;
    }
    int32_t Get_Apply_Cos_Per_Chip_Q14() const noexcept {
        return apply_cos_per_q14_;
    }

    void SetEnabled(bool en) noexcept { runtime_enabled_ = en; }
    bool IsEnabled() const noexcept { return runtime_enabled_; }

    void ApplyDerotate(const int16_t* in_I, const int16_t* in_Q,
                       int16_t* out_I, int16_t* out_Q, int chips,
                       int32_t cfo_hz) noexcept;

    /// Per-chip CFO 역회전 (HTS_CFO_Compensator::Apply 와 동일 Q14 누적·정규화).
    void Set_Apply_Cfo(int32_t cfo_hz) noexcept;
    /// Hz 대신 Q14 per-chip 직접 지정 (Estimate 직후 `cfo_.Get_*` 값과 비트 정합).
    void Set_Apply_SinCosPerChip_Q14(int32_t sin_per_chip_q14,
                                     int32_t cos_per_chip_q14) noexcept;
    void Reset_Apply_Phase() noexcept;
    void Advance_Phase_Only(int chips) noexcept;
    void Apply_Per_Chip(int16_t& chip_I, int16_t& chip_Q) noexcept;

private:
    int32_t last_cfo_hz_;
    bool runtime_enabled_;
    int16_t work_I_[kPreambleChips];
    int16_t work_Q_[kPreambleChips];
    int64_t fine_energies_[32];

    int32_t apply_cfo_hz_{ 0 };
    int32_t apply_sin_per_q14_{ 0 };
    int32_t apply_cos_per_q14_{ 16384 };
    int32_t apply_cos_acc_q14_{ 16384 };
    int32_t apply_sin_acc_q14_{ 0 };
    int32_t apply_chip_counter_{ 0 };
};

}  // namespace rx_cfo
}  // namespace hts
