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

    /// Holo P0: Compensator 와 동일 atan2·Q12(block) / lag → Hz → Set_Apply_Cfo.
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

    /// Per-chip CFO 역회전 (레거시 Q14 per-chip 누적·정규화 경로와 동일 수식).
    void Set_Apply_Cfo(int32_t cfo_hz) noexcept;
    /// Hz 대신 Q14 per-chip 직접 지정 (legacy compensator per-chip Q14 와 비트 정합).
    void Set_Apply_SinCosPerChip_Q14(int32_t sin_per_chip_q14,
                                     int32_t cos_per_chip_q14) noexcept;
    void Reset_Apply_Phase() noexcept;
    void Advance_Phase_Only(int chips) noexcept;
    void Apply_Per_Chip(int16_t& chip_I, int16_t& chip_Q) noexcept;

    /// 게이트: `Estimate` 직후 Walsh P0 는 mag_approx≥1000,
    /// `Estimate_From_Autocorr` 직후 Holo 는 |ac|²+|aq|²≥1e6 과 동등 (Compensator).
    bool IsApplyAllowed() const noexcept;

    /// `Set_Apply_Cfo` / `Set_Apply_SinCosPerChip_Q14` 로 비항등 per-chip 이 설정됨
    /// (Dispatcher 가 legacy `Is_Apply_Active` 없이도 `Apply_Per_Chip` 호출 판단).
    bool IsApplyDriveActive() const noexcept;

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

    /// `Estimate_From_Autocorr`: ac_I²+ac_Q². `Estimate`: preamble mag_approx
    /// (레거시 preamble dot 기반 CFO 추정 식과 동일).
    int64_t last_apply_gate_mag_{ 0 };
    /// true → 임계 1e6 (autocorr), false → 임계 1000 (preamble mag_approx).
    bool last_apply_gate_autocorr_{ false };
};

}  // namespace rx_cfo
}  // namespace hts
