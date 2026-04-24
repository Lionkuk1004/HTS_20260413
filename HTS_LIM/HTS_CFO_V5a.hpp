// =============================================================================
// HTS_CFO_V5a.hpp — INNOViD HTS Rx CFO estimator (V5a port, Phase 1+2)
// =============================================================================
#pragma once

#include <cstdint>

namespace hts {
namespace rx_cfo {

#ifndef HTS_CFO_V5A_ENABLE
#define HTS_CFO_V5A_ENABLE 1
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

    int32_t GetLastCfoHz() const noexcept { return last_cfo_hz_; }

    void SetEnabled(bool en) noexcept { runtime_enabled_ = en; }
    bool IsEnabled() const noexcept { return runtime_enabled_; }

    void ApplyDerotate(const int16_t* in_I, const int16_t* in_Q,
                       int16_t* out_I, int16_t* out_Q, int chips,
                       int32_t cfo_hz) noexcept;

    /// `Feed_Chip` 경로: 칩 1개, `Derotate_impl` 과 동일한 위상 누적.
    void ResetPayloadDerotatePhase() noexcept;
    /// P0 스캔 192 chip 구간과 동기: `HTS_CFO_Compensator::Advance_Phase_Only`
    /// 가 샘플 없이 위상만 n_chips 전진하는 것과 동일한 Q32 위상 누적.
    void AdvancePayloadDerotatePhase(int n_chips, int32_t cfo_hz) noexcept;
    void ApplyPayloadChip(int16_t& chipI, int16_t& chipQ,
                          int32_t cfo_hz) noexcept;

private:
    int32_t last_cfo_hz_;
    bool runtime_enabled_;
    int16_t work_I_[kPreambleChips];
    int16_t work_Q_[kPreambleChips];
    int64_t fine_energies_[32];
    uint32_t payload_phase_q32_{ 0u };
};

}  // namespace rx_cfo
}  // namespace hts
