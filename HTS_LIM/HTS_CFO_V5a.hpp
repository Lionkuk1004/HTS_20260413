// =============================================================================
// HTS_CFO_V5a.hpp — INNOViD HTS Rx CFO estimator (V5a port, Phase 1+2)
// =============================================================================
#pragma once

#include <cstdint>

namespace hts {
namespace rx_cfo {

/// `Estimate` / `Estimate_From_Autocorr` 반환형 — TU 전반에서 완전 형이 먼저
/// 보이도록 `CFO_V5a` 앞에 둔다 (IDE/분석기 incomplete-type 회피).
struct CFO_Result {
    int32_t cfo_hz;
    int64_t peak_energy;
    bool valid;
};

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

/// Lab/양산 선택: 정의 시 `Estimate` 의 L&R 경로가 Walsh-aware M&M DPTE 로 대체됨.
/// 미정의 시 기존 Luise–Reggiannini 세그먼트 LR 유지.
// #define HTS_USE_MNM_WALSH  (빌드 스크립트에서 /DHTS_USE_MNM_WALSH)

#if defined(HTS_CFO_V5A_PTE_DIAG)
void V5a_Pte_Diag_Reset() noexcept;
void V5a_Pte_Diag_Set_Context(const char* tag) noexcept;
void V5a_Pte_Diag_Print_Summary() noexcept;
#endif

#if defined(HTS_LR_DIAG)
/// `LR_Estimate_impl` + `Estimate` 마지막 패스에서만 갱신 (Lab DIAG).
struct LrDiagSnapshot {
    int32_t cb_cfo{ 0 };
    int32_t fine_refined{ 0 };
    int64_t seg_I[8]{};
    int64_t seg_Q[8]{};
    int64_t R_re[kLR_MaxLag]{};
    int64_t R_im[kLR_MaxLag]{};
    int32_t lag_cfo_hz[kLR_MaxLag]{};
    int64_t Z_re{ 0 };
    int64_t Z_im{ 0 };
    int32_t Z_phase_q15{ 0 };
    int32_t lr_cfo_hz{ 0 };
    int32_t cfo_estimate_hz{ 0 };
};
extern LrDiagSnapshot g_lr_diag;
#endif

#if defined(HTS_ALLOW_HOST_BUILD)
namespace test_export {
int32_t LR_Estimate(const int16_t* rI, const int16_t* rQ) noexcept;
#if defined(HTS_USE_MNM_WALSH)
int32_t MnM_Walsh_Estimate_Dpte_Table(const int16_t* rI,
                                    const int16_t* rQ) noexcept;
int32_t Atan2_Dpte_Q15_Table(int64_t y, int64_t x) noexcept;
#endif
}  // namespace test_export
#endif

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

#if defined(HTS_V5A_BUG9_DIAG)
    /// BUG-9 진단: `Estimate` / `Estimate_From_Autocorr` 직후 `last_apply_gate_mag_`.
    int64_t Get_Last_Apply_Gate_Mag() const noexcept { return last_apply_gate_mag_; }
#endif

#if defined(HTS_V5A_TESTONLY)
    /// `Estimate` 마지막 패스의 coarse/fine 에너지 뱅크 인덱스 (Lab 측정 전용).
    int Get_Last_Coarse_Bin() const noexcept { return last_cb_bin_; }
    int Get_Last_Fine_Bin() const noexcept { return last_fb_bin_; }
#endif

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

#if defined(HTS_V5A_TESTONLY)
    int last_cb_bin_{ -1 };
    int last_fb_bin_{ -1 };
#endif
};

#if defined(HTS_USE_PACD)
/// PaCD: preamble correlation Z → DPTE atan2 (Q15, π = 32768). V5a `atan2_dpte_q15` 와 동일 코어.
int32_t Atan2_Correlation_Dpte_Q15(int64_t y, int64_t x) noexcept;
#endif

}  // namespace rx_cfo
}  // namespace hts
