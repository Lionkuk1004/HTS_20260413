// =========================================================================
// HTS_Channel_Calibration.h
// 위치: HTS_TEST/calibration/ — PC 전용 (ARM 펌웨어·hts_cs_core 미포함)
//
// 채널 교정 래퍼 — 물리 본체는 HTS_LIM/HTS_Channel_Physics (SSoT)
// =========================================================================
#pragma once

#if defined(__arm__) || defined(__TARGET_ARCH_ARM) || \
    defined(__TARGET_ARCH_THUMB) || defined(__ARM_ARCH)
#error "[HTS_FATAL] HTS_Channel_Calibration — PC 전용"
#endif

#include <cstdint>
#include <random>
#include <vector>

// -------------------------------------------------------------------------
//  교정 계수 (근거 없는 수치 추측 금지 — 기본은 항등)
//  현장 링크 예산·계측 확정 후 값을 채움. 미확정 항목은 보고서 [요검토].
// -------------------------------------------------------------------------
namespace HTS_ChannelCalibrationCoeffs {

/// AWGN: UI 스윕 intensity_db(SNR dB)에 가산 (dB). [요검토]
inline constexpr double kAwgnIntensityOffsetDb = 0.0;

/// AWGN: 선형 스케일 배율 (intensity_db에 곱함). [요검토]
inline constexpr double kAwgnIntensityScale = 1.0;

/// 바라지: J/S dB 스윕에 가산 (dB). [요검토]
inline constexpr double kBarrageJsOffsetDb = 0.0;

inline constexpr double kBarrageJsScale = 1.0;

/// CW: J/S dB 스윕에 가산 (dB). [요검토]
inline constexpr double kCwJsOffsetDb = 0.0;

inline constexpr double kCwJsScale = 1.0;

/// EMP: UI는 파괴율 % (0~100). 내부 destroy_rate = eff/100.
/// eff 에 가산 (% 포인트). [요검토]
inline constexpr double kEmpPercentOffset = 0.0;

inline constexpr double kEmpPercentScale = 1.0;

} // namespace HTS_ChannelCalibrationCoeffs

// -------------------------------------------------------------------------
//  EMP: “텐서 샘플 파괴 확률 p_sample” 과 “칩 단위 파괴”를 혼동하지 말 것.
//
//  [요검토] 독립 동일 분포(i.i.d.) 가정 시:
//    한 심볼(또는 칩)이 L개 샘플로 표현되고 각 샘플이 독립적으로
//    파괴될 확률이 p_sample 이라면,
//    심볼 전체가 “손상”될 확률은 P_sym = 1 - (1 - p_sample)^L.
//  역변환: p_sample = 1 - (1 - P_sym)^(1/L).
//  실제 파이프라인에서 샘플이 독립인지, 인터리버 후인지에 따라
//  식이 달라질 수 있음 — 스펙 확정 전까지 설계 입력용으로만 사용.
// -------------------------------------------------------------------------

/// A 경로 채널 종류 (종합재밍 ChannelType 과 동일 의미)
enum class HTS_CalChannelType : std::uint8_t {
    AWGN = 0,
    BARRAGE = 1,
    CW = 2,
    EMP = 3
};

struct HTS_ChannelCalibrationOptions {
    /// true: CW 간섭을 텐서 전 인덱스에 주입 (캘리브레이션용 확장 경로)
    /// false: Base(A 동기화)의 부분 대역 CW
    bool cw_full_tensor = false;

    /// intensity 매핑 계수 적용 여부 (false 이면 raw intensity 그대로 Base)
    bool apply_intensity_mapping = true;

    static HTS_ChannelCalibrationOptions Default() noexcept {
        return HTS_ChannelCalibrationOptions{};
    }
};

/// intensity_db 를 교정 계수로 매핑 (채널별)
double HTS_Map_Intensity_Calibrated(
    HTS_CalChannelType type,
    double intensity_db,
    const HTS_ChannelCalibrationOptions& opt) noexcept;

/// 래퍼: 매핑된 intensity + (옵션) CW 전대역 — 내부에서 HTS_Core::Physics 호출
void HTS_Apply_Channel_Calibrated(
    const std::vector<double>& tx,
    std::mt19937& rng,
    std::vector<double>& rx,
    HTS_CalChannelType type,
    double intensity_db,
    const HTS_ChannelCalibrationOptions& opt);
