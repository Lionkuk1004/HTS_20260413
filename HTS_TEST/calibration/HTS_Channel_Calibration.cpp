// =========================================================================
// HTS_Channel_Calibration.cpp
// 위치: HTS_TEST/calibration/ — PC 전용 (코어 ARM 타깃 CMake 에 미포함)
//
// 물리 채널 본체: HTS_LIM/HTS_Channel_Physics (단일 구현)
// 본 TU는 intensity 매핑 + CW 전대역 옵션만 담당.
// =========================================================================
#if defined(__arm__) || defined(__TARGET_ARCH_ARM) || \
    defined(__TARGET_ARCH_THUMB) || defined(__ARM_ARCH)
#error "[HTS_FATAL] HTS_Channel_Calibration — PC 전용"
#endif

#include "HTS_Channel_Calibration.h"
#include "HTS_Channel_Physics.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace {

using HTS_ChannelCalibrationCoeffs::kAwgnIntensityOffsetDb;
using HTS_ChannelCalibrationCoeffs::kAwgnIntensityScale;
using HTS_ChannelCalibrationCoeffs::kBarrageJsOffsetDb;
using HTS_ChannelCalibrationCoeffs::kBarrageJsScale;
using HTS_ChannelCalibrationCoeffs::kCwJsOffsetDb;
using HTS_ChannelCalibrationCoeffs::kCwJsScale;
using HTS_ChannelCalibrationCoeffs::kEmpPercentOffset;
using HTS_ChannelCalibrationCoeffs::kEmpPercentScale;

double clamp_emp_percent(double x) noexcept {
    if (x < 0.0) { return 0.0; }
    if (x > 100.0) { return 100.0; }
    return x;
}

} // namespace

double HTS_Map_Intensity_Calibrated(
    HTS_CalChannelType type,
    double intensity_db,
    const HTS_ChannelCalibrationOptions& opt) noexcept
{
    if (!opt.apply_intensity_mapping) {
        return intensity_db;
    }
    switch (type) {
    case HTS_CalChannelType::AWGN:
        return intensity_db * kAwgnIntensityScale + kAwgnIntensityOffsetDb;
    case HTS_CalChannelType::BARRAGE:
        return intensity_db * kBarrageJsScale + kBarrageJsOffsetDb;
    case HTS_CalChannelType::CW:
        return intensity_db * kCwJsScale + kCwJsOffsetDb;
    case HTS_CalChannelType::EMP:
        return clamp_emp_percent(
            intensity_db * kEmpPercentScale + kEmpPercentOffset);
    }
    return intensity_db;
}

void HTS_Apply_Channel_Calibrated(
    const std::vector<double>& tx,
    std::mt19937& rng,
    std::vector<double>& rx,
    HTS_CalChannelType type,
    double intensity_db,
    const HTS_ChannelCalibrationOptions& opt)
{
    if (rx.size() != tx.size()) {
        rx.resize(tx.size());
    }
    const double eff = HTS_Map_Intensity_Calibrated(type, intensity_db, opt);

    if (type == HTS_CalChannelType::CW && opt.cw_full_tensor) {
        HTS_Core::Physics::Apply_Cw_Full_Tensor(tx, rng, rx, eff);
        return;
    }

    HTS_Core::Physics::Apply_Parametric_Channel(
        tx, rng, rx,
        static_cast<HTS_Core::Physics::ParametricChannel>(
            static_cast<std::uint8_t>(type)),
        eff);
}
