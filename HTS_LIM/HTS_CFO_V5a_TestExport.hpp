// =============================================================================
// HTS_CFO_V5a_TestExport.hpp — INNOViD HTS CFO V5a symbols for host unit tests only
// (Not part of production API; link HTS_CFO_V5a.cpp with HTS_ALLOW_HOST_BUILD.)
// =============================================================================
#pragma once

#include <cstdint>

#if defined(HTS_ALLOW_HOST_BUILD)
namespace hts {
namespace rx_cfo {
namespace test_export {

void Derotate_Table(const int16_t* rI, const int16_t* rQ, int16_t* oI,
                    int16_t* oQ, int chips, int32_t cfo_hz) noexcept;

void Walsh63_Dot_Table(const int16_t* rI, const int16_t* rQ, int32_t& dI,
                       int32_t& dQ) noexcept;

int64_t Energy_Multiframe_Table(const int16_t* rI,
                                const int16_t* rQ) noexcept;

int32_t LR_Estimate(const int16_t* rI, const int16_t* rQ) noexcept;

}  // namespace test_export
}  // namespace rx_cfo
}  // namespace hts
#endif
