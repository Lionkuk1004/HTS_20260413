// =========================================================================
/// @file HTS_RS_GF16.h
/// @brief GF(2^4) Reed–Solomon RS(15,8), t≤3 오류 정정 (narrow-sense, primitive x^4+x+1)
/// @target STM32F407 / PC — 힙 금지, LUT 고정
///
/// 계약: 심볼은 하위 4비트만 유효(0..15).
/// 다항식 r(x)=Σ_{i=0}^{14} sym[i]·x^i (GF(16) 계수). sym[0..7]=정보부(x^0..x^7), sym[8..14]=패리티(x^8..x^{14}).
/// 신드롼: r(α^m)=0, m=1..7 (narrow-sense).
// =========================================================================
#pragma once

#include <cstdint>

namespace ProtectedEngine {

/// @brief RS(15,8) 인코드 — 데이터 8심볼 → 15심볼 코드워드(제자리 패리티 기입)
void HTS_RS_GF16_Encode15_8(const uint8_t data8[8], uint8_t out15[15]) noexcept;

/// @brief RS(15,8) 디코드 — inout15 정정, 성공 시 true (실패 시 버퍼 불변)
[[nodiscard]] bool HTS_RS_GF16_Decode15_8(uint8_t inout15[15]) noexcept;

} // namespace ProtectedEngine
