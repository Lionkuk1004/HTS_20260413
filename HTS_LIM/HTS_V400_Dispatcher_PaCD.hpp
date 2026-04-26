// =============================================================================
// HTS_V400_Dispatcher_PaCD.hpp — INNOViD V400 PaCD (Phase-only) helper
// Build: /DHTS_USE_PACD — 미정의 시 본 헤더의 API 선언은 비활성(TU 영향 없음).
// =============================================================================
#pragma once

#include <cstdint>

#if defined(HTS_USE_PACD)

namespace detail {

/// 128칩 RX preamble × 정적 TX preamble 상관 → ψ_hat → 64칩 페이로드만 derotate.
void pacd_apply_payload(const int16_t* rx_pre_I, const int16_t* rx_pre_Q,
                        const int16_t* tx_pre_I, const int16_t* tx_pre_Q,
                        int16_t* payload_I, int16_t* payload_Q) noexcept;

/// T6 기본 `kAmp=1000`, `pre_boost_=1`, `HTS_HOLO_PREAMBLE` 미정의: Walsh PRE_SYM0(63)+PRE_SYM1(0).
const int16_t* pacd_tx_preamble128_I() noexcept;
const int16_t* pacd_tx_preamble128_Q() noexcept;

}  // namespace detail

#endif  // HTS_USE_PACD
