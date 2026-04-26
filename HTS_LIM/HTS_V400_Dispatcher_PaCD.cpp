// =============================================================================
// HTS_V400_Dispatcher_PaCD.cpp — INNOViD V400 PaCD helper (build-flag gated)
// /DHTS_USE_PACD 시에만 본 TU 가 실질 코드를 포함한다.
// =============================================================================
#include "HTS_V400_Dispatcher_PaCD.hpp"

#if defined(HTS_USE_PACD)

#include "HTS_CFO_V5a.hpp"
#include "HTS_Rx_CFO_SinCos_Table.hpp"
#include "HTS_Secure_Memory.h"

#include <array>
#include <cstdint>
#include <utility>

namespace detail {
namespace {

constexpr uint32_t pc_popc32(uint32_t x) noexcept {
    x = x - ((x >> 1u) & 0x55555555u);
    x = (x & 0x33333333u) + ((x >> 2u) & 0x33333333u);
    return (((x + (x >> 4u)) & 0x0F0F0F0Fu) * 0x01010101u) >> 24u;
}

/// `walsh_enc` 와 동일: T6 `kAmp=1000`, `pre_boost_=1` → preamble `pre_amp=1000`.
constexpr int32_t kPacdTxPreambleAmp = 1000;

constexpr int16_t pc_walsh_enc_chip(uint8_t sym, int j,
                                    int32_t amp) noexcept {
    const uint32_t p =
        pc_popc32(static_cast<uint32_t>(sym) & static_cast<uint32_t>(j)) & 1u;
    return static_cast<int16_t>(amp * (1 - 2 * static_cast<int32_t>(p)));
}

template <std::size_t... I>
constexpr std::array<int16_t, 128> make_tx_preamble128(
    std::index_sequence<I...>) noexcept {
    return std::array<int16_t, 128>{
        {pc_walsh_enc_chip((I < 64u) ? static_cast<uint8_t>(0x3Fu)
                                     : static_cast<uint8_t>(0x00u),
                           static_cast<int>(I % 64u), kPacdTxPreambleAmp)...}};
}

alignas(4) constexpr auto k_tx_preamble128_I =
    make_tx_preamble128(std::make_index_sequence<128>{});
/// Walsh preamble: Q 채널은 `walsh_enc` 와 동일하게 I 와 동일 칩값.
alignas(4) constexpr auto k_tx_preamble128_Q = k_tx_preamble128_I;

static inline int32_t pacd_dot_q14_round(int32_t x, int32_t c_q14, int32_t y,
                                           int32_t s_q14) noexcept {
    const int64_t p = static_cast<int64_t>(x) * static_cast<int64_t>(c_q14) +
                      static_cast<int64_t>(y) * static_cast<int64_t>(s_q14);
    if (p >= 0) {
        return static_cast<int32_t>((p + (1LL << 13)) >> 14);
    }
    return static_cast<int32_t>((p - (1LL << 13)) >> 14);
}

static inline int16_t pacd_sat_i16(int32_t v) noexcept {
    if (v > 32767) {
        return 32767;
    }
    if (v < -32768) {
        return -32768;
    }
    return static_cast<int16_t>(v);
}

/// Q15 위상(π=32768) → `Lookup_Cos` / `Lookup_Sin` 용 Q32 래핑 위상.
static inline uint32_t pacd_phase_q32_from_q15(int32_t phase_q15) noexcept {
    const int64_t p = static_cast<int64_t>(phase_q15);
    return static_cast<uint32_t>((p * (1LL << 31)) >> 15);
}

}  // namespace

void pacd_apply_payload(const int16_t* rx_pre_I, const int16_t* rx_pre_Q,
                        const int16_t* tx_pre_I, const int16_t* tx_pre_Q,
                        int16_t* payload_I, int16_t* payload_Q) noexcept {
    if (rx_pre_I == nullptr || rx_pre_Q == nullptr || tx_pre_I == nullptr ||
        tx_pre_Q == nullptr || payload_I == nullptr || payload_Q == nullptr) {
        return;
    }

    int64_t z_re = 0;
    int64_t z_im = 0;
    for (int c = 0; c < 128; ++c) {
        const int64_t rxI = static_cast<int64_t>(rx_pre_I[c]);
        const int64_t rxQ = static_cast<int64_t>(rx_pre_Q[c]);
        const int64_t txI = static_cast<int64_t>(tx_pre_I[c]);
        const int64_t txQ = static_cast<int64_t>(tx_pre_Q[c]);
        z_re += rxI * txI + rxQ * txQ;
        z_im += rxQ * txI - rxI * txQ;
    }

    int32_t psi_hat_q15 =
        hts::rx_cfo::Atan2_Correlation_Dpte_Q15(z_im, z_re);
    const uint32_t phase_q32 = pacd_phase_q32_from_q15(psi_hat_q15);
    const int32_t cos_psi_q14 =
        static_cast<int32_t>(hts::rx_cfo::Lookup_Cos(phase_q32));
    const int32_t sin_psi_q14 =
        static_cast<int32_t>(hts::rx_cfo::Lookup_Sin(phase_q32));

    for (int c = 0; c < 64; ++c) {
        const int32_t orig_I = static_cast<int32_t>(payload_I[c]);
        const int32_t orig_Q = static_cast<int32_t>(payload_Q[c]);
        payload_I[c] = pacd_sat_i16(
            pacd_dot_q14_round(orig_I, cos_psi_q14, orig_Q, sin_psi_q14));
        payload_Q[c] = pacd_sat_i16(
            pacd_dot_q14_round(orig_Q, cos_psi_q14, orig_I, -sin_psi_q14));
    }

    ProtectedEngine::SecureMemory::secureWipe(static_cast<void*>(&psi_hat_q15),
                                              sizeof(psi_hat_q15));
    ProtectedEngine::SecureMemory::secureWipe(static_cast<void*>(&z_re), sizeof(z_re));
    ProtectedEngine::SecureMemory::secureWipe(static_cast<void*>(&z_im), sizeof(z_im));
}

const int16_t* pacd_tx_preamble128_I() noexcept {
    return k_tx_preamble128_I.data();
}

const int16_t* pacd_tx_preamble128_Q() noexcept {
    return k_tx_preamble128_Q.data();
}

}  // namespace detail

#endif  // HTS_USE_PACD
