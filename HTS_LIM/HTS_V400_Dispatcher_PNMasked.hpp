// =============================================================================
// HTS_V400_Dispatcher_PNMasked.hpp — INNOViD V400 PN-masked TX preamble LUT (Step 1)
// Build: /DHTS_USE_PN_MASKED — 미정의 시 본 헤더는 빈 헤더(영향 없음).
//
// 64×128칩 compile-time LUT: row r → walsh_enc(r)×64 + walsh_enc(0)×64,
// pre_amp=1000 (PaCD Step 2 / walsh_enc 와 동일 매핑).
// Flash: I+Q 동일 복사본 constexpr 시 약 32 KiB (.rodata).
// =============================================================================
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <utility>

#if defined(HTS_USE_PN_MASKED)

namespace detail {

constexpr int kPnMaskedNumRows = 64;
constexpr int kPnMaskedPreLen = 128;

constexpr uint32_t pn_masked_popc32(uint32_t x) noexcept {
    x = x - ((x >> 1u) & 0x55555555u);
    x = (x & 0x33333333u) + ((x >> 2u) & 0x33333333u);
    return (((x + (x >> 4u)) & 0x0F0F0F0Fu) * 0x01010101u) >> 24u;
}

/// walsh_enc 동일: chip = amp × (1 − 2 × (popc(sym & j) & 1))
constexpr int16_t pn_masked_walsh_enc_chip(uint8_t sym, int j,
                                           int32_t amp) noexcept {
    const uint32_t p =
        pn_masked_popc32(static_cast<uint32_t>(sym) &
                          static_cast<uint32_t>(j)) &
        1u;
    return static_cast<int16_t>(amp * (1 - 2 * static_cast<int32_t>(p)));
}

/// T6 기본 pre_amp (PaCD `kPacdTxPreambleAmp` 와 동일).
constexpr int32_t kPnMaskedTxPreambleAmp = 1000;

template <std::size_t RowSym, std::size_t... I>
constexpr std::array<int16_t, 128> pn_masked_make_row128(
    std::index_sequence<I...>) noexcept {
    static_assert(RowSym < 64u, "row is 6-bit Walsh index");
    return std::array<int16_t, 128>{
        {pn_masked_walsh_enc_chip(
            (I < 64u) ? static_cast<uint8_t>(RowSym) : static_cast<uint8_t>(0u),
            static_cast<int>(I % 64u), kPnMaskedTxPreambleAmp)...}};
}

template <std::size_t... Rows>
constexpr std::array<std::array<int16_t, kPnMaskedPreLen>, sizeof...(Rows)>
pn_masked_make_lut64(std::index_sequence<Rows...>) noexcept {
    return std::array<std::array<int16_t, kPnMaskedPreLen>, sizeof...(Rows)>{
        {pn_masked_make_row128<Rows>(std::make_index_sequence<128>{})...}};
}

alignas(4) inline constexpr auto kPnMaskedTxPreambleI =
    pn_masked_make_lut64(std::make_index_sequence<64>{});

/// Q 기본 모드: I 와 동일 칩열 (PaCD `k_tx_preamble128_Q` 패턴).
alignas(4) inline constexpr auto kPnMaskedTxPreambleQ = kPnMaskedTxPreambleI;

/// row ∈ [0, 63] BPTE 클램프 (비밀값 아닌 LUT 인덱스용).
inline int pn_masked_row_clamp_bpte(int row) noexcept {
    int32_t x = static_cast<int32_t>(row);
    x = x & ~(x >> 31);
    const int32_t over_hi =
        x - static_cast<int32_t>(kPnMaskedNumRows - 1);
    const int32_t mhi = over_hi >> 31;
    return static_cast<int>(
        (kPnMaskedNumRows - 1) + ((x - (kPnMaskedNumRows - 1)) & mhi));
}

inline const int16_t* GetPnMaskedPreambleI(int row) noexcept {
    const int r = pn_masked_row_clamp_bpte(row);
    return kPnMaskedTxPreambleI[static_cast<std::size_t>(r)].data();
}

inline const int16_t* GetPnMaskedPreambleQ(int row) noexcept {
    const int r = pn_masked_row_clamp_bpte(row);
    return kPnMaskedTxPreambleQ[static_cast<std::size_t>(r)].data();
}

}  // namespace detail

#endif  // HTS_USE_PN_MASKED
