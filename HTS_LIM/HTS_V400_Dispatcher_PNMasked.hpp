// =============================================================================
// HTS_V400_Dispatcher_PNMasked.hpp — INNOViD V400 PN-masked TX preamble LUT (Step 1)
// Build: /DHTS_USE_PN_MASKED — 미정의 시 본 헤더는 빈 헤더(영향 없음).
//
// 64×128칩 TX LUT: row r → walsh_enc(r)×64 + walsh_enc(0)×64, pre_amp=1000
// (PaCD Step 2 / walsh_enc 와 동일 매핑).
//
// Default Q=I: Q LUT 미저장 → Flash 약 16 KiB (.rodata).
// Row 인덱스 BPTE 클램프 (분기 없음, constant-time). 래퍼 alignas(4) (word/SIMD).
//
// HTS_PN_MASKED_CONSTEXPR_LUT: 1(기본) = compile-time LUT. 0 은 미구현(별도 TU).
// =============================================================================
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <utility>

#if defined(HTS_USE_PN_MASKED)

#ifndef HTS_PN_MASKED_CONSTEXPR_LUT
#define HTS_PN_MASKED_CONSTEXPR_LUT 1
#endif

#if HTS_PN_MASKED_CONSTEXPR_LUT != 1
#error \
    "HTS_PN_MASKED_CONSTEXPR_LUT=0 requires runtime LUT TU (not wired in default builds)"
#endif

namespace detail {

inline constexpr int kPnMaskedNumRows = 64;
inline constexpr int kPnMaskedPreLen = 128;
inline constexpr int16_t kPnMaskedAmp = 1000;

// --- Compile-time 무결성 (오버플로·차원·크기·정렬) ----------------------------

static_assert(kPnMaskedAmp > 0 && kPnMaskedAmp <= 32767,
              "kPnMaskedAmp must fit int16_t positive range");

static_assert(kPnMaskedNumRows == 64,
              "PN-masked design requires exactly 64 Walsh rows");
static_assert(kPnMaskedPreLen == 128,
              "PN-masked preamble length must be 128 chips (PaCD compat)");

using PnMaskedPreambleTable =
    std::array<std::array<int16_t, static_cast<std::size_t>(kPnMaskedPreLen)>,
               static_cast<std::size_t>(kPnMaskedNumRows)>;

static_assert(sizeof(int16_t) * static_cast<std::size_t>(kPnMaskedNumRows) *
                          static_cast<std::size_t>(kPnMaskedPreLen) ==
                  16384u,
              "PN-masked LUT must be exactly 16 KiB (I only, Q=I optimization)");

static_assert(static_cast<int32_t>(kPnMaskedAmp) * 1 <= 32767,
              "amp x (+1 walsh factor) must not overflow int16_t");
static_assert(static_cast<int32_t>(kPnMaskedAmp) * -1 >= -32768,
              "amp x (-1 walsh factor) must not overflow int16_t");

static_assert(std::is_standard_layout_v<PnMaskedPreambleTable>,
              "LUT must be standard-layout for contiguous row-major access");

// --- LUT 생성 (walsh_enc 와 동일 constexpr 칩식) -----------------------------

constexpr uint32_t pn_masked_popc32(uint32_t x) noexcept {
    x = x - ((x >> 1u) & 0x55555555u);
    x = (x & 0x33333333u) + ((x >> 2u) & 0x33333333u);
    return (((x + (x >> 4u)) & 0x0F0F0F0Fu) * 0x01010101u) >> 24u;
}

constexpr int16_t pn_masked_walsh_enc_chip(uint8_t sym, int j,
                                           int32_t amp) noexcept {
    const uint32_t p =
        pn_masked_popc32(static_cast<uint32_t>(sym) &
                          static_cast<uint32_t>(j)) &
        1u;
    return static_cast<int16_t>(amp * (1 - 2 * static_cast<int32_t>(p)));
}

constexpr int32_t kPnMaskedTxPreambleAmp32 =
    static_cast<int32_t>(kPnMaskedAmp);

template <std::size_t RowSym, std::size_t... I>
constexpr std::array<int16_t, 128> pn_masked_make_row128(
    std::index_sequence<I...>) noexcept {
    static_assert(RowSym < 64u, "row is 6-bit Walsh index");
    return std::array<int16_t, 128>{
        {pn_masked_walsh_enc_chip(
            (I < 64u) ? static_cast<uint8_t>(RowSym) : static_cast<uint8_t>(0u),
            static_cast<int>(I % 64u), kPnMaskedTxPreambleAmp32)...}};
}

template <std::size_t... Rows>
constexpr PnMaskedPreambleTable pn_masked_make_lut64(
    std::index_sequence<Rows...>) noexcept {
    return PnMaskedPreambleTable{
        {pn_masked_make_row128<Rows>(std::make_index_sequence<128>{})...}};
}

/// MSVC: nested std::array alignof 가 2일 수 있어 LUT 를 alignas(4) 래퍼에 둔다.
struct alignas(4) PnMaskedTxPreamblePack {
    PnMaskedPreambleTable kPnMaskedTxPreamble;
    constexpr PnMaskedTxPreamblePack() noexcept
        : kPnMaskedTxPreamble(
              pn_masked_make_lut64(std::make_index_sequence<64>{})) {}
};

inline constexpr PnMaskedTxPreamblePack kPnMaskedTxPreamblePack{};

static_assert(alignof(decltype(kPnMaskedTxPreamblePack)) >= 4u,
              "PN-masked LUT pack must be 4-byte aligned for ARM word / SMUAD");

// --- Public API: BPTE row 클램프 (분기 없음, constant-time) ------------------

inline const int16_t* GetPnMaskedPreambleI(int row) noexcept {
    const int32_t below_mask = static_cast<int32_t>(row) >> 31;
    int32_t row_clamped = static_cast<int32_t>(row) & ~below_mask;
    const int32_t above_diff = 63 - row_clamped;
    const int32_t above_mask = above_diff >> 31;
    row_clamped = (row_clamped & ~above_mask) | (63 & above_mask);
    return kPnMaskedTxPreamblePack
        .kPnMaskedTxPreamble[static_cast<std::size_t>(row_clamped)]
        .data();
}

/// Q 기본 모드: I 와 동일 (Q LUT 미저장). HTS_HOLO_PREAMBLE 분기는 향후.
inline const int16_t* GetPnMaskedPreambleQ(int row) noexcept {
    return GetPnMaskedPreambleI(row);
}

}  // namespace detail

#endif  // HTS_USE_PN_MASKED
