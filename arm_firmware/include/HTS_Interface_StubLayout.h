// HTS/3DPOWNS Phase 2 — 가상 외부 규격(스텁). 규격서 확정 시 본 헤더의 상수·필드만 교체.
#pragma once

#include <cstddef>
#include <cstdint>

namespace HTS_Interface_StubLayout {

/// 고정 필드 크기 (가상: [Header 4][Payload N][CRC 2])
inline constexpr std::size_t HEADER_BYTES = 4u;
inline constexpr std::size_t CRC_BYTES = 2u;

/// BSS 정적 버퍼에 올릴 최대 페이로드 (보드 메모리 예산 — 규격 확정 후 조정)
inline constexpr std::size_t MAX_PAYLOAD_BYTES = 512u;

inline constexpr std::size_t MAX_FRAME_BYTES =
    HEADER_BYTES + MAX_PAYLOAD_BYTES + CRC_BYTES;

/// 스텁 매직 (리틀엔디안으로 기록: 'H', 'T', 0x00, 0x00 상위 바이트는 예약)
inline constexpr std::uint16_t STUB_MAGIC = 0x5448u; // 'H' | ('T'<<8) on LE wire as H T

/// 헤더 인코딩: [0..1] magic LE, [2..3] payload_len LE (최대 MAX_PAYLOAD_BYTES)
inline constexpr void Encode_Header(std::uint8_t* dst, std::uint16_t payload_len) noexcept
{
    dst[0] = static_cast<std::uint8_t>(STUB_MAGIC & 0xFFu);
    dst[1] = static_cast<std::uint8_t>((STUB_MAGIC >> 8) & 0xFFu);
    dst[2] = static_cast<std::uint8_t>(payload_len & 0xFFu);
    dst[3] = static_cast<std::uint8_t>((payload_len >> 8) & 0xFFu);
}

inline constexpr bool Decode_Header(
    const std::uint8_t* src,
    std::uint16_t& out_magic,
    std::uint16_t& out_payload_len) noexcept
{
    out_magic = static_cast<std::uint16_t>(
        static_cast<std::uint16_t>(src[0])
        | (static_cast<std::uint16_t>(src[1]) << 8));
    out_payload_len = static_cast<std::uint16_t>(
        static_cast<std::uint16_t>(src[2])
        | (static_cast<std::uint16_t>(src[3]) << 8));
    return (out_magic == STUB_MAGIC)
        && (static_cast<std::size_t>(out_payload_len) <= MAX_PAYLOAD_BYTES);
}

} // namespace HTS_Interface_StubLayout
