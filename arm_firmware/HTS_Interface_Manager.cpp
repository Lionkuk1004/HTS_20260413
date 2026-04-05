// HTS/3DPOWNS Phase 2 — HTS_Interface_Manager 구현 (ARM 껍데기 전용)

#include "HTS_Interface_Manager.h"

#include <atomic>
#include <cstring>

namespace HTS_Interface {

namespace {

namespace SL = HTS_Interface_StubLayout;

/// CRC16-CCITT (poly 0x1021, init 0xFFFF). 비트 시프트만 사용 — '/' , '%' 없음.
[[nodiscard]] std::uint16_t Crc16_Stub(const std::uint8_t* p, std::uint32_t len) noexcept
{
    std::uint32_t crc = 0xFFFFu;
    for (std::uint32_t i = 0u; i < len; ++i) {
        crc ^= static_cast<std::uint32_t>(p[i]) << 8u;
        for (int j = 0; j < 8; ++j) {
            const std::uint32_t high = crc & 0x8000u;
            crc <<= 1u;
            crc &= 0xFFFFu;
            if (high != 0u) {
                crc ^= 0x1021u;
            }
        }
    }
    return static_cast<std::uint16_t>(crc & 0xFFFFu);
}

alignas(4) std::uint8_t g_frame_buffer[SL::MAX_FRAME_BYTES]{};
alignas(4) TxRxRing1024 g_uplink_ring{};
alignas(4) TxRxRing1024 g_downlink_ring{};
/// Payload_Write_Span 예약 길이 — Finalize 시 반드시 일치해야 함 (단일 코어에서 가시성 확보)
alignas(4) std::atomic<std::uint32_t> g_pending_payload_len{0u};

[[nodiscard]] WrappedPacketView FinalizeBuffer(std::uint32_t payload_len) noexcept
{
    WrappedPacketView out{};
    if (payload_len == 0u
        || payload_len > static_cast<std::uint32_t>(SL::MAX_PAYLOAD_BYTES)) {
        return out;
    }
    const std::uint32_t total = static_cast<std::uint32_t>(SL::HEADER_BYTES)
        + payload_len + static_cast<std::uint32_t>(SL::CRC_BYTES);
    SL::Encode_Header(g_frame_buffer, static_cast<std::uint16_t>(payload_len));
    const std::uint8_t* crc_start = g_frame_buffer;
    const std::uint32_t crc_len = static_cast<std::uint32_t>(SL::HEADER_BYTES) + payload_len;
    const std::uint16_t crc = Crc16_Stub(crc_start, crc_len);
    std::uint8_t* crc_dst = g_frame_buffer + SL::HEADER_BYTES
        + static_cast<std::size_t>(payload_len);
    crc_dst[0] = static_cast<std::uint8_t>(crc & 0xFFu);
    crc_dst[1] = static_cast<std::uint8_t>((crc >> 8) & 0xFFu);
    out.data = g_frame_buffer;
    out.len = total;
    out.ok = true;
    return out;
}

} // namespace

WrappedPacketView Wrap_Secure_Packet(
    const std::uint8_t* secure_payload,
    std::uint32_t payload_len) noexcept
{
    g_pending_payload_len.store(0u, std::memory_order_release);
    WrappedPacketView out{};
    if (secure_payload == nullptr || payload_len == 0u
        || payload_len > static_cast<std::uint32_t>(SL::MAX_PAYLOAD_BYTES)) {
        return out;
    }
    std::uint8_t* payload_dst = g_frame_buffer + SL::HEADER_BYTES;
    std::memcpy(
        payload_dst,
        secure_payload,
        static_cast<std::size_t>(payload_len));
    return FinalizeBuffer(payload_len);
}

std::uint8_t* Payload_Write_Span(std::uint32_t payload_len) noexcept
{
    if (payload_len == 0u
        || payload_len > static_cast<std::uint32_t>(SL::MAX_PAYLOAD_BYTES)) {
        g_pending_payload_len.store(0u, std::memory_order_release);
        return nullptr;
    }
    g_pending_payload_len.store(payload_len, std::memory_order_release);
    return g_frame_buffer + SL::HEADER_BYTES;
}

WrappedPacketView Finalize_After_Payload_Write(std::uint32_t payload_len) noexcept
{
    const std::uint32_t pending =
        g_pending_payload_len.load(std::memory_order_acquire);
    if (pending == 0u || payload_len != pending) {
        g_pending_payload_len.store(0u, std::memory_order_release);
        return WrappedPacketView{};
    }
    g_pending_payload_len.store(0u, std::memory_order_release);
    return FinalizeBuffer(payload_len);
}

TxRxRing1024& UplinkRing() noexcept
{
    return g_uplink_ring;
}

TxRxRing1024& DownlinkRing() noexcept
{
    return g_downlink_ring;
}

} // namespace HTS_Interface
