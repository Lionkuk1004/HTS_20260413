// HTS/3DPOWNS Phase 2 — 외부망(DSN/DLMS 등) 연동용 인터페이스 계층 (코어 비수정).
#pragma once

#include "HTS_Interface_StubLayout.h"
#include "HTS_LockFree_Ring.h"

#include <cstddef>
#include <cstdint>

namespace HTS_Interface {

/// 래핑 결과: `data`는 정적 BSS 버퍼 내부 — 호출자는 복사 없이 TX DMA 등에 전달 가능(Zero 새 할당).
struct WrappedPacketView final
{
    const std::uint8_t* data{nullptr};
    std::uint32_t len{0u};
    bool ok{false};
};

/// 보안 페이로드를 스텁 규격 프레임으로 묶음. malloc 없음; 내부 정적 버퍼 1개만 사용.
/// 페이로드가 별도 버퍼에 있을 때 1회 memcpy(불가피). 제로카피 경로는 `Payload_Write_Span()` 사용.
[[nodiscard]] WrappedPacketView Wrap_Secure_Packet(
    const std::uint8_t* secure_payload,
    std::uint32_t payload_len) noexcept;

/// 정적 프레임 버퍼에서 페이로드 구간 포인터 (헤더 직후). 길이 `payload_len <= MAX_PAYLOAD`.
/// 코어/암호화 출력을 이 구간에 직접 쓴 뒤 `Finalize_After_Payload_Write()` 호출.
[[nodiscard]] std::uint8_t* Payload_Write_Span(std::uint32_t payload_len) noexcept;

/// 페이로드 길이 확정 후 헤더·CRC 기록. 성공 시 `Wrap_Secure_Packet`과 동일 뷰 반환.
[[nodiscard]] WrappedPacketView Finalize_After_Payload_Write(
    std::uint32_t payload_len) noexcept;

/// 송신·수신 파이프 예시: 1KiB SPSC 링 (필요 시 별도 인스턴스 추가).
using TxRxRing1024 = HTS_LockFree::ByteRingSpsc<1024u>;

[[nodiscard]] TxRxRing1024& UplinkRing() noexcept;
[[nodiscard]] TxRxRing1024& DownlinkRing() noexcept;

} // namespace HTS_Interface
