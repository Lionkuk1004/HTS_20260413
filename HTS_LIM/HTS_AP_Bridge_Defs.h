#pragma once
// =========================================================================
/// @file  HTS_AP_Bridge_Defs.h
/// @brief M4(AMI/보안 코프로세서) ↔ A55(무선·무거운 로직) 애플리케이션 브리지 계약
///
/// @details
///   HTS B-CDMA 전수검사 통합 기준서 v5 — 0-2 유무선 변환(A55) 역할과 정합.
///   SPI/UART 와이어 프레이밍·CRC·CMD 열거는 **변경하지 않으며**
///   `HTS_IPC_Protocol_Defs.h` 가 단일 소스다.
///
///   본 헤더는 **IPC 페이로드 내부 관례**만 정의한다:
///   - `IPC_Command::DATA_TX` / `DATA_RX` 페이로드가 A55 쪽 애플리케이션으로
///     위임되는 경우, 바이트 0에 `APBridge::ServiceId`, 이후에 서비스별 바디.
///   - M4는 RF·세션·짧은 제어·IPC만 담당; 대용량·집계·콘솔 UI 등은 A55에서 처리.
///
///   초기 핸드셰이크(권장):
///   - A55(SPI 마스터)가 `IPC_Command::AP_CONTRACT_REQ`(페이로드 0바이트) 전송.
///   - M4(SPI 슬레이브)가 즉시 `IPC_Command::AP_CONTRACT_RSP` 및 4바이트 페이로드
///     (`CONTRACT_RSP_MAGIC`, MAJOR, MINOR, reserved) 응답.
///   - A55 앱은 `Parse_Contract_Rsp_Payload`로 검증 후 `CONTRACT_*`와 비교.
///
/// @note  float/double 금지, 힙 없음 — 기준서 0-5와 동일 정책.
// =========================================================================

#include "HTS_IPC_Protocol_Defs.h"

#include <cstdint>

namespace ProtectedEngine {
namespace APBridge {

    /// 앱 계약 버전 (와이어 SYNC/CRC와 무관). A55 앱과 M4가 동일 값을 맞출 것.
    static constexpr uint8_t CONTRACT_MAJOR = 1u;
    static constexpr uint8_t CONTRACT_MINOR = 0u;

    /// DATA_TX / DATA_RX 페이로드 관례: [0]=ServiceId, [1..]=서비스 바디
    enum class ServiceId : uint8_t {
        NONE = 0x00u,
        CONSOLE_TUNNEL = 0x01u,   ///< 통합콘솔 스위치 무선화 터널(바디는 A55 규격)
        WIRELESS_STATUS = 0x02u,  ///< 무선 링크·RSSI 등 짧은 상태
        SECURITY_NOTIFY = 0x03u,  ///< 알림성 보안 이벤트(짧은 페이로드)
        RESERVED_END = 0xFEu
    };

    /// ServiceId 1바이트를 제외한 최대 바디 길이 (IPC 상한과 정합)
    static constexpr uint32_t MAX_SERVICE_BODY =
        (IPC_MAX_PAYLOAD > 0u) ? (IPC_MAX_PAYLOAD - 1u) : 0u;

    static_assert(MAX_SERVICE_BODY + 1u == IPC_MAX_PAYLOAD,
        "APBridge payload convention must match IPC_MAX_PAYLOAD");

    // ── IPC_Command::AP_CONTRACT_RSP 페이로드 (와이어 LEN=4, 바이트 순서 고정) ──
    // [0] CONTRACT_RSP_MAGIC | [1] CONTRACT_MAJOR | [2] CONTRACT_MINOR | [3] reserved(0)
    static constexpr uint8_t CONTRACT_RSP_MAGIC = 0xABu;
    static constexpr uint32_t CONTRACT_HANDSHAKE_BYTES = 4u;

    /// A55 앱: M4가 보낸 AP_CONTRACT_RSP 페이로드 검증 및 버전 추출
    inline bool Parse_Contract_Rsp_Payload(
        const uint8_t* payload, uint32_t len,
        uint8_t& out_major, uint8_t& out_minor) noexcept
    {
        if (payload == nullptr || len < CONTRACT_HANDSHAKE_BYTES) {
            return false;
        }
        if (payload[0] != CONTRACT_RSP_MAGIC) {
            return false;
        }
        out_major = payload[1];
        out_minor = payload[2];
        return true;
    }

} // namespace APBridge
} // namespace ProtectedEngine
