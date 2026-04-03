// =========================================================================
/// @file  HTS_Universal_API.h
/// @brief ProtectedEngine 내부 보안 게이트 / 세션 검증 / 물리적 파쇄
///        + HTS_API 외부 연동 인터페이스 선언
/// @target STM32F407VGT6 (Cortex-M4F, 168MHz)
// =========================================================================
#ifndef HTS_UNIVERSAL_API_H
#define HTS_UNIVERSAL_API_H

#include <cstdint>
#include <cstddef>

namespace ProtectedEngine {

    /// @class Universal_API
    /// @brief BB1_Core_Engine이 호출하는 보안 인터페이스
    class Universal_API {
    public:
        /// 성공: 0xFFFFFFFF — 호출자는 if 분기 대신 & / 산술 마스크로 결합할 것
        static constexpr uint32_t SECURE_GATE_MASK_OK = 0xFFFFFFFFu;
        /// 실패: 0x00000000
        static constexpr uint32_t SECURE_GATE_MASK_FAIL = 0x00000000u;

        /// @brief 세션 ID 기반 보안 게이트 (constant-time 검증, uint32_t 풀마스크 반환)
        [[nodiscard]]
        static uint32_t Secure_Gate_Open(uint64_t session_id) noexcept;

        /// @brief 실시간 세션 무결성 연속 검증 (반환 규격 동일)
        [[nodiscard]]
        static uint32_t Continuous_Session_Verification(uint64_t session_id) noexcept;

        /// @brief 엔트로피 XOR 스크램블 후(1·2단계 사이 DSE 배리어) SecureMemory::secureWipe로 최종 소거 (D-2)
        static void Absolute_Trace_Erasure(void* target, size_t size) noexcept;
    };

} // namespace ProtectedEngine

// =========================================================================
// HTS_API — 외부 연동 인터페이스 (구현부는 HTS_API.cpp)
// =========================================================================
namespace HTS_API {

    enum class HTS_Status : uint32_t {
        OK = 0x00u,
        ERR_ALREADY_INITIALIZED = 0x01u,
        ERR_NULL_POINTER = 0x02u,
        ERR_POST_FAILED = 0x03u,
        ERR_BUFFER_UNDERFLOW = 0x04u,
        ERR_RECOVERY_FAILED = 0x05u,
        ERR_TAMPERED = 0x06u,
        ERR_NOT_INITIALIZED = 0x07u,
        ERR_UNSUPPORTED_MEDIUM = 0x08u
    };

    enum class HTS_CommMedium : uint32_t {
        B_CDMA_RAW_RF = 0x01u,
        DIGITAL_5G_LTE = 0x02u,
        WIRED_ETHERNET = 0x03u,
        SATELLITE_LINK = 0x04u
    };

    HTS_Status Initialize_Core(
        volatile uint32_t* hw_irq_status_reg,
        volatile uint32_t* hw_irq_clear_reg,
        volatile int16_t* hw_rx_fifo_addr,
        HTS_CommMedium     target_medium
    ) noexcept;

    HTS_Status Fetch_And_Heal_Rx_Payload(
        uint32_t* out_buffer, size_t required_size) noexcept;

    HTS_Status Is_System_Operational() noexcept;

} // namespace HTS_API

#endif // HTS_UNIVERSAL_API_H