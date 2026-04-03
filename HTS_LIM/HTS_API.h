// =========================================================================
// [하드웨어 보안 락] ARM Cortex-M 전용
// =========================================================================
#if !defined(__arm__) && !defined(__TARGET_ARCH_ARM) && !defined(STM32F407xx) && !defined(_MSC_VER)
#error "SECURITY: This firmware is licensed for Embedded ARM only."
#endif

// =========================================================================
/// @file  HTS_API.h
/// @brief 외부 파트너사 연동 API 인터페이스
/// @target STM32F407VGT6 (Cortex-M4F)
// =========================================================================
#pragma once
// ─────────────────────────────────────────────────────────
//  외주 업체 통합 가이드
// ─────────────────────────────────────────────────────────
//  [사용법] 기본 사용 예시를 여기에 기재하세요.
//  [메모리] sizeof(클래스명) 확인 후 전역/정적 배치 필수.
//  [보안]   복사/이동 연산자 = delete (키 소재 복제 차단).
//
//  ⚠ [파트너사 필수 확인]
//    HW 레지스터 주소(UART/WDT 등)는 보드 설계에 맞게 교체.
//    IRQ 번호는 STM32F407 RM0090 벡터 테이블 기준으로 교체.
// ─────────────────────────────────────────────────────────

#include <cstdint>
#include <cstddef>

// ARM GCC/Clang 가시성 매크로
#if defined(__GNUC__) || defined(__clang__)
#define HTS_API_EXPORT __attribute__((visibility("default")))
#else
#define HTS_API_EXPORT
#endif

namespace HTS_API {

    // Standard secure tokens for boolean/health checks.
    static constexpr uint32_t SECURE_TRUE = 0x5A5A5A5Au;
    static constexpr uint32_t SECURE_FALSE = 0xA5A5A5A5u;

    enum class HTS_Status : uint32_t {
        OK = 0x00u,
        ERR_ALREADY_INITIALIZED = 0x01u,
        ERR_NULL_POINTER = 0x02u,
        ERR_POST_FAILED = 0x03u,
        ERR_BUFFER_UNDERFLOW = 0x04u,
        ERR_RECOVERY_FAILED = 0x05u,
        ERR_TAMPERED = 0x06u,
        ERR_NOT_INITIALIZED = 0x07u,
        ERR_UNSUPPORTED_MEDIUM = 0x08u,
        /// Unified_Scheduler → DMA 실패 또는 HTS_Tx_Scheduler Push 실패
        ERR_TX_PIPELINE_FAILED = 0x09u
    };

    enum class HTS_CommMedium : uint32_t {
        B_CDMA_RAW_RF = 0x01u,
        DIGITAL_5G_LTE = 0x02u,
        WIRED_ETHERNET = 0x03u,
        SATELLITE_LINK = 0x04u
    };

    // C++ namespace 스코프 + HTS_API_EXPORT로 가시성 보장
    [[nodiscard]] HTS_API_EXPORT
        HTS_Status Initialize_Core(
            volatile uint32_t* hw_irq_status_reg,
            volatile uint32_t* hw_irq_clear_reg,
            volatile int16_t* hw_rx_fifo_addr,
            HTS_CommMedium     target_medium) noexcept;

    [[nodiscard]] HTS_API_EXPORT
        HTS_Status Fetch_And_Heal_Rx_Payload(
            uint32_t* out_buffer,
            size_t    required_size) noexcept;

    [[nodiscard]] HTS_API_EXPORT
        uint32_t Is_System_Operational() noexcept;

    /// 통합 TX 서비스 기본 주기(ms). HTS_Voice_Codec_Bridge_Defs::VocoderProfile::frame_period_ms(23)과 동일 —
    /// 음성 프레임 단위 송신과 맞출 때 그대로 사용. B-CDMA 심볼/칩 타이밍에 맞출 때는
    /// Set_Unified_Tx_Service_Period_Ms 로 심볼 주기(ms) 또는 64칩×칩주기(ms) 등으로 설정.
    inline constexpr uint32_t kDefaultUnifiedTxServicePeriodMs = 23u;

    /// @brief 메인 루프(systick ms)마다 호출 — 내부에서 주기 게이트 후 Schedule_Unified_Tx_And_Queue 수행
    /// @param monotonic_ms HAL_GetTick 등 래핑 없는 ms (unsigned 래핑 허용)
    /// @note period==0 이면 매 호출마다 실행(벤치 전용). 기본 period는 kDefaultUnifiedTxServicePeriodMs.
    /// @note 실센서/ADC 버퍼는 Service_Unified_Tx_If_Due(monotonic_ms, buf, n) 사용.
    [[nodiscard]] HTS_API_EXPORT
        HTS_Status Service_Unified_Tx_If_Due(
            uint32_t    monotonic_ms,
            uint16_t*   sensor_samples,
            size_t      sample_count) noexcept;

    /// @brief 내부 정적 스크래치(1024×uint16)로 Service_Unified_Tx_If_Due 호출 — IPC/콘솔과 동일 메인 틱에서 사용
    /// @details 스크래치는 .bss 0 초기화. 실제 RF/센서 입력을 넣으려면 If_Due(버퍼 직접)로 교체.
    [[nodiscard]] HTS_API_EXPORT
        HTS_Status Service_Unified_Tx_Main_Tick_Default(
            uint32_t monotonic_ms) noexcept;

    HTS_API_EXPORT void Set_Unified_Tx_Service_Period_Ms(
        uint32_t period_ms) noexcept;
    [[nodiscard]] HTS_API_EXPORT uint32_t Get_Unified_Tx_Service_Period_Ms()
        noexcept;
    [[nodiscard]] HTS_API_EXPORT uint32_t Get_Unified_Tx_Service_Run_Count()
        noexcept;
    [[nodiscard]] HTS_API_EXPORT uint32_t Get_Unified_Tx_Service_Skip_Count()
        noexcept;

    /// 듀얼 텐서 → Unified_Scheduler( DMA ) → HTS_Tx_Scheduler 링( Push_Waveform_Chunk )
    /// @pre Initialize_Core 성공( TX 링 초기화 포함 )
    [[nodiscard]] HTS_API_EXPORT
        HTS_Status Schedule_Unified_Tx_And_Queue(
            uint16_t* sensor_samples,
            size_t      sample_count) noexcept;

} // namespace HTS_API
