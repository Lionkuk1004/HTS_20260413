// =========================================================================
// [하드웨어 보안 락] ARM Cortex-M 전용
// =========================================================================
#if !defined(__arm__) && !defined(__TARGET_ARCH_ARM) && !defined(STM32F407xx) && !defined(_MSC_VER)
#error "SECURITY: This firmware is licensed for Embedded ARM only."
#endif

// =========================================================================
/// @file  HTS_Universal_Adapter.h
/// @brief 장치 체급별 텐서 자동 최적화 어댑터 (BB1_Core_Engine 래퍼)
/// @target STM32F407VGT6 (Cortex-M4F, 168MHz)
///
/// @warning sizeof(HTS_Adapter) ≈ 82KB (BB1_Core_Engine impl_buf_[81920] 내장)
///          반드시 전역/정적 변수로 배치할 것 — 스택 선언 시 Cortex-M4 즉시 오버플로우
// =========================================================================
#pragma once
#include <cstdint>
#include <cstddef>
#include <atomic>

#include "HTS_Dynamic_Config.h"
#include "BB1_Core_Engine.hpp"

// =========================================================================
//  C++20 [[likely]]/[[unlikely]] — C++14/17에서는 빈 매크로
// =========================================================================
#if __cplusplus >= 202002L
#define HTS_ADAPTER_UNLIKELY [[unlikely]]
#define HTS_ADAPTER_LIKELY   [[likely]]
#else
#define HTS_ADAPTER_UNLIKELY
#define HTS_ADAPTER_LIKELY
#endif

namespace ProtectedEngine {

    enum class DeviceType : uint8_t {
        SERVER_STORAGE = 0u,
        AMI_ENDPOINT = 1u,
        CONSOLE_SWITCH = 2u,
        ROUTER_AP = 3u
    };

    class HTS_Adapter {
    private:
        BB1_Core_Engine     hts_core;
        HTS_Sys_Config      m_active_profile{};
        std::atomic<bool>   m_is_initialized{ false };
        /// Initialize_Device 다중 진입 직렬화
        std::atomic_flag    m_init_spin = ATOMIC_FLAG_INIT;

        static constexpr size_t MAX_ELEMENTS = 4096u;

    public:
        HTS_Adapter() noexcept = default;
        ~HTS_Adapter() noexcept = default;
        HTS_Adapter(const HTS_Adapter&) = delete;
        HTS_Adapter& operator=(const HTS_Adapter&) = delete;
        HTS_Adapter(HTS_Adapter&&) = delete;
        HTS_Adapter& operator=(HTS_Adapter&&) = delete;

        void Initialize_Device(DeviceType type) noexcept;

        /// Universal_API::SECURE_GATE_MASK_* 와 동일 규격 — 호출자는 &·산술 마스크로 결합 가능
        static constexpr uint32_t STREAM_MASK_OK = 0xFFFFFFFFu;
        static constexpr uint32_t STREAM_MASK_FAIL = 0x00000000u;

        /// @brief TX 보안 스트림
        template <typename T>
        [[nodiscard]]
#if defined(__GNUC__) || defined(__clang__)
        __attribute__((always_inline))
#endif
            inline uint32_t Secure_Data_Stream(T* data, size_t size,
                uint64_t session_id) noexcept {
            // acquire: Initialize_Device store(release)와 쌍 — 전제는 uint32_t 마스크 OR (단축 평가 분기 최소화)
            const uint32_t bad_ptr =
                static_cast<uint32_t>(data == nullptr);
            const uint32_t bad_size0 =
                static_cast<uint32_t>(size == 0u);
            const uint32_t bad_init = static_cast<uint32_t>(
                m_is_initialized.load(std::memory_order_acquire) == false);
            const uint32_t bad_range =
                static_cast<uint32_t>(size > MAX_ELEMENTS);
            const uint32_t bad =
                bad_ptr | bad_size0 | bad_init | bad_range;
            if (bad != 0u) {
                return STREAM_MASK_FAIL;
            }
            const uint32_t ok = static_cast<uint32_t>(
                hts_core.Process_Tensor_Pipeline(
                    data, size, session_id,
                    m_active_profile.temporal_slice_chunk,
                    0u, false, false));
            return 0u - ok;
        }

        /// @brief RX 복구 스트림
        template <typename T>
        [[nodiscard]]
#if defined(__GNUC__) || defined(__clang__)
        __attribute__((always_inline))
#endif
            inline uint32_t Recover_Data_Stream(T* data, size_t size,
                uint64_t session_id) noexcept {
            const uint32_t bad_ptr =
                static_cast<uint32_t>(data == nullptr);
            const uint32_t bad_size0 =
                static_cast<uint32_t>(size == 0u);
            const uint32_t bad_init = static_cast<uint32_t>(
                m_is_initialized.load(std::memory_order_acquire) == false);
            const uint32_t bad_range =
                static_cast<uint32_t>(size > MAX_ELEMENTS);
            const uint32_t bad =
                bad_ptr | bad_size0 | bad_init | bad_range;
            if (bad != 0u) {
                return STREAM_MASK_FAIL;
            }
            const uint32_t ok = static_cast<uint32_t>(
                hts_core.Recover_Tensor_Pipeline(
                    data, size, session_id,
                    m_active_profile.temporal_slice_chunk,
                    0u, false, false));
            return 0u - ok;
        }
    };

    // 빌드 타임 SRAM 예산 검증
    // BB1_Core_Engine(~82KB) + HTS_Sys_Config(16B) + atomic(1B) + 패딩
    // DMA SRAM 128KB의 70% ≈ 90KB 이내 (나머지 30%: 스택+ISR+PHY)
    static_assert(sizeof(HTS_Adapter) <= 92160u,
        "HTS_Adapter exceeds 90KB SRAM budget — "
        "BB1_Core_Engine IMPL_BUF_SIZE 또는 멤버 축소 필요");

} // namespace ProtectedEngine