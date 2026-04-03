// =========================================================================
// HTS_Tamper_HAL.cpp
// 물리적 변조 감지 HAL 구현부
// Target: STM32F407 (Cortex-M4)
//
// [제약] try-catch 0, float/double 0, heap 0
// [보안] 케이스/온도 알람: 함수 포인터 테이블 O(1) 디스패치 (단일 CMP+인덱스 콜)
// [실시간] CPSID/PRIMASK·ADC EOC 스핀 금지 — SWSTART는 s_adc_poll_state 비차단 FSM
// =========================================================================
#include "HTS_Tamper_HAL.h"
#include "HTS_Secure_Logger.h"
#include <atomic>
#include <cstddef>
#include <cstdint>
#if defined(__arm__) || defined(__TARGET_ARCH_ARM) || \
    defined(__TARGET_ARCH_THUMB) || defined(__ARM_ARCH)
#define HTS_TAMPER_ARM
#include "HTS_Anti_Debug.h"
#include "HTS_Hardware_Init.h"
#endif
namespace {
    [[maybe_unused]] static void dummy_noop_cb(ProtectedEngine::TamperEvent) noexcept {}
} // namespace
namespace ProtectedEngine {
#if !defined(HTS_TAMPER_HAL_SKIP_PHYS_TRUST)
#if defined(HTS_ALLOW_OPEN_DEBUG) || !defined(NDEBUG)
#define HTS_TAMPER_HAL_SKIP_PHYS_TRUST 1
#else
#define HTS_TAMPER_HAL_SKIP_PHYS_TRUST 0
#endif
#endif
#if HTS_TAMPER_HAL_SKIP_PHYS_TRUST == 0 && defined(HTS_TAMPER_ARM)
    [[noreturn]] static void TamperHAL_PhysicalTrust_Fault() noexcept {
        Hardware_Init_Manager::Terminal_Fault_Action();
    }
    static void TamperHAL_AssertPhysicalTrustOrFault() noexcept {
        volatile const uint32_t* const dhcsr =
            reinterpret_cast<volatile const uint32_t*>(ADDR_DHCSR);
        const uint32_t d0 = *dhcsr;
#if defined(__GNUC__) || defined(__clang__)
        __asm__ __volatile__("dsb sy" ::: "memory");
#endif
        const uint32_t d1 = *dhcsr;
        if (d0 != d1) {
            TamperHAL_PhysicalTrust_Fault();
        }
        if ((d0 & DHCSR_DEBUG_MASK) != 0u) {
            TamperHAL_PhysicalTrust_Fault();
        }
        volatile const uint32_t* const optcr =
            reinterpret_cast<volatile const uint32_t*>(HTS_FLASH_OPTCR_ADDR);
        const uint32_t o0 = *optcr;
#if defined(__GNUC__) || defined(__clang__)
        __asm__ __volatile__("dsb sy" ::: "memory");
#endif
        const uint32_t o1 = *optcr;
        if (o0 != o1) {
            TamperHAL_PhysicalTrust_Fault();
        }
        const uint32_t rdp = (o0 & HTS_RDP_OPTCR_MASK) >> 8u;
        if (rdp != HTS_RDP_EXPECTED_BYTE_VAL) {
            TamperHAL_PhysicalTrust_Fault();
        }
    }
#else
    // 비-ARM·스킵 구성: TU에 미참조 static을 두지 않음(C4505). 호출부는 HTS_TAMPER_ARM 가드 안.
#define TamperHAL_AssertPhysicalTrustOrFault() ((void)0)
#endif
    static std::atomic<TamperResponseFunc> s_case_response{ nullptr };
    static std::atomic<TamperResponseFunc> s_temp_response{ nullptr };
    static std::atomic<uint32_t> s_case_gpio_port{ 0u };
    static std::atomic<uint8_t>  s_case_pin{ 0u };
    static std::atomic<uint8_t>  s_temp_adc_ch{ 0u };
    static std::atomic<uint16_t> s_temp_high{ 0xFFFFu };
    static std::atomic<uint16_t> s_temp_low{ 0u };
    static std::atomic<uint16_t> s_temp_dma_sample{ 0u };
    static std::atomic<uint32_t> s_temp_dma_sample_ready{ 0u };
    static constexpr uint8_t TAMPER_ADC1_CH_MAX = 18u;
#if defined(HTS_TAMPER_ARM)
    namespace {
        static constexpr uint32_t ADC1_BASE     = 0x40012000u;
        static constexpr uint32_t ADC1_SR_OFF   = 0x00u;
        static constexpr uint32_t ADC1_CR2_OFF  = 0x08u;
        static constexpr uint32_t ADC1_SQR1_OFF = 0x2Cu;
        static constexpr uint32_t ADC1_SQR3_OFF = 0x34u;
        static constexpr uint32_t ADC1_DR_OFF   = 0x4Cu;
        static constexpr uint32_t ADC_SR_EOC         = (1u << 1u);
        static constexpr uint32_t ADC_CR2_ADON       = (1u << 0u);
        static constexpr uint32_t ADC_CR2_DMA        = (1u << 8u);
        static constexpr uint32_t ADC_CR2_SWSTART    = (1u << 30u);
        static constexpr uint32_t ADC_SQR1_L_MASK   = (0xFu << 20u);
        static constexpr uint32_t ADC_SQR3_SQ1_MASK = 0x1Fu;
        static constexpr uint32_t ADC_POLL_IDLE        = 0u;
        static constexpr uint32_t ADC_POLL_PENDING_EOC = 1u;
        static std::atomic<uint32_t> s_adc_poll_state{ ADC_POLL_IDLE };
        using TamperCaseInvokeFn = void(*)(TamperResponseFunc) noexcept;
        static void tamper_case_invoke_noop(TamperResponseFunc) noexcept {}
        static void tamper_case_invoke_fire(TamperResponseFunc cb) noexcept {
            cb(TamperEvent::CASE_OPEN);
        }
        static TamperCaseInvokeFn const k_case_invoke[2] = {
            tamper_case_invoke_noop,
            tamper_case_invoke_fire,
        };
        using TamperTempInvokeFn = void(*)(TamperResponseFunc) noexcept;
        static void tamper_temp_invoke_noop(TamperResponseFunc) noexcept {}
        static void tamper_temp_invoke_high(TamperResponseFunc cb) noexcept {
            cb(TamperEvent::TEMPERATURE_HIGH);
        }
        static void tamper_temp_invoke_low(TamperResponseFunc cb) noexcept {
            cb(TamperEvent::TEMPERATURE_LOW);
        }
        static TamperTempInvokeFn const k_temp_high_invoke[2] = {
            tamper_temp_invoke_noop,
            tamper_temp_invoke_high,
        };
        static TamperTempInvokeFn const k_temp_low_invoke[2] = {
            tamper_temp_invoke_noop,
            tamper_temp_invoke_low,
        };
        static void tamper_poll_adc_dma(uint16_t& raw, uint32_t& raw_valid) noexcept {
            s_adc_poll_state.store(ADC_POLL_IDLE, std::memory_order_relaxed);
            const uint32_t had = s_temp_dma_sample_ready.exchange(
                0u, std::memory_order_acq_rel);
            raw_valid = had;
            raw = s_temp_dma_sample.load(std::memory_order_relaxed);
        }
        static void tamper_poll_adc_sw_fsm(
            volatile uint32_t* adc_sr,
            volatile uint32_t* adc_cr2,
            volatile uint32_t* adc_sqr1,
            volatile uint32_t* adc_sqr3,
            volatile uint32_t* adc_dr,
            uint8_t ch,
            uint16_t& raw,
            uint32_t& raw_valid) noexcept
        {
            raw_valid = 0u;
            if (((*adc_cr2) & ADC_CR2_ADON) == 0u) {
                s_adc_poll_state.store(ADC_POLL_IDLE, std::memory_order_relaxed);
                return;
            }
            const uint32_t st =
                s_adc_poll_state.load(std::memory_order_relaxed);
            if (st == ADC_POLL_PENDING_EOC) {
                if (((*adc_sr) & ADC_SR_EOC) == 0u) {
                    return;
                }
                raw = static_cast<uint16_t>((*adc_dr) & 0xFFFu);
                s_adc_poll_state.store(ADC_POLL_IDLE, std::memory_order_relaxed);
                raw_valid = 1u;
                return;
            }
            if (((*adc_sr) & ADC_SR_EOC) != 0u) {
                (void)static_cast<uint32_t>(*adc_dr);
            }
            uint32_t r_sqr1 = *adc_sqr1;
            r_sqr1 &= ~ADC_SQR1_L_MASK;
            r_sqr1 |= (0u << 20u);
            *adc_sqr1 = r_sqr1;
            uint32_t r_sqr3 = *adc_sqr3;
            r_sqr3 &= ~ADC_SQR3_SQ1_MASK;
            r_sqr3 |= static_cast<uint32_t>(ch) & ADC_SQR3_SQ1_MASK;
            *adc_sqr3 = r_sqr3;
            *adc_cr2 |= ADC_CR2_SWSTART;
            s_adc_poll_state.store(ADC_POLL_PENDING_EOC, std::memory_order_relaxed);
        }
        using AdcPollPathFn = void(*)(
            volatile uint32_t* adc_sr,
            volatile uint32_t* adc_cr2,
            volatile uint32_t* adc_sqr1,
            volatile uint32_t* adc_sqr3,
            volatile uint32_t* adc_dr,
            uint8_t ch,
            uint16_t& raw,
            uint32_t& raw_valid) noexcept;
        static void adc_poll_path_sw(
            volatile uint32_t* adc_sr,
            volatile uint32_t* adc_cr2,
            volatile uint32_t* adc_sqr1,
            volatile uint32_t* adc_sqr3,
            volatile uint32_t* adc_dr,
            uint8_t ch,
            uint16_t& raw,
            uint32_t& raw_valid) noexcept
        {
            tamper_poll_adc_sw_fsm(
                adc_sr, adc_cr2, adc_sqr1, adc_sqr3, adc_dr, ch, raw, raw_valid);
        }
        static void adc_poll_path_dma(
            volatile uint32_t*,
            volatile uint32_t*,
            volatile uint32_t*,
            volatile uint32_t*,
            volatile uint32_t*,
            uint8_t,
            uint16_t& raw,
            uint32_t& raw_valid) noexcept
        {
            tamper_poll_adc_dma(raw, raw_valid);
        }
        static AdcPollPathFn const k_adc_poll_path[2] = {
            adc_poll_path_sw,
            adc_poll_path_dma,
        };
    } // namespace
#endif
    void Tamper_HAL::Register_Case_Open(
        uint32_t gpio_port, uint8_t pin_number,
        TamperResponseFunc response) noexcept {
        const uint32_t bad =
            static_cast<uint32_t>(response == nullptr)
            | static_cast<uint32_t>(gpio_port == 0u)
            | static_cast<uint32_t>(pin_number > 15u);
        if (bad != 0u) {
            return;
        }
        s_case_gpio_port.store(gpio_port, std::memory_order_relaxed);
        s_case_pin.store(pin_number, std::memory_order_relaxed);
#if defined(HTS_TAMPER_ARM)
        volatile uint32_t* const moder = reinterpret_cast<volatile uint32_t*>(
            static_cast<uintptr_t>(gpio_port) + 0x00u);
        const uint32_t shift = static_cast<uint32_t>(pin_number) * 2u;
        *moder &= ~(3u << shift);
#endif
        s_case_response.store(response, std::memory_order_release);
        SecureLogger::logSecurityEvent(
            "TAMPER_REG", "Case-open switch registered.");
    }
    void Tamper_HAL::Register_Temperature_Monitor(
        uint8_t adc_channel,
        uint16_t high_limit, uint16_t low_limit,
        TamperResponseFunc response) noexcept {
        const uint32_t bad =
            static_cast<uint32_t>(response == nullptr)
            | static_cast<uint32_t>(high_limit < low_limit)
            | static_cast<uint32_t>(adc_channel > TAMPER_ADC1_CH_MAX);
        if (bad != 0u) {
            return;
        }
        s_temp_adc_ch.store(adc_channel, std::memory_order_relaxed);
        s_temp_high.store(high_limit, std::memory_order_relaxed);
        s_temp_low.store(low_limit, std::memory_order_relaxed);
        s_temp_response.store(response, std::memory_order_release);
        SecureLogger::logSecurityEvent(
            "TAMPER_REG", "Temperature monitor registered.");
    }
    void Tamper_HAL::Submit_Temperature_ADC_Sample(uint16_t raw12) noexcept {
#if defined(HTS_TAMPER_ARM)
        s_temp_dma_sample.store(raw12 & 0xFFFu, std::memory_order_relaxed);
        s_temp_dma_sample_ready.store(1u, std::memory_order_release);
#else
        (void)raw12;
#endif
    }
    void Tamper_HAL::Poll_Tamper_Status() noexcept {
#if defined(HTS_TAMPER_ARM)
        TamperHAL_AssertPhysicalTrustOrFault();
        TamperResponseFunc const case_cb =
            s_case_response.load(std::memory_order_acquire);
        const uint32_t gpio_port =
            s_case_gpio_port.load(std::memory_order_relaxed);
        const uint8_t pin = s_case_pin.load(std::memory_order_relaxed);
        const uint32_t case_arm =
            static_cast<uint32_t>(case_cb != nullptr)
            & static_cast<uint32_t>(gpio_port != 0u);
        uint32_t opened = 0u;
        if (case_arm != 0u) {
            volatile uint32_t* const idr = reinterpret_cast<volatile uint32_t*>(
                static_cast<uintptr_t>(gpio_port) + 0x10u);
            const uint32_t pin_mask = 1u << static_cast<uint32_t>(pin);
            opened = static_cast<uint32_t>(((*idr) & pin_mask) != 0u);
        }
        const uint32_t case_idx = opened & case_arm;
        k_case_invoke[case_idx](case_cb);
        TamperResponseFunc const temp_cb =
            s_temp_response.load(std::memory_order_acquire);
        const uint32_t temp_arm = static_cast<uint32_t>(temp_cb != nullptr);
        if (temp_arm == 0u) {
            return;
        }
        const uint16_t th = s_temp_high.load(std::memory_order_relaxed);
        const uint16_t tl = s_temp_low.load(std::memory_order_relaxed);
        const uint8_t  ch = s_temp_adc_ch.load(std::memory_order_relaxed);
        volatile uint32_t* const adc_sr =
            reinterpret_cast<volatile uint32_t*>(
                static_cast<uintptr_t>(ADC1_BASE) + ADC1_SR_OFF);
        volatile uint32_t* const adc_cr2 =
            reinterpret_cast<volatile uint32_t*>(
                static_cast<uintptr_t>(ADC1_BASE) + ADC1_CR2_OFF);
        volatile uint32_t* const adc_sqr1 =
            reinterpret_cast<volatile uint32_t*>(
                static_cast<uintptr_t>(ADC1_BASE) + ADC1_SQR1_OFF);
        volatile uint32_t* const adc_sqr3 =
            reinterpret_cast<volatile uint32_t*>(
                static_cast<uintptr_t>(ADC1_BASE) + ADC1_SQR3_OFF);
        volatile uint32_t* const adc_dr =
            reinterpret_cast<volatile uint32_t*>(
                static_cast<uintptr_t>(ADC1_BASE) + ADC1_DR_OFF);
        uint16_t raw = 0u;
        uint32_t raw_valid = 0u;
        const uint32_t cr2v  = *adc_cr2;
        const uint32_t dma_on = (cr2v & ADC_CR2_DMA) >> 8u;
        k_adc_poll_path[dma_on](
            adc_sr, adc_cr2, adc_sqr1, adc_sqr3, adc_dr, ch, raw, raw_valid);
        const uint32_t hi_ev = static_cast<uint32_t>(raw > th) & raw_valid;
        const uint32_t lo_ev = static_cast<uint32_t>(raw < tl) & raw_valid;
        const uint32_t hi_idx = hi_ev & temp_arm;
        const uint32_t lo_idx = lo_ev & temp_arm;
        k_temp_high_invoke[hi_idx](temp_cb);
        k_temp_low_invoke[lo_idx](temp_cb);
#endif
    }
    void Tamper_HAL::Case_Open_ISR() noexcept {
#if defined(HTS_TAMPER_ARM)
        TamperHAL_AssertPhysicalTrustOrFault();
        TamperResponseFunc const cb =
            s_case_response.load(std::memory_order_acquire);
        const uint32_t arm = static_cast<uint32_t>(cb != nullptr);
        k_case_invoke[arm](cb);
#else
        TamperResponseFunc const cb =
            s_case_response.load(std::memory_order_acquire);
        if (cb != nullptr) {
            cb(TamperEvent::CASE_OPEN);
        }
#endif
    }
} // namespace ProtectedEngine
