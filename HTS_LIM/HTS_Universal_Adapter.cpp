// =========================================================================
// HTS_Universal_Adapter.cpp
// 장치 체급별 텐서 자동 최적화 어댑터 구현부
// Target: STM32F407VGT6 (Cortex-M4F, 168MHz)
//
#include "HTS_Universal_Adapter.h"
#include "HTS_Hardware_Init.h"

#include <cstdint>
#include <cstdlib>
#if defined(_MSC_VER)
#include <intrin.h>
#endif

namespace ProtectedEngine {

    // =====================================================================
    //  출처: ARM Cortex-M4 TRM (DDI0439B) 4.3.4 AIRCR
    //  AIRCR 주소  : 0xE000ED0C (SCB→AIRCR, Cortex-M3/M4/M7 공통)
    //  VECTKEY     : 0x05FA0000 (쓰기 시 필수 인증 키, 읽기 시 0xFA05)
    //  SYSRESETREQ : bit[2] = 0x04 (시스템 리셋 요청)
    // =====================================================================
    namespace {
        constexpr uintptr_t AIRCR_ADDR = 0xE000ED0Cu;
        constexpr uint32_t  AIRCR_VECTKEY = 0x05FA0000u;
        constexpr uint32_t  AIRCR_SYSRESETREQ = 0x04u;

        // DeviceType 열거 0..3 과 동일 순서 — 인덱스는 항상 idx & 3u 로만 접근(OOB 불가)
        static constexpr HTS_Sys_Tier k_device_tierLUT[4] = {
            HTS_Sys_Tier::HYPER_SERVER,   // SERVER_STORAGE
            HTS_Sys_Tier::STANDARD_CHIP, // AMI_ENDPOINT
            HTS_Sys_Tier::WORKSTATION,   // CONSOLE_SWITCH
            HTS_Sys_Tier::STANDARD_CHIP, // ROUTER_AP
        };
    } // anonymous namespace

    void HTS_Adapter::Initialize_Device(DeviceType type) noexcept {
        // DCLP 상단 조기 탈출 제거 — 항상 락 획득 시도 후 내부에서만 초기화 여부 판단
        // 스핀 금지: test_and_set 1회만 — 경합 시 초기화 미완이면 터미널 폴트(교착 방지)
        const bool lock_busy =
            m_init_spin.test_and_set(std::memory_order_acquire);
        if (lock_busy) {
            if (m_is_initialized.load(std::memory_order_acquire)) {
                return;
            }
#if defined(__arm__) || defined(__TARGET_ARCH_ARM) || \
    defined(__TARGET_ARCH_THUMB) || defined(__ARM_ARCH)
            Hardware_Init_Manager::Terminal_Fault_Action();
#else
            std::abort();
#endif
        }

        if (m_is_initialized.load(std::memory_order_acquire)) {
            m_init_spin.clear(std::memory_order_release);
            return;
        }

        const uint32_t idx = static_cast<uint32_t>(type);
        const uint32_t bad_type = static_cast<uint32_t>(idx > 3u);
        const uint32_t ok = 1u - bad_type;
        const uint32_t lut_i = idx & 3u;

        const HTS_Sys_Config prof =
            HTS_Sys_Config_Factory::Get_Tier_Profile(k_device_tierLUT[lut_i]);
        // 비정상 idx 시 ok=0 → 필드 전부 0으로 소거(글리치로 분기 우회돼도 쓰레기 프로필 미적용)
        m_active_profile.node_count = prof.node_count * ok;
        m_active_profile.vdf_iterations = prof.vdf_iterations * ok;
        m_active_profile.temporal_slice_chunk = prof.temporal_slice_chunk * ok;
        m_active_profile.anchor_ratio_percent = static_cast<uint8_t>(
            static_cast<uint32_t>(prof.anchor_ratio_percent) * ok);
        m_active_profile.reserved[0] = static_cast<uint8_t>(
            static_cast<uint32_t>(prof.reserved[0]) * ok);
        m_active_profile.reserved[1] = static_cast<uint8_t>(
            static_cast<uint32_t>(prof.reserved[1]) * ok);
        m_active_profile.reserved[2] = static_cast<uint8_t>(
            static_cast<uint32_t>(prof.reserved[2]) * ok);

#if defined(__arm__) || defined(__TARGET_ARCH_ARM) || \
    defined(__TARGET_ARCH_THUMB) || defined(__ARM_ARCH)
#if defined(__GNUC__) || defined(__clang__)
        __asm__ __volatile__("cpsid i" ::: "memory");
        __asm__ __volatile__("dsb" ::: "memory");
#endif
        volatile uint32_t* const aircr = reinterpret_cast<volatile uint32_t*>(
            reinterpret_cast<void*>(static_cast<uintptr_t>(AIRCR_ADDR)));
        const uint32_t rst_word = AIRCR_VECTKEY | AIRCR_SYSRESETREQ;
        const uint32_t msel = 0u - bad_type;
        const uint32_t old_air = *aircr;
        *aircr = (rst_word & msel) | (old_air & ~msel);
#if defined(__GNUC__) || defined(__clang__)
        __asm__ __volatile__("" ::: "memory");
        __asm__ __volatile__("" : : "r"(aircr), "r"(rst_word) : "memory");
        __asm__ __volatile__("dsb" ::: "memory");
        __asm__ __volatile__("isb" ::: "memory");
#elif defined(_MSC_VER)
        _ReadWriteBarrier();
#endif
#else
        (void)bad_type;
        if (ok == 0u) {
            std::abort();
        }
#endif

        m_is_initialized.store(ok != 0u, std::memory_order_release);

#if defined(__arm__) || defined(__TARGET_ARCH_ARM) || \
    defined(__TARGET_ARCH_THUMB) || defined(__ARM_ARCH)
        if (ok != 0u) {
#if defined(__GNUC__) || defined(__clang__)
            __asm__ __volatile__("cpsie i" ::: "memory");
#endif
            m_init_spin.clear(std::memory_order_release);
        } else {
            Hardware_Init_Manager::Terminal_Fault_Action();
        }
#else
        m_init_spin.clear(std::memory_order_release);
#endif
    }

} // namespace ProtectedEngine
