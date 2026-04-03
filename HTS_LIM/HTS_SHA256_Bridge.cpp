// =========================================================================
// HTS_SHA256_Bridge.cpp
// FIPS 180-4 SHA-256 래퍼 구현부
// Target: STM32F407 (Cortex-M4) / Cortex-A55 / PC
//
// [KISA SHA-256 C 라이브러리 연결]
//  SHA256_Init → SHA256_Process → SHA256_Close (원샷 API 미사용 — 스택 잔류 방지)
//
// [제약] try-catch 0, float/double 0, heap 0
// [안티 글리칭] fail_pre 시 safe_len=0으로 Process 길이 강제 — 조기 return 우회 시 OOB 방지
// [안티포렌식] SHA256_INFO 선언 직후·Init 직전 secureWipe(패딩 포함) / ARM Release: Hash 진입 시 DHCSR·RDP 폴링
// =========================================================================
#include "HTS_SHA256_Bridge.h"
#include "HTS_Secure_Memory.h"
#if defined(__arm__) || defined(__TARGET_ARCH_ARM) || defined(__TARGET_ARCH_THUMB) || defined(__ARM_ARCH)
#include "HTS_Anti_Debug.h"
#include "HTS_Hardware_Init.h"
#endif
extern "C" {
#include "KISA_SHA256.h"
}
namespace ProtectedEngine {
    static_assert(
        static_cast<size_t>(SHA256_DIGEST_VALUELEN) == SHA256_Bridge::DIGEST_LEN,
        "KISA SHA256_DIGEST_VALUELEN must match DIGEST_LEN");
#if !defined(HTS_SHA256_BRIDGE_SKIP_PHYS_TRUST)
#if defined(HTS_ALLOW_OPEN_DEBUG) || !defined(NDEBUG)
#define HTS_SHA256_BRIDGE_SKIP_PHYS_TRUST 1
#else
#define HTS_SHA256_BRIDGE_SKIP_PHYS_TRUST 0
#endif
#endif
#if HTS_SHA256_BRIDGE_SKIP_PHYS_TRUST == 0 && \
    (defined(__arm__) || defined(__TARGET_ARCH_ARM) || defined(__TARGET_ARCH_THUMB) || defined(__ARM_ARCH))
    [[noreturn]] static void SHA256Bridge_PhysicalTrust_Fault() noexcept {
        Hardware_Init_Manager::Terminal_Fault_Action();
    }
    static void SHA256Bridge_AssertPhysicalTrustOrFault() noexcept {
        volatile const uint32_t* const dhcsr =
            reinterpret_cast<volatile const uint32_t*>(ADDR_DHCSR);
        const uint32_t d0 = *dhcsr;
#if defined(__GNUC__) || defined(__clang__)
        __asm__ __volatile__("dsb sy" ::: "memory");
#endif
        const uint32_t d1 = *dhcsr;
        if (d0 != d1) {
            SHA256Bridge_PhysicalTrust_Fault();
        }
        if ((d0 & DHCSR_DEBUG_MASK) != 0u) {
            SHA256Bridge_PhysicalTrust_Fault();
        }
        volatile const uint32_t* const optcr =
            reinterpret_cast<volatile const uint32_t*>(HTS_FLASH_OPTCR_ADDR);
        const uint32_t o0 = *optcr;
#if defined(__GNUC__) || defined(__clang__)
        __asm__ __volatile__("dsb sy" ::: "memory");
#endif
        const uint32_t o1 = *optcr;
        if (o0 != o1) {
            SHA256Bridge_PhysicalTrust_Fault();
        }
        const uint32_t rdp = (o0 & HTS_RDP_OPTCR_MASK) >> 8u;
        if (rdp != HTS_RDP_EXPECTED_BYTE_VAL) {
            SHA256Bridge_PhysicalTrust_Fault();
        }
    }
#else
    static void SHA256Bridge_AssertPhysicalTrustOrFault() noexcept {}
#endif
    bool SHA256_Bridge::Hash(
        const uint8_t* data, size_t data_len,
        uint8_t* output_32) noexcept {
        SHA256Bridge_AssertPhysicalTrustOrFault();
        const uint32_t ok_out = static_cast<uint32_t>(output_32 != nullptr);
        const uint32_t bad_in =
            static_cast<uint32_t>(data == nullptr)
            & static_cast<uint32_t>(data_len != 0u);
        const uint32_t bad_len = static_cast<uint32_t>(data_len > 0xFFFFFFFFu);
        const uint32_t fail_pre = (1u - ok_out) | bad_in | bad_len;
        SecureMemory::secureWipe(
            output_32,
            static_cast<size_t>(DIGEST_LEN)
                * static_cast<size_t>(fail_pre & ok_out));
        static const uint8_t k_sha256_dummy_src[1] = { 0u };
        const uintptr_t da = reinterpret_cast<uintptr_t>(data);
        const uintptr_t za = reinterpret_cast<uintptr_t>(k_sha256_dummy_src);
        const uint32_t has_d = static_cast<uint32_t>(data != nullptr);
        const uintptr_t ptr_mask = 0u - static_cast<uintptr_t>(has_d);
        const uintptr_t paddr = (da & ptr_mask) | (za & (~ptr_mask));
        const uint8_t* const pmsg = reinterpret_cast<const uint8_t*>(paddr);
        const size_t safe_len =
            data_len * static_cast<size_t>(1u - fail_pre);
        if (fail_pre != 0u) {
            return false;
        }
        SHA256_INFO info;
        SecureMemory::secureWipe(&info, sizeof(info));
        SecureMemory::secureWipe(&info, sizeof(info));
        SHA256_Init(&info);
        SHA256_Process(&info, pmsg, static_cast<UINT>(safe_len));
        SHA256_Close(&info, output_32);
        SecureMemory::secureWipe(&info, sizeof(info));
        return true;
    }
} // namespace ProtectedEngine