// =========================================================================
// HTS_Secure_Memory.cpp
// 보안 메모리 잠금 + 안티포렌식 소거 구현부
// Target: STM32F407 (Cortex-M4)
//
#include "HTS_Secure_Memory.h"
#include <atomic>
#include <cstdint>
#include <climits>
#if defined(__arm__) || defined(__TARGET_ARCH_ARM) || defined(__TARGET_ARCH_THUMB) || defined(__ARM_ARCH)
#include "HTS_Anti_Debug.h"
#include "HTS_Hardware_Init.h"
#endif
#if defined(_MSC_VER)
#include <intrin.h>
#endif

// TBAA: char* 경로와 동일 저장소를 워드 단위로 소거할 때 may_alias로 DCE 방지(GCC/Clang).
#if defined(__GNUC__) || defined(__clang__)
typedef volatile uint32_t __attribute__((__may_alias__)) hts_volatile_wipe_u32_t;
#else
typedef volatile uint32_t hts_volatile_wipe_u32_t;
#endif

#if defined(HTS_SECURE_MPU_LOCK_REGION)
#if HTS_SECURE_MPU_LOCK_REGION > 7u
#error "HTS_SECURE_MPU_LOCK_REGION must be 0..7"
#endif
#if HTS_SECURE_MPU_LOCK_REGION == 7u
#error "Region 7 is low-address guard — pick another HTS_SECURE_MPU_LOCK_REGION"
#endif
#if HTS_SECURE_MPU_LOCK_REGION == 6u
#error "Region 6 is CoreSight — pick another HTS_SECURE_MPU_LOCK_REGION"
#endif
#endif

namespace {

#if defined(__arm__) || defined(__TARGET_ARCH_ARM) || defined(__TARGET_ARCH_THUMB) || defined(__ARM_ARCH)

static constexpr uintptr_t SEC_MPU_RBAR = 0xE000ED9Cu;
static constexpr uintptr_t SEC_MPU_RASR = 0xE000EDA0u;

#if !defined(HTS_SECURE_MEMORY_SKIP_PHYS_TRUST)
#if defined(HTS_ALLOW_OPEN_DEBUG) || !defined(NDEBUG)
#define HTS_SECURE_MEMORY_SKIP_PHYS_TRUST 1
#else
#define HTS_SECURE_MEMORY_SKIP_PHYS_TRUST 0
#endif
#endif

#if HTS_SECURE_MEMORY_SKIP_PHYS_TRUST == 0
[[noreturn]] static void SecureMemory_PhysicalTrust_Fault() noexcept {
    ProtectedEngine::Hardware_Init_Manager::Terminal_Fault_Action();
}

// lockMemory / secureWipe 진입: DHCSR(핫어태치)·OPTCR RDP 이중 샘플 — 완화 시 빌드에서 HTS_ALLOW_OPEN_DEBUG
static void SecureMemory_AssertPhysicalTrustOrFault() noexcept {
    volatile const uint32_t* const dhcsr =
        reinterpret_cast<volatile const uint32_t*>(ProtectedEngine::ADDR_DHCSR);
    const uint32_t d0 = *dhcsr;
#if defined(__GNUC__) || defined(__clang__)
    __asm__ __volatile__("dsb sy" ::: "memory");
#endif
    const uint32_t d1 = *dhcsr;
    if (d0 != d1) {
        SecureMemory_PhysicalTrust_Fault();
    }
    const uint32_t dbg = d0 & ProtectedEngine::DHCSR_DEBUG_MASK;
    if (dbg != 0u) {
        SecureMemory_PhysicalTrust_Fault();
    }
    volatile const uint32_t* const optcr =
        reinterpret_cast<volatile const uint32_t*>(ProtectedEngine::HTS_FLASH_OPTCR_ADDR);
    const uint32_t o0 = *optcr;
#if defined(__GNUC__) || defined(__clang__)
    __asm__ __volatile__("dsb sy" ::: "memory");
#endif
    const uint32_t o1 = *optcr;
    if (o0 != o1) {
        SecureMemory_PhysicalTrust_Fault();
    }
    const uint32_t rdp = (o0 & ProtectedEngine::HTS_RDP_OPTCR_MASK) >> 8u;
    if (rdp != ProtectedEngine::HTS_RDP_EXPECTED_BYTE_VAL) {
        SecureMemory_PhysicalTrust_Fault();
    }
}
#else
static void SecureMemory_AssertPhysicalTrustOrFault() noexcept {}
#endif

#if defined(HTS_SECURE_MPU_LOCK_REGION)

static std::atomic<void*> g_mpu_locked_ptr{nullptr};
static std::atomic<size_t> g_mpu_locked_size{0u};
static std::atomic<uintptr_t> g_mpu_hw_base{0u};

static bool RasrSizeFieldFromPowerOfTwo(uint32_t r, uint32_t& rasr_size_field) noexcept {
    if (r < 32u) {
        return false;
    }
#if (defined(__GNUC__) || defined(__clang__)) && UINT_MAX == 0xffffffffu
    const uint32_t bits =
        31u - static_cast<uint32_t>(__builtin_clz(static_cast<unsigned int>(r)));
#elif defined(_MSC_VER)
    unsigned long idx = 0u;
    if (_BitScanReverse(&idx, static_cast<unsigned long>(r)) == 0) {
        return false;
    }
    const uint32_t bits = static_cast<uint32_t>(idx);
#else
    uint32_t bits = 0u;
    for (uint32_t t = r; t > 1u; t >>= 1u) {
        ++bits;
    }
#endif
    if (bits < 2u) {
        return false;
    }
    rasr_size_field = bits - 1u;
    return true;
}

static bool MpuComputeRegion(uintptr_t ptr, size_t size, uintptr_t& base_out, uint32_t& rasr_size_field) noexcept {
    if (size == 0u) {
        return false;
    }
    uint32_t r = 32u;
    for (;;) {
        const uintptr_t mask = static_cast<uintptr_t>(r) - 1u;
        const uintptr_t base = ptr & ~mask;
        if (base <= ptr && base + static_cast<uintptr_t>(r) >= ptr + size) {
            base_out = base;
            return RasrSizeFieldFromPowerOfTwo(r, rasr_size_field);
        }
        if (r >= 0x80000000u) {
            return false;
        }
        r <<= 1u;
    }
}

#endif

static void MpuHardwareBarrier() noexcept {
#if defined(__GNUC__) || defined(__clang__)
    __asm__ __volatile__("dsb sy\n\tisb" ::: "memory");
#else
    std::atomic_thread_fence(std::memory_order_acq_rel);
#endif
}

#if defined(HTS_SECURE_MPU_LOCK_REGION)

// RBAR VALID=1 → REGION 필드로 RNR 동기 + 베이스 갱신(ARMv7-M). 별도 RNR 쓰기 없음.
static void MpuProgramRegion(uintptr_t base, uint32_t rasr_val) noexcept {
    volatile uint32_t* const rbar =
        reinterpret_cast<volatile uint32_t*>(SEC_MPU_RBAR);
    volatile uint32_t* const rasr =
        reinterpret_cast<volatile uint32_t*>(SEC_MPU_RASR);
    const uint32_t region = static_cast<uint32_t>(HTS_SECURE_MPU_LOCK_REGION) & 0xFu;
    const uint32_t rbar_val =
        (static_cast<uint32_t>(base) & ~0x1Fu) | (1u << 4) | region;
    *rbar = rbar_val;
    *rasr = rasr_val;
    MpuHardwareBarrier();
}

static void MpuDisableDynamicRegion(uintptr_t base) noexcept {
    volatile uint32_t* const rbar =
        reinterpret_cast<volatile uint32_t*>(SEC_MPU_RBAR);
    volatile uint32_t* const rasr =
        reinterpret_cast<volatile uint32_t*>(SEC_MPU_RASR);
    const uint32_t region = static_cast<uint32_t>(HTS_SECURE_MPU_LOCK_REGION) & 0xFu;
    const uint32_t rbar_val =
        (static_cast<uint32_t>(base) & ~0x1Fu) | (1u << 4) | region;
    *rbar = rbar_val;
    *rasr = 0u;
    MpuHardwareBarrier();
}

#endif

#endif

// B-CDMA D-2: volatile 소거 + asm(memory) + release fence. ARM: 소거 전·후 dsb.
// LTO: ptr 입력 의존 asm + noinline. 정렬 구간: hts_volatile_wipe_u32_t(may_alias), mid>>2 / mid&3.
#if defined(__GNUC__) || defined(__clang__)
__attribute__((noinline))
#endif
#if defined(_MSC_VER)
__declspec(noinline)
#endif
static void Force_Secure_Wipe(void* ptr, size_t size) noexcept {
    if (ptr == nullptr || size == 0u) {
        return;
    }

#if defined(__GNUC__) || defined(__clang__)
    __asm__ __volatile__("" ::: "memory");
#elif defined(_MSC_VER)
    _ReadWriteBarrier();
#endif
#if (defined(__GNUC__) || defined(__clang__)) && \
    (defined(__arm__) || defined(__thumb__) || defined(__ARM_ARCH) || \
     defined(__TARGET_ARCH_ARM) || defined(__TARGET_ARCH_THUMB))
    __asm__ __volatile__("dsb sy" ::: "memory");
#endif

    auto* const bytes = static_cast<unsigned char*>(ptr);
    const uintptr_t addr = reinterpret_cast<uintptr_t>(bytes);
    size_t prefix = static_cast<size_t>((4u - (addr & 3u)) & 3u);
    if (prefix > size) {
        prefix = size;
    }

    volatile unsigned char* const vb = static_cast<volatile unsigned char*>(ptr);
    for (size_t i = 0u; i < prefix; ++i) {
        vb[i] = 0x00u;
    }

    const size_t mid = size - prefix;
    hts_volatile_wipe_u32_t* const vw =
        reinterpret_cast<hts_volatile_wipe_u32_t*>(bytes + prefix);
    const size_t words = mid >> 2u;
    for (size_t i = 0u; i < words; ++i) {
        vw[i] = 0u;
    }

    const size_t tail = mid & 3u;
    volatile unsigned char* const vt =
        static_cast<volatile unsigned char*>(bytes + prefix + (words << 2u));
    for (size_t i = 0u; i < tail; ++i) {
        vt[i] = 0x00u;
    }

#if defined(__GNUC__) || defined(__clang__)
    __asm__ __volatile__("" : : "r"(ptr) : "memory");
#elif defined(_MSC_VER)
    _ReadWriteBarrier();
#endif

    std::atomic_thread_fence(std::memory_order_release);
#if (defined(__GNUC__) || defined(__clang__)) && \
    (defined(__arm__) || defined(__thumb__) || defined(__ARM_ARCH) || \
     defined(__TARGET_ARCH_ARM) || defined(__TARGET_ARCH_THUMB))
    __asm__ __volatile__("dsb sy" ::: "memory");
#endif
}

} // namespace

namespace ProtectedEngine {

#if defined(__arm__) || defined(__TARGET_ARCH_ARM) || defined(__TARGET_ARCH_THUMB) || defined(__ARM_ARCH)

#if defined(HTS_SECURE_MPU_LOCK_REGION)

void SecureMemory::lockMemory(void* ptr, size_t size) noexcept {
    if (ptr == nullptr || size == 0u) {
        return;
    }
    SecureMemory_AssertPhysicalTrustOrFault();
    uintptr_t base = 0u;
    uint32_t size_enc = 0u;
    if (!MpuComputeRegion(reinterpret_cast<uintptr_t>(ptr), size, base, size_enc)) {
        return;
    }
    void* exp_null = nullptr;
    if (!g_mpu_locked_ptr.compare_exchange_strong(
            exp_null, ptr, std::memory_order_acq_rel, std::memory_order_acquire)) {
        return;
    }
    g_mpu_locked_size.store(size, std::memory_order_release);
    g_mpu_hw_base.store(base, std::memory_order_release);
    // XN=1, AP=110(특권 RO·비특권 접근 불가), Inner WBWA — Region1과 동일 TEX/S/C/B, SIZE=size_enc
    const uint32_t rasr = (1u << 28) | (6u << 24) | (0u << 19) | (1u << 18) | (1u << 17) | (1u << 16)
        | (size_enc << 1) | (1u);
    MpuProgramRegion(base, rasr);
}

static void UnlockMpuIfMatches(void* ptr, size_t size) noexcept {
    void* cur = g_mpu_locked_ptr.load(std::memory_order_acquire);
    if (cur == nullptr) {
        return;
    }
    if (cur != ptr) {
        return;
    }
    if (g_mpu_locked_size.load(std::memory_order_acquire) != size) {
        return;
    }
    void* exp = ptr;
    if (!g_mpu_locked_ptr.compare_exchange_strong(
            exp, nullptr, std::memory_order_acq_rel, std::memory_order_acquire)) {
        return;
    }
    const uintptr_t base = g_mpu_hw_base.load(std::memory_order_relaxed);
    MpuDisableDynamicRegion(base);
}

#else

void SecureMemory::lockMemory(void* ptr, size_t size) noexcept {
    (void)ptr;
    (void)size;
}

static void UnlockMpuIfMatches(void* /*ptr*/, size_t /*size*/) noexcept {}

#endif

#else

void SecureMemory::lockMemory(void* ptr, size_t size) noexcept {
    (void)ptr;
    (void)size;
}

// 비-ARM: secureWipe에서 Unlock 호출이 #if __arm__로 배제 — 스텁 불필요(C4505).

#endif

#if defined(__GNUC__) || defined(__clang__)
__attribute__((noinline, used))
#endif
#if defined(_MSC_VER)
__declspec(noinline)
#endif
void SecureMemory::secureWipe(void* ptr, size_t size) noexcept {
    if (ptr == nullptr || size == 0u) {
        return;
    }
#if defined(__arm__) || defined(__TARGET_ARCH_ARM) || defined(__TARGET_ARCH_THUMB) || defined(__ARM_ARCH)
    SecureMemory_AssertPhysicalTrustOrFault();
    UnlockMpuIfMatches(ptr, size);
#endif
    Force_Secure_Wipe(ptr, size);
}

} // namespace ProtectedEngine
