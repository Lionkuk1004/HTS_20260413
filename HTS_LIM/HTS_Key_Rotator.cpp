// =========================================================================
// HTS_Key_Rotator.cpp
// Forward Secrecy 기반 동적 시드 로테이터 구현부
// Target: STM32F407 (Cortex-M4, 168MHz)
//
// [양산 수정 이력 — 17건]
//  BUG-01~15 (이전 세션)
//  BUG-16 [CRIT] try-catch 4블록 제거 + seq_cst→release 3곳
//         · Impl 내부 try-catch 2블록: 정적 배열 전환으로 OOM 소멸
//         · deriveNextSeed try-catch 1블록: 삭제
//         · Secure_Zero_KR/Murmur3/deriveNextSeed seq_cst → release
//  BUG-17 [CRIT] currentSeed vector → uint8_t[32] 정적 배열
//         · 시드 크기 항상 32바이트 고정 (resize(32) 강제)
//         · Impl 생성자: vector 복사 → memcpy (힙 0회, OOM 불가)
//         · <vector> include 삭제 (반환 타입만 잔존 — 호출자 전환 후 제거)
// =========================================================================
#include "HTS_Key_Rotator.h"

// ── Self-Contained 표준 헤더 [BUG-11] ───────────────────────────────
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <new>

// [BUG-FIX CRIT] C++20 std::rotl 사용을 위한 조건부 포함
#if __cplusplus >= 202002L || (defined(_MSVC_LANG) && _MSVC_LANG >= 202002L)
#include <bit>
#endif
#if !defined(__arm__) && !defined(__TARGET_ARCH_ARM) && \
    !defined(__TARGET_ARCH_THUMB) && !defined(__ARM_ARCH)
#include <vector>
#endif

namespace ProtectedEngine {
    // [FIX-WIPE] 3중 방어 보안 소거 — impl_buf_ 전체 파쇄
    static void Key_Rotator_Secure_Wipe(void* p, size_t n) noexcept {
        if (p == nullptr || n == 0u) { return; }
        volatile uint8_t* q = static_cast<volatile uint8_t*>(p);
        for (size_t i = 0u; i < n; ++i) { q[i] = 0u; }
#if defined(__GNUC__) || defined(__clang__)
        __asm__ __volatile__("" : : "r"(p));
#endif
        std::atomic_thread_fence(std::memory_order_release);
    }



    // =====================================================================
    //  Murmur3-32 해시 (시드 파생 전용)
    //  [BUG-08/09] uint32_t k, r=0 UB 가드
    //  [BUG-14]   반환 전 h, k1 보안 소거
    // =====================================================================
    static inline uint32_t RotL32(uint32_t x, uint32_t r) noexcept {
        r &= 31u;
        if (r == 0u) { return x; }
        // [BUG-FIX CRIT] C++20 std::rotl 가드 (RotL64 패턴 통일)
        //  기존: 32u-r 잠재적 UB → 정적 분석기 경고
        //  수정: C++20에서 std::rotl 사용 (UB 원천 불가)
#if __cplusplus >= 202002L || (defined(_MSVC_LANG) && _MSVC_LANG >= 202002L)
        return std::rotl(x, static_cast<int>(r));
#else
        return (x << r) | (x >> (32u - r));
#endif
    }

    static uint32_t Murmur3_Mix(
        const uint8_t* data, size_t len, uint32_t seed) noexcept {

        uint32_t h = seed;
        // [⑨-FIX] /4u → >>2u (2의제곱 시프트 전환)
        const size_t nblocks = len >> 2u;

        for (size_t i = 0u; i < nblocks; ++i) {
            uint32_t k = 0u;
            std::memcpy(&k, data + i * 4u, 4u);

            k *= 0xCC9E2D51u;
            k = RotL32(k, 15u);
            k *= 0x1B873593u;

            h ^= k;
            h = RotL32(h, 13u);
            h = h * 5u + 0xE6546B64u;
        }

        // tail — [BUG-10] [[fallthrough]] + C++14 주석 병행
        const uint8_t* tail = data + nblocks * 4u;
        uint32_t k1 = 0u;
        switch (len & 3u) {
        case 3:
            k1 ^= static_cast<uint32_t>(tail[2]) << 16u;
#if __cplusplus >= 201703L || (defined(_MSVC_LANG) && _MSVC_LANG >= 201703L)
            [[fallthrough]];
#endif
            // fallthrough (C++14)
        case 2:
            k1 ^= static_cast<uint32_t>(tail[1]) << 8u;
#if __cplusplus >= 201703L || (defined(_MSVC_LANG) && _MSVC_LANG >= 201703L)
            [[fallthrough]];
#endif
            // fallthrough
        case 1:
            k1 ^= static_cast<uint32_t>(tail[0]);
            k1 *= 0xCC9E2D51u;
            k1 = RotL32(k1, 15u);
            k1 *= 0x1B873593u;
            h ^= k1;
            break;
        default:
            break;
        }

        // fmix32
        h ^= static_cast<uint32_t>(len);
        h ^= h >> 16u;
        h *= 0x85EBCA6Bu;
        h ^= h >> 13u;
        h *= 0xC2B2AE35u;
        h ^= h >> 16u;

        // [BUG-14] 로컬 중간값 소거
        const uint32_t result = h;
        {
            volatile uint32_t* v_h =
                reinterpret_cast<volatile uint32_t*>(&h);
            volatile uint32_t* v_k1 =
                reinterpret_cast<volatile uint32_t*>(&k1);
            *v_h = 0u;
            *v_k1 = 0u;
            // [BUG-16] seq_cst → release
            std::atomic_thread_fence(std::memory_order_release);
        }

        return result;
    }

    // =====================================================================
    //  Pimpl 구현
    //  [BUG-17] currentSeed: vector → uint8_t[SEED_LEN] 정적 배열
    //   시드 크기: 항상 32바이트 (resize(32u) 강제)
    //   try-catch 제거: 정적 배열 → OOM 경로 소멸
    // =====================================================================
    struct DynamicKeyRotator::Impl {
        static constexpr size_t SEED_LEN = 32u;
        uint8_t currentSeed[SEED_LEN] = {};
        size_t  seed_len = 0u;

        // [FIX-RACE] Spinlock — 단일 진입 강제 (Writer 상호 배제)
        //  std::atomic_flag: LDREX/STREX 기반 → ISR 안전
        //  SeqLock 삭제: Writer-Writer 경합에 무효
        std::atomic_flag spin_lock = ATOMIC_FLAG_INIT;

        explicit Impl(const uint8_t* master, size_t master_len) noexcept {
            // [BUG-17] memcpy 직접 복사 (힙 0회, OOM 불가)
            seed_len = (master_len < SEED_LEN) ? master_len : SEED_LEN;
            if (master != nullptr && seed_len > 0u) {
                std::memcpy(currentSeed, master, seed_len);
            }
            // 32바이트 미만 시 나머지는 이미 0으로 초기화됨
            seed_len = SEED_LEN;  // 항상 32바이트 고정
        }

        ~Impl() noexcept {
            Key_Rotator_Secure_Wipe(currentSeed, sizeof(currentSeed));
        }
    };

    // =====================================================================
    //  [BUG-15] 컴파일 타임 크기·정렬 검증 + get_impl()
    // =====================================================================
    DynamicKeyRotator::Impl* DynamicKeyRotator::get_impl() noexcept {
        static_assert(sizeof(Impl) <= IMPL_BUF_SIZE,
            "Impl이 IMPL_BUF_SIZE(256B)를 초과합니다 — 버퍼 크기를 늘려주세요");
        static_assert(alignof(Impl) <= IMPL_BUF_ALIGN,
            "Impl 정렬 요구가 impl_buf_ alignas(8)을 초과합니다");
        return impl_valid_.load(std::memory_order_acquire)
            ? reinterpret_cast<Impl*>(impl_buf_) : nullptr;
    }

    const DynamicKeyRotator::Impl*
        DynamicKeyRotator::get_impl() const noexcept {
        return impl_valid_.load(std::memory_order_acquire)
            ? reinterpret_cast<const Impl*>(impl_buf_)
            : nullptr;
    }

    // =====================================================================
    //  [BUG-15] 생성자 — placement new (zero-heap)
    //
    //  기존: std::make_unique<Impl>(masterSeed) + try-catch
    //  수정: impl_buf_ SecWipe → ::new Impl(masterSeed) → impl_valid_ = true
    //  Impl(masterSeed) 내부 try-catch가 OOM 처리 → 생성자 자체는 noexcept 유지
    // =====================================================================
#if !defined(__arm__) && !defined(__TARGET_ARCH_ARM) && \
    !defined(__TARGET_ARCH_THUMB) && !defined(__ARM_ARCH)
    DynamicKeyRotator::DynamicKeyRotator(
        const std::vector<uint8_t>& masterSeed) noexcept
        : DynamicKeyRotator(masterSeed.data(), masterSeed.size())
    {
    }
#endif

    DynamicKeyRotator::DynamicKeyRotator(
        const uint8_t* masterSeed, size_t master_len) noexcept
        : impl_valid_(false)
    {
        Key_Rotator_Secure_Wipe(impl_buf_, sizeof(impl_buf_));
        ::new (static_cast<void*>(impl_buf_)) Impl(
            masterSeed, master_len);
        impl_valid_.store(true, std::memory_order_release);
    }

    // =====================================================================
    //  [BUG-15] 소멸자 — 명시적 (= default 제거)
    //  Impl 소멸자(currentSeed 보안 소거) → impl_buf_ 전체 SecWipe
    // =====================================================================
    DynamicKeyRotator::~DynamicKeyRotator() noexcept {
        if (impl_valid_.load(std::memory_order_acquire)) {
            impl_valid_.store(false, std::memory_order_release);
            reinterpret_cast<Impl*>(impl_buf_)->~Impl();
        }
        // [FIX-WIPE] impl_buf_ 전체 3중 방어 소거
        Key_Rotator_Secure_Wipe(impl_buf_, sizeof(impl_buf_));
    }

    // =====================================================================
    //  deriveNextSeed — 블록 인덱스 기반 단방향 시드 파생
    //  [BUG-16] try-catch 제거 (-fno-exceptions)
    //  [BUG-17] 내부: 정적 배열 직접 사용
    //  반환: 호출자 버퍼에 복사 (Raw API)
    //  기존 vector 반환 API는 헤더에서 유지 (호출자 마이그레이션 후 제거)
    // =====================================================================
    // =====================================================================
    //  [FIX-HEAP] Raw API — 힙 할당 0회, noexcept 보장
    //  [FIX-RACE] Spinlock 보호 — Writer 상호 배제 (ISR 안전)
    // =====================================================================
    bool DynamicKeyRotator::deriveNextSeed(
        uint32_t blockIndex,
        uint8_t* out_buf, size_t out_len) noexcept {

        Impl* p = get_impl();
        if (p == nullptr || p->seed_len == 0u ||
            out_buf == nullptr || out_len == 0u) {
            return false;
        }

        // [FIX-RACE] Spinlock 획득 — 단일 Writer 진입 강제
        // ─────────────────────────────────────────────────────────────
        //  [BUG-FIX FATAL] spin_lock → PRIMASK 크리티컬 섹션 (ARM)
        //
        //  위협: 단일코어 Cortex-M4에서 atomic_flag spinlock은 데드락 확정
        //    메인루프가 lock 보유 중 ISR 발생 → ISR이 동일 lock spin
        //    → ISR이 CPU 점유 → 메인루프 영원히 복귀 불가 → 즉사(Deadlock)
        //
        //  수정: ARM — PRIMASK로 인터럽트 차단/복원 (ISR 선점 자체를 차단)
        //        PC  — atomic_flag 유지 (멀티스레드 테스트 환경)
        //        A55 — atomic_flag 유지 (멀티코어 Linux)
        // ─────────────────────────────────────────────────────────────
#if defined(__arm__) || defined(__TARGET_ARCH_ARM)
        // Cortex-M4: PRIMASK 인터럽트 차단 (cpsid i)
        uint32_t primask_saved;
        __asm__ __volatile__("mrs %0, primask\n\t"
            "cpsid i"
            : "=r"(primask_saved) :: "memory");
#else
        // PC / A55: atomic_flag spinlock (멀티스레드 안전)
        uint32_t spin_guard = 1000000u;
        while (p->spin_lock.test_and_set(std::memory_order_acquire)) {
            // 무기한 대기 방지: fail-closed 타임아웃
            if (--spin_guard == 0u) {
                return false;
            }
        }
#endif

        uint8_t* seed = p->currentSeed;
        const size_t seed_len = p->seed_len;

        // 1단계: blockIndex 혼합 (선두 4바이트)
        for (size_t i = 0u; i < 4u && i < seed_len; ++i) {
            seed[i] ^= static_cast<uint8_t>(
                (blockIndex >> (i * 8u)) & 0xFFu);
        }

        // 2단계: Murmur3 기반 전체 시드 비가역 혼합
        uint32_t running_hash = blockIndex ^ 0x5BD1E995u;
        // [⑨-FIX] /4u → >>2u (2의제곱 시프트 전환)
        size_t num_passes = (seed_len + 3u) >> 2u;
        if (num_passes == 0u) { num_passes = 1u; }

        for (size_t pass = 0u; pass < num_passes; ++pass) {
            running_hash = Murmur3_Mix(
                seed, seed_len,
                running_hash ^ static_cast<uint32_t>(pass));

            const size_t offset = (pass * 4u) % seed_len;
            for (size_t b = 0u; b < 4u && (offset + b) < seed_len; ++b) {
                seed[offset + b] ^= static_cast<uint8_t>(
                    (running_hash >> (b * 8u)) & 0xFFu);
            }
        }

        // 키 파생 중간값 보안 소거
        {
            volatile uint32_t* v_rh =
                reinterpret_cast<volatile uint32_t*>(&running_hash);
            *v_rh = 0u;
            std::atomic_thread_fence(std::memory_order_release);
        }

        // 3단계: 출력 — 임계 구역 내부에서 복사 완료 후 해제
        //  [FIX-RACE] memcpy가 lock 내부 → Tearing Read 불가
        const size_t copy_len = (out_len < seed_len) ? out_len : seed_len;
        std::memcpy(out_buf, seed, copy_len);

        // [BUG-FIX FATAL] 임계 구역 종료 — PRIMASK 복원 (ARM) / flag 해제 (PC)
#if defined(__arm__) || defined(__TARGET_ARCH_ARM)
        // Cortex-M4: PRIMASK 이전 상태 복원
        __asm__ __volatile__("msr primask, %0" :: "r"(primask_saved) : "memory");
#else
        // PC / A55: atomic_flag 해제
        p->spin_lock.clear(std::memory_order_release);
#endif

        return true;
    }

#if !defined(__arm__) && !defined(__TARGET_ARCH_ARM) && \
    !defined(__TARGET_ARCH_THUMB) && !defined(__ARM_ARCH)
    // [호환] 기존 vector API — Raw API 래퍼 (마이그레이션 후 삭제)
    std::vector<uint8_t> DynamicKeyRotator::deriveNextSeed(
        uint32_t blockIndex) noexcept {
        uint8_t buf[Impl::SEED_LEN];
        if (!deriveNextSeed(blockIndex, buf, sizeof(buf))) {
            return std::vector<uint8_t>();
        }
        return std::vector<uint8_t>(buf, buf + Impl::SEED_LEN);
    }
#endif

} // namespace ProtectedEngine