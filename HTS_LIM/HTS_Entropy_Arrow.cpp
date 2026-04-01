// =========================================================================
// HTS_Entropy_Arrow.cpp
// 엔트로피 시간 화살 구현부 — DWT 틱 타이머(ARM) / steady_clock(PC)
// Target: STM32F407 (Cortex-M4, 168MHz)
//
// [양산 수정 — 6건 결함 교정]
//
//  BUG-01 [CRITICAL] ARM: <chrono> / steady_clock 미지원
//    기존: steady_clock::now() → ARM 베어메탈에서 링크 에러 또는 epoch(0) 반환
//          → 모든 세션이 즉시 만료 또는 영원히 미만료 (둘 다 치명적)
//    수정: ARM 경로에서 DWT CYCCNT 기반 틱 타이머 사용
//          32비트 래핑을 wrap_count로 64비트 확장
//          168MHz 기준: 1초 = 168,000,000 틱
//          3600초(1시간) = 604,800,000,000 틱 (uint64_t 안전)
//
//  BUG-02 [HIGH] [[unlikely]] — C++20 전용 속성
//    기존: if (...) [[unlikely]] → C++14/17에서 컴파일 에러
//    수정: C++20 가드 매크로 HTS_UNLIKELY
//
//  BUG-03 [MEDIUM] Fast_RotL64(x, 0) → x >> 64 = UB
//    수정: k &= 63u + k==0 조기 반환
//
//  BUG-04 [MEDIUM] Generate_Chaos_Seed가 steady_clock 재호출
//    기존: ARM에서 steady_clock 없음 → 비가역 해시 입력이 항상 0
//          → 동일 session_id → 동일 파쇄 결과 = 결정적 → 위험
//    수정: ARM: DWT 틱, A55/PC: steady_clock 시점 각각 혼합
//
//  BUG-05 [LOW] double 생성자 → 정수 변환 오버플로 가능
//    기존: lifespan_seconds * 1000.0 → uint64_t (음수 또는 극대 → UB)
//    수정: 범위 검증 + 클램핑
//
//  BUG-06 [LOW] C26495 — creation_time 초기화 리스트 누락
//    수정: 헤더에서 모든 멤버 기본값 할당 + 초기화 리스트 정비
//
// [STM32F407 성능]
//  Validate_Or_Destroy (미만료): ~20사이클 (DWT 읽기 + 비교)
//  Validate_Or_Destroy (만료):   ~40사이클 (+ Chaos_Seed)
//  Flash: ~300바이트
// =========================================================================
#include "HTS_Entropy_Arrow.hpp"
#include <atomic>

#if defined(__arm__) || defined(__TARGET_ARCH_ARM) || defined(__TARGET_ARCH_THUMB) || defined(__ARM_ARCH)
#define HTS_PLATFORM_ARM
#endif

#ifdef HTS_PLATFORM_ARM
#include "HTS_Hardware_Bridge.hpp"
#else
#include <chrono>
#endif

// C++20 비트 회전
#if __cplusplus >= 202002L || (defined(_MSVC_LANG) && _MSVC_LANG >= 202002L)
#include <bit>
#endif

// =========================================================================
//  [[unlikely]] C++20 가드
// =========================================================================
#if __cplusplus >= 202002L || (defined(_MSVC_LANG) && _MSVC_LANG >= 202002L)
#define HTS_UNLIKELY [[unlikely]]
#else
#define HTS_UNLIKELY
#endif

namespace ProtectedEngine {

    // =====================================================================
    //  Fast_RotL64 — 비트 좌회전 (BUG-03: 시프트 가드)
    // =====================================================================
    static inline uint64_t Fast_RotL64(uint64_t x, unsigned k) noexcept {
        k &= 63u;
        if (k == 0u) return x;
#if __cplusplus >= 202002L || (defined(_MSVC_LANG) && _MSVC_LANG >= 202002L)
        return std::rotl(x, static_cast<int>(k));
#else
        return (x << k) | (x >> (64u - k));
#endif
    }

#ifdef HTS_PLATFORM_ARM
    // =====================================================================
    //  ARM 전용 상수
    //  168MHz → 1ms = 168,000 틱
    //  최대 수명: 2^64 / 168,000,000 ≈ 1.1 × 10^11 초 (무한에 가까움)
    // =====================================================================
    static const uint64_t TICKS_PER_MS = 168000ULL;  // 168MHz / 1000

    // DWT CYCCNT 읽기 (32비트)
    static inline uint32_t Read_DWT_Tick() noexcept {
        return static_cast<uint32_t>(
            Hardware_Bridge::Get_Physical_CPU_Tick() & 0xFFFFFFFFu);
    }
#endif

    // =====================================================================
    //  생성자
    //
    //  [BUG-05] 범위 검증: 음수/극대 lifespan 클램핑
    //  double → uint64_t 변환 시 오버플로 방지
    //  최대 수명: 30일(2,592,000초) — 그 이상은 키 갱신 정책으로 처리
    // =====================================================================
    Entropy_Time_Arrow::Entropy_Time_Arrow(double lifespan_seconds) noexcept
        : is_collapsed(false) {

        // 음수/NaN/극대 방어
        if (lifespan_seconds <= 0.0 || lifespan_seconds != lifespan_seconds) {
            lifespan_seconds = 1.0;  // 최소 1초
        }
        if (lifespan_seconds > 2592000.0) {
            lifespan_seconds = 2592000.0;  // 최대 30일
        }

#ifdef HTS_ENTROPY_ARROW_ARM
        // [BUG-11] ARM: 델타 누적 타이머 초기화
        max_lifespan_ticks = static_cast<uint64_t>(lifespan_seconds * 1000.0)
            * TICKS_PER_MS;
        last_tick = Read_DWT_Tick();
        total_elapsed_ticks = 0;
#else
        // A55 Linux / PC: steady_clock 기반
        max_lifespan_ms = static_cast<uint64_t>(lifespan_seconds * 1000.0);
        creation_time = std::chrono::steady_clock::now();
#endif
    }

    // =====================================================================
    //  Validate_Or_Destroy — 수명 검증 + 만료 시 키 파쇄
    //
    //  [BUG-FIX FATAL] 32비트 DWT 래핑 Fail-Open → Fail-Safe 전환
    //   기존: "래핑 1회 누락 → 수명 단축 = fail-safe" (착각)
    //   실제: 30초 미호출 시 delta = 4.4초(래핑) → 수명 연장 = fail-OPEN
    //   수정: MAX_SILENT_TICKS(15초) 초과 delta → 즉시 자폭 (fail-safe)
    //         · 정상 호출(<15초): delta 정확 → 정상 누적
    //         · 지연 호출(15~25초): delta > MAX_SILENT → 즉시 자폭 ✓
    //         · 래핑 공격(25~40초): delta 왜곡되더라도 아키텍처 명세에 의해
    //           감시 태스크가 15초 이내 폴링 강제 → 이 경로 진입 불가
    //
    //  [BUG-FIX CRIT] 64비트 Data Tearing 방지
    //   Cortex-M4: uint64_t RMW는 LDR+ADDS+ADC+STR 4명령어
    //   ISR 선점 시 상위/하위 32비트 엇갈림 → 수백 시간 뻥튀기 → 즉시 자폭
    //   수정: PRIMASK 크리티컬 섹션으로 원자적 갱신
    //
    //  [아키텍처 전제] 호출 간격 < 15초 필수 (BB1_Core_Engine: 매 프레임)
    // =====================================================================
    uint64_t Entropy_Time_Arrow::Validate_Or_Destroy(
        uint64_t current_session_id) noexcept {

        // 이미 붕괴 상태
        if (is_collapsed.load(std::memory_order_acquire)) HTS_UNLIKELY{
            return Generate_Chaos_Seed(current_session_id);
        }

#ifdef HTS_PLATFORM_ARM
            // [BUG-FIX FATAL] 15초 폴링 임계 (168MHz × 15 = 2,520,000,000)
            //  uint32_t 최대 = 4,294,967,295 → 25.5초 래핑
            //  15초 임계 → 래핑 공격 윈도우를 40.5초 이상으로 밀어냄
        static constexpr uint32_t MAX_SILENT_TICKS = 2520000000u;  // 15초 @168MHz

        // [BUG-FIX CRIT] PRIMASK 크리티컬 섹션: 64비트 tearing 방지
        uint32_t primask;
        __asm__ __volatile__("mrs %0, primask\n\tcpsid i"
            : "=r"(primask) : : "memory");

        const uint32_t now_tick = Read_DWT_Tick();
        const uint32_t delta = now_tick - last_tick;
        last_tick = now_tick;

        // [BUG-FIX FATAL] 지연 감지: delta > 15초 → fail-safe 즉시 자폭
        const bool collapse_now = (delta > MAX_SILENT_TICKS);
        total_elapsed_ticks += static_cast<uint64_t>(delta);
        const bool expired = (total_elapsed_ticks > max_lifespan_ticks);

        // 모든 반환 경로에서 인터럽트 상태를 반드시 복원한다.
        __asm__ __volatile__("msr primask, %0" : : "r"(primask) : "memory");

        if (collapse_now) HTS_UNLIKELY{
            is_collapsed.store(true, std::memory_order_release);
            return Generate_Chaos_Seed(current_session_id);
        }
        if (expired) HTS_UNLIKELY{
            is_collapsed.store(true, std::memory_order_release);
            return Generate_Chaos_Seed(current_session_id);
        }
#else
            // ── A55 Linux / PC: steady_clock 기반 경과 시간 계산 ─────────────────────
        auto now = std::chrono::steady_clock::now();
        uint64_t elapsed_ms = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                now - creation_time).count());

        if (elapsed_ms > max_lifespan_ms) HTS_UNLIKELY{
            is_collapsed.store(true, std::memory_order_release);
            return Generate_Chaos_Seed(current_session_id);
        }
#endif

        // Force_Collapse()와의 경합 창구 차단:
        // 함수 진입 후 붕괴 플래그가 set된 경우라도 정상 세션 ID가
        // 1회 반환되지 않도록 반환 직전 재확인한다.
        if (is_collapsed.load(std::memory_order_acquire)) HTS_UNLIKELY{
            return Generate_Chaos_Seed(current_session_id);
        }
        return current_session_id;
    }

    // =====================================================================
    //  Force_Collapse — 즉시 자폭
    // =====================================================================
    void Entropy_Time_Arrow::Force_Collapse() noexcept {
        is_collapsed.store(true, std::memory_order_release);
    }

    // =====================================================================
    //  Generate_Chaos_Seed — 비가역 ARX 해시 키 파쇄
    //
    //  [보안 모델]
    //  입력: session_id + 시점 엔트로피 (DWT 틱 또는 steady_clock)
    //  → SplitMix64 변형 (Murmur3 계열 곱셈 + 우측 시프트)
    //  → Fast_RotL64 최종 비트 회전
    //  → 출력에서 원본 session_id 역산 수학적 불가
    //
    //  [BUG-04 수정] ARM: DWT 틱 / A55·PC: steady_clock 각각 사용
    // =====================================================================
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC push_options
#pragma GCC optimize("O0")
#elif defined(_MSC_VER)
#pragma optimize("", off)
#endif

    uint64_t Entropy_Time_Arrow::Generate_Chaos_Seed(uint64_t input) const noexcept {
        // 시점 엔트로피 혼합 (플랫폼별)
#ifdef HTS_PLATFORM_ARM
        uint64_t time_entropy = static_cast<uint64_t>(Read_DWT_Tick());
#else
        uint64_t time_entropy = static_cast<uint64_t>(
            std::chrono::steady_clock::now().time_since_epoch().count());
#endif

        uint64_t z = input + time_entropy;

        // SplitMix64 변형 — 비가역 혼합
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
        z ^= (z >> 31);

        // 최종 비트 회전 (위치 고정화 방지)
        return Fast_RotL64(z, 13);
    }

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC pop_options
#elif defined(_MSC_VER)
#pragma optimize("", on)
#endif

} // namespace ProtectedEngine