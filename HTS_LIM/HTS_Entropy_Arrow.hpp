// =========================================================================
// HTS_Entropy_Arrow.hpp
// 엔트로피 시간 화살 — 세션 키 시한 폭파 (Time-Bound Key Destruction)
// Target: STM32F407 (Cortex-M4)
//
// [설계 목적]
//  PQC 세션 키에 시간 제한을 부여하여, 수명 초과 시 키를 비가역적으로 파쇄
//  → 장기 키 탈취 공격 무력화 (Perfect Forward Secrecy 보강)
//  → Force_Collapse()로 외부 침입 감지 시 즉시 자폭
//
// [사용 예시] (BB1_Core_Engine.cpp)
//  Entropy_Time_Arrow time_arrow(3600000u);  // 1시간 수명 (밀리초)
//  ...
//  validated_session = time_arrow.Validate_Or_Destroy(session_id);
//  // 수명 이내: session_id 그대로 반환
//  // 수명 초과: 비가역 해시로 파쇄된 값 반환 → 이후 복호화 불가
//
// =========================================================================
#pragma once

#include <cstdint>
#include <atomic>

// =========================================================================
//  플랫폼 분기 — 존재 기반 (#ifdef) 안전 패턴
//
//  #ifdef HTS_ENTROPY_ARROW_ARM — #if 미정의=0 혼입 방지
//
//  STM32 (Cortex-M4 베어메탈): DWT CYCCNT 틱 기반 (steady_clock 없음)
//  통합콘솔 (A55 Linux) / PC: std::chrono::steady_clock
// =========================================================================
#if defined(__arm__) || defined(__TARGET_ARCH_ARM) || defined(__TARGET_ARCH_THUMB) || defined(__ARM_ARCH)
#define HTS_ENTROPY_ARROW_ARM
#else
// A55 Linux: steady_clock 정상 지원 (glibc CLOCK_MONOTONIC)
// PC MSVC:   steady_clock 정상 지원 (QueryPerformanceCounter)
// HTS_ENTROPY_ARROW_ARM 미정의 → #ifdef 분기에서 자동 #else 진입
#include <chrono>
#endif

namespace ProtectedEngine {

    class Entropy_Time_Arrow {
    private:
#ifdef HTS_ENTROPY_ARROW_ARM
        // ── ARM: DWT CYCCNT 델타 누적 타이머 ────────────────────────
        // DWT 32비트 래핑 대응: 델타 누적(64비트 total_elapsed_ticks)
        uint32_t last_tick = 0;              ///< 이전 호출 시 DWT 틱
        uint64_t total_elapsed_ticks = 0;    ///< 누적 경과 틱 (64비트)
        uint64_t max_lifespan_ticks = 0;     ///< 수명 한도 (틱)
#else
        // ── PC: steady_clock 기반 타이머 ─────────────────────────────
        std::chrono::steady_clock::time_point creation_time;
        uint64_t max_lifespan_ms = 0;
#endif

        // ── 공통: 붕괴 상태 (원자적) ─────────────────────────────────
        std::atomic<bool> is_collapsed{ false };

        uint64_t Generate_Chaos_Seed(uint64_t input) const noexcept;

    public:
        /// @param lifespan_ms 세션 키 수명 (밀리초). 0이면 최소 1초로 보정, 상한 30일.
        Entropy_Time_Arrow(uint32_t lifespan_ms) noexcept;
        uint64_t Validate_Or_Destroy(uint64_t current_session_id) noexcept;
        void Force_Collapse() noexcept;
    };

} // namespace ProtectedEngine