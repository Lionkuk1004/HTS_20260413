// =========================================================================
/// @file  HTS_BitOps.h
/// @brief 프로젝트 공통 비트 연산 헬퍼 (popcount32)
/// @target STM32F407VGT6 (Cortex-M4F) / PC
///
/// [양산 수정 이력 — 11건]
///  BUG-01~07 (이전 세션)
///  BUG-08 [CRIT] libgcc __popcountsi2 호출 → SWAR 알고리즘 (ALU 12cyc)
///  BUG-09 [HIGH] inline → constexpr (static_assert 호환)
///  BUG-10 [MED]  <intrin.h> 종속성 제거 (크로스 플랫폼 O(1))
///  BUG-11 [MED]  pragma_once.h → HTS_BitOps.h 리네이밍
///                (파일명이 역할 미반영 + #pragma once 지시어와 혼동)
// =========================================================================
#pragma once

#include <cstdint>

// C++20 <bit> — PC/x86 빌드에서만 사용 (ARM은 SWAR 고정)
#if !defined(__arm__) && !defined(__TARGET_ARCH_ARM) && \
    !defined(__TARGET_ARCH_THUMB) && !defined(__ARM_ARCH) && \
    !defined(__aarch64__) && \
    (__cplusplus >= 202002L || (defined(_MSVC_LANG) && _MSVC_LANG >= 202002L))
#include <bit>
#endif

namespace ProtectedEngine {

    /// @brief 32비트 정수의 세트 비트(1) 개수 반환
    /// @param x 입력 값
    /// @return 0~32 범위의 세트 비트 수
    [[nodiscard]]
    constexpr uint32_t popcount32(uint32_t x) noexcept {
        // [BUG-FIX CRIT] C++20 std::popcount ARM 분기 차단
        //
        //  기존: C++20이면 무조건 std::popcount(x)
        //  문제: std::popcount → __builtin_popcount → Cortex-M4에 HW 없음
        //        → libgcc __popcountsi2 함수 호출 (프롤로그+에필로그+BL ~30cyc)
        //        → BUG-08에서 제거한 오버헤드가 C++20 전환 시 부활
        //
        //  수정: ARM 타겟은 C++ 버전 무관하게 SWAR 고정 (12cyc ALU)
        //        PC/x86만 std::popcount 허용 (HW POPCNT 단일 사이클)
        //
        //  SWAR: 분기 0, 메모리 참조 0, ALU 12cyc O(1)
        //  Cortex-M4 MUL: 단일 사이클 (HW 곱셈기)

#if !defined(__arm__) && !defined(__TARGET_ARCH_ARM) && \
    !defined(__TARGET_ARCH_THUMB) && !defined(__ARM_ARCH) && \
    !defined(__aarch64__) && \
    (__cplusplus >= 202002L || (defined(_MSVC_LANG) && _MSVC_LANG >= 202002L))
        // PC/x86 빌드 전용: HW POPCNT 활용 (단일 사이클)
        return static_cast<uint32_t>(std::popcount(x));
#else
        // ARM (Cortex-M4/A55) + C++17 이하 PC: SWAR 알고리즘
        x = x - ((x >> 1u) & 0x55555555u);
        x = (x & 0x33333333u) + ((x >> 2u) & 0x33333333u);
        return (((x + (x >> 4u)) & 0x0F0F0F0Fu) * 0x01010101u) >> 24u;
#endif
    }

} // namespace ProtectedEngine