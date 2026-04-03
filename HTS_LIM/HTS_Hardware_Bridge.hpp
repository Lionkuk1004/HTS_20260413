// =========================================================================
// HTS_Hardware_Bridge.hpp
// CPU 물리 틱 추출 및 보안 메모리 소거
// Target: STM32F407 (Cortex-M4, 168MHz)
//
// ─────────────────────────────────────────────────────────────────────────
//  외주 업체 통합 가이드
// ─────────────────────────────────────────────────────────────────────────
//
//  [설계 목적]
//  Get_Physical_CPU_Tick: OS 시간 API 우회 — 물리 사이클 카운터 직접 접근
//    → 타이밍 사이드채널 방어, VDF 지연 측정, 엔트로피 혼합
//  Secure_Erase: 컴파일러 최적화 차단 보안 소거
//    → KCMVP 키 소재 잔존 방지(Key Zeroization) 범용 유틸리티
//
//  [사용법]
//   1. uint64_t tick = Hardware_Bridge::Get_Physical_CPU_Tick();
//      → VDF/엔트로피에서 호출 — ISR 내부 호출 안전 (단일 명령어)
//
//   2. Hardware_Bridge::Secure_Erase_Memory(vec);
//      → vector<uint8_t> 전체 소거 (크기 유지, capacity 유지)
//
//   3. Hardware_Bridge::Secure_Erase_Raw(ptr, bytes);
//      → 키, HMAC 컨텍스트, 세션 구조체 등 비벡터 소거
//      → RAII_Secure_Wiper 내부에서도 이 함수 사용 가능
//
//  [ARM DWT CYCCNT 주의사항 — STM32F407]
//   32비트 카운터 @ 168MHz → 2^32 / 168,000,000 ≈ 25.56초마다 래핑
//   반환값: uint64_t이지만 ARM에서는 하위 32비트만 유효
//   래핑 추적 필요 시: Entropy_Time_Arrow 참조 (wrap_count 64비트 확장)
//   HTS_Hardware_Init::Initialize_System() — 부팅 순서 WDT→NVIC→MPU→DWT(H-2)
//   에서 DWT CYCCNT 활성화. DEMCR TRCENA + DWT CYCCNTENA 필수
//   미활성 시 0 고정 → 타이밍 방어 무력화 (부팅 직후 반드시 확인)
//
//  [AArch64 CNTVCT_EL0 — 통합콘솔 Cortex-A55]
//   64비트 Generic Timer → 래핑 거의 없음 (x86 TSC와 유사)
//   Linux 커널이 EL0 접근 활성화 (CNTKCTL_EL1.EL0VCTEN=1, 기본 활성)
//   주파수: CNTFRQ_EL0 (SoC 의존, 일반적 24MHz~1GHz)
//
//  [메모리 요구량]
//   순수 static 함수 — 인스턴스 없음, 힙 할당 없음
//   코드 크기: ~200B (ARM Thumb2)
//
//  [보안 설계]
//   3중 DCE 방지: pragma O0 + volatile + atomic_thread_fence(release)
//   컴파일러 배리어: MSVC _ReadWriteBarrier / GCC asm volatile
//   미지원 CPU: #error로 컴파일 타임 차단 (0 반환 무력화 방지)
//
// ─────────────────────────────────────────────────────────────────────────
#pragma once

#include <cstdint>
#include <cstddef>

#if defined(__arm__) || defined(__TARGET_ARCH_ARM) || defined(__TARGET_ARCH_THUMB) || defined(__ARM_ARCH)
#define HTS_HWBRIDGE_ARM
#endif

#ifndef HTS_HWBRIDGE_ARM
#include <vector>
#endif

namespace ProtectedEngine {

    class Hardware_Bridge {
    public:
        /// @brief CPU 물리 사이클 카운터 읽기
        /// @return x86: TSC 64비트 / ARM: DWT CYCCNT 하위 32비트 / AArch64: CNTVCT_EL0 64비트
        /// @note  ISR 내부 호출 안전 (단일 명령어, 힙 0, 잠금 0)
        /// @note  ARM: Initialize_System()에서 DWT 활성화 필수
        /// @note  AArch64: Linux 커널 EL0 접근 기본 활성
        /// @note  미지원 CPU: 컴파일 에러 (#error)
        static uint64_t Get_Physical_CPU_Tick() noexcept;

        /// @brief vector<uint8_t> 보안 소거 (PC/A55 전용)
        /// @param buffer  소거 대상 벡터 (빈 벡터 시 무동작)
        /// @post  buffer 내용 전부 0 (크기/capacity 유지)
        /// @note  [DCE 방지] pragma O0 + volatile + atomic_thread_fence
#ifndef HTS_HWBRIDGE_ARM
        static void Secure_Erase_Memory(std::vector<uint8_t>& buffer) noexcept;
#endif

        /// @brief raw 포인터 보안 소거
        /// @param ptr        소거 대상 시작 주소 (nullptr 시 무동작)
        /// @param size_bytes 소거할 바이트 수 (0 시 무동작)
        /// @post  [ptr, ptr+size_bytes) 범위 전부 0
        /// @note  KCMVP Key Zeroization: 키, HMAC, 세션 상태 소거용
        /// @note  [DCE 방지] pragma O0 + volatile + atomic_thread_fence
        static void Secure_Erase_Raw(void* ptr, size_t size_bytes) noexcept;
    };

} // namespace ProtectedEngine

#undef HTS_HWBRIDGE_ARM