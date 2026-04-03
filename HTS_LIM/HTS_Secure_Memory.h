// =========================================================================
// HTS_Secure_Memory.h
// 보안 메모리 잠금 + 안티포렌식 소거 (코어 인터페이스)
// Target: STM32F407 (Cortex-M4)
//
// ─────────────────────────────────────────────────────────────────────────
//  외주 업체 통합 가이드
// ─────────────────────────────────────────────────────────────────────────
//
//  [설계 목적]
//  민감 데이터(키, PRNG 상태, 평문)의 물리 RAM 고정 + 안티포렌식 소거
//  프로젝트 전반의 모든 보안 모듈이 이 API에 의존
//
//  [플랫폼 동작]
//   ARM Release(NDEBUG)·HTS_ALLOW_OPEN_DEBUG 미정의: lockMemory/secureWipe 진입 시
//     DHCSR·Flash OPTCR(RDP) 이중 샘플 — 이상 시 Hardware_Init_Manager::Terminal_Fault_Action.
//     완화: HTS_ALLOW_OPEN_DEBUG 또는 NDEBUG 미정의 시 검사 생략. 강제 생략: HTS_SECURE_MEMORY_SKIP_PHYS_TRUST=1.
//   HTS_SECURE_MPU_LOCK_REGION(0~5 권장, 6·7 금지) 정의 시:
//     lockMemory = atomic<void*> CAS + MPU RBAR(VALID|REGION) 단일 쓰기 + RASR; ISR 대기 스핀 없음.
//     secureWipe  = CAS 해제 후 volatile 워드·바이트 소거 + 배리어·dsb.
//   매크로 미정의: lockMemory = 스텁. secureWipe는 동일 소거 + (위) 물리 신뢰 검사.
//
//  [사용법]
//   SecureMemory::lockMemory(key_ptr, 32);    // 스왑 방지
//   // ... 키 사용 ...
//   SecureMemory::secureWipe(key_ptr, 32);    // 소거 + 잠금 해제
//
// ─────────────────────────────────────────────────────────────────────────
#pragma once

#include <cstdint>
#include <cstddef>

namespace ProtectedEngine {

    static_assert(sizeof(unsigned char) == 1, "byte must be 1 byte");

    // SecureVector typedef 미사용 — ARM에서 <vector> 힙 인프라 유입 방지

    /// @brief 보안 메모리 잠금 + 안티포렌식 소거 (정적 유틸리티)
    class SecureMemory {
    public:
        /// @brief MPU 동적 잠금(HTS_SECURE_MPU_LOCK_REGION 정의 시) 또는 스텁
        /// @param ptr   대상 메모리
        /// @param size  바이트 수
        /// @note  리전 5 사용 시 HTS_Hardware_Init::Initialize_MPU의 Region 5 정적 설정을 생략할 것
        static void lockMemory(void* ptr, size_t size) noexcept;

        /// @brief 안티포렌식 데이터 파쇄 + 잠금 해제
        /// @param ptr   대상 메모리
        /// @param size  바이트 수
        /// @note  B-CDMA D-2: 정렬 구간은 may_alias volatile 워드 스토어 + 꼬리 바이트; asm(memory·ptr 의존) + release + dsb
        /// @note  MSVC 전용 TU에서만 -fno-strict-aliasing 추가 가능(GCC/Clang은 __may_alias__로 완화).
        /// @note  검수 기준 2: 대용량·고빈도 ISR 경로에서는 여전히 부담 가능 — size·호출 맥락 제한
        /// @note  [M-1] 모듈 전역 로컬 소거 루프는 MSVC에서 배리어 누락 위험이 있으므로
        ///        Key_Rotator·Secure_Boot·Conditional_SelfTest 등은 본 API 위임으로 통일.
        static void secureWipe(void* ptr, size_t size) noexcept;

        SecureMemory() = delete;
        ~SecureMemory() = delete;
        SecureMemory(const SecureMemory&) = delete;
        SecureMemory& operator=(const SecureMemory&) = delete;
        SecureMemory(SecureMemory&&) = delete;
        SecureMemory& operator=(SecureMemory&&) = delete;
    };

} // namespace ProtectedEngine
