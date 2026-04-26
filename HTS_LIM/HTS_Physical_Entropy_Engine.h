// =========================================================================
// HTS_Physical_Entropy_Engine.h
// 물리적 엔트로피 엔진 — PRNG 시드 추출 + 앵커 노드 판정
// Target: STM32F407 (Cortex-M4, 168MHz)
//
// ─────────────────────────────────────────────────────────────────────────
//  외주 업체 통합 가이드
// ─────────────────────────────────────────────────────────────────────────
//
//  [설계 목적]
//  시스템 전역 PRNG 시드 공급원. 하드웨어 물리 엔트로피를 수집하여
//  Murmur3 3단 비가역 혼합 후 32비트 시드를 반환합니다.
//
//  [3단 플랫폼별 엔트로피 소스]
//   STM32 (Cortex-M4): STM32F407 TRNG (0x50060800) + DWT CYCCNT 지속 혼합
//   통합콘솔 (A55):    chrono 초기 시드 + CNTVCT_EL0 동적 지터
//   PC 개발빌드:       chrono 초기 시드 (디버그 전용)
//
//  [사용법]
//   uint32_t seed = Physical_Entropy_Engine::Extract_Quantum_Seed();
//   // → 매 호출 고유 32비트 (ISR/멀티스레드 동시 호출 안전)
//
//   bool is_anchor = Physical_Entropy_Engine::Is_Anchor_Node(index);
//   // → 5% 비율 앵커 (20칩당 1개)
//
//  [동시성 안전]
//   ctr_nonce_state: fetch_add + fetch_xor (하드웨어 원자적 RMW)
//   hw_trng_seeded:  compare_exchange_strong (3상 상태 머신)
//   ISR/멀티스레드 동시 호출 안전 — 잠금(mutex) 0회
//
//  [메모리]
//   정적 클래스 — 인스턴스화 차단 (= delete 6종)
//   힙 0회, 스택 ~32B (지역 변수)
//
// ─────────────────────────────────────────────────────────────────────────
// =========================================================================
#pragma once

#include <cstdint>
#include <cstddef>
#include "HTS_CXX17_Atomic_Safe.h"
namespace ProtectedEngine {

    class Physical_Entropy_Engine {
    public:
        [[nodiscard]]
        static uint32_t Extract_Quantum_Seed() noexcept;

        [[nodiscard]]
        static bool Is_Anchor_Node(size_t index) noexcept;

        Physical_Entropy_Engine() = delete;
        ~Physical_Entropy_Engine() = delete;
        Physical_Entropy_Engine(const Physical_Entropy_Engine&) = delete;
        Physical_Entropy_Engine& operator=(const Physical_Entropy_Engine&) = delete;
        Physical_Entropy_Engine(Physical_Entropy_Engine&&) = delete;
        Physical_Entropy_Engine& operator=(Physical_Entropy_Engine&&) = delete;

    private:
        static std::atomic<uint32_t> ctr_nonce_state;
        static uint32_t PRNG_Mix_Block(uint32_t entropy_seed,
            uint32_t counter) noexcept;
    };

} // namespace ProtectedEngine