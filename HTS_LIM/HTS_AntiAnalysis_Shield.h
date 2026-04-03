// =========================================================================
// HTS_AntiAnalysis_Shield.h
// 실행 속도 기반 디버거/에뮬레이터 탐지 + 기만적 붕괴
// Target: STM32F407 (Cortex-M4, 168MHz)
//
// ─────────────────────────────────────────────────────────────────────────
//  외주 업체 통합 가이드
// ─────────────────────────────────────────────────────────────────────────
//
//  [설계 목적]
//  DWT CYCCNT 타이밍 프로브로 디버거/에뮬레이터를 실시간 탐지.
//  탐지 시 텐서 데이터를 기만적으로 붕괴시켜 패턴 분석 무력화.
//
//  [사용법]
//   1. 부팅 직후: AntiAnalysis_Shield::Calibrate_Baseline()
//      → 정상 속도 측정 (DWT CYCCNT 기반, 5회 중 최소값)
//      → compare_exchange로 이중 진입 원천 봉쇄
//      → 후발 스레드는 bounded spin(최대 50회) 후 안전 폴백
//
//   2. 주기적 검증: AntiAnalysis_Shield::Is_Under_Observation()
//      → 현재 속도가 baseline × 10 초과 시 true (디버거/에뮬레이터)
//      → DWT 미초기화(0 반환) 시 false (오탐 방지)
//
//   3. 탐지 시: AntiAnalysis_Shield::Deploy()
//      → 감사 로그 기록 + 상위 레이어에서 세션 파괴 수행
//
//   4. 데이터 파쇄: AntiAnalysis_Shield::Trigger_Deceptive_Collapse(data, n)
//      → Physical_Entropy 기반 비예측 패턴으로 텐서 덮어쓰기
//      → unsigned 타입만 허용 (static_assert)
//
// ─────────────────────────────────────────────────────────────────────────
#pragma once

#include <cstdint>
#include <cstddef>
#include <atomic>

namespace ProtectedEngine {

    /// @brief 실행 속도 기반 디버거/에뮬레이터 탐지 (정적 유틸리티)
    class AntiAnalysis_Shield {
    public:
        /// @brief 부팅 직후 정상 실행 속도 측정 (1회만 실행)
        static void Calibrate_Baseline() noexcept;

        /// @brief 관측 탐지 시 감사 로그 + 기만 상태 돌입
        static void Deploy() noexcept;

        /// @brief 외부 분석/관측 시도 탐지 (타이밍 프로브)
        /// @return true = 디버거/에뮬레이터 탐지됨
        [[nodiscard]]
        static bool Is_Under_Observation() noexcept;

        /// @brief 패턴 분석 무력화 기만적 붕괴 (unsigned 타입 전용)
        /// @param tensor_data  텐서 배열 (nullptr 시 무동작)
        /// @param elements     요소 수 (0 시 무동작)
        template <typename T>
        static void Trigger_Deceptive_Collapse(T* tensor_data, size_t elements) noexcept;

        // 정적 전용 클래스 — 인스턴스화 차단 (6종 완비)
        AntiAnalysis_Shield() = delete;
        ~AntiAnalysis_Shield() = delete;
        AntiAnalysis_Shield(const AntiAnalysis_Shield&) = delete;
        AntiAnalysis_Shield& operator=(const AntiAnalysis_Shield&) = delete;
        AntiAnalysis_Shield(AntiAnalysis_Shield&&) = delete;
        AntiAnalysis_Shield& operator=(AntiAnalysis_Shield&&) = delete;

    private:
        // uint64_t → uint32_t (ARM lock-free 보장)
        static std::atomic<uint32_t> baseline_execution_ticks;

        // bool → uint32_t 3상 상태 머신
        //  0 = CAL_UNINIT:      미초기화
        //  1 = CAL_IN_PROGRESS: 측정 중 (선발 스레드가 점유)
        //  2 = CAL_DONE:        완료 (baseline 유효)
        // 후발 스레드: 상태 2가 될 때까지 spin-wait 또는 기본값 폴백
        static constexpr uint32_t CAL_UNINIT = 0u;
        static constexpr uint32_t CAL_IN_PROGRESS = 1u;
        static constexpr uint32_t CAL_DONE = 2u;
        static std::atomic<uint32_t> cal_state;
    };

    // 빌드 타임 검증
    static_assert(sizeof(uint32_t) == 4, "uint32_t must be 4 bytes");

} // namespace ProtectedEngine