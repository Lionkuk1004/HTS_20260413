// =========================================================================
// HTS_Sparse_Recovery.h
// HTS L1 스파스 하이브리드 복구 및 스텔스 난독화 모듈
// Target: STM32F407 (Cortex-M4, 168MHz)
//
// ─────────────────────────────────────────────────────────────────────────
//  외주 업체 통합 가이드
// ─────────────────────────────────────────────────────────────────────────
//
//  [설계 목적]
//  TX: 텐서 데이터에 XOR 간섭 패턴을 씌워 스텔스 난독화
//      + 블록 단위 XOR 패리티 앵커 삽입 (자가 치유 기반)
//  RX: 간섭 패턴 해제 → 1차 XOR 패리티 확정 복구
//      → 2차 중력 지평선 보간 (연쇄 파괴 시 아날로그 힐링)
//
//  [사용법]
//   // TX 송신단
//   Sparse_Recovery_Engine::Generate_Interference_Pattern<uint32_t>(
//       tensor, elements, session_id, anchor_interval, is_test_mode);
//
//   // RX 수신단
//   RecoveryStats stats;
//   bool ok = Sparse_Recovery_Engine::Execute_L1_Reconstruction<uint32_t>(
//       damaged, elements, session_id, anchor_interval,
//       is_test_mode, strict_mode, stats);
//
//  [anchor_interval 규격]
//   상용망(is_test_mode=false): 1~6 (0 또는 >6 → 자동 보정)
//   테스트(is_test_mode=true):  0 → 기본값 20
//   구현(.cpp): 정규화 후 2의 거듭제곱으로 내림(floor) — 페이로드 판별에 비트 마스크(idx & (interval-1))
//
//  [strict_mode]
//   true:  연쇄 파괴 블록 → 복구 포기 + unrecoverable 통계 + ERASURE 소거
//   false: 중력 지평선 보간 시도 (아날로그 데이터 전용)
//
//  [메모리]
//   힙 0회 — 모든 연산 인플레이스 (텐서 자체를 직접 변환)
//   RecoveryStats: 24B(ARM)/44B(PC) 스택 (noise_ratio_q16 Q16 정수)
//
//  [보안 / 빌드]
//   ARM Release: Generate/Execute 진입 시 DHCSR·OPTCR(RDP) 폴링(.cpp)
//   HTS_SPARSE_RECOVERY_SKIP_PHYS_TRUST / HTS_ALLOW_OPEN_DEBUG 로 스킵 가능
//
// ─────────────────────────────────────────────────────────────────────────
// =========================================================================
#pragma once
#include <cstdint>
#include <cstddef>
namespace ProtectedEngine {
    /// 상용/테스트 앵커 간격 상한 (Generate / Execute 동일 규격)
    struct SparseRecoveryLimits {
        static constexpr uint32_t ANCHOR_INTERVAL_CAP = 6u;
        static constexpr uint32_t TEST_MODE_DEFAULT_ANCHOR = 20u;
    };
    // =====================================================================
    // [통계 및 텔레메트리] L1 복구 엔진 처리 결과 구조체
    // =====================================================================
    struct RecoveryStats {
        size_t total_elements = 0;
        size_t destroyed_count = 0;
        size_t recovered_by_parity = 0;
        size_t recovered_by_gravity = 0;
        size_t unrecoverable = 0;
        // double → uint32_t Q16 (0~65536 = 0.0~1.0), 텔레메트리 로깅 전용
        uint32_t noise_ratio_q16 = 0u;
    };
    // =====================================================================
    // [코어 엔진] HTS L1 스파스 하이브리드 복구 및 스텔스 난독화 모듈
    // =====================================================================
    class Sparse_Recovery_Engine {
    public:
        // [TX 송신단] 데이터 난독화 및 패리티 앵커 생성
        /// @note tensor_block==nullptr 이거나 elements==0 이면 무연산
        /// @note B-2/K-2: `tensor_block` 은 `alignof(T)` 정렬 필수 — 미정렬이면 무연산(데이터 미변경)
        template <typename T>
        static void Generate_Interference_Pattern(
            T* tensor_block,
            size_t elements,
            uint64_t session_id,
            uint32_t anchor_interval,
            bool is_test_mode
        );
        // [RX 수신단] 난독화 해제 및 하이브리드 자가 치유
        // [[nodiscard]] — bool 반환값 무시 방지
        /// @note damaged_tensor==nullptr 또는 elements==0 이면 false, out_stats 는 0으로 초기화
        /// @note B-2/K-2: `damaged_tensor` 는 `alignof(T)` 정렬 필수 — 미정렬이면 false
        template <typename T>
        [[nodiscard]]
        static bool Execute_L1_Reconstruction(
            T* damaged_tensor,
            size_t elements,
            uint64_t session_id,
            uint32_t anchor_interval,
            bool is_test_mode,
            bool strict_mode,
            RecoveryStats& out_stats
        );
    };
} // namespace ProtectedEngine
