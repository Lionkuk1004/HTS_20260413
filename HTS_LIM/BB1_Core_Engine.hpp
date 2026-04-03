// =========================================================================
// BB1_Core_Engine.hpp
// HTS 최상위 코어 엔진 — 공개 인터페이스 (Pimpl 완전 은닉)
// Target: STM32F407 (Cortex-M4, 168MHz, SRAM 192KB) / PC
//
// ─────────────────────────────────────────────────────────────────────────
//  외주 업체 통합 가이드
// ─────────────────────────────────────────────────────────────────────────
//
//  [설계 목적]
//  HTS B-CDMA 보안 통신 시스템의 최상위 TX/RX 텐서 파이프라인 엔진.
//  내부 보안 로직(직교 스크램블, 양자 디코이, 다형성 접힘, 인터리빙,
//  L1 스파스 복구)을 Pimpl 패턴으로 완전 은닉한다.
//
//  [사용법]
//   1. 생성: BB1_Core_Engine engine;
//      → OOM/초기화 실패 시 impl_valid_=false → Process/Recover가 false 반환
//
//   2. TX 송신:
//      bool ok = engine.Process_Tensor_Pipeline(
//          data, count, session_id, slice_chunk, anchor_interval,
//          is_test_mode, strict_mode);
//      → 직교 스크램블 → 간섭 패턴 → 양자 디코이 → 인터리빙
//        → 다형성 접힘 → 파일럿 삽입
//
//   3. RX 수신:
//      bool ok = engine.Recover_Tensor_Pipeline(
//          damaged, count, session_id, slice_chunk, anchor_interval,
//          is_test_mode, strict_mode);
//      → PLL 위상복원 → 역접힘 → 역인터리빙 → L1 하이브리드 복구
//
//   4. 통계: RecoveryStats st = engine.Get_Last_Recovery_Stats();
//      → 마지막 RX 복구 통계 (파괴율, 패리티 복구, 보간 복구 등)
//
//  [메모리 요구량]
//   sizeof(BB1_Core_Engine) ≈ 20488B (impl_buf_[20480] + impl_valid_ + padding)
//   Impl(SRAM In-Place): 정적 배열 16KB + Gyro×2 + TimeArrow×2 + stats ≈ 17KB
//   힙 할당: 0B (전량 정적 배열로 전환 — OOM 경로 소멸)
//   ⚠ 반드시 전역/정적 변수로 배치할 것 (스택 배치 시 ~20KB 소모 → 스택 오버플로우)
//
//  [보안 설계]
//   Impl 소멸자: 모든 내부 상태 보안 소거 (이동평균, 통계, 위상)
//   impl_buf_: 소멸자에서 SecWipe — Impl 전체 이중 소거
//   복사/이동: = delete (보안 상태 복제 원천 차단)
//
// ─────────────────────────────────────────────────────────────────────────
#pragma once

// ── Self-Contained 표준 헤더 ────────────────────────────────────────
#include <cstdint>
#include <cstddef>

// ── 프로젝트 의존 (RecoveryStats 구조체 반환 타입) ──────────────────
#include "HTS_Sparse_Recovery.h"

namespace ProtectedEngine {

    /// @brief HTS 최상위 TX/RX 텐서 파이프라인 코어 엔진
    ///
    /// Pimpl 패턴으로 내부 보안 로직을 완전 은닉한다.
    /// 외부에서는 Process(TX) / Recover(RX) / Get_Last_Recovery_Stats만 사용.
    ///
    /// @note 초기화 실패(OOM) 시 impl_valid_=false → Process/Recover가 false 반환
    /// @note 복사/이동 = delete (보안 상태 복제 차단)
    class BB1_Core_Engine {
    public:
        /// @brief 코어 엔진 생성 (placement new + Reserve_Buffers)
        /// @note  Reserve_Buffers 실패 시 impl_valid_=false → API 호출은 false/{} 반환
        explicit BB1_Core_Engine() noexcept;

        /// @brief 소멸자 — Impl 소멸자 호출 후 impl_buf_ 전체 SecWipe
        ~BB1_Core_Engine() noexcept;

        /// 보안 상태 복제 원천 차단
        BB1_Core_Engine(const BB1_Core_Engine&) = delete;
        BB1_Core_Engine& operator=(const BB1_Core_Engine&) = delete;
        BB1_Core_Engine(BB1_Core_Engine&&) = delete;
        BB1_Core_Engine& operator=(BB1_Core_Engine&&) = delete;

        /// @brief 마지막 RX 복구 통계 반환
        /// @return RecoveryStats (파괴율, 패리티 복구, 보간 복구 등)
        /// @note  impl_valid_=false 시 빈 RecoveryStats{} 반환
        [[nodiscard]]
        RecoveryStats Get_Last_Recovery_Stats() const noexcept;

        /// @brief TX 텐서 파이프라인 (스크램블 → 디코이 → 인터리빙 → 접힘)
        /// @tparam T  데이터 타입 (uint16_t 또는 uint32_t)
        /// @param tensor_data     입력/출력 텐서 (in-place 변환)
        /// @param elements        텐서 원소 개수
        /// @param session_id      64비트 세션 키
        /// @param slice_chunk     슬라이싱 청크 크기
        /// @param anchor_interval 앵커 간격 (0=적응형 자동 결정)
        /// @param is_test_mode    테스트 모드 (보안 게이트 우회)
        /// @param strict_mode     엄격 모드
        /// @return true=성공, false=실패 (OOM/보안 게이트 차단)
        template <typename T>
        [[nodiscard]]
        bool Process_Tensor_Pipeline(
            T* tensor_data, size_t elements, uint64_t session_id,
            uint32_t slice_chunk, uint32_t anchor_interval = 0,
            bool is_test_mode = false, bool strict_mode = false);

        /// @brief RX 텐서 파이프라인 (PLL → 역접힘 → 역인터리빙 → L1 복구)
        /// @tparam T  데이터 타입 (uint16_t 또는 uint32_t)
        /// @param damaged_tensor  손상된 텐서 (in-place 복구)
        /// @param elements        텐서 원소 개수
        /// @param session_id      64비트 세션 키 (TX와 동일 필수)
        /// @param slice_chunk     슬라이싱 청크 크기
        /// @param anchor_interval 앵커 간격 (0=적응형 자동 결정)
        /// @param is_test_mode    테스트 모드 (보안 게이트 우회)
        /// @param strict_mode     엄격 모드
        /// @return true=복구 성공, false=복구 실패 (잔여 데이터 보안 소거됨)
        template <typename T>
        [[nodiscard]]
        bool Recover_Tensor_Pipeline(
            T* damaged_tensor, size_t elements, uint64_t session_id,
            uint32_t slice_chunk, uint32_t anchor_interval = 0,
            bool is_test_mode = false, bool strict_mode = false);

    private:
        // ── [2048 다운사이즈] Pimpl In-Place Storage (zero-heap) ──────────
        //  shared: state_map(8KB) + temp_vec(8KB) = 16KB
        //  erased_bits: 256B (비트 패킹 2048비트)
        //  Gyro×2 + TimeArrow×2 + stats ≈ 0.5KB
        //  합계: ~17KB → IMPL_BUF_SIZE = 20480 (20KB)
        //  절감: 36864 → 20480 = −16,384B (Phase 2 여유 9→25KB)
        static constexpr size_t IMPL_BUF_SIZE = 20480u;
        static constexpr size_t IMPL_BUF_ALIGN = 8u;

        struct Impl;  ///< 내부 상태 완전 은닉 (ABI 안정성 보장)

        alignas(IMPL_BUF_ALIGN) uint8_t impl_buf_[IMPL_BUF_SIZE];
        bool impl_valid_ = false;  ///< placement new 성공 여부

        /// @brief impl_buf_에서 Impl 포인터 반환 (컴파일 타임 크기·정렬 검증 포함)
        Impl* get_impl() noexcept;
        /// @overload
        const Impl* get_impl() const noexcept;
    };

} // namespace ProtectedEngine