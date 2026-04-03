// =========================================================================
// HTS_Gyro_Engine.h
// 다형성 자이로 위상 엔진 (암호학적 결맞음 + 위상 난독화)
// Target: STM32F407 (Cortex-M4)
//
// [설계 개요 — 이중 모드 아키텍처]
//
//  Mode 1: 인스턴스 위상 추적 (BB1_Core_Engine 전용)
//    Initialize_Stabilizer(session_id) → Update_Gyro_Stabilizer() → Get_Current_Phase()
//    → 세션별 결정적 위상 시퀀스 생성 (TX/RX 양단 동기)
//    → BB1_Core_Engine::Impl::locked_gyro_phase에 저장
//
//  Mode 2: 정적 위상 난독화 (Security_Pipeline 전용)
//    Apply_Dynamic_Phase_Stabilization(node) → static, 상태 없음
//    → 각 데이터 요소에 고정 비트 회전 + XOR 적용
//    → 난독화 목적 (암호학적 키가 아님 — XOR 상수 역산 가능)
//    → 역변환: 동일 함수 2회 호출 = 원본 복원 (자기역)
//    주의: 0x5A5A5A5A는 난독화 상수이며, 키로 사용하면 안 됨
//
//  두 모드는 의도적으로 독립 — Mode 1의 위상이 Mode 2에 영향 없음
// =========================================================================
#pragma once

#include <cstdint>
#include <cstddef>

namespace ProtectedEngine {

    class Gyro_Engine {
    private:
        // ── Mode 1 상태 (인스턴스별 위상 추적) ───────────────────────
        uint32_t current_gyro_phase = 0;
        uint32_t sync_counter = 0;

    public:
        // ── Mode 1: 세션별 위상 동기 ─────────────────────────────────
        void Initialize_Stabilizer(uint64_t session_id) noexcept;
        void Update_Gyro_Stabilizer() noexcept;
        uint32_t Get_Current_Phase() const noexcept;

        // ── Mode 2: 정적 위상 난독화 (자기역 — 2회 호출 = 원본) ──────
        static void Apply_Dynamic_Phase_Stabilization(uint32_t& node) noexcept;

        // ── 안티포렌식 메모리 파쇄기 (DCE 방지) ──────────────────────
        template <typename T>
        static void Safe_Buffer_Flush(T* buffer, size_t elements) noexcept;
    };

} // namespace ProtectedEngine