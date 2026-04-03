// =========================================================================
// HTS_Security_Pipeline.h
// 최상위 보안 파이프라인 — 위상 안정화 + 간섭 패턴 + AEAD 태그
// Target: STM32F407 (Cortex-M4, 168MHz)
//
// ─────────────────────────────────────────────────────────────────────────
//  외주 업체 통합 가이드
// ─────────────────────────────────────────────────────────────────────────
//
//  [파이프라인 위치]
//   Open_Session → ★Security_Pipeline★ → FEC → Pulse Shaping → Tx
//
//  [사용법]
//   Security_Pipeline pipeline;
//   std::atomic<bool> abort{false};
//   pipeline.Secure_Master_Worker(data, 0, len, abort, len); // 마지막 인자=전체 버퍼 워드 수(병렬 청크 시 필수)
//   // 또는 AEAD 버전 (32비트 분할 태그):
//   std::atomic<uint32_t> tag_hi{0}, tag_lo{0};
//   pipeline.Secure_Master_Worker_AEAD(data, 0, len, abort, tag_hi, tag_lo, len);
//
//  [AEAD 태그 주의]
//   현재 태그 = FNV-1a 변형 (키 없는 해시) — 진정한 MAC이 아님
//   무결성 검증용이지 인증(Authentication)은 아님
//   향후 HMAC 또는 CMAC으로 교체 권장
//
// ─────────────────────────────────────────────────────────────────────────
#pragma once

#include <cstdint>
#include <cstddef>
#include <atomic>

namespace ProtectedEngine {

    class Security_Pipeline {
    public:
        /// @brief 보안 파이프라인 (AEAD 없음)
        /// @param data          처리 대상 데이터
        /// @param start         시작 인덱스
        /// @param end           종료 인덱스 (미포함)
        /// @param abort_signal       외부 중단 신호
        /// @param buffer_total_words 0이면 실패 시 [start,end)만 소거; 양수면 data[0..) 전체 워드 수(병렬 청크 공유 버퍼)
        void Secure_Master_Worker(
            uint32_t* data, size_t start, size_t end,
            std::atomic<bool>& abort_signal,
            size_t buffer_total_words = 0u) noexcept;

        /// @brief AEAD 무결성 태그 생성 포함 파이프라인
        /// @param data            처리 대상 데이터
        /// @param start           시작 인덱스
        /// @param end             종료 인덱스 (미포함)
        /// @param abort_signal    외부 중단 신호
        /// @param global_tag_hi   전역 태그 상위 32비트 (원자적 XOR 병합)
        /// @param global_tag_lo   전역 태그 하위 32비트 (원자적 XOR 병합)
        /// @param buffer_total_words Secure_Master_Worker와 동일(보안 실패 시 전체 소거 범위)
        /// @note  atomic<uint64_t> → 2×atomic<uint32_t> 분할
        ///        ARM Cortex-M4: LDREX/STREX 단일 사이클 lock-free 보장
        ///        64비트 Tearing/링커 에러/__atomic_fetch_xor_8 미존재 해소
        void Secure_Master_Worker_AEAD(
            uint32_t* data, size_t start, size_t end,
            std::atomic<bool>& abort_signal,
            std::atomic<uint32_t>& global_tag_hi,
            std::atomic<uint32_t>& global_tag_lo,
            size_t buffer_total_words = 0u) noexcept;

        // 파이프라인 상태 복제 방지 (6종 완비)
        Security_Pipeline() noexcept = default;
        Security_Pipeline(const Security_Pipeline&) = delete;
        Security_Pipeline& operator=(const Security_Pipeline&) = delete;
        Security_Pipeline(Security_Pipeline&&) = delete;
        Security_Pipeline& operator=(Security_Pipeline&&) = delete;
    };

} // namespace ProtectedEngine