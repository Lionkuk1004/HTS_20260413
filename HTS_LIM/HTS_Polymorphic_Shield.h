// =========================================================================
// HTS_Polymorphic_Shield.h
// 다형성 암호 쉴드 — CTR 모드 스트림 암호화/복호화
// Target: STM32F407 (Cortex-M4)
//
// ─────────────────────────────────────────────────────────────────────────
//  외주 업체 통합 가이드
// ─────────────────────────────────────────────────────────────────────────
//
//  [설계 목적]
//  SplitMix64 기반 CTR 모드 스트림 암호. 텐서 데이터를
//  블록 단위로 XOR + 비트 회전하여 난독화/복호화합니다.
//
//  [암호학적 구조 — CTR 3요소]
//   Key:     gyro_seed   (자이로 위상 시드)
//   Nonce:   session_id  (세션 고유 ID)
//   Counter: block_index (데이터 위치 — 매 블록 증가)
//   → 3요소 완비 → 키스트림 재사용(Many-Time Pad) 원천 차단
//
//  [사용법]
//   // 암호화 (TX)
//   uint32_t ct = Polymorphic_Shield::Apply_Holographic_Folding<uint32_t>(
//       plaintext, gyro_seed, session_id, block_index);
//
//   // 복호화 (RX) — Apply의 정확한 역순
//   uint32_t pt = Polymorphic_Shield::Reverse_Holographic_Folding<uint32_t>(
//       ciphertext, gyro_seed, session_id, block_index);
//
//   // 레거시 32비트 API (block_index=0 고정)
//   uint32_t stream = Polymorphic_Shield::Generate_AES_CTR_Stream(
//       session_id, gyro_seed);
//
//  [타입 지원] uint8_t, uint16_t, uint32_t, uint64_t (명시적 인스턴스화)
//  [자기역 아님] Apply ≠ Reverse (비트 회전 방향이 반대)
//
//  [메모리]
//   힙 0회, 스택 ~32B, 정적 클래스 (상태 없음)
//
// ─────────────────────────────────────────────────────────────────────────
// =========================================================================
#pragma once

#include <cstdint>
#include <type_traits>

namespace ProtectedEngine {

    class Polymorphic_Shield {
    public:
        /// @brief 32비트 레거시 스트림 생성 (하위 호환)
        static uint32_t Generate_AES_CTR_Stream(
            uint64_t session_id, uint32_t gyro_seed) noexcept;

        /// @brief CTR 모드 암호화 (매 블록 고유 스트림 보장)
        /// @param data         평문 데이터
        /// @param gyro_seed    자이로 시드 (키)
        /// @param session_id   세션 ID (논스)
        /// @param block_index  블록 인덱스 (CTR 카운터 — 매 호출 고유)
        template <typename T>
        static T Apply_Holographic_Folding(
            T data, uint32_t gyro_seed,
            uint64_t session_id, uint32_t block_index) noexcept;

        /// @brief CTR 모드 복호화 (Apply의 정확한 역순)
        template <typename T>
        static T Reverse_Holographic_Folding(
            T folded_data, uint32_t gyro_seed,
            uint64_t session_id, uint32_t block_index) noexcept;
    };

} // namespace ProtectedEngine