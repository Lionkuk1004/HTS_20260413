// =========================================================================
// HTS_Storage_Adapter.hpp
// 원시(Raw) 파티션 기반 앵커 백업/복원 어댑터
// Target: STM32F407 (Cortex-M4, 168MHz)
//
// ─────────────────────────────────────────────────────────────────────────
//  외주 업체 통합 가이드
// ─────────────────────────────────────────────────────────────────────────
//
//  [설계 목적]
//  앵커 금고(Anchor_Vault) 데이터를 Raw 파티션(은닉 섹터)에 백업/복원
//  USB 하드웨어 토큰 + 세션 바인딩 매직 넘버로 이중 인증
//
//  [사용법]
//   Backup:  Storage_Adapter::Backup_To_Raw_Volume(session, vault, disk_mb, ratio)
//   Restore: Storage_Adapter::Restore_From_Raw_Volume(session, vault, words, n)
//   → 모든 함수 static — 인스턴스 생성 불필요/불가
//
// ─────────────────────────────────────────────────────────────────────────
#pragma once

#include <cstdint>
#include <cstddef>

namespace ProtectedEngine {

    // 전방 선언 (Backup: const&, Restore: non-const&)
    class Anchor_Vault;

    /// @brief Raw 파티션 앵커 백업/복원 (정적 유틸리티 클래스)
    class Storage_Adapter {
    public:
        /// @brief 앵커 데이터를 Raw 파티션에 백업 (읽기 전용 — 금고 비변조)
        ///
        /// @note `Anchor_Vault::Export_Anchor()`가 반환하는 앵커 포맷은
        ///       `payload||HMAC-SHA256(32B)`이며, restore 시
        ///       `Anchor_Vault::Import_Anchor()`가 동일 포맷을 fail-closed
        ///       방식으로 검증합니다.
        /// @param session_id            현재 PQC 세션 ID
        /// @param vault                 앵커 금고 참조 (const: 백업은 원본 읽기만)
        /// @param total_disk_mb         디스크 전체 용량 (MB)
        /// @param anchor_ratio_percent  앵커 파티션 비율 (5~30%)
        /// @return true=성공, false=인증 실패/범위 초과
        [[nodiscard]]
        static bool Backup_To_Raw_Volume(
            uint64_t session_id,
            const Anchor_Vault& vault,
            size_t total_disk_mb,
            uint8_t anchor_ratio_percent) noexcept;

        /// @brief Raw 파티션에서 앵커 데이터 복원
        /// @param session_id     현재 PQC 세션 ID
        /// @param vault          앵커 금고 참조 (복원 대상)
        /// @param partition_words 파티션 원시 워드 배열 (최소 2워드: 매직 헤더)
        /// @param word_count      `partition_words` 유효 길이(워드). 0이면 빈 입력
        /// @return true=성공, false=인증/매직 실패
        ///
        /// @note restore 결과로 `Anchor_Vault::Import_Anchor()`에 전달되는
        ///       앵커도 `payload||HMAC-SHA256(32B)` 포맷을 그대로 따릅니다.
        /// @note H-1: `partition_words == nullptr && word_count != 0` → false
        /// @note B-2: 워드 접근 시 `partition_words` 는 4바이트 정렬 권장 (구현부에서 검증)
        [[nodiscard]]
        static bool Restore_From_Raw_Volume(
            uint64_t session_id,
            Anchor_Vault& vault,
            const uint32_t* partition_words,
            size_t word_count) noexcept;

        // 정적 전용 클래스 — 인스턴스화 차단
        Storage_Adapter() = delete;
        ~Storage_Adapter() = delete;
        Storage_Adapter(const Storage_Adapter&) = delete;
        Storage_Adapter& operator=(const Storage_Adapter&) = delete;
        Storage_Adapter(Storage_Adapter&&) = delete;
        Storage_Adapter& operator=(Storage_Adapter&&) = delete;

    private:
        static const uint64_t AUTHORIZED_USB_SERIAL;
        static bool Check_Hardware_Token() noexcept;
    };

} // namespace ProtectedEngine