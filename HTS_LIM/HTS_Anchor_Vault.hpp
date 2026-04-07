// =========================================================================
// HTS_Anchor_Vault.hpp
// 5% 위상 닻(Anchor) 보안 금고 — 추출/격리/복원/반출/반입
// Target: Cortex-A55 (CORE-X Pro 메인CPU) / Server
//
// [⚠ STM32 베어메탈 사용 금지]
//  std::map + std::vector 사용 → 힙 할당 + 코드 팽창
//  A55(Linux) 또는 서버 환경에서만 사용
//
// =========================================================================
#pragma once

// STM32 (Cortex-M) 빌드 차단 — <vector>/<map> 힙 인프라
// A55 (aarch64) 및 PC는 정상 통과
#if (defined(__arm__) || defined(__TARGET_ARCH_ARM) || \
     defined(__TARGET_ARCH_THUMB) || defined(__ARM_ARCH)) && \
    !defined(__aarch64__)
#error "[HTS_FATAL] HTS_Anchor_Vault.hpp는 A55/서버 전용입니다. STM32 빌드에서 제외하십시오."
#endif

#include <vector>
#include <map>
#include <mutex>    // public 메서드 직렬화
#include <cstdint>

namespace ProtectedEngine {

    /// @brief 5% 위상 닻 보안 금고 (A55/서버 전용)
    ///
    /// @par 스레드 안전성
    ///   모든 public 메서드는 내부 mutex로 직렬화됩니다.
    ///   A55 Linux 멀티스레드 환경에서 동시 접근 안전합니다.
    ///   std::map Red-Black Tree 재균형 중 SIGSEGV 방지.
    ///
    /// @par 보안 소거
    ///   소멸자에서 Clear_Vault()를 호출하여 앵커 데이터를
    ///   volatile 소거 후 힙 반환합니다. 명시적 Clear_Vault() 호출 없이
    ///   객체가 소멸되어도 콜드부트/힙 스캔 공격을 방어합니다.
    class Anchor_Vault {
    private:
        mutable std::mutex mtx_;   ///< 전 메서드 직렬화
        std::map<uint64_t, std::vector<uint8_t>> secret_enclave;

    public:
        Anchor_Vault() = default;

        /// @brief Anchor export payload 뒤에 덧붙는 HMAC-SHA256 태그 길이(바이트)
        /// @note 64비트 block_id + payload를 HMAC-SHA256로 인증합니다.
        static constexpr size_t ANCHOR_HMAC_TAG_SIZE_BYTES = 32u;

        /// @brief 소멸자 — 모든 앵커 데이터 보안 소거 후 해제
        /// 소멸 시 Clear_Vault()
        ~Anchor_Vault() noexcept { Clear_Vault(); }

        // std::mutex는 non-copyable/non-movable → 클래스도 동일
        Anchor_Vault(const Anchor_Vault&) = delete;
        Anchor_Vault& operator=(const Anchor_Vault&) = delete;
        Anchor_Vault(Anchor_Vault&&) = delete;
        Anchor_Vault& operator=(Anchor_Vault&&) = delete;

        void Sequestrate_Anchors(uint64_t block_id,
            const std::vector<uint8_t>& tensor,
            uint8_t ratio_percent) noexcept;

        void Replant_Anchors(uint64_t block_id,
            std::vector<uint8_t>& damaged_tensor,
            uint8_t ratio_percent) noexcept;

        /// @brief 외부 반출 — `out`에 payload||HMAC tag 기록 (반환 vector 복사 없음)
        ///
        /// @param[out] out (payload + 32바이트 tag). 금고에 없거나 실패 시 clear.
        /// @return true=성공, false=없음/세션·HMAC 실패
        [[nodiscard]] bool Export_Anchor(
            uint64_t block_id,
            std::vector<uint8_t>& out) noexcept;

        /// @brief 외부 반입 — HMAC tag 검증 후 payload만 저장
        ///
        /// @param external_anchor  (payload + 32바이트 HMAC-SHA256 tag)
        /// @note  검증 실패 시 기존 저장값을 덮어쓰지 않습니다 (fail-closed).
        void Import_Anchor(uint64_t block_id,
            const std::vector<uint8_t>& external_anchor) noexcept;

        /// @brief 처리 완료된 앵커 보안 소거 + 맵 삭제 (OOM 방지)
        /// secret_enclave 누적 방지 — ACK 후 해당 block_id 소거 + erase
        void Discard_Anchor(uint64_t block_id) noexcept;

        void Clear_Vault() noexcept;
    };

} // namespace ProtectedEngine