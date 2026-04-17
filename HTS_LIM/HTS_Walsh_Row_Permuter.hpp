/// @file  HTS_Walsh_Row_Permuter.hpp
/// @brief Walsh row XOR 마스크 기반 심볼 순열 — CW 취약 주파수 회피 + 경량 LPI
/// 보조
/// @note  ARM only. Pimpl, placement-new, static array, zero heap.
///
///        [역할]
///         · TX: Build_Packet payload 심볼 인코딩 직전 XOR 마스크 적용
///         · RX: walsh_dec_* 결과 심볼에 동일 XOR 마스크 역적용
///         · 적용 범위: 64-chip DATA payload + 16-chip VOICE payload 만
///                      (프리앰블/헤더 제외 — 동기 무결성 보호)
///
///        [보안 포지셔닝]
///         본 모듈 단독 보안 공간: 6bit=64, 4bit=16 가지 permutation.
///         HTS 전체 다층 보안(ARIA/LEA/LSH + WFRFT + Holo_Tensor + PUF) 중
///         "Walsh row CW 공명 회피" 용도에 한정. 단독 암호 강도로 설계되지
///         않음.
///
///        [연산]
///         · TX/RX 심볼당: 1 EOR (1 cyc @ Cortex-M4)
///         · Update_Key: tx_seq_ 또는 harq_round 변경 시 1회 세션 material 파생
///
/// @author Lim Young-jun
/// @copyright INNOViD 2026. All rights reserved.
#pragma once
#include <atomic>
#include <cstddef>
#include <cstdint>
namespace ProtectedEngine {
/// @brief Walsh row XOR permutation engine
/// @warning sizeof(Impl) 는 IMPL_BUF_SIZE 이하 — static_assert 로 빌드타임 강제
class Walsh_Row_Permuter final {
  public:
    /// SECURE_TRUE / SECURE_FALSE — 다른 HTS 모듈과 반환 규약 일치
    static constexpr uint32_t SECURE_TRUE = 0x5A5A5A5Au;
    static constexpr uint32_t SECURE_FALSE = 0x00000000u;
    /// @brief 6bit 마스크 범위 상수 (64-chip DATA 심볼)
    static constexpr uint8_t MASK_6BIT = 0x3Fu;
    /// @brief 4bit 마스크 범위 상수 (16-chip VOICE 심볼)
    static constexpr uint8_t MASK_4BIT = 0x0Fu;
    Walsh_Row_Permuter() noexcept;
    ~Walsh_Row_Permuter() noexcept;
    Walsh_Row_Permuter(const Walsh_Row_Permuter &) = delete;
    Walsh_Row_Permuter &operator=(const Walsh_Row_Permuter &) = delete;
    Walsh_Row_Permuter(Walsh_Row_Permuter &&) = delete;
    Walsh_Row_Permuter &operator=(Walsh_Row_Permuter &&) = delete;
    /// @brief 초기화 — Session_Gateway 에서 세션 material 파생 후 초기 키 구성
    /// @param tx_seq 초기 tx_seq_ (packet sequence)
    /// @param harq_round 초기 HARQ round (0 = 첫 전송)
    /// @return SECURE_TRUE on success
    uint32_t Initialize(uint32_t tx_seq, uint8_t harq_round) noexcept;
    /// @brief 키 갱신 — tx_seq_ 또는 harq_round 가 변경될 때 호출
    /// @note TX/RX 양측에서 동일 (tx_seq, harq_round) 로 호출해야 동기
    uint32_t Update_Key(uint32_t tx_seq, uint8_t harq_round) noexcept;
    /// @brief TX: raw 심볼 → 암호화된 심볼 (XOR mask 적용)
    /// @param raw_sym6 6bit raw 심볼 (64-chip DATA 용)
    /// @return 암호화된 6bit 심볼
    uint8_t Encode_6bit(uint8_t raw_sym6) const noexcept;
    /// @brief RX: 암호화된 심볼 → raw 심볼 (XOR 자기 역원)
    /// @param enc_sym6 암호화된 6bit 심볼
    /// @return 복호된 6bit raw 심볼
    uint8_t Decode_6bit(uint8_t enc_sym6) const noexcept;
    /// @brief TX: 4bit VOICE 심볼 → 암호화
    uint8_t Encode_4bit(uint8_t raw_sym4) const noexcept;
    /// @brief RX: 4bit VOICE 심볼 → 복호
    uint8_t Decode_4bit(uint8_t enc_sym4) const noexcept;
    /// @brief 초기화 여부 조회
    bool Is_Initialized() const noexcept {
        return initialized_.load(std::memory_order_acquire);
    }
    /// @brief 키 갱신 횟수 (진단용)
    uint32_t Get_Update_Count() const noexcept;

  private:
    // Pimpl (placement-new) — 헤더에 구현 의존 노출 금지
    // Impl ≈ 32 byte (mask 2 + counter 4 + keys 16 + padding)
    // 빌드타임 보증: static_assert(sizeof(Impl) <= IMPL_BUF_SIZE)
    static constexpr size_t IMPL_BUF_SIZE = 64u;
    static constexpr size_t IMPL_BUF_ALIGN = 8u;
    alignas(IMPL_BUF_ALIGN) uint8_t impl_buf_[IMPL_BUF_SIZE]{};
    std::atomic<bool> initialized_{false};
    struct Impl;
    Impl *get_impl() noexcept;
    const Impl *get_impl() const noexcept;
};
} // namespace ProtectedEngine
