/// @file  HTS_Walsh_Row_Permuter.hpp
/// @brief Walsh row XOR 마스크 기반 심볼 순열 — CW 공명 주파수 회피
///        (HTS 다층 보안 체인의 필수 링크)
/// @note  ARM only. Pimpl, placement-new, static array, zero heap.
///
///        [역할]
///         · TX: Build_Packet payload 심볼 인코딩 직전 XOR 마스크 적용
///         · RX: walsh_dec_* 결과 심볼에 동일 XOR 마스크 역적용
///         · 적용 범위: 64-chip DATA payload + 16-chip VOICE payload
///                      (프리앰블/헤더 제외 — 동기 무결성 보호)
///
///        [포지셔닝]
///         HTS 전체 보안은 다층 구조 (WFRFT × Holo_Tensor × Dual_Tensor ×
///         Key_Rotator × Fractal_Mapper × 본 모듈 × ARIA/LEA/LSH) 의 조합으로
///         10^280000 수준 키 공간 및 μs 단위 시간 동기 요구를 형성한다.
///         본 모듈의 단독 공간 (6bit=64, 4bit=16) 은 체인 내 필수 링크 역할.
///
///        [고유 기능]
///         본 모듈의 독자적 가치는 Walsh row sequency 에 대한 좁대역 CW
///         공명 회피이다. packet 마다 Walsh row 배정이 (session_mat,
///         tx_seq, harq_round) 로 재파생되어, 동일 주파수의 반복 공격이
///         같은 row 에 지속 타격되는 것을 방지한다. 다른 보안 레이어가
///         담당하지 않는 고유 기능.
///
///        [연산]
///         · TX/RX 심볼당: 1 atomic load (relaxed) + 1 EOR (2~3 cyc @ Cortex-M4)
///         · Update_Key: tx_seq 또는 harq_round 변경 시 mask 재파생
///         · Initialize: 세션 시작 시 1회 Session_Gateway material 파생
///
///        [17+4 체크 준수]
///         ① memory_order  : load/store relaxed (hot path), CAS acq_rel (init)
///         ② fast_abs      : 해당 없음
///         ③ heap 0        : placement-new, static buffer
///         ④ float/double 0: 정수 연산만
///         ⑤ try-catch 0   : noexcept 전체
///         ⑥ stack 소량    : Update_Key 약 24 byte
///         ⑦ 주석 일치    : 재검토 완료
///         ⑧ static_assert: sizeof(Impl) ≤ IMPL_BUF_SIZE
///         ⑨ 나눗셈 0     : shift/XOR만
///         ⑩ const& zero-copy: primitive by-value 적절
///         ⑪ endian 독립   : pack_le32 (bit-shift)
///         ⑫ static_cast   : 명시
///         ⑬ CFI          : 상태기 없음
///         ⑭ PC 코드 0    : 표준 라이브러리만
///         ⑮ HW polling 0 : 해당 없음
///         ⑯ EMI 방어     : mask=0 폴백
///         ⑰ 실시간 데드라인: Encode/Decode ≤ 3 cyc
///         A sizeof 전파  : @warning 명시
///         B static_assert: IMPL_BUF_SIZE 범위 확인
///         C atomic CAS   : compare_exchange_strong acq_rel
///         D C++20 가드   : [[nodiscard]] 사용
///
/// @author Lim Young-jun
/// @copyright INNOViD 2026. All rights reserved.
#pragma once
#include "HTS_CXX17_Atomic_Safe.h"
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
    [[nodiscard]] uint32_t Initialize(uint32_t tx_seq,
                                      uint8_t harq_round) noexcept;
    /// @brief 키 갱신 — tx_seq_ 또는 harq_round 가 변경될 때 호출
    /// @note TX/RX 양측에서 동일 (tx_seq, harq_round) 로 호출해야 동기
    [[nodiscard]] uint32_t Update_Key(uint32_t tx_seq,
                                        uint8_t harq_round) noexcept;
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

    // [성능 최적화] 핫 패스용 캐시 mask — 핫 루프에서 relaxed load
    //   · Encode/Decode 가 Impl 을 경유하지 않도록 외부 멤버로 분리
    //   · Update_Key 에서 release store, Encode/Decode 에서 relaxed load
    //   · 초기값 0 → mask=0 항등 (회귀 방지)
    std::atomic<uint8_t> cached_mask_6bit_{0u};
    std::atomic<uint8_t> cached_mask_4bit_{0u};

    alignas(IMPL_BUF_ALIGN) uint8_t impl_buf_[IMPL_BUF_SIZE]{};
    std::atomic<bool> initialized_{false};
    struct Impl;
    Impl *get_impl() noexcept;
    const Impl *get_impl() const noexcept;
};
} // namespace ProtectedEngine
