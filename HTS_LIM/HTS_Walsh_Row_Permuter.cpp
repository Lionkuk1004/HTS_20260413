// =========================================================================
// HTS_Walsh_Row_Permuter.cpp
// Walsh row XOR 마스크 기반 심볼 순열 구현부
// Target: STM32F407 (Cortex-M4, 168MHz)
//
// [제약] new/malloc/vector/float/double/try-catch 0회. 나눗셈/% 0회.
//        런타임 힙 할당 0회. 모든 버퍼 정적.
//        32bit 곱: Update_Key 내부 세션 material 파생 경로에만 허용.
// =========================================================================
#include "HTS_Walsh_Row_Permuter.hpp"
#include "HTS_Secure_Memory.h"
#include "HTS_Session_Gateway.hpp"
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <new>
namespace ProtectedEngine {
namespace {
/// Session_Gateway 도메인 문자열 — 다른 DOMAIN_* 과 겹치지 않는 고유값
/// Session_Gateway::Derive_Session_Material 시그니처:
///   size_t(const char* domain, uint8_t* buf, size_t len)
static constexpr const char *DOMAIN_WALSH_ROW_PERM = "HTS_WALSH_ROW_PERM";
/// 세션 material 요구 바이트 수 (16 byte = 128bit 엔트로피)
static constexpr size_t SESSION_MATERIAL_BYTES = 16u;
} // namespace
// =====================================================================
//  Impl — Pimpl 은닉
//
//  레이아웃:
//   mask_6bit_  (1 byte)  ← 64-chip DATA 용
//   mask_4bit_  (1 byte)  ← 16-chip VOICE 용
//   pad_        (2 byte)
//   tx_seq_cached_    (4 byte)
//   harq_round_cached_(1 byte)
//   pad2_       (3 byte)
//   update_count_ (4 byte)
//   session_mat_ (16 byte) ← Session_Gateway 파생 원천
//   합계: 32 byte (IMPL_BUF_SIZE=64 내)
// =====================================================================
struct Walsh_Row_Permuter::Impl {
    uint8_t mask_6bit_{0u};
    uint8_t mask_4bit_{0u};
    uint8_t pad0_[2]{};
    uint32_t tx_seq_cached_{0u};
    uint8_t harq_round_cached_{0u};
    uint8_t pad1_[3]{};
    uint32_t update_count_{0u};
    uint8_t session_mat_[SESSION_MATERIAL_BYTES]{};
    Impl() noexcept = default;
    ~Impl() noexcept {
        SecureMemory::secureWipe(static_cast<void *>(session_mat_),
                                 sizeof(session_mat_));
        SecureMemory::secureWipe(static_cast<void *>(&mask_6bit_),
                                 sizeof(mask_6bit_));
        SecureMemory::secureWipe(static_cast<void *>(&mask_4bit_),
                                 sizeof(mask_4bit_));
    }
    Impl(const Impl &) = delete;
    Impl &operator=(const Impl &) = delete;
    Impl(Impl &&) = delete;
    Impl &operator=(Impl &&) = delete;
};
// =====================================================================
//  get_impl — 분기 없는 atomic-gated 포인터
// =====================================================================
Walsh_Row_Permuter::Impl *Walsh_Row_Permuter::get_impl() noexcept {
    static_assert(sizeof(Impl) <= IMPL_BUF_SIZE,
                  "Walsh_Row_Permuter::Impl exceeds IMPL_BUF_SIZE");
    static_assert(alignof(Impl) <= IMPL_BUF_ALIGN,
                  "Walsh_Row_Permuter::Impl alignment exceeds IMPL_BUF_ALIGN");
    return initialized_.load(std::memory_order_acquire)
               ? reinterpret_cast<Impl *>(impl_buf_)
               : nullptr;
}
const Walsh_Row_Permuter::Impl *Walsh_Row_Permuter::get_impl() const noexcept {
    return initialized_.load(std::memory_order_acquire)
               ? reinterpret_cast<const Impl *>(impl_buf_)
               : nullptr;
}
// =====================================================================
//  생성자 / 소멸자
// =====================================================================
Walsh_Row_Permuter::Walsh_Row_Permuter() noexcept : initialized_{false} {
    std::memset(impl_buf_, 0, sizeof(impl_buf_));
}
Walsh_Row_Permuter::~Walsh_Row_Permuter() noexcept {
    if (!initialized_.load(std::memory_order_acquire)) {
        return;
    }
    Impl *p = reinterpret_cast<Impl *>(impl_buf_);
    p->~Impl();
    SecureMemory::secureWipe(static_cast<void *>(impl_buf_), sizeof(impl_buf_));
#if defined(__GNUC__) || defined(__clang__)
    __asm__ __volatile__("" ::: "memory");
#endif
    std::atomic_thread_fence(std::memory_order_release);
    initialized_.store(false, std::memory_order_release);
}
// =====================================================================
//  Initialize — 첫 세션 키 구성
//
//   1) Session_Gateway::Derive_Session_Material() 로 16 byte 원천 파생
//   2) tx_seq / harq_round 로 마스크 계산 (Update_Key 재사용)
//   3) CAS 로 initialized_ 설정 (compare_exchange_strong, acq_rel)
// =====================================================================
uint32_t Walsh_Row_Permuter::Initialize(uint32_t tx_seq,
                                        uint8_t harq_round) noexcept {
    bool expected = false;
    if (!initialized_.compare_exchange_strong(expected, true,
                                              std::memory_order_acq_rel,
                                              std::memory_order_acquire)) {
        //  재초기화 시도는 기존 상태 유지 → SECURE_TRUE 반환
        return SECURE_TRUE;
    }
    Impl *p = ::new (static_cast<void *>(impl_buf_)) Impl{};
    const size_t derived_len = Session_Gateway::Derive_Session_Material(
        DOMAIN_WALSH_ROW_PERM, p->session_mat_, sizeof(p->session_mat_));
    //  엔트로피 부족 시 Fail-Closed (Dual_Tensor_16bit 와 동일 정책)
    if (derived_len < SESSION_MATERIAL_BYTES) {
        p->~Impl();
        std::memset(impl_buf_, 0, sizeof(impl_buf_));
        initialized_.store(false, std::memory_order_release);
        return SECURE_FALSE;
    }
    //  초기 마스크 계산 (Update_Key 인라인 equivalent)
    return Update_Key(tx_seq, harq_round);
}
// =====================================================================
//  Update_Key — tx_seq / harq_round 변경 시 마스크 재파생
//
//   마스크 파생:
//     base64_0..3 = session_mat_[0..15] 의 32bit word 4개
//     mix = base_word ^ rot(tx_seq, 13) ^ (harq_round << 3)
//     mask_6bit = (mix ^ (mix >> 6) ^ (mix >> 12) ^ (mix >> 18)) & 0x3F
//     mask_4bit = (mix ^ (mix >> 4) ^ (mix >> 8)  ^ (mix >> 16)) & 0x0F
//
//  · 32bit 곱 0회 (XOR / SHIFT / OR 만)
//  · XOR-fold 로 session material 엔트로피를 6bit/4bit 에 압축
// =====================================================================
uint32_t Walsh_Row_Permuter::Update_Key(uint32_t tx_seq,
                                        uint8_t harq_round) noexcept {
    Impl *p = reinterpret_cast<Impl *>(impl_buf_);
    if (p == nullptr) {
        return SECURE_FALSE;
    }
    // session_mat_ 16 byte → 32bit word 4개
    uint32_t s0 = 0u;
    uint32_t s1 = 0u;
    uint32_t s2 = 0u;
    uint32_t s3 = 0u;
    std::memcpy(&s0, &p->session_mat_[0], 4u);
    std::memcpy(&s1, &p->session_mat_[4], 4u);
    std::memcpy(&s2, &p->session_mat_[8], 4u);
    std::memcpy(&s3, &p->session_mat_[12], 4u);
    // tx_seq rotate 13 (MISRA: 명시 캐스팅 + 32bit 불변)
    const uint32_t ts_rot13 = (tx_seq << 13u) | (tx_seq >> (32u - 13u));
    const uint32_t hr_shift = static_cast<uint32_t>(harq_round) << 3u;
    // 믹싱: 4개 word 를 XOR 체인 + tx_seq/harq 삽입
    uint32_t mix = s0 ^ s1 ^ s2 ^ s3 ^ ts_rot13 ^ hr_shift;
    // XOR-fold: 32bit → 6bit / 4bit 압축 (엔트로피 보존 + 편향 완화)
    const uint32_t fold6 =
        mix ^ (mix >> 6u) ^ (mix >> 12u) ^ (mix >> 18u) ^ (mix >> 24u);
    const uint32_t fold4 = mix ^ (mix >> 4u) ^ (mix >> 8u) ^ (mix >> 12u) ^
                           (mix >> 16u) ^ (mix >> 20u) ^ (mix >> 24u) ^
                           (mix >> 28u);
    p->mask_6bit_ = static_cast<uint8_t>(fold6 & MASK_6BIT);
    p->mask_4bit_ = static_cast<uint8_t>(fold4 & MASK_4BIT);
    p->tx_seq_cached_ = tx_seq;
    p->harq_round_cached_ = harq_round;
    p->update_count_++;
    // 가시성: 다른 코어/ISR 이 mask 읽기 전에 반드시 커밋되어야 함
#if defined(__GNUC__) || defined(__clang__)
    __asm__ __volatile__("" ::: "memory");
#endif
    std::atomic_thread_fence(std::memory_order_release);
    return SECURE_TRUE;
}
// =====================================================================
//  Encode / Decode — 심볼 XOR (자기 역원)
//
//   encode_N(raw)   = raw ^ mask_N
//   decode_N(enc)   = enc ^ mask_N   (XOR 자기 역원 → encode 와 동일 연산)
//
//   비초기화 상태에서 호출 시: mask=0 → permutation 미적용 (호환성 보장)
//   → 상위 레이어 실수로 Initialize 누락해도 raw sym 그대로 통과
// =====================================================================
uint8_t Walsh_Row_Permuter::Encode_6bit(uint8_t raw_sym6) const noexcept {
    const Impl *p = get_impl();
    const uint8_t m = (p != nullptr) ? p->mask_6bit_ : 0u;
    return static_cast<uint8_t>((raw_sym6 ^ m) & MASK_6BIT);
}
uint8_t Walsh_Row_Permuter::Decode_6bit(uint8_t enc_sym6) const noexcept {
    const Impl *p = get_impl();
    const uint8_t m = (p != nullptr) ? p->mask_6bit_ : 0u;
    return static_cast<uint8_t>((enc_sym6 ^ m) & MASK_6BIT);
}
uint8_t Walsh_Row_Permuter::Encode_4bit(uint8_t raw_sym4) const noexcept {
    const Impl *p = get_impl();
    const uint8_t m = (p != nullptr) ? p->mask_4bit_ : 0u;
    return static_cast<uint8_t>((raw_sym4 ^ m) & MASK_4BIT);
}
uint8_t Walsh_Row_Permuter::Decode_4bit(uint8_t enc_sym4) const noexcept {
    const Impl *p = get_impl();
    const uint8_t m = (p != nullptr) ? p->mask_4bit_ : 0u;
    return static_cast<uint8_t>((enc_sym4 ^ m) & MASK_4BIT);
}
uint32_t Walsh_Row_Permuter::Get_Update_Count() const noexcept {
    const Impl *p = get_impl();
    return (p != nullptr) ? p->update_count_ : 0u;
}
} // namespace ProtectedEngine
