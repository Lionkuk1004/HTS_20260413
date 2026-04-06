// =========================================================================
// HTS_Dynamic_Fractal_Mapper.cpp — 12비트 키드 Feistel 순열 (O(1) 원소)
// [제약] Forward/Inverse/F: 32비트 곱 0, float/double/힙/나눗셈/모듈로 0
//        Update_Frame: 프레임당 1회, 여기서만 32비트 곱 허용
// =========================================================================
#include "HTS_Dynamic_Fractal_Mapper.h"

namespace ProtectedEngine {

namespace {

/// Murmur3 fmix32 — Update_Frame 전용 (32비트 곱 2회)
inline void Fmix32_Update_Only(uint32_t& s) noexcept
{
    s ^= s >> 16u;
    s *= 0x85EBCA6Bu;
    s ^= s >> 13u;
    s *= 0xC2B2AE35u;
    s ^= s >> 16u;
}

} // namespace

void Dynamic_Fractal_Mapper::Update_Frame(
    uint64_t session_id, uint32_t frame_counter) noexcept
{
    const uint32_t base =
        static_cast<uint32_t>(session_id + static_cast<uint64_t>(frame_counter))
        ^ static_cast<uint32_t>(session_id >> 32u);

    // NUM_ROUNDS == 4 → 전개 + round_keys_[· & (NUM_ROUNDS-1)] 하드 클램프
    {
        uint32_t s = base ^ 0u;
        Fmix32_Update_Only(s);
        round_keys_[0u & (NUM_ROUNDS - 1u)] = s;
    }
    {
        uint32_t s = base ^ 0x9E3779B9u;
        Fmix32_Update_Only(s);
        round_keys_[1u & (NUM_ROUNDS - 1u)] = s;
    }
    {
        uint32_t s = base ^ 0x3C6EF372u; // 2 * 0x9E3779B9u
        Fmix32_Update_Only(s);
        round_keys_[2u & (NUM_ROUNDS - 1u)] = s;
    }
    {
        uint32_t s = base ^ 0xDAA66D2Bu; // 3 * 0x9E3779B9u (mod 2^32)
        Fmix32_Update_Only(s);
        round_keys_[3u & (NUM_ROUNDS - 1u)] = s;
    }
}

uint32_t Dynamic_Fractal_Mapper::F(uint32_t x, uint32_t key) noexcept
{
    x &= HALF_MASK;
    // 6비트 → 32비트 타일 전개 (9비트용 <<9 패턴의 6비트 대응)
    const uint32_t ext_x =
        x | (x << 6u) | (x << 12u) | (x << 18u) | (x << 24u);
    uint32_t v = ext_x ^ key;
    v += (key >> 3u);
    v ^= ((v << 5u) | (v >> 27u));
    v += (key >> 7u);
    v ^= ((v << 11u) | (v >> 21u));
    return v & HALF_MASK;
}

uint32_t Dynamic_Fractal_Mapper::Forward(uint32_t index) const noexcept
{
    const uint32_t is_valid =
        0u - static_cast<uint32_t>(index <= FULL_MASK);
    uint32_t L = (index >> HALF_BITS) & HALF_MASK;
    uint32_t R = index & HALF_MASK;
    for (uint32_t i = 0u; i < NUM_ROUNDS; ++i) {
        const uint32_t tmp = R;
        R = L ^ F(R, round_keys_[i & (NUM_ROUNDS - 1u)]);
        L = tmp;
    }
    const uint32_t mapped = ((L << HALF_BITS) | R) & FULL_MASK;
    return (mapped & is_valid) | (index & ~is_valid);
}

uint32_t Dynamic_Fractal_Mapper::Inverse(uint32_t index) const noexcept
{
    const uint32_t is_valid =
        0u - static_cast<uint32_t>(index <= FULL_MASK);
    uint32_t L = (index >> HALF_BITS) & HALF_MASK;
    uint32_t R = index & HALF_MASK;
    for (uint32_t i = NUM_ROUNDS; i > 0u; --i) {
        const uint32_t tmp = L;
        const uint32_t ki = (i - 1u) & (NUM_ROUNDS - 1u);
        L = R ^ F(L, round_keys_[ki]);
        R = tmp;
    }
    const uint32_t mapped = ((L << HALF_BITS) | R) & FULL_MASK;
    return (mapped & is_valid) | (index & ~is_valid);
}

} // namespace ProtectedEngine
