#pragma once
#include <cstdint>
#include <cstddef>

namespace ProtectedEngine {

class Dynamic_Fractal_Mapper {
public:
    // 12비트 Feistel (4096 = 2^12), 6비트씩 분할 — SRAM·와이어 슬랩 정합
    static constexpr uint32_t INDEX_BITS = 12u;
    static constexpr uint32_t HALF_BITS = 6u;
    static constexpr uint32_t HALF_MASK = 63u;
    static constexpr uint32_t FULL_MASK = 4095u;
    static constexpr uint32_t NUM_ROUNDS = 4u;
    static_assert((NUM_ROUNDS & (NUM_ROUNDS - 1u)) == 0u,
        "NUM_ROUNDS must be power of two for round key index mask");

    void Update_Frame(uint64_t session_id, uint32_t frame_counter) noexcept;

    [[nodiscard]]
    uint32_t Forward(uint32_t index) const noexcept;

    [[nodiscard]]
    uint32_t Inverse(uint32_t index) const noexcept;

private:
    uint32_t round_keys_[NUM_ROUNDS] = {};

    // 곱 없는 라운드 함수: XOR + ADD + 시프트만
    static uint32_t F(uint32_t x, uint32_t key) noexcept;
};

} // namespace ProtectedEngine
