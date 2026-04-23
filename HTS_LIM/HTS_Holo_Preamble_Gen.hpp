// =============================================================================
// HTS_Holo_Preamble_Gen.hpp — CMYK holo preamble chip generator (TX, optional)
// =============================================================================
#pragma once

#include "HTS_Config.h"

#if defined(HTS_HOLO_PREAMBLE) && HTS_HOLO_CMYK_MODE

#include "HTS_Holo_Tensor_4D_Defs.h"

namespace ProtectedEngine {
namespace detail {

void fwht_raw(int32_t* d, int n) noexcept;

/// CMYK: block_id 0..3 → independent 64-chip preamble; block_id==0 matches legacy ctx=slot.
/// Integer-only. INNOViD HOLO_6FACE_FINAL.
inline void gen_holo_sequence_cmyk(const uint32_t seed[4], uint32_t slot,
                                   uint32_t block_id, int16_t amp,
                                   int16_t* const out_I) noexcept {
    auto mix = [](const uint32_t z0) noexcept -> uint32_t {
        uint32_t z = z0;
        z = (z ^ (z >> 16u)) * 0x45D9F3Bu;
        z = (z ^ (z >> 16u)) * 0x45D9F3Bu;
        return z ^ (z >> 16u);
    };
    const uint32_t ctx = slot ^ (block_id * 0xDEADBEEFu);
    Xoshiro128ss rng{};
    rng.s[0] = mix(seed[0] ^ ctx);
    rng.s[1] = mix(seed[1] ^ (ctx * 0x9E3779B9u));
    rng.s[2] = mix(seed[2] ^ (ctx * 0x517CC1B7u));
    rng.s[3] = mix(seed[3] ^ (ctx * 0x6C62272Eu));
    rng.s[0] ^= 0x50524500u;
    for (int w = 0; w < 8; ++w) {
        (void)rng.Next();
    }

    int32_t buf[64];
    for (int i = 0; i < 64; ++i) {
        buf[i] = (rng.Next() & 1u) ? 1 : -1;
    }
    fwht_raw(buf, 64);

    static constexpr uint8_t k_perm[24][4] = {
        {0, 1, 2, 3}, {0, 1, 3, 2}, {0, 2, 1, 3}, {0, 2, 3, 1}, {0, 3, 1, 2},
        {0, 3, 2, 1}, {1, 0, 2, 3}, {1, 0, 3, 2}, {1, 2, 0, 3}, {1, 2, 3, 0},
        {1, 3, 0, 2}, {1, 3, 2, 0}, {2, 0, 1, 3}, {2, 0, 3, 1}, {2, 1, 0, 3},
        {2, 1, 3, 0}, {2, 3, 0, 1}, {2, 3, 1, 0}, {3, 0, 1, 2}, {3, 0, 2, 1},
        {3, 1, 0, 2}, {3, 1, 2, 0}, {3, 2, 0, 1}, {3, 2, 1, 0}};
    for (int i = 0; i < 64; i += 4) {
        const uint32_t gyro = rng.Next();
        for (int j = 0; j < 4; ++j) {
            if ((gyro >> static_cast<uint32_t>(j)) & 1u) {
                buf[i + j] = -buf[i + j];
            }
        }
        const uint8_t pi =
            static_cast<uint8_t>(((gyro >> 4u) & 0x1Fu) % 24u);
        const uint8_t* p = k_perm[pi];
        const int32_t v0 = buf[i + p[0]];
        const int32_t v1 = buf[i + p[1]];
        const int32_t v2 = buf[i + p[2]];
        const int32_t v3 = buf[i + p[3]];
        buf[i + 0] = v0 + v1 + v2 + v3;
        buf[i + 1] = v0 - v1 + v2 - v3;
        buf[i + 2] = v0 + v1 - v2 - v3;
        buf[i + 3] = v0 - v1 - v2 + v3;
    }
    fwht_raw(buf, 64);

    for (int i = 0; i < 64; ++i) {
        const int32_t b = buf[i];
        out_I[i] = (b >= 0)
                       ? amp
                       : static_cast<int16_t>(-static_cast<int32_t>(amp));
    }
}

} // namespace detail
} // namespace ProtectedEngine

#endif // HTS_HOLO_PREAMBLE && HTS_HOLO_CMYK_MODE
