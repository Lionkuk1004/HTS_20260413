#include "HTS_Holo_LPI.h"
#include <cstring>

namespace ProtectedEngine {
namespace {

struct LPI_Xoshiro128 {
    uint32_t s[4];
    static uint32_t rotl(uint32_t x, int k) noexcept {
        return (x << static_cast<uint32_t>(k))
             | (x >> (32u - static_cast<uint32_t>(k)));
    }
    uint32_t next() noexcept {
        const uint32_t r = rotl(s[1] * 5u, 7) * 9u;
        const uint32_t t = s[1] << 9u;
        s[2] ^= s[0]; s[3] ^= s[1];
        s[1] ^= s[2]; s[0] ^= s[3];
        s[2] ^= t; s[3] = rotl(s[3], 11);
        return r;
    }
    void seed_from(const uint32_t m[4], uint32_t ctx) noexcept {
        auto mix = [](uint32_t z) noexcept -> uint32_t {
            z = (z ^ (z >> 16u)) * 0x45D9F3Bu;
            z = (z ^ (z >> 16u)) * 0x45D9F3Bu;
            return z ^ (z >> 16u);
        };
        s[0] = mix(m[0] ^ ctx);
        s[1] = mix(m[1] ^ (ctx * 0x9E3779B9u));
        s[2] = mix(m[2] ^ (ctx * 0x517CC1B7u));
        s[3] = mix(m[3] ^ (ctx * 0x6C62272Eu));
        for (int i = 0; i < 4; ++i) { (void)next(); }
    }
};

// FWHT in-place N=64
static void fwht64(int32_t* d) noexcept {
    for (int len = 1; len < 64; len <<= 1) {
        for (int i = 0; i < 64; i += (len << 1)) {
            for (int j = 0; j < len; ++j) {
                const int32_t u = d[i + j];
                const int32_t v = d[i + j + len];
                d[i + j]       = u + v;
                d[i + j + len] = u - v;
            }
        }
    }
}

// 4D 회전 (Holo_Tensor_Engine 동일 구조)
static constexpr uint8_t k_perm[24][4] = {
    {0,1,2,3},{0,1,3,2},{0,2,1,3},{0,2,3,1},{0,3,1,2},{0,3,2,1},
    {1,0,2,3},{1,0,3,2},{1,2,0,3},{1,2,3,0},{1,3,0,2},{1,3,2,0},
    {2,0,1,3},{2,0,3,1},{2,1,0,3},{2,1,3,0},{2,3,0,1},{2,3,1,0},
    {3,0,1,2},{3,0,2,1},{3,1,0,2},{3,1,2,0},{3,2,0,1},{3,2,1,0}
};

static void rotate_4d(int32_t* b, uint32_t seed) noexcept {
    for (int i = 0; i < 4; ++i) {
        if ((seed >> static_cast<uint32_t>(i)) & 1u) { b[i] = -b[i]; }
    }
    const uint8_t pi = static_cast<uint8_t>(
        ((seed >> 4u) & 0x1Fu) % 24u);
    const uint8_t* p = k_perm[pi];
    const int32_t v[4] = {b[p[0]], b[p[1]], b[p[2]], b[p[3]]};
    b[0] = v[0]+v[1]+v[2]+v[3];
    b[1] = v[0]-v[1]+v[2]-v[3];
    b[2] = v[0]+v[1]-v[2]-v[3];
    b[3] = v[0]-v[1]-v[2]+v[3];
}

} // anonymous namespace

uint32_t HTS_Holo_LPI::Generate_Scalars(
    const uint32_t master_seed[4],
    uint32_t time_slot,
    uint8_t mix_ratio_q8,
    int16_t* out_scalars) noexcept
{
    if (master_seed == nullptr || out_scalars == nullptr) {
        return SECURE_FALSE;
    }

    // 1. 시드 → PRNG → ±1 초기값
    LPI_Xoshiro128 rng;
    rng.seed_from(master_seed, time_slot);

    int32_t buf[64];
    for (int i = 0; i < 64; ++i) {
        buf[i] = (rng.next() & 1u) ? 1 : -1;
    }

    // 2. FWHT 혼합 (CLT → 가우시안 분포)
    fwht64(buf);

    // 3. 4D 회전 (비선형 섞기)
    for (int i = 0; i < 64; i += 4) {
        rotate_4d(&buf[i], rng.next());
    }

    // 4. 2차 FWHT
    fwht64(buf);

    // 5. 최대 절대값 탐색
    int32_t max_abs = 1;
    for (int i = 0; i < 64; ++i) {
        const int32_t a = (buf[i] >= 0) ? buf[i] : -buf[i];
        if (a > max_abs) { max_abs = a; }
    }

    // 6. Mix 적용: scalar = (256-mix)*8192/256 + mix*norm*8192/256
    //    mix_ratio_q8=128 → 0.50
    const int32_t mix = static_cast<int32_t>(mix_ratio_q8);
    const int32_t inv = 256 - mix;

    for (int i = 0; i < 64; ++i) {
        // norm = buf[i] * 4096 / max_abs  ([-4096, +4096])
        const int32_t norm = (buf[i] * 4096) / max_abs;
        // mixed = inv*4096/256 + mix*norm/256
        //       = (inv*4096 + mix*norm) / 256
        const int32_t mixed = (inv * 4096 + mix * norm) >> 8;
        // mixed 범위: [0, 8192] (mix=128 → [0, 8192])
        // 클램프
        int32_t v = mixed;
        if (v > 8191) { v = 8191; }
        if (v < 0)    { v = 0; }
        out_scalars[i] = static_cast<int16_t>(v);
    }

    return SECURE_TRUE;
}

void HTS_Holo_LPI::Apply(
    int16_t* chips_I,
    int16_t* chips_Q,
    uint16_t chip_count,
    const int16_t* scalars) noexcept
{
    if (chips_I == nullptr || chips_Q == nullptr || scalars == nullptr) {
        return;
    }
    for (uint16_t i = 0u; i < chip_count; ++i) {
        const int16_t sc = scalars[i & 63u];
        chips_I[i] = static_cast<int16_t>(
            (static_cast<int32_t>(chips_I[i]) * sc) >> 13);
        chips_Q[i] = static_cast<int16_t>(
            (static_cast<int32_t>(chips_Q[i]) * sc) >> 13);
    }
}

} // namespace ProtectedEngine
