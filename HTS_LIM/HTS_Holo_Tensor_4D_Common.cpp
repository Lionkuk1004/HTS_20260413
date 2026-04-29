/// @file  HTS_Holo_Tensor_4D_Common.cpp
#include "HTS_Holo_Tensor_4D_Common.h"
#include <cstddef>
#include <cstring>
#if defined(_MSC_VER) && (defined(_M_X64) || defined(_M_IX86))
#include <intrin.h>
#endif
#if defined(__arm__) || defined(__TARGET_ARCH_ARM) || \
    defined(__TARGET_ARCH_THUMB) || defined(__ARM_ARCH)
#if defined(__has_include)
#if __has_include(<cmsis_compiler.h>)
#include <cmsis_compiler.h>
#elif __has_include("cmsis_compiler.h")
#include "cmsis_compiler.h"
#endif
#else
#include <cmsis_compiler.h>
#endif
#endif

namespace ProtectedEngine {
    namespace {
        struct KWalshInit {
            int8_t t[64][64];
            constexpr KWalshInit() noexcept : t() {
                for (uint32_t r = 0u; r < 64u; ++r) {
                    for (uint32_t c = 0u; c < 64u; ++c) {
                        uint32_t x = r & c;
                        x ^= (x >> 16u);
                        x ^= (x >> 8u);
                        x ^= (x >> 4u);
                        x ^= (x >> 2u);
                        x ^= (x >> 1u);
                        t[r][c] = static_cast<int8_t>(
                            1 - 2 * static_cast<int32_t>(x & 1u));
                    }
                }
            }
        };
        static constexpr KWalshInit g_walsh_k{};
    } // namespace

    int8_t Holo4D_Walsh_Code(uint32_t row, uint32_t col) noexcept
    {
        return g_walsh_k.t[row & 63u][col & 63u];
    }

    void Holo4D_Generate_Partitioned_Params(
        const uint32_t        master_seed[4],
        uint32_t              t_slot,
        uint16_t*             row_workspace,
        uint16_t*             col_perm,
        uint16_t              K,
        uint16_t              N,
        uint8_t               L) noexcept
    {
        if (N == 0u || N > HOLO_CHIP_COUNT) { return; }
        if (K == 0u || K > N) { return; }
        if (row_workspace == nullptr || col_perm == nullptr) { return; }

        Xoshiro128ss rng;
        rng.Seed_With_Context(master_seed, 0xFFFFFFFFu, 0xEEEEEEEEu, t_slot);

        for (uint16_t j = 0u; j < N; ++j) {
            row_workspace[static_cast<size_t>(j)] = j;
        }

        for (uint16_t j = static_cast<uint16_t>(N - 1u); j > 0u; --j) {
            const uint32_t range = static_cast<uint32_t>(j) + 1u;
            const uint32_t r = static_cast<uint32_t>(
                (static_cast<uint64_t>(rng.Next()) * range) >> 32u);
            const int32_t dr =
                static_cast<int32_t>(r) - static_cast<int32_t>(range);
            const uint32_t mask = static_cast<uint32_t>(dr >> 31);
            const uint16_t r_idx = static_cast<uint16_t>(
                (r & mask) | ((range - 1u) & ~mask));
            const uint16_t tmp = row_workspace[static_cast<size_t>(j)];
            row_workspace[static_cast<size_t>(j)] =
                row_workspace[static_cast<size_t>(r_idx)];
            row_workspace[static_cast<size_t>(r_idx)] = tmp;
        }

        const uint32_t L32 = static_cast<uint32_t>(L);
        const uint32_t K32p = static_cast<uint32_t>(K);
        const uint16_t total_rows = static_cast<uint16_t>(L32 * K32p);
        (void)total_rows; /* L*K row ids: row_workspace[0..L*K-1] of shuffled 0..N-1 (same as legacy) */
        for (uint16_t j = 0u; j < N; ++j) {
            col_perm[static_cast<size_t>(j)] = j;
        }
        for (uint16_t j = static_cast<uint16_t>(N - 1u); j > 0u; --j) {
            const uint32_t range2 = static_cast<uint32_t>(j) + 1u;
            const uint32_t r = static_cast<uint32_t>(
                (static_cast<uint64_t>(rng.Next()) * range2) >> 32u);
            const int32_t dr2 =
                static_cast<int32_t>(r) - static_cast<int32_t>(range2);
            const uint32_t mask2 = static_cast<uint32_t>(dr2 >> 31);
            const uint16_t r_idx = static_cast<uint16_t>(
                (r & mask2) | ((range2 - 1u) & ~mask2));
            const uint16_t tmp = col_perm[static_cast<size_t>(j)];
            col_perm[static_cast<size_t>(j)] =
                col_perm[static_cast<size_t>(r_idx)];
            col_perm[static_cast<size_t>(r_idx)] = tmp;
        }
    }

    uint64_t Holo4D_Generate_Phase_Mask(
        const uint32_t        master_seed[4],
        uint32_t              time_slot) noexcept
    {
        Xoshiro128ss mask_rng;
        mask_rng.Seed_With_Context(
            master_seed, 0xBBBBBBBBu, 0xCCCCCCCCu, time_slot);
        return (static_cast<uint64_t>(mask_rng.Next()) << 32u) |
            static_cast<uint64_t>(mask_rng.Next());
    }

    static constexpr uint32_t kSecureTrue = 0x5A5A5A5Au;
    static constexpr uint32_t kSecureFalse = 0xA5A5A5A5u;

    uint32_t Holo4D_Cfi_Transition(
        HoloState&            state,
        uint8_t&              cfi_violation_count,
        HoloState             target) noexcept
    {
        if (!Holo_Is_Legal_Transition(state, target)) {
            if (Holo_Is_Legal_Transition(state, HoloState::ERROR)) {
                state = HoloState::ERROR;
            }
            else {
                state = HoloState::OFFLINE;
            }
            ++cfi_violation_count;
            return kSecureFalse;
        }
        state = target;
        return kSecureTrue;
    }

    int32_t Holo4D_Atan2_Q16(int32_t y, int32_t x) noexcept
    {
        if (x == 0 && y == 0) {
            return 0;
        }
        if (x == 0) {
            return (y > 0) ? 16384 : -16384;
        }
        int64_t X = static_cast<int64_t>(x);
        int64_t Y = static_cast<int64_t>(y);
        int32_t Z = 0;
        if (X < 0) {
            if (Y >= 0) {
                Z += 32768;
                X = -X;
                Y = -Y;
            } else {
                Z -= 32768;
                X = -X;
                Y = -Y;
            }
        }
        if (Y == 0) {
            return Z;
        }
        static constexpr int32_t k_at_q16[16] = {
            8192, 4839, 2555, 1292, 651, 326, 163, 82,
            41, 20, 10, 5, 3, 1, 1, 0
        };
        for (int i = 0; i < 16; ++i) {
            const int64_t xi = X;
            const int64_t yi = Y;
            if (yi < 0) {
                X = xi - (yi >> i);
                Y = yi + (xi >> i);
                Z -= k_at_q16[i];
            } else {
                X = xi + (yi >> i);
                Y = yi - (xi >> i);
                Z += k_at_q16[i];
            }
        }
        return Z;
    }

} // namespace ProtectedEngine
