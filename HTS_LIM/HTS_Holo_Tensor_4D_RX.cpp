/// @file  HTS_Holo_Tensor_4D_RX.cpp
#include "HTS_Holo_Tensor_4D_RX.h"
#include "HTS_Atan2_Q16.h"
#include "HTS_Arm_Irq_Mask_Guard.h"
#include "HTS_Secure_Memory.h"
#include <new>
#include <cstring>
#if defined(HTS_CFO_V5A_S5H_STEPD_RX) || defined(HTS_HOLO_RX_DECODE_AUDIT)
#include <cstdio>
#endif
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
        struct Holo4D_RX_Busy_Guard final {
            std::atomic_flag* flag;
            bool locked;
            explicit Holo4D_RX_Busy_Guard(std::atomic_flag& f) noexcept
                : flag(&f), locked(false) {
                locked = !flag->test_and_set(std::memory_order_acq_rel);
            }
            ~Holo4D_RX_Busy_Guard() noexcept {
                if (locked) { flag->clear(std::memory_order_release); }
            }
            Holo4D_RX_Busy_Guard(const Holo4D_RX_Busy_Guard&) = delete;
            Holo4D_RX_Busy_Guard& operator=(const Holo4D_RX_Busy_Guard&) = delete;
        };

        static inline void holo4d_rx_busy_spin_yield() noexcept {
#if defined(_MSC_VER) && (defined(_M_X64) || defined(_M_IX86))
            _mm_pause();
#elif defined(__GNUC__) && (defined(__i386__) || defined(__x86_64__))
            __builtin_ia32_pause();
#elif defined(__arm__) && !defined(__aarch64__)
#if defined(__ARM_ARCH) && (__ARM_ARCH >= 7)
            __asm__ __volatile__("yield" ::: "memory");
#else
            __asm__ __volatile__("" ::: "memory");
#endif
#else
            (void)0;
#endif
        }

        /// [TASK-022] HTS_Holo_Tensor_4D_Kernel 제거: PRNG 기반 행/열 순열 비활성(항등).
        static inline void holo4d_task022_identity_params(
            uint16_t* row_workspace,
            uint16_t* col_perm,
            uint16_t K,
            uint16_t N) noexcept
        {
            if (N == 0u || N > HOLO_CHIP_COUNT) { return; }
            if (K == 0u || K > N) { return; }
            if (row_workspace == nullptr || col_perm == nullptr) { return; }
            for (uint16_t j = 0u; j < N; ++j) {
                row_workspace[static_cast<size_t>(j)] = j;
                col_perm[static_cast<size_t>(j)] = j;
            }
        }
    } // namespace

    struct HTS_Holo_Tensor_4D_RX::Impl {
        HoloState state;
        uint8_t   cfi_violation_count;
        uint8_t   pad_[2];
        uint32_t  decode_count;
        static constexpr uint16_t ACCUM_SIZE =
            (HOLO_MAX_BLOCK_BITS >= HOLO_CHIP_COUNT)
            ? HOLO_MAX_BLOCK_BITS : HOLO_CHIP_COUNT;
        int32_t   accum[ACCUM_SIZE];
        uint16_t  scratch_rows[HOLO_CHIP_COUNT];
        uint16_t  scratch_perm[HOLO_CHIP_COUNT];
        int16_t   scratch_rx[HOLO_CHIP_COUNT];

        void Wipe_Sensitive() noexcept
        {
            SecureMemory::secureWipe(
                static_cast<void*>(scratch_rows), sizeof(scratch_rows));
            SecureMemory::secureWipe(
                static_cast<void*>(scratch_perm), sizeof(scratch_perm));
            SecureMemory::secureWipe(
                static_cast<void*>(scratch_rx), sizeof(scratch_rx));
            SecureMemory::secureWipe(
                static_cast<void*>(accum), sizeof(accum));
#if defined(__GNUC__) || defined(__clang__)
            __asm__ __volatile__("" ::: "memory");
#endif
            std::atomic_thread_fence(std::memory_order_release);
        }

        uint32_t Decode(HTS_Holo_Tensor_4D_RX& o, const int16_t* rx_chips,
            uint16_t N, uint64_t valid_mask,
            int8_t* output_bits, uint16_t K) noexcept
        {
            const uint8_t L = o.profile_.num_layers;
            const uint32_t L32d = static_cast<uint32_t>(L);
            const uint32_t K32d = static_cast<uint32_t>(K);
            const uint32_t N32d = static_cast<uint32_t>(N);
            const uint32_t valid =
                static_cast<uint32_t>(N <= HOLO_CHIP_COUNT) &
                static_cast<uint32_t>(K <= HOLO_MAX_BLOCK_BITS) &
                static_cast<uint32_t>(K <= N) &
                static_cast<uint32_t>(N != 0u) &
                static_cast<uint32_t>((N & static_cast<uint16_t>(N - 1u)) == 0u) &
                static_cast<uint32_t>(L32d * K32d <= N32d);
            const uint16_t N_eff = static_cast<uint16_t>(N32d * valid);
            const uint16_t K_eff = static_cast<uint16_t>(K32d * valid);

#if defined(HTS_HOLO_RX_DECODE_AUDIT)
            {
                std::printf("[AUDIT-IN] rx_chips[0..7]=");
                for (uint16_t c = 0u; c < 8u && c < N; ++c) {
                    std::printf("%d ", static_cast<int>(rx_chips[static_cast<size_t>(c)]));
                }
                std::printf("\n");
                std::printf("[AUDIT-IN] rx_chips[56..63]=");
                {
                    const uint16_t c0 = (N >= 8u) ? static_cast<uint16_t>(N - 8u) : 0u;
                    for (uint16_t c = c0; c < N; ++c) {
                        std::printf("%d ", static_cast<int>(rx_chips[static_cast<size_t>(c)]));
                    }
                }
                std::printf("\n");
                std::printf(
                    "[AUDIT-IN] N=%u K=%u L=%u valid_mask=%016llX\n",
                    static_cast<unsigned>(N),
                    static_cast<unsigned>(K),
                    static_cast<unsigned>(o.profile_.num_layers),
                    static_cast<unsigned long long>(valid_mask));
            }
#endif

            holo4d_task022_identity_params(
                scratch_rows, scratch_perm, K_eff, N_eff);

            const uint64_t mask_bits = 0ull; /* [TASK-022] phase mask off */

#if defined(HTS_CFO_V5A_S5H_STEPD_RX)
            std::printf(
                "[RX-SYNC] seed=[%08X,%08X,%08X,%08X] tslot=%u L=%u K=%u N=%u "
                "profK=%u profN=%u\n",
                o.master_seed_[0], o.master_seed_[1], o.master_seed_[2],
                o.master_seed_[3], static_cast<unsigned>(o.time_slot_),
                static_cast<unsigned>(L), static_cast<unsigned>(K_eff),
                static_cast<unsigned>(N_eff),
                static_cast<unsigned>(o.profile_.block_bits),
                static_cast<unsigned>(o.profile_.chip_count));
            std::printf("[RX-MASK] %016llX\n",
                        static_cast<unsigned long long>(mask_bits));
            std::printf(
                "[RX-PERM] %d,%d,%d,%d,%d,%d,%d,%d (first 8)\n",
                static_cast<int>(scratch_perm[0]),
                static_cast<int>(scratch_perm[1]),
                static_cast<int>(scratch_perm[2]),
                static_cast<int>(scratch_perm[3]),
                static_cast<int>(scratch_perm[4]),
                static_cast<int>(scratch_perm[5]),
                static_cast<int>(scratch_perm[6]),
                static_cast<int>(scratch_perm[7]));
            std::printf(
                "[RX-ROWS] %d,%d,%d,%d,%d,%d,%d,%d (first 8 of L*K)\n",
                static_cast<int>(scratch_rows[0]),
                static_cast<int>(scratch_rows[1]),
                static_cast<int>(scratch_rows[2]),
                static_cast<int>(scratch_rows[3]),
                static_cast<int>(scratch_rows[4]),
                static_cast<int>(scratch_rows[5]),
                static_cast<int>(scratch_rows[6]),
                static_cast<int>(scratch_rows[7]));
#endif

            std::memset(scratch_rx, 0, static_cast<size_t>(N_eff) * sizeof(int16_t));
            for (uint16_t i = 0u; i < N_eff; ++i) {
                const uint16_t phys = scratch_perm[static_cast<size_t>(i)];
                const uint32_t phys_u = static_cast<uint32_t>(phys) & 63u;
                const uint32_t vbit = static_cast<uint32_t>(
                    (valid_mask >> phys_u) & 1ull);
                const int32_t chip_valid = -static_cast<int32_t>(vbit);
                int32_t rx_val =
                    static_cast<int32_t>(rx_chips[static_cast<size_t>(phys_u)]) & chip_valid;
                const int32_t ms = 1 - (static_cast<int32_t>(
                    (mask_bits >> static_cast<uint64_t>(i)) & 1ull) << 1);
                scratch_rx[static_cast<size_t>(i)] =
                    static_cast<int16_t>(rx_val * ms);
            }

#if defined(HTS_HOLO_RX_DECODE_AUDIT)
            {
                std::printf("[AUDIT-SCR] scratch_rx[0..15]=");
                for (uint16_t i = 0u; i < 16u && i < N_eff; ++i) {
                    std::printf("%d ", static_cast<int>(scratch_rx[static_cast<size_t>(i)]));
                }
                std::printf("\n");
                std::printf("[AUDIT-SCR] scratch_rx[16..31]=");
                for (uint16_t i = 16u; i < 32u && i < N_eff; ++i) {
                    std::printf("%d ", static_cast<int>(scratch_rx[static_cast<size_t>(i)]));
                }
                std::printf("\n");
            }
#endif

            std::memset(accum, 0, static_cast<size_t>(K_eff) * sizeof(int32_t));

            for (uint16_t k = 0u; k < K_eff; ++k) {
                int32_t sumL = 0;
                for (uint8_t layer = 0u; layer < L; ++layer) {
                    const uint16_t row_offset = static_cast<uint16_t>(
                        static_cast<uint32_t>(layer) * K32d);
                    const uint16_t* row_sel =
                        &scratch_rows[static_cast<size_t>(row_offset)];
                    const uint32_t row_k =
                        static_cast<uint32_t>(row_sel[static_cast<size_t>(k)]);
                    (void)row_k; /* [TASK-022] Walsh stubbed */
                    int32_t acc = 0;
                    uint16_t ii = 0u;
                    for (; ii + 3u < N_eff; ii += 4u) {
                        const int32_t w0 = 0; /* [TASK-022] Walsh off */
                        const int32_t w1 = 0;
                        const int32_t w2 = 0;
                        const int32_t w3 = 0;
                        const int32_t r0 =
                            static_cast<int32_t>(scratch_rx[static_cast<size_t>(ii)]);
                        const int32_t r1 =
                            static_cast<int32_t>(scratch_rx[static_cast<size_t>(ii + 1u)]);
                        const int32_t r2 =
                            static_cast<int32_t>(scratch_rx[static_cast<size_t>(ii + 2u)]);
                        const int32_t r3 =
                            static_cast<int32_t>(scratch_rx[static_cast<size_t>(ii + 3u)]);
                        acc += r0 * w0 + r1 * w1 + r2 * w2 + r3 * w3;
                    }
                    for (; ii < N_eff; ++ii) {
                        const int8_t w = 0; /* [TASK-022] Walsh off */
                        acc += static_cast<int32_t>(scratch_rx[static_cast<size_t>(ii)]) *
                            static_cast<int32_t>(w);
                    }
                    sumL += acc;
                }
                accum[static_cast<size_t>(k)] += sumL;
            }

#if defined(HTS_HOLO_RX_DECODE_AUDIT)
            {
                std::printf("[AUDIT-ACC] accum[0..K-1]=");
                for (uint16_t k = 0u; k < K_eff; ++k) {
                    std::printf("%lld ",
                        static_cast<long long>(accum[static_cast<size_t>(k)]));
                }
                std::printf("\n");
            }
#endif

            for (uint16_t k = 0u; k < K_eff; ++k) {
                const uint32_t sign_bit =
                    static_cast<uint32_t>(accum[static_cast<size_t>(k)]) >> 31u;
                output_bits[static_cast<size_t>(k)] = static_cast<int8_t>(
                    1 - 2 * static_cast<int32_t>(sign_bit));
            }
#if defined(HTS_HOLO_RX_DECODE_AUDIT)
            {
                std::printf("[AUDIT-OUT] bits=");
                for (uint16_t k = 0u; k < K_eff; ++k) {
                    std::printf("%d ", static_cast<int>(output_bits[static_cast<size_t>(k)]));
                }
                std::printf("\n");
            }
#endif
            Wipe_Sensitive();
            return (HTS_Holo_Tensor_4D_RX::SECURE_TRUE & (0u - valid)) |
                (HTS_Holo_Tensor_4D_RX::SECURE_FALSE & (0u - (valid ^ 1u)));
        }
    };

    namespace {
        /// 인접 칩 lag=1 복소 상관 Σ z[n+1]·conj(z[n]) → arg(Σ) = 잔여 dφ(Q16/칩).
        /// (제곱 위상 추정은 저 CFO에서 과회전을 유발할 수 있어 선형 누적 사용.)
        /// lag 항마다 동일한 칩간 위상이면 arg(Σ)가 그 위상과 같으므로 /(N-1) 불필요.
        static int32_t holo4d_estimate_residual_dphi_q16(
            const int16_t* rx_I, const int16_t* rx_Q, uint16_t N) noexcept
        {
            if (N < 2u) { return 0; }
            int64_t sum_I = 0;
            int64_t sum_Q = 0;
            for (uint16_t c = 0u; c + 1u < N; ++c) {
                const int32_t i0 = static_cast<int32_t>(rx_I[static_cast<size_t>(c)]);
                const int32_t q0 = static_cast<int32_t>(rx_Q[static_cast<size_t>(c)]);
                const int32_t i1 = static_cast<int32_t>(rx_I[static_cast<size_t>(c + 1u)]);
                const int32_t q1 = static_cast<int32_t>(rx_Q[static_cast<size_t>(c + 1u)]);
                const int64_t c_I = static_cast<int64_t>(i1) * i0 +
                    static_cast<int64_t>(q1) * q0;
                const int64_t c_Q = static_cast<int64_t>(q1) * i0 -
                    static_cast<int64_t>(i1) * q0;
                sum_I += c_I;
                sum_Q += c_Q;
            }
            int32_t sh = 0;
            for (;;) {
                const int64_t tI = sum_I >> sh;
                const int64_t tQ = sum_Q >> sh;
                const int64_t ax = tI >> 63;
                const int64_t ai = (tI ^ ax) - ax;
                const int64_t ay = tQ >> 63;
                const int64_t aq = (tQ ^ ay) - ay;
                const int64_t m = (ai > aq) ? ai : aq;
                if (m <= static_cast<int64_t>(2147483647) || sh >= 40) {
                    break;
                }
                ++sh;
            }
            const int32_t I_clamped = static_cast<int32_t>(sum_I >> sh);
            const int32_t Q_clamped = static_cast<int32_t>(sum_Q >> sh);
            const int32_t dphi_q16 = Holo4D_Atan2_Q16(Q_clamped, I_clamped);
            const int32_t over_max = dphi_q16 - 4096;
            const int32_t over_max_mask = ~(over_max >> 31);
            int32_t result = (dphi_q16 & ~over_max_mask) | (4096 & over_max_mask);
            const int32_t under_min = -4096 - result;
            const int32_t under_min_mask = ~(under_min >> 31);
            result = (result & ~under_min_mask) | (-4096 & under_min_mask);
            return result;
        }

        static void holo4d_chip_derotate_q15(
            const int16_t* rx_I, const int16_t* rx_Q,
            int32_t dphi_q16, uint16_t N,
            int16_t* out_I, int16_t* out_Q) noexcept
        {
            int32_t phi_q16 = 0;
            for (uint16_t c = 0u; c < N; ++c) {
                const uint16_t phi_lut = static_cast<uint16_t>(
                    static_cast<uint32_t>(phi_q16));
                const int32_t cos_q15 = static_cast<int32_t>(Cos_Q15(phi_lut));
                const int32_t sin_q15 = static_cast<int32_t>(Sin_Q15(phi_lut));
                const int32_t I = static_cast<int32_t>(rx_I[static_cast<size_t>(c)]);
                const int32_t Q = static_cast<int32_t>(rx_Q[static_cast<size_t>(c)]);
                int32_t Ip = (I * cos_q15 + Q * sin_q15) >> 15;
                int32_t Qp = (Q * cos_q15 - I * sin_q15) >> 15;
                if (Ip > 32767) { Ip = 32767; }
                else if (Ip < -32768) { Ip = -32768; }
                if (Qp > 32767) { Qp = 32767; }
                else if (Qp < -32768) { Qp = -32768; }
                out_I[static_cast<size_t>(c)] = static_cast<int16_t>(Ip);
                out_Q[static_cast<size_t>(c)] = static_cast<int16_t>(Qp);
                phi_q16 += dphi_q16;
            }
        }

    } // namespace

    uint32_t detail_holo4d_decode_two_candidates(
        HTS_Holo_Tensor_4D_RX& o,
        HTS_Holo_Tensor_4D_RX::Impl* im,
        const int16_t* rx_I, const int16_t* rx_Q,
        uint16_t N, uint64_t valid_mask,
        int8_t* output_bits_cand0,
        int8_t* output_bits_cand1,
        uint16_t K,
        int32_t* output_metric) noexcept
    {
        if (output_bits_cand0 == nullptr || output_bits_cand1 == nullptr) {
            return HTS_Holo_Tensor_4D_RX::SECURE_FALSE;
        }
        const uint8_t L = o.profile_.num_layers;
        const uint32_t L32d = static_cast<uint32_t>(L);
        const uint32_t K32d = static_cast<uint32_t>(K);
        const uint32_t N32d = static_cast<uint32_t>(N);
        const uint32_t valid =
            static_cast<uint32_t>(N <= HOLO_CHIP_COUNT) &
            static_cast<uint32_t>(K <= HOLO_MAX_BLOCK_BITS) &
            static_cast<uint32_t>(K <= N) &
            static_cast<uint32_t>(N != 0u) &
            static_cast<uint32_t>((N & static_cast<uint16_t>(N - 1u)) == 0u) &
            static_cast<uint32_t>(L32d * K32d <= N32d);
        const uint16_t N_eff = static_cast<uint16_t>(N32d * valid);
        const uint16_t K_eff = static_cast<uint16_t>(K32d * valid);

        holo4d_task022_identity_params(
            im->scratch_rows, im->scratch_perm, K_eff, N_eff);

        const uint64_t mask_bits = 0ull; /* [TASK-022] phase mask off */

#if defined(HTS_CFO_V5A_S5H_STEPD_RX)
        std::printf(
            "[RX-SYNC-PH] seed=[%08X,%08X,%08X,%08X] tslot=%u L=%u K=%u N=%u\n",
            o.master_seed_[0], o.master_seed_[1], o.master_seed_[2],
            o.master_seed_[3], static_cast<unsigned>(o.time_slot_),
            static_cast<unsigned>(L), static_cast<unsigned>(K_eff),
            static_cast<unsigned>(N_eff));
#endif

        int16_t der_I[HOLO_CHIP_COUNT];
        int16_t der_Q[HOLO_CHIP_COUNT];

        const int32_t dphi_q16 =
            holo4d_estimate_residual_dphi_q16(rx_I, rx_Q, N_eff);
        holo4d_chip_derotate_q15(
            rx_I, rx_Q, dphi_q16, N_eff, der_I, der_Q);

        int16_t rx_comb_phys[HOLO_CHIP_COUNT];
        for (uint16_t c = 0u; c < N_eff; ++c) {
            const int32_t s = static_cast<int32_t>(der_I[static_cast<size_t>(c)]) +
                static_cast<int32_t>(der_Q[static_cast<size_t>(c)]);
            rx_comb_phys[static_cast<size_t>(c)] = static_cast<int16_t>(s / 2);
        }

        std::memset(im->scratch_rx, 0, static_cast<size_t>(N_eff) * sizeof(int16_t));
        for (uint16_t i = 0u; i < N_eff; ++i) {
            const uint16_t phys = im->scratch_perm[static_cast<size_t>(i)];
            const uint32_t phys_u = static_cast<uint32_t>(phys) & 63u;
            const uint32_t vbit = static_cast<uint32_t>(
                (valid_mask >> phys_u) & 1ull);
            const int32_t chip_valid = -static_cast<int32_t>(vbit);
            int32_t rx_val = static_cast<int32_t>(
                rx_comb_phys[static_cast<size_t>(phys_u)]) & chip_valid;
            const int32_t ms = 1 - (static_cast<int32_t>(
                (mask_bits >> static_cast<uint64_t>(i)) & 1ull) << 1);
            im->scratch_rx[static_cast<size_t>(i)] =
                static_cast<int16_t>(rx_val * ms);
        }

        std::memset(im->accum, 0, static_cast<size_t>(K_eff) * sizeof(int32_t));

        for (uint16_t k = 0u; k < K_eff; ++k) {
            int32_t sumL = 0;
            for (uint8_t layer = 0u; layer < L; ++layer) {
                const uint16_t row_offset = static_cast<uint16_t>(
                    static_cast<uint32_t>(layer) * K32d);
                const uint16_t* row_sel =
                    &im->scratch_rows[static_cast<size_t>(row_offset)];
                const uint32_t row_k =
                    static_cast<uint32_t>(row_sel[static_cast<size_t>(k)]);
                (void)row_k; /* [TASK-022] Walsh stubbed */
                int32_t acc = 0;
                uint16_t ii = 0u;
                for (; ii + 3u < N_eff; ii += 4u) {
                    const int32_t w0 = 0; /* [TASK-022] Walsh off */
                    const int32_t w1 = 0;
                    const int32_t w2 = 0;
                    const int32_t w3 = 0;
                    const int32_t r0 =
                        static_cast<int32_t>(im->scratch_rx[static_cast<size_t>(ii)]);
                    const int32_t r1 =
                        static_cast<int32_t>(im->scratch_rx[static_cast<size_t>(ii + 1u)]);
                    const int32_t r2 =
                        static_cast<int32_t>(im->scratch_rx[static_cast<size_t>(ii + 2u)]);
                    const int32_t r3 =
                        static_cast<int32_t>(im->scratch_rx[static_cast<size_t>(ii + 3u)]);
                    acc += r0 * w0 + r1 * w1 + r2 * w2 + r3 * w3;
                }
                for (; ii < N_eff; ++ii) {
                    const int8_t w = 0; /* [TASK-022] Walsh off */
                    acc += static_cast<int32_t>(
                        im->scratch_rx[static_cast<size_t>(ii)]) *
                        static_cast<int32_t>(w);
                }
                sumL += acc;
            }
            im->accum[static_cast<size_t>(k)] += sumL;
        }

        for (uint16_t k = 0u; k < K_eff; ++k) {
            const uint32_t sign_bit =
                static_cast<uint32_t>(im->accum[static_cast<size_t>(k)]) >> 31u;
            const int8_t b0 = static_cast<int8_t>(
                1 - 2 * static_cast<int32_t>(sign_bit));
            output_bits_cand0[static_cast<size_t>(k)] = b0;
            output_bits_cand1[static_cast<size_t>(k)] = static_cast<int8_t>(-b0);
        }

        if (output_metric != nullptr) {
            for (uint16_t k = 0u; k < K_eff; ++k) {
                output_metric[static_cast<size_t>(k)] =
                    im->accum[static_cast<size_t>(k)];
            }
        }

        SecureMemory::secureWipe(static_cast<void*>(der_I), sizeof(der_I));
        SecureMemory::secureWipe(static_cast<void*>(der_Q), sizeof(der_Q));
        SecureMemory::secureWipe(
            static_cast<void*>(rx_comb_phys), sizeof(rx_comb_phys));
        im->Wipe_Sensitive();
        return (HTS_Holo_Tensor_4D_RX::SECURE_TRUE & (0u - valid)) |
            (HTS_Holo_Tensor_4D_RX::SECURE_FALSE & (0u - (valid ^ 1u)));
    }

    HTS_Holo_Tensor_4D_RX::HTS_Holo_Tensor_4D_RX() noexcept
        : initialized_{ false }
    {
        static_assert(sizeof(Impl) <= IMPL_BUF_SIZE, "Holo4D_RX::Impl OOB");
        time_slot_ = 0u;
        master_seed_[0] = master_seed_[1] = master_seed_[2] = master_seed_[3] = 0u;
        std::memset(impl_buf_, 0, IMPL_BUF_SIZE);
    }

    HTS_Holo_Tensor_4D_RX::~HTS_Holo_Tensor_4D_RX() noexcept
    {
        if (!initialized_.load(std::memory_order_acquire)) { return; }
        while (op_busy_.test_and_set(std::memory_order_acq_rel)) {
            holo4d_rx_busy_spin_yield();
        }
#if defined(__arm__) && !defined(__aarch64__)
        Armv7m_Irq_Mask_Guard irq_primask;
#endif
        reinterpret_cast<Impl*>(impl_buf_)->~Impl();
        SecureMemory::secureWipe(static_cast<void*>(impl_buf_), IMPL_BUF_SIZE);
#if defined(__GNUC__) || defined(__clang__)
        __asm__ __volatile__("" ::: "memory");
#endif
        std::atomic_thread_fence(std::memory_order_release);
        initialized_.store(false, std::memory_order_release);
        op_busy_.clear(std::memory_order_release);
    }

    uint32_t HTS_Holo_Tensor_4D_RX::Initialize(
        const uint32_t master_seed[4],
        const HoloTensor_Profile* profile) noexcept
    {
        if (master_seed == nullptr) { return SECURE_FALSE; }
        Holo4D_RX_Busy_Guard g(op_busy_);
        if (!g.locked) { return SECURE_FALSE; }
        bool ex = false;
        if (!initialized_.compare_exchange_strong(
            ex, true, std::memory_order_acq_rel)) {
            return SECURE_TRUE;
        }
        Impl* im = new (impl_buf_) Impl{};
        master_seed_[0] = master_seed[0];
        master_seed_[1] = master_seed[1];
        master_seed_[2] = master_seed[2];
        master_seed_[3] = master_seed[3];
        if (profile != nullptr) { profile_ = *profile; }
        else { profile_ = k_holo_profiles[1]; }
        if (profile_.block_bits == 0u ||
            profile_.block_bits > HOLO_MAX_BLOCK_BITS) {
            profile_.block_bits = HOLO_DEFAULT_BLOCK;
        }
        if (profile_.chip_count == 0u ||
            profile_.chip_count > HOLO_CHIP_COUNT) {
            profile_.chip_count = HOLO_CHIP_COUNT;
        }
        if (profile_.num_layers == 0u ||
            profile_.num_layers > HOLO_MAX_LAYERS) {
            profile_.num_layers = HOLO_DEFAULT_LAYERS;
        }
        im->state = HoloState::OFFLINE;
        im->cfi_violation_count = 0u;
        im->decode_count = 0u;
        for (uint32_t i = 0u; i < HOLO_MAX_BLOCK_BITS; ++i) {
            im->accum[static_cast<size_t>(i)] = 0;
        }
        time_slot_ = 0u;
        im->state = HoloState::READY; /* [TASK-022] CFI stub */
        return SECURE_TRUE;
    }

    void HTS_Holo_Tensor_4D_RX::Shutdown() noexcept
    {
        if (!initialized_.load(std::memory_order_acquire)) { return; }
        while (op_busy_.test_and_set(std::memory_order_acq_rel)) {
            holo4d_rx_busy_spin_yield();
        }
#if defined(__arm__) && !defined(__aarch64__)
        Armv7m_Irq_Mask_Guard irq_primask;
#endif
        if (!initialized_.load(std::memory_order_acquire)) {
            op_busy_.clear(std::memory_order_release);
            return;
        }
        reinterpret_cast<Impl*>(impl_buf_)->~Impl();
        SecureMemory::secureWipe(static_cast<void*>(impl_buf_), IMPL_BUF_SIZE);
#if defined(__GNUC__) || defined(__clang__)
        __asm__ __volatile__("" ::: "memory");
#endif
        std::atomic_thread_fence(std::memory_order_release);
        initialized_.store(false, std::memory_order_release);
        op_busy_.clear(std::memory_order_release);
    }

    void HTS_Holo_Tensor_4D_RX::Rotate_Seed(
        const uint32_t new_seed[4]) noexcept
    {
        if (new_seed == nullptr) { return; }
        Holo4D_RX_Busy_Guard g(op_busy_);
        if (!g.locked) { return; }
        if (!initialized_.load(std::memory_order_acquire)) { return; }
        SecureMemory::secureWipe(
            static_cast<void*>(master_seed_), sizeof(master_seed_));
        master_seed_[0] = new_seed[0];
        master_seed_[1] = new_seed[1];
        master_seed_[2] = new_seed[2];
        master_seed_[3] = new_seed[3];
        std::atomic_thread_fence(std::memory_order_release);
    }

    uint32_t HTS_Holo_Tensor_4D_RX::Set_Profile(
        const HoloTensor_Profile* profile) noexcept
    {
        if (profile == nullptr) { return SECURE_FALSE; }
        Holo4D_RX_Busy_Guard g(op_busy_);
        if (!g.locked) { return SECURE_FALSE; }
        if (!initialized_.load(std::memory_order_acquire)) { return SECURE_FALSE; }
        profile_ = *profile;
        if (profile_.block_bits == 0u ||
            profile_.block_bits > HOLO_MAX_BLOCK_BITS) {
            profile_.block_bits = HOLO_DEFAULT_BLOCK;
        }
        if (profile_.chip_count == 0u || profile_.chip_count > HOLO_CHIP_COUNT) {
            profile_.chip_count = HOLO_CHIP_COUNT;
        }
        if (profile_.num_layers == 0u || profile_.num_layers > HOLO_MAX_LAYERS) {
            profile_.num_layers = HOLO_DEFAULT_LAYERS;
        }
        return SECURE_TRUE;
    }

    uint32_t HTS_Holo_Tensor_4D_RX::Decode_Block(
        const int16_t* rx_chips, uint16_t N,
        uint64_t valid_mask,
        int8_t* output_bits, uint16_t K) noexcept
    {
        if (rx_chips == nullptr) { return SECURE_FALSE; }
        if (output_bits == nullptr) { return SECURE_FALSE; }
        Holo4D_RX_Busy_Guard g(op_busy_);
        if (!g.locked) { return SECURE_FALSE; }
        if (!initialized_.load(std::memory_order_acquire)) { return SECURE_FALSE; }
        Impl* im = reinterpret_cast<Impl*>(impl_buf_);
        im->state = HoloState::DECODING; /* [TASK-022] CFI stub */
        const uint32_t ok = im->Decode(
            *this, rx_chips, N, valid_mask, output_bits, K);
        im->state = HoloState::READY;
        if (ok == SECURE_TRUE) { ++im->decode_count; }
        return ok;
    }

    uint32_t HTS_Holo_Tensor_4D_RX::Decode_Block_Two_Candidates(
        const int16_t* rx_I, const int16_t* rx_Q,
        uint16_t N, uint64_t valid_mask,
        int8_t* output_bits_cand0,
        int8_t* output_bits_cand1,
        uint16_t K) noexcept
    {
        if (rx_I == nullptr) { return SECURE_FALSE; }
        if (rx_Q == nullptr) { return SECURE_FALSE; }
        if (output_bits_cand0 == nullptr) { return SECURE_FALSE; }
        if (output_bits_cand1 == nullptr) { return SECURE_FALSE; }
        Holo4D_RX_Busy_Guard g(op_busy_);
        if (!g.locked) { return SECURE_FALSE; }
        if (!initialized_.load(std::memory_order_acquire)) { return SECURE_FALSE; }
        Impl* im = reinterpret_cast<Impl*>(impl_buf_);
        im->state = HoloState::DECODING; /* [TASK-022] CFI stub */
        const uint32_t ok = detail_holo4d_decode_two_candidates(
            *this, im, rx_I, rx_Q, N, valid_mask, output_bits_cand0, output_bits_cand1,
            K, nullptr);
        im->state = HoloState::READY;
        if (ok == SECURE_TRUE) { ++im->decode_count; }
        return ok;
    }

    uint32_t HTS_Holo_Tensor_4D_RX::Decode_Block_Two_Candidates_With_Metric(
        const int16_t* rx_I, const int16_t* rx_Q,
        uint16_t N, uint64_t valid_mask,
        int8_t* output_bits_cand0,
        int8_t* output_bits_cand1,
        int32_t* output_metric,
        uint16_t K) noexcept
    {
        if (rx_I == nullptr) { return SECURE_FALSE; }
        if (rx_Q == nullptr) { return SECURE_FALSE; }
        if (output_bits_cand0 == nullptr) { return SECURE_FALSE; }
        if (output_bits_cand1 == nullptr) { return SECURE_FALSE; }
        if (output_metric == nullptr) { return SECURE_FALSE; }
        Holo4D_RX_Busy_Guard g(op_busy_);
        if (!g.locked) { return SECURE_FALSE; }
        if (!initialized_.load(std::memory_order_acquire)) { return SECURE_FALSE; }
        Impl* im = reinterpret_cast<Impl*>(impl_buf_);
        im->state = HoloState::DECODING; /* [TASK-022] CFI stub */
        const uint32_t ok = detail_holo4d_decode_two_candidates(
            *this, im, rx_I, rx_Q, N, valid_mask, output_bits_cand0, output_bits_cand1,
            K, output_metric);
        im->state = HoloState::READY;
        if (ok == SECURE_TRUE) { ++im->decode_count; }
        return ok;
    }

    uint32_t HTS_Holo_Tensor_4D_RX::Decode_Block_With_Phase(
        const int16_t* rx_I, const int16_t* rx_Q,
        uint16_t N, uint64_t valid_mask,
        int8_t* output_bits, uint16_t K,
        CrcVerifyFn crc_fn, void* crc_ctx) noexcept
    {
        if (rx_I == nullptr) { return SECURE_FALSE; }
        if (rx_Q == nullptr) { return SECURE_FALSE; }
        if (output_bits == nullptr) { return SECURE_FALSE; }
        Holo4D_RX_Busy_Guard g(op_busy_);
        if (!g.locked) { return SECURE_FALSE; }
        if (!initialized_.load(std::memory_order_acquire)) { return SECURE_FALSE; }
        Impl* im = reinterpret_cast<Impl*>(impl_buf_);
        im->state = HoloState::DECODING; /* [TASK-022] CFI stub */
        int8_t b0[HOLO_MAX_BLOCK_BITS];
        int8_t b1[HOLO_MAX_BLOCK_BITS];
        const uint32_t ok = detail_holo4d_decode_two_candidates(
            *this, im, rx_I, rx_Q, N, valid_mask, b0, b1, K, nullptr);
        if (ok != SECURE_TRUE) {
            im->state = HoloState::READY;
            return ok;
        }
        uint32_t mask_use_cand1 = 0u;
        if (crc_fn != nullptr) {
            const bool crc0 = crc_fn(b0, K, crc_ctx);
            const bool crc1 = crc_fn(b1, K, crc_ctx);
            if (crc1 && !crc0) {
                mask_use_cand1 = 0xFFFFFFFFu;
            } else if (!crc1 && crc0) {
                mask_use_cand1 = 0u;
            } else if (!crc0 && !crc1) {
                mask_use_cand1 = 0u;
            } else {
                mask_use_cand1 = 0u;
            }
        }
        for (uint16_t k = 0u; k < K; ++k) {
            const int32_t v0 = static_cast<int32_t>(b0[static_cast<size_t>(k)]);
            const int32_t v1 = static_cast<int32_t>(b1[static_cast<size_t>(k)]);
            output_bits[static_cast<size_t>(k)] = static_cast<int8_t>(
                (v0 & ~static_cast<int32_t>(mask_use_cand1)) |
                (v1 & static_cast<int32_t>(mask_use_cand1)));
        }
        SecureMemory::secureWipe(static_cast<void*>(b0), sizeof(b0));
        SecureMemory::secureWipe(static_cast<void*>(b1), sizeof(b1));
        im->state = HoloState::READY;
        if (ok == SECURE_TRUE) { ++im->decode_count; }
        return ok;
    }

    uint32_t HTS_Holo_Tensor_4D_RX::Advance_Time_Slot() noexcept
    {
        Holo4D_RX_Busy_Guard g(op_busy_);
        if (!g.locked) { return SECURE_FALSE; }
        if (!initialized_.load(std::memory_order_acquire)) { return SECURE_FALSE; }
        ++time_slot_;
        return SECURE_TRUE;
    }

    uint32_t HTS_Holo_Tensor_4D_RX::Set_Time_Slot(uint32_t frame_no) noexcept
    {
        Holo4D_RX_Busy_Guard g(op_busy_);
        if (!g.locked) { return SECURE_FALSE; }
        if (!initialized_.load(std::memory_order_acquire)) { return SECURE_FALSE; }
        time_slot_ = frame_no;
        return SECURE_TRUE;
    }

    HoloState HTS_Holo_Tensor_4D_RX::Get_State() const noexcept
    {
        Holo4D_RX_Busy_Guard g(op_busy_);
        if (!g.locked) { return HoloState::OFFLINE; }
        if (!initialized_.load(std::memory_order_acquire)) { return HoloState::OFFLINE; }
        return reinterpret_cast<const Impl*>(impl_buf_)->state;
    }

    uint32_t HTS_Holo_Tensor_4D_RX::Get_Decode_Count() const noexcept
    {
        Holo4D_RX_Busy_Guard g(op_busy_);
        if (!g.locked) { return 0u; }
        if (!initialized_.load(std::memory_order_acquire)) { return 0u; }
        return reinterpret_cast<const Impl*>(impl_buf_)->decode_count;
    }

    uint32_t HTS_Holo_Tensor_4D_RX::Get_Time_Slot() const noexcept
    {
        Holo4D_RX_Busy_Guard g(op_busy_);
        if (!g.locked) { return 0u; }
        if (!initialized_.load(std::memory_order_acquire)) { return 0u; }
        return time_slot_;
    }

} // namespace ProtectedEngine
