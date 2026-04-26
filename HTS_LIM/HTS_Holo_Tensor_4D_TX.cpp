/// @file  HTS_Holo_Tensor_4D_TX.cpp
#include "HTS_Holo_Tensor_4D_TX.h"
#include "HTS_Holo_Tensor_4D_Common.h"
#include "HTS_Arm_Irq_Mask_Guard.h"
#include "HTS_Secure_Memory.h"
#include <new>
#include <cstring>
#if defined(HTS_CFO_V5A_S5H_ENCODE_DIAG) || defined(HTS_CFO_V5A_S5H_STEPC_FULL) || \
    defined(HTS_CFO_V5A_S5H_STEPD_TX) || defined(HTS_HOLO_TX_ENCODE_AUDIT)
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
        struct Holo4D_TX_Busy_Guard final {
            std::atomic_flag* flag;
            bool locked;
            explicit Holo4D_TX_Busy_Guard(std::atomic_flag& f) noexcept
                : flag(&f), locked(false) {
                locked = !flag->test_and_set(std::memory_order_acq_rel);
            }
            ~Holo4D_TX_Busy_Guard() noexcept {
                if (locked) { flag->clear(std::memory_order_release); }
            }
            Holo4D_TX_Busy_Guard(const Holo4D_TX_Busy_Guard&) = delete;
            Holo4D_TX_Busy_Guard& operator=(const Holo4D_TX_Busy_Guard&) = delete;
        };

        static inline void holo4d_tx_busy_spin_yield() noexcept {
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
    } // namespace

    struct HTS_Holo_Tensor_4D_TX::Impl {
        HoloState state;
        uint8_t   cfi_violation_count;
        uint8_t   pad_[2];
        uint32_t  encode_count;
        static constexpr uint16_t ACCUM_SIZE =
            (HOLO_MAX_BLOCK_BITS >= HOLO_CHIP_COUNT)
            ? HOLO_MAX_BLOCK_BITS : HOLO_CHIP_COUNT;
        int32_t   accum[ACCUM_SIZE];
        uint16_t  scratch_rows[HOLO_CHIP_COUNT];
        uint16_t  scratch_perm[HOLO_CHIP_COUNT];

        static_assert(ACCUM_SIZE >= HOLO_CHIP_COUNT, "accum N");
        static_assert(ACCUM_SIZE >= HOLO_MAX_BLOCK_BITS, "accum K");

        void Wipe_Sensitive() noexcept
        {
            SecureMemory::secureWipe(
                static_cast<void*>(scratch_rows), sizeof(scratch_rows));
            SecureMemory::secureWipe(
                static_cast<void*>(scratch_perm), sizeof(scratch_perm));
            SecureMemory::secureWipe(
                static_cast<void*>(accum), sizeof(accum));
#if defined(__GNUC__) || defined(__clang__)
            __asm__ __volatile__("" ::: "memory");
#endif
            std::atomic_thread_fence(std::memory_order_release);
        }

        uint32_t Encode(HTS_Holo_Tensor_4D_TX& o, const int8_t* data,
            uint16_t K, int8_t* chips, uint16_t N) noexcept
        {
            const uint8_t L = o.profile_.num_layers;
            const uint32_t L32e = static_cast<uint32_t>(L);
            const uint32_t K32e = static_cast<uint32_t>(K);
            const uint32_t N32e = static_cast<uint32_t>(N);
            const uint32_t valid =
                static_cast<uint32_t>(N <= HOLO_CHIP_COUNT) &
                static_cast<uint32_t>(K <= HOLO_MAX_BLOCK_BITS) &
                static_cast<uint32_t>(K <= N) &
                static_cast<uint32_t>(N != 0u) &
                static_cast<uint32_t>((N & static_cast<uint16_t>(N - 1u)) == 0u) &
                static_cast<uint32_t>(L32e * K32e <= N32e);
            const uint16_t N_eff = static_cast<uint16_t>(N32e * valid);
            const uint16_t K_eff = static_cast<uint16_t>(K32e * valid);

            Holo4D_Generate_Partitioned_Params(
                o.master_seed_, o.time_slot_,
                scratch_rows, scratch_perm, K_eff, N_eff, L);

            const uint64_t mask_bits = Holo4D_Generate_Phase_Mask(
                o.master_seed_, o.time_slot_);

#if defined(HTS_CFO_V5A_S5H_STEPD_TX)
            std::printf(
                "[TX-SYNC] seed=[%08X,%08X,%08X,%08X] tslot=%u L=%u K=%u N=%u "
                "profK=%u profN=%u\n",
                o.master_seed_[0], o.master_seed_[1], o.master_seed_[2],
                o.master_seed_[3], static_cast<unsigned>(o.time_slot_),
                static_cast<unsigned>(L), static_cast<unsigned>(K_eff),
                static_cast<unsigned>(N_eff),
                static_cast<unsigned>(o.profile_.block_bits),
                static_cast<unsigned>(o.profile_.chip_count));
            std::printf("[TX-MASK] %016llX\n",
                        static_cast<unsigned long long>(mask_bits));
            std::printf(
                "[TX-PERM] %d,%d,%d,%d,%d,%d,%d,%d (first 8)\n",
                static_cast<int>(scratch_perm[0]),
                static_cast<int>(scratch_perm[1]),
                static_cast<int>(scratch_perm[2]),
                static_cast<int>(scratch_perm[3]),
                static_cast<int>(scratch_perm[4]),
                static_cast<int>(scratch_perm[5]),
                static_cast<int>(scratch_perm[6]),
                static_cast<int>(scratch_perm[7]));
            std::printf(
                "[TX-ROWS] %d,%d,%d,%d,%d,%d,%d,%d (first 8 of L*K)\n",
                static_cast<int>(scratch_rows[0]),
                static_cast<int>(scratch_rows[1]),
                static_cast<int>(scratch_rows[2]),
                static_cast<int>(scratch_rows[3]),
                static_cast<int>(scratch_rows[4]),
                static_cast<int>(scratch_rows[5]),
                static_cast<int>(scratch_rows[6]),
                static_cast<int>(scratch_rows[7]));
#endif

            std::memset(accum, 0, static_cast<size_t>(N_eff) * sizeof(int32_t));

            for (uint16_t i = 0u; i < N_eff; ++i) {
                const int32_t ms = 1 - (static_cast<int32_t>(
                    (mask_bits >> static_cast<uint64_t>(i)) & 1ull) << 1);
                const uint16_t phys = scratch_perm[static_cast<size_t>(i)];
                const uint32_t col_i = static_cast<uint32_t>(i);
                int32_t sumL = 0;
                for (uint8_t layer = 0u; layer < L; ++layer) {
                    const uint16_t row_offset = static_cast<uint16_t>(
                        static_cast<uint32_t>(layer) * K32e);
                    const uint16_t* row_sel =
                        &scratch_rows[static_cast<size_t>(row_offset)];
                    int32_t chip_acc = 0;
                    uint16_t k = 0u;
                    for (; k + 3u < K_eff; k += 4u) {
                        const int32_t dk0 =
                            static_cast<int32_t>(data[static_cast<size_t>(k)]);
                        const int32_t dk1 =
                            static_cast<int32_t>(data[static_cast<size_t>(k + 1u)]);
                        const int32_t dk2 =
                            static_cast<int32_t>(data[static_cast<size_t>(k + 2u)]);
                        const int32_t dk3 =
                            static_cast<int32_t>(data[static_cast<size_t>(k + 3u)]);
                        const uint32_t rk0 =
                            static_cast<uint32_t>(row_sel[static_cast<size_t>(k)]);
                        const uint32_t rk1 =
                            static_cast<uint32_t>(row_sel[static_cast<size_t>(k + 1u)]);
                        const uint32_t rk2 =
                            static_cast<uint32_t>(row_sel[static_cast<size_t>(k + 2u)]);
                        const uint32_t rk3 =
                            static_cast<uint32_t>(row_sel[static_cast<size_t>(k + 3u)]);
                        const int32_t w0 =
                            static_cast<int32_t>(Holo4D_Walsh_Code(rk0, col_i));
                        const int32_t w1 =
                            static_cast<int32_t>(Holo4D_Walsh_Code(rk1, col_i));
                        const int32_t w2 =
                            static_cast<int32_t>(Holo4D_Walsh_Code(rk2, col_i));
                        const int32_t w3 =
                            static_cast<int32_t>(Holo4D_Walsh_Code(rk3, col_i));
                        chip_acc += dk0 * w0 + dk1 * w1 + dk2 * w2 + dk3 * w3;
                    }
                    for (; k < K_eff; ++k) {
                        const int8_t w = Holo4D_Walsh_Code(
                            static_cast<uint32_t>(row_sel[static_cast<size_t>(k)]),
                            col_i);
                        chip_acc += static_cast<int32_t>(data[static_cast<size_t>(k)]) *
                            static_cast<int32_t>(w);
                    }
                    sumL += chip_acc;
                }
                accum[static_cast<size_t>(phys)] += sumL * ms;
            }

            for (uint16_t i = 0u; i < N_eff; ++i) {
#if defined(__arm__) || defined(__TARGET_ARCH_ARM)
                chips[static_cast<size_t>(i)] = static_cast<int8_t>(
                    __SSAT(accum[static_cast<size_t>(i)], 8));
#else
                const int32_t v = accum[static_cast<size_t>(i)];
                const uint32_t ov_hi = static_cast<uint32_t>(v > 127);
                const uint32_t ov_lo = static_cast<uint32_t>(v < -127);
                const int32_t mhi = -static_cast<int32_t>(ov_hi);
                const int32_t mlo = -static_cast<int32_t>(ov_lo);
                chips[static_cast<size_t>(i)] = static_cast<int8_t>(
                    (127 & mhi) | (-127 & mlo) | (v & ~mhi & ~mlo));
#endif
            }
#if defined(HTS_HOLO_TX_ENCODE_AUDIT)
            {
                std::printf("[AUDIT-TX-CHIP] chips[0..7]=");
                for (uint16_t i = 0u; i < 8u && i < N_eff; ++i) {
                    std::printf("%d ", static_cast<int>(chips[static_cast<size_t>(i)]));
                }
                std::printf("\n");
            }
#endif
            Wipe_Sensitive();
            return (HTS_Holo_Tensor_4D_TX::SECURE_TRUE & (0u - valid)) |
                (HTS_Holo_Tensor_4D_TX::SECURE_FALSE & (0u - (valid ^ 1u)));
        }
    };

    HTS_Holo_Tensor_4D_TX::HTS_Holo_Tensor_4D_TX() noexcept
        : initialized_{ false }
    {
        static_assert(sizeof(Impl) <= IMPL_BUF_SIZE, "Holo4D_TX::Impl OOB");
        time_slot_ = 0u;
        master_seed_[0] = master_seed_[1] = master_seed_[2] = master_seed_[3] = 0u;
        std::memset(impl_buf_, 0, IMPL_BUF_SIZE);
    }

    HTS_Holo_Tensor_4D_TX::~HTS_Holo_Tensor_4D_TX() noexcept
    {
        if (!initialized_.load(std::memory_order_acquire)) { return; }
        while (op_busy_.test_and_set(std::memory_order_acq_rel)) {
            holo4d_tx_busy_spin_yield();
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

    uint32_t HTS_Holo_Tensor_4D_TX::Initialize(
        const uint32_t master_seed[4],
        const HoloTensor_Profile* profile) noexcept
    {
        if (master_seed == nullptr) { return SECURE_FALSE; }
        Holo4D_TX_Busy_Guard g(op_busy_);
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
        im->encode_count = 0u;
        for (uint32_t i = 0u; i < HOLO_MAX_BLOCK_BITS; ++i) {
            im->accum[static_cast<size_t>(i)] = 0;
        }
        time_slot_ = 0u;
        if (Holo4D_Cfi_Transition(im->state, im->cfi_violation_count, HoloState::READY) !=
            SECURE_TRUE) {
            return SECURE_FALSE;
        }
        return SECURE_TRUE;
    }

    void HTS_Holo_Tensor_4D_TX::Shutdown() noexcept
    {
        if (!initialized_.load(std::memory_order_acquire)) { return; }
        while (op_busy_.test_and_set(std::memory_order_acq_rel)) {
            holo4d_tx_busy_spin_yield();
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

    void HTS_Holo_Tensor_4D_TX::Rotate_Seed(
        const uint32_t new_seed[4]) noexcept
    {
        if (new_seed == nullptr) { return; }
        Holo4D_TX_Busy_Guard g(op_busy_);
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

    uint32_t HTS_Holo_Tensor_4D_TX::Set_Profile(
        const HoloTensor_Profile* profile) noexcept
    {
        if (profile == nullptr) { return SECURE_FALSE; }
        Holo4D_TX_Busy_Guard g(op_busy_);
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

    uint32_t HTS_Holo_Tensor_4D_TX::Encode_Block(
        const int8_t* data_bits, uint16_t K,
        int8_t* output_chips, uint16_t N) noexcept
    {
        if (data_bits == nullptr) { return SECURE_FALSE; }
        if (output_chips == nullptr) { return SECURE_FALSE; }
        Holo4D_TX_Busy_Guard g(op_busy_);
        if (!g.locked) { return SECURE_FALSE; }
        if (!initialized_.load(std::memory_order_acquire)) { return SECURE_FALSE; }
        Impl* im = reinterpret_cast<Impl*>(impl_buf_);
        if (Holo4D_Cfi_Transition(
            im->state, im->cfi_violation_count, HoloState::ENCODING) != SECURE_TRUE) {
            return SECURE_FALSE;
        }
        const uint32_t ok = im->Encode(*this, data_bits, K, output_chips, N);
        Holo4D_Cfi_Transition(im->state, im->cfi_violation_count, HoloState::READY);
        if (ok == SECURE_TRUE) {
            ++im->encode_count;
#if defined(HTS_CFO_V5A_S5H_ENCODE_DIAG)
            if (N >= 64) {
                std::printf(
                    "[ENC] chip[0]=%d chip[31]=%d chip[63]=%d\n",
                    static_cast<int>(output_chips[0]),
                    static_cast<int>(output_chips[31]),
                    static_cast<int>(output_chips[63]));
#if defined(HTS_CFO_V5A_S5H_STEPC_FULL)
                for (uint16_t ci = 0; ci < 64 && ci < N; ++ci) {
                    std::printf(
                        "[ENC-CHIP] k=%u v=%d\n",
                        static_cast<unsigned>(ci),
                        static_cast<int>(output_chips[ci]));
                }
#endif
            }
#endif
        }
        return ok;
    }

    uint32_t HTS_Holo_Tensor_4D_TX::Advance_Time_Slot() noexcept
    {
        Holo4D_TX_Busy_Guard g(op_busy_);
        if (!g.locked) { return SECURE_FALSE; }
        if (!initialized_.load(std::memory_order_acquire)) { return SECURE_FALSE; }
        ++time_slot_;
        return SECURE_TRUE;
    }

    uint32_t HTS_Holo_Tensor_4D_TX::Set_Time_Slot(uint32_t frame_no) noexcept
    {
        Holo4D_TX_Busy_Guard g(op_busy_);
        if (!g.locked) { return SECURE_FALSE; }
        if (!initialized_.load(std::memory_order_acquire)) { return SECURE_FALSE; }
        time_slot_ = frame_no;
        return SECURE_TRUE;
    }

    HoloState HTS_Holo_Tensor_4D_TX::Get_State() const noexcept
    {
        Holo4D_TX_Busy_Guard g(op_busy_);
        if (!g.locked) { return HoloState::OFFLINE; }
        if (!initialized_.load(std::memory_order_acquire)) { return HoloState::OFFLINE; }
        return reinterpret_cast<const Impl*>(impl_buf_)->state;
    }

    uint32_t HTS_Holo_Tensor_4D_TX::Get_Encode_Count() const noexcept
    {
        Holo4D_TX_Busy_Guard g(op_busy_);
        if (!g.locked) { return 0u; }
        if (!initialized_.load(std::memory_order_acquire)) { return 0u; }
        return reinterpret_cast<const Impl*>(impl_buf_)->encode_count;
    }

    uint32_t HTS_Holo_Tensor_4D_TX::Get_Time_Slot() const noexcept
    {
        Holo4D_TX_Busy_Guard g(op_busy_);
        if (!g.locked) { return 0u; }
        if (!initialized_.load(std::memory_order_acquire)) { return 0u; }
        return time_slot_;
    }

} // namespace ProtectedEngine
