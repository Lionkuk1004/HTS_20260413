/// @file  HTS_Holo_Tensor_4D_RX.cpp
#include "HTS_Holo_Tensor_4D_RX.h"
#include "HTS_Holo_Tensor_4D_Common.h"
#include "HTS_Arm_Irq_Mask_Guard.h"
#include "HTS_Secure_Memory.h"
#include <new>
#include <atomic>
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
        struct Holo4D_Busy_Guard final {
            std::atomic_flag* flag;
            bool locked;
            explicit Holo4D_Busy_Guard(std::atomic_flag& f) noexcept
                : flag(&f), locked(false) {
                locked = !flag->test_and_set(std::memory_order_acq_rel);
            }
            ~Holo4D_Busy_Guard() noexcept {
                if (locked) { flag->clear(std::memory_order_release); }
            }
            Holo4D_Busy_Guard(const Holo4D_Busy_Guard&) = delete;
            Holo4D_Busy_Guard& operator=(const Holo4D_Busy_Guard&) = delete;
        };

        static inline void holo4d_busy_spin_yield() noexcept {
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

            Holo4D_Generate_Partitioned_Params(
                o.master_seed_, o.time_slot_,
                scratch_rows, scratch_perm, K_eff, N_eff, L);

            const uint64_t mask_bits = Holo4D_Generate_Phase_Mask(
                o.master_seed_, o.time_slot_);

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
                    int32_t acc = 0;
                    uint16_t ii = 0u;
                    for (; ii + 3u < N_eff; ii += 4u) {
                        const uint32_t i0 = static_cast<uint32_t>(ii);
                        const uint32_t i1 = static_cast<uint32_t>(ii + 1u);
                        const uint32_t i2 = static_cast<uint32_t>(ii + 2u);
                        const uint32_t i3 = static_cast<uint32_t>(ii + 3u);
                        const int32_t w0 =
                            static_cast<int32_t>(Holo4D_Walsh_Code(row_k, i0));
                        const int32_t w1 =
                            static_cast<int32_t>(Holo4D_Walsh_Code(row_k, i1));
                        const int32_t w2 =
                            static_cast<int32_t>(Holo4D_Walsh_Code(row_k, i2));
                        const int32_t w3 =
                            static_cast<int32_t>(Holo4D_Walsh_Code(row_k, i3));
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
                        const int8_t w = Holo4D_Walsh_Code(
                            row_k, static_cast<uint32_t>(ii));
                        acc += static_cast<int32_t>(scratch_rx[static_cast<size_t>(ii)]) *
                            static_cast<int32_t>(w);
                    }
                    sumL += acc;
                }
                accum[static_cast<size_t>(k)] += sumL;
            }

            for (uint16_t k = 0u; k < K_eff; ++k) {
                const uint32_t sign_bit =
                    static_cast<uint32_t>(accum[static_cast<size_t>(k)]) >> 31u;
                output_bits[static_cast<size_t>(k)] = static_cast<int8_t>(
                    1 - 2 * static_cast<int32_t>(sign_bit));
            }
            Wipe_Sensitive();
            return (HTS_Holo_Tensor_4D_RX::SECURE_TRUE & (0u - valid)) |
                (HTS_Holo_Tensor_4D_RX::SECURE_FALSE & (0u - (valid ^ 1u)));
        }
    };

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
            holo4d_busy_spin_yield();
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
        Holo4D_Busy_Guard g(op_busy_);
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
        if (Holo4D_Cfi_Transition(im->state, im->cfi_violation_count, HoloState::READY) !=
            SECURE_TRUE) {
            return SECURE_FALSE;
        }
        return SECURE_TRUE;
    }

    void HTS_Holo_Tensor_4D_RX::Shutdown() noexcept
    {
        if (!initialized_.load(std::memory_order_acquire)) { return; }
        while (op_busy_.test_and_set(std::memory_order_acq_rel)) {
            holo4d_busy_spin_yield();
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
        Holo4D_Busy_Guard g(op_busy_);
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
        Holo4D_Busy_Guard g(op_busy_);
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
        Holo4D_Busy_Guard g(op_busy_);
        if (!g.locked) { return SECURE_FALSE; }
        if (!initialized_.load(std::memory_order_acquire)) { return SECURE_FALSE; }
        Impl* im = reinterpret_cast<Impl*>(impl_buf_);
        if (Holo4D_Cfi_Transition(
            im->state, im->cfi_violation_count, HoloState::DECODING) != SECURE_TRUE) {
            return SECURE_FALSE;
        }
        const uint32_t ok = im->Decode(
            *this, rx_chips, N, valid_mask, output_bits, K);
        Holo4D_Cfi_Transition(im->state, im->cfi_violation_count, HoloState::READY);
        if (ok == SECURE_TRUE) { ++im->decode_count; }
        return ok;
    }

    uint32_t HTS_Holo_Tensor_4D_RX::Advance_Time_Slot() noexcept
    {
        Holo4D_Busy_Guard g(op_busy_);
        if (!g.locked) { return SECURE_FALSE; }
        if (!initialized_.load(std::memory_order_acquire)) { return SECURE_FALSE; }
        ++time_slot_;
        return SECURE_TRUE;
    }

    uint32_t HTS_Holo_Tensor_4D_RX::Set_Time_Slot(uint32_t frame_no) noexcept
    {
        Holo4D_Busy_Guard g(op_busy_);
        if (!g.locked) { return SECURE_FALSE; }
        if (!initialized_.load(std::memory_order_acquire)) { return SECURE_FALSE; }
        time_slot_ = frame_no;
        return SECURE_TRUE;
    }

    HoloState HTS_Holo_Tensor_4D_RX::Get_State() const noexcept
    {
        Holo4D_Busy_Guard g(op_busy_);
        if (!g.locked) { return HoloState::OFFLINE; }
        if (!initialized_.load(std::memory_order_acquire)) { return HoloState::OFFLINE; }
        return reinterpret_cast<const Impl*>(impl_buf_)->state;
    }

    uint32_t HTS_Holo_Tensor_4D_RX::Get_Decode_Count() const noexcept
    {
        Holo4D_Busy_Guard g(op_busy_);
        if (!g.locked) { return 0u; }
        if (!initialized_.load(std::memory_order_acquire)) { return 0u; }
        return reinterpret_cast<const Impl*>(impl_buf_)->decode_count;
    }

    uint32_t HTS_Holo_Tensor_4D_RX::Get_Time_Slot() const noexcept
    {
        Holo4D_Busy_Guard g(op_busy_);
        if (!g.locked) { return 0u; }
        if (!initialized_.load(std::memory_order_acquire)) { return 0u; }
        return time_slot_;
    }

} // namespace ProtectedEngine
