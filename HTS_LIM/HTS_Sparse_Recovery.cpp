// =========================================================================
// HTS_Sparse_Recovery.cpp
// L1 스파스 하이브리드 복구 구현부
// Target: STM32F407 (Cortex-M4, 168MHz)
//
// [보안] ARM Release: Generate/Execute 진입 시 DHCSR·OPTCR(RDP) 폴링
// [Branchless] 앵커 정규화·역난독화 블렌드·min(span,remain)·중력 나눗셈·PC noise Q16
// =========================================================================
#include "HTS_Sparse_Recovery.h"
#include <climits>
#include <cstdint>
#include <type_traits>
#if defined(__arm__) || defined(__TARGET_ARCH_ARM) || defined(__TARGET_ARCH_THUMB) || defined(__ARM_ARCH)
#include "HTS_Anti_Debug.h"
#include "HTS_Hardware_Init.h"
#endif
namespace ProtectedEngine {
#if !defined(HTS_SPARSE_RECOVERY_SKIP_PHYS_TRUST)
#if defined(HTS_ALLOW_OPEN_DEBUG) || !defined(NDEBUG)
#define HTS_SPARSE_RECOVERY_SKIP_PHYS_TRUST 1
#else
#define HTS_SPARSE_RECOVERY_SKIP_PHYS_TRUST 0
#endif
#endif
#if HTS_SPARSE_RECOVERY_SKIP_PHYS_TRUST == 0 && \
    (defined(__arm__) || defined(__TARGET_ARCH_ARM) || defined(__TARGET_ARCH_THUMB) || defined(__ARM_ARCH))
    [[noreturn]] static void SparseRecovery_PhysicalTrust_Fault() noexcept {
        Hardware_Init_Manager::Terminal_Fault_Action();
    }
    static void SparseRecovery_AssertPhysicalTrustOrFault() noexcept {
        volatile const uint32_t* const dhcsr =
            reinterpret_cast<volatile const uint32_t*>(ADDR_DHCSR);
        const uint32_t d0 = *dhcsr;
#if defined(__GNUC__) || defined(__clang__)
        __asm__ __volatile__("dsb sy" ::: "memory");
#endif
        const uint32_t d1 = *dhcsr;
        if (d0 != d1) {
            SparseRecovery_PhysicalTrust_Fault();
        }
        if ((d0 & DHCSR_DEBUG_MASK) != 0u) {
            SparseRecovery_PhysicalTrust_Fault();
        }
        volatile const uint32_t* const optcr =
            reinterpret_cast<volatile const uint32_t*>(HTS_FLASH_OPTCR_ADDR);
        const uint32_t o0 = *optcr;
#if defined(__GNUC__) || defined(__clang__)
        __asm__ __volatile__("dsb sy" ::: "memory");
#endif
        const uint32_t o1 = *optcr;
        if (o0 != o1) {
            SparseRecovery_PhysicalTrust_Fault();
        }
        const uint32_t rdp = (o0 & HTS_RDP_OPTCR_MASK) >> 8u;
        if (rdp != HTS_RDP_EXPECTED_BYTE_VAL) {
            SparseRecovery_PhysicalTrust_Fault();
        }
    }
#else
    static void SparseRecovery_AssertPhysicalTrustOrFault() noexcept {}
#endif
    template <typename T>
    static T Safe_Deobfuscate(T obfuscated, T interference) noexcept;
    namespace SparseRecovery_detail {
        inline size_t sparse_block_end_exclusive(
            const size_t i,
            const size_t span,
            const size_t elements) noexcept
        {
            if (i >= elements) {
                return elements;
            }
            const size_t remain = elements - i;
            const size_t span_gt = static_cast<size_t>(span > remain);
            const size_t take = span - (span - remain) * span_gt;
            return i + take;
        }
        inline void recovery_stats_zero(RecoveryStats& s) noexcept {
            s.total_elements = 0u;
            s.destroyed_count = 0u;
            s.recovered_by_parity = 0u;
            s.recovered_by_gravity = 0u;
            s.unrecoverable = 0u;
            s.noise_ratio_q16 = 0u;
        }
        template <typename T>
        inline bool sparse_ptr_aligned(const T* p) noexcept {
            constexpr std::size_t al = alignof(T);
            static_assert((al & (al - 1u)) == 0u, "alignof(T) must be power of 2");
            const uintptr_t a = reinterpret_cast<uintptr_t>(p);
            const uintptr_t mask = static_cast<uintptr_t>(al - 1u);
            return (a & mask) == 0u;
        }
        inline uint32_t floor_pow2_u32(uint32_t x) noexcept {
            if (x <= 1u) {
                return x;
            }
            uint32_t p = 1u;
            while ((p << 1u) <= x) {
                p <<= 1u;
            }
            return p;
        }
        inline uint32_t normalize_anchor_interval(
            uint32_t ai,
            bool is_test_mode) noexcept
        {
            const uint32_t is_test = static_cast<uint32_t>(is_test_mode);
            const uint32_t not_test = 1u - is_test;
            ai *= static_cast<uint32_t>(ai != 1u);
            const uint32_t bad_prod = not_test
                & (static_cast<uint32_t>(ai == 0u)
                    | static_cast<uint32_t>(ai > SparseRecoveryLimits::ANCHOR_INTERVAL_CAP));
            const uint32_t ai_nz = static_cast<uint32_t>(ai != 0u);
            ai = ai * (1u - bad_prod)
                + (SparseRecoveryLimits::ANCHOR_INTERVAL_CAP * ai_nz) * bad_prod;
            const uint32_t test_fix = is_test & static_cast<uint32_t>(ai == 0u);
            ai = ai * (1u - test_fix)
                + SparseRecoveryLimits::TEST_MODE_DEFAULT_ANCHOR * test_fix;
            if (ai > 1u) {
                ai = floor_pow2_u32(ai);
            }
            return ai;
        }
        struct GravRecipQ16Lut {
            uint16_t v[257];
            constexpr GravRecipQ16Lut() noexcept : v{} {
                for (uint32_t s = 2u; s <= 256u; ++s) {
                    v[s] = static_cast<uint16_t>((65536u + (s >> 1u)) / s);
                }
            }
        };
        static constexpr GravRecipQ16Lut k_grav_recip_q16{};
        template <typename T>
        inline void deobfuscate_cell_blend(
            T& cell,
            const T& interference,
            const T& marker,
            size_t& destroyed) noexcept
        {
            const uint32_t ok = static_cast<uint32_t>(cell != marker);
            const uint64_t cu = static_cast<uint64_t>(cell);
            const uint64_t du = static_cast<uint64_t>(
                Safe_Deobfuscate(cell, interference));
            cell = static_cast<T>(
                du * static_cast<uint64_t>(ok)
                + cu * static_cast<uint64_t>(1u - ok));
            destroyed += static_cast<size_t>(1u - ok);
        }
        template <typename T>
        inline void deobfuscate_anchor_blend(
            T& cell,
            const T& anchor_mask,
            const T& marker,
            size_t& destroyed) noexcept
        {
            const uint32_t ok = static_cast<uint32_t>(cell != marker);
            const uint64_t cu = static_cast<uint64_t>(cell);
            const uint64_t du = static_cast<uint64_t>(
                Safe_Deobfuscate(cell, anchor_mask));
            cell = static_cast<T>(
                du * static_cast<uint64_t>(ok)
                + cu * static_cast<uint64_t>(1u - ok));
            destroyed += static_cast<size_t>(1u - ok);
        }
    } // namespace SparseRecovery_detail
    template <typename T>
    static constexpr T modular_inverse_3() noexcept {
        static_assert(std::is_unsigned<T>::value,
            "modular_inverse_3: unsigned 타입만 허용");
        constexpr T inv = static_cast<T>(
            static_cast<T>(static_cast<T>(~static_cast<T>(0)) / static_cast<T>(3))
            * static_cast<T>(2) + static_cast<T>(1));
        static_assert(static_cast<T>(static_cast<T>(3) * inv) == static_cast<T>(1),
            "Modular inverse verification failed: 3 * INV3 != 1");
        return inv;
    }
    template <typename T>
    constexpr T Get_Erasure_Marker() {
        return static_cast<T>(~static_cast<T>(0));
    }
    template <typename T>
    static T Safe_Obfuscate(T plaintext, T interference) noexcept {
        static_assert(std::is_unsigned<T>::value,
            "Safe_Obfuscate: unsigned 타입만 허용");
        const T MARKER = Get_Erasure_Marker<T>();
        const uint32_t m0 = static_cast<uint32_t>(plaintext == MARKER);
        plaintext ^= static_cast<T>(m0);
        static constexpr T INV3 = modular_inverse_3<T>();
        const T neg_inv3 = static_cast<T>(~INV3 + static_cast<T>(1u));
        const T bad_plaintext = static_cast<T>(neg_inv3 ^ interference);
        const uint32_t m1 = static_cast<uint32_t>(plaintext == bad_plaintext);
        plaintext ^= static_cast<T>(m1);
        const uint32_t m2 = m1 & static_cast<uint32_t>(plaintext == MARKER);
        plaintext ^= static_cast<T>(m2 * 2u);
        return static_cast<T>(
            static_cast<T>(plaintext ^ interference) * static_cast<T>(3u));
    }
    template <typename T>
    static T Safe_Deobfuscate(T obfuscated, T interference) noexcept {
        static_assert(std::is_unsigned<T>::value,
            "Safe_Deobfuscate: unsigned 타입만 허용");
        static constexpr T INV3 = modular_inverse_3<T>();
        return static_cast<T>(
            static_cast<T>(obfuscated * INV3) ^ interference);
    }
    template <typename T>
    static T gravity_blend_q16_unsigned(
        T mL,
        T mR,
        uint32_t dL,
        uint32_t sd,
        const uint16_t* lut) noexcept
    {
        const uint64_t uL = static_cast<uint64_t>(mL);
        const uint64_t uR = static_cast<uint64_t>(mR);
        const uint32_t is_desc = static_cast<uint32_t>(uR < uL);
        const uint64_t abs_d = is_desc != 0u ? (uL - uR) : (uR - uL);
        const uint64_t dLu = static_cast<uint64_t>(dL);
        uint64_t safe_abs = abs_d;
        const uint64_t denom_guard = dLu > 0u ? dLu : 1u;
        const uint64_t max_abs = UINT64_MAX / denom_guard;
        if (safe_abs > max_abs) {
            safe_abs = max_abs;
        }
        const uint64_t prod = safe_abs * dLu;
        const uint64_t recip = static_cast<uint64_t>(lut[sd]);
        const uint64_t delta = (prod * recip) >> 16;
        const uint64_t out_u = is_desc != 0u ? (uL - delta) : (uL + delta);
        return static_cast<T>(out_u);
    }
    static uint32_t compute_noise_ratio_q16(
        size_t destroyed, size_t total) noexcept
    {
#if defined(__arm__) || defined(__TARGET_ARCH_ARM) || \
    defined(__TARGET_ARCH_THUMB) || defined(__ARM_ARCH) || \
    defined(__aarch64__)
        (void)destroyed;
        (void)total;
        return 0u;
#else
        const uint32_t nz = static_cast<uint32_t>(total != 0u);
        const uint64_t denom = static_cast<uint64_t>(total)
            + static_cast<uint64_t>(1u - nz);
        return static_cast<uint32_t>(
            (static_cast<uint64_t>(destroyed) << 16u) / denom
                * static_cast<uint64_t>(nz));
#endif
    }
    template <typename T>
    static T Make_Anchor_Mask(uint32_t master_seed) {
        const uint32_t lo = master_seed ^ 0x3D414E43u;
        const uint32_t hi = lo * 0x9E3779B9u;
        const uint64_t wide = (static_cast<uint64_t>(hi) << 32)
            | static_cast<uint64_t>(lo);
        const uint64_t narrow = static_cast<uint64_t>(lo);
        const uint32_t use_wide = static_cast<uint32_t>(sizeof(T) > 4u);
        const uint64_t pick =
            (wide & (static_cast<uint64_t>(0u) - static_cast<uint64_t>(use_wide)))
            | (narrow
                & (static_cast<uint64_t>(0u)
                    - static_cast<uint64_t>(1u - use_wide)));
        return static_cast<T>(pick);
    }
    template <typename T>
    static T Make_Interference(uint32_t master_seed, uint32_t idx) {
        const uint32_t zeta = (master_seed ^ idx) * 0x9E3779B9u;
        const uint32_t rot = (zeta >> 5) | (zeta << 27);
        const uint32_t hi = zeta * 0x85EBCA6Bu;
        const uint64_t wide =
            (static_cast<uint64_t>(hi) << 32) | static_cast<uint64_t>(rot);
        const uint64_t narrow = static_cast<uint64_t>(rot);
        const uint32_t use_wide = static_cast<uint32_t>(sizeof(T) > 4u);
        const uint64_t pick =
            (wide & (static_cast<uint64_t>(0u) - static_cast<uint64_t>(use_wide)))
            | (narrow
                & (static_cast<uint64_t>(0u)
                    - static_cast<uint64_t>(1u - use_wide)));
        return static_cast<T>(pick);
    }
    template <typename T>
    void Sparse_Recovery_Engine::Generate_Interference_Pattern(
        T* tensor_block,
        size_t elements,
        uint64_t session_id,
        uint32_t anchor_interval,
        bool is_test_mode)
    {
        SparseRecovery_AssertPhysicalTrustOrFault();
        if ((tensor_block == nullptr) || (elements == 0u)) {
            return;
        }
        if (!SparseRecovery_detail::sparse_ptr_aligned(tensor_block)) {
            return;
        }
        anchor_interval = SparseRecovery_detail::normalize_anchor_interval(
            anchor_interval, is_test_mode);
        const uint32_t master_seed = static_cast<uint32_t>(session_id ^ 0x3D485453);
        const T ANCHOR_MASK = Make_Anchor_Mask<T>(master_seed);
        if (anchor_interval > 0) {
            for (size_t i = 0; i < elements; i += anchor_interval) {
                T parity = 0;
                const size_t end_idx = SparseRecovery_detail::sparse_block_end_exclusive(
                    i, static_cast<size_t>(anchor_interval), elements);
                for (size_t j = i + 1; j < end_idx; ++j) {
                    parity ^= tensor_block[j];
                    const T interference =
                        Make_Interference<T>(master_seed, static_cast<uint32_t>(j));
                    tensor_block[j] = Safe_Obfuscate(tensor_block[j], interference);
                }
                tensor_block[i] = Safe_Obfuscate(parity, ANCHOR_MASK);
            }
        }
        else {
            for (size_t i = 0; i < elements; ++i) {
                const T interference =
                    Make_Interference<T>(master_seed, static_cast<uint32_t>(i));
                tensor_block[i] = Safe_Obfuscate(tensor_block[i], interference);
            }
        }
    }
    template <typename T>
    bool Sparse_Recovery_Engine::Execute_L1_Reconstruction(
        T* damaged_tensor,
        size_t elements,
        uint64_t session_id,
        uint32_t anchor_interval,
        bool is_test_mode,
        bool strict_mode,
        RecoveryStats& out_stats)
    {
        SparseRecovery_AssertPhysicalTrustOrFault();
        SparseRecovery_detail::recovery_stats_zero(out_stats);
        if ((damaged_tensor == nullptr) || (elements == 0u)) {
            return false;
        }
        if (!SparseRecovery_detail::sparse_ptr_aligned(damaged_tensor)) {
            return false;
        }
        out_stats.total_elements = elements;
        anchor_interval = SparseRecovery_detail::normalize_anchor_interval(
            anchor_interval, is_test_mode);
        const T ERASURE_MARKER = Get_Erasure_Marker<T>();
        const uint32_t master_seed = static_cast<uint32_t>(session_id ^ 0x3D485453);
        const T ANCHOR_MASK = Make_Anchor_Mask<T>(master_seed);
        size_t total_destroyed = 0;
        if (anchor_interval > 0) {
            for (size_t i = 0; i < elements; i += anchor_interval) {
                SparseRecovery_detail::deobfuscate_anchor_blend(
                    damaged_tensor[i], ANCHOR_MASK, ERASURE_MARKER, total_destroyed);
                const size_t end_idx = SparseRecovery_detail::sparse_block_end_exclusive(
                    i, static_cast<size_t>(anchor_interval), elements);
                for (size_t j = i + 1; j < end_idx; ++j) {
                    const T interference =
                        Make_Interference<T>(master_seed, static_cast<uint32_t>(j));
                    SparseRecovery_detail::deobfuscate_cell_blend(
                        damaged_tensor[j], interference, ERASURE_MARKER, total_destroyed);
                }
            }
        }
        else {
            for (size_t i = 0; i < elements; ++i) {
                const T interference =
                    Make_Interference<T>(master_seed, static_cast<uint32_t>(i));
                SparseRecovery_detail::deobfuscate_cell_blend(
                    damaged_tensor[i], interference, ERASURE_MARKER, total_destroyed);
            }
            out_stats.destroyed_count = total_destroyed;
            out_stats.noise_ratio_q16 =
                compute_noise_ratio_q16(total_destroyed, elements);
            return true;
        }
        out_stats.destroyed_count = total_destroyed;
        if (total_destroyed == 0) {
            out_stats.noise_ratio_q16 = 0u;
            return true;
        }
        bool is_reconstruction_successful = true;
        constexpr uint32_t k_max_erasures_per_block = 64u;
        const uint32_t aim = anchor_interval - 1u;
        const uint16_t* const grav_lut = SparseRecovery_detail::k_grav_recip_q16.v;
        for (size_t block_start = 0; block_start < elements;
            block_start += anchor_interval)
        {
            const size_t block_end = SparseRecovery_detail::sparse_block_end_exclusive(
                block_start, static_cast<size_t>(anchor_interval), elements);
            size_t local_destroyed_count = 0;
            size_t last_destroyed_idx = block_start;
            T block_xor_sum = 0;
            size_t erase_pos[k_max_erasures_per_block];
            uint32_t erase_count = 0u;
            for (size_t j = block_start; j < block_end; ++j) {
                const uint32_t is_m =
                    static_cast<uint32_t>(damaged_tensor[j] == ERASURE_MARKER);
                local_destroyed_count += static_cast<size_t>(is_m);
                last_destroyed_idx =
                    j * static_cast<size_t>(is_m)
                    + last_destroyed_idx * static_cast<size_t>(1u - is_m);
                block_xor_sum ^= static_cast<T>(
                    static_cast<typename std::make_unsigned<T>::type>(damaged_tensor[j])
                    * (static_cast<typename std::make_unsigned<T>::type>(1u)
                        - static_cast<typename std::make_unsigned<T>::type>(is_m)));
                if (is_m != 0u && erase_count < k_max_erasures_per_block) {
                    erase_pos[erase_count++] = j;
                }
            }
            if (local_destroyed_count == 0) {
                continue;
            }
            if (local_destroyed_count == 1) {
                damaged_tensor[last_destroyed_idx] = block_xor_sum;
                out_stats.recovered_by_parity++;
            }
            else {
                if (strict_mode) {
                    out_stats.unrecoverable += local_destroyed_count;
                    for (size_t j = block_start; j < block_end; ++j) {
                        const uint32_t is_m =
                            static_cast<uint32_t>(damaged_tensor[j] == ERASURE_MARKER);
                        damaged_tensor[j] = static_cast<T>(
                            static_cast<typename std::make_unsigned<T>::type>(0)
                            * static_cast<typename std::make_unsigned<T>::type>(is_m)
                            + static_cast<typename std::make_unsigned<T>::type>(
                                damaged_tensor[j])
                            * (static_cast<typename std::make_unsigned<T>::type>(1u)
                                - static_cast<typename std::make_unsigned<T>::type>(
                                    is_m)));
                    }
                    is_reconstruction_successful = false;
                    continue;
                }
                for (uint32_t e = 0u; e < erase_count; ++e) {
                    const size_t j = erase_pos[e];
                    if (j == block_start) {
                        damaged_tensor[j] = 0;
                        continue;
                    }
                    out_stats.recovered_by_gravity++;
                    size_t L_idx = block_start;
                    uint32_t found_L = 0u;
                    for (uint32_t delta = 1u; delta < anchor_interval; ++delta) {
                        const size_t idx = j - static_cast<size_t>(delta);
                        const uint32_t in_rng =
                            static_cast<uint32_t>(idx > block_start);
                        const uint32_t not_anc =
                            static_cast<uint32_t>(idx != block_start);
                        const uint32_t not_mar = static_cast<uint32_t>(
                            damaged_tensor[idx] != ERASURE_MARKER);
                        const uint32_t hit = in_rng & not_anc & not_mar;
                        const uint32_t take = hit & (1u - found_L);
                        found_L |= take;
                        L_idx = L_idx * static_cast<size_t>(1u - take)
                            + idx * static_cast<size_t>(take);
                    }
                    size_t R_idx = j;
                    uint32_t found_R = 0u;
                    for (uint32_t delta = 1u; delta < anchor_interval; ++delta) {
                        const size_t idx = j + static_cast<size_t>(delta);
                        const uint32_t in_b =
                            static_cast<uint32_t>(idx < block_end);
                        const uint32_t not_m = static_cast<uint32_t>(
                            damaged_tensor[idx] != ERASURE_MARKER);
                        const uint32_t pay =
                            static_cast<uint32_t>((idx & aim) != 0u);
                        const uint32_t hit = in_b & not_m & pay;
                        const uint32_t take = hit & (1u - found_R);
                        found_R |= take;
                        R_idx = R_idx * static_cast<size_t>(1u - take)
                            + idx * static_cast<size_t>(take);
                    }
                    for (uint32_t delta = 0u; delta < anchor_interval; ++delta) {
                        const size_t idx = block_end + static_cast<size_t>(delta);
                        const uint32_t in_e =
                            static_cast<uint32_t>(idx < elements);
                        const uint32_t not_m = static_cast<uint32_t>(
                            damaged_tensor[idx] != ERASURE_MARKER);
                        const uint32_t pay =
                            static_cast<uint32_t>((idx & aim) != 0u);
                        const uint32_t allow = 1u - found_R;
                        const uint32_t hit = in_e & not_m & pay & allow;
                        const uint32_t take = hit & (1u - found_R);
                        found_R |= take;
                        R_idx = R_idx * static_cast<size_t>(1u - take)
                            + idx * static_cast<size_t>(take);
                    }
                    const uint32_t both = found_L & found_R;
                    const uint32_t only_l = found_L & (1u - found_R);
                    const uint32_t only_r = found_R & (1u - found_L);
                    const uint32_t none = (1u - found_L) & (1u - found_R);
                    if (both != 0u) {
                        const uint32_t dL =
                            static_cast<uint32_t>(j - L_idx);
                        const uint32_t dR =
                            static_cast<uint32_t>(R_idx - j);
                        const uint32_t sd = dL + dR;
                        damaged_tensor[j] = gravity_blend_q16_unsigned(
                            damaged_tensor[L_idx],
                            damaged_tensor[R_idx],
                            dL,
                            sd,
                            grav_lut);
                    }
                    if (only_l != 0u) {
                        damaged_tensor[j] = damaged_tensor[L_idx];
                    }
                    if (only_r != 0u) {
                        damaged_tensor[j] = damaged_tensor[R_idx];
                    }
                    if (none != 0u) {
                        damaged_tensor[j] = 0;
                        out_stats.unrecoverable++;
                    }
                }
            }
        }
        out_stats.noise_ratio_q16 =
            compute_noise_ratio_q16(out_stats.destroyed_count, elements);
        return is_reconstruction_successful;
    }
    template void Sparse_Recovery_Engine::Generate_Interference_Pattern<uint8_t>(
        uint8_t*, size_t, uint64_t, uint32_t, bool);
    template void Sparse_Recovery_Engine::Generate_Interference_Pattern<uint16_t>(
        uint16_t*, size_t, uint64_t, uint32_t, bool);
    template void Sparse_Recovery_Engine::Generate_Interference_Pattern<uint32_t>(
        uint32_t*, size_t, uint64_t, uint32_t, bool);
    template void Sparse_Recovery_Engine::Generate_Interference_Pattern<uint64_t>(
        uint64_t*, size_t, uint64_t, uint32_t, bool);
    template bool Sparse_Recovery_Engine::Execute_L1_Reconstruction<uint8_t>(
        uint8_t*, size_t, uint64_t, uint32_t, bool, bool, RecoveryStats&);
    template bool Sparse_Recovery_Engine::Execute_L1_Reconstruction<uint16_t>(
        uint16_t*, size_t, uint64_t, uint32_t, bool, bool, RecoveryStats&);
    template bool Sparse_Recovery_Engine::Execute_L1_Reconstruction<uint32_t>(
        uint32_t*, size_t, uint64_t, uint32_t, bool, bool, RecoveryStats&);
    template bool Sparse_Recovery_Engine::Execute_L1_Reconstruction<uint64_t>(
        uint64_t*, size_t, uint64_t, uint32_t, bool, bool, RecoveryStats&);
} // namespace ProtectedEngine
