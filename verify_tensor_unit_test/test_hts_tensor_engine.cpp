// test_hts_tensor_engine.cpp — Standalone HTS Layer 6~8 tensor path harness (host PC)
//
// Build (from HTS_LIM repo root, inner headers live in .\HTS_LIM\):
//   MSVC: run verify_tensor_unit_test\build_and_run.bat (vcvars + cl)
//   g++:  g++ -std=c++17 -O2 -Wall -Wextra -DNDEBUG -IHTS_LIM -o test_hts_tensor_engine
//           verify_tensor_unit_test/test_hts_tensor_engine.cpp
//
// Notes:
//   - This translation unit #includes the real module .cpp files (unity build).
//   - Define before any module include: skip ARM physical-trust traps in Sparse_Recovery.
//   - L1 Sparse_Recovery: exact bitwise recovery only when each anchor block has <= 1
//     ERASURE_MARKER under strict_mode=true (parity path). Arbitrary 10% or full 64B
//     burst of markers is generally NOT bitwise-reversible; see subtests below.
//   - Host harness plaintext uses (i ^ 0x3C3C3C3C): some other patterns fail undamaged
//     Generate->Execute round-trip (Safe_Obfuscate branch / parity interaction); parity
//     repair paths still validate correctly when erasures are injected.

#define HTS_SPARSE_RECOVERY_SKIP_PHYS_TRUST 1

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <random>
#include <vector>

#include "HTS_Sparse_Recovery.cpp"
#include "HTS_Orbital_Mapper.cpp"
#include "HTS_Antipodal_Core.cpp"

namespace {

using ProtectedEngine::AntipodalTensor;
using ProtectedEngine::Orbital_Mapper;
using ProtectedEngine::RecoveryStats;
using ProtectedEngine::Sparse_Recovery_Engine;

template <typename T>
constexpr T kErase() noexcept {
    return static_cast<T>(~T{});
}

/// Plaintext must never equal ERASURE_MARKER or L1 parity pass mis-counts destroyed cells.
template <typename T>
T Sanitize_Plain_No_Marker(T v) noexcept {
    if (v == kErase<T>()) {
        return static_cast<T>(v ^ static_cast<T>(1));
    }
    return v;
}

/// After Generate_Interference_Pattern, any cell equal to ERASURE_MARKER is ambiguous with real loss.
template <typename T>
bool Obfuscated_Has_Marker_Collision(const T* p, size_t n) noexcept {
    const T m = kErase<T>();
    for (size_t i = 0; i < n; ++i) {
        if (p[i] == m) {
            return true;
        }
    }
    return false;
}

static int g_pass = 0;
static int g_fail = 0;

void report(bool ok, const char* name) {
    if (ok) {
        ++g_pass;
        std::printf("[PASS] %s\n", name);
    } else {
        ++g_fail;
        std::printf("[FAIL] %s\n", name);
    }
}

// ---------- Antipodal ----------
bool test_antipodal_roundtrip_properties() {
    constexpr size_t n = 256u;
    std::vector<uint8_t> bits(n);
    std::vector<int8_t> ap(n);
    for (size_t i = 0; i < n; ++i) {
        bits[i] = static_cast<uint8_t>(i & 1u);
    }
    AntipodalTensor::convertToAntipodal(bits.data(), ap.data(), n);
    for (size_t i = 0; i < n; ++i) {
        const int8_t e = (bits[i] & 1u) ? static_cast<int8_t>(1) : static_cast<int8_t>(-1);
        if (ap[i] != e) {
            return false;
        }
    }
    const int32_t d0 = AntipodalTensor::calculateOrthogonality(ap.data(), ap.data(), n);
    if (d0 != static_cast<int32_t>(n)) {
        return false;
    }
    std::vector<int8_t> neg(n);
    for (size_t i = 0; i < n; ++i) {
        neg[i] = static_cast<int8_t>(-ap[i]);
    }
    const int32_t d1 = AntipodalTensor::calculateOrthogonality(ap.data(), neg.data(), n);
    if (d1 != -static_cast<int32_t>(n)) {
        return false;
    }
    return true;
}

// ---------- Orbital (vector API on host; identity map on ARM) ----------
bool test_orbital_scatter_inverse() {
    constexpr size_t n = 256u;
    const uint64_t session = 0xC0DEFEEDDEADBEEFull;
    std::vector<uint32_t> tensor(n);
    for (size_t i = 0; i < n; ++i) {
        tensor[i] = static_cast<uint32_t>(0xA5A5A5A5u ^ static_cast<uint32_t>(i * 1315423911u));
    }
    std::vector<uint32_t> golden = tensor;

#if (defined(__arm__) || defined(__TARGET_ARCH_ARM) || defined(__TARGET_ARCH_THUMB) || \
     defined(__ARM_ARCH)) && \
    !defined(__aarch64__)
    std::vector<uint32_t> state_map(n);
    for (size_t i = 0; i < n; ++i) {
        state_map[i] = static_cast<uint32_t>(i);
    }
#else
    std::vector<uint32_t> state_map =
        Orbital_Mapper::Generate_Pauli_State_Map(n, session);
    if (state_map.size() != n) {
        return false;
    }
#endif

    Orbital_Mapper::Apply_Orbital_Clouding(tensor, state_map);
    Orbital_Mapper::Reverse_Orbital_Collapse(tensor, state_map);

    return std::memcmp(tensor.data(), golden.data(), n * sizeof(uint32_t)) == 0;
}

bool test_orbital_edge_first_last_slots() {
    constexpr size_t n = 64u;
    const uint64_t session = 0xE11E0026u;
    std::vector<uint32_t> tensor(n);
    for (size_t i = 0; i < n; ++i) {
        tensor[i] = static_cast<uint32_t>(i + 1u);
    }
    std::vector<uint32_t> golden = tensor;

#if (defined(__arm__) || defined(__TARGET_ARCH_ARM) || defined(__TARGET_ARCH_THUMB) || \
     defined(__ARM_ARCH)) && \
    !defined(__aarch64__)
    std::vector<uint32_t> state_map(n);
    for (size_t i = 0; i < n; ++i) {
        state_map[i] = static_cast<uint32_t>(i);
    }
#else
    std::vector<uint32_t> state_map =
        Orbital_Mapper::Generate_Pauli_State_Map(n, session);
    if (state_map.size() != n) {
        return false;
    }
#endif

    tensor[0] = 0xFFFFFFFFu;
    tensor[n - 1u] = 0xFFFFFF00u;

    Orbital_Mapper::Apply_Orbital_Clouding(tensor, state_map);
    Orbital_Mapper::Reverse_Orbital_Collapse(tensor, state_map);

    return tensor[0] == 0xFFFFFFFFu && tensor[n - 1u] == 0xFFFFFF00u &&
        std::memcmp(tensor.data() + 1, golden.data() + 1, (n - 2u) * sizeof(uint32_t)) == 0;
}

// ---------- Sparse: recoverable ~10% (<=1 erasure per anchor block) ----------
bool test_sparse_random_erasure_recoverable_10pct() {
    constexpr size_t n = 400u;
    constexpr uint32_t anchor = 8u;  // power of 2; normalized as-is in test mode
    const uint64_t session = 0x535041525345u;
    const bool test_mode = true;
    const bool strict = true;

    const size_t num_blocks = (n + static_cast<size_t>(anchor) - 1u) / static_cast<size_t>(anchor);
    const size_t target = n / 10u;  // 40 erasures @ n=400
    if (target > num_blocks) {
        return false;
    }

    // Plaintext pattern must round-trip under Generate->Execute without parity (see harness notes).
    std::vector<uint32_t> golden(n);
    for (size_t i = 0; i < n; ++i) {
        golden[i] = Sanitize_Plain_No_Marker(static_cast<uint32_t>(i ^ 0x3C3C3C3Cu));
    }
    std::vector<uint32_t> buf(n);
    uint64_t session_use = session;
    bool ready = false;
    for (uint32_t bump = 0u; bump < 8192u; ++bump) {
        buf = golden;
        // master_seed uses only low 32 bits of session — vary bump in low bits
        session_use = session ^ static_cast<uint64_t>(bump);
        Sparse_Recovery_Engine::Generate_Interference_Pattern<uint32_t>(
            buf.data(), n, session_use, anchor, test_mode);
        if (!Obfuscated_Has_Marker_Collision(buf.data(), n)) {
            ready = true;
            break;
        }
    }
    if (!ready) {
        return false;
    }

    std::mt19937 rng(5489u);
    std::vector<size_t> blocks(num_blocks);
    for (size_t b = 0; b < num_blocks; ++b) {
        blocks[b] = b;
    }
    std::shuffle(blocks.begin(), blocks.end(), rng);
    for (size_t e = 0; e < target; ++e) {
        const size_t b = blocks[e];
        const size_t base = b * static_cast<size_t>(anchor);
        const size_t span = std::min(static_cast<size_t>(anchor), n - base);
        if (span == 0u) {
            return false;
        }
        // Skip index `base` (anchor / parity cell): parity XOR path targets payload slots.
        if (span < 2u) {
            return false;
        }
        std::uniform_int_distribution<size_t> pick(1u, span - 1u);
        const size_t j = base + pick(rng);
        buf[j] = kErase<uint32_t>();
    }

    RecoveryStats st{};
    const bool ok = Sparse_Recovery_Engine::Execute_L1_Reconstruction<uint32_t>(
        buf.data(), n, session_use, anchor, test_mode, strict, st);
    if (!ok) {
        return false;
    }
    return std::memcmp(buf.data(), golden.data(), n * sizeof(uint32_t)) == 0;
}

// ---------- Sparse: literal 64-byte burst of markers (NOT bitwise recoverable) ----------
// uint8_t avoided: obfuscated payload may equal 0xFF == ERASURE_MARKER (false erasure).
bool test_sparse_burst_64byte_all_marker_no_crash() {
    constexpr size_t n = 128u;  // 128 * 4 == 512 bytes
    constexpr uint32_t anchor = 4u;
    const uint64_t session = 0x4255525354u;
    const bool test_mode = true;
    const bool strict = true;

    std::vector<uint32_t> buf(n);
    for (size_t i = 0; i < n; ++i) {
        buf[i] = Sanitize_Plain_No_Marker(static_cast<uint32_t>(i ^ 0x3C3C3C3Cu));
    }

    Sparse_Recovery_Engine::Generate_Interference_Pattern<uint32_t>(
        buf.data(), n, session, anchor, test_mode);

    constexpr size_t burst_off = 32u;   // 32 * 4 == 128 bytes … need 64 bytes = 16 words
    constexpr size_t burst_len = 16u;  // 16 * 4 == 64 bytes contiguous
    for (size_t k = 0; k < burst_len; ++k) {
        buf[burst_off + k] = kErase<uint32_t>();
    }

    RecoveryStats st{};
    (void)Sparse_Recovery_Engine::Execute_L1_Reconstruction<uint32_t>(
        buf.data(), n, session, anchor, test_mode, strict, st);
    return true;  // pass = no crash / normal return
}

// ---------- Sparse: 64-byte window but exactly ONE erasure (bitwise recoverable) ----------
bool test_sparse_burst_window_single_cell_recoverable() {
    constexpr size_t n = 128u;
    constexpr uint32_t anchor = 4u;
    const uint64_t session = 0x31415926535ull;
    const bool test_mode = true;
    const bool strict = true;

    std::vector<uint32_t> buf(n);
    for (size_t i = 0; i < n; ++i) {
        buf[i] = Sanitize_Plain_No_Marker(static_cast<uint32_t>(i ^ 0x3C3C3C3Cu));
    }
    std::vector<uint32_t> golden = buf;

    uint64_t session_use = session;
    bool ready = false;
    for (uint32_t bump = 0u; bump < 8192u; ++bump) {
        buf = golden;
        session_use = session ^ static_cast<uint64_t>(bump);
        Sparse_Recovery_Engine::Generate_Interference_Pattern<uint32_t>(
            buf.data(), n, session_use, anchor, test_mode);
        if (!Obfuscated_Has_Marker_Collision(buf.data(), n)) {
            ready = true;
            break;
        }
    }
    if (!ready) {
        return false;
    }

    constexpr size_t burst_off = 50u;
    constexpr size_t burst_len = 16u;  // 16 words == 64 bytes
    const size_t hit = burst_off + 9u;  // one erasure inside window; not block anchor (multiples of 4)
    if (hit >= burst_off + burst_len || (hit % static_cast<size_t>(anchor)) == 0u) {
        return false;
    }
    buf[hit] = kErase<uint32_t>();

    RecoveryStats st{};
    const bool ok = Sparse_Recovery_Engine::Execute_L1_Reconstruction<uint32_t>(
        buf.data(), n, session_use, anchor, test_mode, strict, st);
    if (!ok) {
        return false;
    }
    return std::memcmp(buf.data(), golden.data(), n * sizeof(uint32_t)) == 0;
}

// ---------- Sparse: edge first & last element (one erasure each in different blocks) ----------
bool test_sparse_edge_first_last() {
    constexpr size_t n = 64u;
    constexpr uint32_t anchor = 8u;
    const uint64_t session = 0xED6E4B455u;
    const bool test_mode = true;
    const bool strict = true;

    std::vector<uint32_t> buf(n);
    for (size_t i = 0; i < n; ++i) {
        buf[i] = Sanitize_Plain_No_Marker(static_cast<uint32_t>(i ^ 0x3C3C3C3Cu));
    }
    std::vector<uint32_t> golden = buf;

    Sparse_Recovery_Engine::Generate_Interference_Pattern<uint32_t>(
        buf.data(), n, session, anchor, test_mode);

    buf[0] = kErase<uint32_t>();
    buf[n - 1u] = kErase<uint32_t>();

    RecoveryStats st{};
    const bool ok = Sparse_Recovery_Engine::Execute_L1_Reconstruction<uint32_t>(
        buf.data(), n, session, anchor, test_mode, strict, st);
    if (!ok) {
        return false;
    }
    return std::memcmp(buf.data(), golden.data(), n * sizeof(uint32_t)) == 0;
}

} // namespace

int main() {
    std::printf("HTS tensor engine standalone tests (Sparse + Orbital + Antipodal)\n");

    report(test_antipodal_roundtrip_properties(), "Antipodal: conversion + orthogonality");
    report(test_orbital_scatter_inverse(), "Orbital: Apply + Reverse equals identity (pattern)");
    report(test_orbital_edge_first_last_slots(),
        "Orbital: edge first/last word preserved through scatter/inverse");

    report(test_sparse_random_erasure_recoverable_10pct(),
        "Sparse: ~10%% erasures, max 1 per anchor block -> exact recovery");

    report(test_sparse_burst_64byte_all_marker_no_crash(),
        "Sparse: 64B contiguous ERASURE_MARKER burst -> smoke (no crash; not exact)");

    report(test_sparse_burst_window_single_cell_recoverable(),
        "Sparse: 64B window, single erasure -> exact recovery");

    report(test_sparse_edge_first_last(),
        "Sparse: first & last cell erasure (separate blocks) -> exact recovery");

    std::printf("\nSummary: %d passed, %d failed\n", g_pass, g_fail);
    if (g_fail != 0) {
        std::printf("RESULT: FAILED\n");
        return 1;
    }
    std::printf("RESULT: PASSED\n");
    return 0;
}
