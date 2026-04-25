// =============================================================================
// HTS_CFO_Bank_Test.cpp
// Isolated lab scaffold: 4D tensor TX/RX + optional V5a + optional holo sync
// =============================================================================
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "../HTS_LIM/HTS_CFO_V5a.hpp"
#include "../HTS_LIM/HTS_Holo_Tensor_4D.h"
#include "../HTS_LIM/HTS_Holo_Tensor_4D_Defs.h"
#include "../HTS_LIM/HTS_Preamble_Holographic.h"
#include "../HTS_LIM/HTS_Rx_CFO_SinCos_Table.hpp"

#if !defined(HTS_ALLOW_HOST_BUILD)
#error "Host-only lab scaffold (HTS_ALLOW_HOST_BUILD required)"
#endif

namespace {
using ProtectedEngine::HTS_Holo_Tensor_4D;
using ProtectedEngine::HoloTensor_Profile;
using ProtectedEngine::k_holo_profiles;
using ProtectedEngine::HOLO_MAX_BLOCK_BITS;
using ProtectedEngine::HOLO_CHIP_COUNT;

constexpr int kTrials = 100;
constexpr int16_t kAmp = 1000;
constexpr int kLag = 32;
constexpr double kPi = 3.14159265358979323846;
constexpr double kFs = 1000000.0;
constexpr uint64_t kAllValidMask = 0xFFFFFFFFFFFFFFFFull;
constexpr uint32_t kSeedMixA = 0x9E3779B9u;
constexpr uint32_t kSeedMixB = 0xA5A5A5A5u;
constexpr uint32_t kSeedMixC = 0xC3C3C3C3u;

// ────────────────────────────────────
// 영준님이 VS 에서 수정 가능 영역:
// - CFO sweep 범위: kCfoSweepHz
// - AWGN 세기: kAwgnSigma
// - V5a 적용 여부: kApplyV5a
// - HOLO sync 임계값: kHoloSyncMinEnergy
// - rx_soft 생성 (현재 Dispatcher 동일): make_rx_soft_iq_avg_
// ────────────────────────────────────
constexpr int kCfoSweepHz[] = {0, 100, 200, 300, 500, 700, 1000, 2000, 5000};
constexpr double kAwgnSigma = 0.0;
constexpr bool kApplyV5a = true;
constexpr int64_t kHoloSyncMinEnergy = 10000000LL;

struct TrialStats {
    int pass = 0;
    int bit_errors = 0;
    int total_bits = 0;
};

static uint32_t lcg_next_(uint32_t& s) noexcept {
    s = s * 1664525u + 1013904223u;
    return s;
}

static int16_t sat_i16_(int32_t v) noexcept {
    if (v > 32767) return 32767;
    if (v < -32768) return -32768;
    return static_cast<int16_t>(v);
}

static void fill_info_(uint32_t seed, int trial, uint8_t out[8]) noexcept {
    uint32_t s = seed ^ static_cast<uint32_t>(trial * 2654435761u);
    for (int i = 0; i < 8; ++i) {
        out[i] = static_cast<uint8_t>(lcg_next_(s) >> 24);
    }
}

static void bytes_to_bpsk_bits_(const uint8_t* bytes, int8_t* bits,
                                int bit_count) noexcept {
    for (int i = 0; i < bit_count; ++i) {
        const uint8_t b = static_cast<uint8_t>(
            (bytes[i >> 3] >> (7 - (i & 7))) & 1u);
        bits[i] = (b != 0u) ? 1 : -1;
    }
}

static void bpsk_to_bytes_(const int8_t* bits, uint8_t out[8]) noexcept {
    std::memset(out, 0, 8);
    for (int i = 0; i < 64; ++i) {
        if (bits[i] > 0) {
            out[i >> 3] = static_cast<uint8_t>(
                out[i >> 3] | (1u << (7 - (i & 7))));
        }
    }
}

static int bit_errors_(const uint8_t a[8], const uint8_t b[8]) noexcept {
    int err = 0;
    for (int i = 0; i < 8; ++i) {
        uint8_t d = static_cast<uint8_t>(a[i] ^ b[i]);
        d = static_cast<uint8_t>(d - ((d >> 1) & 0x55u));
        d = static_cast<uint8_t>((d & 0x33u) + ((d >> 2) & 0x33u));
        d = static_cast<uint8_t>((d + (d >> 4)) & 0x0Fu);
        err += static_cast<int>(d);
    }
    return err;
}

static void derive_master_seed_(uint32_t seed, uint32_t master[4]) noexcept {
    master[0] = seed;
    master[1] = seed ^ kSeedMixA;
    master[2] = seed ^ kSeedMixB;
    master[3] = seed ^ kSeedMixC;
}

static bool build_tensor_tx_chips_(HTS_Holo_Tensor_4D& tx, uint32_t seed,
                                   uint32_t slot, const uint8_t info[8],
                                   int16_t out_i[64], int16_t out_q[64]) noexcept {
    const HoloTensor_Profile profile = k_holo_profiles[1];
    int8_t in_bits[HOLO_MAX_BLOCK_BITS]{};
    int8_t chip_bpsk[HOLO_CHIP_COUNT]{};
    bytes_to_bpsk_bits_(info, in_bits, static_cast<int>(profile.block_bits));
    if (tx.Set_Time_Slot(slot) != HTS_Holo_Tensor_4D::SECURE_TRUE) return false;
    if (tx.Encode_Block(in_bits, profile.block_bits, chip_bpsk,
                        profile.chip_count) != HTS_Holo_Tensor_4D::SECURE_TRUE) {
        return false;
    }
    const int lk = static_cast<int>(profile.num_layers) *
                   static_cast<int>(profile.block_bits);
    const int denom_raw = (lk > 0) ? lk : 32;
    const int denom = (denom_raw > 1) ? (denom_raw >> 1) : 1;
    for (int c = 0; c < 64; ++c) {
        const int32_t prod =
            static_cast<int32_t>(chip_bpsk[c]) * static_cast<int32_t>(kAmp);
        const int32_t v =
            (prod >= 0) ? ((prod + (denom >> 1)) / denom)
                        : ((prod - (denom >> 1)) / denom);
        out_i[c] = sat_i16_(v);
        out_q[c] = sat_i16_(v);  // Dispatcher TX 동일: I=Q
    }
    (void)seed;
    return true;
}

static void apply_channel_cfo_awgn_(const int16_t in_i[64], const int16_t in_q[64],
                                    int cfo_hz, uint32_t noise_seed,
                                    int16_t out_i[64], int16_t out_q[64]) noexcept {
    const double w = 2.0 * kPi * static_cast<double>(cfo_hz) / kFs;
    uint32_t s = noise_seed;
    for (int n = 0; n < 64; ++n) {
        const double ph = w * static_cast<double>(n);
        const double c = std::cos(ph);
        const double si = std::sin(ph);
        const double ri = static_cast<double>(in_i[n]) * c -
                          static_cast<double>(in_q[n]) * si;
        const double rq = static_cast<double>(in_i[n]) * si +
                          static_cast<double>(in_q[n]) * c;
        double ni = 0.0;
        double nq = 0.0;
        if (kAwgnSigma > 0.0) {
            const double u1 = (static_cast<double>(lcg_next_(s) & 0x00FFFFFFu) + 1.0) /
                              16777217.0;
            const double u2 = (static_cast<double>(lcg_next_(s) & 0x00FFFFFFu) + 1.0) /
                              16777217.0;
            const double r = std::sqrt(-2.0 * std::log(u1));
            const double th = 2.0 * kPi * u2;
            ni = kAwgnSigma * r * std::cos(th);
            nq = kAwgnSigma * r * std::sin(th);
        }
        out_i[n] = sat_i16_(static_cast<int32_t>(std::llround(ri + ni)));
        out_q[n] = sat_i16_(static_cast<int32_t>(std::llround(rq + nq)));
    }
}

static void apply_v5a_if_enabled_(int16_t io_i[64], int16_t io_q[64]) noexcept {
    if (!kApplyV5a) return;
    int64_t ac_i = 0;
    int64_t ac_q = 0;
    for (int n = 0; n + kLag < 64; ++n) {
        const int32_t i0 = io_i[n];
        const int32_t q0 = io_q[n];
        const int32_t i1 = io_i[n + kLag];
        const int32_t q1 = io_q[n + kLag];
        ac_i += static_cast<int64_t>(i0) * i1 + static_cast<int64_t>(q0) * q1;
        ac_q += static_cast<int64_t>(q0) * i1 - static_cast<int64_t>(i0) * q1;
    }
    hts::rx_cfo::CFO_V5a v5a{};
    v5a.Init();
    v5a.Estimate_From_Autocorr(static_cast<int32_t>(ac_i), static_cast<int32_t>(ac_q),
                               kLag);
    for (int n = 0; n < 64; ++n) {
        v5a.Apply_Per_Chip(io_i[n], io_q[n]);
    }
}

static bool sync_gate_(bool use_holo_sync, const int16_t in_i[64],
                       const int16_t in_q[64]) noexcept {
    if (!use_holo_sync) {
        return true;  // 일반 sync/우회 실험 모드
    }
    const int64_t e = ProtectedEngine::Holographic::holographic_dot_segmented(
        in_i, in_q);
    return e >= kHoloSyncMinEnergy;
}

static void make_rx_soft_iq_avg_(const int16_t in_i[64], const int16_t in_q[64],
                                 int16_t out_soft[64]) noexcept {
    // Dispatcher RX 동일 경로: (I + Q) / 2
    for (int c = 0; c < 64; ++c) {
        const int32_t sum = static_cast<int32_t>(in_i[c]) +
                            static_cast<int32_t>(in_q[c]);
        out_soft[c] = static_cast<int16_t>(sum / 2);
    }
}

static bool decode_tensor_rx_(HTS_Holo_Tensor_4D& rx, uint32_t slot,
                              const int16_t rx_i[64], const int16_t rx_q[64],
                              uint8_t out[8]) noexcept {
    const HoloTensor_Profile profile = k_holo_profiles[1];
    int16_t rx_soft[64]{};
    int8_t out_bits[HOLO_MAX_BLOCK_BITS]{};
    make_rx_soft_iq_avg_(rx_i, rx_q, rx_soft);
    if (rx.Set_Time_Slot(slot) != HTS_Holo_Tensor_4D::SECURE_TRUE) return false;
    if (rx.Decode_Block(rx_soft, profile.chip_count, kAllValidMask,
                        out_bits, profile.block_bits) != HTS_Holo_Tensor_4D::SECURE_TRUE) {
        return false;
    }
    bpsk_to_bytes_(out_bits, out);
    return true;
}

static TrialStats run_sweep_mode_(bool use_holo_sync, int cfo_hz) noexcept {
    TrialStats st{};
    for (int t = 0; t < kTrials; ++t) {
        const uint32_t seed = 0xA5100000u ^ static_cast<uint32_t>(t * 1103515245u);
        const uint32_t slot = 0u;
        uint8_t tx_info[8]{};
        fill_info_(seed, t, tx_info);

        uint32_t master[4]{};
        derive_master_seed_(seed, master);
        HTS_Holo_Tensor_4D tx{};
        HTS_Holo_Tensor_4D rx{};
        if (tx.Initialize(master, nullptr) != HTS_Holo_Tensor_4D::SECURE_TRUE) continue;
        if (rx.Initialize(master, nullptr) != HTS_Holo_Tensor_4D::SECURE_TRUE) continue;

        int16_t tx_i[64]{};
        int16_t tx_q[64]{};
        if (!build_tensor_tx_chips_(tx, seed, slot, tx_info, tx_i, tx_q)) continue;

        int16_t ch_i[64]{};
        int16_t ch_q[64]{};
        apply_channel_cfo_awgn_(tx_i, tx_q, cfo_hz, seed ^ 0x55AA1234u, ch_i, ch_q);
        apply_v5a_if_enabled_(ch_i, ch_q);
        if (!sync_gate_(use_holo_sync, ch_i, ch_q)) {
            st.bit_errors += 64;
            st.total_bits += 64;
            continue;
        }

        uint8_t rx_info[8]{};
        const bool ok = decode_tensor_rx_(rx, slot, ch_i, ch_q, rx_info);
        const int ber = ok ? bit_errors_(tx_info, rx_info) : 64;
        if (ok && ber == 0) {
            st.pass++;
        }
        st.bit_errors += ber;
        st.total_bits += 64;
        rx.Shutdown();
        tx.Shutdown();
    }
    return st;
}

void test_holo_tensor_cfo_sweep() {
    const char* const mode_name[2] = {"S5-like(sync-off)", "S5H-like(sync-on)"};
    for (int mode = 0; mode < 2; ++mode) {
        const bool use_holo_sync = (mode != 0);
        std::printf("\n[ISOLATED] mode=%s v5a=%d awgn_sigma=%.2f\n",
                    mode_name[mode], kApplyV5a ? 1 : 0, kAwgnSigma);
        for (int i = 0; i < static_cast<int>(sizeof(kCfoSweepHz) / sizeof(kCfoSweepHz[0])); ++i) {
            const int cfo = kCfoSweepHz[i];
            const TrialStats st = run_sweep_mode_(use_holo_sync, cfo);
            const double ber =
                (st.total_bits > 0) ? (static_cast<double>(st.bit_errors) /
                                       static_cast<double>(st.total_bits))
                                    : 1.0;
            std::printf("cfo=%5d Hz: %3d/%3d PASS  BER=%.6f\n",
                        cfo, st.pass, kTrials, ber);
        }
    }
}

}  // namespace

int main() {
    hts::rx_cfo::Build_SinCos_Table();
    test_holo_tensor_cfo_sweep();
    std::printf("HTS_CFO_Bank_Test: PASS\n");
    return 0;
}
