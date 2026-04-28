// =============================================================================
// HTS_Payload_UT.cpp — Payload 양산 분기 단위 테스트
// 목적: TX → 채널 회전 → RX → CRC 검증 모든 단계 dump
// =============================================================================

#include "HTS_Holo_Tensor_4D_TX.h"
#include "HTS_Holo_Tensor_4D_RX.h"
#include "HTS_Holo_Tensor_4D_Common.h"
#include "HTS_FEC_HARQ.hpp"

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cmath>

using namespace ProtectedEngine;

// ─────────────────────────────────────────────────────────────
// 유틸리티 함수 (Payload.cpp 의 bpsk_to_bytes_ / bytes_to_bpsk_bits_ 와 동일)
// ─────────────────────────────────────────────────────────────

uint16_t bytes_to_bpsk_bits_ut(const uint8_t* bytes, size_t byte_len,
                               int8_t* bpsk_bits, uint16_t max_bits) {
    if (bpsk_bits == nullptr) {
        return 0u;
    }
    for (uint16_t i = 0; i < max_bits; ++i) {
        bpsk_bits[i] = -1;
    }
    if (bytes == nullptr || byte_len == 0u) {
        return 0u;
    }
    uint16_t bit_idx = 0u;
    for (size_t b = 0u; b < byte_len; ++b) {
        for (int i = 7; i >= 0; --i) {
            if (bit_idx >= max_bits) {
                return bit_idx;
            }
            const uint8_t bit = (bytes[b] >> i) & 1u;
            bpsk_bits[bit_idx] = (bit != 0u) ? 1 : -1;
            ++bit_idx;
        }
    }
    return bit_idx;
}

size_t bpsk_to_bytes_ut(const int8_t* bpsk_bits, uint16_t bit_count,
                        uint8_t* bytes, size_t max_bytes) {
    if (bpsk_bits == nullptr || bytes == nullptr) {
        return 0u;
    }
    const size_t byte_count_full = (bit_count + 7u) >> 3u;
    const size_t byte_count =
        (byte_count_full > max_bytes) ? max_bytes : byte_count_full;
    std::memset(bytes, 0, byte_count);
    for (uint16_t i = 0u; i < bit_count; ++i) {
        const size_t bi = i >> 3u;
        if (bi >= byte_count) {
            break;
        }
        if (bpsk_bits[i] > 0) {
            bytes[bi] |= (1u << (7u - (i & 7u)));
        }
    }
    return byte_count;
}

bool packet_crc16_ok_8_ut(const uint8_t* p) {
    const uint16_t calc = FEC_HARQ::CRC16(p, 6);
    const uint16_t rx = static_cast<uint16_t>(p[6]) |
                        (static_cast<uint16_t>(p[7]) << 8);
    return calc == rx;
}

// ─────────────────────────────────────────────────────────────
// 채널 시뮬 (S2 위상 회전)
// ─────────────────────────────────────────────────────────────

void channel_rotate_iq_same(const int8_t* tx_chips, int N, double theta_deg,
                            int16_t* rx_I, int16_t* rx_Q) {
    // IQ_SAME: TX I = Q = chip
    const double rad = theta_deg * 3.14159265358979 / 180.0;
    const double cos_p = std::cos(rad);
    const double sin_p = std::sin(rad);
    for (int c = 0; c < N; ++c) {
        const double I_in = static_cast<double>(tx_chips[c]);
        const double Q_in = static_cast<double>(tx_chips[c]);
        const double I_out = I_in * cos_p - Q_in * sin_p;
        const double Q_out = I_in * sin_p + Q_in * cos_p;
        // amp 100배 (실제 시뮬 환경 모방)
        rx_I[c] = static_cast<int16_t>(I_out * 100.0);
        rx_Q[c] = static_cast<int16_t>(Q_out * 100.0);
    }
}

// ─────────────────────────────────────────────────────────────
// 단일 패킷 테스트 (Payload.cpp on_sym_ + try_decode_ 흐름 모방)
// ─────────────────────────────────────────────────────────────

struct UT_Result {
    bool pass;
    int bit_errors;
    bool crc_a;
    bool crc_b;
    bool selected_alt;
    uint8_t bytes_main[8];
    uint8_t bytes_alt[8];
    uint8_t truth[8];
};

UT_Result test_payload_at_angle(double theta_deg) {
    UT_Result r{};

    // 1. truth packet 생성 (6 byte data + 2 byte CRC)
    uint8_t truth_data[6] = {0xAB, 0xCD, 0x12, 0x34, 0x56, 0x78};
    uint16_t crc = FEC_HARQ::CRC16(truth_data, 6);
    uint8_t truth_packet[8];
    std::memcpy(truth_packet, truth_data, 6);
    truth_packet[6] = static_cast<uint8_t>(crc & 0xFF);
    truth_packet[7] = static_cast<uint8_t>((crc >> 8) & 0xFF);
    std::memcpy(r.truth, truth_packet, 8);

    std::printf("\n[UT @ %.0f deg]\n", theta_deg);
    std::printf("  truth: ");
    for (int i = 0; i < 8; ++i) {
        std::printf("%02X ", truth_packet[i]);
    }
    std::printf("\n");

    // 2. 8 byte → 64 bit BPSK
    int8_t truth_bits[64];
    bytes_to_bpsk_bits_ut(truth_packet, 8, truth_bits, 64);

    // 3. TX: 4 block × 16 bit → 4 × 64 chip = 256 chip
    HTS_Holo_Tensor_4D_TX tx;
    uint32_t seed[4] = {0x12345678, 0x9ABCDEF0, 0xCAFEBABE, 0xDEADBEEF};
    (void)tx.Initialize(seed, &k_holo_profiles[1]);  // K=16, N=64
    (void)tx.Set_Time_Slot(0);

    HTS_Holo_Tensor_4D_RX rx;
    (void)rx.Initialize(seed, &k_holo_profiles[1]);
    (void)rx.Set_Time_Slot(0);

    const uint16_t K = k_holo_profiles[1].block_bits;
    const uint16_t N = k_holo_profiles[1].chip_count;

    uint8_t bytes_main[8] = {};
    uint8_t bytes_alt[8] = {};
    size_t bytes_pos = 0;

    int8_t tx_chips[64];
    int16_t rx_I[64], rx_Q[64];
    int16_t rx_soft[64];
    int8_t rx_bits[16];
    int8_t sym0[16], sym1[16];

    for (int blk = 0; blk < 4; ++blk) {
        // 한 block = 16 bit
        const int8_t* block_bits = &truth_bits[blk * 16];

        // TX encode
        const uint32_t enc_ok = tx.Encode_Block(block_bits, K, tx_chips, N);
        if (enc_ok != HTS_Holo_Tensor_4D_TX::SECURE_TRUE) {
            std::printf("  [block %d] TX encode FAIL\n", blk);
            continue;
        }

        // 채널 회전
        channel_rotate_iq_same(tx_chips, N, theta_deg, rx_I, rx_Q);

        // RX 양산 분기: rx_soft = (I+Q)/2
        for (int c = 0; c < N; ++c) {
            int32_t s = static_cast<int32_t>(rx_I[c]) + static_cast<int32_t>(rx_Q[c]);
            rx_soft[c] = static_cast<int16_t>(s / 2);
        }

        // RX Decode_Block
        const uint32_t dec_ok = rx.Decode_Block(rx_soft, N, 0xFFFFFFFFFFFFFFFFull,
                                                  rx_bits, K);

        if (dec_ok == HTS_Holo_Tensor_4D_RX::SECURE_TRUE) {
            for (uint16_t k = 0; k < K; ++k) {
                sym0[k] = rx_bits[k];
                sym1[k] = static_cast<int8_t>(-rx_bits[k]);
            }

            // bytes / bytes_alt 누적
            const size_t room_main = 8 - bytes_pos;
            const size_t wr0 =
                bpsk_to_bytes_ut(sym0, K, &bytes_main[bytes_pos], room_main);
            const size_t wr1 =
                bpsk_to_bytes_ut(sym1, K, &bytes_alt[bytes_pos], room_main);

            std::printf(
                "  [block %d] dec_ok=1 wr0=%zu rx_bits[0..7]=%d %d %d %d %d %d %d %d\n",
                blk, wr0, rx_bits[0], rx_bits[1], rx_bits[2], rx_bits[3],
                rx_bits[4], rx_bits[5], rx_bits[6], rx_bits[7]);
            (void)wr1;

            bytes_pos += wr0;
        } else {
            std::printf("  [block %d] dec_ok=0 (decode FAIL)\n", blk);
        }
    }

    std::memcpy(r.bytes_main, bytes_main, 8);
    std::memcpy(r.bytes_alt, bytes_alt, 8);

    // CRC 검증
    r.crc_a = packet_crc16_ok_8_ut(bytes_main);
    r.crc_b = packet_crc16_ok_8_ut(bytes_alt);
    r.selected_alt = (r.crc_b && !r.crc_a);

    std::printf("  bytes_main: ");
    for (int i = 0; i < 8; ++i) {
        std::printf("%02X ", bytes_main[i]);
    }
    std::printf("(crc_a=%d)\n", static_cast<int>(r.crc_a));

    std::printf("  bytes_alt:  ");
    for (int i = 0; i < 8; ++i) {
        std::printf("%02X ", bytes_alt[i]);
    }
    std::printf("(crc_b=%d)\n", static_cast<int>(r.crc_b));

    std::printf("  selected:   %s\n", r.selected_alt ? "alt" : "main");

    // BER 계산
    const uint8_t* selected = r.selected_alt ? bytes_alt : bytes_main;
    int bit_err = 0;
    for (int i = 0; i < 6; ++i) {  // data 6 byte 만 비교
        for (int j = 0; j < 8; ++j) {
            const uint8_t a = (selected[i] >> (7 - j)) & 1;
            const uint8_t t = (truth_packet[i] >> (7 - j)) & 1;
            if (a != t) {
                ++bit_err;
            }
        }
    }
    r.bit_errors = bit_err;
    r.pass = (bit_err == 0);

    std::printf("  bit_errors: %d / 48 (%.4f) %s\n", bit_err,
                static_cast<double>(bit_err) / 48.0, r.pass ? "PASS" : "FAIL");

    tx.Shutdown();
    rx.Shutdown();

    return r;
}

// ─────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    (void)argc;
    (void)argv;
    std::printf("============================================\n");
    std::printf("  HTS Payload Unit Test (UT-2)\n");
    std::printf("  양산 분기: (I+Q)/2 → Decode_Block → CRC\n");
    std::printf("============================================\n");

    constexpr int kNumAngles = 8;
    const double angles[kNumAngles] = {0, 45, 90, 135, 180, 225, 270, 315};
    int pass_count = 0;

    for (int ai = 0; ai < kNumAngles; ++ai) {
        const double a = angles[ai];
        UT_Result r = test_payload_at_angle(a);
        if (r.pass) {
            ++pass_count;
        }
    }

    std::printf("\n============================================\n");
    std::printf("  결과: %d / %d PASS\n", pass_count, kNumAngles);
    std::printf("============================================\n");

    return 0;
}
