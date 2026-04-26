// =============================================================================
// HTS_V400_Dispatcher_Payload.cpp — on_sym / try_decode / HARQ / hdr / retx
// =============================================================================
#include "HTS_V400_Dispatcher.hpp"
#include "HTS_V400_Dispatcher_Internal.hpp"
#include "HTS_Secure_Memory.h"
#if defined(HTS_HARQ_DIAG)
#include "HTS_HARQ_Diag.hpp"
#endif
#if defined(HTS_AMP_DIAG)
#include "HTS_Amp_Diag.hpp"
#endif
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <cstddef>
namespace ProtectedEngine {
using namespace detail;
#if defined(HTS_USE_HOLO_TENSOR_4D)
namespace {
uint16_t bytes_to_bpsk_bits_(const uint8_t* bytes, size_t byte_len,
                             int8_t* bpsk_bits, uint16_t max_bits) noexcept {
    if (bpsk_bits == nullptr) {
        return 0u;
    }
    for (uint16_t i = 0; i < max_bits; ++i) {
        bpsk_bits[i] = static_cast<int8_t>(-1);
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
            const uint8_t bit = static_cast<uint8_t>((bytes[b] >> i) & 1u);
            bpsk_bits[bit_idx] =
                (bit != 0u) ? static_cast<int8_t>(1) : static_cast<int8_t>(-1);
            ++bit_idx;
        }
    }
    return bit_idx;
}

size_t bpsk_to_bytes_(const int8_t* bpsk_bits, uint16_t bit_count, uint8_t* bytes,
                      size_t max_bytes) noexcept {
    if (bpsk_bits == nullptr || bytes == nullptr) {
        return 0u;
    }
    const size_t byte_count_full =
        (static_cast<size_t>(bit_count) + 7u) >> 3u;
    const size_t byte_count =
        (byte_count_full > max_bytes) ? max_bytes : byte_count_full;
    std::memset(bytes, 0, byte_count);
    for (uint16_t i = 0u; i < bit_count; ++i) {
        const size_t bi = static_cast<size_t>(i >> 3u);
        if (bi >= byte_count) {
            break;
        }
        if (bpsk_bits[i] > 0) {
            bytes[bi] = static_cast<uint8_t>(
                static_cast<uint32_t>(bytes[bi]) |
                (1u << (7u - (static_cast<uint32_t>(i) & 7u))));
        }
    }
    return byte_count;
}
#if defined(HTS_USE_4D_DIVERSITY)
/// B' RX: ts_I = rx_slot, ts_Q = rx_slot ^ 1 — int64 metric 합 + BPTE 부호.
/// Step 2-3e: 호출은 A2' 로 전환; 본 함수는 보존. T6 단일 TU /WX C4505 방지.
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4505)
#endif
static uint32_t holo_tensor_decode_4d_diversity_slots_(
    HTS_Holo_Tensor_4D_RX& holo_rx,
    uint32_t rx_slot,
    const int16_t* buf_I,
    const int16_t* buf_Q,
    uint16_t N_chip,
    uint16_t K_bits,
    int8_t* out_sym0,
    int8_t* out_sym1) noexcept
{
    int8_t cand0_I[HOLO_MAX_BLOCK_BITS];
    int8_t cand1_I[HOLO_MAX_BLOCK_BITS];
    int32_t metric_I[HOLO_MAX_BLOCK_BITS];
    int8_t cand0_Q[HOLO_MAX_BLOCK_BITS];
    int8_t cand1_Q[HOLO_MAX_BLOCK_BITS];
    int32_t metric_Q[HOLO_MAX_BLOCK_BITS];

    (void)holo_rx.Set_Time_Slot(rx_slot);
    const uint32_t ok_I = holo_rx.Decode_Block_Two_Candidates_With_Metric(
        buf_I, buf_Q, N_chip, 0xFFFFFFFFFFFFFFFFull, cand0_I, cand1_I, metric_I,
        K_bits);
    (void)holo_rx.Set_Time_Slot(rx_slot ^ 1u);
    const uint32_t ok_Q = holo_rx.Decode_Block_Two_Candidates_With_Metric(
        buf_I, buf_Q, N_chip, 0xFFFFFFFFFFFFFFFFull, cand0_Q, cand1_Q, metric_Q,
        K_bits);
    (void)holo_rx.Set_Time_Slot(rx_slot);

    if (ok_I != HTS_Holo_Tensor_4D_RX::SECURE_TRUE ||
        ok_Q != HTS_Holo_Tensor_4D_RX::SECURE_TRUE) {
        SecureMemory::secureWipe(static_cast<void*>(metric_I), sizeof(metric_I));
        SecureMemory::secureWipe(static_cast<void*>(metric_Q), sizeof(metric_Q));
        SecureMemory::secureWipe(static_cast<void*>(cand0_I), sizeof(cand0_I));
        SecureMemory::secureWipe(static_cast<void*>(cand1_I), sizeof(cand1_I));
        SecureMemory::secureWipe(static_cast<void*>(cand0_Q), sizeof(cand0_Q));
        SecureMemory::secureWipe(static_cast<void*>(cand1_Q), sizeof(cand1_Q));
        return HTS_Holo_Tensor_4D_RX::SECURE_FALSE;
    }

    for (uint16_t k = 0u; k < K_bits; ++k) {
        const int64_t llr = static_cast<int64_t>(metric_I[static_cast<size_t>(k)]) +
                            static_cast<int64_t>(metric_Q[static_cast<size_t>(k)]);
        const uint64_t sign_u =
            static_cast<uint64_t>(static_cast<uint64_t>(llr) >> 63u);
        const int8_t cand0 =
            static_cast<int8_t>(1 - 2 * static_cast<int32_t>(sign_u));
        out_sym0[static_cast<size_t>(k)] = cand0;
        out_sym1[static_cast<size_t>(k)] = static_cast<int8_t>(-cand0);
    }

    SecureMemory::secureWipe(static_cast<void*>(metric_I), sizeof(metric_I));
    SecureMemory::secureWipe(static_cast<void*>(metric_Q), sizeof(metric_Q));
    SecureMemory::secureWipe(static_cast<void*>(cand0_I), sizeof(cand0_I));
    SecureMemory::secureWipe(static_cast<void*>(cand1_I), sizeof(cand1_I));
    SecureMemory::secureWipe(static_cast<void*>(cand0_Q), sizeof(cand0_Q));
    SecureMemory::secureWipe(static_cast<void*>(cand1_Q), sizeof(cand1_Q));
    return HTS_Holo_Tensor_4D_RX::SECURE_TRUE;
}
#if defined(_MSC_VER)
#pragma warning(pop)
#endif

/// A2' RX (Step 2-3e): I/Q 각각 `Decode_Block`(단일 칩열, (I+Q)/2 없음) 후 비트 합 BPTE.
/// B' 헬퍼(`holo_tensor_decode_4d_diversity_slots_`)는 보존; 호출만 A2' 로 전환.
static uint32_t holo_tensor_decode_4d_a2_prime_(
    HTS_Holo_Tensor_4D_RX& holo_rx,
    uint32_t rx_slot,
    const int16_t* buf_I,
    const int16_t* buf_Q,
    uint16_t N_chip,
    uint16_t K_bits,
    int8_t* out_sym0,
    int8_t* out_sym1) noexcept
{
    int8_t bits_I[HOLO_MAX_BLOCK_BITS];
    int8_t bits_Q[HOLO_MAX_BLOCK_BITS];

    (void)holo_rx.Set_Time_Slot(rx_slot);
    const uint32_t ok_I = holo_rx.Decode_Block(
        buf_I, N_chip, 0xFFFFFFFFFFFFFFFFull, bits_I, K_bits);
    (void)holo_rx.Set_Time_Slot(rx_slot ^ 1u);
    const uint32_t ok_Q = holo_rx.Decode_Block(
        buf_Q, N_chip, 0xFFFFFFFFFFFFFFFFull, bits_Q, K_bits);
    (void)holo_rx.Set_Time_Slot(rx_slot);

    if (ok_I != HTS_Holo_Tensor_4D_RX::SECURE_TRUE ||
        ok_Q != HTS_Holo_Tensor_4D_RX::SECURE_TRUE) {
        SecureMemory::secureWipe(static_cast<void*>(bits_I), sizeof(bits_I));
        SecureMemory::secureWipe(static_cast<void*>(bits_Q), sizeof(bits_Q));
        return HTS_Holo_Tensor_4D_RX::SECURE_FALSE;
    }

    for (uint16_t k = 0u; k < K_bits; ++k) {
        const int32_t sum = static_cast<int32_t>(bits_I[static_cast<size_t>(k)]) +
                            static_cast<int32_t>(bits_Q[static_cast<size_t>(k)]);
        const uint32_t sign_u = static_cast<uint32_t>(sum) >> 31u;
        const int8_t cand0 =
            static_cast<int8_t>(1 - 2 * static_cast<int32_t>(sign_u));
        out_sym0[static_cast<size_t>(k)] = cand0;
        out_sym1[static_cast<size_t>(k)] = static_cast<int8_t>(-cand0);
    }

    SecureMemory::secureWipe(static_cast<void*>(bits_I), sizeof(bits_I));
    SecureMemory::secureWipe(static_cast<void*>(bits_Q), sizeof(bits_Q));
    return HTS_Holo_Tensor_4D_RX::SECURE_TRUE;
}
#endif
#if defined(HTS_HOLO_RX_PHASE_B)
/// 8바이트 페이로드: 앞 6바이트 CRC-16/CCITT, 리틀엔디안 CRC at [6],[7]
static bool holo_tensor_packet_crc16_ok_8(const uint8_t* p) noexcept {
    const uint16_t calc = FEC_HARQ::CRC16(p, 6);
    const uint16_t rx = static_cast<uint16_t>(
        static_cast<uint16_t>(p[6]) |
        (static_cast<uint16_t>(p[7]) << 8));
    return calc == rx;
}
#endif
} // namespace
#endif
void HTS_V400_Dispatcher::tpc_rx_feedback_after_decode_(
    DecodedPacket& pkt) noexcept {
    // ── P0-FIX-005 Stage 0: TPC feedback 임시 격리 ──
    //  구(舊): Embed_Feedback 이 pkt.data[7] 상위 2 bit 를 RSSI 로 덮어써
    //   info payload 의 bit 56, 57 을 파괴 (DIAG-CORRUPT 실증).
    //   Extract_Feedback 은 TX 가 사전 embed 하지 않은 payload 를 오독하여
    //   잘못된 TPC state 로 갱신.
    //  신(新): 함수 전체를 no-op 화. TPC 기능은 Stage 2 에서 헤더 기반으로
    //   재구현 예정 (info payload 무결성 보장).
    (void)pkt;
}
void HTS_V400_Dispatcher::fill_sic_expected_64_() noexcept {
    sic_expect_valid_ = false;
    if (!sic_ir_enabled_ || ir_state_ == nullptr) {
        return;
    }
    if (ir_state_->sic_tentative_valid == 0u) {
        return;
    }
    std::memset(g_sic_bits, 0, sizeof(g_sic_bits));
    uint8_t *const syms = g_v400_sym_scratch;
    const int rv_fb = (ir_rv_ + 3) & 3;
    const uint32_t il = seed_ ^ (rx_seq_ * 0xA5A5A5A5u);
    const int enc_n =
        FEC_HARQ::Encode64_IR(ir_state_->sic_tentative, FEC_HARQ::MAX_INFO,
                              syms, il, cur_bps64_, rv_fb, wb_);
    if (enc_n <= 0) {
        SecureMemory::secureWipe(static_cast<void *>(syms),
                                 sizeof(g_v400_sym_scratch));
        return;
    }
    // Walsh 부호 비트맵 — walsh_enc()와 동일: chip = amp×(1−2×(popc(sym&j)&1))
    for (int s = 0; s < enc_n; ++s) {
        const uint32_t sym_u = static_cast<uint32_t>(
            syms[static_cast<std::size_t>(s)]);
        uint64_t bits = 0u;
        for (int c = 0; c < 64; ++c) {
            const uint32_t neg =
                popc32(sym_u & static_cast<uint32_t>(c)) & 1u;
            bits |= (static_cast<uint64_t>(neg)
                     << static_cast<uint32_t>(c));
        }
        g_sic_bits[static_cast<std::size_t>(s)] = bits;
    }
    sic_expect_valid_ = true;
    SecureMemory::secureWipe(static_cast<void *>(syms),
                             sizeof(g_v400_sym_scratch));
}
uint32_t HTS_V400_Dispatcher::parse_hdr_(PayloadMode &mode,
                                         int &plen) noexcept {
    const uint16_t hdr = (static_cast<uint16_t>(hdr_syms_[0]) << 6u) |
                         static_cast<uint16_t>(hdr_syms_[1]);
    const uint8_t mb = static_cast<uint8_t>((hdr >> 10u) & 0x03u);
    const uint32_t rx_iq_split =
        static_cast<uint32_t>((hdr & HDR_IQ_BIT) != 0u);
    plen = static_cast<int>(hdr & 0x01FFu);
    iq_mode_ = static_cast<IQ_Mode>(
        static_cast<uint32_t>(IQ_Mode::IQ_INDEPENDENT) * rx_iq_split +
        static_cast<uint32_t>(IQ_Mode::IQ_SAME) * (1u - rx_iq_split));
    static constexpr PayloadMode k_hdr_modes[4] = {
        PayloadMode::VIDEO_1,
        PayloadMode::VIDEO_16,
        PayloadMode::VOICE,
        PayloadMode::DATA,
    };
    mode = k_hdr_modes[static_cast<size_t>(mb & 3u)];
    const uint32_t m0 = static_cast<uint32_t>(mb == 0u);
    const uint32_t m1 = static_cast<uint32_t>(mb == 1u);
    const uint32_t m2 = static_cast<uint32_t>(mb == 2u);
    const uint32_t m3 = static_cast<uint32_t>(mb == 3u);
    const uint32_t plen_ok_v1 = static_cast<uint32_t>(plen == FEC_HARQ::NSYM1);
    const uint32_t plen_ok_v16 =
        static_cast<uint32_t>(plen == FEC_HARQ::NSYM16);
    const int bps = FEC_HARQ::bps_from_nsym(plen);
    const uint32_t bps_ge =
        static_cast<uint32_t>(bps >= FEC_HARQ::BPS64_MIN_OPERABLE);
    const uint32_t bps_le = static_cast<uint32_t>(bps <= FEC_HARQ::BPS64_MAX);
    const uint32_t plen_sym =
        static_cast<uint32_t>(plen == FEC_HARQ::nsym_for_bps(bps));
    const uint32_t data_ok = m3 & bps_ge & bps_le & plen_sym;
#if defined(HTS_USE_HOLO_TENSOR_4D)
    const uint32_t tensor_k =
        static_cast<uint32_t>(k_holo_profiles[1].block_bits);
    const uint32_t tensor_ok = m3 & static_cast<uint32_t>(
                                       static_cast<uint32_t>(plen) == tensor_k);
#else
    const uint32_t tensor_ok = 0u;
#endif
    const uint32_t ok_u = (m0 & plen_ok_v1) | (m1 & plen_ok_v16) |
                          (m2 & plen_ok_v16) | data_ok | tensor_ok;
    const int d_int = static_cast<int>(data_ok);
    cur_bps64_ =
        (bps * static_cast<int>(data_ok)) + (cur_bps64_ * (1 - d_int));
#if defined(HTS_USE_HOLO_TENSOR_4D)
    holo_tensor_payload_mode_ = (tensor_ok != 0u);
#if defined(HTS_PHASE_H_DIAG)
    static uint32_t s_hdr_diag_count = 0u;
    const bool force_diag_hdr = (g_phase_h_diag_force != 0) &&
                                (g_phase_h_diag_seed == seed_);
    if (force_diag_hdr || s_hdr_diag_count < 4u) {
        ++s_hdr_diag_count;
        std::printf(
            "[PHASE-H][RX-HDR] mb=%u plen=%d data_ok=%u tensor_ok=%u payload_mode=%u\n",
            static_cast<unsigned>(mb), plen, static_cast<unsigned>(data_ok),
            static_cast<unsigned>(tensor_ok),
            static_cast<unsigned>(holo_tensor_payload_mode_ ? 1u : 0u));
    }
#endif
#endif
    if (ir_mode_) {
        iq_mode_ = IQ_Mode::IQ_SAME;
        iq_upgrade_count_ = 0u;
    }
    return static_cast<uint32_t>(0u - ok_u);
}
void HTS_V400_Dispatcher::on_sym_() noexcept {
    pay_recv_++;
    if (cur_mode_ == PayloadMode::VIDEO_1) {
        if (v1_idx_ < 80) {
            v1_rx_[v1_idx_++] = buf_I_[0];
        } else {
            full_reset_();
            return;
        }
    } else if (cur_mode_ == PayloadMode::VIDEO_16 ||
               cur_mode_ == PayloadMode::VOICE ||
               cur_mode_ == PayloadMode::DATA) {
        const int nc = (cur_mode_ == PayloadMode::DATA) ? 64 : 16;
#if defined(HTS_USE_HOLO_TENSOR_4D)
        if (cur_mode_ == PayloadMode::DATA && holo_tensor_payload_mode_) {
            if (sym_idx_ >= pay_total_) {
                full_reset_();
                return;
            }
            if (ensure_holo_tensor_ready_() != HTS_Holo_Tensor_4D_TX::SECURE_TRUE) {
                holo_tensor_decode_failed_ = true;
            } else {
                (void)holo_rx_.Set_Time_Slot(rx_seq_);
#if !defined(HTS_HOLO_RX_PHASE_B)
#if !defined(HTS_USE_4D_DIVERSITY)
                int16_t rx_soft[HOLO_CHIP_COUNT];
#endif
#endif
#if defined(HTS_PHASE_H_DIAG)
                static uint32_t s_tensor_rx_pkt_diag = 0u;
                const bool force_diag = (g_phase_h_diag_force != 0) &&
                                        (g_phase_h_diag_seed == seed_);
                const bool diag_this_pkt = force_diag || (s_tensor_rx_pkt_diag == 0u);
                if (diag_this_pkt && sym_idx_ == 0) {
                    std::printf(
                        "[PHASE-H][RX] rx_seq=%u profile(K,N)=(%u,%u) valid_mask=0x%016llX\n",
                        static_cast<unsigned>(rx_seq_),
                        static_cast<unsigned>(holo_tensor_profile_.block_bits),
                        static_cast<unsigned>(holo_tensor_profile_.chip_count),
                        static_cast<unsigned long long>(0xFFFFFFFFFFFFFFFFull));
                    std::printf("[PHASE-H][RX] post-V5a chips first16:");
                    for (int i = 0; i < 16; ++i) {
                        std::printf(" (%d,%d)", static_cast<int>(buf_I_[i]),
                                    static_cast<int>(buf_Q_[i]));
                    }
                    std::printf("\n");
                    std::printf("[PHASE-H][RX] post-V5a chips last16:");
                    for (int i = 48; i < 64; ++i) {
                        std::printf(" (%d,%d)", static_cast<int>(buf_I_[i]),
                                    static_cast<int>(buf_Q_[i]));
                    }
                    std::printf("\n");
                }
#else
                const bool diag_this_pkt = false;
#endif
#if defined(HTS_HOLO_RX_PHASE_B)
                if (diag_this_pkt && sym_idx_ == 0) {
                    std::printf("[PHASE-H][RX] decode_input_IQ64 (Phase-B path)\n");
                }
#if defined(HTS_HOLO_RX_PHASE_REF) && defined(HTS_HOLO_RX_PHASE_REF_APPLY)
                // Rev2: ref 회전+Decode_Block 은 블라인드(PS-LTE) 페이로드에만.
                // 홀로 Sync 경로는 PRE_SYM1 ref 와 V5a payload 위상 정렬이 달라
                // Two_Candidates 유지(S5H 저역 회귀 방지).
                uint32_t dec_ok = HTS_Holo_Tensor_4D_RX::SECURE_FALSE;
                if (holo_rx_phase_ref_valid_ && !use_holographic_sync_) {
                    const uint16_t Nc = holo_tensor_profile_.chip_count;
                    const uint16_t n_rot =
                        (Nc > HOLO_CHIP_COUNT) ? HOLO_CHIP_COUNT : Nc;
                    const uint16_t ref_lut = static_cast<uint16_t>(
                        static_cast<uint32_t>(holo_rx_phase_ref_q16_));
                    const int32_t cos_ref = static_cast<int32_t>(Cos_Q15(ref_lut));
                    const int32_t sin_ref = static_cast<int32_t>(Sin_Q15(ref_lut));
                    for (uint16_t c = 0u; c < n_rot; ++c) {
                        const int32_t I = static_cast<int32_t>(buf_I_[c]);
                        const int32_t Q = static_cast<int32_t>(buf_Q_[c]);
                        int32_t rot_i = (I * cos_ref + Q * sin_ref) >> 15;
                        int32_t rot_q = (Q * cos_ref - I * sin_ref) >> 15;
                        int32_t s = (rot_i + rot_q) / 2;
                        if (s > 32767) {
                            s = 32767;
                        } else if (s < -32768) {
                            s = -32768;
                        }
                        rx_soft_rotated_[c] = static_cast<int16_t>(s);
                    }
                    dec_ok = holo_rx_.Decode_Block(
                        rx_soft_rotated_, Nc, 0xFFFFFFFFFFFFFFFFull,
                        holo_tensor_rx_bits_,
                        holo_tensor_profile_.block_bits);
                } else {
#if defined(HTS_USE_4D_DIVERSITY)
                    dec_ok = holo_tensor_decode_4d_a2_prime_(
                        holo_rx_, rx_seq_, buf_I_, buf_Q_,
                        holo_tensor_profile_.chip_count,
                        holo_tensor_profile_.block_bits,
                        holo_tensor_sym_bits0_, holo_tensor_sym_bits1_);
#else
                    dec_ok = holo_rx_.Decode_Block_Two_Candidates(
                        buf_I_, buf_Q_,
                        holo_tensor_profile_.chip_count, 0xFFFFFFFFFFFFFFFFull,
                        holo_tensor_sym_bits0_, holo_tensor_sym_bits1_,
                        holo_tensor_profile_.block_bits);
#endif
                }
#else
#if defined(HTS_USE_4D_DIVERSITY)
                const uint32_t dec_ok = holo_tensor_decode_4d_a2_prime_(
                    holo_rx_, rx_seq_, buf_I_, buf_Q_,
                    holo_tensor_profile_.chip_count,
                    holo_tensor_profile_.block_bits,
                    holo_tensor_sym_bits0_, holo_tensor_sym_bits1_);
#else
                const uint32_t dec_ok = holo_rx_.Decode_Block_Two_Candidates(
                    buf_I_, buf_Q_,
                    holo_tensor_profile_.chip_count, 0xFFFFFFFFFFFFFFFFull,
                    holo_tensor_sym_bits0_, holo_tensor_sym_bits1_,
                    holo_tensor_profile_.block_bits);
#endif
#endif
#else
#if defined(HTS_USE_4D_DIVERSITY)
                const uint32_t dec_ok = holo_tensor_decode_4d_a2_prime_(
                    holo_rx_, rx_seq_, buf_I_, buf_Q_,
                    holo_tensor_profile_.chip_count,
                    holo_tensor_profile_.block_bits,
                    holo_tensor_sym_bits0_, holo_tensor_sym_bits1_);
#else
                for (int c = 0; c < HOLO_CHIP_COUNT; ++c) {
                    const int32_t sum = static_cast<int32_t>(buf_I_[c]) +
                                        static_cast<int32_t>(buf_Q_[c]);
                    rx_soft[c] = static_cast<int16_t>(sum / 2);
                }
                if (diag_this_pkt && sym_idx_ == 0) {
                    std::printf("[PHASE-H][RX] decode_input_rx_soft64:");
                    for (int i = 0; i < 64; ++i) {
                        std::printf(" %d", static_cast<int>(rx_soft[i]));
                    }
                    std::printf("\n");
                }
#if defined(HTS_HOLO_RX_PHASE_REF) && defined(HTS_HOLO_RX_PHASE_REF_APPLY)
                // Phase-B 와 동일: 홀로 Sync 시 ref 회전 미적용.
                uint32_t dec_ok = HTS_Holo_Tensor_4D_RX::SECURE_FALSE;
                if (holo_rx_phase_ref_valid_ && !use_holographic_sync_) {
                    const uint16_t Nc = holo_tensor_profile_.chip_count;
                    const uint16_t n_rot =
                        (Nc > HOLO_CHIP_COUNT) ? HOLO_CHIP_COUNT : Nc;
                    const uint16_t ref_lut = static_cast<uint16_t>(
                        static_cast<uint32_t>(holo_rx_phase_ref_q16_));
                    const int32_t cos_ref = static_cast<int32_t>(Cos_Q15(ref_lut));
                    const int32_t sin_ref = static_cast<int32_t>(Sin_Q15(ref_lut));
                    for (uint16_t c = 0u; c < n_rot; ++c) {
                        const int32_t I = static_cast<int32_t>(buf_I_[c]);
                        const int32_t Q = static_cast<int32_t>(buf_Q_[c]);
                        int32_t rot_i = (I * cos_ref + Q * sin_ref) >> 15;
                        int32_t rot_q = (Q * cos_ref - I * sin_ref) >> 15;
                        int32_t s = (rot_i + rot_q) / 2;
                        if (s > 32767) {
                            s = 32767;
                        } else if (s < -32768) {
                            s = -32768;
                        }
                        rx_soft_rotated_[c] = static_cast<int16_t>(s);
                    }
                    dec_ok = holo_rx_.Decode_Block(
                        rx_soft_rotated_, Nc, 0xFFFFFFFFFFFFFFFFull,
                        holo_tensor_rx_bits_,
                        holo_tensor_profile_.block_bits);
                } else {
                    dec_ok = holo_rx_.Decode_Block(
                        rx_soft, holo_tensor_profile_.chip_count,
                        0xFFFFFFFFFFFFFFFFull,
                        holo_tensor_rx_bits_, holo_tensor_profile_.block_bits);
                }
#else
                const uint32_t dec_ok = holo_rx_.Decode_Block(
                    rx_soft, holo_tensor_profile_.chip_count, 0xFFFFFFFFFFFFFFFFull,
                    holo_tensor_rx_bits_, holo_tensor_profile_.block_bits);
#endif
#endif
#endif
                if (dec_ok != HTS_Holo_Tensor_4D_RX::SECURE_TRUE) {
                    holo_tensor_decode_failed_ = true;
                } else if (holo_tensor_rx_len_ < sizeof(holo_tensor_rx_bytes_)) {
#if defined(HTS_USE_4D_DIVERSITY) && !defined(HTS_HOLO_RX_PHASE_B)
                    std::memcpy(
                        holo_tensor_rx_bits_, holo_tensor_sym_bits0_,
                        static_cast<size_t>(holo_tensor_profile_.block_bits) *
                            sizeof(int8_t));
#endif
#if defined(HTS_HOLO_RX_PHASE_B)
#if defined(HTS_HOLO_RX_PHASE_REF) && defined(HTS_HOLO_RX_PHASE_REF_APPLY)
                    if (!(holo_rx_phase_ref_valid_ && !use_holographic_sync_)) {
                        std::memcpy(
                            holo_tensor_rx_bits_, holo_tensor_sym_bits0_,
                            static_cast<size_t>(holo_tensor_profile_.block_bits) *
                                sizeof(int8_t));
                    }
#else
                    std::memcpy(
                        holo_tensor_rx_bits_, holo_tensor_sym_bits0_,
                        static_cast<size_t>(holo_tensor_profile_.block_bits) *
                            sizeof(int8_t));
#endif
#endif
                    if (diag_this_pkt && sym_idx_ == 0) {
                        std::printf("[PHASE-H][RX] decode_output_bits16:");
                        for (int i = 0; i < static_cast<int>(holo_tensor_profile_.block_bits); ++i) {
                            std::printf(" %d", static_cast<int>(holo_tensor_rx_bits_[i]));
                        }
                        std::printf("\n");
                    }
                    const size_t room =
                        sizeof(holo_tensor_rx_bytes_) - holo_tensor_rx_len_;
#if defined(HTS_HOLO_RX_PHASE_B)
#if defined(HTS_HOLO_RX_PHASE_REF) && defined(HTS_HOLO_RX_PHASE_REF_APPLY)
                    if (holo_rx_phase_ref_valid_ && !use_holographic_sync_) {
                        const uint16_t Kblk = holo_tensor_profile_.block_bits;
                        const size_t wr = bpsk_to_bytes_(
                            holo_tensor_rx_bits_, Kblk,
                            &holo_tensor_rx_bytes_[holo_tensor_rx_len_], room);
                        if (wr == 0u) {
                            holo_tensor_decode_failed_ = true;
                        } else {
                            std::memcpy(
                                &holo_tensor_rx_bytes_alt_[holo_tensor_rx_len_],
                                &holo_tensor_rx_bytes_[holo_tensor_rx_len_], wr);
                            holo_tensor_rx_len_ =
                                static_cast<uint8_t>(holo_tensor_rx_len_ + wr);
                        }
                    } else {
                        const uint16_t Kblk = holo_tensor_profile_.block_bits;
                        const size_t wr0 = bpsk_to_bytes_(
                            holo_tensor_sym_bits0_, Kblk,
                            &holo_tensor_rx_bytes_[holo_tensor_rx_len_], room);
                        const size_t wr1 = bpsk_to_bytes_(
                            holo_tensor_sym_bits1_, Kblk,
                            &holo_tensor_rx_bytes_alt_[holo_tensor_rx_len_], room);
                        if (wr0 == 0u || wr0 != wr1) {
                            holo_tensor_decode_failed_ = true;
                        } else {
                            holo_tensor_rx_len_ =
                                static_cast<uint8_t>(holo_tensor_rx_len_ + wr0);
                        }
                    }
#else
                    const uint16_t Kblk = holo_tensor_profile_.block_bits;
                    const size_t wr0 = bpsk_to_bytes_(
                        holo_tensor_sym_bits0_, Kblk,
                        &holo_tensor_rx_bytes_[holo_tensor_rx_len_], room);
                    const size_t wr1 = bpsk_to_bytes_(
                        holo_tensor_sym_bits1_, Kblk,
                        &holo_tensor_rx_bytes_alt_[holo_tensor_rx_len_], room);
                    if (wr0 == 0u || wr0 != wr1) {
                        holo_tensor_decode_failed_ = true;
                    } else {
                        holo_tensor_rx_len_ =
                            static_cast<uint8_t>(holo_tensor_rx_len_ + wr0);
                    }
#endif
#else
                    const size_t wr = bpsk_to_bytes_(
                        holo_tensor_rx_bits_, holo_tensor_profile_.block_bits,
                        &holo_tensor_rx_bytes_[holo_tensor_rx_len_], room);
                    holo_tensor_rx_len_ = static_cast<uint8_t>(holo_tensor_rx_len_ + wr);
#endif
                }
#if defined(HTS_PHASE_H_DIAG)
                if (diag_this_pkt && (pay_recv_ >= pay_total_) && !force_diag) {
                    ++s_tensor_rx_pkt_diag;
                }
#endif
            }
            ++sym_idx_;
            buf_idx_ = 0;
            if (pay_recv_ >= pay_total_) {
                try_decode_();
            }
            return;
        }
#endif
        /* IR-HARQ: RV마다 송신 파형이 다르므로 칩 도메인 += 금지.
           결합은 FEC Decode*_IR 의 ir_state_(LLR)에서만 수행; 수신 칩은 매 심볼
           덮어쓰기. */
        std::memcpy(orig_I_, buf_I_, nc * sizeof(int16_t));
        std::memcpy(orig_Q_, buf_Q_, nc * sizeof(int16_t));
        // [Walsh_Row_Permuter] TX: enc = raw ^ mask 후 walsh_enc(enc).
        //  RX: chip[j] *= (−1)^{popc(mask∧j)} — Walsh-Hadamard 행 XOR 성질로
        //  FEC(Feed*) 가 기대하는 raw 도메인 칩으로 복원 (mask=0 이면 항등).
        if (nc == 16) {
            const uint8_t m4 = walsh_permuter_.Encode_4bit(0u);
            for (int c = 0; c < 16; ++c) {
                const uint32_t pu =
                    popc32(static_cast<uint32_t>(m4) &
                           static_cast<uint32_t>(c)) &
                    1u;
                const int32_t sgn = 1 - 2 * static_cast<int32_t>(pu);
                buf_I_[c] = static_cast<int16_t>(
                    static_cast<int32_t>(buf_I_[c]) * sgn);
                buf_Q_[c] = static_cast<int16_t>(
                    static_cast<int32_t>(buf_Q_[c]) * sgn);
            }
        } else {
            const uint8_t m6 = walsh_permuter_.Encode_6bit(0u);
            for (int c = 0; c < 64; ++c) {
                const uint32_t pu =
                    popc32(static_cast<uint32_t>(m6) &
                           static_cast<uint32_t>(c)) &
                    1u;
                const int32_t sgn = 1 - 2 * static_cast<int32_t>(pu);
                buf_I_[c] = static_cast<int16_t>(
                    static_cast<int32_t>(buf_I_[c]) * sgn);
                buf_Q_[c] = static_cast<int16_t>(
                    static_cast<int32_t>(buf_Q_[c]) * sgn);
            }
        }
        // [REMOVED Step2] cw_cancel_64_(buf_I_, buf_Q_) — NOP 확정
        // [REMOVED Step3] if (ajc_enabled_) ajc_.Process(...) — 학습 구조 불일치로 NOP
        // [REMOVED Step1] soft_clip_iq — Gaussian noise 에 무효 확정
        if (nc == 16) {
            if (sym_idx_ < FEC_HARQ::NSYM16) {
                    if (ir_mode_) {
                    /* IR 칩: ECCM 후 버퍼 저장 (CW 제거 효과 반영) */
                    const int base = sym_idx_ * FEC_HARQ::C16;
                    alignas(16) int32_t wI[16];
                    alignas(16) int32_t wQ[16];
                    for (int c = 0; c < nc; ++c) {
                        wI[c] = static_cast<int32_t>(buf_I_[c]);
                        wQ[c] = static_cast<int32_t>(buf_Q_[c]);
                    }
                    fwht_raw(wI, nc);
                    fwht_raw(wQ, nc);
                    const int32_t pk = fwht_post_peak_max_abs_(wI, wQ, nc);
                    const int sh = fwht_int16_store_downshift_(pk);
#if defined(HTS_DIAG_FWHT_INTERNAL) || defined(HTS_DIAG_AGC_TRACE)
                    fwht_post_shift_hist_record(sh);
#endif
                    for (int c = 0; c < nc; ++c) {
                        ir_chip_I_[base + c] = ssat16_dispatch_(
                            static_cast<int32_t>(buf_I_[c]) >> sh);
                        ir_chip_Q_[base + c] = ssat16_dispatch_(
                            static_cast<int32_t>(buf_Q_[c]) >> sh);
                    }
                } else {
                    FEC_HARQ::Feed16_1sym(rx_.m16, buf_I_, buf_Q_, sym_idx_);
                }
                for (int c = 0; c < nc; ++c) {
                    const uint8_t hiI =
                        static_cast<uint8_t>((orig_I_[c] >> 12) & 0x0Fu);
                    const uint8_t hiQ =
                        static_cast<uint8_t>((orig_Q_[c] >> 12) & 0x0Fu);
                    orig_acc_.acc16.iq4[sym_idx_][c] =
                        static_cast<uint8_t>((hiI << 4u) | hiQ);
                }
                sym_idx_++;
            } else {
                full_reset_();
                return;
            }
        } else {
            const int nsym64 = cur_nsym64_();
            if (iq_mode_ == IQ_Mode::IQ_INDEPENDENT && !ir_mode_) {
                // ── [적응형 I/Q] I/Q 독립 RX: 칩슬롯당 2심볼 ──
                //  I 채널 → 짝수 sym_idx_, Q 채널 → 홀수 sym_idx_
                //  HARQ I 누적기: 짝수 심볼 전용
                //  HARQ Q 누적기: 홀수 심볼 전용
                const int si_I = sym_idx_;     // I 채널 심볼 인덱스
                const int si_Q = sym_idx_ + 1; // Q 채널 심볼 인덱스
                if (si_Q < nsym64) {
                    // I 채널 HARQ 누적 (짝수 심볼)
                    for (int c = 0; c < nc; ++c) {
                        rx_.m64_I.aI[si_I][c] +=
                            static_cast<int32_t>(buf_I_[c]);
                    }
                    // Q 채널 HARQ 누적 (홀수 심볼)
                    for (int c = 0; c < nc; ++c) {
                        rx_.m64_I.aI[si_Q][c] +=
                            static_cast<int32_t>(buf_Q_[c]);
                    }
                    for (int c = 0; c < nc; ++c) {
                        const uint8_t hiI =
                            static_cast<uint8_t>((orig_I_[c] >> 12) & 0x0Fu);
                        orig_acc_.acc64.iq4[si_I][c] =
                            static_cast<uint8_t>(hiI << 4u);
                        const uint8_t hiQ =
                            static_cast<uint8_t>((orig_Q_[c] >> 12) & 0x0Fu);
                        orig_acc_.acc64.iq4[si_Q][c] =
                            static_cast<uint8_t>(hiQ << 4u);
                    }
                    sym_idx_ += 2;
                    pay_recv_++; // 칩슬롯 기준 카운트
                } else {
                    full_reset_();
                    return;
                }
            } else {
                // ── I=Q 동일 또는 IR-HARQ (칩 보관) ──
                if (sym_idx_ < nsym64) {
                    if (ir_mode_) {
                        /* IR 칩: ECCM 후 버퍼 + SIC (CW 제거 효과 반영) */
                        const int base = sym_idx_ * FEC_HARQ::C64;
                        const uint32_t use_sic_u =
                            static_cast<uint32_t>(sic_expect_valid_) & 1u;
                        // SIC: 1비트 부호 비트맵에서 Walsh 칩 복원 (I=Q 동일)
                        const uint64_t sic_sym_bits =
                            g_sic_bits[static_cast<std::size_t>(sym_idx_)];
                        const int32_t sic_amp =
                            static_cast<int32_t>(sic_walsh_amp_);
                        {
                            // Derotation 제거: FEC_HARQ::Decode64_IR이 위상 불변
                            // 디코딩(I²+Q² 에너지 기반)을 수행하므로 Dispatcher
                            // 레벨 derotation은 불필요하며, RF 환경에서 est 벡터의
                            // 노이즈로 인해 스케일 왜곡을 유발하여 FEC 실패 원인이 됨.
                            // T9(FEC 직접 호출)가 99/100 성공하는 것이 증명.
                            alignas(16) int32_t preI[64];
                            alignas(16) int32_t preQ[64];
                            for (int c = 0; c < nc; ++c) {
                                int32_t vi = static_cast<int32_t>(buf_I_[c]);
                                int32_t vq = static_cast<int32_t>(buf_Q_[c]);
                                const uint32_t neg = static_cast<uint32_t>(
                                    (sic_sym_bits >>
                                     static_cast<uint32_t>(c)) & 1u);
                                const int32_t sic_chip =
                                    sic_amp - 2 * sic_amp *
                                    static_cast<int32_t>(neg);
                                const int32_t sub =
                                    sic_chip *
                                    static_cast<int32_t>(use_sic_u);
                                vi -= sub;
                                vq -= sub;
                                preI[c] = vi;
                                preQ[c] = vq;
                            }
                            alignas(16) int32_t wI[64];
                            alignas(16) int32_t wQ[64];
                            for (int c = 0; c < nc; ++c) {
                                wI[c] = preI[c];
                                wQ[c] = preQ[c];
                            }
                            fwht_raw(wI, nc);
                            fwht_raw(wQ, nc);
                            const int32_t pk =
                                fwht_post_peak_max_abs_(wI, wQ, nc);
                            const int sh = fwht_int16_store_downshift_(pk);
#if defined(HTS_DIAG_FWHT_INTERNAL) || defined(HTS_DIAG_AGC_TRACE)
                            fwht_post_shift_hist_record(sh);
#endif
                            for (int c = 0; c < nc; ++c) {
                                ir_chip_I_[base + c] =
                                    ssat16_dispatch_(preI[c] >> sh);
                                ir_chip_Q_[base + c] =
                                    ssat16_dispatch_(preQ[c] >> sh);
                            }
                        }
                        for (int c = 0; c < nc; ++c) {
                            const uint8_t hiI = static_cast<uint8_t>(
                                (orig_I_[c] >> 12) & 0x0Fu);
                            const uint8_t hiQ = static_cast<uint8_t>(
                                (orig_Q_[c] >> 12) & 0x0Fu);
                            orig_acc_.acc64.iq4[sym_idx_][c] =
                                static_cast<uint8_t>((hiI << 4u) | hiQ);
                        }
                        sym_idx_++;
                    } else {
                        // Derotation 제거 (위 IR 경로 주석 참조)
                        for (int c = 0; c < nc; ++c) {
                            rx_.m64_I.aI[sym_idx_][c] +=
                                static_cast<int32_t>(buf_I_[c]);
                        }
                        for (int c = 0; c < nc; ++c) {
                            harq_Q_[sym_idx_][c] +=
                                static_cast<int32_t>(buf_Q_[c]);
                        }
                        for (int c = 0; c < nc; ++c) {
                            const uint8_t hiI = static_cast<uint8_t>(
                                (orig_I_[c] >> 12) & 0x0Fu);
                            const uint8_t hiQ = static_cast<uint8_t>(
                                (orig_Q_[c] >> 12) & 0x0Fu);
                            orig_acc_.acc64.iq4[sym_idx_][c] =
                                static_cast<uint8_t>((hiI << 4u) | hiQ);
                        }
                        sym_idx_++;
                    }
                } else {
                    full_reset_();
                    return;
                }
            }
        }
        // [REMOVED Step3] AJC walsh 피드백 블록 (Update_AJC × N) — 구조적 NOP
    }
    buf_idx_ = 0;
    if (pay_recv_ >= pay_total_)
        try_decode_();
}
void HTS_V400_Dispatcher::try_decode_() noexcept {
    // [BUG-FIX-IR5] wb_ 초기화: Encode 잔류 데이터가 Decode 경로를 오염시키는
    // 것을 방지. 16칩(NSYM16=172)은 wb_ 사용 영역이 작아 영향 없으나,
    // 64칩(NSYM64=172, BPS=4)은 전체 TOTAL_CODED(688) 슬롯을 사용하므로 필수.
    std::memset(&wb_, 0, sizeof(wb_));
    const uint8_t walsh_shift_payload = current_walsh_shift_();
    DecodedPacket pkt = {};
    pkt.mode = cur_mode_;
    pkt.success_mask = DecodedPacket::DECODE_MASK_FAIL;
    uint32_t il = seed_ ^ (rx_seq_ * 0xA5A5A5A5u);
    /* WorkBuf: try_decode_ 진입 시 memset 초기화 완료 (BUG-FIX-IR5). */
    if (cur_mode_ == PayloadMode::VIDEO_1) {
        pkt.success_mask =
            static_cast<uint32_t>(0u - static_cast<uint32_t>(FEC_HARQ::Decode1(
                                           v1_rx_, pkt.data, &pkt.data_len)));
        pkt.harq_k = 1;
        handle_video_(pkt.success_mask);
        if (pkt.success_mask != 0u && pkt.data_len >= 8) {
            tpc_rx_feedback_after_decode_(pkt);
        }
        if (on_pkt_ != nullptr) {
            on_pkt_(pkt);
        }
        rx_seq_++;
#if defined(HTS_HARQ_DIAG)
        HARQ_Diag::note_packet_finished(
            1u, static_cast<uint32_t>(max_harq_),
            static_cast<uint32_t>(pkt.success_mask != 0u));
#endif
        full_reset_();
    } else if (cur_mode_ == PayloadMode::VIDEO_16 ||
               cur_mode_ == PayloadMode::VOICE) {
        if (ir_mode_) {
            harq_round_++;
            const int rv = ir_rv_;
#if defined(HTS_IR_DIAG_ENABLE)
            if (g_hts_ir_diag_chip0 != 0 && ir_chip_I_ != nullptr &&
                ir_state_ != nullptr) {
                std::printf("[IR-DIAG] pre-Decode16_IR feed=%d harq_round_=%d "
                            "ir_chip_I_[0]=%d ir_state.rounds_done=%d\n",
                            static_cast<int>(g_hts_ir_diag_feed_idx),
                            harq_round_, static_cast<int>(ir_chip_I_[0]),
                            ir_state_->rounds_done);
            }
#endif
            const bool ir16_ok = (ir_state_ != nullptr &&
                                  ir_chip_I_ != nullptr &&
                                  ir_chip_Q_ != nullptr);
            pkt.success_mask = static_cast<uint32_t>(
                0u -
                static_cast<uint32_t>(
                    ir16_ok &&
                    FEC_HARQ::Decode16_IR(
                        ir_chip_I_, ir_chip_Q_, FEC_HARQ::NSYM16,
                        FEC_HARQ::C16, FEC_HARQ::BPS16, il, rv, *ir_state_,
                        pkt.data, &pkt.data_len, wb_, walsh_shift_payload)));
#if defined(HTS_DIAG_PRINTF)
            if (ir16_ok) {
                std::printf(
                    "[PAYLOAD-SHIFT] walsh_shift=%u dom=%u\n",
                    static_cast<unsigned>(walsh_shift_payload),
                    static_cast<unsigned>(dominant_row_));
            }
#endif
            ir_rv_ = (ir_rv_ + 1) & 3;
            sic_expect_valid_ = false;
        } else {
            FEC_HARQ::Advance_Round_16(rx_.m16);
            harq_round_++;
            pkt.success_mask = static_cast<uint32_t>(
                0u - static_cast<uint32_t>(FEC_HARQ::Decode16(
                         rx_.m16, pkt.data, &pkt.data_len, il, wb_)));
        }
        pkt.harq_k = harq_round_;
        const uint32_t dec_ok = static_cast<uint32_t>(pkt.success_mask != 0u);
        const uint32_t harq_ex =
            static_cast<uint32_t>(harq_round_ >= max_harq_);
        // 연속모드에서는 harq 소진을 하네스 외부(feeds 루프)에서 관리
        // max_harq_는 DATA_K(800)로 통일
        const uint32_t finish = dec_ok | harq_ex;
        if (finish != 0u) {
#if defined(HTS_HARQ_DIAG)
            HARQ_Diag::note_packet_finished(
                static_cast<uint32_t>(harq_round_),
                static_cast<uint32_t>(max_harq_), dec_ok);
#endif
            if (dec_ok != 0u) {
                harq_feedback_seed_(pkt.data, pkt.data_len, 16, il);
            }
            if (cur_mode_ == PayloadMode::VIDEO_16) {
                handle_video_(pkt.success_mask);
            }
            if (pkt.success_mask != 0u && pkt.data_len >= 8) {
                tpc_rx_feedback_after_decode_(pkt);
            }
            if (on_pkt_ != nullptr) {
                on_pkt_(pkt);
            }
            rx_seq_++;
            full_reset_();
        } else {
            pay_recv_ = 0;
            sym_idx_ = 0;
            /* IR 칩: on_sym_ 라운드마다 덮어쓰기 — memset 불필요 */
            /* BUG-FIX-RETX5: 실패 시 READ_PAYLOAD 유지, 재동기 회피 */
            retx_ready_ = true;
            buf_idx_ = 0;
            // set_phase_(RxPhase::WAIT_SYNC);
        }
    } else if (cur_mode_ == PayloadMode::DATA) {
#if defined(HTS_USE_HOLO_TENSOR_4D)
        if (holo_tensor_payload_mode_) {
            pkt.harq_k = 1;
#if defined(HTS_HOLO_RX_PHASE_B)
            const uint32_t ok = static_cast<uint32_t>(
                (holo_tensor_decode_failed_ == false) &
                (holo_tensor_rx_len_ >= 8u));
            pkt.success_mask = static_cast<uint32_t>(0u - ok);
            if (ok != 0u) {
                const bool crc_a = holo_tensor_packet_crc16_ok_8(holo_tensor_rx_bytes_);
                const bool crc_b = holo_tensor_packet_crc16_ok_8(holo_tensor_rx_bytes_alt_);
                if (crc_b && !crc_a) {
                    std::memcpy(pkt.data, holo_tensor_rx_bytes_alt_, 8u);
                } else {
                    std::memcpy(pkt.data, holo_tensor_rx_bytes_, 8u);
                }
                pkt.data_len = 8;
            } else {
                pkt.data_len = 0;
            }
#else
            const uint32_t ok = static_cast<uint32_t>(
                (holo_tensor_decode_failed_ == false) &
                (holo_tensor_rx_len_ >= 8u));
            pkt.success_mask = static_cast<uint32_t>(0u - ok);
            if (ok != 0u) {
                std::memcpy(pkt.data, holo_tensor_rx_bytes_, 8u);
                pkt.data_len = 8;
            } else {
                pkt.data_len = 0;
            }
#endif
            if (on_pkt_ != nullptr) {
                on_pkt_(pkt);
            }
            rx_seq_++;
            full_reset_();
            return;
        }
#endif
        harq_round_++;
        if (ir_mode_) {
                const int bps = cur_bps64_;
                if (bps >= FEC_HARQ::BPS64_MIN_OPERABLE &&
                    bps <= FEC_HARQ::BPS64_MAX && ir_state_ != nullptr &&
                    ir_chip_I_ != nullptr && ir_chip_Q_ != nullptr) {
                    const int nsym_ir = FEC_HARQ::nsym_for_bps(bps);
                    const int rv = ir_rv_;
#if defined(HTS_IR_DIAG_ENABLE)
                    if (g_hts_ir_diag_chip0 != 0) {
                        std::printf(
                            "[IR-DIAG] pre-Decode64_IR feed=%d harq_round_=%d "
                            "ir_chip_I_[0]=%d ir_state.rounds_done=%d\n",
                            static_cast<int>(g_hts_ir_diag_feed_idx),
                            harq_round_, static_cast<int>(ir_chip_I_[0]),
                            ir_state_->rounds_done);
                        std::printf(
                            "[IR-DIAG] Decode64_IR args: bps=%d nsym_ir=%d "
                            "il=0x%08x rv=%d rounds_done=%d\n",
                            bps, nsym_ir, static_cast<unsigned>(il), rv,
                            ir_state_->rounds_done);
                    }
#endif
                    pkt.success_mask = static_cast<uint32_t>(
                        0u -
                        static_cast<uint32_t>(FEC_HARQ::Decode64_IR(
                            ir_chip_I_, ir_chip_Q_, nsym_ir, FEC_HARQ::C64, bps,
                            il, rv, *ir_state_, pkt.data, &pkt.data_len, wb_,
                            walsh_shift_payload)));
#if defined(HTS_DIAG_PRINTF)
                    std::printf(
                        "[PAYLOAD-SHIFT] walsh_shift=%u dom=%u\n",
                        static_cast<unsigned>(walsh_shift_payload),
                        static_cast<unsigned>(dominant_row_));
#endif
                }
                ir_rv_ = (ir_rv_ + 1) & 3;
                {
                    const int bps_sic = cur_bps64_;
                    const bool ir64_attempt =
                        (bps_sic >= FEC_HARQ::BPS64_MIN_OPERABLE &&
                         bps_sic <= FEC_HARQ::BPS64_MAX &&
                         ir_state_ != nullptr && ir_chip_I_ != nullptr &&
                         ir_chip_Q_ != nullptr);
                    if (ir64_attempt) {
                        if (pkt.success_mask != 0u) {
                            sic_expect_valid_ = false;
                        } else if (sic_ir_enabled_) {
                            fill_sic_expected_64_();
                        } else {
                            sic_expect_valid_ = false;
                        }
                    } else {
                        sic_expect_valid_ = false;
                    }
                }
            } else {
                sic_expect_valid_ = false;
                if (!rx_.m64_I.ok)
                    rx_.m64_I.k++;
                {
                    const int bps = cur_bps64_;
                    if (bps >= FEC_HARQ::BPS64_MIN_OPERABLE &&
                        bps <= FEC_HARQ::BPS64_MAX) {
                        const int nsym = FEC_HARQ::nsym_for_bps(bps);
                        pkt.success_mask = static_cast<uint32_t>(
                            0u -
                            static_cast<uint32_t>(FEC_HARQ::Decode_Core_Split(
                                &rx_.m64_I.aI[0][0], harq_Q_[0], nsym,
                                FEC_HARQ::C64, bps, pkt.data, &pkt.data_len, il,
                                wb_)));
                    }
                }
            }
        pkt.harq_k = harq_round_;
        const uint32_t dec_ok = static_cast<uint32_t>(pkt.success_mask != 0u);
        const uint32_t harq_ex =
            static_cast<uint32_t>(harq_round_ >= max_harq_);
        const uint32_t finish = dec_ok | harq_ex;
        if (finish != 0u) {
#if defined(HTS_HARQ_DIAG)
            HARQ_Diag::note_packet_finished(
                static_cast<uint32_t>(harq_round_),
                static_cast<uint32_t>(max_harq_), dec_ok);
#endif
            if (dec_ok != 0u) {
                if (!ir_mode_) {
                    rx_.m64_I.ok = true;
                }
                harq_feedback_seed_(pkt.data, pkt.data_len, 64, il);
            }
            if (pkt.success_mask != 0u && pkt.data_len >= 8) {
                tpc_rx_feedback_after_decode_(pkt);
            }
            if (on_pkt_ != nullptr) {
                on_pkt_(pkt);
            }
            rx_seq_++;
            full_reset_();
        } else {
            pay_recv_ = 0;
            sym_idx_ = 0;
            /* IR 칩: on_sym_ 라운드마다 덮어쓰기 — memset 불필요 */
            /* BUG-FIX-RETX5: 실패 시 READ_PAYLOAD 유지, 재동기 회피 */
            retx_ready_ = true;
            buf_idx_ = 0;
            // set_phase_(RxPhase::WAIT_SYNC);
        }
    }
}
void HTS_V400_Dispatcher::harq_feedback_seed_(const uint8_t *data, int data_len,
                                              int nc, uint32_t il) noexcept {
    if (!data || data_len <= 0)
        return;
    if (nc == 16) {
        uint8_t *const correct_syms = g_v400_sym_scratch;
        int enc_n = 0;
        if (ir_mode_) {
            const int rv_fb = (ir_rv_ + 3) & 3;
            enc_n = FEC_HARQ::Encode16_IR(data, data_len, correct_syms, il,
                                          rv_fb, wb_);
        } else {
            enc_n = FEC_HARQ::Encode16(data, data_len, correct_syms, il, wb_);
        }
        if (enc_n <= 0) {
            SecureMemory::secureWipe(static_cast<void *>(correct_syms),
                                     sizeof(g_v400_sym_scratch));
            return;
        }
        // [REMOVED Step3] HARQ 피드백 루프의 AJC Update_AJC — AJC 제거
        SecureMemory::secureWipe(static_cast<void *>(correct_syms),
                                 sizeof(g_v400_sym_scratch));
    } else if (nc == 64) {
        uint8_t *const correct_syms = g_v400_sym_scratch;
        int enc_n = 0;
        if (ir_mode_) {
            // try_decode_: Decode64_IR 직후 ir_rv_ 가 +1 되므로
            // 피드백용 직전 라운드 RV ≡ (ir_rv_ + 3) & 3
            const int rv_fb = (ir_rv_ + 3) & 3;
            enc_n = FEC_HARQ::Encode64_IR(data, data_len, correct_syms, il,
                                          cur_bps64_, rv_fb, wb_);
        } else {
            enc_n = FEC_HARQ::Encode64_A(data, data_len, correct_syms, il,
                                         cur_bps64_, wb_);
        }
        if (enc_n <= 0) {
            SecureMemory::secureWipe(static_cast<void *>(correct_syms),
                                     sizeof(g_v400_sym_scratch));
            return;
        }
        // [REMOVED Step3] HARQ 피드백 루프의 AJC Update_AJC — AJC 제거
        SecureMemory::secureWipe(static_cast<void *>(correct_syms),
                                 sizeof(g_v400_sym_scratch));
    }
}
void HTS_V400_Dispatcher::handle_video_(uint32_t decode_ok_mask) noexcept {
    const uint32_t ok = decode_ok_mask & 1u;
    if (ok != 0u) {
        vid_succ_++;
        vid_fail_ = 0;
        if (active_video_ == PayloadMode::VIDEO_16 &&
            vid_succ_ >= VIDEO_RECOVER_TH) {
            active_video_ = PayloadMode::VIDEO_1;
            vid_succ_ = 0;
            if (on_ctrl_ != nullptr) {
                on_ctrl_(PayloadMode::VIDEO_1);
            }
        }
    } else {
        vid_fail_++;
        vid_succ_ = 0;
        if (active_video_ == PayloadMode::VIDEO_1 &&
            vid_fail_ >= VIDEO_FAIL_TH) {
            active_video_ = PayloadMode::VIDEO_16;
            vid_fail_ = 0;
            if (on_ctrl_ != nullptr) {
                on_ctrl_(PayloadMode::VIDEO_16);
            }
        }
    }
}
/* BUG-FIX-RETX4: HARQ 연속모드 RX */
void HTS_V400_Dispatcher::Feed_Retx_Chip(int16_t rx_I, int16_t rx_Q) noexcept {
    if (phase_ == RxPhase::RF_SETTLING) {
        return;
    }
    if (!retx_ready_ || phase_ != RxPhase::READ_PAYLOAD)
        return;
    if (buf_idx_ >= 64)
        return;
    if (buf_idx_ == 0) {
        holo_lpi_rx_chip_idx_ = 0u;
        holo_lpi_rx_scalars_seq_ = 0xFFFFFFFFu;
        // [Walsh_Row_Permuter] HARQ 재전송 슬롯 — Build_Retx 의 (tx_seq_prev, ir_rv_)
        // 와 동일 (tx_seq_prev == 미완료 PDU 에 대한 rx_seq_)
        if (walsh_permuter_.Is_Initialized()) {
            (void)walsh_permuter_.Update_Key(
                rx_seq_, static_cast<uint8_t>(ir_rv_ & 0xFF));
        }
    }
    int16_t ti = rx_I;
    int16_t tq = rx_Q;
    const uint32_t retx_slot =
        (rx_seq_ > 0u) ? (rx_seq_ - 1u) : 0u;
    apply_holo_lpi_inverse_rx_chip_(ti, tq, retx_slot);
    buf_I_[buf_idx_] = ti;
    buf_Q_[buf_idx_] = tq;
    buf_idx_++;
    if (buf_idx_ >= pay_cps_)
        on_sym_();
}
void HTS_V400_Dispatcher::Inject_Payload_Phase(PayloadMode mode,
                                               int bps) noexcept {
    // 동기/헤더 단계를 건너뛰고 READ_PAYLOAD로 직접 진입
    // IR 상태 초기화 포함
    full_reset_();

    cur_mode_ = mode;
    if (mode == PayloadMode::DATA) {
        cur_bps64_ = FEC_HARQ::bps_clamp_runtime(bps);
        pay_cps_ = 64;
#if defined(HTS_USE_HOLO_TENSOR_4D)
        if (holo_tensor_payload_mode_) {
            const int k_bits = static_cast<int>(k_holo_profiles[1].block_bits);
            pay_total_ = (64 + k_bits - 1) / k_bits;
        } else {
            pay_total_ = FEC_HARQ::nsym_for_bps(cur_bps64_);
        }
#else
        pay_total_ = FEC_HARQ::nsym_for_bps(cur_bps64_);
#endif
    } else if (mode == PayloadMode::VOICE || mode == PayloadMode::VIDEO_16) {
        pay_cps_ = 16;
        pay_total_ = FEC_HARQ::NSYM16;
    } else {
        return; // VIDEO_1은 미지원
    }

    pay_recv_ = 0;
    sym_idx_ = 0;
    max_harq_ = FEC_HARQ::DATA_K; // 800
    phase_ = RxPhase::READ_PAYLOAD;
    buf_idx_ = 0;

    // HARQ/IR 초기화
    SecureMemory::secureWipe(static_cast<void *>(&g_harq_ccm_union),
                             sizeof(g_harq_ccm_union));
    if (ir_mode_ && ir_state_ != nullptr) {
        FEC_HARQ::IR_Init(*ir_state_);
    }
    ir_rv_ = 0;
    harq_inited_ = true;
    retx_ready_ = false;

    // [Walsh_Row_Permuter] 동기/헤더 우회 시험 경로 — 일반 RX 와 동일 키
    if (!walsh_permuter_.Is_Initialized()) {
        (void)walsh_permuter_.Initialize(rx_seq_, 0u);
    } else {
        (void)walsh_permuter_.Update_Key(rx_seq_, 0u);
    }

    // [REMOVED Step3] pay_cps_ 변경 시 ajc_.Reset — AJC 제거
}
} // namespace ProtectedEngine
#if defined(HTS_DIAG_FWHT_INTERNAL) || defined(HTS_DIAG_AGC_TRACE)
namespace HtsFwhtInt16Scale {
std::atomic<uint32_t> g_post_shift_hist[8]{};
} // namespace HtsFwhtInt16Scale
#endif
