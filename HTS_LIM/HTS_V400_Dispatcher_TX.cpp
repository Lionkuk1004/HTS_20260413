// =============================================================================
// HTS_V400_Dispatcher_TX.cpp — Build_Packet / Build_Retx
// =============================================================================
#include "HTS_V400_Dispatcher.hpp"
#include "HTS_V400_Dispatcher_Internal.hpp"
#include "HTS_Holo_LPI.h"
#include "HTS_Secure_Memory.h"
#if defined(HTS_USE_HOLO_TENSOR_4D)
#include "HTS_Holo_Tensor_4D_TX.h"
#include "HTS_Holo_Tensor_4D_Defs.h"
#endif
#if defined(HTS_HOLO_PREAMBLE)
#include "HTS_Holo_Tensor_4D_Defs.h"
#endif
#if defined(HTS_USE_PN_MASKED)
#include "HTS_V400_Dispatcher_PNMasked.hpp"
#endif
#include <cstdint>
#include <cstddef>
namespace ProtectedEngine {
using namespace detail;
namespace {
alignas(64) static int16_t g_bp_sink_i[64];
alignas(64) static int16_t g_bp_sink_q[64];
#if defined(HTS_USE_HOLO_TENSOR_4D)
static uint16_t bytes_to_bpsk_bits_tx_(const uint8_t* bytes, size_t byte_len,
                                       int8_t* bpsk_bits,
                                       uint16_t max_bits) noexcept {
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
#endif
static inline int16_t *bp_dst_i(int16_t *oI, int pos,
                                std::uintptr_t okm) noexcept {
    const std::uintptr_t p = reinterpret_cast<std::uintptr_t>(&oI[pos]);
    return reinterpret_cast<int16_t *>(
        (p & okm) |
        (reinterpret_cast<std::uintptr_t>(g_bp_sink_i) & ~okm));
}
static inline int16_t *bp_dst_q(int16_t *oQ, int pos,
                                std::uintptr_t okm) noexcept {
    const std::uintptr_t p = reinterpret_cast<std::uintptr_t>(&oQ[pos]);
    return reinterpret_cast<int16_t *>(
        (p & okm) |
        (reinterpret_cast<std::uintptr_t>(g_bp_sink_q) & ~okm));
}
} // namespace

#if defined(HTS_HOLO_PREAMBLE)
void HTS_V400_Dispatcher::generate_holo_preamble_(
    int16_t* const out_I, const int16_t amp, const uint32_t seed[4],
    const uint32_t slot) noexcept {
    auto mix = [](const uint32_t z0) noexcept -> uint32_t {
        uint32_t z = z0;
        z = (z ^ (z >> 16u)) * 0x45D9F3Bu;
        z = (z ^ (z >> 16u)) * 0x45D9F3Bu;
        return z ^ (z >> 16u);
    };
    Xoshiro128ss rng{};
    rng.s[0] = mix(seed[0] ^ slot);
    rng.s[1] = mix(seed[1] ^ (slot * 0x9E3779B9u));
    rng.s[2] = mix(seed[2] ^ (slot * 0x517CC1B7u));
    rng.s[3] = mix(seed[3] ^ (slot * 0x6C62272Eu));
    rng.s[0] ^= 0x50524500u;
    for (int w = 0; w < 8; ++w) {
        (void)rng.Next();
    }

    int32_t buf[64];
    for (int i = 0; i < 64; ++i) {
        buf[i] = (rng.Next() & 1u) ? 1 : -1;
    }
    fwht_raw(buf, 64);

    static constexpr uint8_t k_holo_tx_perm24[24][4] = {
        {0, 1, 2, 3}, {0, 1, 3, 2}, {0, 2, 1, 3}, {0, 2, 3, 1}, {0, 3, 1, 2},
        {0, 3, 2, 1}, {1, 0, 2, 3}, {1, 0, 3, 2}, {1, 2, 0, 3}, {1, 2, 3, 0},
        {1, 3, 0, 2}, {1, 3, 2, 0}, {2, 0, 1, 3}, {2, 0, 3, 1}, {2, 1, 0, 3},
        {2, 1, 3, 0}, {2, 3, 0, 1}, {2, 3, 1, 0}, {3, 0, 1, 2}, {3, 0, 2, 1},
        {3, 1, 0, 2}, {3, 1, 2, 0}, {3, 2, 0, 1}, {3, 2, 1, 0}};
    for (int i = 0; i < 64; i += 4) {
        const uint32_t gyro = rng.Next();
        for (int j = 0; j < 4; ++j) {
            if ((gyro >> static_cast<uint32_t>(j)) & 1u) {
                buf[i + j] = -buf[i + j];
            }
        }
        const uint8_t pi =
            static_cast<uint8_t>(((gyro >> 4u) & 0x1Fu) % 24u);
        const uint8_t* p = k_holo_tx_perm24[pi];
        const int32_t v0 = buf[i + p[0]];
        const int32_t v1 = buf[i + p[1]];
        const int32_t v2 = buf[i + p[2]];
        const int32_t v3 = buf[i + p[3]];
        buf[i + 0] = v0 + v1 + v2 + v3;
        buf[i + 1] = v0 - v1 + v2 - v3;
        buf[i + 2] = v0 + v1 - v2 - v3;
        buf[i + 3] = v0 - v1 - v2 + v3;
    }
    fwht_raw(buf, 64);

    for (int i = 0; i < 64; ++i) {
        const int32_t b = buf[i];
        out_I[i] = (b >= 0)
                       ? amp
                       : static_cast<int16_t>(-static_cast<int32_t>(amp));
    }
}
#endif

int HTS_V400_Dispatcher::Build_Packet(PayloadMode mode, const uint8_t *info,
                                      int ilen, int16_t amp, int16_t *oI,
                                      int16_t *oQ, int max_c) noexcept {
    if (phase_ == RxPhase::RF_SETTLING) {
        return 0;
    }
    if (info == nullptr || oI == nullptr || oQ == nullptr)
        return 0;
    if (ilen < 0 || max_c <= 0)
        return 0;
    // [Walsh_Row_Permuter] payload 스크램블 키 — TX tx_seq_·첫 라운드(ir_rv_=0 아래)
    if (!walsh_permuter_.Is_Initialized()) {
        (void)walsh_permuter_.Initialize(tx_seq_, 0u);
    } else {
        (void)walsh_permuter_.Update_Key(tx_seq_, 0u);
    }
    // TPC는 외부에서 명시적으로 사용할 때만 적용
    // 기본: caller의 amp 파라미터 사용 (기존 호환)
    tx_amp_ = amp;
    /* 신규 PDU 송신: 인코드 RV는 0부터 (try_decode_ 첫 라운드 rv=0 과 정합) */
    ir_rv_ = 0;
    /* BUG-FIX-PRE5: 프리앰블 반복 폐기, amp 부스트로 교체.
       프리앰블+헤더를 boost 배 진폭으로 전송.
       RX: 동기·헤더는 Walsh 인덱스 0..63 — walsh_dec_full_(…, cap=false)로
       전빈 탐색. 페이로드만 2^BPS 제한(cap=true)으로 FEC 심볼 집합과 정합. */
    const int16_t pre_amp =
        static_cast<int16_t>(static_cast<int32_t>(amp) * pre_boost_);
    // [BUG-FIX-PRE2] 프리앰블 반복 전송 — pre_reps_ × PRE_SYM0 + 1 × PRE_SYM1
    const int pre_chips = (pre_reps_ + 1) * 64;
    const uint32_t il = seed_ ^ (tx_seq_ * 0xA5A5A5A5u);
    static constexpr uint8_t k_tx_mb[4] = {0u, 1u, 2u, 3u};
    static constexpr int k_tx_psyms[4] = {FEC_HARQ::NSYM1, FEC_HARQ::NSYM16,
                                          FEC_HARQ::NSYM16, 0};
    const uint32_t mi = static_cast<uint32_t>(mode);
    if (mi > 3u) {
        return 0;
    }
    const uint32_t u0 = static_cast<uint32_t>(mi == 0u);
    const uint32_t u1 = static_cast<uint32_t>(mi == 1u);
    const uint32_t u2 = static_cast<uint32_t>(mi == 2u);
    const uint32_t u3 = static_cast<uint32_t>(mi == 3u);
    const uint32_t u16 = u1 | u2;
    const uint8_t mb = k_tx_mb[mi];
    const int nsym64_live = cur_nsym64_();
#if defined(HTS_USE_HOLO_TENSOR_4D)
    const uint32_t tensor_data_u =
        static_cast<uint32_t>(mode == PayloadMode::DATA);
    const int tensor_k = static_cast<int>(k_holo_profiles[1].block_bits);
    const int tensor_n = static_cast<int>(k_holo_profiles[1].chip_count);
    const int tensor_block_bytes = tensor_k / 8;
    const int tensor_blocks =
        (ilen + ((tensor_block_bytes > 0) ? (tensor_block_bytes - 1) : 0)) /
        ((tensor_block_bytes > 0) ? tensor_block_bytes : 1);
    const int tensor_psyms = tensor_k;
#else
    const uint32_t tensor_data_u = 0u;
    const int tensor_n = 64;
    const int tensor_blocks = 0;
    const int tensor_psyms = 0;
#endif
    // TPE: header psyms — LUT + DATA runtime nsym64
    const int psyms = k_tx_psyms[mi] + static_cast<int>(u3) * nsym64_live +
                      static_cast<int>(tensor_data_u) *
                          (tensor_psyms - nsym64_live);
    const uint32_t ir_hdr_iq_same_u = static_cast<uint32_t>(ir_mode_);
    const uint32_t iq_ind_u =
        static_cast<uint32_t>(iq_mode_ == IQ_Mode::IQ_INDEPENDENT);
    const uint16_t iq_bit =
        static_cast<uint16_t>((iq_ind_u & (1u - ir_hdr_iq_same_u)) *
                              static_cast<uint32_t>(HDR_IQ_BIT));
    uint16_t hdr = (static_cast<uint16_t>(mb & 0x03u) << 10u) | iq_bit |
                   (static_cast<uint16_t>(psyms) & 0x01FFu);

    uint8_t syms_v1[80] = {};
    uint8_t syms16_ir[FEC_HARQ::NSYM16] = {};
    uint8_t syms16_pl[FEC_HARQ::NSYM16] = {};
    uint8_t syms64_ir[FEC_HARQ::NSYM64] = {};
    uint8_t syms64_pl[FEC_HARQ::NSYM64] = {};
    const int irb = static_cast<int>(ir_mode_);

    const int il_v1 = ilen * static_cast<int>(u0);
    const int il_16 = ilen * static_cast<int>(u16);
    const int il_64 = ilen * static_cast<int>(u3);

    const int n_v1 = FEC_HARQ::Encode1(info, il_v1, syms_v1);
    const int enc16_ir =
        FEC_HARQ::Encode16_IR(info, il_16, syms16_ir, il, ir_rv_, wb_);
    const int enc16_pl = FEC_HARQ::Encode16(info, il_16, syms16_pl, il, wb_);
    const int enc16 = enc16_ir * irb + enc16_pl * (1 - irb);
    const int enc64_ir = FEC_HARQ::Encode64_IR(info, il_64, syms64_ir, il,
                                               cur_bps64_, ir_rv_, wb_);
    const int enc64_pl =
        FEC_HARQ::Encode64_A(info, il_64, syms64_pl, il, cur_bps64_, wb_);
    const int enc64 = enc64_ir * irb + enc64_pl * (1 - irb);

    // IR/plain 선택 (TPE 비트마스크 — 분기 없음)
    uint8_t syms16[FEC_HARQ::NSYM16] = {};
    uint8_t syms64[FEC_HARQ::NSYM64] = {};
    const uint32_t ir_mask = 0u - static_cast<uint32_t>(irb);
    const uint32_t pl_mask = ~ir_mask;
    for (int i = 0; i < FEC_HARQ::NSYM16; ++i) {
        syms16[i] = static_cast<uint8_t>(
            (static_cast<uint32_t>(syms16_ir[i]) & ir_mask) |
            (static_cast<uint32_t>(syms16_pl[i]) & pl_mask));
    }
    for (int i = 0; i < FEC_HARQ::NSYM64; ++i) {
        syms64[i] = static_cast<uint8_t>(
            (static_cast<uint32_t>(syms64_ir[i]) & ir_mask) |
            (static_cast<uint32_t>(syms64_pl[i]) & pl_mask));
    }

    // TPE: per-mode encode validity — inactive mode is always “ok”
    const uint32_t ok_v1 =
        (1u - u0) | (0u - static_cast<uint32_t>(n_v1 > 0));
    const uint32_t ok_16 =
        (1u - u16) | (0u - static_cast<uint32_t>(enc16 > 0));
    const uint32_t ok_64 =
        (1u - u3) | (0u - static_cast<uint32_t>(enc64 > 0));
    uint32_t go_enc = ~(0u);
    go_enc &= ok_v1 & ok_16 & ok_64;

    // TPE: pay chip budget from masked mode contributions
    int pay_raw = (n_v1 * static_cast<int>(u0)) +
                  (FEC_HARQ::NSYM16 * 16 * static_cast<int>(u16)) +
                  (nsym64_live * 64 * static_cast<int>(u3)) +
                  static_cast<int>(tensor_data_u) *
                      (tensor_blocks * tensor_n - nsym64_live * 64);
    pay_raw &= -static_cast<int>(go_enc & 1u);
    const int total_need = pre_chips + 128 + pay_raw;
    uint32_t go = go_enc;
    go &= static_cast<uint32_t>(total_need <= max_c);

    // TPE: pointer sink mask — intptr_t sign, no 0ull−bit wrap
    const std::uintptr_t okm = static_cast<std::uintptr_t>(
        -static_cast<std::intptr_t>(go & 1u));
    const int inc = static_cast<int>(go & 1u);

    int pos = 0;
#if defined(HTS_HOLO_PREAMBLE)
    int16_t holo_pre_I[64];
    generate_holo_preamble_(holo_pre_I, pre_amp, holo_lpi_seed_, tx_seq_);
#endif
    for (int r = 0; r < pre_reps_; ++r) {
#if defined(HTS_HOLO_PREAMBLE)
        const int16_t* const holo_src = holo_pre_I;
        for (int j = 0; j < 64; ++j) {
            int16_t* const di = bp_dst_i(oI, pos + j, okm);
            int16_t* const dq = bp_dst_q(oQ, pos + j, okm);
            di[0] = holo_src[j];
            dq[0] = 0;
        }
#else
#if defined(HTS_USE_PN_MASKED)
        const int pre_sym0_row =
            ProtectedEngine::detail::pn_masked_get_device_row();
        walsh_enc(static_cast<uint8_t>(pre_sym0_row), 64, pre_amp,
                  bp_dst_i(oI, pos, okm), bp_dst_q(oQ, pos, okm));
#else
        walsh_enc(PRE_SYM0, 64, pre_amp, bp_dst_i(oI, pos, okm),
                  bp_dst_q(oQ, pos, okm));
#endif
#endif
        pos += 64 * inc;
    }
    walsh_enc(PRE_SYM1, 64, pre_amp, bp_dst_i(oI, pos, okm),
              bp_dst_q(oQ, pos, okm));
    pos += 64 * inc;
    walsh_enc(static_cast<uint8_t>((hdr >> 6u) & 0x3Fu), 64, pre_amp,
              bp_dst_i(oI, pos, okm), bp_dst_q(oQ, pos, okm));
    pos += 64 * inc;
    walsh_enc(static_cast<uint8_t>(hdr & 0x3Fu), 64, pre_amp,
              bp_dst_i(oI, pos, okm), bp_dst_q(oQ, pos, okm));
    pos += 64 * inc;

    const int n_send_v1 = n_v1 * inc * static_cast<int>(u0);
    for (int s = 0; s < n_send_v1; ++s) {
        const uint8_t sv = syms_v1[static_cast<std::size_t>(s)];
        const uint32_t nz = 0u - static_cast<uint32_t>(sv != 0u);
        const int32_t amp32 = static_cast<int32_t>(amp);
        const int32_t v32 =
            amp32 + (static_cast<int32_t>(nz) & (-2 * amp32));
        const int16_t v = static_cast<int16_t>(v32);
        int16_t *const di = bp_dst_i(oI, pos, okm);
        int16_t *const dq = bp_dst_q(oQ, pos, okm);
        di[0] = v;
        dq[0] = v;
        pos += inc;
    }
    SecureMemory::secureWipe(static_cast<void *>(syms_v1), sizeof(syms_v1));

    const int n_send16 = FEC_HARQ::NSYM16 * inc * static_cast<int>(u16);
    for (int s = 0; s < n_send16; ++s) {
        const uint8_t raw_sym4 = syms16[static_cast<std::size_t>(s)];
        const uint8_t enc_sym4 = walsh_permuter_.Encode_4bit(raw_sym4);
        walsh_enc(enc_sym4, 16, amp, bp_dst_i(oI, pos, okm),
                  bp_dst_q(oQ, pos, okm));
        pos += 16 * inc;
    }
    SecureMemory::secureWipe(static_cast<void *>(syms16), sizeof(syms16));
    SecureMemory::secureWipe(static_cast<void *>(syms16_ir), sizeof(syms16_ir));
    SecureMemory::secureWipe(static_cast<void *>(syms16_pl), sizeof(syms16_pl));

    // TPE: DATA split path gated by u3 — non-DATA ⇒ 0 Walsh iterations
    const uint32_t split_u =
        static_cast<uint32_t>(static_cast<uint32_t>(!ir_mode_) &
                              iq_ind_u & u3 & (1u - tensor_data_u));
    const int npairs =
        ((nsym64_live + 1) / 2) * inc * static_cast<int>(u3) *
        static_cast<int>(1u - tensor_data_u);
    const int nwal = nsym64_live * inc * static_cast<int>(u3) *
                     static_cast<int>(1u - tensor_data_u);
    const int n_spl = npairs * static_cast<int>(split_u & 1u);
    const int n_sim = nwal * static_cast<int>((split_u ^ 1u) & 1u);
    for (int p = 0; p < n_spl; ++p) {
        const int s = p * 2;
        const uint8_t sI = syms64[static_cast<std::size_t>(s)];
        const uint32_t s2u = static_cast<uint32_t>(s + 1);
        const uint32_t nsymu = static_cast<uint32_t>(nsym64_live);
        const uint32_t have2 = 0u - static_cast<uint32_t>(s2u < nsymu);
        const size_t idx2 =
            static_cast<size_t>(s2u) & static_cast<size_t>(have2);
        const uint8_t sQ = static_cast<uint8_t>(
            static_cast<uint32_t>(syms64[idx2]) & (have2 & 0xFFu));
        const uint8_t enc_sI = walsh_permuter_.Encode_6bit(sI);
        const uint8_t enc_sQ = walsh_permuter_.Encode_6bit(sQ);
        walsh_enc_split(enc_sI, enc_sQ, 64, amp, bp_dst_i(oI, pos, okm),
                        bp_dst_q(oQ, pos, okm));
        pos += 64 * inc;
    }
    for (int s = 0; s < n_sim; ++s) {
        const uint8_t raw_sym6 = syms64[static_cast<std::size_t>(s)];
        const uint8_t enc_sym6 = walsh_permuter_.Encode_6bit(raw_sym6);
        walsh_enc(enc_sym6, 64, amp, bp_dst_i(oI, pos, okm),
                  bp_dst_q(oQ, pos, okm));
        pos += 64 * inc;
    }

#if defined(HTS_USE_HOLO_TENSOR_4D)
    if ((tensor_data_u & static_cast<uint32_t>(go & 1u)) != 0u) {
#if defined(HTS_PHASE_H_DIAG)
        static uint32_t s_tensor_tx_pkt_diag = 0u;
        const bool force_diag = (g_phase_h_diag_force != 0) &&
                                (g_phase_h_diag_seed == seed_);
        const bool diag_this_pkt = force_diag || (s_tensor_tx_pkt_diag == 0u);
#else
        const bool diag_this_pkt = false;
#endif
        if (ensure_holo_tensor_ready_() != HTS_Holo_Tensor_4D_TX::SECURE_TRUE) {
            return 0;
        }
        (void)holo_tx_.Set_Time_Slot(tx_seq_);
        if (diag_this_pkt) {
            std::printf(
                "[PHASE-H][TX] tx_seq=%u header_plen=%d blocks=%d profile(K,N,L)=(%u,%u,%u)\n",
                static_cast<unsigned>(tx_seq_), psyms, tensor_blocks,
                static_cast<unsigned>(holo_tensor_profile_.block_bits),
                static_cast<unsigned>(holo_tensor_profile_.chip_count),
                static_cast<unsigned>(holo_tensor_profile_.num_layers));
        }
        const uint32_t lk = static_cast<uint32_t>(holo_tensor_profile_.num_layers) *
                            static_cast<uint32_t>(holo_tensor_profile_.block_bits);
        for (int blk = 0; blk < tensor_blocks; ++blk) {
            int8_t data_bits[HOLO_MAX_BLOCK_BITS];
#if defined(HTS_USE_4D_DIVERSITY)
            int8_t chip_bpsk_I[HOLO_CHIP_COUNT];
            int8_t chip_bpsk_Q[HOLO_CHIP_COUNT];
#else
            int8_t chip_bpsk[HOLO_CHIP_COUNT];
#endif
            const int byte_off = blk * tensor_block_bytes;
            const int remain = (ilen > byte_off) ? (ilen - byte_off) : 0;
            const int use_bytes =
                (remain > tensor_block_bytes) ? tensor_block_bytes : remain;
            bytes_to_bpsk_bits_tx_(
                (use_bytes > 0) ? &info[byte_off] : nullptr,
                static_cast<size_t>((use_bytes > 0) ? use_bytes : 0), data_bits,
                static_cast<uint16_t>(tensor_k));
#if defined(HTS_USE_4D_DIVERSITY)
            (void)holo_tx_.Set_Time_Slot(tx_seq_);
            if (holo_tx_.Encode_Block(data_bits, holo_tensor_profile_.block_bits,
                                      chip_bpsk_I, holo_tensor_profile_.chip_count) !=
                HTS_Holo_Tensor_4D_TX::SECURE_TRUE) {
                SecureMemory::secureWipe(static_cast<void*>(data_bits), sizeof(data_bits));
                SecureMemory::secureWipe(static_cast<void*>(chip_bpsk_I),
                                         sizeof(chip_bpsk_I));
                return 0;
            }
            (void)holo_tx_.Set_Time_Slot(tx_seq_ ^ 1u);
            if (holo_tx_.Encode_Block(data_bits, holo_tensor_profile_.block_bits,
                                      chip_bpsk_Q, holo_tensor_profile_.chip_count) !=
                HTS_Holo_Tensor_4D_TX::SECURE_TRUE) {
                SecureMemory::secureWipe(static_cast<void*>(data_bits), sizeof(data_bits));
                SecureMemory::secureWipe(static_cast<void*>(chip_bpsk_I),
                                         sizeof(chip_bpsk_I));
                SecureMemory::secureWipe(static_cast<void*>(chip_bpsk_Q),
                                         sizeof(chip_bpsk_Q));
                (void)holo_tx_.Set_Time_Slot(tx_seq_);
                return 0;
            }
            (void)holo_tx_.Set_Time_Slot(tx_seq_);
#else
            if (holo_tx_.Encode_Block(data_bits, holo_tensor_profile_.block_bits,
                                            chip_bpsk, holo_tensor_profile_.chip_count) !=
                HTS_Holo_Tensor_4D_TX::SECURE_TRUE) {
                SecureMemory::secureWipe(static_cast<void*>(data_bits), sizeof(data_bits));
                SecureMemory::secureWipe(static_cast<void*>(chip_bpsk), sizeof(chip_bpsk));
                return 0;
            }
#endif
            const int denom_raw = (lk > 0u) ? static_cast<int>(lk) : 32;
            const int denom = (denom_raw > 1) ? (denom_raw >> 1) : 1;
            if (diag_this_pkt && blk == 0) {
                std::printf("[PHASE-H][TX] time_slot(tx_seq)=%u input_bits16:",
                            static_cast<unsigned>(tx_seq_));
                for (int i = 0; i < tensor_k; ++i) {
                    std::printf(" %d", static_cast<int>(data_bits[i]));
                }
                std::printf("\n");
                std::printf("[PHASE-H][TX] encoded_chip_bpsk64:");
#if defined(HTS_USE_4D_DIVERSITY)
                for (int i = 0; i < tensor_n; ++i) {
                    std::printf(" %d", static_cast<int>(chip_bpsk_I[i]));
                }
#else
                for (int i = 0; i < tensor_n; ++i) {
                    std::printf(" %d", static_cast<int>(chip_bpsk[i]));
                }
#endif
                std::printf("\n");
            }
            for (int c = 0; c < tensor_n; ++c) {
#if defined(HTS_USE_4D_DIVERSITY)
                const int32_t prod_I =
                    static_cast<int32_t>(chip_bpsk_I[c]) * static_cast<int32_t>(amp);
                const int32_t v_I =
                    (prod_I >= 0) ? ((prod_I + (denom >> 1)) / denom)
                                  : ((prod_I - (denom >> 1)) / denom);
                const int32_t prod_Q =
                    static_cast<int32_t>(chip_bpsk_Q[c]) * static_cast<int32_t>(amp);
                const int32_t v_Q =
                    (prod_Q >= 0) ? ((prod_Q + (denom >> 1)) / denom)
                                  : ((prod_Q - (denom >> 1)) / denom);
                oI[pos] = static_cast<int16_t>(v_I);
                oQ[pos] = static_cast<int16_t>(v_Q);
                if (diag_this_pkt && blk == 0) {
                    std::printf("[PHASE-H][TX] chip[%d]: bpskI=%d bpskQ=%d modI=%d modQ=%d\n",
                                c, static_cast<int>(chip_bpsk_I[c]),
                                static_cast<int>(chip_bpsk_Q[c]),
                                static_cast<int>(oI[pos]),
                                static_cast<int>(oQ[pos]));
                }
#else
                const int32_t prod =
                    static_cast<int32_t>(chip_bpsk[c]) * static_cast<int32_t>(amp);
                const int32_t v =
                    (prod >= 0) ? ((prod + (denom >> 1)) / denom)
                                : ((prod - (denom >> 1)) / denom);
                oI[pos] = static_cast<int16_t>(v);
                oQ[pos] = static_cast<int16_t>(v);
                if (diag_this_pkt && blk == 0) {
                    std::printf("[PHASE-H][TX] chip[%d]: bpsk=%d modI=%d modQ=%d\n",
                                c, static_cast<int>(chip_bpsk[c]),
                                static_cast<int>(oI[pos]),
                                static_cast<int>(oQ[pos]));
                }
#endif
                ++pos;
            }
            SecureMemory::secureWipe(static_cast<void*>(data_bits), sizeof(data_bits));
#if defined(HTS_USE_4D_DIVERSITY)
            SecureMemory::secureWipe(static_cast<void*>(chip_bpsk_I), sizeof(chip_bpsk_I));
            SecureMemory::secureWipe(static_cast<void*>(chip_bpsk_Q), sizeof(chip_bpsk_Q));
#else
            SecureMemory::secureWipe(static_cast<void*>(chip_bpsk), sizeof(chip_bpsk));
#endif
        }
#if defined(HTS_PHASE_H_DIAG)
        if (diag_this_pkt && !force_diag) {
            ++s_tensor_tx_pkt_diag;
        }
#endif
    }
#endif
    SecureMemory::secureWipe(static_cast<void *>(syms64), sizeof(syms64));
    SecureMemory::secureWipe(static_cast<void *>(syms64_ir), sizeof(syms64_ir));
    SecureMemory::secureWipe(static_cast<void *>(syms64_pl), sizeof(syms64_pl));

    if ((go & 1u) != 0u && pos > 0 &&
        holo_lpi_en_ == HTS_Holo_LPI::SECURE_TRUE) {
        if (HTS_Holo_LPI::Generate_Scalars(holo_lpi_seed_, tx_seq_,
                                           holo_lpi_mix_q8_,
                                           holo_lpi_scalars_) ==
            HTS_Holo_LPI::SECURE_TRUE) {
            const int pc = (pos > 65535) ? 65535 : pos;
            HTS_Holo_LPI::Apply(oI, oQ, static_cast<uint16_t>(pc),
                                holo_lpi_scalars_);
        }
    }

    tx_seq_ += static_cast<uint32_t>(go & 1u);
    return pos;
}
/* BUG-FIX-RETX3: HARQ 연속모드 TX — 프리앰블/헤더 생략 */
int HTS_V400_Dispatcher::Build_Retx(PayloadMode mode, const uint8_t *info,
                                    int ilen, int16_t amp, int16_t *oI,
                                    int16_t *oQ, int max_c) noexcept {
    if (phase_ == RxPhase::RF_SETTLING) {
        return 0;
    }
    if (info == nullptr || oI == nullptr || oQ == nullptr)
        return 0;
    if (ilen < 0 || max_c <= 0)
        return 0;
    // TPC는 외부에서 명시적으로 사용할 때만 적용
    // 기본: caller의 amp 파라미터 사용 (기존 호환)
    tx_amp_ = amp;
    int pos = 0;
    /* Build_Packet가 tx_seq_++ 한 뒤이므로 Retx il은 직전 송신
       시퀀스(tx_seq_-1)와 수신 il(seed_ ^ rx_seq_*0xA5A5A5A5) 정합 */
    const uint32_t tx_seq_prev = (tx_seq_ > 0u) ? (tx_seq_ - 1u) : 0u;
    // [Walsh_Row_Permuter] 재전송 — 직전 PDU tx_seq_prev + 현재 IR RV
    if (!walsh_permuter_.Is_Initialized()) {
        (void)walsh_permuter_.Initialize(
            tx_seq_prev, static_cast<uint8_t>(ir_rv_ & 0xFF));
    } else {
        (void)walsh_permuter_.Update_Key(
            tx_seq_prev, static_cast<uint8_t>(ir_rv_ & 0xFF));
    }
    const uint32_t il = seed_ ^ (tx_seq_prev * 0xA5A5A5A5u);
    const uint32_t mi = static_cast<uint32_t>(mode);
    if (mi > 3u || mi == 0u) {
        return 0;
    }
    const uint32_t u1 = static_cast<uint32_t>(mi == 1u);
    const uint32_t u2 = static_cast<uint32_t>(mi == 2u);
    const uint32_t u3 = static_cast<uint32_t>(mi == 3u);
    const uint32_t u16 = u1 | u2;
    const uint32_t iq_ind_u =
        static_cast<uint32_t>(iq_mode_ == IQ_Mode::IQ_INDEPENDENT);

    uint8_t syms16_ir[FEC_HARQ::NSYM16] = {};
    uint8_t syms16_pl[FEC_HARQ::NSYM16] = {};
    uint8_t syms64_ir[FEC_HARQ::NSYM64] = {};
    uint8_t syms64_pl[FEC_HARQ::NSYM64] = {};
    const int irb = static_cast<int>(ir_mode_);

    const int il_16 = ilen * static_cast<int>(u16);
    const int il_64 = ilen * static_cast<int>(u3);

    const int enc16_ir =
        FEC_HARQ::Encode16_IR(info, il_16, syms16_ir, il, ir_rv_, wb_);
    const int enc16_pl = FEC_HARQ::Encode16(info, il_16, syms16_pl, il, wb_);
    const int enc16 = enc16_ir * irb + enc16_pl * (1 - irb);

    const int enc64_ir = FEC_HARQ::Encode64_IR(info, il_64, syms64_ir, il,
                                               cur_bps64_, ir_rv_, wb_);
    const int enc64_pl =
        FEC_HARQ::Encode64_A(info, il_64, syms64_pl, il, cur_bps64_, wb_);
    const int enc64 = enc64_ir * irb + enc64_pl * (1 - irb);

    // IR/plain 선택 (TPE 비트마스크)
    uint8_t syms16[FEC_HARQ::NSYM16] = {};
    uint8_t syms64[FEC_HARQ::NSYM64] = {};
    const uint32_t ir_mask = 0u - static_cast<uint32_t>(irb);
    const uint32_t pl_mask = ~ir_mask;
    for (int i = 0; i < FEC_HARQ::NSYM16; ++i) {
        syms16[i] = static_cast<uint8_t>(
            (static_cast<uint32_t>(syms16_ir[i]) & ir_mask) |
            (static_cast<uint32_t>(syms16_pl[i]) & pl_mask));
    }
    for (int i = 0; i < FEC_HARQ::NSYM64; ++i) {
        syms64[i] = static_cast<uint8_t>(
            (static_cast<uint32_t>(syms64_ir[i]) & ir_mask) |
            (static_cast<uint32_t>(syms64_pl[i]) & pl_mask));
    }

    const uint32_t bad_enc =
        (u16 & (0u - static_cast<uint32_t>(enc16 <= 0))) |
        (u3 & (0u - static_cast<uint32_t>(enc64 <= 0)));
    if (bad_enc != 0u) {
        SecureMemory::secureWipe(static_cast<void *>(syms16), sizeof(syms16));
        SecureMemory::secureWipe(static_cast<void *>(syms16_ir), sizeof(syms16_ir));
        SecureMemory::secureWipe(static_cast<void *>(syms16_pl), sizeof(syms16_pl));
        SecureMemory::secureWipe(static_cast<void *>(syms64), sizeof(syms64));
        SecureMemory::secureWipe(static_cast<void *>(syms64_ir), sizeof(syms64_ir));
        SecureMemory::secureWipe(static_cast<void *>(syms64_pl), sizeof(syms64_pl));
        return 0;
    }

    const int n16_loop = FEC_HARQ::NSYM16 * static_cast<int>(u16);
    for (int s = 0; s < n16_loop; ++s) {
        const int space = max_c - pos;
        if (space < 16) {
            SecureMemory::secureWipe(static_cast<void *>(syms16),
                                     sizeof(syms16));
            SecureMemory::secureWipe(static_cast<void *>(syms16_ir),
                                     sizeof(syms16_ir));
            SecureMemory::secureWipe(static_cast<void *>(syms16_pl),
                                     sizeof(syms16_pl));
            SecureMemory::secureWipe(static_cast<void *>(syms64),
                                     sizeof(syms64));
            SecureMemory::secureWipe(static_cast<void *>(syms64_ir),
                                     sizeof(syms64_ir));
            SecureMemory::secureWipe(static_cast<void *>(syms64_pl),
                                     sizeof(syms64_pl));
            return 0;
        }
        const uint8_t raw_sym4 = syms16[static_cast<std::size_t>(s)];
        const uint8_t enc_sym4 = walsh_permuter_.Encode_4bit(raw_sym4);
        walsh_enc(enc_sym4, 16, amp, &oI[pos], &oQ[pos]);
        pos += 16;
    }

    const int nsym = cur_nsym64_() * static_cast<int>(u3);
    const uint32_t split_u =
        static_cast<uint32_t>(static_cast<uint32_t>(!ir_mode_) & iq_ind_u &
                              u3);
    const int npairs = ((nsym + 1) / 2) * static_cast<int>(u3);
    const int n_spl = npairs * static_cast<int>(split_u & 1u);
    const int n_sim =
        nsym * static_cast<int>(u3) * static_cast<int>((split_u ^ 1u) & 1u);

    for (int p = 0; p < n_spl; ++p) {
        const int s = p * 2;
        const int space = max_c - pos;
        if (space < 64) {
            SecureMemory::secureWipe(static_cast<void *>(syms16),
                                     sizeof(syms16));
            SecureMemory::secureWipe(static_cast<void *>(syms16_ir),
                                     sizeof(syms16_ir));
            SecureMemory::secureWipe(static_cast<void *>(syms16_pl),
                                     sizeof(syms16_pl));
            SecureMemory::secureWipe(static_cast<void *>(syms64),
                                     sizeof(syms64));
            SecureMemory::secureWipe(static_cast<void *>(syms64_ir),
                                     sizeof(syms64_ir));
            SecureMemory::secureWipe(static_cast<void *>(syms64_pl),
                                     sizeof(syms64_pl));
            return 0;
        }
        const uint8_t sI = syms64[static_cast<std::size_t>(s)];
        const uint32_t s2u = static_cast<uint32_t>(s + 1);
        const uint32_t nsymu = static_cast<uint32_t>(nsym);
        const uint32_t have2 = 0u - static_cast<uint32_t>(s2u < nsymu);
        const size_t idx2 =
            static_cast<size_t>(s2u) & static_cast<size_t>(have2);
        const uint8_t sQ = static_cast<uint8_t>(
            static_cast<uint32_t>(syms64[idx2]) & (have2 & 0xFFu));
        const uint8_t enc_sI = walsh_permuter_.Encode_6bit(sI);
        const uint8_t enc_sQ = walsh_permuter_.Encode_6bit(sQ);
        walsh_enc_split(enc_sI, enc_sQ, 64, amp, &oI[pos], &oQ[pos]);
        pos += 64;
    }
    for (int s = 0; s < n_sim; ++s) {
        const int space = max_c - pos;
        if (space < 64) {
            SecureMemory::secureWipe(static_cast<void *>(syms16),
                                     sizeof(syms16));
            SecureMemory::secureWipe(static_cast<void *>(syms16_ir),
                                     sizeof(syms16_ir));
            SecureMemory::secureWipe(static_cast<void *>(syms16_pl),
                                     sizeof(syms16_pl));
            SecureMemory::secureWipe(static_cast<void *>(syms64),
                                     sizeof(syms64));
            SecureMemory::secureWipe(static_cast<void *>(syms64_ir),
                                     sizeof(syms64_ir));
            SecureMemory::secureWipe(static_cast<void *>(syms64_pl),
                                     sizeof(syms64_pl));
            return 0;
        }
        const uint8_t raw_sym6 = syms64[static_cast<std::size_t>(s)];
        const uint8_t enc_sym6 = walsh_permuter_.Encode_6bit(raw_sym6);
        walsh_enc(enc_sym6, 64, amp, &oI[pos], &oQ[pos]);
        pos += 64;
    }

    SecureMemory::secureWipe(static_cast<void *>(syms16), sizeof(syms16));
    SecureMemory::secureWipe(static_cast<void *>(syms16_ir), sizeof(syms16_ir));
    SecureMemory::secureWipe(static_cast<void *>(syms16_pl), sizeof(syms16_pl));
    SecureMemory::secureWipe(static_cast<void *>(syms64), sizeof(syms64));
    SecureMemory::secureWipe(static_cast<void *>(syms64_ir), sizeof(syms64_ir));
    SecureMemory::secureWipe(static_cast<void *>(syms64_pl), sizeof(syms64_pl));

    if (pos > 0 && holo_lpi_en_ == HTS_Holo_LPI::SECURE_TRUE) {
        if (HTS_Holo_LPI::Generate_Scalars(
                holo_lpi_seed_, tx_seq_prev, holo_lpi_mix_q8_,
                holo_lpi_scalars_) == HTS_Holo_LPI::SECURE_TRUE) {
            const int pc = (pos > 65535) ? 65535 : pos;
            HTS_Holo_LPI::Apply(oI, oQ, static_cast<uint16_t>(pc),
                                holo_lpi_scalars_);
        }
    }

    return pos;
}
} // namespace ProtectedEngine
