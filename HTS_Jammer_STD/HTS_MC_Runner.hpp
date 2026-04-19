// =============================================================================
/// @file  HTS_MC_Runner.hpp
/// @brief Monte Carlo 50회 실행기 — Phase 6 HARQ Matrix 전용
/// @target PC 시뮬 (HTS_Jammer_STD 프로젝트)
///
/// [꼼수 방어 — 명령서 v1.0 §2 Dispatcher Harness 준수]
///   - 매 trial 마다 Dispatcher 새 인스턴스 (seed/rx_seq_/tx_seq_ 교차 오염
///   금지)
///   - Chase / IR 경로 직렬 실행 (BSS 스크래치 공유로 병렬 금지)
///   - IR-SIC 기본 OFF 고정
///   - BPS 고정 (적응형 분리 실험은 별도)
///   - 실패 trial 포함 (SPEC_002 §15.7)
///   - 매 trial 독립 φ, 독립 재밍 위치 (§6.2, Partial 심볼 선택)
///
/// [동기/헤더 100% 가정]
///   Build_Packet 전체 칩열 주입 후 재밍은 "페이로드 구간만" 적용.
///   즉 프리앰블·헤더는 재밍 영향 없음 → 순수 FEC/HARQ 한계 측정.
///   보고서에 이 가정을 명시할 것.
// =============================================================================
#pragma once
#include "HTS_Clopper_Pearson.hpp"
#include "HTS_Jammer_STD.hpp"
// 메인트리 복사본 — HTS_Jammer_STD 프로젝트에 포함된 헤더
#include "HTS_FEC_HARQ.hpp"
#include "HTS_V400_Dispatcher.hpp"
#include <cmath>
#include <cstdint>
#include <cstring>
#include <random>
#include <vector>
namespace HTS_MC_Runner {
// ── 결과 구조체 ───────────────────────────────────────────────
struct CellResult {
    int crc_ok_count;          ///< 50회 중 CRC 성공 수
    int total;                 ///< N_MC (=50)
    double crc_rate;           ///< 성공률 (0.0~1.0)
    double ci_low;             ///< Clopper-Pearson 95% 하한
    double ci_high;            ///< Clopper-Pearson 95% 상한
    double ber;                ///< 평균 비트 오류율 (실패 trial 포함)
    int total_harq_rounds;     ///< 라운드 수 합산
    double avg_harq_per_block; ///< 블록당 평균 라운드
    int max_harq_rounds;       ///< 최대 라운드 (관측)
    double saturation_rate;    ///< int16 포화율 (페이로드 구간 평균)
    uint64_t base_seed;        ///< 재현성 로그
};
// ── Trial 단위 결과 (내부) ────────────────────────────────────
struct TrialOutcome {
    bool crc_ok;
    int harq_k;
    int bit_errors;          ///< 최대 64 (8 bytes)
    double saturation_ratio; ///< 0.0~1.0
};
// ── 콜백 전역 (단일 스레드 실행 가정) ─────────────────────────
namespace cb_detail {
struct CbContext {
    uint8_t original_info[8]{};
    bool crc_ok{false};
    int harq_k{0};
    int bit_errors{64};
    bool received{false};
};
inline CbContext &global() {
    static CbContext ctx;
    return ctx;
}
inline void on_packet(const ProtectedEngine::DecodedPacket &pkt) {
    auto &c = global();
    c.crc_ok =
        (pkt.success_mask == ProtectedEngine::DecodedPacket::DECODE_MASK_OK);
    c.harq_k = pkt.harq_k;
    c.received = true;
    // BER: 원본 info 8바이트 vs pkt.data 8바이트 XOR → popcount
    int errs = 0;
    const int blen = (pkt.data_len < 8) ? pkt.data_len : 8;
    for (int i = 0; i < blen; ++i) {
        uint8_t d = static_cast<uint8_t>(c.original_info[i] ^ pkt.data[i]);
        // popcount 8-bit
        d = (d & 0x55u) + ((d >> 1) & 0x55u);
        d = (d & 0x33u) + ((d >> 2) & 0x33u);
        d = (d & 0x0Fu) + ((d >> 4) & 0x0Fu);
        errs += d;
    }
    // 누락된 바이트는 전부 에러 가정 (정직)
    errs += (8 - blen) * 8;
    c.bit_errors = errs;
}
} // namespace cb_detail
// ── 재밍 주입 (페이로드 구간만) ────────────────────────────────
inline void Apply_Jammer_On_Payload(int16_t *chipI_payload,
                                    int16_t *chipQ_payload, int payload_chips,
                                    int nsym, int chips_per_sym,
                                    HTS_Jammer_STD::ChannelType ch,
                                    double intensity, double P_s,
                                    std::mt19937 &rng_jam) noexcept {
    using namespace HTS_Jammer_STD;
    switch (ch) {
    case ChannelType::Clean:
        return;
    case ChannelType::AWGN:
        Add_AWGN(chipI_payload, chipQ_payload, payload_chips, intensity, P_s,
                 rng_jam);
        return;
    case ChannelType::Barrage:
        Add_Barrage(chipI_payload, chipQ_payload, payload_chips, intensity, P_s,
                    rng_jam);
        return;
    case ChannelType::CW:
        Add_CW(chipI_payload, chipQ_payload, payload_chips, intensity, P_s,
               StdParams::CW_F_OFFSET_HZ, StdParams::F_CHIP_HZ, rng_jam);
        return;
    case ChannelType::Pulse:
        Add_Pulse(chipI_payload, chipQ_payload, payload_chips, intensity, P_s,
                  StdParams::PULSE_DUTY, StdParams::PULSE_T_PERIOD,
                  StdParams::CW_F_OFFSET_HZ, StdParams::F_CHIP_HZ, rng_jam);
        return;
    case ChannelType::MultiTone:
        Add_MultiTone(chipI_payload, chipQ_payload, payload_chips, intensity,
                      P_s, StdParams::MULTI_N_TONES,
                      StdParams::F_CHIP_HZ, // BW=f_chip as full Nyquist
                      StdParams::F_CHIP_HZ, rng_jam);
        return;
    case ChannelType::Swept:
        Add_Swept(chipI_payload, chipQ_payload, payload_chips, intensity, P_s,
                  StdParams::SWEPT_F_START_HZ, StdParams::SWEPT_F_END_HZ,
                  StdParams::SWEPT_RATE_HZ_S, StdParams::F_CHIP_HZ, rng_jam);
        return;
    case ChannelType::Partial_Barrage:
        Add_Partial_Barrage(chipI_payload, chipQ_payload, nsym, chips_per_sym,
                            intensity, /* ratio % */
                            StdParams::PARTIAL_JSR_DB, P_s, rng_jam);
        return;
    }
}
// ── int16 포화율 측정 ─────────────────────────────────────────
inline double Measure_Saturation(const int16_t *I, const int16_t *Q,
                                 int N) noexcept {
    if (N <= 0)
        return 0.0;
    int cnt = 0;
    for (int n = 0; n < N; ++n) {
        if (I[n] == 32767 || I[n] == -32768 || Q[n] == 32767 || Q[n] == -32768)
            ++cnt;
    }
    return static_cast<double>(cnt) / static_cast<double>(N);
}
// ── 단일 trial 실행 ───────────────────────────────────────────
inline TrialOutcome Run_Single_Trial(bool use_ir, bool is_voice,
                                     HTS_Jammer_STD::ChannelType channel,
                                     double intensity, uint64_t trial_seed,
                                     int max_harq_rounds) {
    using namespace ProtectedEngine;
    TrialOutcome out{};
    out.crc_ok = false;
    out.harq_k = 0;
    out.bit_errors = 64;
    out.saturation_ratio = 0.0;
    // ── 난수 분리 (signal / jam / noise 독립) ────────────────
    std::mt19937 rng_sig(static_cast<uint32_t>(trial_seed));
    std::mt19937 rng_jam(static_cast<uint32_t>(trial_seed ^ 0xBEEFCAFEull));
    std::mt19937 rng_noise(static_cast<uint32_t>(trial_seed ^ 0xC0DEF00Dull));
    // ── 매 trial 무작위 info 8바이트 ──────────────────────────
    uint8_t info[8];
    for (int i = 0; i < 8; ++i)
        info[i] = static_cast<uint8_t>(rng_sig() & 0xFFu);
    // ── 콜백 컨텍스트 초기화 ──────────────────────────────────
    auto &ctx = cb_detail::global();
    std::memset(&ctx, 0, sizeof(ctx));
    std::memcpy(ctx.original_info, info, 8);
    ctx.bit_errors = 64;
    // ── TX Dispatcher (매 trial 새 인스턴스) ──────────────────
    auto *p_tx = new HTS_V400_Dispatcher();
    p_tx->Set_Seed(static_cast<uint32_t>(trial_seed));
    p_tx->Set_IR_Mode(use_ir);
    if (!is_voice) {
        p_tx->Set_Lab_BPS64(4); // DATA 모드 BPS=4 고정
    }
    p_tx->Set_IR_SIC_Enabled(false); // OFF 고정
    // ── RX Dispatcher (매 trial 새 인스턴스) ──────────────────
    auto *p_rx = new HTS_V400_Dispatcher();
    p_rx->Set_Seed(static_cast<uint32_t>(trial_seed));
    p_rx->Set_IR_Mode(use_ir);
    if (!is_voice) {
        p_rx->Set_Lab_BPS64(4);
    }
    p_rx->Set_IR_SIC_Enabled(false);
    p_rx->Set_Packet_Callback(&cb_detail::on_packet);
    const PayloadMode mode = is_voice ? PayloadMode::VOICE : PayloadMode::DATA;
    const int chips_per_sym = is_voice ? 16 : 64;
    // ── 칩 버퍼 ───────────────────────────────────────────────
    const int MAX_CHIPS = 65536;
    std::vector<int16_t> chipI(MAX_CHIPS, 0);
    std::vector<int16_t> chipQ(MAX_CHIPS, 0);
    // ── 페이로드 오프셋 (프리앰블 + PRE_SYM1 + 헤더 2심볼) ───
    const int pre_reps = p_tx->Get_Preamble_Reps();
    const int payload_offset = (pre_reps + 1) * 64 + 128;
    int16_t amp = 300;
    double sat_accum = 0.0;
    int sat_rounds = 0;
    // ── HARQ 루프 (harness 외부 상한 = max_harq_rounds) ──────
    for (int round = 0; round < max_harq_rounds; ++round) {
        // TX 칩 생성
        int nchips = 0;
        if (round == 0) {
            nchips = p_tx->Build_Packet(mode, info, 8, amp, chipI.data(),
                                        chipQ.data(), MAX_CHIPS);
        } else {
            nchips = p_tx->Build_Retx(mode, info, 8, amp, chipI.data(),
                                      chipQ.data(), MAX_CHIPS);
        }
        if (nchips <= 0)
            break;
        // 페이로드 구간 식별
        int pay_start, pay_chips;
        if (round == 0) {
            pay_start = payload_offset;
            pay_chips = nchips - payload_offset;
        } else {
            pay_start = 0; // Build_Retx 는 페이로드만 생성
            pay_chips = nchips;
        }
        if (pay_chips <= 0)
            break;
        const int nsym = pay_chips / chips_per_sym;
        // P_s 측정 (페이로드 구간, RMS 평균)
        double P_s = HTS_Jammer_STD::Measure_Signal_Power(
            chipI.data() + pay_start, chipQ.data() + pay_start, pay_chips);
        if (P_s <= 0.0)
            P_s = 1.0; // 보호
        // 재밍 주입 (페이로드 구간만)
        Apply_Jammer_On_Payload(
            chipI.data() + pay_start, chipQ.data() + pay_start, pay_chips, nsym,
            chips_per_sym, channel, intensity, P_s, rng_jam);
        // 포화율 측정 (누적)
        sat_accum += Measure_Saturation(chipI.data() + pay_start,
                                        chipQ.data() + pay_start, pay_chips);
        ++sat_rounds;
        // RX 주입
        ctx.received = false;
        if (round == 0) {
            for (int c = 0; c < nchips; ++c) {
                p_rx->Feed_Chip(chipI[c], chipQ[c]);
            }
        } else {
            for (int c = 0; c < nchips; ++c) {
                p_rx->Feed_Retx_Chip(chipI[c], chipQ[c]);
            }
        }
        // 콜백 수신 확인 (성공/실패 여부)
        if (ctx.received && ctx.crc_ok) {
            break; // CRC 성공
        }
        // CRC 실패: 다음 라운드 진행
    }
    // ── 결과 집계 ─────────────────────────────────────────────
    out.crc_ok = ctx.crc_ok;
    out.harq_k = ctx.received ? ctx.harq_k : max_harq_rounds;
    out.bit_errors = ctx.crc_ok ? 0 : ctx.bit_errors;
    out.saturation_ratio =
        (sat_rounds > 0) ? (sat_accum / static_cast<double>(sat_rounds)) : 0.0;
    delete p_rx;
    delete p_tx;
    return out;
}
// ── Cell 실행 (N_MC trial 반복) ───────────────────────────────
inline CellResult Run_Cell(bool use_ir, bool is_voice,
                           HTS_Jammer_STD::ChannelType channel,
                           double intensity, int N_MC, uint64_t base_seed,
                           int max_harq_rounds) {
    CellResult r{};
    r.total = N_MC;
    r.base_seed = base_seed;
    int ok_cnt = 0;
    int total_harq = 0;
    int max_observed_harq = 0;
    long long total_bit_errors = 0;
    double sat_accum = 0.0;
    for (int t = 0; t < N_MC; ++t) {
        uint64_t trial_seed = base_seed + static_cast<uint64_t>(t) *
                                              0x100000001B3ull; // FNV prime
        TrialOutcome o = Run_Single_Trial(use_ir, is_voice, channel, intensity,
                                          trial_seed, max_harq_rounds);
        if (o.crc_ok)
            ++ok_cnt;
        total_harq += o.harq_k;
        total_bit_errors += o.bit_errors;
        sat_accum += o.saturation_ratio;
        if (o.harq_k > max_observed_harq)
            max_observed_harq = o.harq_k;
    }
    r.crc_ok_count = ok_cnt;
    r.crc_rate = static_cast<double>(ok_cnt) / static_cast<double>(N_MC);
    r.total_harq_rounds = total_harq;
    r.avg_harq_per_block =
        static_cast<double>(total_harq) / static_cast<double>(N_MC);
    r.max_harq_rounds = max_observed_harq;
    r.saturation_rate = sat_accum / static_cast<double>(N_MC);
    // BER: 총 비트 오류 / (N_MC × 64 bits)
    r.ber = static_cast<double>(total_bit_errors) /
            (static_cast<double>(N_MC) * 64.0);
    // Clopper-Pearson 95% CI
    auto ci = HTS_Clopper_Pearson::Compute(ok_cnt, N_MC, 0.05);
    r.ci_low = ci.p_lower;
    r.ci_high = ci.p_upper;
    return r;
}
} // namespace HTS_MC_Runner
