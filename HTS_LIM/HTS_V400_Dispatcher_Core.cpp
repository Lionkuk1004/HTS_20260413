// =============================================================================
// HTS_V400_Dispatcher_Core.cpp — ctor / FHSS / 설정 / CCM·스크래치 정의
// =============================================================================
#include "HTS_V400_Dispatcher.hpp"
#include "HTS_V400_Dispatcher_Internal.hpp"
#include "HTS_Holo_LPI.h"
#include "HTS_RF_Metrics.h"
#include "HTS_Secure_Memory.h"
#include <cstdio>
#include <cstring>
extern "C" volatile int g_hts_ir_diag_chip0 = 0;
extern "C" volatile int g_hts_ir_diag_feed_idx = -1;
extern "C" void Mock_RF_Synth_Set_Channel(uint8_t channel) noexcept {
    const unsigned ch = static_cast<unsigned>(channel) & 0x7Fu;
    std::printf("[Mock_RF_Synth] ch=%u\n", ch);
}
namespace ProtectedEngine {
using namespace detail;
alignas(64) uint8_t g_v400_sym_scratch[FEC_HARQ::NSYM64];
static_assert(sizeof(g_v400_sym_scratch) >= FEC_HARQ::NSYM16,
              "sym scratch must cover NSYM16 encode");
alignas(64) HTS_CCM_SECTION V400HarqCcm g_harq_ccm_union{};
static_assert(sizeof(g_harq_ccm_union) >= sizeof(V400HarqCcmChase),
              "CCM union must hold Chase harq_Q");
static_assert(sizeof(g_harq_ccm_union) >= sizeof(V400HarqCcmIr),
              "CCM union must hold IR chip buffers + IR_RxState");
static_assert(sizeof(int16_t) == 2, "int16_t must be 2 bytes");
static_assert(sizeof(int32_t) == 4, "int32_t must be 4 bytes");
static_assert(sizeof(uint64_t) == 8, "uint64_t required for FWHT energy");
// SIC 1비트 압축: Walsh 칩 = ±amp (I=Q 동일), 부호만 저장
// bit c = 1 → 음(−amp), bit c = 0 → 양(+amp)
// NSYM64 × 8B (기존 int16 I/Q 각 NSYM64×64 대비 약 97% 절감)
alignas(64) uint64_t g_sic_bits[FEC_HARQ::NSYM64];
static_assert(sizeof(g_sic_bits) == FEC_HARQ::NSYM64 * 8u,
              "SIC bit-pack size mismatch");
static_assert(FEC_HARQ::C64 == 64, "SIC bit-pack requires C64==64");
#if defined(HTS_PHASE0_WALSH_BANK)
uint8_t k_W63_FWHT_ROW = 255u;
namespace {
uint8_t calc_kw63_fwht_row_() noexcept {
    int32_t T_I[64];
    int32_t T_Q[64];
    for (int i = 0; i < 64; ++i) {
        T_I[i] = static_cast<int32_t>(k_w63[static_cast<std::size_t>(i)]) * 1000;
        T_Q[i] = 0;
    }
    fwht_64_complex_inplace_(T_I, T_Q);

    int max_row = 0;
    int64_t max_e = 0;
    for (int r = 0; r < 64; ++r) {
        const int64_t e = static_cast<int64_t>(T_I[r]) * T_I[r];
        if (e > max_e) {
            max_e = e;
            max_row = r;
        }
    }
    return static_cast<uint8_t>(max_row);
}
} // namespace
#endif
HTS_V400_Dispatcher::HTS_V400_Dispatcher() noexcept
    : phase_(RxPhase::WAIT_SYNC), cur_mode_(PayloadMode::UNKNOWN),
      active_video_(PayloadMode::VIDEO_1), seed_(0x12345678u), tx_seq_(0u),
      rx_seq_(0u), on_pkt_(nullptr), on_ctrl_(nullptr), buf_I_{}, buf_Q_{},
      buf_idx_(0), pre_phase_(0), pre_reps_(1), pre_boost_(1), hdr_syms_{},
      hdr_count_(0), hdr_fail_(0), pay_cps_(0), pay_total_(0), pay_recv_(0),
      harq_round_(0), max_harq_(0), vid_fail_(0), vid_succ_(0), v1_rx_{},
      v1_idx_(0), rx_{}, sym_idx_(0), harq_inited_(false), retx_ready_(false),
      harq_Q_(g_harq_ccm_union.chase.harq_Q),
      ir_chip_I_(&g_harq_ccm_union.ir.chip_I[0][0]),
      ir_chip_Q_(&g_harq_ccm_union.ir.chip_Q[0][0]),
      ir_state_(&g_harq_ccm_union.ir.ir_state), sic_ir_enabled_(false),
      sic_expect_valid_(false), sic_walsh_amp_(300),
      wb_{} // wb 유니온 (반이중 TDM)
      ,
      orig_acc_{}, orig_I_{}, orig_Q_{},
      holo_lpi_en_(HTS_Holo_LPI::SECURE_FALSE),
      holo_lpi_seed_{}, holo_lpi_mix_q8_(128), holo_lpi_scalars_{},
      holo_lpi_rx_slot_(0u), holo_lpi_rx_chip_idx_(0u),
      holo_lpi_rx_scalars_seq_(0xFFFFFFFFu), dec_wI_{}, dec_wQ_{}
#if defined(HTS_SYNC_USE_MATCHED_FILTER)
      ,
      mf_engine_(HTS_Sys_Tier::STANDARD_CHIP), mf_enabled_(false),
      mf_ref_ready_(false), mf_synced_(false),
      mf_ref_rx_seq_(0xFFFFFFFFu), mf_estimated_offset_q16_(0)
#endif
{
#if defined(HTS_USE_HOLO_TENSOR_4D)
    holo_tensor_profile_ = k_holo_profiles[1];
    clear_holo_tensor_rx_state_();
#endif
#if defined(HTS_DIAG_PRINTF)
    // PROMPT 38: AntiJamEngine 본문은 Step3 제거 — 플래그·대체 경로만 실측
    std::printf(
        "[AJC-STATE] anti_jam_engine_present=0 reason=removed_step3 "
        "notch_path=n/a iq_mode=%d ir_mode=%d lpi_on=%d "
        "p_metrics_bound=%d nf_q16_hi=-1(tx_ctor) tx_amp=%d\n",
        static_cast<int>(iq_mode_),
        ir_mode_ ? 1 : 0,
        (holo_lpi_en_ == HTS_Holo_LPI::SECURE_TRUE) ? 1 : 0,
        (p_metrics_ != nullptr) ? 1 : 0,
        static_cast<int>(tx_amp_));
#endif
    cfo_v5a_.Init();
#if (HTS_CFO_V5A_ENABLE != 0)
    cfo_v5a_.SetEnabled(true);
#else
    cfo_v5a_.SetEnabled(false);
#endif
}
#if defined(HTS_USE_HOLO_TENSOR_4D)
uint32_t HTS_V400_Dispatcher::ensure_holo_tensor_ready_() noexcept {
    if (holo_tensor_ready_) {
        return HTS_Holo_Tensor_4D::SECURE_TRUE;
    }
    const uint32_t master_seed[4] = {
        seed_,
        seed_ ^ 0x9E3779B9u,
        seed_ ^ 0xA5A5A5A5u,
        seed_ ^ 0xC3C3C3C3u,
    };
    if (holo_tensor4d_.Initialize(master_seed, nullptr) !=
        HTS_Holo_Tensor_4D::SECURE_TRUE) {
        return HTS_Holo_Tensor_4D::SECURE_FALSE;
    }
    holo_tensor_ready_ = true;
    return HTS_Holo_Tensor_4D::SECURE_TRUE;
}

void HTS_V400_Dispatcher::clear_holo_tensor_rx_state_() noexcept {
    holo_tensor_payload_mode_ = false;
    holo_tensor_decode_failed_ = false;
    holo_tensor_rx_len_ = 0u;
    SecureMemory::secureWipe(static_cast<void*>(holo_tensor_rx_bits_),
                             sizeof(holo_tensor_rx_bits_));
    SecureMemory::secureWipe(static_cast<void*>(holo_tensor_rx_bytes_),
                             sizeof(holo_tensor_rx_bytes_));
}
#endif
HTS_V400_Dispatcher::~HTS_V400_Dispatcher() noexcept {
    // [CRIT] sizeof(*this) 통째 wipe 금지 — 멤버 역순 소멸 전 다른 서브객체
    // 손상. 순서: CCM → full_reset_(HARQ/work 버퍼) → 칩/워킹 스크래치 →
    // 시퀀스/시드 → 콜백/메트릭 무효화
    SecureMemory::secureWipe(static_cast<void *>(&g_harq_ccm_union),
                             sizeof(g_harq_ccm_union));
    SecureMemory::secureWipe(static_cast<void *>(g_sic_bits),
                             sizeof(g_sic_bits));
    full_reset_();
    SecureMemory::secureWipe(static_cast<void *>(buf_I_), sizeof(buf_I_));
    SecureMemory::secureWipe(static_cast<void *>(buf_Q_), sizeof(buf_Q_));
    SecureMemory::secureWipe(static_cast<void *>(dec_wI_), sizeof(dec_wI_));
    SecureMemory::secureWipe(static_cast<void *>(dec_wQ_), sizeof(dec_wQ_));
    SecureMemory::secureWipe(static_cast<void *>(scratch_mag_),
                             sizeof(scratch_mag_));
    SecureMemory::secureWipe(static_cast<void *>(scratch_sort_),
                             sizeof(scratch_sort_));
    holo_lpi_en_ = HTS_Holo_LPI::SECURE_FALSE;
    holo_lpi_rx_slot_ = 0u;
    holo_lpi_rx_chip_idx_ = 0u;
    holo_lpi_rx_scalars_seq_ = 0xFFFFFFFFu;
    SecureMemory::secureWipe(static_cast<void *>(holo_lpi_seed_),
                             sizeof(holo_lpi_seed_));
    SecureMemory::secureWipe(static_cast<void *>(holo_lpi_scalars_),
                             sizeof(holo_lpi_scalars_));
    SecureMemory::secureWipe(static_cast<void *>(&seed_), sizeof(seed_));
    SecureMemory::secureWipe(static_cast<void *>(&tx_seq_), sizeof(tx_seq_));
    SecureMemory::secureWipe(static_cast<void *>(&rx_seq_), sizeof(rx_seq_));
    SecureMemory::secureWipe(static_cast<void *>(hdr_syms_), sizeof(hdr_syms_));
    on_pkt_ = nullptr;
    on_ctrl_ = nullptr;
    p_metrics_ = nullptr;
}
void HTS_V400_Dispatcher::Set_Seed(uint32_t s) noexcept {
    seed_ = s;
#if defined(HTS_USE_HOLO_TENSOR_4D)
    holo_tensor4d_.Shutdown();
    holo_tensor_ready_ = false;
    clear_holo_tensor_rx_state_();
#endif
}

void HTS_V400_Dispatcher::Enable_Holo_LPI(
    const uint32_t lpi_seed[4]) noexcept {
    if (lpi_seed == nullptr) {
        Disable_Holo_LPI();
        return;
    }
    holo_lpi_seed_[0] = lpi_seed[0];
    holo_lpi_seed_[1] = lpi_seed[1];
    holo_lpi_seed_[2] = lpi_seed[2];
    holo_lpi_seed_[3] = lpi_seed[3];
    holo_lpi_rx_slot_ = 0u;
    holo_lpi_rx_chip_idx_ = 0u;
    holo_lpi_rx_scalars_seq_ = 0xFFFFFFFFu;
    holo_lpi_en_ = HTS_Holo_LPI::SECURE_TRUE;
}

void HTS_V400_Dispatcher::Disable_Holo_LPI() noexcept {
    holo_lpi_en_ = HTS_Holo_LPI::SECURE_FALSE;
    holo_lpi_rx_slot_ = 0u;
    holo_lpi_rx_chip_idx_ = 0u;
    holo_lpi_rx_scalars_seq_ = 0xFFFFFFFFu;
    SecureMemory::secureWipe(static_cast<void *>(holo_lpi_seed_),
                             sizeof(holo_lpi_seed_));
    SecureMemory::secureWipe(static_cast<void *>(holo_lpi_scalars_),
                             sizeof(holo_lpi_scalars_));
}

void HTS_V400_Dispatcher::apply_holo_lpi_inverse_rx_chip_(
    int16_t& chip_I, int16_t& chip_Q,
    uint32_t scalar_time_slot) noexcept {
    if (holo_lpi_en_ != HTS_Holo_LPI::SECURE_TRUE) {
        return;
    }
    holo_lpi_rx_slot_ = scalar_time_slot;
    if (holo_lpi_rx_scalars_seq_ != scalar_time_slot) {
        if (HTS_Holo_LPI::Generate_Scalars(holo_lpi_seed_, scalar_time_slot,
                                           holo_lpi_mix_q8_,
                                           holo_lpi_scalars_) !=
            HTS_Holo_LPI::SECURE_TRUE) {
            return;
        }
        holo_lpi_rx_scalars_seq_ = scalar_time_slot;
    }
    const int16_t sc =
        holo_lpi_scalars_[static_cast<std::size_t>(holo_lpi_rx_chip_idx_) &
                           63u];
    ++holo_lpi_rx_chip_idx_;
    if (sc == 0) {
        return;
    }
    const int64_t i64 =
        (static_cast<int64_t>(chip_I) << 13) / static_cast<int64_t>(sc);
    const int64_t q64 =
        (static_cast<int64_t>(chip_Q) << 13) / static_cast<int64_t>(sc);
    const int64_t hi = 32767;
    const int64_t lo = -32768;
    const int64_t ci = (i64 > hi) ? hi : ((i64 < lo) ? lo : i64);
    const int64_t cq = (q64 > hi) ? hi : ((q64 < lo) ? lo : q64);
    chip_I = static_cast<int16_t>(ci);
    chip_Q = static_cast<int16_t>(cq);
}
void HTS_V400_Dispatcher::Set_Packet_Callback(PacketCB cb) noexcept {
    on_pkt_ = cb;
}
void HTS_V400_Dispatcher::Set_Control_Callback(ControlCB cb) noexcept {
    on_ctrl_ = cb;
}
RxPhase HTS_V400_Dispatcher::Get_Phase() const noexcept { return phase_; }
PayloadMode HTS_V400_Dispatcher::Get_Mode() const noexcept { return cur_mode_; }
int HTS_V400_Dispatcher::Get_Video_Fail_Count() const noexcept {
    return vid_fail_;
}
int HTS_V400_Dispatcher::Get_Current_BPS64() const noexcept {
    return cur_bps64_;
}
IQ_Mode HTS_V400_Dispatcher::Get_IQ_Mode() const noexcept { return iq_mode_; }
void HTS_V400_Dispatcher::Set_IR_Mode(bool enable) noexcept {
    if (ir_mode_ != enable) {
        ir_mode_ = enable;
        full_reset_();
    }
}
bool HTS_V400_Dispatcher::Get_IR_Mode() const noexcept { return ir_mode_; }
void HTS_V400_Dispatcher::Set_IR_SIC_Enabled(bool enable) noexcept {
    if (sic_ir_enabled_ == enable) {
        return;
    }
    sic_ir_enabled_ = enable;
    sic_expect_valid_ = false;
    std::memset(g_sic_bits, 0, sizeof(g_sic_bits));
}
bool HTS_V400_Dispatcher::Get_IR_SIC_Enabled() const noexcept {
    return sic_ir_enabled_;
}
void HTS_V400_Dispatcher::Set_SIC_Walsh_Amp(int16_t amp) noexcept {
    sic_walsh_amp_ = amp;
}
void HTS_V400_Dispatcher::Set_Tx_Amp(int16_t amp) noexcept {
    if (amp < 64) {
        amp = 64;
    }
    if (amp > 1024) {
        amp = 1024;
    }
    tx_amp_ = amp;
}
HTS_TPC_Controller& HTS_V400_Dispatcher::Get_TPC() noexcept { return tpc_; }
const HTS_TPC_Controller& HTS_V400_Dispatcher::Get_TPC() const noexcept {
    return tpc_;
}
void HTS_V400_Dispatcher::Update_Adaptive_BPS(uint32_t nf) noexcept {
    // HTS_RF_Metrics + HTS_Adaptive_BPS_Controller 경로가 연결된 경우
    // current_bps의 단일 진실은 컨트롤러(히스테리시스)이다.
    // bps_from_nf(nf)로 cur_bps64_를 즉시 덮어쓰면 Tick_Adaptive_BPS()가
    // 방금 올린 BPS를 한 프레임 만에 되돌리는 이중 경로 충돌이 난다.
    if (p_metrics_ != nullptr) {
        (void)nf;
        return;
    }
    const int new_bps = FEC_HARQ::bps_from_nf(nf);
    if (new_bps >= FEC_HARQ::BPS64_MIN_OPERABLE &&
        new_bps <= FEC_HARQ::BPS64_MAX) {
        cur_bps64_ = new_bps;
    }
    // IQ 모드 전환은 Tick_Adaptive_BPS()에서만 수행 (히스테리시스 보장)
}
void HTS_V400_Dispatcher::Set_Lab_BPS64(int bps) noexcept {
    cur_bps64_ = FEC_HARQ::bps_clamp_runtime(bps);
}
void HTS_V400_Dispatcher::Set_Lab_IQ_Mode_Jam_Harness() noexcept {
    iq_mode_ = IQ_Mode::IQ_SAME;
    iq_upgrade_count_ = 0u;
}
void HTS_V400_Dispatcher::Set_RF_Metrics(HTS_RF_Metrics *p) noexcept {
    // 비소유 포인터 저장 — nullptr 허용 (Tick 무동작 모드)
    p_metrics_ = p;
}
void HTS_V400_Dispatcher::Tick_Adaptive_BPS() noexcept {
    if (p_metrics_ == nullptr) {
        return;
    }
    const RxPhase prev_phase = phase_;
    const int prev_bps = cur_bps64_;
    const IQ_Mode prev_iq = iq_mode_;
    uint32_t need_reset = 0u;
    const uint8_t bps = p_metrics_->current_bps.load(std::memory_order_acquire);
    if (bps >= static_cast<uint8_t>(FEC_HARQ::BPS64_MIN) &&
        bps <= static_cast<uint8_t>(FEC_HARQ::BPS64_MAX)) {
        const int new_bps = FEC_HARQ::bps_clamp_runtime(static_cast<int>(bps));
        if (new_bps != prev_bps) {
            cur_bps64_ = new_bps;
            need_reset |=
                static_cast<uint32_t>(prev_phase != RxPhase::WAIT_SYNC);
        }
    }
    // ── 적응형 I/Q 모드 전환 (히스테리시스) ──────────────
    const uint32_t nf = p_metrics_->ajc_nf.load(std::memory_order_acquire);
    if (nf >= NF_IQ_SAME_TH) {
        iq_mode_ = IQ_Mode::IQ_SAME;
        iq_upgrade_count_ = 0u;
        if (cur_bps64_ > FEC_HARQ::BPS64_MIN_OPERABLE) {
            cur_bps64_ = FEC_HARQ::BPS64_MIN_OPERABLE;
            need_reset |=
                static_cast<uint32_t>(prev_phase != RxPhase::WAIT_SYNC);
        }
        if (prev_iq != IQ_Mode::IQ_SAME) {
            need_reset |=
                static_cast<uint32_t>(prev_phase != RxPhase::WAIT_SYNC);
        }
    } else if (nf < NF_IQ_SPLIT_TH) {
        if (iq_upgrade_count_ < IQ_UPGRADE_GUARD) {
            iq_upgrade_count_++;
        }
        if (iq_upgrade_count_ >= IQ_UPGRADE_GUARD) {
            iq_mode_ = IQ_Mode::IQ_INDEPENDENT;
            if (cur_bps64_ < IQ_BPS_PEACETIME) {
                cur_bps64_ = IQ_BPS_PEACETIME;
                need_reset |=
                    static_cast<uint32_t>(prev_phase != RxPhase::WAIT_SYNC);
            }
            if (prev_iq != IQ_Mode::IQ_INDEPENDENT) {
                need_reset |=
                    static_cast<uint32_t>(prev_phase != RxPhase::WAIT_SYNC);
            }
        }
    } else {
        iq_upgrade_count_ = 0u;
    }
    if (ir_mode_) {
        iq_mode_ = IQ_Mode::IQ_SAME;
        iq_upgrade_count_ = 0u;
    }
    if (need_reset != 0u) {
        full_reset_();
    }
}
void HTS_V400_Dispatcher::full_reset_() noexcept {
    // WAIT_SYNC 전이는 모든 상태에서 무조건 합법
    phase_ = RxPhase::WAIT_SYNC;
    rf_settle_chips_remaining_ = 0;
    cur_mode_ = PayloadMode::UNKNOWN;
    buf_idx_ = 0;
    pre_phase_ = 0;
    first_c63_ = 0;
    m63_gap_ = 0;
    dc_est_I_ = 0;
    dc_est_Q_ = 0;
    cfo_v5a_.Init();
#if (HTS_CFO_V5A_ENABLE != 0)
    cfo_v5a_.SetEnabled(true);
#else
    cfo_v5a_.SetEnabled(false);
#endif
    cfo_v5a_last_cfo_hz_ = 0;
    cfo_v5a_last_valid_ = false;
#if defined(HTS_USE_HOLO_TENSOR_4D)
    clear_holo_tensor_rx_state_();
#endif
    tpc_.Init();
    pre_agc_.Init();
#if defined(HTS_PHASE0_WALSH_BANK)
    if (k_W63_FWHT_ROW == 255u) {
        k_W63_FWHT_ROW = calc_kw63_fwht_row_();
#if defined(HTS_DIAG_PRINTF)
        std::printf("[DIAG-SELF-CAL] k_W63_FWHT_ROW = %u (Walsh ordering 무관)\n",
                    static_cast<unsigned>(k_W63_FWHT_ROW));
#endif
    }
#endif
    p0_chip_count_ = 0;
    p0_carry_count_ = 0;
    p1_carry_pending_ = 0;
    p1_carry_prefix_ = 0;
    p1_tail_collect_rem_ = 0;
    p1_tail_idx_ = 0;
    psal_pending_ = false;
    psal_off_ = 0;
    psal_e63_ = 0;
    dominant_row_ = 63u;
    est_I_ = 0;
    est_Q_ = 0;
    est_count_ = 0;
    derot_shift_ = 17;
    std::memset(p0_buf128_I_, 0, sizeof(p0_buf128_I_));
    std::memset(p0_buf128_Q_, 0, sizeof(p0_buf128_Q_));
    std::memset(p0_carry_I_, 0, sizeof(p0_carry_I_));
    std::memset(p0_carry_Q_, 0, sizeof(p0_carry_Q_));
    wait_sync_head_ = 0;
    wait_sync_count_ = 0;
    hdr_count_ = 0;
    hdr_fail_ = 0;
    pay_recv_ = 0;
    v1_idx_ = 0;
    sym_idx_ = 0;
    harq_round_ = 0;
    harq_inited_ = false;
    SecureMemory::secureWipe(static_cast<void *>(&rx_), sizeof(rx_));
    SecureMemory::secureWipe(static_cast<void *>(v1_rx_), sizeof(v1_rx_));
    SecureMemory::secureWipe(static_cast<void *>(orig_I_), sizeof(orig_I_));
    SecureMemory::secureWipe(static_cast<void *>(orig_Q_), sizeof(orig_Q_));
    SecureMemory::secureWipe(static_cast<void *>(&orig_acc_),
                             sizeof(orig_acc_));
    SecureMemory::secureWipe(static_cast<void *>(&wb_), sizeof(wb_));
    // CCM union 전체 — file-scope `g_harq_ccm_union` 직접 참조 (Chase/IR 공용)
    SecureMemory::secureWipe(static_cast<void *>(&g_harq_ccm_union),
                             sizeof(g_harq_ccm_union));
    ir_rv_ = 0;
    sic_expect_valid_ = false;
    retx_ready_ = false;
    std::memset(g_sic_bits, 0, sizeof(g_sic_bits));
    std::memset(buf_I_, 0, sizeof(buf_I_));
    std::memset(buf_Q_, 0, sizeof(buf_Q_));
    if (ir_mode_ && ir_state_ != nullptr) {
        FEC_HARQ::IR_Init(*ir_state_);
    }
    holo_lpi_rx_chip_idx_ = 0u;
    holo_lpi_rx_scalars_seq_ = 0xFFFFFFFFu;
#if defined(HTS_SYNC_USE_MATCHED_FILTER)
    mf_reset_();
#endif
}
void HTS_V400_Dispatcher::update_derot_shift_from_est_() noexcept {
    const int32_t mI = est_I_ >> 31;
    const int32_t mQ = est_Q_ >> 31;
    const int32_t abs_eI = (est_I_ + mI) ^ mI;
    const int32_t abs_eQ = (est_Q_ + mQ) ^ mQ;
    const uint64_t est_mag64 =
        static_cast<uint64_t>(static_cast<uint32_t>(abs_eI)) +
        static_cast<uint64_t>(static_cast<uint32_t>(abs_eQ));
    if (est_mag64 == 0ull) {
        derot_shift_ = 17;
        return;
    }
    const uint32_t mag_u =
        (est_mag64 > static_cast<uint64_t>(UINT32_MAX))
            ? UINT32_MAX
            : static_cast<uint32_t>(est_mag64);
    int sh = 0;
    for (int b = 31; b >= 0; --b) {
        if ((mag_u & (1u << static_cast<unsigned>(b))) != 0u) {
            sh = b;
            break;
        }
    }
    derot_shift_ = sh;
}
// =====================================================================
//
//  CFI: key=(from<<2)|to, 16슬롯 LUT(RF_SETTLING 포함) + 클램프
// =====================================================================
uint32_t HTS_V400_Dispatcher::set_phase_(RxPhase target) noexcept {
    const uint32_t f = static_cast<uint32_t>(phase_);
    const uint32_t t = static_cast<uint32_t>(target);
    const uint32_t key = (f << 2u) | t;
    static constexpr uint8_t k_trans_legal[16] = {
        1u, 1u, 0u, 1u, 1u, 0u, 1u, 1u, 1u, 0u, 0u, 1u, 1u, 0u, 0u, 0u};
    const uint32_t k_ok = static_cast<uint32_t>(key < 16u);
    const uint32_t idx = key * k_ok + 15u * (1u - k_ok);
    const uint32_t legal_u = static_cast<uint32_t>(k_trans_legal[idx]);
    const uint32_t p = static_cast<uint32_t>(phase_);
    phase_ = static_cast<RxPhase>(t * legal_u + p * (1u - legal_u));
#if defined(HTS_DIAG_PRINTF)
    if (legal_u != 0u) {
        static int s_phase_trans = 0;
        ++s_phase_trans;
        if (s_phase_trans <= 50) {
            std::printf("[PHASE] old=%d new=%d count=%d\n",
                        static_cast<int>(p), static_cast<int>(phase_),
                        s_phase_trans);
        }
    }
#endif
    if (legal_u == 0u) {
        full_reset_();
        return PHASE_TRANSFER_MASK_FAIL;
    }
    return PHASE_TRANSFER_MASK_OK;
}
void HTS_V400_Dispatcher::fhss_abort_rx_for_hop_() noexcept {
    buf_idx_ = 0;
    wait_sync_head_ = 0;
    wait_sync_count_ = 0;
    pre_phase_ = 0;
    first_c63_ = 0;
    m63_gap_ = 0;
    p0_chip_count_ = 0;
    p0_carry_count_ = 0;
    p1_carry_pending_ = 0;
    p1_carry_prefix_ = 0;
    p1_tail_collect_rem_ = 0;
    p1_tail_idx_ = 0;
    psal_pending_ = false;
    psal_off_ = 0;
    psal_e63_ = 0;
    est_I_ = 0;
    est_Q_ = 0;
    est_count_ = 0;
    derot_shift_ = 17;
    hdr_count_ = 0;
    hdr_fail_ = 0;
    pay_recv_ = 0;
    pay_total_ = 0;
    pay_cps_ = 0;
    v1_idx_ = 0;
    sym_idx_ = 0;
    harq_round_ = 0;
    harq_inited_ = false;
    retx_ready_ = false;
    cur_mode_ = PayloadMode::UNKNOWN;
#if defined(HTS_SYNC_USE_MATCHED_FILTER)
    mf_reset_();
#endif
    // [TX-5] Hop 경계 CFO/AGC 리셋
    //        이전 채널의 CFO 위상 누적·AGC 게인이 새 채널에 잔재 방지.
    //        시뮬 검증: 리셋 O 100% / 리셋 X 0% (CFO 차이가 큰 hop 에서).
    pre_agc_.Init();
}
uint8_t HTS_V400_Dispatcher::FHSS_Derive_Channel(uint32_t seed,
                                                 uint32_t seq) noexcept {
    uint32_t x = seed ^ seq;
    x ^= x << 13u;
    x ^= x >> 17u;
    x ^= x << 5u;
    x ^= seq << 15u;
    x ^= seed >> 3u;
    x ^= (seq << 7u) ^ (seed << 11u);
    return static_cast<uint8_t>(x & static_cast<uint32_t>(0x7Fu));
}
uint8_t HTS_V400_Dispatcher::FHSS_Request_Hop_As_Tx() noexcept {
    const uint32_t in_settle =
        static_cast<uint32_t>(phase_ == RxPhase::RF_SETTLING);
    if (in_settle != 0u) {
        return static_cast<uint8_t>(0xFFu);
    }
    const uint8_t ch = FHSS_Derive_Channel(seed_, tx_seq_);
    Mock_RF_Synth_Set_Channel(ch);
    tx_seq_ = tx_seq_ + 1u;
    fhss_abort_rx_for_hop_();
    rf_settle_chips_remaining_ = FHSS_SETTLE_CHIPS;
    const uint32_t ph_ok = set_phase_(RxPhase::RF_SETTLING);
    if (ph_ok == PHASE_TRANSFER_MASK_FAIL) {
        return static_cast<uint8_t>(0xFFu);
    }
    return ch;
}
uint8_t HTS_V400_Dispatcher::FHSS_Request_Hop_As_Rx() noexcept {
    const uint32_t in_settle =
        static_cast<uint32_t>(phase_ == RxPhase::RF_SETTLING);
    if (in_settle != 0u) {
        return static_cast<uint8_t>(0xFFu);
    }
    const uint8_t ch = FHSS_Derive_Channel(seed_, rx_seq_);
    Mock_RF_Synth_Set_Channel(ch);
    rx_seq_ = rx_seq_ + 1u;
    fhss_abort_rx_for_hop_();
    rf_settle_chips_remaining_ = FHSS_SETTLE_CHIPS;
    const uint32_t ph_ok = set_phase_(RxPhase::RF_SETTLING);
    if (ph_ok == PHASE_TRANSFER_MASK_FAIL) {
        return static_cast<uint8_t>(0xFFu);
    }
    return ch;
}
bool HTS_V400_Dispatcher::FHSS_Is_Rf_Settling() const noexcept {
    return static_cast<uint32_t>(phase_) ==
           static_cast<uint32_t>(RxPhase::RF_SETTLING);
}
void HTS_V400_Dispatcher::Reset() noexcept {
    full_reset_();
    // [REMOVED Step3] ajc_.Reset(16), ajc_last_nc_ — AntiJam 엔진 제거
}
} // namespace ProtectedEngine
