// =============================================================================
// HTS_CFO_V5a.cpp — INNOViD HTS Rx CFO V5a
// Phase 1-3: Derotate / Walsh63_Dot / Energy_Multiframe (file-static).
// Phase 1-4: L&R segment autocorr estimator (Luise & Reggiannini 1995).
// Lab 전용: `/DHTS_V5A_DISABLE_PTE` 시 coarse/fine PTE(parabolic) 오프셋을 0으로
// 고정(미정의 시 기존과 동일).
// Lab 전용: `/DHTS_V5A_DISABLE_LR` 시 Estimate 루프에서 L&R(LR_Estimate_impl) 보정 0.
// Lab 전용: `/DHTS_LR_DIAG` 시 `g_lr_diag` 에 L&R 단계 스냅샷(세그먼트·R·Z·lag별 Hz).
// =============================================================================
#include "HTS_CFO_V5a.hpp"
#include "HTS_Rx_CFO_SinCos_Table.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>
#if defined(HTS_V5A_DIAG_PER_SCENARIO)
#include "HTS_CFO_V5a_T6_Scenario.hpp"
#endif

namespace hts {
namespace rx_cfo {

#if defined(HTS_V5A_DIAG_PER_SCENARIO)
enum class V5aPerScnId : unsigned {
    None = 0u,
    S1 = 1u,
    S2_0,
    S2_90,
    S2_180,
    S2_315,
    S5_0,
    S5_500,
    S5_2000,
    Count
};
static int g_v5a_per_scn_visit[static_cast<unsigned>(V5aPerScnId::Count)];

static V5aPerScnId v5a_per_scn_resolve() noexcept {
    const char* const lab = g_t6_current_scenario_label;
    if (lab == nullptr) {
        return V5aPerScnId::None;
    }
    if (std::strcmp(lab, "S1") == 0) {
        return V5aPerScnId::S1;
    }
    if (std::strcmp(lab, "S2") == 0) {
        switch (g_t6_current_phase_deg) {
        case 0:
            return V5aPerScnId::S2_0;
        case 90:
            return V5aPerScnId::S2_90;
        case 180:
            return V5aPerScnId::S2_180;
        case 315:
            return V5aPerScnId::S2_315;
        default:
            return V5aPerScnId::None;
        }
    }
    if (std::strcmp(lab, "S5") == 0) {
        switch (g_t6_current_cfo_hz) {
        case 0:
            return V5aPerScnId::S5_0;
        case 500:
            return V5aPerScnId::S5_500;
        case 2000:
            return V5aPerScnId::S5_2000;
        default:
            return V5aPerScnId::None;
        }
    }
    return V5aPerScnId::None;
}

static const char* v5a_per_scn_tag(V5aPerScnId id) noexcept {
    switch (id) {
    case V5aPerScnId::S1:
        return "S1-CLEAN";
    case V5aPerScnId::S2_0:
        return "S2-000DEG-PASS";
    case V5aPerScnId::S2_90:
        return "S2-090DEG-FAIL05";
    case V5aPerScnId::S2_180:
        return "S2-180DEG-BUG9";
    case V5aPerScnId::S2_315:
        return "S2-315DEG-PASS";
    case V5aPerScnId::S5_0:
        return "S5-0HZ-PASS";
    case V5aPerScnId::S5_500:
        return "S5-500HZ-FAIL";
    case V5aPerScnId::S5_2000:
        return "S5-2000HZ-FAIL";
    default:
        return "";
    }
}

static bool v5a_per_scn_begin_visit(V5aPerScnId* out_id,
                                    const char** out_tag) noexcept {
    *out_id = v5a_per_scn_resolve();
    *out_tag = v5a_per_scn_tag(*out_id);
    if (*out_id == V5aPerScnId::None) {
        return false;
    }
    const unsigned ui = static_cast<unsigned>(*out_id);
    if (ui >= static_cast<unsigned>(V5aPerScnId::Count)) {
        return false;
    }
    return g_v5a_per_scn_visit[ui] < 3;
}

static void v5a_per_scn_end_visit(V5aPerScnId id) noexcept {
    if (id == V5aPerScnId::None) {
        return;
    }
    const unsigned ui = static_cast<unsigned>(id);
    if (ui < static_cast<unsigned>(V5aPerScnId::Count)) {
        ++g_v5a_per_scn_visit[ui];
    }
}
#endif

// 레거시 preamble/autocorr CFO 경로(정수 구현)와 동일 — V5A_DIAG 호출 덤프에서 참조.
inline constexpr int64_t kPreambleMagApproxThreshold = 1000LL;
inline constexpr int64_t kAutocorrMag2Threshold = 1000000LL;

#if defined(HTS_V5A_DIAG)
const char* g_v5a_lab_scenario = "";
int32_t g_v5a_lab_param_hz = 0;
static int g_v5a_est_log_n = 0;
static int g_v5a_ac_log_n = 0;
static int g_v5a_set_log_n = 0;
static int g_v5a_apply_prints = 0;
static int g_v5a_burst_est = 0;
static int g_v5a_burst_ac = 0;
static int g_v5a_ac_call_dump = 0;
static int g_v5a_ac_calls = 0;
static int g_v5a_ac_skip_lag = 0;
static int g_v5a_ac_skip_mag2 = 0;
static int g_v5a_ac_ok = 0;
static bool g_v5a_have_prev_lab = false;

static void v5a_diag_ac_reset() noexcept
{
    g_v5a_ac_calls = 0;
    g_v5a_ac_skip_lag = 0;
    g_v5a_ac_skip_mag2 = 0;
    g_v5a_ac_ok = 0;
}

static void v5a_diag_ac_print_summary(const char* lab_sc, int lab_hz) noexcept
{
    const int calls = g_v5a_ac_calls;
    const double pct =
        (calls > 0) ? (100.0 * static_cast<double>(g_v5a_ac_skip_mag2) /
                       static_cast<double>(calls))
                    : 0.0;
    std::printf(
        "[V5A-EST-AC-SK] lab=%s/%dHz call=%d skip_lag=%d skip_mag2=%d ok=%d thr=%lld "
        "skip_mag2_pct=%.1f\n",
        (lab_sc != nullptr) ? lab_sc : "", lab_hz, calls, g_v5a_ac_skip_lag,
        g_v5a_ac_skip_mag2, g_v5a_ac_ok,
        static_cast<long long>(kAutocorrMag2Threshold), pct);
}

static void v5a_diag_emit_est(
    int pass,
    int coarse_idx,
    int fine_idx,
    int32_t coarse_start_hz,
    int32_t cb_cfo_hz,
    int32_t cb_refined_hz,
    int32_t fine_start_hz,
    int32_t best_cfo_hz,
    int32_t fine_refined_hz,
    int32_t lr_hz,
    int32_t total_hz) noexcept
{
    if (g_v5a_burst_est > 0) {
        --g_v5a_burst_est;
    } else {
        ++g_v5a_est_log_n;
        if ((g_v5a_est_log_n % 50) != 0) {
            return;
        }
    }
    std::printf(
        "[V5A-EST] lab=%s/%dHz pass=%d cstart=%d cb_idx=%d cb_cfo=%d cb_ref=%d "
        "fstart=%d bestCfo=%d fb_idx=%d fineRef=%d lr=%d totalHz=%d\n",
        g_v5a_lab_scenario, static_cast<int>(g_v5a_lab_param_hz), pass,
        static_cast<int>(coarse_start_hz), coarse_idx, static_cast<int>(cb_cfo_hz),
        static_cast<int>(cb_refined_hz), static_cast<int>(fine_start_hz),
        static_cast<int>(best_cfo_hz), fine_idx, static_cast<int>(fine_refined_hz),
        static_cast<int>(lr_hz), static_cast<int>(total_hz));
}

static void v5a_diag_emit_ac(
    int32_t ac_I, int32_t ac_Q, int32_t lag_chips, int32_t cfo_hz) noexcept
{
    if (g_v5a_burst_ac > 0) {
        --g_v5a_burst_ac;
    } else {
        ++g_v5a_ac_log_n;
        if ((g_v5a_ac_log_n % 50) != 0) {
            return;
        }
    }
    std::printf(
        "[V5A-EST-AC] lab=%s/%dHz ac_I=%d ac_Q=%d lag=%d cfoHz=%d\n",
        g_v5a_lab_scenario, static_cast<int>(g_v5a_lab_param_hz),
        static_cast<int>(ac_I), static_cast<int>(ac_Q),
        static_cast<int>(lag_chips), static_cast<int>(cfo_hz));
}

static void v5a_diag_emit_set(
    int32_t cfo_hz, int32_t sin_q14, int32_t cos_q14) noexcept
{
    ++g_v5a_set_log_n;
    if ((g_v5a_set_log_n % 50) != 0) {
        return;
    }
    std::printf(
        "[V5A-SET] lab=%s/%dHz apply_cfoHz=%d sin_q14=%d cos_q14=%d\n",
        g_v5a_lab_scenario, static_cast<int>(g_v5a_lab_param_hz),
        static_cast<int>(cfo_hz), static_cast<int>(sin_q14),
        static_cast<int>(cos_q14));
}

static void v5a_diag_emit_apply(
    int chip_idx,
    int32_t cos_acc_q14,
    int32_t sin_acc_q14,
    int32_t cos_step_q14,
    int32_t sin_step_q14,
    int32_t I_in,
    int32_t Q_in,
    int32_t I_out,
    int32_t Q_out) noexcept
{
    if (g_v5a_apply_prints >= 5 || chip_idx >= 64) {
        return;
    }
    ++g_v5a_apply_prints;
    std::printf(
        "[V5A-APPLY] lab=%s/%dHz chip=%d cos_acc=%d sin_acc=%d cos_step=%d sin_step=%d "
        "I:%d->%d Q:%d->%d\n",
        g_v5a_lab_scenario, static_cast<int>(g_v5a_lab_param_hz), chip_idx,
        static_cast<int>(cos_acc_q14), static_cast<int>(sin_acc_q14),
        static_cast<int>(cos_step_q14), static_cast<int>(sin_step_q14),
        static_cast<int>(I_in), static_cast<int>(I_out), static_cast<int>(Q_in),
        static_cast<int>(Q_out));
}

void V5a_Diag_Label(const char* scenario, int param_hz) noexcept
{
    if (g_v5a_have_prev_lab) {
        v5a_diag_ac_print_summary(g_v5a_lab_scenario, static_cast<int>(g_v5a_lab_param_hz));
    }
    v5a_diag_ac_reset();
    g_v5a_ac_call_dump = 0;
    g_v5a_apply_prints = 0;
    g_v5a_burst_est = 12;
    g_v5a_burst_ac = 12;
    g_v5a_have_prev_lab = true;
    g_v5a_lab_scenario = (scenario != nullptr) ? scenario : "";
    g_v5a_lab_param_hz = static_cast<int32_t>(param_hz);
    std::printf("[V5A-LAB] scenario=%s param_hz=%d\n", g_v5a_lab_scenario,
        static_cast<int>(g_v5a_lab_param_hz));
}

void V5a_Diag_Ac_Call_Pre(int32_t ac_I, int32_t ac_Q, int lag_chips) noexcept
{
    if (g_v5a_ac_call_dump >= 8) {
        return;
    }
    const int64_t mag2_pre =
        static_cast<int64_t>(ac_I) * static_cast<int64_t>(ac_I) +
        static_cast<int64_t>(ac_Q) * static_cast<int64_t>(ac_Q);
    const int ok_gate = (mag2_pre >= kAutocorrMag2Threshold) ? 1 : 0;
    std::printf(
        "[V5A-AC-CALL] lab=%s/%dHz n=%d ac_I=%d ac_Q=%d mag2=%lld lag=%d thr=%lld "
        "mag2_ok=%d\n",
        g_v5a_lab_scenario, static_cast<int>(g_v5a_lab_param_hz), g_v5a_ac_call_dump,
        static_cast<int>(ac_I), static_cast<int>(ac_Q),
        static_cast<long long>(mag2_pre), lag_chips,
        static_cast<long long>(kAutocorrMag2Threshold), ok_gate);
    ++g_v5a_ac_call_dump;
}
#endif

namespace {
#if defined(HTS_CFO_V5A_PTE_DIAG)
struct V5aPteDiagBucket {
    char tag[12];
    uint32_t estimate_calls;
    uint32_t coarse_enter;
    uint32_t fine_enter;
    uint32_t coarse_nonzero;
    uint32_t fine_nonzero;
    uint64_t coarse_abs_sum_q15;
    uint64_t fine_abs_sum_q15;
    uint64_t lr_abs_sum_hz;
    uint32_t pte_samples;
    int32_t pte_cfo_min_hz;
    int32_t pte_cfo_max_hz;
    int64_t pte_cfo_sum_hz;
    uint32_t coarse_hist[5];
    uint32_t fine_hist[5];
};

static V5aPteDiagBucket g_v5a_pte_diag[16]{};
static int g_v5a_pte_diag_count = 0;
static int g_v5a_pte_diag_cur = 0;

static inline uint32_t pte_hist_bin_from_abs_q15(int32_t abs_q15) noexcept {
    if (abs_q15 == 0) return 0u;
    if (abs_q15 <= 4096) return 1u;
    if (abs_q15 <= 8192) return 2u;
    if (abs_q15 <= 16384) return 3u;
    return 4u;
}

static int diag_find_or_add_context_(const char* tag) noexcept {
    if (tag == nullptr || tag[0] == '\0') {
        return 0;
    }
    for (int i = 0; i < g_v5a_pte_diag_count; ++i) {
        if (std::strncmp(g_v5a_pte_diag[i].tag, tag,
                         sizeof(g_v5a_pte_diag[i].tag) - 1) == 0) {
            return i;
        }
    }
    if (g_v5a_pte_diag_count >= static_cast<int>(sizeof(g_v5a_pte_diag) /
                                                 sizeof(g_v5a_pte_diag[0]))) {
        return 0;
    }
    V5aPteDiagBucket& b = g_v5a_pte_diag[g_v5a_pte_diag_count];
    std::memset(&b, 0, sizeof(b));
    std::strncpy(b.tag, tag, sizeof(b.tag) - 1);
    b.pte_cfo_min_hz = 2147483647;
    b.pte_cfo_max_hz = -2147483647 - 1;
    ++g_v5a_pte_diag_count;
    return g_v5a_pte_diag_count - 1;
}
#endif

// Same sequence as HTS_V400_Dispatcher_Internal.hpp k_w63 (Walsh-Hadamard row 63).
static constexpr int8_t kWalsh63Row63[64] = {
    +1, -1, -1, +1, -1, +1, +1, -1, -1, +1, +1, -1, +1, -1, -1, +1,
    -1, +1, +1, -1, +1, -1, -1, +1, +1, -1, -1, +1, -1, +1, +1, -1,
    -1, +1, +1, -1, +1, -1, -1, +1, +1, -1, -1, +1, -1, +1, +1, -1,
    +1, -1, -1, +1, -1, +1, +1, -1, -1, +1, +1, -1, +1, -1, -1, +1};

/// BPTE i32 → i16 saturation (constant-time). `sat_i32_to_i16` / `Apply_Per_Chip`.
static inline int16_t saturate_i32_to_i16_bpte(int32_t v) noexcept {
    const int32_t high_diff = v - 32767;
    const int32_t high_mask = high_diff >> 31;
    int32_t r = (high_mask & v) | (~high_mask & 32767);

    const int32_t low_diff = r - (-32768);
    const int32_t low_mask = low_diff >> 31;
    r = (low_mask & static_cast<int32_t>(-32768)) | (~low_mask & r);

    return static_cast<int16_t>(r);
}

static inline int16_t sat_i32_to_i16(int32_t v) noexcept {
    return saturate_i32_to_i16_bpte(v);
}

// (x*A + y*B) / 2^14 with signed rounding (int64 inner product).
static inline int32_t dot_q14_round(int32_t x, int32_t a_q14, int32_t y,
                                    int32_t b_q14) noexcept {
    const int64_t p = static_cast<int64_t>(x) * static_cast<int64_t>(a_q14) +
                      static_cast<int64_t>(y) * static_cast<int64_t>(b_q14);
    const int64_t sign_mask = p >> 63;
    const int64_t bias =
        ((static_cast<int64_t>(1) << 13) ^ sign_mask) - sign_mask;
    return static_cast<int32_t>((p + bias) >> 14);
}

// --- Q14 per-chip path (레거시 Apply/Advance 정수 수식) ---
static inline int32_t apply_int_root_q14(int64_t c2) noexcept {
    if (c2 <= 0) {
        return 0;
    }
    int32_t lo = 0;
    constexpr int32_t kQ14 = 16384;
    int32_t hi = kQ14;
    while (lo < hi) {
        const int32_t mid = (lo + hi + 1) >> 1;
        if (static_cast<int64_t>(mid) * mid <= c2) {
            lo = mid;
        } else {
            hi = mid - 1;
        }
    }
    return lo;
}

static inline void apply_renorm_q14_accum(int32_t& ca, int32_t& sa) noexcept {
    const int64_t fc = static_cast<int64_t>(ca);
    const int64_t fs = static_cast<int64_t>(sa);
    const int64_t mag2 = fc * fc + fs * fs;
    if (mag2 <= 0) {
        return;
    }
    int64_t lo = 0;
    int64_t hi = 3037000499LL;
    while (lo < hi) {
        const int64_t mid = (lo + hi + 1) >> 1;
        if (mid != 0 && mid <= mag2 / mid) {
            lo = mid;
        } else {
            hi = mid - 1;
        }
    }
    const int64_t sm = lo;
    if (sm == 0) {
        return;
    }
    constexpr int64_t k14 = 16384;
    ca = static_cast<int32_t>((fc * k14 + (sm >> 1)) / sm);
    sa = static_cast<int32_t>((fs * k14 + (sm >> 1)) / sm);
}

static inline uint32_t apply_phase_inc_q32_from_hz(int32_t cfo_hz) noexcept {
    const int64_t phase_inc_s64 =
        (-static_cast<int64_t>(cfo_hz) * 4294967296LL) /
        static_cast<int64_t>(kChipRateHz);
    return static_cast<uint32_t>(phase_inc_s64);
}

static void Derotate_impl(const int16_t* rI, const int16_t* rQ, int16_t* oI,
                          int16_t* oQ, int chips, int32_t cfo_hz) noexcept {
    const int64_t phase_inc_s64 =
        (-static_cast<int64_t>(cfo_hz) * 4294967296LL) /
        static_cast<int64_t>(kChipRateHz);
    const uint32_t phase_inc_q32 = static_cast<uint32_t>(phase_inc_s64);
    uint32_t phase_q32 = 0u;
    for (int k = 0; k < chips; ++k) {
        const int32_t cos_q14 = static_cast<int32_t>(Lookup_Cos(phase_q32));
        const int32_t sin_q14 = static_cast<int32_t>(Lookup_Sin(phase_q32));
        const int32_t x = static_cast<int32_t>(rI[k]);
        const int32_t y = static_cast<int32_t>(rQ[k]);
        const int32_t a = dot_q14_round(x, cos_q14, y, -sin_q14);
        const int32_t b = dot_q14_round(x, sin_q14, y, cos_q14);
        oI[k] = sat_i32_to_i16(a);
        oQ[k] = sat_i32_to_i16(b);
        phase_q32 += phase_inc_q32;
    }
}

static void Walsh63_Dot_impl(const int16_t* rI, const int16_t* rQ, int32_t& dI,
                             int32_t& dQ) noexcept {
    int32_t aI = 0;
    int32_t aQ = 0;
    for (int k = 0; k < kChipsPerSym; ++k) {
        const int32_t w = static_cast<int32_t>(kWalsh63Row63[k]);
        aI += static_cast<int32_t>(rI[k]) * w;
        aQ += static_cast<int32_t>(rQ[k]) * w;
    }
    dI = aI;
    dQ = aQ;
}

static int64_t Energy_Multiframe_impl(const int16_t* rI,
                                      const int16_t* rQ) noexcept {
    int32_t d0I = 0;
    int32_t d0Q = 0;
    int32_t d1I = 0;
    int32_t d1Q = 0;
    Walsh63_Dot_impl(&rI[0], &rQ[0], d0I, d0Q);
    Walsh63_Dot_impl(&rI[kChipsPerSym], &rQ[kChipsPerSym], d1I, d1Q);
    const int64_t e0 =
        static_cast<int64_t>(d0I) * d0I + static_cast<int64_t>(d0Q) * d0Q;
    const int64_t e1 =
        static_cast<int64_t>(d1I) * d1I + static_cast<int64_t>(d1Q) * d1Q;
    return e0 + e1;
}

#if defined(HTS_BYPASS_WALSH_DISPREAD)
/// Lab bypass: 64+64 칩 에너지 합(Walsh 디스프레드 없음).
static int64_t Energy_Multiframe_Raw_impl(const int16_t* rI,
                                          const int16_t* rQ) noexcept {
    int64_t e0 = 0;
    int64_t e1 = 0;
    for (int k = 0; k < kChipsPerSym; ++k) {
        const int64_t di = static_cast<int64_t>(rI[k]);
        const int64_t dq = static_cast<int64_t>(rQ[k]);
        e0 += di * di + dq * dq;
    }
    for (int k = 0; k < kChipsPerSym; ++k) {
        const int64_t di = static_cast<int64_t>(rI[kChipsPerSym + k]);
        const int64_t dq = static_cast<int64_t>(rQ[kChipsPerSym + k]);
        e1 += di * di + dq * dq;
    }
    return e0 + e1;
}
#endif

static inline int64_t V5a_energy_mf(const int16_t* rI,
                                    const int16_t* rQ) noexcept {
#if defined(HTS_BYPASS_WALSH_DISPREAD)
    return Energy_Multiframe_Raw_impl(rI, rQ);
#else
    return Energy_Multiframe_impl(rI, rQ);
#endif
}

static inline int32_t q15_mul_hz_round(int32_t offset_q15,
                                       int32_t step_hz) noexcept {
    const int64_t prod =
        static_cast<int64_t>(offset_q15) * static_cast<int64_t>(step_hz);
    const int64_t adj = (prod >= 0) ? (1LL << 14) : -(1LL << 14);
    return static_cast<int32_t>((prod + adj) >> 15);
}

static inline int32_t parabolic_offset_q15_impl(int64_t em1_s64, int64_t e0_s64,
                                                int64_t ep1_s64) noexcept {
    int64_t max_abs = em1_s64;
    if (max_abs < 0) {
        max_abs = -max_abs;
    }
    int64_t t = e0_s64;
    if (t < 0) {
        t = -t;
    }
    if (t > max_abs) {
        max_abs = t;
    }
    t = ep1_s64;
    if (t < 0) {
        t = -t;
    }
    if (t > max_abs) {
        max_abs = t;
    }

    int sh = 0;
    while (max_abs > 0x3FFFFFFFLL) {
        max_abs >>= 1;
        ++sh;
    }
    const int32_t em1 = static_cast<int32_t>(em1_s64 >> sh);
    const int32_t e0 = static_cast<int32_t>(e0_s64 >> sh);
    const int32_t ep1 = static_cast<int32_t>(ep1_s64 >> sh);

    const int32_t denom_signed = em1 - (e0 << 1) + ep1;
    const int32_t abs_denom_mask = static_cast<int32_t>(denom_signed >> 31);
    const int32_t abs_denom =
        static_cast<int32_t>((denom_signed ^ abs_denom_mask) - abs_denom_mask);

    const int64_t safe_diff = static_cast<int64_t>(abs_denom) - 1LL;
    const int32_t valid_mask = static_cast<int32_t>(~(safe_diff >> 63));

    const int32_t numer = em1 - ep1;
    const int32_t offset_q15_raw = static_cast<int32_t>(
        (static_cast<int64_t>(numer) << 14) /
        static_cast<int64_t>(abs_denom + (1 - (valid_mask & 1))));
    const int32_t offset_q15_signed =
        static_cast<int32_t>((offset_q15_raw ^ abs_denom_mask) - abs_denom_mask);
    int32_t offset_q15 = static_cast<int32_t>(offset_q15_signed & valid_mask);

    if (offset_q15 > 32768) {
        offset_q15 = 32768;
    }
    if (offset_q15 < -32768) {
        offset_q15 = -32768;
    }
    return offset_q15;
}

// --- Integer atan2·Q12 (레거시 동일 LUT): ARM L&R + Holo autocorr ---
static inline int32_t isc_atan_frac_q12(int32_t y, int32_t x) noexcept {
    if (x <= 0 || y < 0 || y > x) {
        return 0;
    }
    if (y == 0) {
        return 0;
    }
    static constexpr int16_t kLut[17] = {
        0,    256,  511,  763,  1018, 1266, 1508, 1741,
        1965, 2178, 2380, 2571, 2749, 2915, 3068, 3208, 3217};
    const int64_t ratio_q14 =
        (static_cast<int64_t>(y) << 14) / static_cast<int64_t>(x);
    const int64_t scaled = ratio_q14 * 16;
    int32_t idx = static_cast<int32_t>(scaled >> 14);
    if (idx >= 16) {
        return static_cast<int32_t>(kLut[16]);
    }
    const int32_t base = static_cast<int32_t>(kLut[idx]);
    const int32_t next = static_cast<int32_t>(kLut[idx + 1]);
    const int32_t frac = static_cast<int32_t>(scaled & ((1 << 14) - 1));
    return base + static_cast<int32_t>(
               (static_cast<int64_t>(next - base) * frac) >> 14);
}

// --- atan primary branch [0, π/4): PTE on legacy kLut (BPTE idx clamp, linear 동일 idx) ---
static inline int32_t isc_atan_frac_pte_q12(int32_t v, int32_t u) noexcept {
    static constexpr int16_t kLut[17] = {
        0,    256,  511,  763,  1018, 1266, 1508, 1741,
        1965, 2178, 2380, 2571, 2749, 2915, 3068, 3208, 3217};
    const int64_t ratio_q14 =
        (static_cast<int64_t>(v) << 14) / static_cast<int64_t>(u);
    const int64_t scaled = ratio_q14 * 16;
    int32_t idx = static_cast<int32_t>(scaled >> 14);
    const int32_t t_hi = (16 - idx) >> 31;
    idx = (idx & ~t_hi) | (16 & t_hi);
    const int32_t dz = idx ^ 16;
    const int32_t is_lut16 = ~((dz | -dz) >> 31);
    const int32_t t_lo = (idx - 1) >> 31;
    const int32_t lo_i = (idx - 1) & ~t_lo;
    const int32_t sum = idx + 1;
    const int32_t ov2 = (16 - sum) >> 31;
    const int32_t hi_i = (sum & ~ov2) | (16 & ov2);
    const int32_t y_lo = static_cast<int32_t>(kLut[lo_i]);
    const int32_t y_md = static_cast<int32_t>(kLut[idx]);
    const int32_t y_hi = static_cast<int32_t>(kLut[hi_i]);
    const int32_t frac = static_cast<int32_t>(scaled & ((1 << 14) - 1));
    const int32_t a2 = y_hi - y_lo;
    const int32_t b2 = y_hi - (y_md << 1) + y_lo;
    const int64_t t1 = (static_cast<int64_t>(a2) * static_cast<int64_t>(frac)) >> 14;
    const int64_t x_sq =
        (static_cast<int64_t>(frac) * static_cast<int64_t>(frac)) >> 14;
    const int64_t t2 = (static_cast<int64_t>(b2) * x_sq) >> 14;
    const int32_t pte =
        y_md + static_cast<int32_t>(t1 + t2);
    return (pte & ~is_lut16) |
           (static_cast<int32_t>(kLut[16]) & is_lut16);
}

// --- Full-plane atan2 Q12: BPTE masks only (no if/else/?:) ---
static inline int32_t isc_atan2_q12_dpte(int32_t y, int32_t x) noexcept {
    const int32_t nz = static_cast<int32_t>(
        ((static_cast<int64_t>(x) | static_cast<int64_t>(y)) |
         (-(static_cast<int64_t>(x) | static_cast<int64_t>(y)))) >>
        63);
    const int32_t x_mask = x >> 31;
    const int32_t y_mask = y >> 31;
    const int32_t ax = (x ^ x_mask) - x_mask;
    const int32_t ay = (y ^ y_mask) - y_mask;
    const int32_t swap_gt = (ax - ay) >> 31;
    const int32_t u = (ax & ~swap_gt) | (ay & swap_gt);
    const int32_t v = (ay & ~swap_gt) | (ax & swap_gt);
    const int32_t uz_m = (u | -u) >> 31;
    const int32_t u_safe = u + 1 + uz_m;
    const int32_t sgn_y = y >> 31;
    const int32_t ret_u0 = (6434 ^ sgn_y) - sgn_y;
    const int32_t ang_pte = isc_atan_frac_pte_q12(v, u_safe);
    const int32_t ang_core = (ang_pte & uz_m) | (ret_u0 & ~uz_m);
    const int32_t ang_sw = (ang_core & ~swap_gt) | ((6434 - ang_core) & swap_gt);
    const int32_t ang_x = (ang_sw & ~x_mask) | ((12868 - ang_sw) & x_mask);
    const int32_t ang_y = (ang_x & ~y_mask) | ((-ang_x) & y_mask);
    return ang_y & nz;
}

static inline int32_t atan2_dpte_q15(int64_t y, int64_t x) noexcept {
    while ((y > 0x7FFFFFFFLL) || (y < -0x80000000LL) ||
           (x > 0x7FFFFFFFLL) || (x < -0x80000000LL)) {
        y >>= 1;
        x >>= 1;
    }
    const int32_t phase_q12 =
        isc_atan2_q12_dpte(static_cast<int32_t>(y), static_cast<int32_t>(x));
    return static_cast<int32_t>((static_cast<int64_t>(phase_q12) * 32768LL) /
                                12868LL);
}

static inline int32_t isc_atan2_q12(int32_t y, int32_t x) noexcept {
    if (x == 0 && y == 0) {
        return 0;
    }
    const int32_t ax = (x < 0) ? -x : x;
    const int32_t ay = (y < 0) ? -y : y;
    const bool swap = ay > ax;
    const int32_t u = swap ? ay : ax;
    const int32_t v = swap ? ax : ay;
    if (u == 0) {
        return (y >= 0) ? 6434 : -6434;
    }
    int32_t ang = isc_atan_frac_q12(v, u);
    if (swap) {
        ang = 6434 - ang;
    }
    if (x < 0) {
        ang = 12868 - ang;
    }
    if (y < 0) {
        ang = -ang;
    }
    return ang;
}

// L&R: atan2 → phase_q15 (arg(Z) × 32768 / π)
#if defined(HTS_PLATFORM_ARM)

static inline int32_t Atan2_To_Q15(int64_t y, int64_t x) noexcept {
    while ((y > 0x7FFFFFFFLL) || (y < -0x80000000LL) ||
           (x > 0x7FFFFFFFLL) || (x < -0x80000000LL)) {
        y >>= 1;
        x >>= 1;
    }
    const int32_t phase_q12 =
        isc_atan2_q12(static_cast<int32_t>(y), static_cast<int32_t>(x));
    return static_cast<int32_t>((static_cast<int64_t>(phase_q12) * 32768LL) /
                                12868LL);
}

#else

static inline int32_t Atan2_To_Q15(int64_t y, int64_t x) noexcept {
    const double yd = static_cast<double>(y);
    const double xd = static_cast<double>(x);
    const double phase_rad = std::atan2(yd, xd);
    constexpr double kInvPi = 0.31830988618379067154;
    const double phase_q15_d = phase_rad * 32768.0 * kInvPi;
    if (phase_q15_d > 32767.0) {
        return 32767;
    }
    if (phase_q15_d < -32768.0) {
        return -32768;
    }
    return static_cast<int32_t>(std::llround(phase_q15_d));
}

#endif

/// 레거시 autocorr CFO: block atan2·Q12 후 /lag.
static int32_t Autocorr_Block_Atan2_Q12(int32_t ac_I, int32_t ac_Q) noexcept {
    int64_t y = static_cast<int64_t>(ac_Q);
    int64_t x = static_cast<int64_t>(ac_I);
    while ((y > 0x7FFFFFFFLL) || (y < -0x80000000LL) ||
           (x > 0x7FFFFFFFLL) || (x < -0x80000000LL)) {
        y >>= 1;
        x >>= 1;
    }
    return isc_atan2_q12(static_cast<int32_t>(y), static_cast<int32_t>(x));
}

#if defined(HTS_USE_CLASSIC_LR)
/// 인접 세그먼트(M-1) 미적분 L&R: Σ_k Z_k·Z_{k-1}^* → atan2 → Hz.
/// 세그먼트 Z는 **칩 합**(Walsh 가중 없음). 비모호 범위는 L=kLR_SegSize 칩에 대해
/// ±Fs/(2L) 근방(정수식은 `atan2_dpte_q15`·분모와 함께 T6에서 검증).
static int32_t LR_Classic_Estimate_impl(const int16_t* rI,
                                        const int16_t* rQ) noexcept {
    if (rI == nullptr || rQ == nullptr) {
        return 0;
    }

    int64_t Z_re[kLR_NumSeg]{};
    int64_t Z_im[kLR_NumSeg]{};

    for (int k = 0; k < kLR_NumSeg; ++k) {
        int64_t sum_I = 0;
        int64_t sum_Q = 0;
        const int start = k * kLR_SegSize;
        for (int n = 0; n < kLR_SegSize; ++n) {
            sum_I += static_cast<int64_t>(rI[start + n]);
            sum_Q += static_cast<int64_t>(rQ[start + n]);
        }
        Z_re[k] = sum_I;
        Z_im[k] = sum_Q;
    }

    int64_t sum_re = 0;
    int64_t sum_im = 0;
    for (int k = 1; k < kLR_NumSeg; ++k) {
        const int64_t re_part =
            Z_re[k] * Z_re[k - 1] + Z_im[k] * Z_im[k - 1];
        const int64_t im_part =
            Z_im[k] * Z_re[k - 1] - Z_re[k] * Z_im[k - 1];
        sum_re += re_part;
        sum_im += im_part;
    }

    const int32_t angle_q15 = atan2_dpte_q15(sum_im, sum_re);
    constexpr int64_t kDenLrClassic =
        2LL * 32768LL * static_cast<int64_t>(kLR_SegSize);
    const int64_t numer =
        static_cast<int64_t>(angle_q15) *
        static_cast<int64_t>(kChipRateHz);
    const int32_t f_est = static_cast<int32_t>(
        (numer >= 0) ? ((numer + kDenLrClassic / 2) / kDenLrClassic)
                     : ((numer - kDenLrClassic / 2) / kDenLrClassic));
    return f_est;
}
#endif

// Luise & Reggiannini (1995): Δf = arg(Z) / (π·(M+1)·T_seg)
//                         = phase_q15 × 10^7 / 26214400  (Hz), M=4, T_seg=16 µs.
static int32_t LR_Estimate_impl(const int16_t* rI,
                                const int16_t* rQ) noexcept {
    int32_t seg_I[kLR_NumSeg];
    int32_t seg_Q[kLR_NumSeg];

    for (int s = 0; s < kLR_NumSeg; ++s) {
        int32_t aI = 0;
        int32_t aQ = 0;
        const int base = s * kLR_SegSize;
        for (int k = 0; k < kLR_SegSize; ++k) {
            const int walsh_idx = (base + k) & (kChipsPerSym - 1);
            const int8_t w = kWalsh63Row63[walsh_idx];
            aI += static_cast<int32_t>(rI[base + k]) * static_cast<int32_t>(w);
            aQ += static_cast<int32_t>(rQ[base + k]) * static_cast<int32_t>(w);
        }
        seg_I[s] = aI;
        seg_Q[s] = aQ;
    }

#if defined(HTS_LR_DIAG)
    for (int s = 0; s < kLR_NumSeg; ++s) {
        g_lr_diag.seg_I[s] = static_cast<int64_t>(seg_I[s]);
        g_lr_diag.seg_Q[s] = static_cast<int64_t>(seg_Q[s]);
    }
    int64_t R_re_lag[kLR_MaxLag]{};
    int64_t R_im_lag[kLR_MaxLag]{};
#endif

    int64_t Z_re = 0;
    int64_t Z_im = 0;

    for (int lag = 1; lag <= kLR_MaxLag; ++lag) {
        int64_t acc_re = 0;
        int64_t acc_im = 0;
        for (int n = 0; n + lag < kLR_NumSeg; ++n) {
            const int64_t aI = seg_I[n + lag];
            const int64_t aQ = seg_Q[n + lag];
            const int64_t bI = seg_I[n];
            const int64_t bQ = seg_Q[n];
            const int64_t cr = aI * bI + aQ * bQ;
            const int64_t ci = aQ * bI - aI * bQ;
            acc_re += cr;
            acc_im += ci;
            Z_re += cr;
            Z_im += ci;
        }
#if defined(HTS_LR_DIAG)
        R_re_lag[lag - 1] = acc_re;
        R_im_lag[lag - 1] = acc_im;
#endif
    }

    const int32_t phase_q15 = Atan2_To_Q15(Z_im, Z_re);
    const int32_t lr_hz = static_cast<int32_t>(
        (static_cast<int64_t>(phase_q15) * 10000000LL) / 26214400LL);

#if defined(HTS_LR_DIAG)
    for (int d = 0; d < kLR_MaxLag; ++d) {
        g_lr_diag.R_re[d] = R_re_lag[d];
        g_lr_diag.R_im[d] = R_im_lag[d];
        const int32_t ph_d = Atan2_To_Q15(R_im_lag[d], R_re_lag[d]);
        g_lr_diag.lag_cfo_hz[d] = static_cast<int32_t>(
            (static_cast<int64_t>(ph_d) * 10000000LL) / 26214400LL);
    }
    g_lr_diag.Z_re = Z_re;
    g_lr_diag.Z_im = Z_im;
    g_lr_diag.Z_phase_q15 = phase_q15;
    g_lr_diag.lr_cfo_hz = lr_hz;
#endif

    return lr_hz;
}

#if defined(HTS_USE_MNM_WALSH)
// M&M (1997) style weights Q15, M=16 chip lags (constexpr table).
#define HTS_MNM_W_ROW(d_) \
    static_cast<int32_t>( \
        (3LL * (256LL - static_cast<int64_t>(16 - (d_)) * \
                            static_cast<int64_t>(16 - (d_))) * \
         32768LL) / \
        4096LL)
static constexpr int32_t kMnM_W_Q15[16] = {
    HTS_MNM_W_ROW(1),  HTS_MNM_W_ROW(2),  HTS_MNM_W_ROW(3),
    HTS_MNM_W_ROW(4),  HTS_MNM_W_ROW(5),  HTS_MNM_W_ROW(6),
    HTS_MNM_W_ROW(7),  HTS_MNM_W_ROW(8),  HTS_MNM_W_ROW(9),
    HTS_MNM_W_ROW(10), HTS_MNM_W_ROW(11), HTS_MNM_W_ROW(12),
    HTS_MNM_W_ROW(13), HTS_MNM_W_ROW(14), HTS_MNM_W_ROW(15),
    HTS_MNM_W_ROW(16)};
#undef HTS_MNM_W_ROW

// Walsh-aware chip-domain M&M + atan2 DPTE (parallel to LR_Estimate_impl).
static int32_t MnM_Walsh_Estimate_dpte_impl(const int16_t* rI,
                                            const int16_t* rQ) noexcept {
    int32_t pure_I[kPreambleChips];
    int32_t pure_Q[kPreambleChips];
    for (int n = 0; n < kPreambleChips; ++n) {
        const int32_t sym0_m = static_cast<int32_t>(n - 64) >> 31;
        const int32_t w_raw =
            static_cast<int32_t>(kWalsh63Row63[static_cast<size_t>(n) & 63u]);
        const int32_t w_eff = (w_raw & sym0_m) | (1 & ~sym0_m);
        const int32_t sm_eff = (w_eff >> 31) & sym0_m;
        const int32_t ri = static_cast<int32_t>(rI[n]);
        const int32_t rq = static_cast<int32_t>(rQ[n]);
        pure_I[n] = (ri ^ sm_eff) - sm_eff;
        pure_Q[n] = (rq ^ sm_eff) - sm_eff;
    }
    int64_t R_re[16]{};
    int64_t R_im[16]{};
    for (int d = 1; d <= 16; ++d) {
        int64_t acc_re = 0;
        int64_t acc_im = 0;
        const int n_max = kPreambleChips - d;
        for (int n = 0; n < n_max; ++n) {
            const int64_t aI = static_cast<int64_t>(pure_I[n + d]);
            const int64_t aQ = static_cast<int64_t>(pure_Q[n + d]);
            const int64_t bI = static_cast<int64_t>(pure_I[n]);
            const int64_t bQ = static_cast<int64_t>(pure_Q[n]);
            acc_re += aI * bI + aQ * bQ;
            acc_im += aQ * bI - aI * bQ;
        }
        R_re[d - 1] = acc_re;
        R_im[d - 1] = acc_im;
    }
    int64_t Z_re = 0;
    int64_t Z_im = 0;
    for (int d = 0; d < 16; ++d) {
        const int32_t rr = static_cast<int32_t>(R_re[d] >> 16);
        const int32_t ri = static_cast<int32_t>(R_im[d] >> 16);
        Z_re += static_cast<int64_t>(rr) * static_cast<int64_t>(kMnM_W_Q15[d]);
        Z_im += static_cast<int64_t>(ri) * static_cast<int64_t>(kMnM_W_Q15[d]);
    }
    const int32_t phase_q15 = atan2_dpte_q15(Z_im, Z_re);
    return static_cast<int32_t>(
        (static_cast<int64_t>(phase_q15) * 10000000LL) / 26214400LL);
}
#endif

}  // namespace

#if defined(HTS_USE_PACD)
int32_t Atan2_Correlation_Dpte_Q15(int64_t y, int64_t x) noexcept {
    return atan2_dpte_q15(y, x);
}
#endif

#if defined(HTS_LR_DIAG)
LrDiagSnapshot g_lr_diag{};
#endif

CFO_V5a::CFO_V5a() noexcept
    : last_cfo_hz_(0),
      runtime_enabled_(false)
#if defined(HTS_V5A_TESTONLY)
      ,
      last_cb_bin_(-1),
      last_fb_bin_(-1)
#endif
{
    for (int i = 0; i < kPreambleChips; ++i) {
        work_I_[i] = 0;
        work_Q_[i] = 0;
    }
    for (int j = 0; j < 32; ++j) {
        fine_energies_[j] = 0;
    }
}

void CFO_V5a::Init() noexcept {
    Build_SinCos_Table();
    last_cfo_hz_ = 0;
    last_apply_gate_mag_ = 0;
    last_apply_gate_autocorr_ = false;
#if defined(HTS_V5A_TESTONLY)
    last_cb_bin_ = -1;
    last_fb_bin_ = -1;
#endif
    runtime_enabled_ = (HTS_CFO_V5A_ENABLE != 0);
    Set_Apply_Cfo(0);

    // [TASK-017 Stage A] one-shot coarse grid (pass-0, cfo_estimate=0 동일)
    {
        static bool s_task017_dump_done = false;
        if (!s_task017_dump_done) {
            s_task017_dump_done           = true;
            const int32_t coarse_center = 0;
            const int32_t coarse_start  = coarse_center - kCfoRangeHz;
            std::printf(
                "[TASK017-COARSE-BANK] range_hz=%d step=%d banks=%d\n",
                static_cast<int>(kCfoRangeHz),
                static_cast<int>(kCfoCoarseStep),
                static_cast<int>(kCfoCoarseBanks));
            for (int i = 0; i < kCfoCoarseBanks; ++i) {
                const int32_t hz = coarse_start + i * kCfoCoarseStep;
                std::printf("[TASK017-BANK] idx=%d hz=%d\n", i,
                            static_cast<int>(hz));
            }
            std::fflush(stdout);
        }
    }
}

bool CFO_V5a::IsApplyAllowed() const noexcept {
    const int64_t thr = last_apply_gate_autocorr_ ? kAutocorrMag2Threshold
                                                   : kPreambleMagApproxThreshold;
    const int64_t diff = last_apply_gate_mag_ - thr;
    const int32_t mask = static_cast<int32_t>(~(diff >> 63));
    return mask != 0;
}

bool CFO_V5a::IsApplyDriveActive() const noexcept {
    return apply_sin_per_q14_ != 0 || apply_cos_per_q14_ != 16384;
}

CFO_Result CFO_V5a::Estimate(const int16_t* rx_I,
                             const int16_t* rx_Q) noexcept {
    CFO_Result res{};
    res.cfo_hz = 0;
    res.peak_energy = 0;
    res.valid = false;
    last_cfo_hz_ = 0;

    if (rx_I == nullptr || rx_Q == nullptr) {
        last_apply_gate_mag_ = 0;
        last_apply_gate_autocorr_ = false;
#if defined(HTS_V5A_TESTONLY)
        last_cb_bin_ = -1;
        last_fb_bin_ = -1;
#endif
        return res;
    }

#if defined(HTS_V5A_DIAG_PER_SCENARIO)
    V5aPerScnId per_sid = V5aPerScnId::None;
    const char* per_tag = nullptr;
    const bool per_scn_active = v5a_per_scn_begin_visit(&per_sid, &per_tag);
#endif

#if defined(HTS_V5A_STEP_A_DIAG)
    {
        static int s_input_dump_n = 0;
        constexpr int kInputDumpLimit = 3;
        if (s_input_dump_n < kInputDumpLimit) {
#if defined(HTS_V5A_DIAG)
            std::printf("[V5A-A-INPUT] call=%d lab=%s/%dHz chips=%d\n",
                        s_input_dump_n, g_v5a_lab_scenario,
                        static_cast<int>(g_v5a_lab_param_hz), kPreambleChips);
#else
            std::printf("[V5A-A-INPUT] call=%d chips=%d\n", s_input_dump_n,
                        kPreambleChips);
#endif
            for (int n = 0; n < kPreambleChips; ++n) {
                std::printf("[V5A-A-IN] n=%d I=%d Q=%d\n", n,
                            static_cast<int>(rx_I[n]),
                            static_cast<int>(rx_Q[n]));
            }
            ++s_input_dump_n;
        }
    }
#endif

#if defined(HTS_V5A_DIAG_PER_SCENARIO)
    if (per_scn_active) {
        const unsigned pui = static_cast<unsigned>(per_sid);
        const int seq = g_v5a_per_scn_visit[pui];
        std::printf(
            "[V5A-PER-SCN-INPUT] tag=%s seq=%d chips=%d\n", per_tag, seq,
            kPreambleChips);
        for (int n = 0; n < 16 && n < kPreambleChips; ++n) {
            std::printf(
                "[V5A-PER-SCN-IN] tag=%s n=%d I=%d Q=%d\n", per_tag, n,
                static_cast<int>(rx_I[n]), static_cast<int>(rx_Q[n]));
        }
        for (int n = 64; n < 80 && n < kPreambleChips; ++n) {
            std::printf(
                "[V5A-PER-SCN-IN-MID] tag=%s n=%d I=%d Q=%d\n", per_tag, n,
                static_cast<int>(rx_I[n]), static_cast<int>(rx_Q[n]));
        }
    }
#endif

#if defined(HTS_V5A_STEP_B1_DIAG) && defined(HTS_V5A_DIAG)
    {
        static int s_b1_dump_n = 0;
        constexpr int kB1DumpLimit = 5;
        const bool is_target =
            (g_v5a_lab_scenario != nullptr) &&
            (std::strcmp(g_v5a_lab_scenario, "S5") == 0) &&
            (g_v5a_lab_param_hz == 500);
        if (is_target && s_b1_dump_n < kB1DumpLimit) {
            std::printf("[V5A-B1-INPUT] call=%d lab=%s/%dHz chips=%d\n",
                        s_b1_dump_n, g_v5a_lab_scenario,
                        static_cast<int>(g_v5a_lab_param_hz), kPreambleChips);
            for (int n = 0; n < kPreambleChips; ++n) {
                std::printf("[V5A-B1-IN] n=%d I=%d Q=%d\n", n,
                            static_cast<int>(rx_I[n]),
                            static_cast<int>(rx_Q[n]));
            }
            ++s_b1_dump_n;
        }
    }
#endif

#if defined(HTS_V5A_STEP_F0_DIAG) && defined(HTS_V5A_DIAG)
    {
        const char* const lab = g_v5a_lab_scenario;
        const int32_t hz = g_v5a_lab_param_hz;
        const bool is_s5 = (lab != nullptr) && (std::strcmp(lab, "S5") == 0);
        const bool is_target =
            is_s5 && ((hz == 0) || (hz == 100) || (hz == 500));

        static int s_f0_dump_n_0 = 0;
        static int s_f0_dump_n_100 = 0;
        static int s_f0_dump_n_500 = 0;
        constexpr int kF0DumpLimit = 3;

        int* p_count = nullptr;
        if (hz == 0) {
            p_count = &s_f0_dump_n_0;
        } else if (hz == 100) {
            p_count = &s_f0_dump_n_100;
        } else if (hz == 500) {
            p_count = &s_f0_dump_n_500;
        }

        if (is_target && (p_count != nullptr) && (*p_count < kF0DumpLimit)) {
            std::printf("[V5A-F0-INPUT] call=%d lab=%s/%dHz chips=%d\n", *p_count,
                        lab, static_cast<int>(hz), kPreambleChips);
            for (int n = 0; n < kPreambleChips; ++n) {
                std::printf("[V5A-F0-IN] hz=%d call=%d n=%d I=%d Q=%d\n",
                            static_cast<int>(hz), *p_count, n,
                            static_cast<int>(rx_I[n]),
                            static_cast<int>(rx_Q[n]));
            }
            ++(*p_count);
        }
    }
#endif

    {
        int32_t d0I = 0;
        int32_t d0Q = 0;
        int32_t d1I = 0;
        int32_t d1Q = 0;
        Walsh63_Dot_impl(rx_I, rx_Q, d0I, d0Q);
        Walsh63_Dot_impl(&rx_I[kChipsPerSym], &rx_Q[kChipsPerSym], d1I, d1Q);
        const int64_t cos_delta = static_cast<int64_t>(d0I) * d1I +
                                  static_cast<int64_t>(d0Q) * d1Q;
        const int64_t sin_delta = static_cast<int64_t>(d0Q) * d1I -
                                  static_cast<int64_t>(d0I) * d1Q;
        const int64_t ac =
            (cos_delta < 0) ? -cos_delta : cos_delta;
        const int64_t as =
            (sin_delta < 0) ? -sin_delta : sin_delta;
        const int64_t mag_approx =
            (ac > as) ? (ac + (as >> 1)) : (as + (ac >> 1));
        last_apply_gate_mag_ = mag_approx;
        last_apply_gate_autocorr_ = false;
    }

    int32_t cfo_estimate = 0;
    int64_t final_peak_e = 0;
#if defined(HTS_V5A_DIAG)
    int trace_cb = 0;
    int trace_fb = 0;
    int32_t trace_fine_ref = 0;
    int32_t trace_lr = 0;
    int trace_pass = 0;
    int32_t trace_coarse_start = 0;
    int32_t trace_cb_cfo = 0;
    int32_t trace_cb_ref = 0;
    int32_t trace_fine_start = 0;
    int32_t trace_best_cfo = 0;
#endif
#if defined(HTS_CFO_V5A_PTE_DIAG)
    V5aPteDiagBucket* diag = &g_v5a_pte_diag[g_v5a_pte_diag_cur];
    diag->estimate_calls++;
#endif

#if defined(HTS_BYPASS_ITERATIVE)
    const int v5a_pass_limit = 1;
#else
    const int v5a_pass_limit = kCfoIterPasses;
#endif
    for (int pass = 0; pass < v5a_pass_limit; ++pass) {
        int64_t coarse_energies[kCfoCoarseBanks]{};
        const int32_t coarse_center = cfo_estimate;
        const int32_t coarse_start = coarse_center - kCfoRangeHz;

        int64_t cb_e = -1;
        int32_t cb_cfo = coarse_start;
        int cb_best_idx = 0;

#if defined(HTS_BYPASS_COARSE_SEARCH)
        {
            const int mid_b = kCfoCoarseBanks / 2;
            cb_best_idx = mid_b;
            cb_cfo = coarse_start + mid_b * kCfoCoarseStep;
            cb_e = 0;
            for (int b = 0; b < kCfoCoarseBanks; ++b) {
                coarse_energies[b] = 0;
            }
        }
#else
        for (int b = 0; b < kCfoCoarseBanks; ++b) {
            const int32_t cfo_total = coarse_start + b * kCfoCoarseStep;
            Derotate_impl(rx_I, rx_Q, work_I_, work_Q_, kPreambleChips,
                          cfo_total);
            const int64_t e = V5a_energy_mf(work_I_, work_Q_);
            coarse_energies[b] = e;
            // strict e > cb_e (tie → 유지), BPTE — `if (e > cb_e)` 와 비트 동치
            const int64_t diff_ce = e - cb_e;
            const int64_t update_ce = ~((diff_ce - 1) >> 63);
            cb_e = (~update_ce & cb_e) | (update_ce & e);
            cb_cfo = static_cast<int32_t>(
                (~update_ce & static_cast<int64_t>(cb_cfo)) |
                (update_ce & static_cast<int64_t>(cfo_total)));
            cb_best_idx = static_cast<int>(
                (~update_ce & static_cast<int64_t>(cb_best_idx)) |
                (update_ce & static_cast<int64_t>(b)));
        }
#endif

#if defined(HTS_V5A_STEP_A_DIAG)
        {
            static int s_coarse_dump_n = 0;
            constexpr int kCoarseDumpLimit = 6;
            if (s_coarse_dump_n < kCoarseDumpLimit) {
                std::printf(
                    "[V5A-A-COARSE] dump=%d pass=%d cstart=%d cb_idx=%d "
                    "cb_cfo=%d cb_e=%lld\n",
                    s_coarse_dump_n, pass, static_cast<int>(coarse_start),
                    cb_best_idx, static_cast<int>(cb_cfo),
                    static_cast<long long>(cb_e));
                for (int b = 0; b < kCfoCoarseBanks; ++b) {
                    const int32_t cfo_b = coarse_start + b * kCfoCoarseStep;
                    std::printf("[V5A-A-CB] dump=%d b=%d cfo=%d e=%lld\n",
                                s_coarse_dump_n, b, static_cast<int>(cfo_b),
                                static_cast<long long>(coarse_energies[b]));
                }
                ++s_coarse_dump_n;
            }
        }
#endif

#if defined(HTS_V5A_DIAG_PER_SCENARIO)
        if (per_scn_active) {
            const unsigned pui = static_cast<unsigned>(per_sid);
            const int seq = g_v5a_per_scn_visit[pui];
            std::printf(
                "[V5A-PER-SCN-COARSE] tag=%s seq=%d pass=%d cstart=%d cb_idx=%d "
                "cb_cfo=%d cb_e=%lld\n",
                per_tag, seq, pass, static_cast<int>(coarse_start), cb_best_idx,
                static_cast<int>(cb_cfo), static_cast<long long>(cb_e));
            for (int b = 0; b < kCfoCoarseBanks; ++b) {
                const int32_t cfo_b = coarse_start + b * kCfoCoarseStep;
                std::printf(
                    "[V5A-PER-SCN-CB] tag=%s seq=%d pass=%d b=%d cfo=%d e=%lld\n",
                    per_tag, seq, pass, b, static_cast<int>(cfo_b),
                    static_cast<long long>(coarse_energies[b]));
            }
        }
#endif

#if defined(HTS_V5A_STEP_B1_DIAG) && defined(HTS_V5A_DIAG)
        {
            static int s_b1_coarse_dump_n = 0;
            constexpr int kB1CoarseDumpLimit = 10;
            const bool is_target =
                (g_v5a_lab_scenario != nullptr) &&
                (std::strcmp(g_v5a_lab_scenario, "S5") == 0) &&
                (g_v5a_lab_param_hz == 500);
            if (is_target && s_b1_coarse_dump_n < kB1CoarseDumpLimit) {
                std::printf(
                    "[V5A-B1-COARSE] dump=%d pass=%d cstart=%d cb_idx=%d cb_cfo=%d "
                    "cb_e=%lld\n",
                    s_b1_coarse_dump_n, pass, static_cast<int>(coarse_start),
                    cb_best_idx, static_cast<int>(cb_cfo),
                    static_cast<long long>(cb_e));
                for (int b = 0; b < kCfoCoarseBanks; ++b) {
                    const int32_t cfo_b = coarse_start + b * kCfoCoarseStep;
                    std::printf("[V5A-B1-CB] dump=%d b=%d cfo=%d e=%lld\n",
                                s_b1_coarse_dump_n, b, static_cast<int>(cfo_b),
                                static_cast<long long>(coarse_energies[b]));
                }
                ++s_b1_coarse_dump_n;
            }
        }
#endif

        int32_t coarse_offset_q15 = 0;
#if !defined(HTS_BYPASS_COARSE_SEARCH)
        if (cb_best_idx > 0 && cb_best_idx < (kCfoCoarseBanks - 1)) {
#if defined(HTS_V5A_DISABLE_PTE) || defined(HTS_BYPASS_PTE)
            coarse_offset_q15 = 0;
#else
            coarse_offset_q15 =
                parabolic_offset_q15_impl(coarse_energies[cb_best_idx - 1],
                                          coarse_energies[cb_best_idx],
                                          coarse_energies[cb_best_idx + 1]);
#endif
#if defined(HTS_V5A_STEP_A_DIAG)
            std::printf(
                "[V5A-A-PTE-C] pass=%d cb_idx=%d em1=%lld e0=%lld ep1=%lld "
                "off_q15=%d\n",
                pass, cb_best_idx,
                static_cast<long long>(coarse_energies[cb_best_idx - 1]),
                static_cast<long long>(coarse_energies[cb_best_idx]),
                static_cast<long long>(coarse_energies[cb_best_idx + 1]),
                static_cast<int>(coarse_offset_q15));
#endif
#if defined(HTS_CFO_V5A_PTE_DIAG)
            diag->coarse_enter++;
#endif
        }
#if defined(HTS_V5A_STEP_A_DIAG)
        if (cb_best_idx <= 0 || cb_best_idx >= (kCfoCoarseBanks - 1)) {
            std::printf(
                "[V5A-A-PTE-C] pass=%d cb_idx=%d em1=na e0=na ep1=na "
                "off_q15=%d (edge/skip)\n",
                pass, cb_best_idx, static_cast<int>(coarse_offset_q15));
        }
#endif
#endif
#if defined(HTS_CFO_V5A_PTE_DIAG)
        {
            const int32_t abs_coarse =
                (coarse_offset_q15 < 0) ? -coarse_offset_q15 : coarse_offset_q15;
            if (abs_coarse != 0) {
                diag->coarse_nonzero++;
            }
            diag->coarse_abs_sum_q15 += static_cast<uint64_t>(abs_coarse);
            diag->coarse_hist[pte_hist_bin_from_abs_q15(abs_coarse)]++;
        }
#endif
        const int32_t cb_refined =
            cb_cfo + q15_mul_hz_round(coarse_offset_q15, kCfoCoarseStep);

        int32_t best_cfo = cb_refined;
        int64_t best_e = -1;
        int fine_best_idx = 0;

        const int32_t fine_start =
            cb_refined - (kCfoFineBanks / 2) * kCfoFineStep;

#if defined(HTS_BYPASS_FINE_SEARCH)
        (void)fine_start;
        fine_best_idx = kCfoFineBanks / 2;
        best_cfo = cb_refined;
        best_e = 0;
        for (int f = 0; f < 32 && f < kCfoFineBanks; ++f) {
            fine_energies_[f] = 0;
        }
#else
        for (int f = 0; f < kCfoFineBanks; ++f) {
            const int32_t cfo = fine_start + f * kCfoFineStep;
            Derotate_impl(rx_I, rx_Q, work_I_, work_Q_, kPreambleChips, cfo);
            const int64_t e = V5a_energy_mf(work_I_, work_Q_);
            if (f < 32) {
                fine_energies_[f] = e;
            }
            // strict e > best_e (tie → 유지), BPTE
            const int64_t diff_fe = e - best_e;
            const int64_t update_fe = ~((diff_fe - 1) >> 63);
            best_e = (~update_fe & best_e) | (update_fe & e);
            best_cfo = static_cast<int32_t>(
                (~update_fe & static_cast<int64_t>(best_cfo)) |
                (update_fe & static_cast<int64_t>(cfo)));
            fine_best_idx = static_cast<int>(
                (~update_fe & static_cast<int64_t>(fine_best_idx)) |
                (update_fe & static_cast<int64_t>(f)));
        }
#endif

#if defined(HTS_V5A_STEP_A_DIAG)
        {
            static int s_fine_dump_n = 0;
            constexpr int kFineDumpLimit = 6;
            if (s_fine_dump_n < kFineDumpLimit) {
                std::printf(
                    "[V5A-A-FINE] dump=%d pass=%d fstart=%d fb_idx=%d "
                    "best_cfo=%d best_e=%lld\n",
                    s_fine_dump_n, pass, static_cast<int>(fine_start),
                    fine_best_idx, static_cast<int>(best_cfo),
                    static_cast<long long>(best_e));
                constexpr int kF = (kCfoFineBanks < 32) ? kCfoFineBanks : 32;
                for (int f = 0; f < kF; ++f) {
                    const int32_t cfo_f = fine_start + f * kCfoFineStep;
                    std::printf("[V5A-A-FB] dump=%d f=%d cfo=%d e=%lld\n",
                                s_fine_dump_n, f, static_cast<int>(cfo_f),
                                static_cast<long long>(fine_energies_[f]));
                }
                ++s_fine_dump_n;
            }
        }
#endif

#if defined(HTS_V5A_DIAG_PER_SCENARIO)
        if (per_scn_active) {
            const unsigned pui = static_cast<unsigned>(per_sid);
            const int seq = g_v5a_per_scn_visit[pui];
            std::printf(
                "[V5A-PER-SCN-FINE] tag=%s seq=%d pass=%d fstart=%d fb_idx=%d "
                "best_cfo=%d best_e=%lld\n",
                per_tag, seq, pass, static_cast<int>(fine_start), fine_best_idx,
                static_cast<int>(best_cfo), static_cast<long long>(best_e));
            constexpr int kF = (kCfoFineBanks < 32) ? kCfoFineBanks : 32;
            for (int f = 0; f < kF; ++f) {
                const int32_t cfo_f = fine_start + f * kCfoFineStep;
                std::printf(
                    "[V5A-PER-SCN-FB] tag=%s seq=%d pass=%d f=%d cfo=%d e=%lld\n",
                    per_tag, seq, pass, f, static_cast<int>(cfo_f),
                    static_cast<long long>(fine_energies_[f]));
            }
        }
#endif

#if defined(HTS_V5A_STEP_B1_DIAG) && defined(HTS_V5A_DIAG)
        {
            static int s_b1_fine_dump_n = 0;
            constexpr int kB1FineDumpLimit = 10;
            const bool is_target =
                (g_v5a_lab_scenario != nullptr) &&
                (std::strcmp(g_v5a_lab_scenario, "S5") == 0) &&
                (g_v5a_lab_param_hz == 500);
            if (is_target && s_b1_fine_dump_n < kB1FineDumpLimit) {
                std::printf(
                    "[V5A-B1-FINE] dump=%d pass=%d fstart=%d fb_idx=%d best_cfo=%d "
                    "best_e=%lld\n",
                    s_b1_fine_dump_n, pass, static_cast<int>(fine_start),
                    fine_best_idx, static_cast<int>(best_cfo),
                    static_cast<long long>(best_e));
                constexpr int kF = (kCfoFineBanks < 32) ? kCfoFineBanks : 32;
                for (int f = 0; f < kF; ++f) {
                    const int32_t cfo_f = fine_start + f * kCfoFineStep;
                    std::printf("[V5A-B1-FB] dump=%d f=%d cfo=%d e=%lld\n",
                                s_b1_fine_dump_n, f, static_cast<int>(cfo_f),
                                static_cast<long long>(fine_energies_[f]));
                }
                ++s_b1_fine_dump_n;
            }
        }
#endif

        int32_t fine_offset_q15 = 0;
#if !defined(HTS_BYPASS_FINE_SEARCH)
        if (fine_best_idx > 0 && fine_best_idx < (kCfoFineBanks - 1)) {
#if defined(HTS_V5A_DISABLE_PTE) || defined(HTS_BYPASS_PTE)
            fine_offset_q15 = 0;
#else
            fine_offset_q15 = parabolic_offset_q15_impl(
                fine_energies_[fine_best_idx - 1], fine_energies_[fine_best_idx],
                fine_energies_[fine_best_idx + 1]);
#endif
#if defined(HTS_V5A_STEP_A_DIAG)
            std::printf(
                "[V5A-A-PTE-F] pass=%d fb_idx=%d em1=%lld e0=%lld ep1=%lld "
                "off_q15=%d\n",
                pass, fine_best_idx,
                static_cast<long long>(fine_energies_[fine_best_idx - 1]),
                static_cast<long long>(fine_energies_[fine_best_idx]),
                static_cast<long long>(fine_energies_[fine_best_idx + 1]),
                static_cast<int>(fine_offset_q15));
#endif
#if defined(HTS_CFO_V5A_PTE_DIAG)
            diag->fine_enter++;
#endif
        }
#if defined(HTS_V5A_STEP_A_DIAG)
        if (fine_best_idx <= 0 || fine_best_idx >= (kCfoFineBanks - 1)) {
            std::printf(
                "[V5A-A-PTE-F] pass=%d fb_idx=%d em1=na e0=na ep1=na "
                "off_q15=%d (edge/skip)\n",
                pass, fine_best_idx, static_cast<int>(fine_offset_q15));
        }
#endif
#endif
#if defined(HTS_CFO_V5A_PTE_DIAG)
        {
            const int32_t abs_fine =
                (fine_offset_q15 < 0) ? -fine_offset_q15 : fine_offset_q15;
            if (abs_fine != 0) {
                diag->fine_nonzero++;
            }
            diag->fine_abs_sum_q15 += static_cast<uint64_t>(abs_fine);
            diag->fine_hist[pte_hist_bin_from_abs_q15(abs_fine)]++;
        }
#endif
        const int32_t fine_refined =
            best_cfo + q15_mul_hz_round(fine_offset_q15, kCfoFineStep);

        Derotate_impl(rx_I, rx_Q, work_I_, work_Q_, kPreambleChips,
                      fine_refined);
#if defined(HTS_V5A_DISABLE_LR) || defined(HTS_BYPASS_LR)
        const int32_t lr_cfo = 0;
#else
#if defined(HTS_USE_MNM_WALSH)
        const int32_t lr_cfo = MnM_Walsh_Estimate_dpte_impl(work_I_, work_Q_);
#elif defined(HTS_USE_CLASSIC_LR)
        const int32_t lr_cfo = LR_Classic_Estimate_impl(work_I_, work_Q_);
#else
        const int32_t lr_cfo = LR_Estimate_impl(work_I_, work_Q_);
#endif
#endif

#if defined(HTS_BYPASS_FINEREF)
        cfo_estimate = lr_cfo;
#else
        cfo_estimate = fine_refined + lr_cfo;
#endif
#if defined(HTS_V5A_DIAG)
        trace_cb = cb_best_idx;
        trace_fb = fine_best_idx;
        trace_fine_ref = fine_refined;
        trace_lr = lr_cfo;
        trace_pass = pass;
        trace_coarse_start = coarse_start;
        trace_cb_cfo = cb_cfo;
        trace_cb_ref = cb_refined;
        trace_fine_start = fine_start;
        trace_best_cfo = best_cfo;
#endif
#if defined(HTS_LR_DIAG)
        g_lr_diag.cb_cfo = cb_cfo;
        g_lr_diag.fine_refined = fine_refined;
        g_lr_diag.cfo_estimate_hz = cfo_estimate;
#endif
#if defined(HTS_CFO_V5A_PTE_DIAG)
        {
            const int32_t lr_abs = (lr_cfo < 0) ? -lr_cfo : lr_cfo;
            diag->lr_abs_sum_hz += static_cast<uint64_t>(lr_abs);
            if (fine_refined < diag->pte_cfo_min_hz) diag->pte_cfo_min_hz = fine_refined;
            if (fine_refined > diag->pte_cfo_max_hz) diag->pte_cfo_max_hz = fine_refined;
            diag->pte_cfo_sum_hz += static_cast<int64_t>(fine_refined);
            diag->pte_samples++;
        }
#endif
        final_peak_e = best_e;
#if defined(HTS_V5A_TESTONLY)
        last_cb_bin_ = cb_best_idx;
        last_fb_bin_ = fine_best_idx;
#endif
#if defined(HTS_V5A_STEP_A_DIAG)
        std::printf(
            "[V5A-A-PASS] pass=%d cb_idx=%d cb_cfo=%d cb_ref=%d fb_idx=%d "
            "best_cfo=%d fine_ref=%d lr=%d cfo_est=%d\n",
            pass, cb_best_idx, static_cast<int>(cb_cfo),
            static_cast<int>(cb_refined), fine_best_idx,
            static_cast<int>(best_cfo), static_cast<int>(fine_refined),
            static_cast<int>(lr_cfo), static_cast<int>(cfo_estimate));
#endif
#if defined(HTS_V5A_DIAG_PER_SCENARIO)
        if (per_scn_active) {
            const unsigned pui = static_cast<unsigned>(per_sid);
            const int seq = g_v5a_per_scn_visit[pui];
            std::printf(
                "[V5A-PER-SCN-PASS] tag=%s seq=%d pass=%d cb_idx=%d cb_cfo=%d "
                "cb_ref=%d fb_idx=%d best_cfo=%d fine_ref=%d lr=%d cfo_est=%d\n",
                per_tag, seq, pass, cb_best_idx, static_cast<int>(cb_cfo),
                static_cast<int>(cb_refined), fine_best_idx,
                static_cast<int>(best_cfo), static_cast<int>(fine_refined),
                static_cast<int>(lr_cfo), static_cast<int>(cfo_estimate));
        }
#endif
    }

#if defined(HTS_V5A_DIAG_PER_SCENARIO)
    if (per_scn_active) {
        const unsigned pui = static_cast<unsigned>(per_sid);
        std::printf(
            "[V5A-PER-SCN-EST] tag=%s seq=%d cfo_hz=%d peak_e=%lld\n", per_tag,
            g_v5a_per_scn_visit[pui], static_cast<int>(cfo_estimate),
            static_cast<long long>(final_peak_e));
        v5a_per_scn_end_visit(per_sid);
    }
#endif

    last_cfo_hz_ = cfo_estimate;
#if defined(HTS_V5A_DIAG)
    v5a_diag_emit_est(trace_pass, trace_cb, trace_fb, trace_coarse_start,
                      trace_cb_cfo, trace_cb_ref, trace_fine_start, trace_best_cfo,
                      trace_fine_ref, trace_lr, cfo_estimate);
#endif
    res.cfo_hz = cfo_estimate;
    res.peak_energy = final_peak_e;
    res.valid = true;
    return res;
}

void CFO_V5a::Estimate_From_Autocorr(int32_t ac_I, int32_t ac_Q,
                                     int32_t lag_chips) noexcept {
#if defined(HTS_V5A_DIAG)
    ++g_v5a_ac_calls;
#endif
    if (lag_chips <= 0) {
        last_apply_gate_mag_ = 0;
        last_apply_gate_autocorr_ = true;
        last_cfo_hz_ = 0;
#if defined(HTS_V5A_DIAG)
        ++g_v5a_ac_skip_lag;
#endif
        Set_Apply_Cfo(0);
        return;
    }
    const int64_t mag2 = static_cast<int64_t>(ac_I) * ac_I +
                         static_cast<int64_t>(ac_Q) * ac_Q;
    last_apply_gate_mag_ = mag2;
    last_apply_gate_autocorr_ = true;
    if (mag2 < kAutocorrMag2Threshold) {
        last_cfo_hz_ = 0;
#if defined(HTS_V5A_DIAG)
        ++g_v5a_ac_skip_mag2;
#endif
        Set_Apply_Cfo(0);
        return;
    }
    const int32_t phase_q12_block = Autocorr_Block_Atan2_Q12(ac_I, ac_Q);
    const int32_t phase_q12_chip = phase_q12_block / lag_chips;
    const int64_t num =
        static_cast<int64_t>(phase_q12_chip) *
        static_cast<int64_t>(kChipRateHz);
    constexpr int64_t kDen = 2LL * 12868LL;  // 2·(π rad in Q12 units)
    const int64_t q =
        (num >= 0) ? (num + kDen / 2) : (num - kDen / 2);
    const int32_t cfo_hz = static_cast<int32_t>(q / kDen);
    last_cfo_hz_ = cfo_hz;
#if defined(HTS_V5A_DIAG)
    ++g_v5a_ac_ok;
    v5a_diag_emit_ac(ac_I, ac_Q, lag_chips, cfo_hz);
#endif
    Set_Apply_Cfo(cfo_hz);
}

void CFO_V5a::ApplyDerotate(const int16_t* in_I, const int16_t* in_Q,
                            int16_t* out_I, int16_t* out_Q, int chips,
                            int32_t cfo_hz) noexcept {
    Derotate_impl(in_I, in_Q, out_I, out_Q, chips, cfo_hz);
}

void CFO_V5a::Set_Apply_Cfo(int32_t cfo_hz) noexcept {
    {
        // [TASK-002 DIAG] Set_Apply_Cfo 입력값 분포 통계
        static uint32_t s_set_call_count = 0u;
        static uint32_t s_set_zero_count = 0u;
        static int32_t s_set_min_hz = 2147483647;
        static int32_t s_set_max_hz = -2147483647 - 1;
        static int64_t s_set_abs_sum = 0;
        ++s_set_call_count;
        if (cfo_hz == 0) {
            ++s_set_zero_count;
        }
        if (cfo_hz < s_set_min_hz) {
            s_set_min_hz = cfo_hz;
        }
        if (cfo_hz > s_set_max_hz) {
            s_set_max_hz = cfo_hz;
        }
        const int32_t abs_hz = (cfo_hz < 0) ? -cfo_hz : cfo_hz;
        s_set_abs_sum += static_cast<int64_t>(abs_hz);
        if (s_set_call_count <= 30u) {
            std::printf("[V5A-SET-INPUT] call#%u cfo_hz=%d\n",
                        static_cast<unsigned>(s_set_call_count),
                        static_cast<int>(cfo_hz));
            std::fflush(stdout);
        }
        if (s_set_call_count == 1u || s_set_call_count == 100u ||
            s_set_call_count == 1000u || s_set_call_count == 10000u ||
            (s_set_call_count > 0u && (s_set_call_count % 50000u) == 0u)) {
            const uint32_t den = (s_set_call_count > 0u) ? s_set_call_count : 1u;
            const int64_t avg_abs = s_set_abs_sum / static_cast<int64_t>(den);
            std::printf("[V5A-SET-STAT] calls=%u zeros=%u nonzero=%u min=%d max=%d avg_abs=%lld\n",
                        static_cast<unsigned>(s_set_call_count),
                        static_cast<unsigned>(s_set_zero_count),
                        static_cast<unsigned>(s_set_call_count - s_set_zero_count),
                        static_cast<int>(s_set_min_hz),
                        static_cast<int>(s_set_max_hz),
                        static_cast<long long>(avg_abs));
            std::fflush(stdout);
        }
    }
    apply_cfo_hz_ = cfo_hz;
    if (cfo_hz == 0) {
        apply_sin_per_q14_ = 0;
        apply_cos_per_q14_ = 16384;
    } else {
        const uint32_t inc_q32 = apply_phase_inc_q32_from_hz(cfo_hz);
        apply_sin_per_q14_ = static_cast<int32_t>(Lookup_Sin(inc_q32));
        apply_cos_per_q14_ = static_cast<int32_t>(Lookup_Cos(inc_q32));
        const int64_t sin_sq =
            static_cast<int64_t>(apply_sin_per_q14_) * apply_sin_per_q14_;
        constexpr int64_t kOne = 16384;
        const int64_t q14_sq = kOne * kOne;
        const int64_t c2 = q14_sq - sin_sq;
        if (c2 <= 0) {
            apply_cos_per_q14_ = 0;
        } else {
            apply_cos_per_q14_ = apply_int_root_q14(c2);
        }
    }
#if defined(HTS_V5A_DIAG)
    v5a_diag_emit_set(cfo_hz, apply_sin_per_q14_, apply_cos_per_q14_);
#endif
    Reset_Apply_Phase();
}

void CFO_V5a::Set_Apply_SinCosPerChip_Q14(int32_t sin_per_chip_q14,
                                         int32_t cos_per_chip_q14) noexcept {
    apply_cfo_hz_ = 0;
    apply_sin_per_q14_ = sin_per_chip_q14;
    apply_cos_per_q14_ = cos_per_chip_q14;
    Reset_Apply_Phase();
}

void CFO_V5a::Reset_Apply_Phase() noexcept {
    apply_cos_acc_q14_ = 16384;
    apply_sin_acc_q14_ = 0;
    apply_chip_counter_ = 0;
}

void CFO_V5a::Advance_Phase_Only(int chips) noexcept {
    if (chips <= 0) {
        return;
    }
    for (int k = 0; k < chips; ++k) {
        const int32_t next_cos = static_cast<int32_t>(
            (static_cast<int64_t>(apply_cos_acc_q14_) *
                 static_cast<int64_t>(apply_cos_per_q14_) -
             static_cast<int64_t>(apply_sin_acc_q14_) *
                 static_cast<int64_t>(apply_sin_per_q14_)) >>
            14);
        const int32_t next_sin = static_cast<int32_t>(
            (static_cast<int64_t>(apply_cos_acc_q14_) *
                 static_cast<int64_t>(apply_sin_per_q14_) +
             static_cast<int64_t>(apply_sin_acc_q14_) *
                 static_cast<int64_t>(apply_cos_per_q14_)) >>
            14);
        apply_cos_acc_q14_ = next_cos;
        apply_sin_acc_q14_ = next_sin;
        ++apply_chip_counter_;
        if ((apply_chip_counter_ & 0x3F) == 0) {
            apply_renorm_q14_accum(apply_cos_acc_q14_, apply_sin_acc_q14_);
        }
    }
}

void CFO_V5a::Apply_Per_Chip(int16_t& chip_I, int16_t& chip_Q) noexcept {
    {
        // [TASK-002 DIAG] Apply_Per_Chip 호출 통계
        static uint64_t s_apply_call_count = 0u;
        ++s_apply_call_count;
        if (s_apply_call_count == 1u || s_apply_call_count == 100u ||
            s_apply_call_count == 1000u || s_apply_call_count == 10000u ||
            s_apply_call_count == 100000u || s_apply_call_count == 1000000u) {
            std::printf("[V5A-APPLY-COUNT] call#%llu cfo_hz=%d sin_per_q14=%d cos_per_q14=%d\n",
                        static_cast<unsigned long long>(s_apply_call_count),
                        static_cast<int>(apply_cfo_hz_),
                        static_cast<int>(apply_sin_per_q14_),
                        static_cast<int>(apply_cos_per_q14_));
            std::fflush(stdout);
        }
    }
#if defined(HTS_BYPASS_APPLY)
    (void)chip_I;
    (void)chip_Q;
#else
    const int32_t ci = static_cast<int32_t>(chip_I);
    const int32_t cq = static_cast<int32_t>(chip_Q);
#ifdef HTS_V5A_APPLY_SIGN_FIX
    // [BUG FIX] Align with Derotate (e^{-j theta}, CFO removal).
    const int32_t ri = (ci * apply_cos_acc_q14_ - cq * apply_sin_acc_q14_) >> 14;
    const int32_t rq = (cq * apply_cos_acc_q14_ + ci * apply_sin_acc_q14_) >> 14;
    {
        // [TASK-001 DIAG] 빌드 매크로 활성 1회 출력 (측정 후 원복 예정)
        static int s_apply_branch_print = 0;
        if (s_apply_branch_print == 0) {
            std::printf("[V5A-APPLY-BRANCH] FIX (e^{-jtheta}) active\n");
            std::fflush(stdout);
            s_apply_branch_print = 1;
        }
    }
    {
        // [TASK-004 DIAG] Apply 출력 ri/rq — nonzero sin_per 시점만 첫 N회 출력
        static uint32_t s_apply_out_count = 0u;
        if (apply_sin_per_q14_ != 0 && s_apply_out_count < 20u) {
            ++s_apply_out_count;
            std::printf("[V5A-APPLY-OUT] hit#%u ci=%d cq=%d ri=%d rq=%d "
                        "cos_acc=%d sin_acc=%d cos_per=%d sin_per=%d cfo_hz=%d\n",
                        static_cast<unsigned>(s_apply_out_count),
                        static_cast<int>(ci), static_cast<int>(cq),
                        static_cast<int>(ri), static_cast<int>(rq),
                        static_cast<int>(apply_cos_acc_q14_),
                        static_cast<int>(apply_sin_acc_q14_),
                        static_cast<int>(apply_cos_per_q14_),
                        static_cast<int>(apply_sin_per_q14_),
                        static_cast<int>(apply_cfo_hz_));
            std::fflush(stdout);
        }
    }
#else
    // [LEGACY] e^{+j theta} (doubles CFO effect vs correct derotation).
    const int32_t ri = (ci * apply_cos_acc_q14_ + cq * apply_sin_acc_q14_) >> 14;
    const int32_t rq = (cq * apply_cos_acc_q14_ - ci * apply_sin_acc_q14_) >> 14;
    {
        // [TASK-001 DIAG] 빌드 매크로 활성 1회 출력 (측정 후 원복 예정)
        static int s_apply_branch_print = 0;
        if (s_apply_branch_print == 0) {
            std::printf("[V5A-APPLY-BRANCH] LEGACY (e^{+jtheta}) active\n");
            std::fflush(stdout);
            s_apply_branch_print = 1;
        }
    }
    {
        // [TASK-004 DIAG] Apply 출력 ri/rq — nonzero sin_per 시점만 첫 N회 출력
        static uint32_t s_apply_out_count = 0u;
        if (apply_sin_per_q14_ != 0 && s_apply_out_count < 20u) {
            ++s_apply_out_count;
            std::printf("[V5A-APPLY-OUT] hit#%u ci=%d cq=%d ri=%d rq=%d "
                        "cos_acc=%d sin_acc=%d cos_per=%d sin_per=%d cfo_hz=%d\n",
                        static_cast<unsigned>(s_apply_out_count),
                        static_cast<int>(ci), static_cast<int>(cq),
                        static_cast<int>(ri), static_cast<int>(rq),
                        static_cast<int>(apply_cos_acc_q14_),
                        static_cast<int>(apply_sin_acc_q14_),
                        static_cast<int>(apply_cos_per_q14_),
                        static_cast<int>(apply_sin_per_q14_),
                        static_cast<int>(apply_cfo_hz_));
            std::fflush(stdout);
        }
    }
#endif
#if defined(HTS_V5A_DIAG)
    v5a_diag_emit_apply(apply_chip_counter_, apply_cos_acc_q14_,
                        apply_sin_acc_q14_, apply_cos_per_q14_, apply_sin_per_q14_,
                        ci, cq, ri, rq);
#endif
    chip_I = saturate_i32_to_i16_bpte(ri);
    chip_Q = saturate_i32_to_i16_bpte(rq);
    const int32_t next_cos = static_cast<int32_t>(
        (static_cast<int64_t>(apply_cos_acc_q14_) *
             static_cast<int64_t>(apply_cos_per_q14_) -
         static_cast<int64_t>(apply_sin_acc_q14_) *
             static_cast<int64_t>(apply_sin_per_q14_)) >>
        14);
    const int32_t next_sin = static_cast<int32_t>(
        (static_cast<int64_t>(apply_cos_acc_q14_) *
             static_cast<int64_t>(apply_sin_per_q14_) +
         static_cast<int64_t>(apply_sin_acc_q14_) *
             static_cast<int64_t>(apply_cos_per_q14_)) >>
        14);
    apply_cos_acc_q14_ = next_cos;
    apply_sin_acc_q14_ = next_sin;
    ++apply_chip_counter_;
    if ((apply_chip_counter_ & 0x3F) == 0) {
        apply_renorm_q14_accum(apply_cos_acc_q14_, apply_sin_acc_q14_);
    }
#endif
}

#if defined(HTS_ALLOW_HOST_BUILD)
namespace test_export {

void Derotate_Table(const int16_t* rI, const int16_t* rQ, int16_t* oI,
                    int16_t* oQ, int chips, int32_t cfo_hz) noexcept {
    Derotate_impl(rI, rQ, oI, oQ, chips, cfo_hz);
}

void Walsh63_Dot_Table(const int16_t* rI, const int16_t* rQ, int32_t& dI,
                       int32_t& dQ) noexcept {
    Walsh63_Dot_impl(rI, rQ, dI, dQ);
}

int64_t Energy_Multiframe_Table(const int16_t* rI,
                                const int16_t* rQ) noexcept {
    return Energy_Multiframe_impl(rI, rQ);
}

int32_t LR_Estimate(const int16_t* rI, const int16_t* rQ) noexcept {
    return LR_Estimate_impl(rI, rQ);
}

#if defined(HTS_USE_MNM_WALSH)
int32_t MnM_Walsh_Estimate_Dpte_Table(const int16_t* rI,
                                    const int16_t* rQ) noexcept {
    return MnM_Walsh_Estimate_dpte_impl(rI, rQ);
}

int32_t Atan2_Dpte_Q15_Table(int64_t y, int64_t x) noexcept {
    return atan2_dpte_q15(y, x);
}
#endif

int32_t Parabolic_Offset_Q15(int64_t em1, int64_t e0, int64_t ep1) noexcept {
    return parabolic_offset_q15_impl(em1, e0, ep1);
}

}  // namespace test_export
#endif

#if defined(HTS_CFO_V5A_PTE_DIAG)
void V5a_Pte_Diag_Reset() noexcept {
    std::memset(g_v5a_pte_diag, 0, sizeof(g_v5a_pte_diag));
    g_v5a_pte_diag_count = 1;
    g_v5a_pte_diag_cur = 0;
    std::strncpy(g_v5a_pte_diag[0].tag, "GLOBAL",
                 sizeof(g_v5a_pte_diag[0].tag) - 1);
    g_v5a_pte_diag[0].pte_cfo_min_hz = 2147483647;
    g_v5a_pte_diag[0].pte_cfo_max_hz = -2147483647 - 1;
}

void V5a_Pte_Diag_Set_Context(const char* tag) noexcept {
    g_v5a_pte_diag_cur = diag_find_or_add_context_(tag);
}

void V5a_Pte_Diag_Print_Summary() noexcept {
    std::printf("[V5A-PTE-DIAG] === summary begin ===\n");
    for (int i = 0; i < g_v5a_pte_diag_count; ++i) {
        const V5aPteDiagBucket& d = g_v5a_pte_diag[i];
        if (d.estimate_calls == 0) {
            continue;
        }
        const uint32_t denom = (d.estimate_calls > 0u) ? d.estimate_calls : 1u;
        const uint32_t coarse_calls = (d.coarse_hist[0] + d.coarse_hist[1] +
                                       d.coarse_hist[2] + d.coarse_hist[3] +
                                       d.coarse_hist[4]);
        const uint32_t fine_calls = (d.fine_hist[0] + d.fine_hist[1] +
                                     d.fine_hist[2] + d.fine_hist[3] +
                                     d.fine_hist[4]);
        const uint64_t coarse_den = (coarse_calls > 0u) ? coarse_calls : 1u;
        const uint64_t fine_den = (fine_calls > 0u) ? fine_calls : 1u;
        const uint64_t avg_abs_coarse = d.coarse_abs_sum_q15 / coarse_den;
        const uint64_t avg_abs_fine = d.fine_abs_sum_q15 / fine_den;
        const uint64_t avg_lr = d.lr_abs_sum_hz / static_cast<uint64_t>(denom);
        const uint32_t pte_den = (d.pte_samples > 0u) ? d.pte_samples : 1u;
        const int64_t avg_pte = d.pte_cfo_sum_hz / static_cast<int64_t>(pte_den);
        std::printf(
            "[V5A-PTE-DIAG][%s] est=%u coarse_enter=%u fine_enter=%u coarse_nz=%u fine_nz=%u avg|coarse_q15|=%llu avg|fine_q15|=%llu avg|lr_hz|=%llu pte_cfo[min,max,avg]=[%d,%d,%lld]\n",
            d.tag, static_cast<unsigned>(d.estimate_calls),
            static_cast<unsigned>(d.coarse_enter), static_cast<unsigned>(d.fine_enter),
            static_cast<unsigned>(d.coarse_nonzero), static_cast<unsigned>(d.fine_nonzero),
            static_cast<unsigned long long>(avg_abs_coarse),
            static_cast<unsigned long long>(avg_abs_fine),
            static_cast<unsigned long long>(avg_lr), static_cast<int>(d.pte_cfo_min_hz),
            static_cast<int>(d.pte_cfo_max_hz), static_cast<long long>(avg_pte));
        std::printf(
            "[V5A-PTE-DIAG][%s] coarse_hist=[%u,%u,%u,%u,%u] fine_hist=[%u,%u,%u,%u,%u]\n",
            d.tag, static_cast<unsigned>(d.coarse_hist[0]),
            static_cast<unsigned>(d.coarse_hist[1]),
            static_cast<unsigned>(d.coarse_hist[2]),
            static_cast<unsigned>(d.coarse_hist[3]),
            static_cast<unsigned>(d.coarse_hist[4]),
            static_cast<unsigned>(d.fine_hist[0]),
            static_cast<unsigned>(d.fine_hist[1]),
            static_cast<unsigned>(d.fine_hist[2]),
            static_cast<unsigned>(d.fine_hist[3]),
            static_cast<unsigned>(d.fine_hist[4]));
    }
    std::printf("[V5A-PTE-DIAG] === summary end ===\n");
}
#endif

}  // namespace rx_cfo
}  // namespace hts
