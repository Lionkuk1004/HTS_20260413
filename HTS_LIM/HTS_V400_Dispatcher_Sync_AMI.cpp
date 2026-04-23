// =============================================================================
// HTS_V400_Dispatcher_Sync_AMI.cpp — AMI 타깃 전용 Sync TU (분리 초기)
// =============================================================================
// Stage A-0: PS-LTE Sync.cpp 복사본으로 시작. 본 TU에서만 HTS_TARGET_AMI 를
// 끄면 기존 #if/#else 분기가 PS-LTE 경로(#else)로 통일된다.
// 향후 AMI 64-chip 사양 반영 시 이 파일만 수정 (PS-LTE Sync.cpp 비변경).
#ifdef HTS_TARGET_AMI
#  undef HTS_TARGET_AMI
#  define HTS_AMI_SYNC_FILE 1
#endif
// =============================================================================
// HTS_V400_Dispatcher_Sync.cpp — Feed_Chip / phase0 / MF (원본과 동일 본문)
// =============================================================================
#include "HTS_V400_Dispatcher.hpp"
#include "HTS_V400_Dispatcher_Internal.hpp"
#ifdef HTS_USE_HOLOGRAPHIC_SYNC
#include "HTS_Preamble_Holographic.h"
#endif
#include "HTS_Secure_Memory.h"
#if defined(HTS_SYNC_USE_MATCHED_FILTER)
#include "HTS_Dynamic_Config.h"
#if defined(_MSC_VER)
#include <intrin.h>
#endif
#endif
#if defined(HTS_SYNC_DIAG)
#include "HTS_Sync_Diag.hpp"
#endif
#if defined(HTS_WALSH_ROW_DIAG)
#include "HTS_Walsh_Row_Diag.hpp"
#endif
#if defined(HTS_AMP_DIAG)
#include "HTS_Amp_Diag.hpp"
#endif
#include <climits>
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <cstddef>
namespace ProtectedEngine {
using namespace detail;
void HTS_V400_Dispatcher::psal_commit_align_() noexcept {
    psal_pending_ = false;
    const int commit_off = psal_off_;
    const int carry = 64 - commit_off;
    p0_carry_count_ = carry;
    if (carry > 0) {
        const int src = commit_off + 128;
        for (int j = 0; j < carry; ++j) {
            p0_carry_I_[j] = p0_buf128_I_[src + j];
            p0_carry_Q_[j] = p0_buf128_Q_[src + j];
        }
    }
    pre_phase_ = 1;
    p1_carry_pending_ = (carry > 0) ? 1 : 0;
    p1_tail_collect_rem_ = 0;
    p1_tail_idx_ = 0;
    p1_carry_prefix_ = 0;
    buf_idx_ = 0;
    wait_sync_head_ = 0;
    wait_sync_count_ = 0;
    p0_chip_count_ = 0;
}

#if defined(HTS_HOLO_PREAMBLE)
void HTS_V400_Dispatcher::phase0_scan_holo_preamble_rx_() noexcept {
#if HTS_HOLO_CMYK_MODE
    phase0_scan_cmyk_gravity_cube_ami_();
    return;
#else
    if (p0_chip_count_ < 192) {
        std::memcpy(p0_buf128_I_, p0_buf128_I_ + 128,
                    static_cast<size_t>(64) * sizeof(int16_t));
        std::memcpy(p0_buf128_Q_, p0_buf128_Q_ + 128,
                    static_cast<size_t>(64) * sizeof(int16_t));
        p0_chip_count_ = 64;
        return;
    }

    int32_t amp_est = 0;
    for (int i = 0; i < 192; ++i) {
        const int32_t ai = static_cast<int32_t>(p0_buf128_I_[i]);
        const int32_t sm = ai >> 31;
        amp_est += (ai ^ sm) - sm;
    }
    amp_est /= 192;
    if (amp_est < 1) {
        amp_est = 1;
    }
    const int32_t amp_thr =
        (tx_amp_ > 0) ? static_cast<int32_t>(tx_amp_) : amp_est;

    int best_off_ac = -1;
    int64_t best_mag2 = -1;

    for (int off = 0; off < 64; ++off) {
        int64_t acI = 0;
        int64_t acQ = 0;
        for (int n = 0; n < 64; ++n) {
            const int32_t r1I =
                static_cast<int32_t>(p0_buf128_I_[off + n]);
            const int32_t r1Q =
                static_cast<int32_t>(p0_buf128_Q_[off + n]);
            const int32_t r2I =
                static_cast<int32_t>(p0_buf128_I_[off + n + 64]);
            const int32_t r2Q =
                static_cast<int32_t>(p0_buf128_Q_[off + n + 64]);
            acI += static_cast<int64_t>(r1I) * r2I +
                   static_cast<int64_t>(r1Q) * r2Q;
            acQ += static_cast<int64_t>(r1I) * r2Q -
                   static_cast<int64_t>(r1Q) * r2I;
        }
        const int64_t mag2 = acI * acI + acQ * acQ;
        if (mag2 > best_mag2) {
            best_mag2 = mag2;
            best_off_ac = off;
        }
    }

#if defined(HTS_DIAG_PRINTF) && defined(HTS_DIAG_CFO_EST)
    std::printf(
        "[P0-TIMING-LAG64] best_off=%d mag2=%lld\n",
        best_off_ac, static_cast<long long>(best_mag2));
#endif

    const int64_t ac_thr =
        (static_cast<int64_t>(amp_thr) * amp_thr * 64LL * 64LL) / 500000LL;
    if (best_off_ac < 0 || best_mag2 < ac_thr) {
        std::memcpy(p0_buf128_I_, p0_buf128_I_ + 128,
                    static_cast<size_t>(64) * sizeof(int16_t));
        std::memcpy(p0_buf128_Q_, p0_buf128_Q_ + 128,
                    static_cast<size_t>(64) * sizeof(int16_t));
        p0_chip_count_ = 64;
        return;
    }

    int64_t cfo_acI = 0;
    int64_t cfo_acQ = 0;
    const int bo = best_off_ac;
    for (int n = 0; n < 48; ++n) {
        const int32_t r1I =
            static_cast<int32_t>(p0_buf128_I_[bo + n]);
        const int32_t r1Q =
            static_cast<int32_t>(p0_buf128_Q_[bo + n]);
        const int32_t r2I =
            static_cast<int32_t>(p0_buf128_I_[bo + n + 16]);
        const int32_t r2Q =
            static_cast<int32_t>(p0_buf128_Q_[bo + n + 16]);
        cfo_acI += static_cast<int64_t>(r1I) * r2I +
                   static_cast<int64_t>(r1Q) * r2Q;
        cfo_acQ += static_cast<int64_t>(r1I) * r2Q -
                   static_cast<int64_t>(r1Q) * r2I;
    }

    {
        int64_t est_acI = cfo_acI;
        int64_t est_acQ = cfo_acQ;
        if (cfo_acQ == 0LL && cfo_acI < 0LL) {
            est_acI = -cfo_acI;
        }

        int sh = 0;
        int64_t mx = (est_acI < 0) ? -est_acI : est_acI;
        const int64_t ax = (est_acQ < 0) ? -est_acQ : est_acQ;
        if (ax > mx) {
            mx = ax;
        }
        while (mx > 2000000000LL && sh < 28) {
            mx >>= 1;
            ++sh;
        }
        const int32_t d1I = static_cast<int32_t>(est_acI >> sh);
        const int32_t d1Q = static_cast<int32_t>(est_acQ >> sh);
#if defined(HTS_DIAG_PRINTF) && defined(HTS_DIAG_CFO_EST)
        std::printf(
            "[P0-CFO-LAG16] acI=%lld acQ=%lld d1I=%d d1Q=%d sh=%d off=%d\n",
            static_cast<long long>(cfo_acI),
            static_cast<long long>(cfo_acQ), static_cast<int>(d1I),
            static_cast<int>(d1Q), sh, bo);
#endif
        cfo_.Estimate_From_Autocorr(d1I, d1Q, 16);
#if defined(HTS_DIAG_PRINTF) && defined(HTS_DIAG_CFO_EST)
        std::printf(
            "[POST-EST-ATAN2] sin14=%d hz=%d\n",
            static_cast<int>(cfo_.Get_Sin_Per_Chip_Q14()),
            static_cast<int>(
                static_cast<double>(cfo_.Get_Sin_Per_Chip_Q14()) * 1e6 /
                (2.0 * 3.14159265358979 * 16384.0)));
#endif
        cfo_.Advance_Phase_Only(192);
    }

    int16_t local_A[64];
    generate_holo_preamble_(local_A, 1000, holo_lpi_seed_, rx_seq_);

    int8_t exp_sign[63];
    for (int k = 0; k < 63; ++k) {
        exp_sign[k] = static_cast<int8_t>(
            ((local_A[k] > 0) == (local_A[k + 1] > 0)) ? 1 : -1);
    }

    int64_t best_xc = 0;
    int chip_start = best_off_ac;
    int64_t second_xc = 0;
    int second_dd = -1;
#if defined(HTS_DIAG_PRINTF) && defined(HTS_DIAG_CFO_EST)
    int64_t xc_samples[129];
    int n_xc_samples = 0;
#endif
    for (int dd = best_off_ac - 64; dd <= best_off_ac + 64; ++dd) {
        if (dd < 0 || dd + 63 >= 192) {
            continue;
        }
        int64_t acc = 0;
        for (int k = 0; k < 63; ++k) {
            const int i1 = dd + k;
            const int i2 = dd + k + 1;
            const int64_t df =
                static_cast<int64_t>(p0_buf128_I_[i1]) * p0_buf128_I_[i2] +
                static_cast<int64_t>(p0_buf128_Q_[i1]) * p0_buf128_Q_[i2];
            if (exp_sign[k] > 0) {
                acc += df;
            } else {
                acc -= df;
            }
        }
        const int64_t xm = acc * acc;
#if defined(HTS_DIAG_PRINTF) && defined(HTS_DIAG_CFO_EST)
        if (n_xc_samples < 129) {
            xc_samples[n_xc_samples++] = xm;
        }
#endif
        if (xm > best_xc) {
            second_xc = best_xc;
            second_dd = chip_start;
            best_xc = xm;
            chip_start = dd;
        } else if (xm > second_xc) {
            second_xc = xm;
            second_dd = dd;
        }
    }

#if defined(HTS_DIAG_PRINTF) && defined(HTS_DIAG_CFO_EST)
    {
        int64_t xc_mean_i64 = 0;
        double peak_ratio = -1.0;
        if (second_xc > 0) {
            peak_ratio = static_cast<double>(best_xc) /
                         static_cast<double>(second_xc);
        }
        if (n_xc_samples > 0) {
            double s = 0.0;
            for (int k = 0; k < n_xc_samples; ++k) {
                s += static_cast<double>(xc_samples[k]);
            }
            xc_mean_i64 = static_cast<int64_t>(s / static_cast<double>(n_xc_samples));
        }
        std::printf(
            "[XC-DIST] best=%lld dd=%d 2nd=%lld dd2=%d peak_ratio=%.4f "
            "xc_mean=%lld n=%d ac_ref_off=%d\n",
            static_cast<long long>(best_xc), chip_start,
            static_cast<long long>(second_xc), second_dd, peak_ratio,
            static_cast<long long>(xc_mean_i64), n_xc_samples, best_off_ac);
    }
#endif

    const int64_t xc_thr =
        (static_cast<int64_t>(amp_thr) * amp_thr * 63LL * 9LL) / 100LL;
    if (best_xc < xc_thr) {
        std::memcpy(p0_buf128_I_, p0_buf128_I_ + 128,
                    static_cast<size_t>(64) * sizeof(int16_t));
        std::memcpy(p0_buf128_Q_, p0_buf128_Q_ + 128,
                    static_cast<size_t>(64) * sizeof(int16_t));
        p0_chip_count_ = 64;
        return;
    }

    if (chip_start < 0) {
        chip_start = 0;
    }
    if (chip_start > 63) {
        chip_start = 63;
    }

    dominant_row_ = 63u;

    int32_t seed_dot_I = 0;
    int32_t seed_dot_Q = 0;
    for (int blk = 0; blk < 2; ++blk) {
        int32_t di = 0;
        int32_t dq = 0;
        walsh63_dot_(&p0_buf128_I_[chip_start + (blk << 6)],
                     &p0_buf128_Q_[chip_start + (blk << 6)], di, dq);
        seed_dot_I += di;
        seed_dot_Q += dq;
    }
    est_I_ = seed_dot_I;
    est_Q_ = seed_dot_Q;
    est_count_ = 2;
    update_derot_shift_from_est_();

    {
        int32_t mag_sum = 0;
        for (int j = 0; j < 64; ++j) {
            const int32_t ai =
                static_cast<int32_t>(p0_buf128_I_[chip_start + j]);
            const int32_t aq =
                static_cast<int32_t>(p0_buf128_Q_[chip_start + j]);
            const int32_t si = ai >> 31;
            mag_sum += (ai ^ si) - si;
            const int32_t sq = aq >> 31;
            mag_sum += (aq ^ sq) - sq;
        }
        const int32_t peak_avg = mag_sum >> 6;
        pre_agc_.Set_From_Peak(peak_avg);
    }

    psal_off_ = chip_start;
    psal_e63_ = static_cast<int32_t>(best_mag2 >> 32);
    if (psal_e63_ <= 0) {
        psal_e63_ = static_cast<int32_t>(best_mag2 >> 20);
    }
    if (psal_e63_ <= 0) {
        psal_e63_ = 1;
    }
#if defined(HTS_DIAG_HOLO_CMYK) && !defined(HTS_PLATFORM_ARM)
    {
        static int s_diag_legacy_state_ami = 0;
        if (s_diag_legacy_state_ami < 40) {
            std::printf(
                "[CMYK-DIAG-LEGACY-STATE-AMI] n=%d "
                "psal_off=%d psal_e63=%d p0_chip_count=%d pre_phase=%d "
                "chip_start=%d off_ac=%d best_mag2=%lld best_xc=%lld "
                "dom=%u estI=%d estQ=%d est_n=%d derot=%d "
                "agc_sh=%d cfo_sin14=%d cfo_ap=%d\n",
                s_diag_legacy_state_ami, static_cast<int>(psal_off_),
                static_cast<int>(psal_e63_),
                static_cast<int>(p0_chip_count_),
                static_cast<int>(pre_phase_), chip_start, best_off_ac,
                static_cast<long long>(best_mag2),
                static_cast<long long>(best_xc),
                static_cast<unsigned>(dominant_row_),
                static_cast<int>(est_I_), static_cast<int>(est_Q_),
                est_count_, derot_shift_,
                static_cast<int>(pre_agc_.Get_Shift()),
                static_cast<int>(cfo_.Get_Sin_Per_Chip_Q14()),
                cfo_.Is_Apply_Active() ? 1 : 0);
            ++s_diag_legacy_state_ami;
            std::fflush(stdout);
        }
    }
#endif
    psal_commit_align_();

#if defined(HTS_DIAG_PRINTF) && defined(HTS_DIAG_CFO_EST)
    {
        static int s_holo_p0_ok_seq = 0;
        ++s_holo_p0_ok_seq;
#if defined(HTS_ALLOW_HOST_BUILD)
        std::printf(
            "[P0-HOLO-OFF] seq=%d off_ac=%d chip_start=%d ac_mag2=%lld "
            "cfo_hz=%.2f sin14=%d cfo_ap=%d\n",
            s_holo_p0_ok_seq, best_off_ac, chip_start,
            static_cast<long long>(best_mag2), cfo_.Get_Est_Hz(200000.0),
            static_cast<int>(cfo_.Get_Sin_Per_Chip_Q14()),
            cfo_.Is_Apply_Active() ? 1 : 0);
#else
        std::printf(
            "[P0-HOLO-OFF] seq=%d off_ac=%d chip_start=%d ac_mag2=%lld "
            "sin14=%d cfo_ap=%d\n",
            s_holo_p0_ok_seq, best_off_ac, chip_start,
            static_cast<long long>(best_mag2),
            static_cast<int>(cfo_.Get_Sin_Per_Chip_Q14()),
            cfo_.Is_Apply_Active() ? 1 : 0);
#endif
    }
#endif
#if defined(HTS_DIAG_PRINTF)
    std::printf(
        "[HOLO-PRE] PASS off_ac=%d chip_start=%d ac_mag2=%lld xc=%lld "
        "amp=%d\n",
        best_off_ac, chip_start, static_cast<long long>(best_mag2),
        static_cast<long long>(best_xc), static_cast<int>(amp_thr));
#endif
#endif
}

#if HTS_HOLO_CMYK_MODE
void HTS_V400_Dispatcher::phase0_scan_cmyk_gravity_cube_ami_() noexcept {
    using namespace detail;

    if (p0_chip_count_ < k_p0_holo_rx_collect_chips_) {
        std::memcpy(p0_buf128_I_, p0_buf128_I_ + 128,
                    static_cast<size_t>(64) * sizeof(int16_t));
        std::memcpy(p0_buf128_Q_, p0_buf128_Q_ + 128,
                    static_cast<size_t>(64) * sizeof(int16_t));
        p0_chip_count_ = 64;
        return;
    }

    int16_t tplA[64];
    int16_t tplB[64];
    int16_t tplC[64];
    int16_t tplD[64];
    gen_holo_sequence_cmyk(holo_lpi_seed_, rx_seq_, 0u, 1000, tplA);
    gen_holo_sequence_cmyk(holo_lpi_seed_, rx_seq_, 1u, 1000, tplB);
    gen_holo_sequence_cmyk(holo_lpi_seed_, rx_seq_, 2u, 1000, tplC);
    gen_holo_sequence_cmyk(holo_lpi_seed_, rx_seq_, 3u, 1000, tplD);

    const int32_t buf_chips = static_cast<int32_t>(p0_chip_count_);

    // Phase 3.B: INNOViD — Stage 1 Sign XC coarse (Template A full XC 대체).
    const uint64_t sA = tpl_to_sign64(tplA);
    const uint64_t sB = tpl_to_sign64(tplB);
    const uint64_t sC = tpl_to_sign64(tplC);
    const uint64_t sD = tpl_to_sign64(tplD);
    int64_t coarse_score = 0;
    const int32_t coarse_best_off = sign_scan_4tmpl(
        p0_buf128_I_, p0_buf128_Q_, 0, buf_chips, sA, sB, sC, sD, &coarse_score);

#if defined(HTS_DIAG_HOLO_CMYK) && !defined(HTS_PLATFORM_ARM)
    std::printf(
        "[CMYK-DIAG-SIGN-COARSE-AMI] off=%d score=%lld thr=%d\n",
        static_cast<int>(coarse_best_off),
        static_cast<long long>(coarse_score),
        static_cast<int>(SIGN_THRESHOLD_INIT));
    std::fflush(stdout);
    std::printf(
        "[CMYK-DIAG-AMI] stage=COARSE best_off=%d score=%lld\n",
        static_cast<int>(coarse_best_off),
        static_cast<long long>(coarse_score));
    std::fflush(stdout);
#endif

    if (coarse_best_off < 0 ||
        coarse_score < static_cast<int64_t>(SIGN_THRESHOLD_INIT)) {
        cmyk_last_pass_ = false;
#if defined(HTS_DIAG_HOLO_CMYK) && !defined(HTS_PLATFORM_ARM)
        if (coarse_best_off < 0) {
            std::printf(
                "[CMYK-DIAG-AMI] stage=COARSE_FAIL reason=no_valid_offset\n");
        } else {
            std::printf(
                "[CMYK-DIAG-AMI] stage=COARSE_FAIL reason=below_sign_thr "
                "score=%lld thr=%d\n",
                static_cast<long long>(coarse_score),
                static_cast<int>(SIGN_THRESHOLD_INIT));
        }
        std::fflush(stdout);
#endif
        std::memcpy(p0_buf128_I_, p0_buf128_I_ + 128,
                    static_cast<size_t>(64) * sizeof(int16_t));
        std::memcpy(p0_buf128_Q_, p0_buf128_Q_ + 128,
                    static_cast<size_t>(64) * sizeof(int16_t));
        p0_chip_count_ = 64;
        return;
    }

    int64_t cfo_acI = 0;
    int64_t cfo_acQ = 0;
    const int bo = coarse_best_off;
    for (int ci = 0; ci < 48; ++ci) {
        const int32_t r1I =
            static_cast<int32_t>(p0_buf128_I_[bo + ci]);
        const int32_t r1Q =
            static_cast<int32_t>(p0_buf128_Q_[bo + ci]);
        const int32_t r2I =
            static_cast<int32_t>(p0_buf128_I_[bo + ci + 16]);
        const int32_t r2Q =
            static_cast<int32_t>(p0_buf128_Q_[bo + ci + 16]);
        cfo_acI += static_cast<int64_t>(r1I) * r2I +
                   static_cast<int64_t>(r1Q) * r2Q;
        cfo_acQ += static_cast<int64_t>(r1I) * r2Q -
                   static_cast<int64_t>(r1Q) * r2I;
    }

    {
        int64_t est_acI = cfo_acI;
        int64_t est_acQ = cfo_acQ;
        if (cfo_acQ == 0LL && cfo_acI < 0LL) {
            est_acI = -cfo_acI;
        }

        int sh = 0;
        int64_t mx = (est_acI < 0) ? -est_acI : est_acI;
        const int64_t ax = (est_acQ < 0) ? -est_acQ : est_acQ;
        if (ax > mx) {
            mx = ax;
        }
        while (mx > 2000000000LL && sh < 28) {
            mx >>= 1;
            ++sh;
        }
        const int32_t d1I = static_cast<int32_t>(est_acI >> sh);
        const int32_t d1Q = static_cast<int32_t>(est_acQ >> sh);
        cfo_.Estimate_From_Autocorr(d1I, d1Q, 16);
        cfo_.Advance_Phase_Only(k_p0_holo_rx_collect_chips_);
    }

    const int64_t nf_total = gravity_estimate_noise_floor(
        p0_buf128_I_, p0_buf128_Q_, buf_chips, tplA, tplB, tplC, tplD);
    const int64_t nf_per_tmpl = nf_total >> 2;

    const int32_t max_fine_off = buf_chips - 256;
    if (max_fine_off < 0) {
        cmyk_last_pass_ = false;
#if defined(HTS_DIAG_HOLO_CMYK) && !defined(HTS_PLATFORM_ARM)
        std::printf(
            "[CMYK-DIAG-AMI] stage=FINE_FAIL reason=buf_lt_256 buf_chips=%d\n",
            static_cast<int>(buf_chips));
        std::fflush(stdout);
#endif
        std::memcpy(p0_buf128_I_, p0_buf128_I_ + 128,
                    static_cast<size_t>(64) * sizeof(int16_t));
        std::memcpy(p0_buf128_Q_, p0_buf128_Q_ + 128,
                    static_cast<size_t>(64) * sizeof(int16_t));
        p0_chip_count_ = 64;
        return;
    }

    const int32_t fine_lo =
        (coarse_best_off > 8) ? (coarse_best_off - 8) : 0;
    int32_t fine_hi = coarse_best_off + 8;
    if (fine_hi > max_fine_off) {
        fine_hi = max_fine_off;
    }
    if (fine_lo > fine_hi) {
        cmyk_last_pass_ = false;
#if defined(HTS_DIAG_HOLO_CMYK) && !defined(HTS_PLATFORM_ARM)
        std::printf(
            "[CMYK-DIAG-AMI] stage=FINE_FAIL reason=empty_window coarse_off=%d "
            "max_fine=%d\n",
            static_cast<int>(coarse_best_off),
            static_cast<int>(max_fine_off));
        std::fflush(stdout);
#endif
        std::memcpy(p0_buf128_I_, p0_buf128_I_ + 128,
                    static_cast<size_t>(64) * sizeof(int16_t));
        std::memcpy(p0_buf128_Q_, p0_buf128_Q_ + 128,
                    static_cast<size_t>(64) * sizeof(int16_t));
        p0_chip_count_ = 64;
        return;
    }

    int64_t best_total = -1;
    int best_off = -1;
    int64_t best_sub_score[4][4];
    int64_t best_sub_xI[4][4];
    int64_t best_sub_xQ[4][4];
    int64_t best_tmpl_xI[4];
    int64_t best_tmpl_xQ[4];
    int64_t best_tmpl_total[4];

    for (int32_t off = fine_lo; off <= fine_hi; ++off) {
        int64_t sub_score[4][4];
        int64_t sub_xI[4][4];
        int64_t sub_xQ[4][4];
        int64_t tmpl_xI[4];
        int64_t tmpl_xQ[4];
        int64_t tmpl_total[4];
        int64_t total = 0;

        gravity_xc_single_offset(
            p0_buf128_I_, p0_buf128_Q_, off, tplA, tplB, tplC, tplD,
            sub_score, sub_xI, sub_xQ, tmpl_xI, tmpl_xQ, tmpl_total,
            &total);

        if (total > best_total) {
            best_total = total;
            best_off = static_cast<int>(off);
            for (int t = 0; t < 4; ++t) {
                for (int s = 0; s < 4; ++s) {
                    best_sub_score[t][s] = sub_score[t][s];
                    best_sub_xI[t][s] = sub_xI[t][s];
                    best_sub_xQ[t][s] = sub_xQ[t][s];
                }
                best_tmpl_xI[t] = tmpl_xI[t];
                best_tmpl_xQ[t] = tmpl_xQ[t];
                best_tmpl_total[t] = tmpl_total[t];
            }
        }
    }

#if defined(HTS_DIAG_HOLO_CMYK) && !defined(HTS_PLATFORM_ARM)
    std::printf(
        "[CMYK-DIAG-AMI] stage=FINE best_off=%d total=%lld nf=%lld\n",
        best_off, static_cast<long long>(best_total),
        static_cast<long long>(nf_total));
    std::fflush(stdout);
#endif

    if (best_off < 0 || best_total < 0) {
        cmyk_last_pass_ = false;
#if defined(HTS_DIAG_HOLO_CMYK) && !defined(HTS_PLATFORM_ARM)
        std::printf("[CMYK-DIAG-AMI] stage=FINE_FAIL reason=no_xc_peak\n");
        std::fflush(stdout);
#endif
        std::memcpy(p0_buf128_I_, p0_buf128_I_ + 128,
                    static_cast<size_t>(64) * sizeof(int16_t));
        std::memcpy(p0_buf128_Q_, p0_buf128_Q_ + 128,
                    static_cast<size_t>(64) * sizeof(int16_t));
        p0_chip_count_ = 64;
        return;
    }

    GravityCube6 cube{};
    cube.best_off = static_cast<int32_t>(best_off);
    gravity_compute_faces(best_sub_score, best_sub_xI, best_sub_xQ,
                          best_tmpl_xI, best_tmpl_xQ, best_tmpl_total,
                          best_total, nf_total, nf_per_tmpl, &cube);
    for (int t = 0; t < 4; ++t) {
        cube.tmpl_xI[t] = best_tmpl_xI[t];
        cube.tmpl_xQ[t] = best_tmpl_xQ[t];
    }

    const bool pass = gravity_cube_pass(&cube);
    cmyk_last_cube_ = cube;
    cmyk_last_pass_ = pass;

#if defined(HTS_DIAG_HOLO_CMYK) && !defined(HTS_PLATFORM_ARM)
    std::printf(
        "[CMYK-DIAG-AMI] stage=CUBE pass=%d A=%d B=%d C=%d D=%d E=%d F=%d\n",
        pass ? 1 : 0, static_cast<int>(cube.face_A_q10),
        static_cast<int>(cube.face_B_q10), static_cast<int>(cube.face_C_q10),
        static_cast<int>(cube.face_D_q10), static_cast<int>(cube.face_E_q10),
        static_cast<int>(cube.face_F_q10));
    std::fflush(stdout);
#endif

    if (!pass) {
        std::memcpy(p0_buf128_I_, p0_buf128_I_ + 128,
                    static_cast<size_t>(64) * sizeof(int16_t));
        std::memcpy(p0_buf128_Q_, p0_buf128_Q_ + 128,
                    static_cast<size_t>(64) * sizeof(int16_t));
        p0_chip_count_ = 64;
        return;
    }

    int chip_start = best_off % 64;
    if (chip_start < 0) {
        chip_start += 64;
    }

    dominant_row_ = 63u;

    int32_t seed_dot_I = 0;
    int32_t seed_dot_Q = 0;
    for (int blk = 0; blk < 2; ++blk) {
        int32_t di = 0;
        int32_t dq = 0;
        walsh63_dot_(&p0_buf128_I_[chip_start + (blk << 6)],
                     &p0_buf128_Q_[chip_start + (blk << 6)], di, dq);
        seed_dot_I += di;
        seed_dot_Q += dq;
    }
    est_I_ = seed_dot_I;
    est_Q_ = seed_dot_Q;
    est_count_ = 2;
    update_derot_shift_from_est_();

    {
        int32_t mag_sum = 0;
        for (int j = 0; j < 64; ++j) {
            const int32_t ai =
                static_cast<int32_t>(p0_buf128_I_[chip_start + j]);
            const int32_t aq =
                static_cast<int32_t>(p0_buf128_Q_[chip_start + j]);
            const int32_t si = ai >> 31;
            mag_sum += (ai ^ si) - si;
            const int32_t sq = aq >> 31;
            mag_sum += (aq ^ sq) - sq;
        }
        const int32_t peak_avg = mag_sum >> 6;
        pre_agc_.Set_From_Peak(peak_avg);
    }

    // Phase 3.F: INNOViD — AGC 동등화: Full XC 에너지 → peak 진폭 (Legacy Set_From_Peak 계약)
    {
        const int64_t mag2_est = best_total >> 10;
        int32_t peak_mag_est =
            static_cast<int32_t>(int_sqrt64(mag2_est));
        if (peak_mag_est < 1) {
            peak_mag_est = 1;
        }
        pre_agc_.Set_From_Peak(peak_mag_est);
#if defined(HTS_DIAG_HOLO_CMYK) && !defined(HTS_PLATFORM_ARM)
        std::printf(
            "[CMYK-DIAG-AGC-AMI] best_total=%lld peak_mag_est=%d agc_sh=%d\n",
            static_cast<long long>(best_total),
            static_cast<int>(peak_mag_est),
            static_cast<int>(pre_agc_.Get_Shift()));
        std::fflush(stdout);
#endif
    }

    psal_off_ = chip_start;
    // Phase 3.C: INNOViD — Option C1: psal_e63_ from Full XC peak (best_total>>14)
    {
        const int64_t sh = best_total >> 14;
        if (sh <= 0) {
            psal_e63_ = 1;
        } else if (sh > static_cast<int64_t>(INT_MAX)) {
            psal_e63_ = INT_MAX;
        } else {
            psal_e63_ = static_cast<int32_t>(sh);
        }
    }
    if (psal_e63_ <= 0) {
        psal_e63_ = 1;
    }
#if defined(HTS_DIAG_HOLO_CMYK) && !defined(HTS_PLATFORM_ARM)
    {
        static int s_diag_cmyk_state_ami = 0;
        if (s_diag_cmyk_state_ami < 40) {
            std::printf(
                "[CMYK-DIAG-CMYK-STATE-AMI] n=%d "
                "psal_off=%d psal_e63=%d p0_chip_count=%d pre_phase=%d "
                "best_off=%d chip_start=%d best_total=%lld "
                "dom=%u estI=%d estQ=%d est_n=%d derot=%d "
                "agc_sh=%d cfo_sin14=%d cfo_ap=%d\n",
                s_diag_cmyk_state_ami, static_cast<int>(psal_off_),
                static_cast<int>(psal_e63_),
                static_cast<int>(p0_chip_count_),
                static_cast<int>(pre_phase_), best_off, chip_start,
                static_cast<long long>(best_total),
                static_cast<unsigned>(dominant_row_),
                static_cast<int>(est_I_), static_cast<int>(est_Q_),
                est_count_, derot_shift_,
                static_cast<int>(pre_agc_.Get_Shift()),
                static_cast<int>(cfo_.Get_Sin_Per_Chip_Q14()),
                cfo_.Is_Apply_Active() ? 1 : 0);
            ++s_diag_cmyk_state_ami;
            std::fflush(stdout);
        }
    }
#endif
    psal_commit_align_();

#if defined(HTS_DIAG_HOLO_CMYK) && !defined(HTS_PLATFORM_ARM)
    std::printf(
        "[CMYK-RX-AMI] off=%d pass=%d A=%d B=%d C=%d D=%d E=%d F=%d\n",
        best_off, pass ? 1 : 0, static_cast<int>(cube.face_A_q10),
        static_cast<int>(cube.face_B_q10), static_cast<int>(cube.face_C_q10),
        static_cast<int>(cube.face_D_q10), static_cast<int>(cube.face_E_q10),
        static_cast<int>(cube.face_F_q10));
    std::fflush(stdout);
#endif
}
#endif

#endif

void HTS_V400_Dispatcher::phase0_scan_() noexcept {
#ifdef HTS_USE_HOLOGRAPHIC_SYNC
    if (use_holographic_sync_) {
        phase0_scan_holographic_();
        return;
    }
#endif
#if defined(HTS_HOLO_PREAMBLE)
    phase0_scan_holo_preamble_rx_();
    return;
#else
    int32_t best_e63 = 0, second_e63 = 0;
    int best_off = -1;
    int64_t sum_all = 0;
    int best_dom_row = 63;
    int32_t best_seed_I = 0;
    int32_t best_seed_Q = 0;
#if defined(HTS_DIAG_PRINTF) && !defined(HTS_PHASE0_WALSH_BANK)
    static int s_stage4_scan_idx = 0;
    ++s_stage4_scan_idx;
    const bool diag_this_scan = (s_stage4_scan_idx <= 3);
    int cons_pass_count = 0;
    int best_cons_off = -1;
    int best_cons_r0 = -1;
    int best_cons_r1 = -1;
    int best_cons_r2 = -1;
    int64_t best_cons_e_sum = 0;
    {
        static int s_p0_buf_dump_idx = 0;
        ++s_p0_buf_dump_idx;
        if (s_p0_buf_dump_idx <= 3) {
            std::printf(
                "[BUF#%d] p0_buf128_I snapshot (at scan entry):\n",
                s_p0_buf_dump_idx);
            for (int blk = 0; blk < 3; ++blk) {
                int nz = 0;
                int abs_max = 0;
                int abs_sum = 0;
                for (int i = 0; i < 64; ++i) {
                    const int v = static_cast<int>(p0_buf128_I_[blk * 64 + i]);
                    const int a = (v >= 0) ? v : -v;
                    if (a > 100) {
                        ++nz;
                    }
                    if (a > abs_max) {
                        abs_max = a;
                    }
                    abs_sum += a;
                }
                std::printf("  blk%d: nonzero=%d/64 abs_max=%d abs_avg=%d\n",
                            blk, nz, abs_max, abs_sum / 64);
            }
            std::printf("  buf[0..15]_I:");
            for (int i = 0; i < 16; ++i) {
                std::printf(" %d", static_cast<int>(p0_buf128_I_[i]));
            }
            std::printf("\n");
            std::printf("  buf[64..79]_I:");
            for (int i = 0; i < 16; ++i) {
                std::printf(" %d", static_cast<int>(p0_buf128_I_[64 + i]));
            }
            std::printf("\n");
            std::printf("  buf[128..143]_I:");
            for (int i = 0; i < 16; ++i) {
                std::printf(" %d", static_cast<int>(p0_buf128_I_[128 + i]));
            }
            std::printf("\n");
        }
    }
    if (s_stage4_scan_idx >= 3 && s_stage4_scan_idx <= 5) {
        std::printf("[DEEP-BUF#%d] entire 192 chip I dump:\n",
                    s_stage4_scan_idx);
        for (int b = 0; b < 3; ++b) {
            std::printf("  blk%d I: ", b);
            for (int i = 0; i < 64; ++i) {
                std::printf("%d ",
                            static_cast<int>(p0_buf128_I_[b * 64 + i]));
            }
            std::printf("\n");
        }
        std::printf("[DEEP-BUF#%d] entire 192 chip Q dump:\n",
                    s_stage4_scan_idx);
        for (int b = 0; b < 3; ++b) {
            std::printf("  blk%d Q: ", b);
            for (int i = 0; i < 64; ++i) {
                std::printf("%d ",
                            static_cast<int>(p0_buf128_Q_[b * 64 + i]));
            }
            std::printf("\n");
        }
    }
    if (s_stage4_scan_idx == 3) {
        alignas(32) int32_t diag_T0_I[64];
        alignas(32) int32_t diag_T0_Q[64];
        alignas(32) int32_t diag_T1_I[64];
        alignas(32) int32_t diag_T1_Q[64];
        for (int i = 0; i < 64; ++i) {
            diag_T0_I[i] = static_cast<int32_t>(p0_buf128_I_[i]);
            diag_T0_Q[i] = static_cast<int32_t>(p0_buf128_Q_[i]);
            diag_T1_I[i] = static_cast<int32_t>(p0_buf128_I_[i + 64]);
            diag_T1_Q[i] = static_cast<int32_t>(p0_buf128_Q_[i + 64]);
        }
        fwht_64_complex_inplace_(diag_T0_I, diag_T0_Q);
        fwht_64_complex_inplace_(diag_T1_I, diag_T1_Q);
        std::printf(
            "[DEEP-FWHT] off=0 block 0 FWHT energy (selected rows):\n");
        {
            const int rows[] = {0,  1,  15, 30, 31, 32,
                                33, 47, 62, 63};
            for (unsigned ri = 0; ri < sizeof(rows) / sizeof(rows[0]); ++ri) {
                const int r = rows[ri];
                const int64_t e =
                    static_cast<int64_t>(diag_T0_I[r]) * diag_T0_I[r] +
                    static_cast<int64_t>(diag_T0_Q[r]) * diag_T0_Q[r];
                std::printf("  blk0 row%d energy=%lld\n", r,
                            static_cast<long long>(e));
            }
        }
        std::printf("[DEEP-FWHT] off=0 block 1 FWHT energy:\n");
        {
            const int rows[] = {0,  1,  15, 30, 31, 32,
                                33, 47, 62, 63};
            for (unsigned ri = 0; ri < sizeof(rows) / sizeof(rows[0]); ++ri) {
                const int r = rows[ri];
                const int64_t e =
                    static_cast<int64_t>(diag_T1_I[r]) * diag_T1_I[r] +
                    static_cast<int64_t>(diag_T1_Q[r]) * diag_T1_Q[r];
                std::printf("  blk1 row%d energy=%lld\n", r,
                            static_cast<long long>(e));
            }
        }
        auto print_top3 = [](const int32_t* T_I, const int32_t* T_Q,
                             const char* lbl) noexcept {
            int idx[3] = {-1, -1, -1};
            int64_t vals[3] = {0, 0, 0};
            for (int r = 0; r < 64; ++r) {
                const int64_t e =
                    static_cast<int64_t>(T_I[r]) * T_I[r] +
                    static_cast<int64_t>(T_Q[r]) * T_Q[r];
                if (e > vals[0]) {
                    vals[2] = vals[1];
                    idx[2] = idx[1];
                    vals[1] = vals[0];
                    idx[1] = idx[0];
                    vals[0] = e;
                    idx[0] = r;
                } else if (e > vals[1]) {
                    vals[2] = vals[1];
                    idx[2] = idx[1];
                    vals[1] = e;
                    idx[1] = r;
                } else if (e > vals[2]) {
                    vals[2] = e;
                    idx[2] = r;
                }
            }
            std::printf("  %s top3: row=[%d,%d,%d] e=[%lld,%lld,%lld]\n", lbl,
                        idx[0], idx[1], idx[2],
                        static_cast<long long>(vals[0]),
                        static_cast<long long>(vals[1]),
                        static_cast<long long>(vals[2]));
        };
        print_top3(diag_T0_I, diag_T0_Q, "blk0");
        print_top3(diag_T1_I, diag_T1_Q, "blk1");
    }
#endif

#if defined(HTS_DIAG_PRINTF) && !defined(HTS_PHASE0_WALSH_BANK)
    {
        static int s_p0_full_dump_idx = 0;
        ++s_p0_full_dump_idx;
        if (s_p0_full_dump_idx <= 5) {
            std::printf("[P0-DUMP#%d] p0_buf128_I (192 chips):\n",
                        s_p0_full_dump_idx);
            for (int i = 0; i < 192; ++i) {
                std::printf("%d%s", static_cast<int>(p0_buf128_I_[i]),
                            (i % 16 == 15) ? "\n" : ",");
            }
            std::printf("[P0-DUMP#%d] p0_buf128_Q (192 chips):\n",
                        s_p0_full_dump_idx);
            for (int i = 0; i < 192; ++i) {
                std::printf("%d%s", static_cast<int>(p0_buf128_Q_[i]),
                            (i % 16 == 15) ? "\n" : ",");
            }
        }
    }
#endif

    for (int off = 0; off < 64; ++off) {
        int32_t accum = 0;
        int max_row = 0;
        int32_t seed_I_fwht = 0;
        int32_t seed_Q_fwht = 0;
        int stage4_b1_row = 0;
#if defined(HTS_PHASE0_WALSH_BANK)
        // ── Walsh Bank Phase 0 — Full FWHT + Max Row + per-block T 보존 ──
        // 2-block FWHT 결과를 dominant row 에서 coherent 합산 → Phase 1 과
        // 동일 Walsh 도메인 시드 (walsh63_dot_ 재계산 경로와 분리).
        alignas(4) int32_t T_I[2][64];
        alignas(4) int32_t T_Q[2][64];
        int64_t row_e[64] = {0};
        for (int blk = 0; blk < 2; ++blk) {
            const int base = off + (blk << 6);
            for (int i = 0; i < 64; ++i) {
                T_I[blk][i] = static_cast<int32_t>(p0_buf128_I_[base + i]);
                T_Q[blk][i] = static_cast<int32_t>(p0_buf128_Q_[base + i]);
            }
            fwht_64_complex_inplace_(T_I[blk], T_Q[blk]);
            for (int r = 0; r < 64; ++r) {
                const int64_t e_I =
                    static_cast<int64_t>(T_I[blk][r]) * T_I[blk][r];
                const int64_t e_Q =
                    static_cast<int64_t>(T_Q[blk][r]) * T_Q[blk][r];
                row_e[r] += (e_I + e_Q);
            }
        }
        // Tie-break: row 63 (PRE_SYM0 / k_w63 FWHT) 동점 시 우선
        int64_t row_max_e = row_e[63];
        max_row = 63;
        for (int r = 0; r < 64; ++r) {
            if (r == 63) {
                continue;
            }
            if (row_e[r] > row_max_e) {
                row_max_e = row_e[r];
                max_row = r;
            }
        }
        accum = static_cast<int32_t>(row_max_e >> 16);
        seed_I_fwht = T_I[0][max_row] + T_I[1][max_row];
        seed_Q_fwht = T_Q[0][max_row] + T_Q[1][max_row];
#else
#if defined(HTS_TARGET_AMI)
        // ── AMI: 32-chip × 2 non-coherent per block (기존) ──
        int64_t e_nc = 0;
        for (int blk = 0; blk < 2; ++blk) {
            const int base = off + (blk << 6);
            for (int half = 0; half < 2; ++half) {
                int32_t dI = 0, dQ = 0;
                for (int j = 0; j < 32; ++j) {
                    const int idx = base + (half << 5) + j;
                    const int32_t sI =
                        static_cast<int32_t>(p0_buf128_I_[idx]);
                    const int32_t sQ =
                        static_cast<int32_t>(p0_buf128_Q_[idx]);
                    const int widx = (half << 5) + j;
                    if (k_w63[static_cast<std::size_t>(widx)] > 0) {
                        dI += sI;
                        dQ += sQ;
                    } else {
                        dI -= sI;
                        dQ -= sQ;
                    }
                }
                e_nc += static_cast<int64_t>(dI) * dI +
                        static_cast<int64_t>(dQ) * dQ;
            }
        }
        accum = static_cast<int32_t>(e_nc >> 16);
#else
        // Stage 4b (PS-LTE): 3×64 FWHT 동일 row 일관성 (pre_reps_>1 시 PRE_SYM0 반복)
        // p0_buf128_*_[192]: off+191<192 일 때만 3블록 안전 → off==0 만 4b 전체.
        // 그 외 off 는 기존 2블록 XOR+fallback (슬라이드 OOB 방지).
        alignas(32) int32_t T0_I[64];
        alignas(32) int32_t T0_Q[64];
        alignas(32) int32_t T1_I[64];
        alignas(32) int32_t T1_Q[64];
        alignas(32) int32_t T2_I[64];
        alignas(32) int32_t T2_Q[64];
        int r0 = 63;
        int r1 = 63;
        int r2 = 63;
        int max_row0 = 63;
        int max_row1 = 63;
        for (int i = 0; i < 64; ++i) {
            T0_I[i] = static_cast<int32_t>(p0_buf128_I_[off + i]);
            T0_Q[i] = static_cast<int32_t>(p0_buf128_Q_[off + i]);
        }
        fwht_64_complex_inplace_(T0_I, T0_Q);
        for (int i = 0; i < 64; ++i) {
            T1_I[i] = static_cast<int32_t>(p0_buf128_I_[off + 64 + i]);
            T1_Q[i] = static_cast<int32_t>(p0_buf128_Q_[off + 64 + i]);
        }
        fwht_64_complex_inplace_(T1_I, T1_Q);
        int64_t max_e0 = static_cast<int64_t>(T0_I[63]) * T0_I[63] +
                         static_cast<int64_t>(T0_Q[63]) * T0_Q[63];
        max_row0 = 63;
        for (int r = 0; r < 64; ++r) {
            if (r == 63) {
                continue;
            }
            const int64_t e = static_cast<int64_t>(T0_I[r]) * T0_I[r] +
                              static_cast<int64_t>(T0_Q[r]) * T0_Q[r];
            if (e > max_e0) {
                max_e0 = e;
                max_row0 = r;
            }
        }
        int64_t max_e1 = static_cast<int64_t>(T1_I[63]) * T1_I[63] +
                         static_cast<int64_t>(T1_Q[63]) * T1_Q[63];
        max_row1 = 63;
        for (int r = 0; r < 64; ++r) {
            if (r == 63) {
                continue;
            }
            const int64_t e = static_cast<int64_t>(T1_I[r]) * T1_I[r] +
                              static_cast<int64_t>(T1_Q[r]) * T1_Q[r];
            if (e > max_e1) {
                max_e1 = e;
                max_row1 = r;
            }
        }
        r0 = max_row0;
        r1 = max_row1;
        int64_t max_e2 = 0;
        if ((off + 128 + 63) < 192) {
            for (int i = 0; i < 64; ++i) {
                T2_I[i] = static_cast<int32_t>(p0_buf128_I_[off + 128 + i]);
                T2_Q[i] = static_cast<int32_t>(p0_buf128_Q_[off + 128 + i]);
            }
            fwht_64_complex_inplace_(T2_I, T2_Q);
#if defined(HTS_DIAG_PRINTF) && !defined(HTS_PHASE0_WALSH_BANK)
            {
                static int s_p0_fwht_dump_idx = 0;
                // PROMPT 39: 전역 한도가 S8 이전에 고갈되지 않도록 상향 (DIAG 전용)
                if (off == 0 && s_p0_fwht_dump_idx < 5000) {
                    ++s_p0_fwht_dump_idx;
                    auto print_top5 = [](const char *label, const int32_t *I,
                                         const int32_t *Q) noexcept {
                        int64_t e[64];
                        for (int m = 0; m < 64; ++m) {
                            e[m] = static_cast<int64_t>(I[m]) * I[m] +
                                   static_cast<int64_t>(Q[m]) * Q[m];
                        }
                        int top_bin[5] = {0, 0, 0, 0, 0};
                        int64_t top_e[5] = {INT64_MIN, INT64_MIN, INT64_MIN,
                                            INT64_MIN, INT64_MIN};
                        for (int m = 0; m < 64; ++m) {
                            for (int k = 0; k < 5; ++k) {
                                if (e[m] > top_e[k]) {
                                    for (int j = 4; j > k; --j) {
                                        top_e[j] = top_e[j - 1];
                                        top_bin[j] = top_bin[j - 1];
                                    }
                                    top_e[k] = e[m];
                                    top_bin[k] = m;
                                    break;
                                }
                            }
                        }
                        std::printf("[P0-FWHT-RAW] %s: ", label);
                        for (int k = 0; k < 5; ++k) {
                            std::printf(
                                "bin%d=%lld ", top_bin[k],
                                static_cast<long long>(top_e[k]));
                        }
                        std::printf("\n");
                    };
                    print_top5("block0", T0_I, T0_Q);
                    print_top5("block1", T1_I, T1_Q);
                    print_top5("block2", T2_I, T2_Q);
                    {
                        auto block_total_e = [](const int32_t *I,
                                                const int32_t *Q) noexcept
                            -> int64_t {
                            int64_t s = 0;
                            for (int m = 0; m < 64; ++m) {
                                s += static_cast<int64_t>(I[m]) * I[m] +
                                     static_cast<int64_t>(Q[m]) * Q[m];
                            }
                            return s;
                        };
                        auto row_energy = [](const int32_t *I, const int32_t *Q,
                                             int r) noexcept -> int64_t {
                            return static_cast<int64_t>(I[r]) * I[r] +
                                   static_cast<int64_t>(Q[r]) * Q[r];
                        };
                        std::printf(
                            "[P0-BLOCK-TOTAL] block=0 total_e=%lld\n",
                            static_cast<long long>(
                                block_total_e(T0_I, T0_Q)));
                        std::printf(
                            "[P0-BLOCK-TOTAL] block=1 total_e=%lld\n",
                            static_cast<long long>(
                                block_total_e(T1_I, T1_Q)));
                        std::printf(
                            "[P0-BLOCK-TOTAL] block=2 total_e=%lld\n",
                            static_cast<long long>(
                                block_total_e(T2_I, T2_Q)));
                        for (int b = 0; b < 3; ++b) {
                            const int32_t *Ib =
                                (b == 0) ? T0_I : ((b == 1) ? T1_I : T2_I);
                            const int32_t *Qb =
                                (b == 0) ? T0_Q : ((b == 1) ? T1_Q : T2_Q);
                            std::printf(
                                "[P0-PINPOINT] block=%d e_bin0=%lld "
                                "e_bin6=%lld e_bin63=%lld\n",
                                b,
                                static_cast<long long>(
                                    row_energy(Ib, Qb, 0)),
                                static_cast<long long>(
                                    row_energy(Ib, Qb, 6)),
                                static_cast<long long>(
                                    row_energy(Ib, Qb, 63)));
                        }
                    }
                }
            }
#endif
            max_e2 = static_cast<int64_t>(T2_I[63]) * T2_I[63] +
                     static_cast<int64_t>(T2_Q[63]) * T2_Q[63];
            r2 = 63;
            for (int r = 0; r < 64; ++r) {
                if (r == 63) {
                    continue;
                }
                const int64_t e = static_cast<int64_t>(T2_I[r]) * T2_I[r] +
                                  static_cast<int64_t>(T2_Q[r]) * T2_Q[r];
                if (e > max_e2) {
                    max_e2 = e;
                    r2 = r;
                }
            }
            const bool cons_ok = (r0 == r1) && (r1 == r2);
            int32_t accum_cohr = 0;
            if (cons_ok) {
                // 기존 2블록 (e0+e1)>>16 판정 스케일에 맞춤: 동일 E면 2E/65536 ≡
                // (2/3)·(3E)/65536
                const int64_t e_scaled =
                    ((max_e0 + max_e1 + max_e2) * 2) / 3;
                accum_cohr = static_cast<int32_t>(e_scaled >> 16);
            } else {
                const int64_t e63_0 =
                    static_cast<int64_t>(T0_I[63]) * T0_I[63] +
                    static_cast<int64_t>(T0_Q[63]) * T0_Q[63];
                const int64_t e63_1 =
                    static_cast<int64_t>(T1_I[63]) * T1_I[63] +
                    static_cast<int64_t>(T1_Q[63]) * T1_Q[63];
                const int64_t e63_2 =
                    static_cast<int64_t>(T2_I[63]) * T2_I[63] +
                    static_cast<int64_t>(T2_Q[63]) * T2_Q[63];
                const int64_t e63_scaled = ((e63_0 + e63_1 + e63_2) * 2) / 3;
                accum_cohr = static_cast<int32_t>(e63_scaled >> 16);
            }
            // Stage 5: 8×8 non-coh (row 7) — 고 CFO에서 coherent 대비 이득
            const int64_t e8x8_0 =
                energy_8x8_noncoh_row7_(&p0_buf128_I_[off],
                                         &p0_buf128_Q_[off]);
            const int64_t e8x8_1 =
                energy_8x8_noncoh_row7_(&p0_buf128_I_[off + 64],
                                         &p0_buf128_Q_[off + 64]);
            const int64_t e8x8_2 =
                energy_8x8_noncoh_row7_(&p0_buf128_I_[off + 128],
                                         &p0_buf128_Q_[off + 128]);
            const int64_t e8x8_sum = e8x8_0 + e8x8_1 + e8x8_2;
            // >>14: 8×8 합을 64칩 coherent(3블록) 스케일에 근사 정합
            const int32_t accum_8x8 =
                static_cast<int32_t>(((e8x8_sum * 2) / 3) >> 14);
            accum = (accum_8x8 > accum_cohr) ? accum_8x8 : accum_cohr;
#if defined(HTS_DIAG_PRINTF) && !defined(HTS_PHASE0_WALSH_BANK)
            if (diag_this_scan) {
                if (cons_ok) {
                    ++cons_pass_count;
                    const int64_t e_triple = max_e0 + max_e1 + max_e2;
                    if (best_cons_off < 0 || e_triple > best_cons_e_sum) {
                        best_cons_e_sum = e_triple;
                        best_cons_off = off;
                        best_cons_r0 = r0;
                        best_cons_r1 = r1;
                        best_cons_r2 = r2;
                    }
                }
            }
            {
                static int s_stage5_diag = 0;
                if (off == 0 && s_stage5_diag < 5) {
                    ++s_stage5_diag;
                    std::printf(
                        "[STAGE5] off=0 cons=%d e_cohr=%d e_8x8_scaled=%d "
                        "accum=%d\n",
                        cons_ok ? 1 : 0, accum_cohr, accum_8x8, accum);
                }
            }
#endif
        } else {
            // 2블록 레거시: XOR=63 (PRE_SYM0/1) + row63 fallback
            int64_t max_e1b = static_cast<int64_t>(T1_I[0]) * T1_I[0] +
                              static_cast<int64_t>(T1_Q[0]) * T1_Q[0];
            max_row1 = 0;
            for (int r = 0; r < 64; ++r) {
                if (r == 0) {
                    continue;
                }
                const int64_t e = static_cast<int64_t>(T1_I[r]) * T1_I[r] +
                                  static_cast<int64_t>(T1_Q[r]) * T1_Q[r];
                if (e > max_e1b) {
                    max_e1b = e;
                    max_row1 = r;
                }
            }
            r1 = max_row1;
            r2 = max_row1;
            const bool xor_ok = ((max_row0 ^ max_row1) == 63);
            if (xor_ok) {
                const int64_t e_sum = max_e0 + max_e1b;
                accum = static_cast<int32_t>(e_sum >> 16);
            } else {
                const int64_t e63_0 =
                    static_cast<int64_t>(T0_I[63]) * T0_I[63] +
                    static_cast<int64_t>(T0_Q[63]) * T0_Q[63];
                const int64_t e63_1 =
                    static_cast<int64_t>(T1_I[63]) * T1_I[63] +
                    static_cast<int64_t>(T1_Q[63]) * T1_Q[63];
                accum = static_cast<int32_t>((e63_0 + e63_1) >> 16);
            }
        }
#if defined(HTS_DIAG_PRINTF) && !defined(HTS_PHASE0_WALSH_BANK)
        {
            static int s_stage4b_diag_line = 0;
            if ((off == 0 || off == 32 || off == 63) &&
                s_stage4b_diag_line < 30) {
                ++s_stage4b_diag_line;
                const bool full3 = ((off + 128 + 63) < 192);
                const int cons_show =
                    full3 ? (((r0 == r1) && (r1 == r2)) ? 1 : 0) : -1;
                std::printf(
                    "[STAGE4B] off=%d full3=%d r0=%d r1=%d r2=%d cons=%d "
                    "accum=%d\n",
                    off, full3 ? 1 : 0, r0, r1, r2, cons_show, accum);
            }
        }
#endif
        max_row = max_row0;
        stage4_b1_row = max_row1;
#endif
#endif
        sum_all += static_cast<int64_t>(accum);
        if (accum > best_e63) {
            second_e63 = best_e63;
            best_e63 = accum;
            best_off = off;
#if defined(HTS_PHASE0_WALSH_BANK)
            best_dom_row = max_row;
            best_seed_I = seed_I_fwht;
            best_seed_Q = seed_Q_fwht;
#if defined(HTS_DIAG_PRINTF)
            std::printf(
                "[P0-ROW-NEW] off=%d dom_row=%d max_e=%d seed=(%d,%d)\n",
                off, max_row, accum, seed_I_fwht, seed_Q_fwht);
#endif
#elif !defined(HTS_TARGET_AMI)
            // Fix A1 (V28): 3-way 일치 시 r0, 아니면 r2.
            // Fix A3 (V34): block-2 FWHT 에너지에서 coherent pair 순차 excision
            // (bin63 하드코딩 없음). pair1=(top1,top2) → pair2=(top3,top4) 검사,
            // 2-level jam이면 top5, 1-level이면 top3를 preamble 후보로 채택.
            const bool full3blk_dom = ((off + 128 + 63) < 192);
            int best_dom_pick = (r0 == r1 && r1 == r2) ? r0 : r2;
            if (full3blk_dom) {
                int64_t e_b2[64];
                for (int m = 0; m < 64; ++m) {
                    e_b2[m] = static_cast<int64_t>(T2_I[m]) * T2_I[m] +
                              static_cast<int64_t>(T2_Q[m]) * T2_Q[m];
                }
                int t1_bin = 0;
                int64_t t1_e = 0;
                find_argmax_64(e_b2, &t1_bin, &t1_e);
                int t2_bin = 0;
                int64_t t2_e = 0;
                int final_pick = best_dom_pick;
                int a3_level = 0;
                int t3_bin_dbg = -1;
                int t4_bin_dbg = -1;
                int t5_bin_dbg = -1;
                int64_t t3_e_dbg = -1;
                int64_t t4_e_dbg = -1;
                int64_t t5_e_dbg = -1;
                if (t1_e > 0) {
                    const int64_t t1_saved = e_b2[t1_bin];
                    e_b2[t1_bin] = 0;
                    find_argmax_64(e_b2, &t2_bin, &t2_e);
                    const bool pair1_coherent =
                        (t2_e >= (t1_e - (t1_e >> 4))) &&
                        (t1_bin != t2_bin);
                    if (pair1_coherent) {
                        const int64_t t2_saved = e_b2[t2_bin];
                        e_b2[t2_bin] = 0;
                        int t3_bin = 0;
                        int64_t t3_e = 0;
                        find_argmax_64(e_b2, &t3_bin, &t3_e);
                        t3_bin_dbg = t3_bin;
                        t3_e_dbg = t3_e;
                        const int64_t t3_saved = e_b2[t3_bin];
                        e_b2[t3_bin] = 0;
                        int t4_bin = 0;
                        int64_t t4_e = 0;
                        find_argmax_64(e_b2, &t4_bin, &t4_e);
                        t4_bin_dbg = t4_bin;
                        t4_e_dbg = t4_e;
                        const bool pair2_coherent =
                            (t3_e > 0) &&
                            (t4_e >= (t3_e - (t3_e >> 4))) &&
                            (t3_bin != t4_bin);
                        if (pair2_coherent) {
                            e_b2[t4_bin] = 0;
                            int t5_bin = 0;
                            int64_t t5_e = 0;
                            find_argmax_64(e_b2, &t5_bin, &t5_e);
                            t5_bin_dbg = t5_bin;
                            t5_e_dbg = t5_e;
                            final_pick = t5_bin;
                            a3_level = 2;
                            e_b2[t3_bin] = t3_saved;
                            e_b2[t2_bin] = t2_saved;
                        } else {
                            final_pick = t3_bin;
                            a3_level = 1;
                            e_b2[t3_bin] = t3_saved;
                            e_b2[t2_bin] = t2_saved;
                        }
                    }
                    e_b2[t1_bin] = t1_saved;
                }
                best_dom_pick = final_pick;
#if defined(HTS_DIAG_PRINTF)
                if (a3_level > 0) {
                    static int s_fixa3_dump = 0;
                    if (s_fixa3_dump < 50) {
                        ++s_fixa3_dump;
                        std::printf(
                            "[FIXA3] off=%d r0=%d r1=%d r2=%d level=%d "
                            "t1=(%d,%lld) t2=(%d,%lld) t3=(%d,%lld) t4=(%d,%lld) "
                            "t5=(%d,%lld) pick=%d (A1=%d)\n",
                            off, r0, r1, r2, a3_level, t1_bin,
                            static_cast<long long>(t1_e), t2_bin,
                            static_cast<long long>(t2_e), t3_bin_dbg,
                            static_cast<long long>(t3_e_dbg), t4_bin_dbg,
                            static_cast<long long>(t4_e_dbg), t5_bin_dbg,
                            static_cast<long long>(t5_e_dbg), final_pick,
                            (r0 == r1 && r1 == r2) ? r0 : r2);
                    }
                }
#endif
            }
            best_dom_row = best_dom_pick;
#if defined(HTS_DIAG_PRINTF)
            std::printf(
                "[STAGE4-P0] off=%d r0=%d r1=%d r2=%d xor_legacy=%d accum=%d "
                "bdr=%d\n",
                off, max_row, stage4_b1_row, r2,
                (max_row ^ stage4_b1_row), accum, best_dom_row);
#endif
#endif
        } else if (accum > second_e63) {
            second_e63 = accum;
        }
    }

#if defined(HTS_WALSH_ROW_DIAG) && !defined(HTS_PHASE0_WALSH_BANK) && \
    !defined(HTS_TARGET_AMI)
    // accum 은 3블록 중 임의 조합(8×8 vs coherent)에서 나올 수 있어,
    // 승리 off 의 "첫 64칩"만 FWHT 하면 에너지가 0인 채로 남는 경우가 있다.
    // 계측: best_off 에서 사용 가능한 각 64칩 블록을 FWHT 한 뒤,
    // row-peak 가 가장 큰 블록의 64-bin 에너지를 기록 (판정 로직 무변경).
    if (best_off >= 0) {
        const int off = best_off;
        const int nblk = ((off + 128 + 63) < 192) ? 3 : 2;
        alignas(32) int32_t wdiag_TI[64];
        alignas(32) int32_t wdiag_TQ[64];
        int64_t wdiag_e_arr[64]{};
        int64_t pick_peak = -1;
        for (int b = 0; b < nblk; ++b) {
            const int base = off + (b << 6);
            for (int i = 0; i < 64; ++i) {
                wdiag_TI[i] = static_cast<int32_t>(p0_buf128_I_[base + i]);
                wdiag_TQ[i] = static_cast<int32_t>(p0_buf128_Q_[base + i]);
            }
            fwht_64_complex_inplace_(wdiag_TI, wdiag_TQ);
            int64_t blk_peak = 0;
            int64_t e_row[64];
            for (int r = 0; r < 64; ++r) {
                const int64_t e =
                    static_cast<int64_t>(wdiag_TI[r]) * wdiag_TI[r] +
                    static_cast<int64_t>(wdiag_TQ[r]) * wdiag_TQ[r];
                e_row[r] = e;
                if (e > blk_peak) {
                    blk_peak = e;
                }
            }
            if (blk_peak > pick_peak) {
                pick_peak = blk_peak;
                for (int r = 0; r < 64; ++r) {
                    wdiag_e_arr[r] = e_row[r];
                }
            }
        }
        WalshRowDiag::record_row_energies(wdiag_e_arr);
    }
#endif

    const int64_t sum_others = sum_all - static_cast<int64_t>(best_e63);

    // r_avg >= 5: (best<<6)-best >= (so<<3)+(so<<1)+(so) = so*5...
    // 아니, 5 = (1<<2)+1: so*5 = (so<<2)+so
    // r_avg >= 5 ↔ best*63 >= so*5
    const int64_t best_x63 = (static_cast<int64_t>(best_e63) << 6) -
                              static_cast<int64_t>(best_e63);
    const int64_t so_x5    = (sum_others << 2) + sum_others;
    const bool r_avg_ok = (sum_others <= 0) || (best_x63 >= so_x5);

    // r_avg >= 8 판정 (sep_ok 활성화용, 곱셈 0)
    // best*63 >= so*8 ↔ best_x63 >= so<<3
    const bool r_avg_high = (sum_others <= 0) ||
        (best_x63 >= (sum_others << 3));

    // 적응형 sep_ok: r_avg≥8일 때만 alias 방어
    const int64_t best_x4 = static_cast<int64_t>(best_e63) << 2;
    const int64_t sec_x5  = (static_cast<int64_t>(second_e63) << 2) +
                             static_cast<int64_t>(second_e63);
    const bool sep_ok = (!r_avg_high) ||
        (second_e63 == 0) || (best_x4 > sec_x5);

    // 적응형 e63_min: noise_floor × 5 (하한 5000), amp×38 하한 (TPC tx_amp_ 연동)
    const int32_t avg_others =
        (sum_others > 0)
            ? static_cast<int32_t>((sum_others * 1040LL) >> 16)
            : 0;
    // avg_others * 5 = (avg<<2)+avg [SHIFT]
    const int32_t adaptive_min =
        (avg_others > 0)
            ? static_cast<int32_t>((static_cast<int64_t>(avg_others) << 2) +
                                    avg_others)
            : 5000;
    // 적응형: AMI amp×19 / PS-LTE amp×38 (shift 조합)
    const int32_t amp32 = static_cast<int32_t>(tx_amp_);
#if defined(HTS_TARGET_AMI)
    // AMI 32-chip × 4 non-coherent: clean peak 약 1/2 → amp × 19
    const int32_t k_E63_ALIGN_MIN =
        (amp32 << 4) + (amp32 << 1) + amp32;  // amp × 19
#else
    // PS-LTE 64-chip × 2 non-coherent: 기존 amp × 38
    const int32_t k_E63_ALIGN_MIN =
        (amp32 << 5) + (amp32 << 2) + (amp32 << 1);  // amp × 38
#endif
    const int32_t e63_min =
        (adaptive_min > k_E63_ALIGN_MIN) ? adaptive_min : k_E63_ALIGN_MIN;

    const bool pass = (best_off >= 0 && r_avg_ok &&
                       best_e63 >= e63_min && sep_ok);

#if defined(HTS_SYNC_DIAG)
    SyncDiag::record_scan(best_off, static_cast<int64_t>(best_e63),
                          static_cast<int64_t>(second_e63), pass);
#endif

#if defined(HTS_DIAG_PRINTF) && !defined(HTS_PHASE0_WALSH_BANK)
    if (diag_this_scan) {
        std::printf(
            "[STAGE4-SCAN#%d] cons_pass=%d best_cons_off=%d r0=%d r1=%d r2=%d "
            "best_cons_esum>>16=%lld final_best_off=%d best_e63=%d second_e63=%d\n",
            s_stage4_scan_idx, cons_pass_count, best_cons_off, best_cons_r0,
            best_cons_r1, best_cons_r2,
            static_cast<long long>(
                (best_cons_e_sum > 0) ? (best_cons_e_sum >> 16) : 0LL),
            best_off, static_cast<int>(best_e63),
            static_cast<int>(second_e63));
        std::printf(
            "[STAGE4-JUDGE#%d] best_off=%d best_e63=%d e63_min=%d "
            "r_avg_ok=%d sep_ok=%d pass=%d sum_others=%lld\n",
            s_stage4_scan_idx, best_off, static_cast<int>(best_e63), e63_min,
            r_avg_ok ? 1 : 0, sep_ok ? 1 : 0, pass ? 1 : 0,
            static_cast<long long>(sum_others));
    }
#endif

#if defined(HTS_DIAG_PRINTF) && defined(HTS_PHASE0_WALSH_BANK)
    // ─── DIAG-SCAN-STAT: scan pass/fail 누적 (로직 무변경) ───
    {
        static int s_scan_total = 0;
        static int s_scan_pass = 0;
        static int s_scan_fail = 0;
        ++s_scan_total;
        if (pass) {
            ++s_scan_pass;
        } else {
            ++s_scan_fail;
        }
        if (s_scan_total > 0 && (s_scan_total % 20) == 0) {
            const int ratio_pct = (s_scan_pass * 100) / s_scan_total;
            std::printf(
                "[DIAG-SCAN-STAT] total=%d pass=%d fail=%d pass_ratio=%d%%\n",
                s_scan_total, s_scan_pass, s_scan_fail, ratio_pct);
        }
    }
#endif

#if defined(HTS_DIAG_PRINTF) && defined(HTS_PHASE0_WALSH_BANK)
    // ─── DIAG-SCAN-FULL: pass 확정 후에만 (빈 버퍼 소모 방지), 최대 2회 ───
    {
        static int s_scan_full_count = 0;
        if (pass && s_scan_full_count < 2) {
            ++s_scan_full_count;
            std::printf("[DIAG-SCAN-FULL] dump#%d — all off scan result\n",
                        s_scan_full_count);
            for (int off = 0; off < 64; ++off) {
                alignas(4) int32_t T_I[2][64];
                alignas(4) int32_t T_Q[2][64];
                int64_t row_e[64] = {0};
                for (int blk = 0; blk < 2; ++blk) {
                    const int base = off + (blk << 6);
                    for (int i = 0; i < 64; ++i) {
                        T_I[blk][i] = static_cast<int32_t>(
                            p0_buf128_I_[base + i]);
                        T_Q[blk][i] = static_cast<int32_t>(
                            p0_buf128_Q_[base + i]);
                    }
                    fwht_64_complex_inplace_(T_I[blk], T_Q[blk]);
                    for (int r = 0; r < 64; ++r) {
                        row_e[r] +=
                            static_cast<int64_t>(T_I[blk][r]) * T_I[blk][r] +
                            static_cast<int64_t>(T_Q[blk][r]) * T_Q[blk][r];
                    }
                }
                int64_t max_e = row_e[0];
                int max_r = 0;
                for (int r = 1; r < 64; ++r) {
                    if (row_e[r] > max_e) {
                        max_e = row_e[r];
                        max_r = r;
                    }
                }
                const int64_t e63 = row_e[63];
                if ((off & 3) == 0) {
                    std::printf(
                        "  off=%2d row_max=%2d e_max=%10lld e63=%10lld\n",
                        off, max_r,
                        static_cast<long long>(max_e >> 16),
                        static_cast<long long>(e63 >> 16));
                }
            }
        }
    }
#endif

#if defined(HTS_DIAG_PRINTF)
    {
        const int32_t r_avg =
            (avg_others > 0)
                ? static_cast<int32_t>(static_cast<int64_t>(best_e63) /
                                       static_cast<int64_t>(avg_others))
                : 999;
        const int32_t r_sep =
            (second_e63 > 0)
                ? static_cast<int32_t>(static_cast<int64_t>(best_e63) /
                                       static_cast<int64_t>(second_e63))
                : 999;
        std::printf("[P0-SCAN] off=%d e63=%d avg_o=%d r_avg=%d r_sep=%d emin=%d sep=%d\n",
                    best_off, best_e63, avg_others, r_avg, r_sep,
                    e63_min, static_cast<int>(sep_ok));
    }
#endif

    if (pass) {
#if defined(HTS_DIAG_PRINTF) && defined(HTS_PHASE0_WALSH_BANK)
        // ─── DIAG-BUF: buf128 실측 dump (Phase 0 detect 직전, 로직 무변경) ───
        {
            static int s_buf_dump_count = 0;
            if (s_buf_dump_count < 3) {
                ++s_buf_dump_count;
                std::printf(
                    "[DIAG-BUF] dump#%d best_off=%d best_dom_row=%d best_e63=%d\n",
                    s_buf_dump_count, best_off, best_dom_row, best_e63);
                std::printf("[DIAG-BUF] buf[0..15]_I:");
                for (int i = 0; i < 16; ++i) {
                    std::printf(" %d", static_cast<int>(p0_buf128_I_[i]));
                }
                std::printf("\n");
                std::printf("[DIAG-BUF] buf[0..15]_Q:");
                for (int i = 0; i < 16; ++i) {
                    std::printf(" %d", static_cast<int>(p0_buf128_Q_[i]));
                }
                std::printf("\n");
                std::printf("[DIAG-BUF] buf[16..31]_I:");
                for (int i = 16; i < 32; ++i) {
                    std::printf(" %d", static_cast<int>(p0_buf128_I_[i]));
                }
                std::printf("\n");
                if (best_off >= 0) {
                    std::printf("[DIAG-BUF] buf[off+0..+15]_I (off=%d):",
                                best_off);
                    for (int i = 0; i < 16; ++i) {
                        std::printf(" %d",
                                    static_cast<int>(
                                        p0_buf128_I_[best_off + i]));
                    }
                    std::printf("\n");
                    std::printf("[DIAG-BUF] buf[off+0..+15]_Q:");
                    for (int i = 0; i < 16; ++i) {
                        std::printf(" %d",
                                    static_cast<int>(
                                        p0_buf128_Q_[best_off + i]));
                    }
                    std::printf("\n");
                    std::printf("[DIAG-BUF] buf[off+64..+79]_I:");
                    for (int i = 0; i < 16; ++i) {
                        std::printf(" %d",
                                    static_cast<int>(
                                        p0_buf128_I_[best_off + 64 + i]));
                    }
                    std::printf("\n");
                    std::printf(
                        "[DIAG-BUF] buf[off+128..+143]_I (header?, <192):");
                    for (int i = 0; i < 16; ++i) {
                        const int idx = best_off + 128 + i;
                        if (idx >= 192) {
                            std::printf(" NA");
                        } else {
                            std::printf(" %d",
                                        static_cast<int>(p0_buf128_I_[idx]));
                        }
                    }
                    std::printf("\n");
                }
                std::printf(
                    "[DIAG-BUF] k_w63[0..15] expected (assume amp=1000):");
                for (int i = 0; i < 16; ++i) {
                    std::printf(" %d",
                                static_cast<int>(k_w63[static_cast<std::size_t>(
                                    i)]) * 1000);
                }
                std::printf("\n");
            }
        }
#endif
#if defined(HTS_TARGET_AMI) && !defined(HTS_PHASE0_WALSH_BANK)
        // Stage A-0 Step 2: AMI best_dom_row 산출 (pass 확정 후 FWHT 2회)
        // WALSH_BANK L243-L272 패턴 축소 차용
        {
            alignas(4) int32_t T_I[2][64];
            alignas(4) int32_t T_Q[2][64];
            int64_t row_e[64] = {0};
            for (int blk = 0; blk < 2; ++blk) {
                const int base = best_off + (blk << 6);
                for (int i = 0; i < 64; ++i) {
                    T_I[blk][i] = static_cast<int32_t>(p0_buf128_I_[base + i]);
                    T_Q[blk][i] = static_cast<int32_t>(p0_buf128_Q_[base + i]);
                }
                fwht_64_complex_inplace_(T_I[blk], T_Q[blk]);
                for (int r = 0; r < 64; ++r) {
                    row_e[r] +=
                        static_cast<int64_t>(T_I[blk][r]) * T_I[blk][r] +
                        static_cast<int64_t>(T_Q[blk][r]) * T_Q[blk][r];
                }
            }
            // argmax with 63 tie-break (WALSH_BANK 관례)
            int ami_max_row = 63;
            int64_t ami_max_e = row_e[63];
            for (int r = 0; r < 64; ++r) {
                if (row_e[r] > ami_max_e) {
                    ami_max_e = row_e[r];
                    ami_max_row = r;
                }
            }
            best_dom_row = ami_max_row;
#if defined(HTS_DIAG_PRINTF)
            std::printf(
                "[AMI-DOM] best_off=%d dom_row=%d e_max>>16=%lld\n",
                best_off, ami_max_row,
                static_cast<long long>(ami_max_e >> 16));
#endif
        }
#endif
        dominant_row_ = static_cast<uint8_t>(best_dom_row);
#if defined(HTS_DIAG_PRINTF) && !defined(HTS_PHASE0_WALSH_BANK)
        std::printf(
            "[STAGE4-P0] lock dominant_row=%u off=%d (seed=walsh63 non-coh)\n",
            static_cast<unsigned>(dominant_row_), best_off);
#endif
        {
            int32_t seed_dot_I = 0, seed_dot_Q = 0;
#if defined(HTS_PHASE0_WALSH_BANK)
            seed_dot_I = best_seed_I;
            seed_dot_Q = best_seed_Q;
#elif defined(HTS_TARGET_AMI)
            for (int blk = 0; blk < 2; ++blk) {
                int32_t di = 0, dq = 0;
                walsh63_dot_(&p0_buf128_I_[best_off + (blk << 6)],
                             &p0_buf128_Q_[best_off + (blk << 6)],
                             di, dq);
                seed_dot_I += di;
                seed_dot_Q += dq;
            }
#else
            // Stage 4: 스캔은 XOR=63 지문만; 시드는 블록별 walsh63 non-coherent 합
            for (int blk = 0; blk < 2; ++blk) {
                int32_t di = 0, dq = 0;
                walsh63_dot_(&p0_buf128_I_[best_off + (blk << 6)],
                             &p0_buf128_Q_[best_off + (blk << 6)],
                             di, dq);
                seed_dot_I += di;
                seed_dot_Q += dq;
            }
#endif
#ifdef HTS_WALSH_V5_PREAMBLE
            // [Step B DIAG] v5 2블록 Walsh-row 시퀀스 매칭 score
            // (기존 walsh63_dot_ seed 와 병렬 계산, 의사결정 비개입)
            if (best_off + 128 <= 192) {
                const int64_t v5_score = walsh_v5_score_2blk_(
                    &p0_buf128_I_[best_off],
                    &p0_buf128_Q_[best_off]);
#if defined(HTS_DIAG_PRINTF)
                std::printf(
                    "[STAGE4-V5-2B] off=%d v5_score=%lld legacy_seed=(%d,%d)\n",
                    best_off, static_cast<long long>(v5_score),
                    seed_dot_I, seed_dot_Q);
#endif
            }
#endif  // HTS_WALSH_V5_PREAMBLE
            est_I_ = seed_dot_I;
            est_Q_ = seed_dot_Q;
            est_count_ = 2;
            // B-2: est seed 확정 직후 mag/recip 즉시 계산
            //   이후 on_sym_ 심볼당 정규화 derotation 사용
            update_derot_shift_from_est_();
            // CFO 는 Walsh 도메인에서 처리 예정 — MCE 철거 (Stage 1)
            // ★ [CFO 4-1] Estimate 복원 (Apply 는 4-2)
            //   블록0/블록1 dot 을 재계산 (seed_dot_* 은 2블록 합이라 분리 불가).
            //   phase0_scan_ 은 P0 락 1회뿐 → 추가 walsh63_dot_ 2회 WCET 영향 무시.
            {
                int32_t d0I = 0, d0Q = 0;
                int32_t d1I = 0, d1Q = 0;
                walsh63_dot_(&p0_buf128_I_[best_off + 0],
                             &p0_buf128_Q_[best_off + 0],
                             d0I, d0Q);
                walsh63_dot_(&p0_buf128_I_[best_off + 64],
                             &p0_buf128_Q_[best_off + 64],
                             d1I, d1Q);
                cfo_.Estimate_From_Preamble(d0I, d0Q, d1I, d1Q, 64);
                // [CFO 4-3] P0 스캔 구간 192 chip 위상 누적 전진
                //           payload 첫 Apply 시 위상이 스캔 구간 끝과 정합
                cfo_.Advance_Phase_Only(192);
            }
            // 프리앰블 AGC: P0 피크에서 수신 진폭 측정
            {
                int32_t mag_sum = 0;
                for (int j = 0; j < 64; ++j) {
                    const int32_t ai = p0_buf128_I_[best_off + j];
                    const int32_t aq = p0_buf128_Q_[best_off + j];
                    const int32_t si = ai >> 31;
                    mag_sum += (ai ^ si) - si;
                    const int32_t sq = aq >> 31;
                    mag_sum += (aq ^ sq) - sq;
                }
                const int32_t peak_avg = mag_sum >> 6;
                pre_agc_.Set_From_Peak(peak_avg);
            }
#if defined(HTS_DIAG_PRINTF)
            std::printf("[P0-SEED] dot=(%d,%d) est=(%d,%d) n=%d\n",
                        seed_dot_I, seed_dot_Q, est_I_, est_Q_,
                        est_count_);
#endif
#if defined(HTS_PHASE0_WALSH_BANK) && defined(HTS_DIAG_PRINTF)
            std::printf("[P0-WBANK] commit_off=%d dominant_row=%u seed=(%d,%d)\n",
                        best_off, static_cast<unsigned>(dominant_row_),
                        seed_dot_I, seed_dot_Q);
#endif
        }
#if defined(HTS_DIAG_PRINTF)
        std::printf("[P0-ALIGNED] off=%d carry=%d\n",
                    best_off, 64 - best_off);
#endif
        psal_off_ = best_off;
        psal_e63_ = best_e63;
        psal_commit_align_();
    } else {
#if defined(HTS_DIAG_PRINTF) && defined(HTS_PHASE0_WALSH_BANK)
        {
            static int s_shift_count = 0;
            ++s_shift_count;
            if (s_shift_count <= 3) {
                std::printf(
                    "[DIAG-SHIFT] count=%d before_shift buf[128..143]_I:",
                    s_shift_count);
                for (int i = 0; i < 16; ++i) {
                    std::printf(" %d",
                                static_cast<int>(p0_buf128_I_[128 + i]));
                }
                std::printf("\n");
            }
        }
#endif
        std::memcpy(p0_buf128_I_, p0_buf128_I_ + 128,
                    static_cast<size_t>(64) * sizeof(int16_t));
        std::memcpy(p0_buf128_Q_, p0_buf128_Q_ + 128,
                    static_cast<size_t>(64) * sizeof(int16_t));
        p0_chip_count_ = 64;
    }
#endif
}

#ifdef HTS_USE_HOLOGRAPHIC_SYNC

// ─────────────────────────────────────────────────────────────
// phase0_scan_holographic_() — AMI, Phase 4.1 + 4.2 (2 block + CFO clutch)
//
//   Pass 1: CFO=0, 2-block k_w63 scan (Phase 4.1)
//   Pass 2: Pass 1 실패 시 hypothesis grid derotate 후 동일 스캔, 최고 L5 ratio
//   L12: (tx_amp×19)²/2  L3: (tx_amp×19)²×0.3  L5: ratio_x10 ≥ 25
//
//   phase0_scan_ 호환: 성공 시 필요하면 buf CFO 보정 후 psal_commit_align_
// ─────────────────────────────────────────────────────────────
void HTS_V400_Dispatcher::phase0_scan_holographic_() noexcept {
    using namespace ProtectedEngine::Holographic;

    if (p0_chip_count_ < 192) {
#if defined(HTS_DIAG_PRINTF)
        std::printf("[P0-HOLO-AMI] off=-1 e=0 ratio_x10=0 l12=0 l5=0 l3=0 "
                    "pass=0 pass2=0 hyp=0\n");
#endif
        std::memcpy(p0_buf128_I_, p0_buf128_I_ + 128,
                    static_cast<size_t>(64) * sizeof(int16_t));
        std::memcpy(p0_buf128_Q_, p0_buf128_Q_ + 128,
                    static_cast<size_t>(64) * sizeof(int16_t));
        p0_chip_count_ = 64;
        return;
    }

    const int64_t amp32 = static_cast<int64_t>(tx_amp_);
    const int64_t amp19 = amp32 * 19LL;
    const int64_t threshold_12 = (amp19 * amp19) / 2LL;
    const int64_t threshold_3 = (amp19 * amp19 * 3LL) / 10LL;
    constexpr int32_t k_RATIO_MIN_X10 = 25;

    int64_t energies[64];

    for (int off = 0; off < 64; ++off) {
        if (off + 128 > p0_chip_count_) {
            energies[off] = 0;
        } else {
            const int64_t e_block0 = holographic_dot_segmented(
                &p0_buf128_I_[off], &p0_buf128_Q_[off]);
            const int64_t e_block1 = holographic_dot_segmented(
                &p0_buf128_I_[off + 64], &p0_buf128_Q_[off + 64]);
            energies[off] = e_block0 + e_block1;
        }
    }

    int64_t best_e = 0;
    int best_off = -1;
    for (int off = 0; off < 64; ++off) {
        if (energies[off] > best_e) {
            best_e = energies[off];
            best_off = off;
        }
    }

    if (best_off < 0 || best_e <= 0) {
#if defined(HTS_DIAG_PRINTF)
        std::printf("[P0-HOLO-AMI] off=-1 e=0 ratio_x10=0 l12=0 l5=0 l3=0 "
                    "pass=0 pass2=0 hyp=0\n");
#endif
        std::memcpy(p0_buf128_I_, p0_buf128_I_ + 128,
                    static_cast<size_t>(64) * sizeof(int16_t));
        std::memcpy(p0_buf128_Q_, p0_buf128_Q_ + 128,
                    static_cast<size_t>(64) * sizeof(int16_t));
        p0_chip_count_ = 64;
        return;
    }

    const int32_t ratio_x10 = peak_to_median_ratio_x10(energies, 64);
    const bool l5_ok = (ratio_x10 >= k_RATIO_MIN_X10);
    const bool l12_ok = (best_e >= threshold_12);

    bool l3_ok = false;
    if (l12_ok && l5_ok) {
        int32_t t_I[64];
        int32_t t_Q[64];
        for (int i = 0; i < 64; ++i) {
            const int idx = best_off + i;
            t_I[i] = static_cast<int32_t>(p0_buf128_I_[idx]);
            t_Q[i] = static_cast<int32_t>(p0_buf128_Q_[idx]);
        }

        fwht_raw(t_I, 64);
        fwht_raw(t_Q, 64);

        int64_t row_e[64];
        for (int i = 0; i < 64; ++i) {
            row_e[i] = static_cast<int64_t>(t_I[i]) * t_I[i] +
                       static_cast<int64_t>(t_Q[i]) * t_Q[i];
        }

        int64_t top4_sum = 0;
        for (int k = 0; k < 4; ++k) {
            int max_idx = 0;
            int64_t max_val = row_e[0];
            for (int i = 1; i < 64; ++i) {
                if (row_e[i] > max_val) {
                    max_val = row_e[i];
                    max_idx = i;
                }
            }
            top4_sum += max_val;
            row_e[max_idx] = -1;
        }

        l3_ok = (top4_sum >= threshold_3);
    }

    const bool pass_p1 = l12_ok && l5_ok && l3_ok;

    if (pass_p1) {
#if defined(HTS_DIAG_PRINTF)
        std::printf("[P0-HOLO-AMI] off=%d e=%lld ratio_x10=%d "
                    "l12=%d l5=%d l3=%d pass=1 pass2=0 hyp=0\n",
                    best_off, static_cast<long long>(best_e), ratio_x10,
                    static_cast<int>(l12_ok), static_cast<int>(l5_ok),
                    static_cast<int>(l3_ok));
#endif
        psal_off_ = best_off;
        psal_e63_ = static_cast<int32_t>(best_e >> 16);
        psal_commit_align_();
        return;
    }

    // Pass 2: CFO hypothesis grid (AMI, chip_rate 200 kcps — Q8 step 그리드)
    static constexpr int8_t k_cfo_hyp_ami[] = {
        -12, -9, -6, -3, 3, 6, 9, 12};
    constexpr int k_num_hyp_ami =
        static_cast<int>(sizeof(k_cfo_hyp_ami) / sizeof(k_cfo_hyp_ami[0]));

    int16_t rot_I[192];
    int16_t rot_Q[192];
    const int n_rot =
        (p0_chip_count_ > 192) ? 192 : p0_chip_count_;

    int8_t best_hyp = 0;
    int best_off_p2 = -1;
    int64_t best_e_p2 = 0;
    int32_t best_ratio_p2 = 0;

    for (int hi = 0; hi < k_num_hyp_ami; ++hi) {
        const int8_t hyp = k_cfo_hyp_ami[hi];
        derotate_buffer_q8(p0_buf128_I_, p0_buf128_Q_, rot_I, rot_Q, n_rot,
                           hyp);

        int64_t en_h[64];
        for (int off = 0; off < 64; ++off) {
            if (off + 128 > p0_chip_count_) {
                en_h[off] = 0;
            } else {
                const int64_t e0 = holographic_dot_segmented(
                    &rot_I[off], &rot_Q[off]);
                const int64_t e1 = holographic_dot_segmented(
                    &rot_I[off + 64], &rot_Q[off + 64]);
                en_h[off] = e0 + e1;
            }
        }

        int64_t be = 0;
        int bo = -1;
        for (int off = 0; off < 64; ++off) {
            if (en_h[off] > be) {
                be = en_h[off];
                bo = off;
            }
        }

        const int32_t r_h =
            (bo >= 0) ? peak_to_median_ratio_x10(en_h, 64) : 0;
        if (r_h > best_ratio_p2 && bo >= 0 && be >= threshold_12) {
            best_ratio_p2 = r_h;
            best_e_p2 = be;
            best_off_p2 = bo;
            best_hyp = hyp;
        }
    }

    const bool l12_p2 =
        (best_off_p2 >= 0) && (best_e_p2 >= threshold_12);
    const bool l5_p2 = (best_ratio_p2 >= k_RATIO_MIN_X10);

    bool l3_p2 = false;
    if (l12_p2 && l5_p2) {
        derotate_buffer_q8(p0_buf128_I_, p0_buf128_Q_, rot_I, rot_Q, n_rot,
                           best_hyp);

        int32_t t_I[64];
        int32_t t_Q[64];
        for (int i = 0; i < 64; ++i) {
            const int idx = best_off_p2 + i;
            t_I[i] = static_cast<int32_t>(rot_I[idx]);
            t_Q[i] = static_cast<int32_t>(rot_Q[idx]);
        }

        fwht_raw(t_I, 64);
        fwht_raw(t_Q, 64);

        int64_t row_e2[64];
        for (int i = 0; i < 64; ++i) {
            row_e2[i] = static_cast<int64_t>(t_I[i]) * t_I[i] +
                        static_cast<int64_t>(t_Q[i]) * t_Q[i];
        }

        int64_t top4_sum2 = 0;
        for (int k = 0; k < 4; ++k) {
            int max_idx = 0;
            int64_t max_val = row_e2[0];
            for (int i = 1; i < 64; ++i) {
                if (row_e2[i] > max_val) {
                    max_val = row_e2[i];
                    max_idx = i;
                }
            }
            top4_sum2 += max_val;
            row_e2[max_idx] = -1;
        }

        l3_p2 = (top4_sum2 >= threshold_3);
    }

    const bool pass_p2 = l12_p2 && l5_p2 && l3_p2;

#if defined(HTS_DIAG_PRINTF)
    {
        const int off_d = (best_off_p2 >= 0) ? best_off_p2 : best_off;
        const int64_t e_d = (best_off_p2 >= 0) ? best_e_p2 : best_e;
        const int32_t r_d =
            (best_off_p2 >= 0) ? best_ratio_p2 : ratio_x10;
        std::printf("[P0-HOLO-AMI] off=%d e=%lld ratio_x10=%d "
                    "l12=%d l5=%d l3=%d pass=%d pass2=1 hyp=%d\n",
                    off_d, static_cast<long long>(e_d), r_d,
                    static_cast<int>(l12_p2), static_cast<int>(l5_p2),
                    static_cast<int>(l3_p2), static_cast<int>(pass_p2),
                    static_cast<int>(best_hyp));
    }
#endif

    if (pass_p2) {
        derotate_buffer_q8(p0_buf128_I_, p0_buf128_Q_, rot_I, rot_Q, n_rot,
                           best_hyp);
        for (int i = 0; i < n_rot; ++i) {
            p0_buf128_I_[i] = rot_I[i];
            p0_buf128_Q_[i] = rot_Q[i];
        }
        psal_off_ = best_off_p2;
        psal_e63_ = static_cast<int32_t>(best_e_p2 >> 16);
        psal_commit_align_();
    } else {
        std::memcpy(p0_buf128_I_, p0_buf128_I_ + 128,
                    static_cast<size_t>(64) * sizeof(int16_t));
        std::memcpy(p0_buf128_Q_, p0_buf128_Q_ + 128,
                    static_cast<size_t>(64) * sizeof(int16_t));
        p0_chip_count_ = 64;
    }
}

#endif  // HTS_USE_HOLOGRAPHIC_SYNC

void HTS_V400_Dispatcher::Feed_Chip(int16_t rx_I, int16_t rx_Q) noexcept {
#if defined(HTS_AMP_DIAG)
    AmpDiag::record_chip(rx_I, rx_Q);
#endif
#if defined(HTS_DIAG_PRINTF) && !defined(HTS_PHASE0_WALSH_BANK)
    static int s_lab_feed_chip_idx = 0;
    ++s_lab_feed_chip_idx;
    if (s_lab_feed_chip_idx <= 300) {
        std::printf("[HARNESS] chip#%d rx_I=%d rx_Q=%d\n",
                    s_lab_feed_chip_idx, static_cast<int>(rx_I),
                    static_cast<int>(rx_Q));
    }
#endif
#if defined(HTS_DIAG_PRINTF) && defined(HTS_PHASE0_WALSH_BANK)
    {
        static int s_feed_call_count = 0;
        static int s_last_logged_count = -1;
        ++s_feed_call_count;
        if (s_feed_call_count < 500 &&
            (s_feed_call_count - s_last_logged_count) >= 50) {
            s_last_logged_count = s_feed_call_count;
            std::printf(
                "[DIAG-FEED] call#%d phase=%d pre_phase=%d p0_count=%d "
                "carry_pend=%d buf_idx=%d rx_I=%d rx_Q=%d\n",
                s_feed_call_count,
                static_cast<int>(phase_),
                pre_phase_,
                p0_chip_count_,
                p1_carry_pending_,
                buf_idx_,
                static_cast<int>(rx_I), static_cast<int>(rx_Q));
        }
    }
#endif
    int16_t chip_I = rx_I;
    int16_t chip_Q = rx_Q;
    // DC 제거 IIR: α=1/128 (shift 기반, 곱셈 0)
    // dc_est = dc_est - (dc_est >> 7) + (chip >> 7)
    dc_est_I_ = dc_est_I_ - (dc_est_I_ >> 7) +
                (static_cast<int32_t>(chip_I) >> 7);
    dc_est_Q_ = dc_est_Q_ - (dc_est_Q_ >> 7) +
                (static_cast<int32_t>(chip_Q) >> 7);
    chip_I = static_cast<int16_t>(static_cast<int32_t>(chip_I) - dc_est_I_);
    chip_Q = static_cast<int16_t>(static_cast<int32_t>(chip_Q) - dc_est_Q_);
    const int16_t before_cfo_I = chip_I;
    const int16_t before_cfo_Q = chip_Q;
#if defined(HTS_DIAG_PRINTF) && !defined(HTS_PHASE0_WALSH_BANK)
    if (s_lab_feed_chip_idx >= 250 && s_lab_feed_chip_idx <= 450 &&
        (s_lab_feed_chip_idx % 16) == 0) {
        std::printf(
            "[DC-AGC] chip#%d rx_I=%d after_DC=%d dc_est_I=%d dc_est_Q=%d\n",
            s_lab_feed_chip_idx, static_cast<int>(rx_I),
            static_cast<int>(chip_I), static_cast<int>(dc_est_I_),
            static_cast<int>(dc_est_Q_));
    }
#endif
    // CFO 역회전 적용 (Estimate 완료 시 active, 미완료 시 no-op)
    // 순서: DC → CFO → AGC
    cfo_.Apply(chip_I, chip_Q);
#if defined(HTS_HOLO_PREAMBLE) && defined(HTS_DIAG_PRINTF) && \
    defined(HTS_DIAG_CFO_EST)
    {
        static int s_p1_apply_chip_log = 0;
        if (pre_phase_ == 0) {
            s_p1_apply_chip_log = 0;
        }
        if (pre_phase_ == 1 && phase_ == RxPhase::WAIT_SYNC &&
            s_p1_apply_chip_log < 10) {
            const int64_t r0 =
                static_cast<int64_t>(before_cfo_I) * chip_I +
                static_cast<int64_t>(before_cfo_Q) * chip_Q;
            const int64_t n0 = static_cast<int64_t>(before_cfo_I) * before_cfo_I +
                               static_cast<int64_t>(before_cfo_Q) * before_cfo_Q;
            const int64_t n1 = static_cast<int64_t>(chip_I) * chip_I +
                               static_cast<int64_t>(chip_Q) * chip_Q;
            int32_t cos_q14_dot = 16384;
            const int64_t m0 = hts_cfo_int_root_u64(n0);
            const int64_t m1 = hts_cfo_int_root_u64(n1);
            if (m0 > 0 && m1 > 0) {
                const int64_t den = m0 * m1;
                cos_q14_dot = static_cast<int32_t>(
                    (r0 * 16384LL + (den >> 1)) / den);
                if (cos_q14_dot > 16384) {
                    cos_q14_dot = 16384;
                }
                if (cos_q14_dot < -16384) {
                    cos_q14_dot = -16384;
                }
            }
            std::printf(
                "[APPLY-P1] chip=%d raw=(%d,%d) apply=(%d,%d) cfo_on=%d "
                "cos14_dot=%d sin14=%d cos14=%d\n",
                s_p1_apply_chip_log, static_cast<int>(before_cfo_I),
                static_cast<int>(before_cfo_Q), static_cast<int>(chip_I),
                static_cast<int>(chip_Q), cfo_.Is_Apply_Active() ? 1 : 0,
                static_cast<int>(cos_q14_dot),
                static_cast<int>(cfo_.Get_Sin_Per_Chip_Q14()),
                static_cast<int>(cfo_.Get_Cos_Per_Chip_Q14()));
            ++s_p1_apply_chip_log;
        }
    }
#endif
    // 프리앰블 AGC
    pre_agc_.Apply(chip_I, chip_Q);
    apply_holo_lpi_inverse_rx_chip_(chip_I, chip_Q, rx_seq_);
    if (phase_ == RxPhase::RF_SETTLING) {
        (void)chip_I;
        (void)chip_Q;
        const uint32_t nz =
            static_cast<uint32_t>(rf_settle_chips_remaining_ > 0);
        rf_settle_chips_remaining_ -= static_cast<int>(nz);
        if (rf_settle_chips_remaining_ == 0) {
            (void)set_phase_(RxPhase::WAIT_SYNC);
        }
        return;
    }
    if (phase_ == RxPhase::WAIT_SYNC) {
        if (pre_phase_ == 0) {
#if defined(HTS_SYNC_USE_MATCHED_FILTER)
            if (mf_enabled_ && !mf_synced_) {
                if (mf_feed_chip_(chip_I, chip_Q)) {
                    mf_synced_ = true;
                }
            }
#endif
            p0_buf128_I_[p0_chip_count_] = chip_I;
            p0_buf128_Q_[p0_chip_count_] = chip_Q;
            ++p0_chip_count_;
#if defined(HTS_HOLO_PREAMBLE) && HTS_HOLO_CMYK_MODE
            if (p0_chip_count_ < k_p0_holo_rx_collect_chips_)
                return;
#else
            if (p0_chip_count_ < 192)
                return;
#endif
#if defined(HTS_DIAG_PRINTF) && defined(HTS_PHASE0_WALSH_BANK)
            {
                static int s_fullbuf_count = 0;
                ++s_fullbuf_count;
                if (s_fullbuf_count <= 5) {
                    std::printf(
                        "[DIAG-FULLBUF] #%d (192 chip reached, about to scan)\n",
                        s_fullbuf_count);
                    for (int b = 0; b < 3; ++b) {
                        int nz = 0;
                        int mag_max = 0;
                        for (int i = 0; i < 64; ++i) {
                            const int mag =
                                (p0_buf128_I_[b * 64 + i] > 0)
                                    ? static_cast<int>(p0_buf128_I_[b * 64 + i])
                                    : -static_cast<int>(
                                          p0_buf128_I_[b * 64 + i]);
                            if (mag > 100) {
                                ++nz;
                            }
                            if (mag > mag_max) {
                                mag_max = mag;
                            }
                        }
                        std::printf("  blk%d: nonzero=%d/64 mag_max=%d\n", b,
                                    nz, mag_max);
                    }
                }
            }
#endif
            phase0_scan_();
            return;
        }
        // ── Phase 1: carry + Walsh-63/0 dual dot ──
        if (p1_tail_collect_rem_ > 0) {
            p0_carry_I_[p1_tail_idx_] = chip_I;
            p0_carry_Q_[p1_tail_idx_] = chip_Q;
            ++p1_tail_idx_;
            if (p1_tail_idx_ < p1_tail_collect_rem_)
                return;
            p0_carry_count_ = p1_tail_collect_rem_;
            p1_carry_pending_ = 1;
            p1_tail_collect_rem_ = 0;
            p1_tail_idx_ = 0;
            return;
        }
        bool carry_only_full = false;
        bool p1_rx_in_buf = false;
        if (buf_idx_ == 0 && p1_carry_pending_) {
            const int cc = p0_carry_count_;
            for (int j = 0; j < cc; ++j) {
                buf_I_[j] = p0_carry_I_[j];
                buf_Q_[j] = p0_carry_Q_[j];
            }
            buf_idx_ = cc;
            p1_carry_prefix_ = cc;
            p1_carry_pending_ = 0;
            if (buf_idx_ == 64) carry_only_full = true;
        }
        if (!carry_only_full) {
            buf_I_[buf_idx_] = chip_I;
            buf_Q_[buf_idx_] = chip_Q;
            ++buf_idx_;
            p1_rx_in_buf = true;
            if (buf_idx_ < 64) return;
        }
        buf_idx_ = 0;
        std::memcpy(orig_I_, buf_I_, 64 * sizeof(int16_t));
        std::memcpy(orig_Q_, buf_Q_, 64 * sizeof(int16_t));
        // [REMOVED Step1] I/Q 클립 경로 제거 — Gaussian noise 에 무효 확정.
        // [REMOVED Step3] if (ajc_enabled_) ajc_.Process(orig_I_, orig_Q_, 64) — NOP
#if defined(HTS_HOLO_PREAMBLE)
        // Phase 1 PRE_SYM0: 직접 교차상관 (Phase 2B). differential 시도는
        // Sync_PSLTE.cpp HOLO 블록 주석과 동일 사유로 유지하지 않음.
        int32_t dot63_I = 0, dot63_Q = 0;
        int32_t dot0_I = 0, dot0_Q = 0;
        int16_t holo_p1_tpl[64];
        {
            const int16_t amp_tpl =
                (tx_amp_ > 0) ? tx_amp_ : static_cast<int16_t>(1000);
            generate_holo_preamble_(holo_p1_tpl, amp_tpl, holo_lpi_seed_,
                                    rx_seq_);
        }
        for (int j = 0; j < 64; ++j) {
            const int32_t cI = static_cast<int32_t>(orig_I_[j]);
            const int32_t cQ = static_cast<int32_t>(orig_Q_[j]);
            const int32_t sg = (holo_p1_tpl[j] > 0) ? 1 : -1;
            dot63_I += cI * sg;
            dot63_Q += cQ * sg;
            dot0_I += cI;
            dot0_Q += cQ;
        }
        const int64_t e63_64 = static_cast<int64_t>(dot63_I) * dot63_I
                             + static_cast<int64_t>(dot63_Q) * dot63_Q;
        const int64_t e0_64 = static_cast<int64_t>(dot0_I) * dot0_I
                            + static_cast<int64_t>(dot0_Q) * dot0_Q;
#elif defined(HTS_PHASE0_WALSH_BANK)
        // ── Walsh Bank Phase 1: FWHT + dominant_row 매칭 ──
        int32_t dot63_I = 0, dot63_Q = 0;
        int32_t dot0_I  = 0, dot0_Q  = 0;
        alignas(4) int32_t T_I[64];
        alignas(4) int32_t T_Q[64];
        for (int j = 0; j < 64; ++j) {
            T_I[j] = static_cast<int32_t>(orig_I_[j]);
            T_Q[j] = static_cast<int32_t>(orig_Q_[j]);
        }
        fwht_64_complex_inplace_(T_I, T_Q);
        {
            const int dr = static_cast<int>(dominant_row_);
            dot63_I = T_I[dr];
            dot63_Q = T_Q[dr];
        }
        {
            const uint8_t sym1_row_u = static_cast<uint8_t>(
                static_cast<unsigned>(dominant_row_) ^
                static_cast<unsigned>(k_W63_FWHT_ROW));
            const int sym1_row = static_cast<int>(sym1_row_u);
            dot0_I = T_I[sym1_row];
            dot0_Q = T_Q[sym1_row];
#if defined(HTS_DIAG_PRINTF)
            static bool s_p1_map_logged = false;
            if (!s_p1_map_logged) {
                std::printf("[DIAG-P1-MAP] dominant_row=%u k_W63_ROW=%u sym1_row=%u\n",
                            static_cast<unsigned>(dominant_row_),
                            static_cast<unsigned>(k_W63_FWHT_ROW),
                            static_cast<unsigned>(sym1_row_u));
                s_p1_map_logged = true;
            }
#endif
        }
        const int64_t e63_64 = static_cast<int64_t>(dot63_I) * dot63_I
                             + static_cast<int64_t>(dot63_Q) * dot63_Q;
        const int64_t e0_64  = static_cast<int64_t>(dot0_I) * dot0_I
                             + static_cast<int64_t>(dot0_Q) * dot0_Q;
#elif defined(HTS_TARGET_AMI)
        // ── AMI: 32-chip × 2 non-coherent ──
        int32_t dot63a_I = 0, dot63a_Q = 0;
        int32_t dot63b_I = 0, dot63b_Q = 0;
        int32_t dot0a_I  = 0, dot0a_Q  = 0;
        int32_t dot0b_I  = 0, dot0b_Q  = 0;
        for (int j = 0; j < 32; ++j) {
            const int32_t cI = static_cast<int32_t>(orig_I_[j]);
            const int32_t cQ = static_cast<int32_t>(orig_Q_[j]);
            dot0a_I += cI;
            dot0a_Q += cQ;
            if (k_w63[static_cast<std::size_t>(j)] > 0) {
                dot63a_I += cI;
                dot63a_Q += cQ;
            } else {
                dot63a_I -= cI;
                dot63a_Q -= cQ;
            }
        }
        for (int j = 32; j < 64; ++j) {
            const int32_t cI = static_cast<int32_t>(orig_I_[j]);
            const int32_t cQ = static_cast<int32_t>(orig_Q_[j]);
            dot0b_I += cI;
            dot0b_Q += cQ;
            if (k_w63[static_cast<std::size_t>(j)] > 0) {
                dot63b_I += cI;
                dot63b_Q += cQ;
            } else {
                dot63b_I -= cI;
                dot63b_Q -= cQ;
            }
        }
        const int64_t e63_64 =
            static_cast<int64_t>(dot63a_I) * dot63a_I
          + static_cast<int64_t>(dot63a_Q) * dot63a_Q
          + static_cast<int64_t>(dot63b_I) * dot63b_I
          + static_cast<int64_t>(dot63b_Q) * dot63b_Q;
        const int64_t e0_64 =
            static_cast<int64_t>(dot0a_I) * dot0a_I
          + static_cast<int64_t>(dot0a_Q) * dot0a_Q
          + static_cast<int64_t>(dot0b_I) * dot0b_I
          + static_cast<int64_t>(dot0b_Q) * dot0b_Q;
        // est 누적용 단일 dot (두 half 합, CFO phase cancel 영향 있음)
        const int32_t dot63_I = dot63a_I + dot63b_I;
        const int32_t dot63_Q = dot63a_Q + dot63b_Q;
#else
        // ── Stage 2 (PS-LTE): Walsh-domain 매칭 (FWHT + dominant_row) ──
        int32_t dot63_I = 0, dot63_Q = 0;
        int32_t dot0_I  = 0, dot0_Q  = 0;
        alignas(4) int32_t T_I[64];
        alignas(4) int32_t T_Q[64];
        for (int j = 0; j < 64; ++j) {
            T_I[j] = static_cast<int32_t>(orig_I_[j]);
            T_Q[j] = static_cast<int32_t>(orig_Q_[j]);
        }
        fwht_64_complex_inplace_(T_I, T_Q);
        {
            // P0 fingerprint only — 블록마다 argmax 하면 XOR 기하 붕괴(PRE_SYM1 미검출)
            const int dom = static_cast<int>(dominant_row_);
            const int sym1_row =
                dom ^ static_cast<int>(k_W63_FWHT_ROW_NATURAL);
            dot63_I = T_I[dom];
            dot63_Q = T_Q[dom];
            dot0_I  = T_I[sym1_row];
            dot0_Q  = T_Q[sym1_row];
        }
        const int64_t e63_64 = static_cast<int64_t>(dot63_I) * dot63_I
                             + static_cast<int64_t>(dot63_Q) * dot63_Q;
        const int64_t e0_64  = static_cast<int64_t>(dot0_I) * dot0_I
                             + static_cast<int64_t>(dot0_Q) * dot0_Q;
#endif
        const int32_t e63_sh = static_cast<int32_t>(e63_64 >> 16);
        const int32_t e0_sh  = static_cast<int32_t>(e0_64 >> 16);
#if defined(HTS_PHASE0_WALSH_BANK)
        static constexpr int32_t k_P1_MIN_E = 1000;  // FWHT row — PS-LTE 동일 하한
#elif defined(HTS_TARGET_AMI)
        static constexpr int32_t k_P1_MIN_E = 500;   // 32-chip × 2 non-coherent (peak 1/2)
#else
        static constexpr int32_t k_P1_MIN_E = 1000;  // 64-chip coherent (기존)
#endif
        const int32_t max_e_sh = (e63_sh >= e0_sh) ? e63_sh : e0_sh;
#if defined(HTS_DIAG_PRINTF) && !defined(HTS_PHASE0_WALSH_BANK)
        {
            static int s_p1_e_count = 0;
            ++s_p1_e_count;
            if (s_p1_e_count <= 500) {
                std::printf(
                    "[P1-E] call#%d e63_sh=%d e0_sh=%d max_e=%d k_P1_MIN=%d "
                    "pass=%s dom=%u\n",
                    s_p1_e_count, e63_sh, e0_sh, max_e_sh, k_P1_MIN_E,
                    (max_e_sh >= k_P1_MIN_E) ? "YES" : "NO",
                    static_cast<unsigned>(dominant_row_));
            }
        }
#endif
#if defined(HTS_DIAG_PRINTF) && defined(HTS_PHASE0_WALSH_BANK)
        // ─── DIAG-P1: Phase 1 FWHT / silent reset 경계 (로직 무변경) ───
        {
            static int s_p1_dump_count = 0;
            if (s_p1_dump_count < 3) {
                ++s_p1_dump_count;
                const uint8_t sym1_row_u = static_cast<uint8_t>(
                    static_cast<unsigned>(dominant_row_) ^
                    static_cast<unsigned>(k_W63_FWHT_ROW));
                std::printf(
                    "[DIAG-P1-DUMP] dump#%d dom_row=%u k_W63_ROW=%u "
                    "sym1_row=%u e63_sh=%d e0_sh=%d max_e_sh=%d "
                    "k_P1_MIN_E=%d pass=%d\n",
                    s_p1_dump_count,
                    static_cast<unsigned>(dominant_row_),
                    static_cast<unsigned>(k_W63_FWHT_ROW),
                    static_cast<unsigned>(sym1_row_u),
                    e63_sh, e0_sh, max_e_sh, k_P1_MIN_E,
                    static_cast<int>(max_e_sh >= k_P1_MIN_E));
                int64_t e_all[64];
                for (int r = 0; r < 64; ++r) {
                    e_all[r] = static_cast<int64_t>(T_I[r]) * T_I[r] +
                             static_cast<int64_t>(T_Q[r]) * T_Q[r];
                }
                std::printf("[DIAG-P1-TOP5]");
                for (int k = 0; k < 5; ++k) {
                    int best_r = 0;
                    int64_t best_ev = INT64_MIN;
                    for (int r = 0; r < 64; ++r) {
                        if (e_all[r] > best_ev) {
                            best_ev = e_all[r];
                            best_r = r;
                        }
                    }
                    std::printf(" row%d=%lld", best_r,
                                static_cast<long long>(best_ev >> 16));
                    e_all[best_r] = INT64_MIN;
                }
                std::printf("\n");
                std::printf("[DIAG-P1-IN] orig[0..15]_I:");
                for (int i = 0; i < 16; ++i) {
                    std::printf(" %d", static_cast<int>(orig_I_[i]));
                }
                std::printf("\n");
                std::printf("[DIAG-P1-IN] carry_prefix=%d\n",
                            p1_carry_prefix_);
            }
        }
#endif
        if (max_e_sh < k_P1_MIN_E) {
#if defined(HTS_DIAG_PRINTF) && defined(HTS_PHASE0_WALSH_BANK)
            {
                static int s_silent_reset_count = 0;
                ++s_silent_reset_count;
                if ((s_silent_reset_count % 5) == 0) {
                    std::printf(
                        "[DIAG-SILENT-RESET] count=%d (dom_row=%u "
                        "carry_prefix=%d)\n",
                        s_silent_reset_count,
                        static_cast<unsigned>(dominant_row_),
                        p1_carry_prefix_);
                }
            }
#endif
#if defined(HTS_DIAG_PRINTF) && !defined(HTS_PHASE0_WALSH_BANK)
            {
                static int s_silent_count = 0;
                ++s_silent_count;
                if ((s_silent_count % 10) == 0) {
                    std::printf("[SILENT] total=%d last_e=%d\n", s_silent_count,
                                max_e_sh);
                }
            }
#endif
            pre_phase_ = 0;
            // P1 실패 후 재동기: 잘못된 chip_start 기준 CFO 가 다음 P0 에 남지 않도록
            cfo_.Reset();
#if defined(HTS_DIAG_PRINTF) && defined(HTS_DIAG_CFO_EST)
            std::printf(
                "[P1-EXIT] hdr=NO reason=energy_gate e63_sh=%d e0_sh=%d "
                "max_e=%d kmin=%d est_count=%d cfo_ap=%d\n",
                e63_sh, e0_sh, max_e_sh, k_P1_MIN_E, est_count_,
                cfo_.Is_Apply_Active() ? 1 : 0);
#endif
#if defined(HTS_SYNC_USE_MATCHED_FILTER)
            mf_reset_();
#endif
            psal_pending_ = false; psal_off_ = 0; psal_e63_ = 0;
            p0_chip_count_ = 0; p0_carry_count_ = 0;
            p1_carry_pending_ = 0; p1_tail_collect_rem_ = 0;
            p1_tail_idx_ = 0; p1_carry_prefix_ = 0;
            buf_idx_ = 0; wait_sync_head_ = 0; wait_sync_count_ = 0;
            if (carry_only_full && !p1_rx_in_buf) {
                p0_buf128_I_[0] = chip_I; p0_buf128_Q_[0] = chip_Q;
                p0_chip_count_ = 1;
            }
            return;
        }
        uint8_t best_m = 0u;
        if (e63_64 >= e0_64) {
            best_m = PRE_SYM0;
            if (first_c63_ == 0) {
                first_c63_ = e63_sh;
            }
            est_I_ += dot63_I;
            est_Q_ += dot63_Q;
            ++est_count_;
            // B-2: Phase 1 est 갱신 시 mag/recip 동기 갱신
            //   est_count_ 증가 → 매 갱신 시 정규화 상수 재계산
            update_derot_shift_from_est_();
        } else {
            best_m = PRE_SYM1;
        }
        const int8_t sym = static_cast<int8_t>(best_m);
        const int tail_rem = p1_carry_prefix_;
        p1_carry_prefix_ = 0;
#if defined(HTS_DIAG_PRINTF)
        std::printf("[P1-NC] sym=%d e63=%d e0=%d est=(%d,%d) n=%d carry_pend=%d\n",
                    static_cast<int>(sym), e63_sh, e0_sh,
                    est_I_, est_Q_, est_count_, p1_carry_pending_);
        {
            static int s_p1_gate_dump = 0;
            if (s_p1_gate_dump < 50000) {
                ++s_p1_gate_dump;
                const int gate_e_ok =
                    (max_e_sh >= k_P1_MIN_E) ? 1 : 0;
                const int gate_ge_dom = (e63_64 >= e0_64) ? 1 : 0;
                const int gate_sym_pre1 =
                    (sym == static_cast<int8_t>(PRE_SYM1)) ? 1 : 0;
                const int gate_sym_pre0 =
                    (sym == static_cast<int8_t>(PRE_SYM0)) ? 1 : 0;
                const int enter_hdr = gate_sym_pre1;
                std::printf(
                    "[P1-GATE] #%d n=%d e63_sh=%d e0_sh=%d max_e=%d "
                    "k_P1_MIN_E=%d gate_e_ok=%d ge_dom=%d "
                    "sym_pre1=%d sym_pre0=%d carry_pend=%d enter_hdr=%d\n",
                    s_p1_gate_dump, est_count_, e63_sh, e0_sh, max_e_sh,
                    k_P1_MIN_E, gate_e_ok, gate_ge_dom, gate_sym_pre1,
                    gate_sym_pre0, p1_carry_pending_, enter_hdr);
            }
        }
#endif
        if (sym == static_cast<int8_t>(PRE_SYM1)) {
            p1_tail_collect_rem_ = 0; p1_tail_idx_ = 0;
            p1_carry_pending_ = 0; p0_carry_count_ = 0;
            update_derot_shift_from_est_();
            set_phase_(RxPhase::READ_HEADER);
            // [REMOVED Step2] CW EMA 상태 제거됨
            hdr_count_ = 0; hdr_fail_ = 0;
            wait_sync_head_ = 0; wait_sync_count_ = 0;
#if defined(HTS_DIAG_PRINTF)
            std::printf("[P1→HDR] est=(%d,%d) n=%d\n",
                        est_I_, est_Q_, est_count_);
#endif
#if defined(HTS_DIAG_PRINTF) && defined(HTS_DIAG_CFO_EST)
            std::printf(
                "[P1-EXIT] hdr=YES est_count=%d e63_sh=%d e0_sh=%d max_e=%d "
                "k_P1_MIN=%d cfo_ap=%d\n",
                est_count_, e63_sh, e0_sh, max_e_sh, k_P1_MIN_E,
                cfo_.Is_Apply_Active() ? 1 : 0);
#endif
            if (carry_only_full && !p1_rx_in_buf) {
                buf_I_[0] = chip_I; buf_Q_[0] = chip_Q; buf_idx_ = 1;
            } else {
                buf_idx_ = 0;
            }
            return;
        }
        if (sym == static_cast<int8_t>(PRE_SYM0)) {
            if (tail_rem > 0) {
                p1_tail_collect_rem_ = tail_rem; p1_tail_idx_ = 0;
                if (carry_only_full) {
                    p0_carry_I_[0] = chip_I; p0_carry_Q_[0] = chip_Q;
                    p1_tail_idx_ = 1;
                    if (p1_tail_idx_ >= p1_tail_collect_rem_) {
                        p0_carry_count_ = tail_rem; p1_carry_pending_ = 1;
                        p1_tail_collect_rem_ = 0; p1_tail_idx_ = 0;
                    }
                }
            }
            buf_idx_ = 0;
            return;
        }
        pre_phase_ = 0;
        cfo_.Reset();
#if defined(HTS_SYNC_USE_MATCHED_FILTER)
        mf_reset_();
#endif
        psal_pending_ = false; psal_off_ = 0; psal_e63_ = 0;
        p0_chip_count_ = 0; p0_carry_count_ = 0;
        p1_carry_pending_ = 0; p1_tail_collect_rem_ = 0;
        p1_tail_idx_ = 0; p1_carry_prefix_ = 0;
        buf_idx_ = 0; wait_sync_head_ = 0; wait_sync_count_ = 0;
        if (carry_only_full && !p1_rx_in_buf) {
            p0_buf128_I_[0] = chip_I; p0_buf128_Q_[0] = chip_Q;
            p0_chip_count_ = 1;
        }
        return;
    }
    if (buf_idx_ >= 64)
        return;
    buf_I_[buf_idx_] = chip_I;
    buf_Q_[buf_idx_] = chip_Q;
    buf_idx_++;
    if (phase_ == RxPhase::READ_HEADER) {
        if (buf_idx_ == 64) {
            std::memcpy(orig_I_, buf_I_, 64 * sizeof(int16_t));
            std::memcpy(orig_Q_, buf_Q_, 64 * sizeof(int16_t));
            // [REMOVED Step1] I/Q 클립 경로 제거 — Gaussian noise 에 무효 확정.
            static int16_t s_hdr_blk0_I[64], s_hdr_blk0_Q[64];
            if (hdr_count_ == 0) {
                const SymDecResult rh =
                    walsh_dec_full_(orig_I_, orig_Q_, 64, false);
                if (rh.sym >= 0 && rh.sym < 64) {
                    const uint8_t walsh_shift = current_walsh_shift_();
                    const uint8_t corrected_sym = static_cast<uint8_t>(
                        (static_cast<unsigned>(rh.sym) & 63u) ^ walsh_shift);
                    hdr_syms_[hdr_count_] = corrected_sym;
#if defined(HTS_DIAG_PRINTF)
                    std::printf(
                        "[HDR-SHIFT] raw_sym=%d shift=%u corrected=%u dom=%u\n",
                        static_cast<int>(rh.sym),
                        static_cast<unsigned>(walsh_shift),
                        static_cast<unsigned>(corrected_sym),
                        static_cast<unsigned>(dominant_row_));
#endif
                    std::memcpy(s_hdr_blk0_I, orig_I_, 64 * sizeof(int16_t));
                    std::memcpy(s_hdr_blk0_Q, orig_Q_, 64 * sizeof(int16_t));
                    hdr_count_++;
                } else {
                    hdr_fail_++;
                    if (hdr_fail_ >= HDR_FAIL_MAX) {
#if defined(HTS_DIAG_PRINTF)
                        std::printf("[HDR-FAIL] max fail, back to P1\n");
#endif
                        hdr_count_ = 0;
                        hdr_fail_ = 0;
                        buf_idx_ = 0;
                        set_phase_(RxPhase::WAIT_SYNC);
                    }
                }
            } else if (hdr_count_ == 1) {
                // 6-bit 패리티 LUT (popcount&1 대체, ARM 1사이클)
                static constexpr uint8_t k_par6[64] = {
                    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
                    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
                    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
                    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
                };
                static constexpr struct {
                    uint8_t mb;
                    uint16_t plen;
                } k_hdr_defs[] = {
                    {0u, static_cast<uint16_t>(FEC_HARQ::NSYM1)},
                    {0u, static_cast<uint16_t>(FEC_HARQ::NSYM1)},
                    {1u, static_cast<uint16_t>(FEC_HARQ::NSYM16)},
                    {1u, static_cast<uint16_t>(FEC_HARQ::NSYM16)},
                    {2u, static_cast<uint16_t>(FEC_HARQ::NSYM16)},
                    {2u, static_cast<uint16_t>(FEC_HARQ::NSYM16)},
                    {3u, static_cast<uint16_t>(FEC_HARQ::nsym_for_bps(4))},
                    {3u, static_cast<uint16_t>(FEC_HARQ::nsym_for_bps(4))},
                    {3u, static_cast<uint16_t>(FEC_HARQ::nsym_for_bps(5))},
                    {3u, static_cast<uint16_t>(FEC_HARQ::nsym_for_bps(5))},
                    {3u, static_cast<uint16_t>(FEC_HARQ::nsym_for_bps(6))},
                    {3u, static_cast<uint16_t>(FEC_HARQ::nsym_for_bps(6))},
                };
                // ── P0-FIX-002: 위상 독립 템플릿 매칭 ──
                // 기존: v = I[j] + Q[j] projection 후 부호 변조 누적.
                //   채널 회전 θ에서 |corr| ∝ |cosθ − sinθ| + |cosθ + sinθ|
                //   → θ=90°/270° 에서 corr_sum 소멸, S2 전멸의 주 원인.
                // 수정: I 채널 상관 cI, Q 채널 상관 cQ 를 각각 구하고
                //   에너지 e = cI² + cQ² 로 비교. 블록 0, 블록 1 의
                //   에너지 합을 템플릿 점수로 사용 — θ 불변.
                //   CRC 없는 헤더이므로 separation gate 는 에너지 도메인
                //   3 dB (best > second × 2) 로 엄격화.
                int best_idx = -1;
                int64_t best_e = 0;
                int64_t second_e = 0;
                // Stage 6A+: CFO 시 RX chip 열이 k_w_{sym XOR shift} 형태로 보임
                const uint8_t walsh_shift = current_walsh_shift_();
                for (int ti = 0; ti < 12; ++ti) {
                    const uint8_t mb = k_hdr_defs[static_cast<size_t>(ti)].mb;
                    const uint16_t pl = k_hdr_defs[static_cast<size_t>(ti)].plen;
                    const uint16_t iq =
                        (static_cast<unsigned>(ti) & 1u) != 0u
                            ? static_cast<uint16_t>(HDR_IQ_BIT)
                            : 0u;
                    const uint16_t hdr_val =
                        (static_cast<uint16_t>(mb) << 10u) | iq | pl;
                    const uint8_t sym0 =
                        static_cast<uint8_t>((hdr_val >> 6u) & 0x3Fu);
                    const uint8_t sym1 =
                        static_cast<uint8_t>(hdr_val & 0x3Fu);
                    const uint8_t sym0_shifted =
                        static_cast<uint8_t>(sym0 ^ walsh_shift);
                    const uint8_t sym1_shifted =
                        static_cast<uint8_t>(sym1 ^ walsh_shift);
                    // 블록 0 (s_hdr_blk0_I/Q) — I 채널 / Q 채널 각각 상관
                    int64_t cI0 = 0;
                    int64_t cQ0 = 0;
                    for (int j = 0; j < 64; ++j) {
                        const int32_t sm = -static_cast<int32_t>(
                            k_par6[static_cast<uint8_t>(sym0_shifted & j)]);
                        const int32_t vI =
                            static_cast<int32_t>(s_hdr_blk0_I[j]);
                        const int32_t vQ =
                            static_cast<int32_t>(s_hdr_blk0_Q[j]);
                        cI0 += static_cast<int64_t>((vI ^ sm) - sm);
                        cQ0 += static_cast<int64_t>((vQ ^ sm) - sm);
                    }
                    // 블록 1 (orig_I_/orig_Q_)
                    int64_t cI1 = 0;
                    int64_t cQ1 = 0;
                    for (int j = 0; j < 64; ++j) {
                        const int32_t sm = -static_cast<int32_t>(
                            k_par6[static_cast<uint8_t>(sym1_shifted & j)]);
                        const int32_t vI = static_cast<int32_t>(orig_I_[j]);
                        const int32_t vQ = static_cast<int32_t>(orig_Q_[j]);
                        cI1 += static_cast<int64_t>((vI ^ sm) - sm);
                        cQ1 += static_cast<int64_t>((vQ ^ sm) - sm);
                    }
                    // 에너지 도메인 (Q16 스케일 다운 — overflow 방어)
                    const int64_t e0 =
                        ((cI0 * cI0) + (cQ0 * cQ0)) >> 16;
                    const int64_t e1 =
                        ((cI1 * cI1) + (cQ1 * cQ1)) >> 16;
                    const int64_t e = e0 + e1;
                    if (e > best_e) {
                        second_e = best_e;
                        best_e = e;
                        best_idx = ti;
                    } else if (e > second_e) {
                        second_e = e;
                    }
                }
                // Fix B: ratio 2.0x -> 1.5x (best >= second + second/2).
                // V29b: linear floor best_e > first_c63_ (same >>16 scale as
                // e63_sh). Squared floor (first_c63_^2) vs template best_e was
                // mismatched and suppressed [HDR-TMPL] globally (V29 sim).
                const int64_t fc63 =
                    static_cast<int64_t>(first_c63_);
                const int64_t second_64 =
                    static_cast<int64_t>(second_e);
                const int64_t best_64 =
                    static_cast<int64_t>(best_e);
                const int64_t best_ge_15x =
                    best_64 >= (second_64 + (second_64 >> 1));
                const uint32_t floor_ok_u =
                    static_cast<uint32_t>((fc63 <= 0) | (best_64 > fc63));
                const bool tmpl_ok =
                    (best_idx >= 0) &&
                    ((second_e == 0) ||
                     (best_ge_15x && (floor_ok_u != 0u)));
                if (tmpl_ok) {
                    const uint8_t mb =
                        k_hdr_defs[static_cast<size_t>(best_idx)].mb;
                    const uint16_t pl =
                        k_hdr_defs[static_cast<size_t>(best_idx)].plen;
                    const uint16_t iq =
                        (static_cast<unsigned>(best_idx) & 1u) != 0u
                            ? static_cast<uint16_t>(HDR_IQ_BIT)
                            : 0u;
                    const uint16_t hdr_val =
                        (static_cast<uint16_t>(mb) << 10u) | iq | pl;
                    hdr_syms_[0] =
                        static_cast<uint8_t>((hdr_val >> 6u) & 0x3Fu);
                    hdr_syms_[1] = static_cast<uint8_t>(hdr_val & 0x3Fu);
                    hdr_count_ = 2;
#if defined(HTS_DIAG_PRINTF)
                    std::printf(
                        "[HDR-TMPL] matched idx=%d hdr=0x%03X shift=%u dom=%u\n",
                        best_idx, static_cast<unsigned>(hdr_val),
                        static_cast<unsigned>(walsh_shift),
                        static_cast<unsigned>(dominant_row_));
#endif
                } else {
                    SymDecResult rh =
                        walsh_dec_full_(orig_I_, orig_Q_, 64, false);
                    int8_t sym = rh.sym;
                    if (sym >= 0 && sym < 64) {
                        if (rh.second_e > 0u &&
                            rh.best_e < rh.second_e * 2u) {
#if defined(HTS_DIAG_PRINTF)
                            std::printf("[HDR-SOFT] unreliable, reset\n");
#endif
                            hdr_count_ = 0;
                            buf_idx_ = 0;
                            return;
                        }
                        const uint8_t sym_cor = static_cast<uint8_t>(
                            (static_cast<unsigned>(sym) & 63u) ^ walsh_shift);
                        hdr_syms_[hdr_count_] = sym_cor;
#if defined(HTS_DIAG_PRINTF)
                        std::printf(
                            "[HDR-SHIFT] raw_sym=%d shift=%u corrected=%u dom=%u\n",
                            static_cast<int>(sym),
                            static_cast<unsigned>(walsh_shift),
                            static_cast<unsigned>(sym_cor),
                            static_cast<unsigned>(dominant_row_));
#endif
                        hdr_count_++;
                    } else {
                        hdr_fail_++;
                        if (hdr_fail_ >= HDR_FAIL_MAX) {
#if defined(HTS_DIAG_PRINTF)
                            std::printf("[HDR-FAIL] max fail, back to P1\n");
#endif
                            hdr_count_ = 0;
                            hdr_fail_ = 0;
                            buf_idx_ = 0;
                            set_phase_(RxPhase::WAIT_SYNC);
                        }
                    }
                }
            } else {
                SymDecResult rh =
                    walsh_dec_full_(orig_I_, orig_Q_, 64, false);
                int8_t sym = rh.sym;
                if (sym >= 0 && sym < 64) {
                    if (rh.second_e > 0u &&
                        rh.best_e < rh.second_e * 2u) {
#if defined(HTS_DIAG_PRINTF)
                        std::printf("[HDR-SOFT] unreliable e=%u/%u, skip\n",
                                    rh.best_e, rh.second_e);
#endif
                        hdr_count_ = 0;
                        buf_idx_ = 0;
                        return;
                    }
                    const uint8_t walsh_shift = current_walsh_shift_();
                    const uint8_t sym_cor = static_cast<uint8_t>(
                        (static_cast<unsigned>(sym) & 63u) ^ walsh_shift);
                    hdr_syms_[hdr_count_] = sym_cor;
#if defined(HTS_DIAG_PRINTF)
                    std::printf(
                        "[HDR-SHIFT] raw_sym=%d shift=%u corrected=%u dom=%u\n",
                        static_cast<int>(sym),
                        static_cast<unsigned>(walsh_shift),
                        static_cast<unsigned>(sym_cor),
                        static_cast<unsigned>(dominant_row_));
#endif
                    hdr_count_++;
                } else {
                    hdr_fail_++;
                    if (hdr_fail_ >= HDR_FAIL_MAX) {
#if defined(HTS_DIAG_PRINTF)
                        std::printf("[HDR-FAIL] max fail, back to P1\n");
#endif
                        hdr_count_ = 0;
                        hdr_fail_ = 0;
                        buf_idx_ = 0;
                        set_phase_(RxPhase::WAIT_SYNC);
                        return;
                    }
                }
            }
            if (hdr_count_ >= HDR_SYMS) {
                PayloadMode mode;
                int plen = 0;
                if (parse_hdr_(mode, plen) != 0u) {
                    cur_mode_ = mode;
                    pay_cps_ = (mode == PayloadMode::VIDEO_1) ? 1
                               : (mode == PayloadMode::DATA)  ? 64
                                                              : 16;
                    pay_total_ = plen;
                    pay_recv_ = 0;
                    v1_idx_ = 0;
                    sym_idx_ = 0;
                    max_harq_ = FEC_HARQ::DATA_K; // =800 (구 주석 "32"와 불일치 정리)
                    set_phase_(RxPhase::READ_PAYLOAD);
                    buf_idx_ = 0;
                    // [Walsh_Row_Permuter] 헤더 완료 후 payload 직전 — TX Build_Packet
                    // 의 tx_seq_(= rx_seq_ 미증가 시점) 및 첫 RV 라운드(0) 와 정합
                    if (!walsh_permuter_.Is_Initialized()) {
                        (void)walsh_permuter_.Initialize(rx_seq_, 0u);
                    } else {
                        (void)walsh_permuter_.Update_Key(rx_seq_, 0u);
                    }
                    //  try_decode_ 내부에서 Init → Feed 이후라 데이터 파괴
                    //  READ_PAYLOAD 진입 시 첫 라운드만 Init
                    if (!harq_inited_) {
                        if (mode == PayloadMode::VIDEO_16 ||
                            mode == PayloadMode::VOICE) {
                            if (ir_mode_) {
                                SecureMemory::secureWipe(
                                    static_cast<void *>(&g_harq_ccm_union),
                                    sizeof(g_harq_ccm_union));
                                if (ir_state_ != nullptr) {
                                    FEC_HARQ::IR_Init(*ir_state_);
                                }
                                ir_rv_ = 0;
                            } else {
                                FEC_HARQ::Init16(rx_.m16);
                            }
                        } else if (mode == PayloadMode::DATA) {
                            // DATA READ_PAYLOAD: CCM union 전체 한 회 wipe
                            SecureMemory::secureWipe(
                                static_cast<void *>(&g_harq_ccm_union),
                                sizeof(g_harq_ccm_union));
                            if (ir_mode_) {
                                if (ir_state_ != nullptr) {
                                    FEC_HARQ::IR_Init(*ir_state_);
                                }
                                ir_rv_ = 0;
                            } else {
                                SecureMemory::secureWipe(
                                    static_cast<void *>(rx_.m64_I.aI),
                                    sizeof(rx_.m64_I.aI));
                                rx_.m64_I.k = 0;
                                rx_.m64_I.ok = false;
                            }
                        }
                        harq_inited_ = true;
                    }
                    // [REMOVED Step3] pay_cps_ 변경 시 ajc_.Reset — AJC 제거
                } else {
                    // HDR parse 실패: P0 재시작 대신 Phase 1로 복귀
                    // est/derot 유지 — 다음 프리앰블(HARQ 라운드)에서 재시도
#if defined(HTS_DIAG_PRINTF)
                    std::printf("[HDR-RETRY] parse fail, back to P1\n");
#endif
                    hdr_count_ = 0;
                    buf_idx_ = 0;
                    phase_ = RxPhase::WAIT_SYNC;
                    // pre_phase_ 유지 → Phase 1 계속
                }
            }
            if (phase_ == RxPhase::READ_HEADER)
                buf_idx_ = 0;
        }
    } else if (phase_ == RxPhase::READ_PAYLOAD) {
        if (buf_idx_ >= pay_cps_)
            on_sym_();
    }
}
#if defined(HTS_SYNC_USE_MATCHED_FILTER)

namespace {

    struct MF_PRNG_State {
        uint32_t s[4];
    };

    static inline void mf_prng_init_(
        MF_PRNG_State& st, uint32_t seed, uint32_t aux) noexcept {
        st.s[0] = seed ^ 0xDEADBEEFu;
        st.s[1] = seed ^ 0xCAFEBABEu;
        st.s[2] = aux ^ 0x12345678u;
        st.s[3] = aux ^ 0x87654321u;
    }

    static inline uint32_t mf_prng_next_(MF_PRNG_State& st) noexcept {
        const uint32_t result = st.s[0] + st.s[3];
        const uint32_t t = st.s[1] << 9u;
        st.s[2] ^= st.s[0];
        st.s[3] ^= st.s[1];
        st.s[1] ^= st.s[2];
        st.s[0] ^= st.s[3];
        st.s[2] ^= t;
        st.s[3] = (st.s[3] << 11u) | (st.s[3] >> 21u);
        return result;
    }

    static inline int mf_popcount_u32_(uint32_t x) noexcept {
#if defined(_MSC_VER)
        return static_cast<int>(__popcnt(x));
#else
        return __builtin_popcount(static_cast<int>(x));
#endif
    }

    static inline int8_t mf_walsh_bit_(uint32_t row, uint32_t col) noexcept {
        const uint32_t andv = row & col;
        const int pc = mf_popcount_u32_(andv);
        return static_cast<int8_t>(1 - ((pc & 1) << 1));
    }

    static void mf_shuffle_u16_perm_(
        uint16_t* p, uint32_t n, MF_PRNG_State& rng) noexcept {
        for (uint32_t i = 0u; i < n; ++i) {
            p[i] = static_cast<uint16_t>(i);
        }
        for (uint32_t i = n - 1u; i > 0u; --i) {
            const uint32_t r = mf_prng_next_(rng) % (i + 1u);
            const uint16_t tmp = p[i];
            p[i] = p[r];
            p[r] = tmp;
        }
    }

} // namespace

void HTS_V400_Dispatcher::Set_Matched_Filter_Sync(bool enable) noexcept {
    mf_enabled_ = enable;
    if (!enable) {
        mf_ref_ready_ = false;
        mf_reset_();
    }
}

bool HTS_V400_Dispatcher::Get_Matched_Filter_Sync() const noexcept {
    return mf_enabled_;
}

int32_t HTS_V400_Dispatcher::Get_Estimated_Timing_Offset_Q16() const noexcept {
    return mf_estimated_offset_q16_;
}

void HTS_V400_Dispatcher::mf_reset_() noexcept {
    std::memset(mf_recv_I_q16_, 0, sizeof(mf_recv_I_q16_));
    std::memset(mf_recv_Q_q16_, 0, sizeof(mf_recv_Q_q16_));
    mf_recv_count_ = 0u;
    mf_synced_ = false;
    mf_estimated_offset_q16_ = 0;
}

uint32_t HTS_V400_Dispatcher::mf_envelope_combine_(
    int32_t corr_I, int32_t corr_Q) noexcept {
    const int64_t i64 = static_cast<int64_t>(corr_I);
    const int64_t q64 = static_cast<int64_t>(corr_Q);
    const int64_t ai = (i64 < 0) ? -i64 : i64;
    const int64_t aq = (q64 < 0) ? -q64 : q64;
    const uint32_t abs_I = (ai > static_cast<int64_t>(UINT32_MAX))
        ? UINT32_MAX
        : static_cast<uint32_t>(ai);
    const uint32_t abs_Q = (aq > static_cast<int64_t>(UINT32_MAX))
        ? UINT32_MAX
        : static_cast<uint32_t>(aq);
    const int32_t diff =
        static_cast<int32_t>(abs_I) - static_cast<int32_t>(abs_Q);
    const uint32_t mask = static_cast<uint32_t>(diff >> 31);
    const uint32_t mx = (abs_I & ~mask) | (abs_Q & mask);
    const uint32_t mn = (abs_Q & ~mask) | (abs_I & mask);
    return mx + (mn >> 1u);
}

void HTS_V400_Dispatcher::mf_generate_reference_(uint32_t rx_seq) noexcept {
    static constexpr int8_t kPreambleData[16] = {
        1, -1, 1, 1, -1, 1, -1, 1, 1, -1, -1, 1, -1, -1, 1, 1
    };
    static constexpr uint32_t kN = 64u;

    MF_PRNG_State rng{};
    mf_prng_init_(rng, seed_, rx_seq);

    uint16_t rows16[16];
    {
        uint16_t tmp64[64];
        mf_shuffle_u16_perm_(tmp64, kN, rng);
        for (uint32_t k = 0u; k < 16u; ++k) {
            rows16[k] = tmp64[k];
        }
    }
    uint16_t col_perm[64];
    mf_shuffle_u16_perm_(col_perm, kN, rng);
    const uint32_t mlo = mf_prng_next_(rng);
    const uint32_t mhi = mf_prng_next_(rng);
    const uint64_t mask64 =
        (static_cast<uint64_t>(mhi) << 32u) | static_cast<uint64_t>(mlo);

    int32_t chips[64] = {};
    for (uint32_t i = 0u; i < kN; ++i) {
        int32_t sum = 0;
        for (uint32_t k = 0u; k < 16u; ++k) {
            const uint32_t r = static_cast<uint32_t>(rows16[k]);
            const int8_t w = mf_walsh_bit_(r, i);
            sum += static_cast<int32_t>(kPreambleData[k]) *
                   static_cast<int32_t>(w);
        }
        const uint32_t phys_u = static_cast<uint32_t>(col_perm[i]);
        const uint32_t phys =
            (phys_u < kN) ? phys_u : (phys_u % kN);
        const uint64_t bit = (mask64 >> static_cast<int>(i)) & 1ull;
        const int32_t ms = static_cast<int32_t>(1 - static_cast<int32_t>(bit << 1u));
        chips[phys] = sum * ms;
    }

    int32_t max_abs = 0;
    for (uint32_t i = 0u; i < kN; ++i) {
        const int32_t a =
            (chips[i] < 0) ? -chips[i] : chips[i];
        if (a > max_abs) {
            max_abs = a;
        }
    }
    if (max_abs == 0) {
        max_abs = 1;
    }
    for (uint32_t i = 0u; i < kN; ++i) {
        const int32_t scaled =
            (chips[i] * static_cast<int32_t>(0x7FFF)) / max_abs;
        mf_ref_q16_[i] = scaled << 16;
    }

    (void)mf_engine_.Set_Reference_Sequence(mf_ref_q16_, kN);
    mf_ref_rx_seq_ = rx_seq;
    mf_ref_ready_ = true;
}

bool HTS_V400_Dispatcher::mf_feed_chip_(int16_t rx_I, int16_t rx_Q) noexcept {
    if (!mf_ref_ready_ || mf_ref_rx_seq_ != rx_seq_) {
        mf_generate_reference_(rx_seq_);
        if (!mf_ref_ready_) {
            return false;
        }
    }

    if (mf_recv_count_ >= kMF_RecvBufLen) {
        constexpr size_t kShift = 16u;
        for (size_t i = 0u; i < kMF_RecvBufLen - kShift; ++i) {
            mf_recv_I_q16_[i] = mf_recv_I_q16_[i + kShift];
            mf_recv_Q_q16_[i] = mf_recv_Q_q16_[i + kShift];
        }
        mf_recv_count_ = static_cast<uint16_t>(
            static_cast<uint32_t>(mf_recv_count_) - static_cast<uint32_t>(kShift));
    }

    mf_recv_I_q16_[mf_recv_count_] =
        static_cast<int32_t>(rx_I) << 16;
    mf_recv_Q_q16_[mf_recv_count_] =
        static_cast<int32_t>(rx_Q) << 16;
    mf_recv_count_ = static_cast<uint16_t>(
        static_cast<uint32_t>(mf_recv_count_) + 1u);

    if (static_cast<uint32_t>(mf_recv_count_) <
        kMF_RefChips + kMF_OffsetRange) {
        return false;
    }

    const bool ok_I = mf_engine_.Apply_Filter(
        mf_recv_I_q16_, static_cast<size_t>(mf_recv_count_), mf_corr_I_);
    if (!ok_I) {
        return false;
    }
    const bool ok_Q = mf_engine_.Apply_Filter(
        mf_recv_Q_q16_, static_cast<size_t>(mf_recv_count_), mf_corr_Q_);
    if (!ok_Q) {
        return false;
    }

    const size_t num_outputs =
        static_cast<size_t>(mf_recv_count_) - kMF_RefChips + 1u;
    const size_t limit =
        (num_outputs < kMF_NumOffsets) ? num_outputs : kMF_NumOffsets;
    for (size_t k = 0u; k < limit; ++k) {
        mf_envelope_[k] =
            mf_envelope_combine_(mf_corr_I_[k], mf_corr_Q_[k]);
    }
    for (size_t k = limit; k < kMF_NumOffsets; ++k) {
        mf_envelope_[k] = 0u;
    }

    uint32_t best_env = 0u;
    int32_t  best_idx = -1;
    for (uint32_t k = 0u; k < limit; ++k) {
        const uint32_t e = mf_envelope_[k];
        const int32_t diff =
            static_cast<int32_t>(e) - static_cast<int32_t>(best_env);
        const int32_t mask = diff >> 31;
        best_env = (e & static_cast<uint32_t>(~mask)) |
                   (best_env & static_cast<uint32_t>(mask));
        best_idx = (static_cast<int32_t>(k) & ~mask) |
                   (best_idx & mask);
    }

    constexpr uint32_t kMF_PeakThreshold = 0x00400000u;
    if (best_env < kMF_PeakThreshold || best_idx < 0) {
        return false;
    }

    mf_estimated_offset_q16_ = mf_pte_refine_(best_idx);
    return true;
}

int32_t HTS_V400_Dispatcher::mf_pte_refine_(int peak_idx) const noexcept {
    const int32_t int_ofs =
        static_cast<int32_t>(peak_idx) -
        static_cast<int32_t>(kMF_OffsetRange);
    if (peak_idx <= 0 ||
        peak_idx >= static_cast<int>(kMF_NumOffsets) - 1) {
        return int_ofs << 16;
    }

    const int64_t y_m1 =
        static_cast<int64_t>(mf_envelope_[static_cast<size_t>(peak_idx - 1)]);
    const int64_t y_0 =
        static_cast<int64_t>(mf_envelope_[static_cast<size_t>(peak_idx)]);
    const int64_t y_p1 =
        static_cast<int64_t>(mf_envelope_[static_cast<size_t>(peak_idx + 1)]);

    const int64_t denom = 2LL * (y_m1 - 2LL * y_0 + y_p1);
    if (denom == 0LL) {
        return int_ofs << 16;
    }

    const int64_t num = (y_m1 - y_p1) << 16;
    int64_t delta_q16 = num / denom;
    constexpr int64_t kQ16Half = 1LL << 15;
    if (delta_q16 > kQ16Half) {
        delta_q16 = kQ16Half;
    } else if (delta_q16 < -kQ16Half) {
        delta_q16 = -kQ16Half;
    }
    return static_cast<int32_t>(
        (static_cast<int64_t>(int_ofs) << 16) + delta_q16);
}

#endif // HTS_SYNC_USE_MATCHED_FILTER
} // namespace ProtectedEngine
