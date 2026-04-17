// =========================================================================
// HTS_L3_Spectrum_Measured_Test.cpp
// PC 전용: Build_Packet NRZ + Walsh×8 Gauss, 국제표준 스펙트럼 지표(간이)
//
// 빌드: CMake 타깃 HTS_L3_Spectrum_Measured_Test
// =========================================================================
#if defined(__arm__) || defined(__TARGET_ARCH_ARM) || \
    defined(__TARGET_ARCH_THUMB) || defined(__ARM_ARCH)
#error "[HTS_FATAL] HTS_L3_Spectrum_Measured_Test — PC 전용"
#endif

#include "HTS_V400_Dispatcher.hpp"
#include "HTS_Gaussian_Pulse.h"
#include "HTS_Holo_Dispatcher.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

// FFT 파라미터
static constexpr int FFT_N = 1024;
static constexpr double kPi = 3.14159265358979323846;

struct Complex {
    double re, im;
};

static void fft_radix2(Complex *data, int n) {
    int j = 0;
    for (int i = 1; i < n; ++i) {
        int bit = n >> 1;
        for (; j & bit; bit >>= 1) {
            j ^= bit;
        }
        j ^= bit;
        if (i < j) {
            Complex tmp = data[i];
            data[i] = data[j];
            data[j] = tmp;
        }
    }
    for (int len = 2; len <= n; len <<= 1) {
        const double ang = -2.0 * kPi / len;
        const Complex wn = {std::cos(ang), std::sin(ang)};
        for (int i = 0; i < n; i += len) {
            Complex w = {1.0, 0.0};
            for (int k = 0; k < len / 2; ++k) {
                Complex u = data[i + k];
                Complex v = {
                    data[i + k + len / 2].re * w.re - data[i + k + len / 2].im * w.im,
                    data[i + k + len / 2].re * w.im + data[i + k + len / 2].im * w.re};
                data[i + k].re = u.re + v.re;
                data[i + k].im = u.im + v.im;
                data[i + k + len / 2].re = u.re - v.re;
                data[i + k + len / 2].im = u.im - v.im;
                Complex w_new = {
                    w.re * wn.re - w.im * wn.im,
                    w.re * wn.im + w.im * wn.re};
                w = w_new;
            }
        }
    }
}

static void apply_hann_fill_fft(const int16_t *s, int L, Complex *buf, int fft_n) {
    for (int i = 0; i < fft_n; ++i) {
        if (i < L && L > 0) {
            const double w =
                (L <= 1)
                    ? 1.0
                    : (0.5 * (1.0 - std::cos(2.0 * kPi * static_cast<double>(i) /
                                            static_cast<double>(L - 1))));
            buf[i].re = static_cast<double>(s[i]) * w;
        } else {
            buf[i].re = 0.0;
        }
        buf[i].im = 0.0;
    }
}

static void compute_psd_lin_half(const int16_t *s, int L, double *psd_lin_half,
                                 int half_n) {
    Complex buf[FFT_N];
    if (L > FFT_N) {
        L = FFT_N;
    }
    apply_hann_fill_fft(s, L, buf, FFT_N);
    fft_radix2(buf, FFT_N);
    for (int i = 0; i < half_n; ++i) {
        const double m2 = buf[i].re * buf[i].re + buf[i].im * buf[i].im;
        psd_lin_half[i] = (m2 > 0.0) ? m2 : 0.0;
    }
}

static int obw99_bins(const double *psd_lin, int n2) {
    double total = 0.0;
    for (int i = 0; i < n2; ++i) {
        total += psd_lin[i];
    }
    if (total <= 0.0) {
        return 0;
    }
    int imax = 0;
    double mx = 0.0;
    for (int i = 0; i < n2; ++i) {
        if (psd_lin[i] > mx) {
            mx = psd_lin[i];
            imax = i;
        }
    }
    const double target = 0.99 * total;
    int L = imax;
    int R = imax;
    double sum = psd_lin[imax];
    while (sum < target && (L > 0 || R < n2 - 1)) {
        const double lv = (L > 0) ? psd_lin[L - 1] : -1.0;
        const double rv = (R < n2 - 1) ? psd_lin[R + 1] : -1.0;
        if (rv >= lv) {
            ++R;
            sum += psd_lin[R];
        } else {
            --L;
            sum += psd_lin[L];
        }
    }
    return R - L + 1;
}

static double aclr_dbc_worst(const double *psd_lin, int n2, int peak,
                             int main_half_bins) {
    int L = peak - main_half_bins;
    if (L < 0) {
        L = 0;
    }
    int R = peak + main_half_bins;
    if (R >= n2) {
        R = n2 - 1;
    }
    double Pm = 0.0;
    for (int i = L; i <= R; ++i) {
        Pm += psd_lin[i];
    }
    if (Pm <= 1e-30) {
        return -200.0;
    }
    const int w = R - L + 1;
    double Pl = 0.0;
    const int L2 = L - w;
    if (L2 >= 0) {
        for (int i = L2; i < L; ++i) {
            Pl += psd_lin[i];
        }
    }
    double Pr = 0.0;
    const int R2 = R + w;
    if (R2 < n2) {
        for (int i = R + 1; i <= R2; ++i) {
            Pr += psd_lin[i];
        }
    }
    const double worst = (Pl > Pr) ? Pl : Pr;
    if (worst <= 1e-30) {
        return -200.0;
    }
    return 10.0 * std::log10(worst / Pm);
}

// ────────────────────────────────────────────────────────────
// [SEM 수정] 3GPP TS 36.104 간이 모델
//   주채널 = OBW-99% 범위
//   측정: 각 영역의 피크 PSD (dBc relative to main peak)
//   마스크: 0~1BW offset → -12 dBc, 1~2BW → -20 dBc, 2~5BW → -25 dBc
//   margin = mask_limit - measured_peak_dBc (양수=PASS)
// ────────────────────────────────────────────────────────────
struct SEM_Result {
    double margin_01bw;   // 0~1BW 영역 여유 (양수=통과, 측정 불가 시 0)
    double margin_12bw;   // 1~2BW 영역 여유
    double margin_25bw;   // 2~5BW 영역 여유
    bool   valid_01bw;    // FFT/OBW 윈도우와 겹침 — false면 측정 불가
    bool   valid_12bw;
    bool   valid_25bw;
    bool   pass;
};

static bool sem_slice_overlap(int lo, int hi, int n2, int *out_lo,
                              int *out_hi) noexcept {
    if (hi < 0 || lo >= n2) {
        return false;
    }
    *out_lo = (lo < 0) ? 0 : lo;
    *out_hi = (hi >= n2) ? (n2 - 1) : hi;
    return *out_lo <= *out_hi;
}

static bool sem_peak_dbc_in(const double *psd_lin, int n2, double peak_power,
                            int lo, int hi, double *out_dbc) noexcept {
    int a = 0;
    int b = 0;
    if (!sem_slice_overlap(lo, hi, n2, &a, &b)) {
        return false;
    }
    double mx = 0.0;
    for (int i = a; i <= b; ++i) {
        if (psd_lin[i] > mx) {
            mx = psd_lin[i];
        }
    }
    if (mx <= 1e-30) {
        *out_dbc = -120.0;
    } else {
        *out_dbc = 10.0 * std::log10(mx / peak_power);
    }
    return true;
}

static SEM_Result compute_sem_corrected(const double *psd_lin, int n2,
                                         int peak_bin, int obw_bins_half) {
    SEM_Result r{};
    r.valid_01bw = false;
    r.valid_12bw = false;
    r.valid_25bw = false;
    r.pass = false;

    double peak_power = 0.0;
    for (int i = 0; i < n2; ++i) {
        if (psd_lin[i] > peak_power) {
            peak_power = psd_lin[i];
        }
    }
    if (peak_power <= 1e-30) {
        return r;
    }

    const int edge_lo = peak_bin - obw_bins_half;
    const int edge_hi = peak_bin + obw_bins_half;
    const int bw = obw_bins_half * 2;
    if (bw <= 0) {
        return r;
    }

    // 0~1BW (양쪽 중 유효한 쪽만 사용)
    double dbc_01_lo = 0.0;
    double dbc_01_hi = 0.0;
    const bool v01_lo =
        sem_peak_dbc_in(psd_lin, n2, peak_power, edge_lo - bw, edge_lo - 1,
                        &dbc_01_lo);
    const bool v01_hi =
        sem_peak_dbc_in(psd_lin, n2, peak_power, edge_hi + 1, edge_hi + bw,
                        &dbc_01_hi);
    r.valid_01bw = v01_lo || v01_hi;
    double dbc_01 = -120.0;
    if (v01_lo && v01_hi) {
        dbc_01 = (dbc_01_lo > dbc_01_hi) ? dbc_01_lo : dbc_01_hi;
    } else if (v01_lo) {
        dbc_01 = dbc_01_lo;
    } else if (v01_hi) {
        dbc_01 = dbc_01_hi;
    }

    double dbc_12_lo = 0.0;
    double dbc_12_hi = 0.0;
    const bool v12_lo = sem_peak_dbc_in(
        psd_lin, n2, peak_power, edge_lo - 2 * bw, edge_lo - bw - 1, &dbc_12_lo);
    const bool v12_hi = sem_peak_dbc_in(
        psd_lin, n2, peak_power, edge_hi + bw + 1, edge_hi + 2 * bw, &dbc_12_hi);
    r.valid_12bw = v12_lo || v12_hi;
    double dbc_12 = -120.0;
    if (v12_lo && v12_hi) {
        dbc_12 = (dbc_12_lo > dbc_12_hi) ? dbc_12_lo : dbc_12_hi;
    } else if (v12_lo) {
        dbc_12 = dbc_12_lo;
    } else if (v12_hi) {
        dbc_12 = dbc_12_hi;
    }

    double dbc_25_lo = 0.0;
    double dbc_25_hi = 0.0;
    const bool v25_lo = sem_peak_dbc_in(
        psd_lin, n2, peak_power, edge_lo - 5 * bw, edge_lo - 2 * bw - 1,
        &dbc_25_lo);
    const bool v25_hi = sem_peak_dbc_in(
        psd_lin, n2, peak_power, edge_hi + 2 * bw + 1, edge_hi + 5 * bw,
        &dbc_25_hi);
    r.valid_25bw = v25_lo || v25_hi;
    double dbc_25 = -120.0;
    if (v25_lo && v25_hi) {
        dbc_25 = (dbc_25_lo > dbc_25_hi) ? dbc_25_lo : dbc_25_hi;
    } else if (v25_lo) {
        dbc_25 = dbc_25_lo;
    } else if (v25_hi) {
        dbc_25 = dbc_25_hi;
    }

    r.margin_01bw = r.valid_01bw ? (-12.0 - dbc_01) : 0.0;
    r.margin_12bw = r.valid_12bw ? (-20.0 - dbc_12) : 0.0;
    r.margin_25bw = r.valid_25bw ? (-25.0 - dbc_25) : 0.0;

    r.pass = true;
    if (r.valid_01bw) {
        r.pass = r.pass && (r.margin_01bw >= 0.0);
    }
    if (r.valid_12bw) {
        r.pass = r.pass && (r.margin_12bw >= 0.0);
    }
    if (r.valid_25bw) {
        r.pass = r.pass && (r.margin_25bw >= 0.0);
    }
    if (!r.valid_01bw && !r.valid_12bw && !r.valid_25bw) {
        r.pass = false;
    }
    return r;
}

// ────────────────────────────────────────────────────────────
// [EVM 수정] 3GPP TS 36.104 간이 모델
//   1. 필터 지연 보정: 대칭 FIR 중심 = (41-1)/2 = 20 샘플
//   2. LS 게인 추정: gain = Σ(rx×ref) / Σ(rx²)
//   3. EVM = sqrt( Σ(ref - gain×rx)² / Σ(ref²) ) × 100%
// ────────────────────────────────────────────────────────────
static double compute_evm_corrected(
    const int16_t* orig_chips, int n_chips,
    const int16_t* shaped_I, int n_samples)
{
    const int DELAY = 20;   // 41-tap 대칭 FIR 중심 지연
    const int OVS_L = 8;
    const int SKIP = 6;     // 양끝 warm-up 스킵
    
    // Pass 1: LS 게인 추정
    double sum_rx_ref = 0.0, sum_rx_rx = 0.0;
    int count = 0;
    for (int c = SKIP; c < n_chips - SKIP; ++c) {
        const int idx = c * OVS_L + DELAY;
        if (idx < 0 || idx >= n_samples) continue;
        const double ref = static_cast<double>(orig_chips[c]);
        const double rx  = static_cast<double>(shaped_I[idx]);
        sum_rx_ref += rx * ref;
        sum_rx_rx  += rx * rx;
        count++;
    }
    if (sum_rx_rx <= 0.0 || count < 10) return 999.0;
    const double gain = sum_rx_ref / sum_rx_rx;
    
    // Pass 2: EVM 계산
    double err_sum = 0.0, ref_sum = 0.0;
    for (int c = SKIP; c < n_chips - SKIP; ++c) {
        const int idx = c * OVS_L + DELAY;
        if (idx < 0 || idx >= n_samples) continue;
        const double ref = static_cast<double>(orig_chips[c]);
        const double rx  = static_cast<double>(shaped_I[idx]) * gain;
        const double e   = rx - ref;
        err_sum += e * e;
        ref_sum += ref * ref;
    }
    if (ref_sum <= 0.0) return 999.0;
    return 100.0 * std::sqrt(err_sum / ref_sum);
}

static void psd_to_db_norm(const double *psd_lin, int n2, double *psd_db) {
    double mx = 0.0;
    for (int i = 0; i < n2; ++i) {
        if (psd_lin[i] > mx) {
            mx = psd_lin[i];
        }
    }
    if (mx <= 1e-30) {
        for (int i = 0; i < n2; ++i) {
            psd_db[i] = -120.0;
        }
        return;
    }
    for (int i = 0; i < n2; ++i) {
        psd_db[i] = (psd_lin[i] > 1e-30)
            ? 10.0 * std::log10(psd_lin[i] / mx)
            : -120.0;
    }
}

// ════════════════════════════════════════════════════════════
//  L3-M: 국제표준 스펙트럼 측정 (OBW/ACLR/SEM/EVM, PC 간이 모델)
// ════════════════════════════════════════════════════════════
static void test_L3M_spectrum_measured() {
    std::printf("\n═══════════════════════════════════════════════\n");
    std::printf("  L3-M: 국제표준 스펙트럼 측정 (OBW/ACLR/SEM/EVM)\n");
    std::printf("═══════════════════════════════════════════════\n\n");

    ProtectedEngine::HTS_V400_Dispatcher tx_disp;
    const uint32_t lpi_seed[4] = { 0x12345678u, 0xABCDEF01u, 0x98765432u,
                                   0xFEDCBA09u };
    tx_disp.Enable_Holo_LPI(lpi_seed);
    tx_disp.Set_Seed(0x12345678u);

    static const uint8_t info[8] = {
        0xAB, 0xCD, 0xEF, 0x12, 0x34, 0x56, 0x78, 0x9A};
    alignas(16) int16_t oI[4096], oQ[4096];

    const int chips = tx_disp.Build_Packet(
        ProtectedEngine::PayloadMode::VOICE,
        info, static_cast<int>(sizeof(info)),
        static_cast<int16_t>(300),
        oI, oQ, 4096);
    if (chips <= 0) {
        std::printf("  [ERROR] Build_Packet failed (chips=%d)\n", chips);
        return;
    }
    std::printf("  TX 파형: %d chips (NRZ)\n", chips);

    ProtectedEngine::Gaussian_Pulse_Shaper shaper(31, 19661u);
    alignas(16) static int16_t gI[32768], gQ[32768];
    const size_t g_len = shaper.Apply_Pulse_Shaping_Walsh_IQ_x8(
        oI, oQ, static_cast<size_t>(chips), gI, gQ, 32768u);
    if (g_len == 0u) {
        std::printf("  [ERROR] Gaussian shaping failed\n");
        return;
    }
    std::printf("  Gauss 파형: %zu samples (1:8)\n\n", g_len);

    constexpr int n2 = FFT_N / 2;
    double psd_nrz[n2], psd_ga[n2];
    double db_nrz[n2], db_ga[n2];

    const int use_nrz = (chips < FFT_N) ? chips : FFT_N;
    const int use_ga =
        (static_cast<int>(g_len) < FFT_N) ? static_cast<int>(g_len) : FFT_N;

    compute_psd_lin_half(oI, use_nrz, psd_nrz, n2);
    compute_psd_lin_half(gI, use_ga, psd_ga, n2);

    // ── OBW ──
    const int obw_nrz = obw99_bins(psd_nrz, n2);
    const int obw_ga = obw99_bins(psd_ga, n2);
    
    // ── 피크 bin ──
    int peak_nrz = 0; double mxn = 0.0;
    for (int i = 0; i < n2; ++i) { if (psd_nrz[i] > mxn) { mxn = psd_nrz[i]; peak_nrz = i; } }
    int peak_ga = 0; double mxg = 0.0;
    for (int i = 0; i < n2; ++i) { if (psd_ga[i] > mxg) { mxg = psd_ga[i]; peak_ga = i; } }

    // ── ACLR ──
    const int main_half = (obw_nrz > 8) ? (obw_nrz / 4) : 8;
    const double aclr_nrz = aclr_dbc_worst(psd_nrz, n2, peak_nrz, main_half);
    const int main_half_ga = (obw_ga > 8) ? (obw_ga / 4) : 8;
    const double aclr_ga = aclr_dbc_worst(psd_ga, n2, peak_ga, main_half_ga);

    // ── SEM (수정) ──
    const SEM_Result sem_nrz = compute_sem_corrected(psd_nrz, n2, peak_nrz, obw_nrz / 2);
    const SEM_Result sem_ga  = compute_sem_corrected(psd_ga, n2, peak_ga, obw_ga / 2);

    // ── EVM (수정) ──
    const double evm_nrz = 0.0;  // NRZ는 성형 없으므로 0%
    const double evm_ga  = compute_evm_corrected(oI, chips, gI, static_cast<int>(g_len));

    // ── dB 윤곽 ──
    psd_to_db_norm(psd_nrz, n2, db_nrz);
    psd_to_db_norm(psd_ga, n2, db_ga);

    const double obw_ratio =
        (obw_ga > 0) ? (static_cast<double>(obw_nrz) /
                        static_cast<double>(obw_ga))
                     : 0.0;

    // ── 출력 ──
    std::printf("  [1] OBW-99%% (ITU-R SM.328)\n");
    std::printf("      NRZ:   %d bin / Gauss: %d bin → %.2fx 압축\n",
                obw_nrz, obw_ga, obw_ratio);
    
    std::printf("  [2] ACLR (3GPP TS 36.101)\n");
    std::printf("      NRZ:   %+.1f dBc / Gauss: %+.1f dBc\n",
                aclr_nrz, aclr_ga);
    std::printf("      LTE 기준 -45 dBc 대비: NRZ %+.1f dB / Gauss %+.1f dB\n",
                -45.0 - aclr_nrz, -45.0 - aclr_ga);
    
    auto fmt_sem_cell = [](char *buf, std::size_t bufsz, bool valid,
                           double margin) noexcept -> void {
        if (!valid) {
            std::snprintf(buf, bufsz, "N/A (범위외)");
        } else {
            std::snprintf(buf, bufsz, "%+.1f", margin);
        }
    };
    char sn1[40], sn2[40], sn3[40], sg1[40], sg2[40], sg3[40];
    fmt_sem_cell(sn1, sizeof(sn1), sem_nrz.valid_01bw, sem_nrz.margin_01bw);
    fmt_sem_cell(sn2, sizeof(sn2), sem_nrz.valid_12bw, sem_nrz.margin_12bw);
    fmt_sem_cell(sn3, sizeof(sn3), sem_nrz.valid_25bw, sem_nrz.margin_25bw);
    fmt_sem_cell(sg1, sizeof(sg1), sem_ga.valid_01bw, sem_ga.margin_01bw);
    fmt_sem_cell(sg2, sizeof(sg2), sem_ga.valid_12bw, sem_ga.margin_12bw);
    fmt_sem_cell(sg3, sizeof(sg3), sem_ga.valid_25bw, sem_ga.margin_25bw);

    std::printf("  [3] SEM (3GPP TS 36.104)\n");
    std::printf("      NRZ:  0~1BW %s / 1~2BW %s / 2~5BW %s → %s\n",
                sn1, sn2, sn3, sem_nrz.pass ? "PASS" : "FAIL");
    std::printf("      Gauss: 0~1BW %s / 1~2BW %s / 2~5BW %s → %s\n",
                sg1, sg2, sg3, sem_ga.pass ? "PASS" : "FAIL");
    
    std::printf("  [4] EVM (3GPP TS 36.104, LS 게인 보정)\n");
    std::printf("      NRZ: %.2f%% / Gauss: %.2f%%\n", evm_nrz, evm_ga);
    std::printf("      기준: QPSK≤17.5%%, 16QAM≤12.5%%, 64QAM≤8%%\n");
    if (evm_ga <= 8.0)       std::printf("      → 64-QAM PASS\n");
    else if (evm_ga <= 12.5) std::printf("      → 16-QAM PASS\n");
    else if (evm_ga <= 17.5) std::printf("      → QPSK PASS\n");
    else                     std::printf("      → FAIL\n");

    // ── Holo Soft Overlay (V400 + 홀로그램 소프트 스칼라) ──
    std::printf("\n  ── Holo Soft Overlay (V400 BPS + Holo Scalar) ──\n");
    {
        // 1. V400 칩 복사 (원본 oI 보존)
        alignas(16) static int16_t hoI[4096], hoQ[4096];
        std::memcpy(hoI, oI, sizeof(int16_t) * static_cast<size_t>(chips));
        std::memcpy(hoQ, oQ, sizeof(int16_t) * static_cast<size_t>(chips));

        // 2. 홀로그램 소프트 스칼라 생성 (Holo_Dispatcher 사용)
        ProtectedEngine::HTS_Holo_Dispatcher holo_disp;
        const uint32_t hseed[4] = {0x12345678u, 0xABCDEF01u, 0x98765432u,
            0xFEDCBA09u};
        const bool holo_ok = (holo_disp.Initialize(hseed) ==
            ProtectedEngine::HTS_Holo_Dispatcher::SECURE_TRUE);

        if (holo_ok) {
            alignas(16) int16_t scI[4096], scQ[4096];
            const size_t sc_chips = holo_disp.Build_Holo_Packet(
                ProtectedEngine::HoloPayload::DATA_HOLO,
                info, sizeof(info), static_cast<int16_t>(300),
                scI, scQ, 4096);

            if (sc_chips > 0 && chips > 0) {
                // 3. Mix=0.50: 원본 50% + 스칼라 50% (Q13 mixed 게인)
                const size_t apply_len =
                    (static_cast<size_t>(chips) < sc_chips)
                        ? static_cast<size_t>(chips)
                        : sc_chips;
                // max 스칼라 절대값 (I 경로 기준)
                int32_t max_sc = 1;
                for (size_t j = 0; j < apply_len; ++j) {
                    const int32_t a =
                        (scI[j] < 0) ? (-static_cast<int32_t>(scI[j]))
                                     : static_cast<int32_t>(scI[j]);
                    if (a > max_sc) {
                        max_sc = a;
                    }
                }
                for (size_t i = 0; i < static_cast<size_t>(chips); ++i) {
                    const size_t si = i % apply_len;
                    // 정규화: [-1,+1] × 4096 + 4096 = [0, 8192]
                    const int64_t norm64 =
                        (static_cast<int64_t>(scI[si]) * 4096) /
                        static_cast<int64_t>(max_sc);
                    const int32_t norm = static_cast<int32_t>(norm64);
                    const int32_t mixed = 4096 + norm;
                    hoI[i] = static_cast<int16_t>(
                        (static_cast<int32_t>(hoI[i]) * mixed) >> 13);
                    hoQ[i] = static_cast<int16_t>(
                        (static_cast<int32_t>(hoQ[i]) * mixed) >> 13);
                }

                // 4. Kurtosis 측정 (오버레이 후)
                double hm = 0.0, hm2 = 0.0, hm4 = 0.0;
                for (int i = 0; i < chips; ++i) {
                    const double v = static_cast<double>(hoI[i]);
                    hm += v;
                    hm2 += v * v;
                    hm4 += v * v * v * v;
                }
                hm /= static_cast<double>(chips);
                hm2 /= static_cast<double>(chips);
                hm4 /= static_cast<double>(chips);
                double hvar = hm2 - hm * hm;
                double hkurt = (hvar > 0.0) ? (hm4 / (hvar * hvar)) : 0.0;

                // 5. 신호 전력 비교
                double pwr_orig = 0.0, pwr_holo = 0.0;
                for (int i = 0; i < chips; ++i) {
                    pwr_orig += static_cast<double>(oI[i]) * static_cast<double>(oI[i]);
                    pwr_holo += static_cast<double>(hoI[i]) * static_cast<double>(hoI[i]);
                }
                double pwr_diff = 0.0;
                if (pwr_orig > 1e-30) {
                    pwr_diff = 10.0 * std::log10(pwr_holo / pwr_orig);
                }

                std::printf("  V400 칩: %d, Holo 스칼라: %zu\n", chips, sc_chips);
                std::printf("  Overlay Kurtosis: %.3f (BPSK=1.0, 잡음=3.0)\n", hkurt);
                std::printf("  전력 변화: %+.1f dB\n", pwr_diff);

                // 6. Gaussian 적용 후 측정
                alignas(16) static int16_t hgI[32768], hgQ[32768];
                const size_t hgl = shaper.Apply_Pulse_Shaping_Walsh_IQ_x8(
                    hoI, hoQ, static_cast<size_t>(chips), hgI, hgQ, 32768u);
                if (hgl > 0) {
                    hm = 0.0;
                    hm2 = 0.0;
                    hm4 = 0.0;
                    for (size_t i = 0; i < hgl; ++i) {
                        const double v = static_cast<double>(hgI[i]);
                        hm += v;
                        hm2 += v * v;
                        hm4 += v * v * v * v;
                    }
                    hm /= static_cast<double>(hgl);
                    hm2 /= static_cast<double>(hgl);
                    hm4 /= static_cast<double>(hgl);
                    hvar = hm2 - hm * hm;
                    hkurt = (hvar > 0.0) ? (hm4 / (hvar * hvar)) : 0.0;
                    std::printf("  Gauss 후 Kurtosis: %.3f\n", hkurt);
                    std::printf("  Overlay Gauss 샘플: %zu\n", hgl);
                }
            } else {
                std::printf("  Build_Holo_Packet: 0 chips\n");
            }
            (void)holo_disp.Shutdown();
        } else {
            std::printf("  Holo_Dispatcher 초기화 실패\n");
        }
    }

    std::printf("\n  ── PSD 윤곽 (Gauss, 10-bin 간격, dB 정규화) ──\n");
    std::printf("  Bin      PSD(dB)\n");
    for (int i = 0; i < n2; i += 10) {
        std::printf("  %4d    %+7.2f\n", i, db_ga[i]);
    }

    std::printf("\n═══════════════════════════════════════════════\n");
}

int main() {
    std::printf("[BUILD ID] HTS_L3_Spectrum_Measured_Test v2\n");
    test_L3M_spectrum_measured();
    return 0;
}

#include "../../HTS_LIM/HTS_Holo_Dispatcher.cpp"
#include "../../HTS_LIM/HTS_Holo_Tensor_4D.cpp"
#include "../../HTS_LIM/HTS_Gaussian_Pulse.cpp"
