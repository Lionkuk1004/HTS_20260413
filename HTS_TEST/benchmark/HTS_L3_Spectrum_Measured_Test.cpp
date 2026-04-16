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

static double sem_margin_db(const double *psd_db_norm, int n2, int peak,
                            int off_lo, int off_hi, double mask_db) {
    double peak_db = -300.0;
    for (int i = 0; i < n2; ++i) {
        if (psd_db_norm[i] > peak_db) {
            peak_db = psd_db_norm[i];
        }
    }
    int a = peak + off_lo;
    int b = peak + off_hi;
    if (a < 0) {
        a = 0;
    }
    if (b >= n2) {
        b = n2 - 1;
    }
    if (a > b) {
        return 0.0;
    }
    double mx = -300.0;
    for (int i = a; i <= b; ++i) {
        if (psd_db_norm[i] > mx) {
            mx = psd_db_norm[i];
        }
    }
    return mask_db - mx;
}

static double evm_rms_percent(const int16_t *s, int L) {
    if (L <= 0) {
        return 0.0;
    }
    double sum2 = 0.0;
    for (int i = 0; i < L; ++i) {
        const double v = static_cast<double>(s[i]);
        sum2 += v * v;
    }
    const double rms = std::sqrt(sum2 / static_cast<double>(L));
    if (rms < 1e-9) {
        return 0.0;
    }
    double e2 = 0.0;
    for (int i = 0; i < L; ++i) {
        const double x = static_cast<double>(s[i]) / rms;
        const double ref = (x >= 0.0) ? 1.0 : -1.0;
        const double e = x - ref;
        e2 += e * e;
    }
    return 100.0 * std::sqrt(e2 / static_cast<double>(L));
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
        psd_db[i] = 10.0 * std::log10(psd_lin[i] / mx);
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

    const int obw_nrz = obw99_bins(psd_nrz, n2);
    const int obw_ga = obw99_bins(psd_ga, n2);
    int peak_nrz = 0;
    double mxn = 0.0;
    for (int i = 0; i < n2; ++i) {
        if (psd_nrz[i] > mxn) {
            mxn = psd_nrz[i];
            peak_nrz = i;
        }
    }
    int peak_ga = 0;
    double mxg = 0.0;
    for (int i = 0; i < n2; ++i) {
        if (psd_ga[i] > mxg) {
            mxg = psd_ga[i];
            peak_ga = i;
        }
    }
    const int main_half = (obw_nrz > 8) ? (obw_nrz / 4) : 8;
    const double aclr_nrz = aclr_dbc_worst(psd_nrz, n2, peak_nrz, main_half);
    const double aclr_ga = aclr_dbc_worst(psd_ga, n2, peak_ga, main_half);

    psd_to_db_norm(psd_nrz, n2, db_nrz);
    psd_to_db_norm(psd_ga, n2, db_ga);

    const double sem_nrz_01 =
        sem_margin_db(db_nrz, n2, peak_nrz, 16, 48, -20.0);
    const double sem_ga_01 =
        sem_margin_db(db_ga, n2, peak_ga, 16, 48, -20.0);
    const double sem_nrz_12 =
        sem_margin_db(db_nrz, n2, peak_nrz, 48, 96, -28.0);
    const double sem_ga_12 =
        sem_margin_db(db_ga, n2, peak_ga, 48, 96, -28.0);

    const double evm_nrz = evm_rms_percent(oI, use_nrz);
    const double evm_ga = evm_rms_percent(gI, use_ga);

    const double obw_ratio =
        (obw_ga > 0) ? (static_cast<double>(obw_nrz) /
                        static_cast<double>(obw_ga))
                     : 0.0;

    std::printf("  [1] OBW-99%% (ITU-R SM.328, 간이 FFT bin 폭)\n");
    std::printf("      NRZ:   %d bin / Gauss: %d bin → %.2fx 압축\n",
                obw_nrz, obw_ga, obw_ratio);
    std::printf("  [2] ACLR (인접 대역/메인 대역 전력비, 간이)\n");
    std::printf("      NRZ:   %.1f dBc / Gauss: %.1f dBc\n",
                aclr_nrz, aclr_ga);
    std::printf(
        "      LTE 기준 -45 dBc 여유: NRZ %+1.1f dB / Gauss %+1.1f dB\n",
        -45.0 - aclr_nrz, -45.0 - aclr_ga);
    std::printf("  [3] SEM (정규화 PSD vs 간이 마스크, bin 오프셋)\n");
    std::printf(
        "      NRZ 0~1BW: %+1.1f dB / 1~2BW: %+1.1f dB (여유, 클수록 좋음)\n",
        sem_nrz_01, sem_nrz_12);
    std::printf(
        "      Gauss 0~1BW: %+1.1f dB / 1~2BW: %+1.1f dB\n",
        sem_ga_01, sem_ga_12);
    std::printf("  [4] EVM (부호 기준 단순 RMS %%)\n");
    std::printf("      NRZ: %.2f %% RMS / Gauss: %.2f %% RMS\n",
                evm_nrz, evm_ga);

    std::printf("\n  ── PSD 윤곽 (Gauss, 10-bin 간격, dB 정규화) ──\n");
    std::printf("  Bin      PSD(dB)\n");
    for (int i = 0; i < n2; i += 10) {
        std::printf("  %4d    %+7.2f\n", i, db_ga[i]);
    }

    std::printf("\n═══════════════════════════════════════════════\n");
}

int main() {
    std::printf("[BUILD ID] HTS_L3_Spectrum_Measured_Test v1\n");
    test_L3M_spectrum_measured();
    return 0;
}

#include "../../HTS_LIM/HTS_Gaussian_Pulse.cpp"
