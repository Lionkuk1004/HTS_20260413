// =========================================================================
// HTS_L3_Spectrum_Measured_Test.cpp
// PC 전용: V400 Dispatcher Build_Packet 경로 실측 FFT PSD (NRZ 기준선)
//
// 빌드: CMake 타깃 HTS_L3_Spectrum_Measured_Test
// =========================================================================
#if defined(__arm__) || defined(__TARGET_ARCH_ARM) || \
    defined(__TARGET_ARCH_THUMB) || defined(__ARM_ARCH)
#error "[HTS_FATAL] HTS_L3_Spectrum_Measured_Test — PC 전용"
#endif

#include "HTS_V400_Dispatcher.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

// FFT 파라미터
static constexpr int FFT_N = 1024;     // FFT 크기
static constexpr int CHIP_COUNT = 256; // 측정용 칩 수

// 간단한 Radix-2 Cooley-Tukey FFT (int32 -> double 변환)
// PC 테스트 전용이므로 double 허용 (ARM 양산 코드 아님)

struct Complex {
    double re, im;
};

static void fft_radix2(Complex *data, int n) {
    // Bit-reversal permutation
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

    // Butterfly
    for (int len = 2; len <= n; len <<= 1) {
        const double ang = -2.0 * 3.14159265358979 / len;
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

// ════════════════════════════════════════════════════════════
//  L3-M: 실측 스펙트럼 측정 (Build_Packet 경로)
// ════════════════════════════════════════════════════════════
static void test_L3M_spectrum_measured() {
    std::printf("\n═══════════════════════════════════════════════\n");
    std::printf("  L3-M: 실측 스펙트럼 (Build_Packet TX 경로)\n");
    std::printf("═══════════════════════════════════════════════\n\n");

    // Step 1: Dispatcher로 실제 TX 파형 생성
    ProtectedEngine::HTS_V400_Dispatcher tx_disp;
    tx_disp.Set_Seed(0x12345678u);

    static const uint8_t info[8] = {
        0xAB, 0xCD, 0xEF, 0x12, 0x34, 0x56, 0x78, 0x9A};
    alignas(16) int16_t oI[4096];
    alignas(16) int16_t oQ[4096];

    const int chips = tx_disp.Build_Packet(
        ProtectedEngine::PayloadMode::DATA,
        info, static_cast<int>(sizeof(info)),
        static_cast<int16_t>(300), // 중간 진폭
        oI, oQ, 4096);

    if (chips <= 0 || chips < CHIP_COUNT) {
        std::printf("  [ERROR] Build_Packet failed (chips=%d)\n", chips);
        return;
    }

    std::printf("  TX 파형 생성: %d chips (NRZ 구형파)\n", chips);

    // Step 2: I 채널을 FFT 입력으로 준비
    Complex fft_buf[FFT_N];
    for (int i = 0; i < FFT_N; ++i) {
        if (i < CHIP_COUNT) {
            fft_buf[i].re = static_cast<double>(oI[i]);
            fft_buf[i].im = 0.0;
        } else {
            fft_buf[i].re = 0.0;
            fft_buf[i].im = 0.0;
        }
    }

    // Step 3: FFT 수행
    fft_radix2(fft_buf, FFT_N);

    // Step 4: PSD 계산 (절반만, 실수 신호)
    double psd[FFT_N / 2];
    double max_psd = 0.0;
    int max_bin = 0;
    for (int i = 0; i < FFT_N / 2; ++i) {
        const double mag2 =
            fft_buf[i].re * fft_buf[i].re + fft_buf[i].im * fft_buf[i].im;
        psd[i] = mag2;
        if (mag2 > max_psd) {
            max_psd = mag2;
            max_bin = i;
        }
    }

    // Step 5: dB 변환 (최대값 기준 정규화)
    double psd_db[FFT_N / 2];
    for (int i = 0; i < FFT_N / 2; ++i) {
        if (psd[i] > 0.0) {
            psd_db[i] = 10.0 * std::log10(psd[i] / max_psd);
        } else {
            psd_db[i] = -120.0;
        }
    }

    // Step 6: 메인 로브 식별 (최대 bin ± 20 bins)
    const int main_lobe_half_width = 20;
    const int main_start =
        (max_bin > main_lobe_half_width) ? max_bin - main_lobe_half_width : 0;
    const int main_end = (max_bin + main_lobe_half_width < FFT_N / 2)
                             ? max_bin + main_lobe_half_width
                             : FFT_N / 2 - 1;

    // Step 7: 사이드로브 최대값 (메인 로브 외부)
    double max_sidelobe_db = -120.0;
    int max_sidelobe_bin = 0;
    for (int i = 0; i < FFT_N / 2; ++i) {
        if (i < main_start || i > main_end) {
            if (psd_db[i] > max_sidelobe_db) {
                max_sidelobe_db = psd_db[i];
                max_sidelobe_bin = i;
            }
        }
    }

    // Step 8: 결과 출력
    std::printf("\n  ── PSD 분석 결과 ──\n");
    std::printf("  FFT 크기:         %d\n", FFT_N);
    std::printf("  분석 칩 수:       %d\n", CHIP_COUNT);
    std::printf("  메인 로브 bin:    %d\n", max_bin);
    std::printf("  메인 로브 dB:     0.00 dB (정규화)\n");
    std::printf("  사이드로브 bin:   %d\n", max_sidelobe_bin);
    std::printf("  사이드로브 dB:    %.2f dB\n", max_sidelobe_db);
    std::printf("  사이드로브 억제:  %.2f dB\n", -max_sidelobe_db);

    std::printf("\n  ── 기준선 평가 ──\n");
    std::printf("  NRZ 이론값:       -13.26 dB (sinc²)\n");
    std::printf("  실측값:           %.2f dB\n", max_sidelobe_db);
    std::printf("  Gaussian BT=0.3:  ~-35.00 dB (예상)\n");
    std::printf("  통합 시 이득:     ~%.2f dB 추가 억제 예상\n",
                -35.0 - max_sidelobe_db);

    // Step 9: 스펙트럼 윤곽 샘플 출력 (10 bin 간격)
    std::printf("\n  ── PSD 윤곽 (10-bin 간격) ──\n");
    std::printf("  Bin      PSD(dB)\n");
    for (int i = 0; i < FFT_N / 2; i += 10) {
        std::printf("  %4d    %+7.2f\n", i, psd_db[i]);
    }

    std::printf("\n═══════════════════════════════════════════════\n");
}

int main() {
    test_L3M_spectrum_measured();
    return 0;
}
