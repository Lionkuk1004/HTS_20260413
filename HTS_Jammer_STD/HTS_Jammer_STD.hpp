// =============================================================================
/// @file  HTS_Jammer_STD.hpp
/// @brief SPEC_002 v1.0 꼼수 방어 수학 정의서 준수 재밍 생성기
/// @target PC 시뮬 전용 (HTS_Jammer_STD 프로젝트, Win32/x64 호스트)
///
/// [적용 범위]
///   Phase 6 HARQ Matrix 시험 전용. M4 TU 이식 금지 (double/mt19937 사용).
///
/// [SPEC_002 준수 체크]
///   §3 P_s  : RMS 평균     — Peak power 금지
///   §4 AWGN : σ² = P_s/10^(SNR/10), I/Q 독립
///   §5 JSR  : P_j/P_s 전력비 (10·log₁₀)
///   §6 CW   : φ uniform 독립, A = sqrt(P_j)
///   §7 Pulse: rectangular envelope, Peak JSR 기본
///   §8 Barrage: complex Gaussian, σ_j² = P_j
///   §9 Multi: 1/√N 전력 보존, φ_k iid
///   §10 Swept: linear chirp, sawtooth wrap
///   §13 Saturation: int16 clip + rate 로깅
///
/// [꼼수 방어]
///   - φ 고정 금지 (매 trial 독립 uniform)
///   - Peak power 사용 금지
///   - Wilson/Normal CI 사용 금지 (Clopper-Pearson 만, 별도 파일)
///   - 1/√N 생략 금지
///   - Smooth envelope 금지
// =============================================================================
#pragma once
#include <cstdint>
#include <random>
namespace HTS_Jammer_STD {
// ── §3 Signal power (RMS 평균) ────────────────────────────────
/// @brief P_s = (1/N) · Σ (I²+Q²). Peak power 금지.
/// @return 측정된 신호 전력 (선형, int16² 스케일)
[[nodiscard]] double Measure_Signal_Power(const int16_t *I, const int16_t *Q,
                                          int N) noexcept;
// ── §4 AWGN 잡음 ─────────────────────────────────────────────
/// @brief 신호에 AWGN 추가. σ² = P_s / 10^(SNR/10). I/Q 독립 N(0, σ²/2).
/// @param I,Q   in-place 수정
/// @param N     샘플 수
/// @param snr_db SNR (dB, 양수=쉬움, 음수=어려움)
/// @param P_s   사전 측정된 신호 전력
void Add_AWGN(int16_t *I, int16_t *Q, int N, double snr_db, double P_s,
              std::mt19937 &rng) noexcept;
// ── §6 CW Tone ───────────────────────────────────────────────
/// @brief j[n] = A·exp(j(2π·f_offset·n/f_chip + φ)), A=sqrt(P_j), φ~U[0,2π).
/// @param f_offset_Hz 재밍 중심 주파수 (carrier 기준)
/// @param f_chip_Hz   칩 레이트 (예 200 kHz)
void Add_CW(int16_t *I, int16_t *Q, int N, double jsr_db, double P_s,
            double f_offset_Hz, double f_chip_Hz, std::mt19937 &rng) noexcept;
// ── §7 Pulse ─────────────────────────────────────────────────
/// @brief Rectangular envelope pulse. 본 시험은 Peak JSR (MIL-STD 관례).
/// @param peak_jsr_db ON 구간 JSR (dB)
/// @param duty        0.0 ~ 1.0 (ON 비율)
/// @param T_period_chips 주기 (칩 단위)
void Add_Pulse(int16_t *I, int16_t *Q, int N, double peak_jsr_db, double P_s,
               double duty, int T_period_chips, double f_offset_Hz,
               double f_chip_Hz, std::mt19937 &rng) noexcept;
// ── §8 Barrage ────────────────────────────────────────────────
/// @brief Complex Gaussian jamming. σ_j² = P_j. I/Q 독립 N(0, σ_j²/2).
void Add_Barrage(int16_t *I, int16_t *Q, int N, double jsr_db, double P_s,
                 std::mt19937 &rng) noexcept;
// ── §9 Multi-tone ─────────────────────────────────────────────
/// @brief N_tones 균등 분포, 1/√N 전력 보존, φ_k iid uniform.
/// @param N_tones 톤 개수 (2, 4, 8, 16)
/// @param BW_Hz   시스템 대역폭
void Add_MultiTone(int16_t *I, int16_t *Q, int N, double jsr_db, double P_s,
                   int N_tones, double BW_Hz, double f_chip_Hz,
                   std::mt19937 &rng) noexcept;
// ── §10 Swept (linear chirp, sawtooth) ────────────────────────
/// @brief f(t) = f_start + rate·t, f_end 도달 시 f_start 로 즉시 wrap.
///        Triangular / smooth wrap 금지.
void Add_Swept(int16_t *I, int16_t *Q, int N, double jsr_db, double P_s,
               double f_start_Hz, double f_end_Hz, double rate_Hz_per_s,
               double f_chip_Hz, std::mt19937 &rng) noexcept;
// ── Partial Barrage (심볼 단위 무작위 재밍) ────────────────────
/// @brief N_sym 심볼 중 ratio% 를 무작위 선택하여 Barrage 주입.
/// @param nsym             페이로드 심볼 수
/// @param chips_per_sym    1심볼당 칩 수 (16 또는 64)
/// @param ratio_percent    재밍 심볼 비율 (5~50%)
/// @param jsr_db           JSR (재밍 심볼 구간 한정)
/// @note 심볼 선택: std::uniform_int_distribution, 매 trial 독립.
void Add_Partial_Barrage(int16_t *I, int16_t *Q, int nsym, int chips_per_sym,
                         double ratio_percent, double jsr_db, double P_s,
                         std::mt19937 &rng) noexcept;
// ── §13 Saturation ────────────────────────────────────────────
/// @brief int16 clip 적용, 포화 샘플 수 반환 (saturation_rate 계산용).
/// @return 포화된 샘플 수 (I 또는 Q 중 하나라도 포화 시 카운트)
[[nodiscard]] uint32_t Saturate_Clip(int16_t *I, int16_t *Q, int N) noexcept;
// ── 편의: intensity_unit ──────────────────────────────────────
enum class ChannelType : uint8_t {
    Clean = 0,
    AWGN = 1,           ///< SNR dB (↓어려움)
    Barrage = 2,        ///< JSR dB (↑어려움)
    CW = 3,             ///< JSR dB (↑어려움)
    Pulse = 4,          ///< Peak JSR dB (↑어려움)
    MultiTone = 5,      ///< JSR dB (↑어려움)
    Swept = 6,          ///< JSR dB (↑어려움)
    Partial_Barrage = 7 ///< ratio % (↑어려움)
};
const char *Channel_Name(ChannelType ch) noexcept;
const char *Channel_Unit(ChannelType ch) noexcept;
// ── 기본 시험 파라미터 (SPEC_002 표준 값) ─────────────────────
struct StdParams {
    // CW: f_offset = 50 kHz (bin 16 부근, §6.3 표준 시험 점 중 하나)
    static constexpr double CW_F_OFFSET_HZ = 50000.0;
    // Pulse: duty 0.1, T_period 1000 chips (5 ms @ 200 kHz)
    static constexpr double PULSE_DUTY = 0.1;
    static constexpr int PULSE_T_PERIOD = 1000;
    // Multi-tone: N=8 (§9.3 AMI 예시)
    static constexpr int MULTI_N_TONES = 8;
    // Swept: -100 kHz ~ +100 kHz, rate 10 kHz/s (§10.4)
    static constexpr double SWEPT_F_START_HZ = -100000.0;
    static constexpr double SWEPT_F_END_HZ = 100000.0;
    static constexpr double SWEPT_RATE_HZ_S = 10000.0;
    // 공통: 칩 레이트 200 kHz (AMI 표준)
    static constexpr double F_CHIP_HZ = 200000.0;
    // Partial_Barrage: JSR 고정 30 dB, ratio sweep
    static constexpr double PARTIAL_JSR_DB = 30.0;
};
} // namespace HTS_Jammer_STD
