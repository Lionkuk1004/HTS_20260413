// HTS_Jammer_STD.hpp — Phase 1 표준 재밍 생성기 (ch_j1 … ch_j6)
// 수학 정의: 프롬프트 47 / SPEC_002 요지. docx 미로드 시에도 동일 수식 유지.
#pragma once

#include <cstdint>
#include <random>

struct SatStats {
    std::int32_t sat_i_count = 0;
    std::int32_t sat_q_count = 0;
    std::int32_t total_samples = 0;

    double sat_rate_pct() const noexcept {
        if (total_samples <= 0) {
            return 0.0;
        }
        return 100.0 * static_cast<double>(sat_i_count + sat_q_count) /
               (2.0 * static_cast<double>(total_samples));
    }
};

/// §3.4: RMS 평균 (I²+Q²)/N — peak / per-I only 금지
double measure_signal_power(const std::int16_t* I, const std::int16_t* Q,
                            int N) noexcept;

/// §13.2: wrap 금지, saturation clip
std::int16_t clip_int16(double x) noexcept;

/// §12.2: 계층 시드 (FNV-1a 유사 mix)
std::uint32_t derive_seed(std::uint32_t base_seed, std::uint32_t trial_idx,
                          const char* channel_name) noexcept;

void ch_j1_awgn(std::int16_t* rI, std::int16_t* rQ, int n_chips, double snr_db,
                double signal_power, std::mt19937& rng,
                SatStats* sat = nullptr) noexcept;

void ch_j2_cw(std::int16_t* rI, std::int16_t* rQ, int n_chips, double jsr_db,
              double signal_power, double f_offset_hz, std::mt19937& rng,
              SatStats* sat = nullptr) noexcept;

/// period_chips = T_period (실수, v1.1 §7.2 fmod) — 정수 반올림 없음
void ch_j3_pulse(std::int16_t* rI, std::int16_t* rQ, int n_chips,
                 double jsr_peak_db, double signal_power, double duty_cycle,
                 double period_chips, double f_offset_hz, int mode,
                 std::mt19937& rng, SatStats* sat = nullptr) noexcept;

void ch_j4_barrage(std::int16_t* rI, std::int16_t* rQ, int n_chips,
                   double jsr_db, double signal_power, std::mt19937& rng,
                   SatStats* sat = nullptr) noexcept;

void ch_j5_multitone(std::int16_t* rI, std::int16_t* rQ, int n_chips,
                     double jsr_db, double signal_power, int tone_count,
                     double bw_system_hz, std::mt19937& rng,
                     SatStats* sat = nullptr) noexcept;

void ch_j6_swept(std::int16_t* rI, std::int16_t* rQ, int n_chips, double jsr_db,
                 double signal_power, double f_start_hz, double f_end_hz,
                 double rate_hz_per_sec, std::mt19937& rng,
                 SatStats* sat = nullptr) noexcept;
