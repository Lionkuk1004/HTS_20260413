// HTS_BER_PER_Measure.hpp — Phase 2: BER/PER + Clopper-Pearson (SPEC_002 v1.1 §11)
#pragma once

#include "HTS_Jammer_STD.hpp"

#include <cstdint>
#include <utility>
#include <vector>

namespace HTS_Phase2 {

/// 단일 시험 조건 식별 (표준 잼머 ch_j1…ch_j6)
enum class JammerId : std::uint8_t { J1 = 1, J2, J3, J4, J5, J6 };

/// 재밍·채널 파라미터 (jammer별로 사용 필드만 유효)
struct ChannelParams {
    JammerId jammer = JammerId::J1;
    double signal_power = 10000.0; ///< J2–J6: 잼 전력 기준. J1 은 measure_ber_per 에서 칩 RMS 로 덮어씀(SPEC §6.1).

    // J1 AWGN
    double snr_db = 0.0;

    // J2 CW
    double jsr_db = 0.0;
    double f_offset_hz = 50000.0;

    // J3 Pulse
    double jsr_peak_db = 0.0;
    double duty_cycle = 0.25;
    double period_chips = 100.0;
    double f_off_pulse_hz = 10000.0;
    int pulse_mode = 0;

    // J5 Multi-tone
    int tone_count = 4;
    double bw_system_hz = 200000.0;

    // J6 Swept
    double f_start_hz = -50000.0;
    double f_end_hz = 50000.0;
    double rate_hz_per_sec = 1e4;
};

struct BERPERResult {
    std::int64_t total_bits = 0;
    std::int64_t bit_errors = 0;
    std::int64_t total_packets = 0;
    std::int64_t packet_errors = 0;

    double ber = 0.0;
    double per = 0.0;

    double ber_ci_lower = 0.0;
    double ber_ci_upper = 1.0;
    double per_ci_lower = 0.0;
    double per_ci_upper = 1.0;

    SatStats rx_sat{};
    std::int32_t timeout_trials = 0;
    std::uint32_t base_seed = 0u;

    bool pass_ber = false;
    bool pass_per = false;
};

/// Clopper-Pearson exact 100·(1−α)% CI for Binomial(n,p) with x “성공” 관측
/// BER/PER: error 비율 CI → x = 오류 수, n = 시행 수 (비트 또는 패킷)
std::pair<double, double> clopper_pearson_ci(std::int64_t trials, std::int64_t successes,
                                              double alpha = 0.05) noexcept;

double theoretical_ber_bpsk_awgn(double snr_db, double gp_db = 18.06) noexcept;

/// Uncoded BPSK+AWGN 이론 대비 시뮬 BER: 이론보다 유리하면 통과, 불리하면 +tol_db 이내만 허용
/// (FEC/확산으로 sim<theory 인 경우 양방향 ±tol 강제 시 오탐 방지)
bool ber_matches_theory(double simulated_ber, double theoretical_ber,
                        double tol_db = 1.0) noexcept;

/// V400 DATA 8바이트 페이로드, 잼머 가산 후 복조 — T6 `feed_raw_ext`와 동일 계측
BERPERResult measure_ber_per(const ChannelParams& params, std::int32_t trial_count,
                             std::uint32_t base_seed, double target_ber = 1e-3,
                             double target_per = 1e-2);

/// `measure_ber_per` 와 동일 측정; 트라이얼별 비트 오류 수만 추가 수집 (Phase3 cliff 리포트용)
BERPERResult measure_ber_per_with_per_trial_bit_errors(
    const ChannelParams& params, std::int32_t trial_count, std::uint32_t base_seed,
    double target_ber, double target_per, std::vector<std::uint32_t>& out_bit_errors_per_trial);

} // namespace HTS_Phase2
