// HTS_BER_PER_UnitTest.cpp — Phase 2: Clopper-Pearson + BER/PER + 이론값 (T6 단일 TU 빌드)
//
// 빌드 (T6와 동일 플래그, HTS_LIM 루트를 include):
//   cd HTS_TEST\t6_sim
//   cl /nologo /O2 /std:c++17 /EHsc /MD /W3 /I"..\..\HTS_LIM" ^
//     /DHTS_ALLOW_HOST_BUILD /DHTS_FEC_SIMULATE_M4_RAM_LAYOUT ^
//     /DHTS_FEC_POLAR_DISABLE /DHTS_DIAG_PRINTF ^
//     /FeHTS_BER_PER_UnitTest.exe HTS_BER_PER_UnitTest.cpp /link /nologo
//
//   또는: build_and_test_phase2.bat (로그·선택 회귀)
//
// Walsh → Session_Gateway::Derive_Session_Material:
//   HTS_T6_SIM_Test.vcxproj 가 HTS_Session_Derive_Stub.cpp 를 링크함.
//   단일 TU 에서도 동일 스텁을 반드시 포함 (LNK2019 방지).
//
#if defined(__arm__) || defined(__TARGET_ARCH_ARM)
#error "[HTS_FATAL] PC 전용"
#endif

#include "HTS_BER_PER_Measure.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace {

int g_failures = 0;

void check_bool(const char* name, bool ok) {
    if (!ok) {
        std::printf("[FAIL] %s\n", name);
        ++g_failures;
    } else {
        std::printf("[ OK ] %s\n", name);
    }
}

void check_near(const char* name, double got, double ref, double tol) {
    const bool ok = std::fabs(got - ref) <= tol;
    if (!ok) {
        std::printf("[FAIL] %s  got=%.12g ref=%.12g tol=%.12g\n", name, got, ref, tol);
        ++g_failures;
    } else {
        std::printf("[ OK ] %s\n", name);
    }
}

void test_clopper_literature() {
    const auto ci = HTS_Phase2::clopper_pearson_ci(107, 85, 0.05);
    check_near("CP Venetoclax n=107 x=85 lower", ci.first, 0.7044, 0.01);
    check_near("CP Venetoclax n=107 x=85 upper", ci.second, 0.8586, 0.01);
}

void test_clopper_edges() {
    const auto z = HTS_Phase2::clopper_pearson_ci(1000, 0, 0.05);
    check_bool("CP x=0 lower==0", z.first == 0.0);
    check_near("CP x=0 upper", z.second, 0.003682, 2e-4);

    const auto a = HTS_Phase2::clopper_pearson_ci(100, 100, 0.05);
    check_near("CP x=n lower", a.first, 0.9637, 2e-3);
    check_bool("CP x=n upper==1", a.second == 1.0);

    const auto mid = HTS_Phase2::clopper_pearson_ci(10000, 10, 0.05);
    check_near("CP n=10000 x=10 lower", mid.first, 4.79e-4, 5e-5);
    check_near("CP n=10000 x=10 upper", mid.second, 1.84e-3, 5e-5);
}

void test_clopper_invalid() {
    const auto i1 = HTS_Phase2::clopper_pearson_ci(0, 0, 0.05);
    check_bool("CP invalid n=0 -> [0,1]", i1.first == 0.0 && i1.second == 1.0);
    const auto i2 = HTS_Phase2::clopper_pearson_ci(10, -1, 0.05);
    check_bool("CP invalid x<0 -> [0,1]", i2.first == 0.0 && i2.second == 1.0);
}

void test_theory_snrs() {
    constexpr double gp = 18.06;
    for (double snr : {-10.0, -5.0, 0.0}) {
        const double th = HTS_Phase2::theoretical_ber_bpsk_awgn(snr, gp);
        HTS_Phase2::ChannelParams p{};
        p.jammer = HTS_Phase2::JammerId::J1;
        p.snr_db = snr;
        constexpr std::uint32_t seed = 0x20260418u;
        constexpr int trials = 200; // 12,800 bits ≥ 10k (Phase 2 UT)
        const auto r = HTS_Phase2::measure_ber_per(p, trials, seed, 1e-3, 1e-2);
        const bool ok = HTS_Phase2::ber_matches_theory(r.ber, th, 1.0);
        char buf[96];
        std::snprintf(buf, sizeof(buf), "Theory J1 SNR=%.0f dB sim vs theory (±1 dB)", snr);
        check_bool(buf, ok);
        std::printf("       BER=%.6g CI[%.6g,%.6g] theory=%.6g bits=%lld err=%lld\n", r.ber,
                    r.ber_ci_lower, r.ber_ci_upper, th,
                    static_cast<long long>(r.total_bits),
                    static_cast<long long>(r.bit_errors));
    }
}

void run_ut_matrix() {
    constexpr std::uint32_t seed = 0x20260418u;
    constexpr int trials = 160; // 10,240 bits

    {
        HTS_Phase2::ChannelParams p{};
        p.jammer = HTS_Phase2::JammerId::J1;
        p.snr_db = -10.0;
        const auto r = HTS_Phase2::measure_ber_per(p, trials, seed, 1e-3, 1e-2);
        const double th = HTS_Phase2::theoretical_ber_bpsk_awgn(-10.0, 18.06);
        check_bool("UT1 J1 -10 dB theory ±1 dB", HTS_Phase2::ber_matches_theory(r.ber, th, 1.0));
        std::printf("       UT1 BER=%.6g sat_rate=%.4f%% timeouts=%d\n", r.ber,
                    r.rx_sat.sat_rate_pct(), static_cast<int>(r.timeout_trials));
    }
    {
        HTS_Phase2::ChannelParams p{};
        p.jammer = HTS_Phase2::JammerId::J1;
        p.snr_db = 0.0;
        const auto r = HTS_Phase2::measure_ber_per(p, trials, seed, 1e-3, 1e-2);
        check_bool("UT2 J1 0 dB ber_ci_upper < 1e-2", r.ber_ci_upper < 1e-2);
        std::printf("       UT2 BER_CI_upper=%.6g\n", r.ber_ci_upper);
    }
    {
        HTS_Phase2::ChannelParams p{};
        p.jammer = HTS_Phase2::JammerId::J1;
        p.snr_db = 10.0;
        const auto r = HTS_Phase2::measure_ber_per(p, trials, seed, 1e-3, 1e-2);
        check_bool("UT3 J1 +10 dB ber_ci_upper < 1e-3 (design target)", r.ber_ci_upper < 1e-3);
        std::printf("       UT3 BER=%.6g CI_upper=%.6g\n", r.ber, r.ber_ci_upper);
    }
    {
        HTS_Phase2::ChannelParams p{};
        p.jammer = HTS_Phase2::JammerId::J2;
        p.jsr_db = 0.0;
        p.f_offset_hz = 50000.0;
        const auto r = HTS_Phase2::measure_ber_per(p, trials, seed, 1e-3, 1e-2);
        check_bool("UT4 J2 CW completed (bits>0)", r.total_bits > 0);
        std::printf("       UT4 PER=%.6g BER=%.6g timeouts=%d\n", r.per, r.ber,
                    static_cast<int>(r.timeout_trials));
    }
    {
        HTS_Phase2::ChannelParams p{};
        p.jammer = HTS_Phase2::JammerId::J4;
        p.jsr_db = 20.0;
        const auto r = HTS_Phase2::measure_ber_per(p, trials, seed, 1e-3, 1e-2);
        check_bool("UT5 J4 barrage completed", r.total_bits > 0);
        std::printf("       UT5 BER=%.6g sat=%.4f%%\n", r.ber, r.rx_sat.sat_rate_pct());
    }
    {
        HTS_Phase2::ChannelParams p{};
        p.jammer = HTS_Phase2::JammerId::J5;
        p.jsr_db = 20.0;
        p.tone_count = 4;
        const auto r = HTS_Phase2::measure_ber_per(p, trials, seed, 1e-3, 1e-2);
        check_bool("UT6 J5 MT completed", r.total_bits > 0);
        std::printf("       UT6 BER=%.6g\n", r.ber);
    }
    {
        HTS_Phase2::ChannelParams p{};
        p.jammer = HTS_Phase2::JammerId::J6;
        p.jsr_db = 20.0;
        p.rate_hz_per_sec = 1e4;
        const auto r = HTS_Phase2::measure_ber_per(p, trials, seed, 1e-3, 1e-2);
        check_bool("UT7 J6 swept completed", r.total_bits > 0);
        std::printf("       UT7 BER=%.6g\n", r.ber);
    }
}

} // namespace

int main() {
    std::printf("=== HTS_BER_PER_UnitTest (Phase 2) ===\n\n");
    std::printf("-- Clopper-Pearson --\n");
    test_clopper_literature();
    test_clopper_edges();
    test_clopper_invalid();
    std::printf("\n-- Theory SNR -10 / -5 / 0 dB (J1) --\n");
    test_theory_snrs();
    std::printf("\n-- UT1..UT7 --\n");
    run_ut_matrix();
    std::printf("\n총 실패: %d\n", g_failures);
    return (g_failures == 0) ? 0 : 1;
}

// ── 소스 링크 (단일 TU 빌드, HTS_T6_SIM_Test.cpp 와 동일) ──
#include "../../HTS_LIM/HTS_Secure_Memory.cpp"
#include "../../HTS_LIM/HTS_Polar_Codec.cpp"
#include "../../HTS_LIM/HTS_FEC_HARQ.cpp"
#include "../../HTS_LIM/HTS_Holo_LPI.cpp"
#include "../../HTS_LIM/HTS_Walsh_Row_Permuter.cpp"
#include "HTS_Session_Derive_Stub.cpp"
#include "../../HTS_LIM/HTS_V400_Dispatcher.cpp"
#include "HTS_BER_PER_Measure.cpp"
#include "HTS_Jammer_STD.cpp"
