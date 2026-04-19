// HTS_Phase3_MIL.hpp — PROMPT 49 v3.0: MIL-STD 기반 한계 탐지 (한계 = 목표, 통과 아님)
// 빌드: HTS_Jammer_STD_UnitTest.cpp 단일 TU + /DHTS_ENABLE_PHASE3_MIL [/DHTS_PHASE3_FULL_MATRIX]
#pragma once

#if defined(HTS_ENABLE_PHASE3_MIL)

#include "HTS_BER_PER_Measure.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

#if defined(HTS_PHASE3_S1_CLIFF)
#include "HTS_Phase3_MIL_Report.hpp"
#endif

namespace Phase3_MIL {

// measure_ber_per: 64 payload bits / trial (Phase 2 고정)
inline constexpr int kBitsPerTrial = 64;
/// PROMPT 49 §0.2: 포인트당 ≥ 100,000 bit
inline constexpr int kTrialsPerPoint =
    (100000 + kBitsPerTrial - 1) / kBitsPerTrial; // 1563 → 100032 bits

struct MilScenario {
    const char* id = "";
    const char* name = "";
    const char* jammer = ""; ///< "J1" … "J6"
    double param_start = 0.0;
    double param_end = 0.0;
    double param_step = 1.0;
    int trials_per_point = kTrialsPerPoint;
    double mil_required_jsr_db = 0.0; ///< AWGN 등 비해당 시 0
    double mil_required_ber = 1e-3;
    const char* mil_std_ref = "";
    std::uint32_t seed_base = 0u;
    /// J3 보조 스윕 (duty 고정 시 JSR만 스윕)
    double j3_duty = 0.10;
};

struct PointResult {
    double param_value = 0.0;
    std::uint64_t total_bits = 0u;
    std::uint64_t errors = 0u;
    double ber = 0.0;
    double ci_lower = 0.0;
    double ci_upper = 1.0;
    double per = 0.0;
    std::uint32_t decode_fail_count = 0u;
    double link_availability = 0.0;
    double mil_margin_db = 0.0; ///< analyze_limits 후 채움
};

struct LimitPoints {
    double param_at_ber_1e2 = std::numeric_limits<double>::quiet_NaN();
    double param_at_ber_1e3 = std::numeric_limits<double>::quiet_NaN();
    double param_at_ber_1e5 = std::numeric_limits<double>::quiet_NaN();
    double param_at_ber_1e6 = std::numeric_limits<double>::quiet_NaN();
    double param_at_decode_fail = std::numeric_limits<double>::quiet_NaN();
    double anti_jam_margin_db = std::numeric_limits<double>::quiet_NaN();
    const char* threat_category = "III"; ///< 보고용 기본값 (MIL-HDBK-238B 참조 문구)
    const char* scenario_id = "";
};

inline std::uint32_t mix_seed(std::uint32_t base, const char* tag, double pv) noexcept {
    std::uint32_t h = base ^ static_cast<std::uint32_t>(static_cast<int>(pv * 1000.0));
    if (tag != nullptr) {
        for (const char* p = tag; *p != '\0'; ++p) {
            h = (h * 16777619u) ^
                static_cast<std::uint32_t>(static_cast<unsigned char>(*p));
        }
    }
    return h;
}

inline void fill_channel(double pv, const char* jam, HTS_Phase2::ChannelParams& cp) noexcept {
    using HTS_Phase2::JammerId;
    if (std::strcmp(jam, "J1_AWGN") == 0) {
        cp.jammer = JammerId::J1;
        cp.snr_db = pv;
        return;
    }
    if (std::strcmp(jam, "J1") == 0) {
        cp.jammer = JammerId::J1;
        cp.snr_db = pv;
        return;
    }
    if (std::strcmp(jam, "J2") == 0) {
        cp.jammer = JammerId::J2;
        cp.jsr_db = pv;
        cp.f_offset_hz = 50000.0;
        return;
    }
    if (std::strcmp(jam, "J3") == 0) {
        cp.jammer = JammerId::J3;
        cp.jsr_peak_db = pv;
        cp.duty_cycle = 0.10;
        cp.period_chips = 100.0;
        cp.f_off_pulse_hz = 10000.0;
        cp.pulse_mode = 0;
        return;
    }
    if (std::strcmp(jam, "J4") == 0) {
        cp.jammer = JammerId::J4;
        cp.jsr_db = pv;
        return;
    }
    if (std::strcmp(jam, "J5") == 0) {
        cp.jammer = JammerId::J5;
        cp.jsr_db = pv;
        cp.tone_count = 4;
        cp.bw_system_hz = 200000.0;
        return;
    }
    if (std::strcmp(jam, "J6") == 0) {
        cp.jammer = JammerId::J6;
        cp.jsr_db = pv;
        cp.rate_hz_per_sec = 10000.0;
        return;
    }
    cp.jammer = JammerId::J1;
    cp.snr_db = pv;
}

#if defined(HTS_PHASE3_S1_CLIFF)
inline MilScenario make_s1_cliff_scenario() {
    MilScenario s{};
    s.id = "S1_CLIFF";
    s.name = "J1 AWGN cliff precision";
    s.jammer = "J1_AWGN";
    s.param_start = -3.0;
    s.param_end = -12.0;
    s.param_step = -1.0;
    s.trials_per_point = 1000;
    s.mil_required_jsr_db = 0.0;
    s.mil_required_ber = 1e-3;
    s.mil_std_ref = "MIL-STD-188-110D";
    s.seed_base = 0xC11FF001u;
    return s;
}

inline void run_scenario_with_stats(const MilScenario& sc, std::vector<PointStats>& out) {
    out.clear();
    const double eps = 1e-9;
    const double lo = std::min(sc.param_start, sc.param_end);
    const double hi = std::max(sc.param_start, sc.param_end);
    double pv = sc.param_start;
    for (int guard = 0; guard < 100000; ++guard) {
        if (pv < lo - eps || pv > hi + eps) {
            break;
        }
        HTS_Phase2::ChannelParams cp{};
        fill_channel(pv, sc.jammer, cp);
        if (std::strcmp(sc.jammer, "J3") == 0) {
            cp.duty_cycle = sc.j3_duty;
        }
        const std::uint32_t seed =
            mix_seed(sc.seed_base, sc.id != nullptr ? sc.id : "", pv);
        std::vector<std::uint32_t> trial_errs;
        const HTS_Phase2::BERPERResult r = HTS_Phase2::measure_ber_per_with_per_trial_bit_errors(
            cp, sc.trials_per_point, seed, sc.mil_required_ber, 1e-3, trial_errs);
        PointStats ps{};
        ps.param_value = pv;
        ps.total_bits = static_cast<std::uint64_t>(r.total_bits);
        ps.errors = static_cast<std::uint64_t>(r.bit_errors);
        ps.ber = r.ber;
        ps.ci_lo = r.ber_ci_lower;
        ps.ci_hi = r.ber_ci_upper;
        ps.trials = static_cast<std::uint32_t>(r.total_packets);
        ps.decode_fail = static_cast<std::uint32_t>(r.timeout_trials);
        if (r.total_packets > 0) {
            ps.decode_ok =
                static_cast<std::uint32_t>(r.total_packets - r.timeout_trials);
            ps.link_avail =
                1.0 - static_cast<double>(r.packet_errors) / static_cast<double>(r.total_packets);
        }
        ps.per = r.per;
        ps.sat_rate = r.rx_sat.sat_rate_pct();
        if (!trial_errs.empty()) {
            const auto mm = std::minmax_element(trial_errs.begin(), trial_errs.end());
            ps.bit_err_min = *mm.first;
            ps.bit_err_max = *mm.second;
            double sum = 0.0;
            double sum2 = 0.0;
            for (const auto e : trial_errs) {
                const double de = static_cast<double>(e);
                sum += de;
                sum2 += de * de;
            }
            const double n = static_cast<double>(trial_errs.size());
            ps.bit_err_mean = sum / n;
            const double var = sum2 / n - ps.bit_err_mean * ps.bit_err_mean;
            ps.bit_err_std = std::sqrt(std::max(0.0, var));
        }
        ps.flag_all_zero = (ps.errors == 0u);
        ps.flag_all_fail = (ps.trials > 0u && ps.decode_fail == ps.trials);
        ps.flag_bimodal = (ps.bit_err_min == 0u && ps.bit_err_max == 64u);
        ps.theory_ber = HTS_Phase2::theoretical_ber_bpsk_awgn(pv, 18.06);
        if (ps.theory_ber > 0.0 && ps.ber > 0.0) {
            ps.deviation_db = 10.0 * std::log10(ps.ber / ps.theory_ber);
        } else {
            ps.deviation_db = 0.0;
        }
        ps.flag_suspicious = (std::fabs(ps.deviation_db) > 10.0);
        out.push_back(ps);
        if (std::fabs(pv - sc.param_end) <= std::fabs(sc.param_step) * 0.5 + eps) {
            break;
        }
        pv += sc.param_step;
    }
}
#endif

inline void run_scenario(const MilScenario& sc, std::vector<PointResult>& out) {
    out.clear();
    const double eps = 1e-9;
    const double lo = std::min(sc.param_start, sc.param_end);
    const double hi = std::max(sc.param_start, sc.param_end);
    double pv = sc.param_start;
    for (int guard = 0; guard < 100000; ++guard) {
        if (pv < lo - eps || pv > hi + eps) {
            break;
        }
        HTS_Phase2::ChannelParams cp{};
        fill_channel(pv, sc.jammer, cp);
        if (std::strcmp(sc.jammer, "J3") == 0) {
            cp.duty_cycle = sc.j3_duty;
        }
        const std::uint32_t seed =
            mix_seed(sc.seed_base, sc.id != nullptr ? sc.id : "", pv);
        const HTS_Phase2::BERPERResult r = HTS_Phase2::measure_ber_per(
            cp, sc.trials_per_point, seed, sc.mil_required_ber, 1e-3);
        PointResult pr{};
        pr.param_value = pv;
        pr.total_bits = static_cast<std::uint64_t>(r.total_bits);
        pr.errors = static_cast<std::uint64_t>(r.bit_errors);
        pr.ber = r.ber;
        pr.ci_lower = r.ber_ci_lower;
        pr.ci_upper = r.ber_ci_upper;
        pr.per = r.per;
        pr.decode_fail_count = static_cast<std::uint32_t>(r.timeout_trials);
        if (r.total_packets > 0) {
            pr.link_availability =
                1.0 - static_cast<double>(r.packet_errors) / static_cast<double>(r.total_packets);
        }
        out.push_back(pr);
        if (std::fabs(pv - sc.param_end) <= std::fabs(sc.param_step) * 0.5 + eps) {
            break;
        }
        pv += sc.param_step;
    }
}

/// 단조 스윕 가정: ber 가 스윕 방향으로 단조 증가(또는 감소)한다고 보고 임계 교차 보간
inline double interp_cross(const std::vector<PointResult>& v, double thr, bool ber_rises_with_index) {
    if (v.size() < 2u) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    for (std::size_t i = 1; i < v.size(); ++i) {
        const double b0 = v[i - 1].ber;
        const double b1 = v[i].ber;
        const double p0 = v[i - 1].param_value;
        const double p1 = v[i].param_value;
        const bool cross =
            ber_rises_with_index ? (b0 < thr && b1 >= thr) : (b0 > thr && b1 <= thr);
        if (!cross) {
            continue;
        }
        const double db = b1 - b0;
        if (std::fabs(db) < 1e-30) {
            return 0.5 * (p0 + p1);
        }
        const double t = (thr - b0) / db;
        return p0 + t * (p1 - p0);
    }
    return std::numeric_limits<double>::quiet_NaN();
}

/// SNR 스윕(+20→-20): 인덱스 증가 시 SNR 감소 → 보통 BER 증가 → ber_rises_with_index = true
/// JSR 스윕(0→40): 인덱스 증가 시 BER 보통 증가
inline LimitPoints analyze_limits(const std::vector<PointResult>& results,
                                    const MilScenario& sc) {
    LimitPoints L{};
    L.scenario_id = sc.id != nullptr ? sc.id : "";
    const bool ber_rises = true;
    L.param_at_ber_1e2 = interp_cross(results, 1e-2, ber_rises);
    L.param_at_ber_1e3 = interp_cross(results, 1e-3, ber_rises);
    L.param_at_ber_1e5 = interp_cross(results, 1e-5, ber_rises);
    L.param_at_ber_1e6 = interp_cross(results, 1e-6, ber_rises);
    for (const auto& p : results) {
        const double trial_approx =
            (p.total_bits > 0u) ? static_cast<double>(p.total_bits / 64u) : 0.0;
        if (trial_approx > 0.0 &&
            static_cast<double>(p.decode_fail_count) > 0.25 * trial_approx) {
            L.param_at_decode_fail = p.param_value;
            break;
        }
    }
    if (sc.mil_required_jsr_db > 1e-6 && std::strcmp(sc.jammer, "J1") != 0) {
        if (!std::isnan(L.param_at_ber_1e3)) {
            L.anti_jam_margin_db = L.param_at_ber_1e3 - sc.mil_required_jsr_db;
        }
    } else if (std::strcmp(sc.jammer, "J1") == 0 || std::strcmp(sc.jammer, "J1_AWGN") == 0) {
        /// MIL-110D voice 1e-3 기준: SNR 한계가 높을수록(덜 음수) 여유
        if (!std::isnan(L.param_at_ber_1e3)) {
            L.anti_jam_margin_db = L.param_at_ber_1e3 - (-10.0); ///< 보고용 앵커(문헌형)
        }
    }
    return L;
}

inline void fmt_nan(FILE* fp, double x) {
    if (std::isnan(x)) {
        std::fprintf(fp, "N/A");
    } else {
        std::fprintf(fp, "%.6g", x);
    }
}

inline void export_csv(FILE* fp, const char* scenario_id, const std::vector<PointResult>& results) {
    if (fp == nullptr) {
        return;
    }
    for (const auto& p : results) {
        std::fprintf(fp, "%s,%.6g,%llu,%llu,%.8g,%.8g,%.8g,%.8g,%u,%.8g,%.6g\n",
                      scenario_id != nullptr ? scenario_id : "", p.param_value,
                      static_cast<unsigned long long>(p.total_bits),
                      static_cast<unsigned long long>(p.errors), p.ber, p.ci_lower, p.ci_upper,
                      p.per, static_cast<unsigned>(p.decode_fail_count), p.link_availability,
                      p.mil_margin_db);
    }
}

inline void export_mil_profile(FILE* fp, const std::vector<LimitPoints>& all,
                               const std::vector<const char*>& ids) {
    if (fp == nullptr) {
        return;
    }
    std::fprintf(fp, "Phase 3 MIL-STD Anti-Jam limit summary (PROMPT 49 v3.0)\n");
    std::fprintf(fp, "============================================\n");
    for (std::size_t i = 0; i < all.size() && i < ids.size(); ++i) {
        const LimitPoints& L = all[i];
        std::fprintf(fp, "\n[%s]\n", ids[i]);
        std::fprintf(fp, "  param@BER1e-2: ");
        fmt_nan(fp, L.param_at_ber_1e2);
        std::fprintf(fp, "\n  param@BER1e-3: ");
        fmt_nan(fp, L.param_at_ber_1e3);
        std::fprintf(fp, "\n  param@BER1e-5: ");
        fmt_nan(fp, L.param_at_ber_1e5);
        std::fprintf(fp, "\n  param@BER1e-6: ");
        fmt_nan(fp, L.param_at_ber_1e6);
        std::fprintf(fp, "\n  decode_fail onset (heuristic): ");
        fmt_nan(fp, L.param_at_decode_fail);
        std::fprintf(fp, "\n  anti_jam_margin_db (program-defined): ");
        fmt_nan(fp, L.anti_jam_margin_db);
        std::fprintf(fp, "\n  threat_category (reporting): %s\n",
                      L.threat_category != nullptr ? L.threat_category : "");
    }
}

inline void export_analysis_md(FILE* fp, const MilScenario& sc,
                                const std::vector<PointResult>& results, const LimitPoints& lim) {
    if (fp == nullptr) {
        return;
    }
    std::fprintf(fp, "## Scenario %s — %s\n\n", sc.id != nullptr ? sc.id : "",
                   sc.name != nullptr ? sc.name : "");
    std::fprintf(fp, "- **Jammer**: `%s`\n", sc.jammer != nullptr ? sc.jammer : "");
    std::fprintf(fp, "- **MIL reference (prompt)**: %s\n",
                 sc.mil_std_ref != nullptr ? sc.mil_std_ref : "");
    std::fprintf(fp, "- **Trials/point**: %d (~%llu bits)\n", sc.trials_per_point,
                 static_cast<unsigned long long>(sc.trials_per_point) * kBitsPerTrial);
    std::fprintf(fp, "\n### Limit estimates (linear interp, monotonic sweep)\n\n");
    std::fprintf(fp, "| Threshold | Param at crossing |\n|-----------|--------------------|\n");
    std::fprintf(fp, "| BER 1e-2 | ");
    fmt_nan(fp, lim.param_at_ber_1e2);
    std::fprintf(fp, " |\n| BER 1e-3 | ");
    fmt_nan(fp, lim.param_at_ber_1e3);
    std::fprintf(fp, " |\n| BER 1e-5 | ");
    fmt_nan(fp, lim.param_at_ber_1e5);
    std::fprintf(fp, " |\n| BER 1e-6 | ");
    fmt_nan(fp, lim.param_at_ber_1e6);
    std::fprintf(fp, " |\n\n");
    std::fprintf(fp, "### Per-point (first / mid / last)\n\n");
    if (!results.empty()) {
        const std::size_t n = results.size();
        const std::size_t mid = n / 2u;
        const PointResult* samples[3] = {&results.front(), &results[mid], &results.back()};
        for (auto* s : samples) {
            std::fprintf(fp, "- param=%.6g BER=%.4g CI=[%.4g,%.4g] PER=%.4g dec_fail=%u sat=N/A\n",
                          s->param_value, s->ber, s->ci_lower, s->ci_upper, s->per,
                          static_cast<unsigned>(s->decode_fail_count));
        }
    }
    std::fprintf(fp, "\n### Interpretation (한계 탐지)\n\n");
    std::fprintf(fp, "- 본 표는 **측정 곡선**에서의 교차 추정치이며, MIL-STD **합격/불합격 판정**이 아님.\n");
    std::fprintf(fp, "- Post-FEC 페이로드 BER (Phase 2 `measure_ber_per`) 기준.\n\n");
}

inline void run_phase3_limit_finding() {
    std::printf("\n\n======================================\n");
    std::printf(" Phase 3: MIL-oriented limit finding (v3.0)\n");
    std::printf(" (limits / margins — not pass/fail)\n");
    std::printf("======================================\n");

    FILE* csv = std::fopen("phase3_all_scenarios.csv", "w");
    if (csv == nullptr) {
        std::printf("[Phase3] ERROR: cannot open phase3_all_scenarios.csv\n");
        return;
    }
    std::fprintf(csv,
                  "scenario,param,total_bits,errors,ber,ci_lo,ci_hi,per,decode_fail,link_avail,"
                  "mil_margin_db\n");

    FILE* profile = std::fopen("phase3_mil_profile.txt", "w");
    FILE* md = std::fopen("phase3_analysis.md", "w");
    if (md != nullptr) {
        std::fprintf(md, "# Phase 3 measurement analysis (auto-generated)\n\n");
        std::fprintf(md, "Clopper-Pearson exact CI (alpha=0.05), bits/point ≥ 100000.\n\n");
    }

    std::vector<LimitPoints> all_limits;
    std::vector<const char*> all_ids;

#if defined(HTS_PHASE3_S1_CLIFF)
    std::printf("\n========== S1_CLIFF: AWGN Cliff Precision ==========\n");
    std::vector<PointStats> s1_cliff_stats;
    const MilScenario s1_cliff_sc = make_s1_cliff_scenario();
    run_scenario_with_stats(s1_cliff_sc, s1_cliff_stats);
    generate_rich_report("S1_CLIFF", s1_cliff_stats);
    std::vector<PointResult> cliff_results;
    cliff_results.reserve(s1_cliff_stats.size());
    for (const auto& ps : s1_cliff_stats) {
        PointResult pr{};
        pr.param_value = ps.param_value;
        pr.total_bits = ps.total_bits;
        pr.errors = ps.errors;
        pr.ber = ps.ber;
        pr.ci_lower = ps.ci_lo;
        pr.ci_upper = ps.ci_hi;
        pr.per = ps.per;
        pr.decode_fail_count = ps.decode_fail;
        pr.link_availability = ps.link_avail;
        pr.mil_margin_db = 0.0;
        cliff_results.push_back(pr);
    }
    export_csv(csv, "S1_CLIFF", cliff_results);
    const LimitPoints lim_c = analyze_limits(cliff_results, s1_cliff_sc);
    all_limits.push_back(lim_c);
    all_ids.push_back("S1_CLIFF");
    if (md != nullptr) {
        export_analysis_md(md, s1_cliff_sc, cliff_results, lim_c);
    }
#endif

    auto run_one = [&](const MilScenario& sc) {
        std::vector<PointResult> results;
        std::printf("[Phase3] Running %s (%s) ...\n", sc.id != nullptr ? sc.id : "",
                    sc.name != nullptr ? sc.name : "");
        run_scenario(sc, results);
        export_csv(csv, sc.id, results);
        const LimitPoints lim = analyze_limits(results, sc);
        all_limits.push_back(lim);
        all_ids.push_back(sc.id);
        if (md != nullptr) {
            export_analysis_md(md, sc, results, lim);
        }
    };

    {
        MilScenario s1{};
#if defined(HTS_PHASE3_DEBUG_S1_MINI)
        s1.id = "S1_DIAG";
        s1.name = "J1 AWGN NOISE DEBUG";
        s1.jammer = "J1";
        s1.param_start = 0.0;
        s1.param_end = -10.0;
        s1.param_step = -5.0;
        s1.trials_per_point = 10;
#else
        s1.id = "S1";
        s1.name = "J1 AWGN (MIL-STD-188-110D ref sweep)";
        s1.jammer = "J1";
        s1.param_start = 20.0;
        s1.param_end = -20.0;
        s1.param_step = -1.0;
        s1.trials_per_point = kTrialsPerPoint;
#endif
        s1.mil_required_jsr_db = 0.0;
        s1.mil_required_ber = 1e-3;
        s1.mil_std_ref = "MIL-STD-188-110D (voice/data BER ref: program table)";
        s1.seed_base = 0xA11A1E01u;
        run_one(s1);
    }

#if defined(HTS_PHASE3_FULL_MATRIX)
    {
        MilScenario s2{};
        s2.id = "S2";
        s2.name = "J2 CW MIL-110D App C style JSR sweep";
        s2.jammer = "J2";
        s2.param_start = 0.0;
        s2.param_end = 40.0;
        s2.param_step = 2.0;
        s2.mil_required_jsr_db = 10.0;
        s2.mil_required_ber = 1e-3;
        s2.mil_std_ref = "MIL-STD-188-110D App C (JSR anchor per PROMPT 49)";
        s2.seed_base = 0xA11A1E02u;
        run_one(s2);
    }
    {
        MilScenario s3{};
        s3.id = "S3";
        s3.name = "J3 pulse JSR_peak sweep (duty fixed)";
        s3.jammer = "J3";
        s3.param_start = 0.0;
        s3.param_end = 30.0;
        s3.param_step = 2.0;
        s3.j3_duty = 0.10;
        s3.mil_required_jsr_db = 20.0;
        s3.mil_required_ber = 1e-3;
        s3.mil_std_ref = "MIL-STD-188-110D pulse family (program anchor)";
        s3.seed_base = 0xA11A1E03u;
        run_one(s3);
    }
    {
        MilScenario s4{};
        s4.id = "S4";
        s4.name = "J4 barrage JSR sweep";
        s4.jammer = "J4";
        s4.param_start = 0.0;
        s4.param_end = 30.0;
        s4.param_step = 2.0;
        s4.mil_required_jsr_db = 10.0;
        s4.mil_required_ber = 1e-3;
        s4.mil_std_ref = "MIL-STD-188-110D partial-band family (anchor)";
        s4.seed_base = 0xA11A1E04u;
        run_one(s4);
    }
    {
        MilScenario s5{};
        s5.id = "S5";
        s5.name = "J5 multi-tone N=4 JSR sweep";
        s5.jammer = "J5";
        s5.param_start = 0.0;
        s5.param_end = 30.0;
        s5.param_step = 2.0;
        s5.mil_required_jsr_db = 15.0;
        s5.mil_required_ber = 1e-3;
        s5.mil_std_ref = "MIL-STD-188-141D multi-tone anchor (program)";
        s5.seed_base = 0xA11A1E05u;
        run_one(s5);
    }
    {
        MilScenario s6{};
        s6.id = "S6";
        s6.name = "J6 swept JSR sweep";
        s6.jammer = "J6";
        s6.param_start = 0.0;
        s6.param_end = 30.0;
        s6.param_step = 2.0;
        s6.mil_required_jsr_db = 15.0;
        s6.mil_required_ber = 1e-3;
        s6.mil_std_ref = "MIL-STD-188-141D swept anchor (program)";
        s6.seed_base = 0xA11A1E06u;
        run_one(s6);
    }
#endif

    if (profile != nullptr) {
        export_mil_profile(profile, all_limits, all_ids);
        std::fclose(profile);
    }
    if (md != nullptr) {
        std::fclose(md);
    }
    std::fclose(csv);

    std::printf("\n[DONE] Phase 3 outputs:\n");
    std::printf("  phase3_all_scenarios.csv\n");
    std::printf("  phase3_mil_profile.txt\n");
    std::printf("  phase3_analysis.md\n");
}

} // namespace Phase3_MIL

#endif // HTS_ENABLE_PHASE3_MIL
