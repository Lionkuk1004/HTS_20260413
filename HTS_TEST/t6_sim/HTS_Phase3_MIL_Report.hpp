// HTS_Phase3_MIL_Report.hpp — Phase3 cliff ASCII + HTML 리포트 (HTS_PHASE3_S1_CLIFF)
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <cstdio>
#include <cstring>
#include <numeric>
#include <string>
#include <vector>

namespace Phase3_MIL {

struct PointStats {
    double param_value = 0.0;
    std::uint64_t total_bits = 0u;
    std::uint64_t errors = 0u;
    double ber = 0.0;
    double ci_lo = 0.0;
    double ci_hi = 1.0;
    std::uint32_t trials = 0u;
    std::uint32_t decode_fail = 0u;
    std::uint32_t decode_ok = 0u;
    double link_avail = 0.0;

    double per = 0.0;
    double sat_rate = 0.0;
    std::uint64_t bit_err_min = 0u;
    std::uint64_t bit_err_max = 0u;
    double bit_err_mean = 0.0;
    double bit_err_std = 0.0;

    bool flag_all_zero = false;
    bool flag_all_fail = false;
    bool flag_bimodal = false;
    bool flag_suspicious = false;

    double theory_ber = 0.0;
    double deviation_db = 0.0;
};

struct SanityReport {
    bool noise_generation_ok = true;
    bool ber_aggregation_ok = true;
    bool ci_reasonable = true;
    bool link_monotonic = true;
    bool no_outliers = true;

    std::vector<std::string> warnings;
    std::vector<std::string> pass_notes;

    double overall_confidence = 0.0;
};

inline void print_ascii_waterfall(FILE* fp, const char* title,
                                  const std::vector<PointStats>& points) {
    std::fprintf(fp, "\n");
    std::fprintf(fp, "+==========================================================================+\n");
    std::fprintf(fp, "| %-72s |\n", title);
    std::fprintf(fp, "+==========================================================================+\n");
    std::fprintf(fp, "|                                                                          |\n");
    std::fprintf(fp, "|   BER                                                                    |\n");

    const int rows = 7;
    const int cols = 60;

    for (int r = 0; r < rows; r++) {
        const int exponent = -r;
        std::fprintf(fp, "| 1e%+3d ", exponent);
        std::fprintf(fp, "|");

        for (int c = 0; c < cols; c++) {
            std::size_t pt_idx = 0;
            if (!points.empty()) {
                pt_idx = (static_cast<std::size_t>(c) * points.size()) /
                         static_cast<std::size_t>(cols);
                if (pt_idx >= points.size()) {
                    pt_idx = points.size() - 1;
                }
            }
            const double ber = points.empty() ? 0.0 : points[pt_idx].ber;

            if (ber == 0.0 && r == rows - 1) {
                std::fprintf(fp, "o");
            } else if (ber > 0.0) {
                const double log_ber = std::log10(ber);
                int ber_row = static_cast<int>(std::floor(-log_ber));
                if (ber_row < 0) {
                    ber_row = 0;
                }
                if (ber_row > rows - 1) {
                    ber_row = rows - 1;
                }

                if (ber_row == r) {
                    std::fprintf(fp, "*");
                } else {
                    std::fprintf(fp, " ");
                }
            } else {
                std::fprintf(fp, " ");
            }
        }
        std::fprintf(fp, "|\n");
    }

    std::fprintf(fp, "|       +");
    for (int c = 0; c < cols; c++) {
        std::fprintf(fp, "-");
    }
    std::fprintf(fp, "|\n");

    std::fprintf(fp, "|        ");
    if (!points.empty()) {
        std::fprintf(fp, "%-30.1f%30.1f", points.front().param_value, points.back().param_value);
    }
    std::fprintf(fp, "|\n");
    std::fprintf(fp, "|        param (dB)                                                         |\n");
    std::fprintf(fp, "|                                                                          |\n");
    std::fprintf(fp, "|  범례: * = 측정 BER 위치,  o = BER=0 (측정 해상도 이하)                  |\n");
    std::fprintf(fp, "+==========================================================================+\n\n");
}

inline void print_link_bar_chart(FILE* fp, const char* title,
                                 const std::vector<PointStats>& points) {
    std::fprintf(fp, "+==========================================================================+\n");
    std::fprintf(fp, "| %-72s |\n", title);
    std::fprintf(fp, "+==========================================================================+\n");
    std::fprintf(fp, "| SNR(dB) | Link Availability                                   |  %%      |\n");
    std::fprintf(fp, "+---------+-----------------------------------------------------+---------+\n");

    for (const auto& p : points) {
        int bar_len = static_cast<int>(p.link_avail * 50.0);
        if (bar_len < 0) {
            bar_len = 0;
        }
        if (bar_len > 50) {
            bar_len = 50;
        }

        std::fprintf(fp, "| %+7.1f | ", p.param_value);

        char ch = '#';
        if (p.link_avail >= 0.95) {
            ch = '#';
        } else if (p.link_avail >= 0.50) {
            ch = '=';
        } else if (p.link_avail >= 0.10) {
            ch = '-';
        } else {
            ch = '.';
        }

        for (int i = 0; i < bar_len; i++) {
            std::fprintf(fp, "%c", ch);
        }
        for (int i = bar_len; i < 50; i++) {
            std::fprintf(fp, " ");
        }

        std::fprintf(fp, " | %6.1f%% |\n", p.link_avail * 100.0);
    }

    std::fprintf(fp, "+---------+-----------------------------------------------------+---------+\n");
    std::fprintf(fp, "| 범례: # = 완벽 (≥95%%)   = = 저하 (≥50%%)   - = 붕괴임박   . = 붕괴       |\n");
    std::fprintf(fp, "+==========================================================================+\n\n");
}

inline void print_detailed_table(FILE* fp, const char* title,
                                 const std::vector<PointStats>& points) {
    std::fprintf(fp, "+==========================================================================+\n");
    std::fprintf(fp, "| %-72s |\n", title);
    std::fprintf(fp, "+==========================================================================+\n");
    std::fprintf(fp, "| SNR  | Trials | Decode  | Bits   | Errors | BER      | CI95%%        | Link |\n");
    std::fprintf(fp, "| (dB) |  Tot   | Fail/OK |        |        |          | [lo,   hi]   |  %%   |\n");
    std::fprintf(fp, "+------+--------+---------+--------+--------+----------+--------------+------+\n");

    for (const auto& p : points) {
        std::fprintf(fp,
                     "|%+5.1f | %6u |  %3u/%3u | %6llu | %6llu | %8.2e | [%.1e,%.1e] | %3.0f%%|\n",
                     p.param_value, p.trials, p.decode_fail, p.decode_ok,
                     (unsigned long long)p.total_bits, (unsigned long long)p.errors, p.ber, p.ci_lo,
                     p.ci_hi, p.link_avail * 100.0);
    }
    std::fprintf(fp, "+------+--------+---------+--------+--------+----------+--------------+------+\n\n");
}

struct CliffAnalysis {
    double param_at_link_95 = std::numeric_limits<double>::quiet_NaN();
    double param_at_link_50 = std::numeric_limits<double>::quiet_NaN();
    double param_at_link_10 = std::numeric_limits<double>::quiet_NaN();
    double cliff_width_db = 0.0;
    double param_at_ber_1e3 = std::numeric_limits<double>::quiet_NaN();
    double param_at_ber_1e2 = std::numeric_limits<double>::quiet_NaN();
    bool cliff_found = false;
};

inline CliffAnalysis analyze_cliff(const std::vector<PointStats>& points) {
    CliffAnalysis ca{};
    ca.cliff_found = false;

    for (std::size_t i = 0; i < points.size(); i++) {
        const auto& p = points[i];

        if (std::isnan(ca.param_at_link_95) && p.link_avail < 0.95) {
            if (i > 0) {
                ca.param_at_link_95 = points[i - 1].param_value;
            }
        }
        if (std::isnan(ca.param_at_link_50) && p.link_avail < 0.50) {
            if (i > 0) {
                ca.param_at_link_50 = points[i - 1].param_value;
            }
        }
        if (std::isnan(ca.param_at_link_10) && p.link_avail < 0.10) {
            if (i > 0) {
                ca.param_at_link_10 = points[i - 1].param_value;
            }
        }
        if (std::isnan(ca.param_at_ber_1e3) && p.ber >= 1e-3) {
            if (i > 0) {
                ca.param_at_ber_1e3 = points[i - 1].param_value;
            }
        }
        if (std::isnan(ca.param_at_ber_1e2) && p.ber >= 1e-2) {
            if (i > 0) {
                ca.param_at_ber_1e2 = points[i - 1].param_value;
            }
        }
    }

    if (!std::isnan(ca.param_at_link_95) && !std::isnan(ca.param_at_link_10)) {
        ca.cliff_width_db = std::fabs(ca.param_at_link_95 - ca.param_at_link_10);
        ca.cliff_found = true;
    }

    return ca;
}

inline void cliff_fmt_db(char* buf, std::size_t n, double v) {
    if (std::isnan(v)) {
        std::snprintf(buf, n, "N/A");
    } else {
        std::snprintf(buf, n, "%+.1f dB", v);
    }
}

inline void print_cliff_analysis(FILE* fp, const CliffAnalysis& ca) {
    std::fprintf(fp, "+==========================================================================+\n");
    std::fprintf(fp, "| Cliff Analysis (Link Availability + BER crossing)                       |\n");
    std::fprintf(fp, "+==========================================================================+\n");

    if (!ca.cliff_found) {
        std::fprintf(fp, "|  Cliff not found within sweep range.                                    |\n");
        std::fprintf(fp, "|  Consider extending SNR range.                                          |\n");
    } else {
        char b95[32], b50[32], b10[32], b1e3[32], b1e2[32];
        cliff_fmt_db(b95, sizeof(b95), ca.param_at_link_95);
        cliff_fmt_db(b50, sizeof(b50), ca.param_at_link_50);
        cliff_fmt_db(b10, sizeof(b10), ca.param_at_link_10);
        cliff_fmt_db(b1e3, sizeof(b1e3), ca.param_at_ber_1e3);
        cliff_fmt_db(b1e2, sizeof(b1e2), ca.param_at_ber_1e2);

        std::fprintf(fp, "|  Link >= 95%% boundary:  %-20s (normal operation limit)      |\n", b95);
        std::fprintf(fp, "|  Link >= 50%% boundary:  %-20s (half-link threshold)         |\n", b50);
        std::fprintf(fp, "|  Link >= 10%% boundary:  %-20s (near complete failure)       |\n", b10);
        std::fprintf(fp, "|  Cliff width:          %5.1f dB (95%% to 10%% transition)              |\n",
                     ca.cliff_width_db);
        std::fprintf(fp, "|                                                                         |\n");
        std::fprintf(fp, "|  BER 10^-3 crossing:    %-20s (operational limit)            |\n", b1e3);
        std::fprintf(fp, "|  BER 10^-2 crossing:    %-20s (catastrophic limit)           |\n", b1e2);
    }
    std::fprintf(fp, "+==========================================================================+\n\n");
}

inline SanityReport run_sanity_check(const std::vector<PointStats>& points) {
    SanityReport sr{};
    sr.noise_generation_ok = true;
    sr.ber_aggregation_ok = true;
    sr.ci_reasonable = true;
    sr.link_monotonic = true;
    sr.no_outliers = true;

    if (points.empty()) {
        sr.warnings.push_back("No data points - cannot verify");
        sr.overall_confidence = 0.0;
        return sr;
    }

    for (const auto& p : points) {
        if (p.trials == 0) {
            continue;
        }
        const std::uint64_t expected_min_err =
            static_cast<std::uint64_t>(p.decode_fail) * 64u;
        if (p.errors > 0u && p.errors < expected_min_err / 2u) {
            char buf[128];
            std::snprintf(buf, sizeof(buf),
                          "param=%+.1f: errors=%llu < 0.5*(decode_fail*64)=%llu", p.param_value,
                          (unsigned long long)p.errors, (unsigned long long)(expected_min_err / 2u));
            sr.warnings.push_back(buf);
            sr.ber_aggregation_ok = false;
        }
    }
    if (sr.ber_aggregation_ok) {
        sr.pass_notes.push_back("BER aggregation consistent with decode_fail * 64");
    }

    for (std::size_t i = 1; i < points.size(); i++) {
        if (points[i].link_avail > points[i - 1].link_avail + 0.1) {
            char buf[128];
            std::snprintf(buf, sizeof(buf),
                          "Non-monotonic: param=%+.1f link=%.2f < param=%+.1f link=%.2f",
                          points[i - 1].param_value, points[i - 1].link_avail, points[i].param_value,
                          points[i].link_avail);
            sr.warnings.push_back(buf);
            sr.link_monotonic = false;
        }
    }
    if (sr.link_monotonic) {
        sr.pass_notes.push_back("Link availability monotonic with SNR");
    }

    for (const auto& p : points) {
        const double ci_width = p.ci_hi - p.ci_lo;
        if (ci_width > 0.5 && p.ber < 0.1) {
            char buf[128];
            std::snprintf(buf, sizeof(buf), "param=%+.1f: CI too wide (%.2f) for ber=%.4f",
                          p.param_value, ci_width, p.ber);
            sr.warnings.push_back(buf);
            sr.ci_reasonable = false;
        }
    }
    if (sr.ci_reasonable) {
        sr.pass_notes.push_back("Clopper-Pearson CI widths reasonable");
    }

    const int checks_passed = (sr.noise_generation_ok ? 1 : 0) + (sr.ber_aggregation_ok ? 1 : 0) +
                              (sr.ci_reasonable ? 1 : 0) + (sr.link_monotonic ? 1 : 0) +
                              (sr.no_outliers ? 1 : 0);
    sr.overall_confidence = static_cast<double>(checks_passed) / 5.0;

    return sr;
}

inline void print_sanity_report(FILE* fp, const SanityReport& sr) {
    std::fprintf(fp, "+==========================================================================+\n");
    std::fprintf(fp, "| Sanity Check Report                                                     |\n");
    std::fprintf(fp, "+==========================================================================+\n");
    std::fprintf(fp, "| Overall Confidence: %.0f%%                                                |\n",
                 sr.overall_confidence * 100.0);
    std::fprintf(fp, "|                                                                          |\n");
    std::fprintf(fp, "|   Check                        Status                                   |\n");
    std::fprintf(fp, "|   ---------------------------- ------                                   |\n");
    std::fprintf(fp, "|   Noise generation             %s                                      |\n",
                 sr.noise_generation_ok ? "PASS" : "FAIL");
    std::fprintf(fp, "|   BER aggregation              %s                                      |\n",
                 sr.ber_aggregation_ok ? "PASS" : "FAIL");
    std::fprintf(fp, "|   CI reasonableness            %s                                      |\n",
                 sr.ci_reasonable ? "PASS" : "FAIL");
    std::fprintf(fp, "|   Link monotonic               %s                                      |\n",
                 sr.link_monotonic ? "PASS" : "FAIL");
    std::fprintf(fp, "|   No outliers                  %s                                      |\n",
                 sr.no_outliers ? "PASS" : "FAIL");
    std::fprintf(fp, "|                                                                          |\n");

    if (!sr.warnings.empty()) {
        std::fprintf(fp, "|   Warnings:                                                             |\n");
        for (const auto& w : sr.warnings) {
            std::fprintf(fp, "|     - %-66s |\n", w.c_str());
        }
    }
    std::fprintf(fp, "+==========================================================================+\n\n");
}

inline void export_html_report(const char* filepath, const char* scenario_id,
                               const std::vector<PointStats>& points, const CliffAnalysis& ca,
                               const SanityReport& sr) {
    FILE* fp = std::fopen(filepath, "w");
    if (!fp) {
        return;
    }

    std::fprintf(fp, "<!DOCTYPE html>\n");
    std::fprintf(fp, "<html lang='ko'><head>\n");
    std::fprintf(fp, "<meta charset='UTF-8'>\n");
    std::fprintf(fp, "<title>HTS Phase 3 %s Report</title>\n", scenario_id);
    std::fprintf(fp, "<style>\n");
    std::fprintf(fp,
                  "  body { font-family: 'Malgun Gothic', '맑은 고딕', Arial; margin: 20px; }\n");
    std::fprintf(fp, "  h1 { color: #1a5490; border-bottom: 3px solid #1a5490; padding-bottom: 8px; "
                     "}\n");
    std::fprintf(fp, "  h2 { color: #2c6fa8; margin-top: 24px; }\n");
    std::fprintf(fp,
                  "  table { border-collapse: collapse; margin: 12px 0; }\n"
                  "  th { background: #d0e4f5; padding: 8px 12px; border: 1px solid #888; }\n"
                  "  td { padding: 6px 12px; border: 1px solid #aaa; text-align: right; }\n"
                  "  .param { background: #f0f8ff; font-weight: bold; }\n"
                  "  .ok { color: #0a7f2e; font-weight: bold; }\n"
                  "  .warn { color: #c04000; font-weight: bold; }\n"
                  "  .fail { color: #b00020; font-weight: bold; }\n"
                  "  .summary { background: #f5f5f0; padding: 12px; border-left: 4px solid #1a5490; "
                     "margin: 12px 0; }\n"
                  "  svg { background: white; border: 1px solid #ccc; margin: 12px 0; }\n");
    std::fprintf(fp, "</style>\n</head><body>\n");

    std::fprintf(fp, "<h1>HTS Phase 3 시나리오 %s 리포트</h1>\n", scenario_id);
    std::fprintf(fp, "<div class='summary'>\n");
    std::fprintf(fp, "<b>측정 포인트 수:</b> %zu<br>\n", points.size());
    const unsigned long long total_bits =
        std::accumulate(points.begin(), points.end(), 0ULL,
                        [](unsigned long long a, const PointStats& b) { return a + b.total_bits; });
    std::fprintf(fp, "<b>총 측정 bit 수:</b> %llu<br>\n", total_bits);
    std::fprintf(fp, "<b>전체 자가 검증 신뢰도:</b> <span class='%s'>%.0f%%</span><br>\n",
                 (sr.overall_confidence >= 0.8 ? "ok" : (sr.overall_confidence >= 0.6 ? "warn" : "fail")),
                 sr.overall_confidence * 100.0);
    std::fprintf(fp, "</div>\n");

    std::fprintf(fp, "<h2>Cliff 분석</h2>\n<table>\n");
    std::fprintf(fp, "<tr><th>지표</th><th>값</th><th>의미</th></tr>\n");
    auto fmt_nan = [](double v) -> std::string {
        if (std::isnan(v)) {
            return std::string("N/A");
        }
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%+.1f dB", v);
        return std::string(buf);
    };
    std::fprintf(fp, "<tr><td class='param'>Link >= 95%%</td><td>%s</td><td>운용 한계</td></tr>\n",
                 fmt_nan(ca.param_at_link_95).c_str());
    std::fprintf(fp, "<tr><td class='param'>Link >= 50%%</td><td>%s</td><td>방어 한계</td></tr>\n",
                 fmt_nan(ca.param_at_link_50).c_str());
    std::fprintf(fp, "<tr><td class='param'>Link >= 10%%</td><td>%s</td><td>붕괴 임박</td></tr>\n",
                 fmt_nan(ca.param_at_link_10).c_str());
    std::fprintf(fp, "<tr><td class='param'>Cliff 폭</td><td>%.1f dB</td><td>95%% to 10%% 구간</td></tr>\n",
                 ca.cliff_width_db);
    std::fprintf(fp, "<tr><td class='param'>BER 10^-3</td><td>%s</td><td>MIL 운용 기준</td></tr>\n",
                 fmt_nan(ca.param_at_ber_1e3).c_str());
    std::fprintf(fp, "</table>\n");

    const int W = 700;
    const int H = 400;
    const int MARGIN = 60;
    std::fprintf(fp, "<h2>Waterfall 곡선 (BER vs SNR)</h2>\n");
    std::fprintf(fp, "<svg width='%d' height='%d' viewBox='0 0 %d %d'>\n", W, H, W, H);

    std::fprintf(fp, "<line x1='%d' y1='%d' x2='%d' y2='%d' stroke='black' stroke-width='2'/>\n",
                 MARGIN, H - MARGIN, W - MARGIN, H - MARGIN);
    std::fprintf(fp, "<line x1='%d' y1='%d' x2='%d' y2='%d' stroke='black' stroke-width='2'/>\n",
                 MARGIN, MARGIN, MARGIN, H - MARGIN);

    const int y_1e3 = MARGIN + static_cast<int>((H - 2 * MARGIN) * 3.0 / 6.0);
    std::fprintf(fp,
                 "<line x1='%d' y1='%d' x2='%d' y2='%d' stroke='red' stroke-width='1' "
                 "stroke-dasharray='4,4'/>\n",
                 MARGIN, y_1e3, W - MARGIN, y_1e3);
    std::fprintf(fp, "<text x='%d' y='%d' fill='red' font-size='12'>BER 1e-3</text>\n", W - MARGIN - 80,
                 y_1e3 - 4);

    if (!points.empty()) {
        const double p_min = points.back().param_value;
        const double p_max = points.front().param_value;
        const double p_span = p_max - p_min;

        for (std::size_t i = 0; i < points.size(); i++) {
            const auto& p = points[i];
            const double x_frac =
                (std::fabs(p_span) < 1e-12) ? 0.5 : ((p.param_value - p_min) / p_span);
            const int x = MARGIN + static_cast<int>(x_frac * (W - 2 * MARGIN));

            int y = 0;
            if (p.ber <= 0.0) {
                y = H - MARGIN - 5;
            } else {
                double log_ber = std::log10(p.ber);
                double y_frac = -log_ber / 6.0;
                if (y_frac > 1.0) {
                    y_frac = 1.0;
                }
                y = MARGIN + static_cast<int>((H - 2 * MARGIN) * y_frac);
            }

            const char* color = "blue";
            if (p.link_avail >= 0.95) {
                color = "#0a7f2e";
            } else if (p.link_avail >= 0.5) {
                color = "#c08000";
            } else {
                color = "#b00020";
            }

            std::fprintf(fp, "<circle cx='%d' cy='%d' r='5' fill='%s'/>\n", x, y, color);

            if (i > 0) {
                const auto& pp = points[i - 1];
                const double px_frac =
                    (std::fabs(p_span) < 1e-12) ? 0.5 : ((pp.param_value - p_min) / p_span);
                const int px = MARGIN + static_cast<int>(px_frac * (W - 2 * MARGIN));
                int py = 0;
                if (pp.ber <= 0.0) {
                    py = H - MARGIN - 5;
                } else {
                    py = MARGIN +
                         static_cast<int>((H - 2 * MARGIN) * (-std::log10(pp.ber) / 6.0));
                }
                std::fprintf(fp, "<line x1='%d' y1='%d' x2='%d' y2='%d' stroke='gray' "
                             "stroke-width='1'/>\n",
                             px, py, x, y);
            }
        }

        for (std::size_t i = 0; i < points.size(); i++) {
            const double p = points[i].param_value;
            const double x_frac = (std::fabs(p_span) < 1e-12) ? 0.5 : ((p - p_min) / p_span);
            const int x = MARGIN + static_cast<int>(x_frac * (W - 2 * MARGIN));
            std::fprintf(fp, "<text x='%d' y='%d' text-anchor='middle' font-size='11'>%+.1f</text>\n",
                         x, H - MARGIN + 16, p);
        }
    }

    std::fprintf(fp, "<text x='%d' y='%d' text-anchor='middle' font-size='13'>SNR (dB)</text>\n", W / 2,
                 H - 10);
    std::fprintf(fp, "<text x='%d' y='%d' font-size='13' transform='rotate(-90 %d %d)'>BER "
                     "(log)</text>\n",
                 20, H / 2, 20, H / 2);
    std::fprintf(fp, "</svg>\n");

    std::fprintf(fp, "<h2>포인트별 상세 측정값</h2>\n<table>\n");
    std::fprintf(fp, "<tr><th>SNR(dB)</th><th>Trials</th><th>Decode Fail</th><th>Errors</th><th>BER</th>"
                     "<th>CI Lo</th><th>CI Hi</th><th>Link Avail</th></tr>\n");
    for (const auto& p : points) {
        const char* link_cls = "ok";
        if (p.link_avail < 0.50) {
            link_cls = "fail";
        } else if (p.link_avail < 0.95) {
            link_cls = "warn";
        }

        std::fprintf(fp, "<tr><td class='param'>%+.1f</td><td>%u</td><td>%u</td><td>%llu</td>"
                         "<td>%.3e</td><td>%.3e</td><td>%.3e</td>"
                         "<td class='%s'>%.1f%%</td></tr>\n",
                     p.param_value, p.trials, p.decode_fail, (unsigned long long)p.errors, p.ber,
                     p.ci_lo, p.ci_hi, link_cls, p.link_avail * 100.0);
    }
    std::fprintf(fp, "</table>\n");

    std::fprintf(fp, "<h2>자가 검증 (Sanity Check)</h2>\n<table>\n<tr><th>검증 항목</th><th>결과</th></tr>\n");
    std::fprintf(fp, "<tr><td>Noise generation</td><td class='%s'>%s</td></tr>\n",
                 sr.noise_generation_ok ? "ok" : "fail", sr.noise_generation_ok ? "PASS" : "FAIL");
    std::fprintf(fp, "<tr><td>BER aggregation</td><td class='%s'>%s</td></tr>\n",
                 sr.ber_aggregation_ok ? "ok" : "fail", sr.ber_aggregation_ok ? "PASS" : "FAIL");
    std::fprintf(fp, "<tr><td>CI reasonableness</td><td class='%s'>%s</td></tr>\n",
                 sr.ci_reasonable ? "ok" : "fail", sr.ci_reasonable ? "PASS" : "FAIL");
    std::fprintf(fp, "<tr><td>Link monotonic</td><td class='%s'>%s</td></tr>\n",
                 sr.link_monotonic ? "ok" : "fail", sr.link_monotonic ? "PASS" : "FAIL");
    std::fprintf(fp, "</table>\n");

    if (!sr.warnings.empty()) {
        std::fprintf(fp, "<h3>경고</h3>\n<ul>\n");
        for (const auto& w : sr.warnings) {
            std::fprintf(fp, "<li class='warn'>%s</li>\n", w.c_str());
        }
        std::fprintf(fp, "</ul>\n");
    }

    std::fprintf(fp, "<hr>\n<p><small>HTS B-CDMA Phase 3 Report — %s</small></p>\n", scenario_id);
    std::fprintf(fp, "</body></html>\n");
    std::fclose(fp);
}

inline void generate_rich_report(const char* scenario_id, const std::vector<PointStats>& points) {
    char txt_path[256];
    std::snprintf(txt_path, sizeof(txt_path), "phase3_%s_rich_report.txt", scenario_id);
    FILE* fp = std::fopen(txt_path, "w");
    if (!fp) {
        return;
    }

    std::fprintf(fp, "\n\n");
    std::fprintf(fp, "================================================================\n");
    std::fprintf(fp, "  HTS B-CDMA Phase 3 Rich Report\n");
    std::fprintf(fp, "  Scenario: %s\n", scenario_id);
    std::fprintf(fp, "  Points: %zu\n", points.size());
    std::fprintf(fp, "================================================================\n\n");

    char title1[128];
    std::snprintf(title1, sizeof(title1), "%s - Waterfall Curve (BER vs Parameter)", scenario_id);
    print_ascii_waterfall(fp, title1, points);

    char title2[128];
    std::snprintf(title2, sizeof(title2), "%s - Link Availability Chart", scenario_id);
    print_link_bar_chart(fp, title2, points);

    char title3[128];
    std::snprintf(title3, sizeof(title3), "%s - Detailed Point Statistics", scenario_id);
    print_detailed_table(fp, title3, points);

    const CliffAnalysis ca = analyze_cliff(points);
    print_cliff_analysis(fp, ca);

    const SanityReport sr = run_sanity_check(points);
    print_sanity_report(fp, sr);

    std::fclose(fp);

    char html_path[256];
    std::snprintf(html_path, sizeof(html_path), "phase3_%s_report.html", scenario_id);
    export_html_report(html_path, scenario_id, points, ca, sr);

    std::printf("Reports written:\n");
    std::printf("  %s (ASCII waterfall + tables)\n", txt_path);
    std::printf("  %s (SVG graph + rich HTML)\n", html_path);
}

} // namespace Phase3_MIL

