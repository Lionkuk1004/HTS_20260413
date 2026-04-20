#!/usr/bin/env python3
"""
Step C-1.5 parser v2: SNR row closes the *previous* SNR bucket (not the next).
Input:  stepC1_4_t6_v5.log
Output: stepC1_5_s3_v5_stats_v2.txt / stepC1_5_s3_v5_stats_v2.csv
"""
from __future__ import annotations

import math
import re
import statistics
import sys
from collections import defaultdict

LOG_PATH = "stepC1_4_t6_v5.log"
LOG_ENCODING = "utf-16"  # stepC1_4_t6_v5.log is UTF-16 LE (BOM)
OUT_TXT = "stepC1_5_s3_v5_stats_v2.txt"
OUT_CSV = "stepC1_5_s3_v5_stats_v2.csv"

S3_HEADER = re.compile(r"│\s*S3\s.*심해 SNR")
S4_HEADER = re.compile(r"│\s*S4\s")
SNR_LABEL = re.compile(r"SNR\s+([+-]?\s*\d+)\s*dB")
STAGE4 = re.compile(
    r"\[STAGE4-V5-2B\]\s+off=(\d+)\s+v5_score=(\d+)\s+legacy_seed=\(([+-]?\d+),([+-]?\d+)\)"
)

EXPECTED_SNRS = [-30, -25, -20, -15, -10, -5, 0, 5, 10]


def parse_log() -> dict[int, list[tuple[int, int, int, int]]]:
    in_s3 = False
    pending: list[tuple[int, int, int, int]] = []
    bucket_index = 0
    data: dict[int, list[tuple[int, int, int, int]]] = defaultdict(list)
    lines_seen = 0

    with open(LOG_PATH, "r", encoding=LOG_ENCODING, errors="replace") as f:
        for line in f:
            lines_seen += 1
            if not in_s3:
                if S3_HEADER.search(line):
                    in_s3 = True
                    pending = []
                    bucket_index = 0
                continue

            if S4_HEADER.search(line):
                if pending:
                    target = (
                        EXPECTED_SNRS[bucket_index]
                        if bucket_index < len(EXPECTED_SNRS)
                        else EXPECTED_SNRS[-1]
                    )
                    data[target].extend(pending)
                    pending = []
                in_s3 = False
                break

            m_st = STAGE4.search(line)
            if m_st:
                off = int(m_st.group(1))
                score = int(m_st.group(2))
                si = int(m_st.group(3))
                sq = int(m_st.group(4))
                pending.append((off, score, si, sq))
                continue

            m_snr = SNR_LABEL.search(line)
            if m_snr:
                if bucket_index < len(EXPECTED_SNRS):
                    expected = EXPECTED_SNRS[bucket_index]
                    label_val = int(m_snr.group(1).replace(" ", ""))
                    if label_val != expected:
                        print(
                            f"WARN: bucket {bucket_index} expected {expected}, label {label_val}",
                            file=sys.stderr,
                        )
                    data[expected].extend(pending)
                    pending = []
                    bucket_index += 1
                continue

    print(f"총 로그 라인: {lines_seen}")
    print(f"S3 버킷 할당 결과: {[(k, len(v)) for k, v in sorted(data.items())]}")
    return data


def analyze(data: dict[int, list]) -> list[dict]:
    results = []
    for snr in sorted(data.keys()):
        vals = data[snr]
        scores = [v[1] for v in vals]
        non_zero = [s for s in scores if s > 0]
        n_total = len(scores)
        n_nonzero = len(non_zero)
        mean_s = statistics.mean(non_zero) if non_zero else 0.0
        median_s = statistics.median(non_zero) if non_zero else 0.0
        std_s = statistics.stdev(non_zero) if len(non_zero) > 1 else 0.0
        max_s = max(non_zero) if non_zero else 0

        nz_vals = [(v[1], v[2] * v[2] + v[3] * v[3]) for v in vals if v[1] > 0]
        corr = 0.0
        if len(nz_vals) >= 2:
            xs = [x[0] for x in nz_vals]
            ys = [x[1] for x in nz_vals]
            mx = statistics.mean(xs)
            my = statistics.mean(ys)
            sx = statistics.stdev(xs) if len(xs) > 1 else 0.0
            sy = statistics.stdev(ys) if len(ys) > 1 else 0.0
            if sx > 0 and sy > 0:
                covar = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / (
                    len(xs) - 1
                )
                corr = covar / (sx * sy)

        results.append(
            {
                "snr": snr,
                "n_total": n_total,
                "n_nonzero": n_nonzero,
                "mean": mean_s,
                "median": median_s,
                "std": std_s,
                "max": max_s,
                "corr_legacy": corr,
            }
        )
    return results


def save(results: list[dict]) -> None:
    with open(OUT_TXT, "w", encoding="utf-8") as f:
        f.write("=" * 115 + "\n")
        f.write("Step C-1.5 (parser v2): S3 SNR별 v5_score 분포\n")
        f.write("SNR 행은 직전 trial 구간의 마감으로 집계\n")
        f.write("=" * 115 + "\n\n")
        f.write(
            f"{'SNR(dB)':>8} {'n_total':>9} {'n_nonzero':>11} {'non_zero%':>11} "
            f"{'mean':>14} {'median':>14} {'stdev':>14} {'max':>14} {'corr':>8}\n"
        )
        f.write("-" * 115 + "\n")
        for r in results:
            pct = 100.0 * r["n_nonzero"] / r["n_total"] if r["n_total"] > 0 else 0.0
            f.write(
                f"{r['snr']:>+8d} {r['n_total']:>9d} {r['n_nonzero']:>11d} "
                f"{pct:>10.1f}% "
                f"{r['mean']:>14.3e} {r['median']:>14.3e} {r['std']:>14.3e} "
                f"{r['max']:>14.3e} {r['corr_legacy']:>8.3f}\n"
            )

        f.write("\n" + "=" * 115 + "\n")
        f.write("LPI 지표 (mean 기준, mean>0 인 최고 SNR을 reference)\n")
        f.write("=" * 115 + "\n\n")
        ref = None
        for r in sorted(results, key=lambda x: -x["snr"]):
            if r["mean"] > 0:
                ref = r
                break
        if ref is None:
            f.write("기준점(mean>0)이 없어 LPI 계산 불가.\n")
        else:
            f.write(f"Reference SNR = {ref['snr']:+d} dB (mean={ref['mean']:.3e})\n\n")
            f.write(f"{'SNR(dB)':>8} {'mean':>14} {'ratio_vs_ref':>14} {'loss_dB':>10}\n")
            f.write("-" * 60 + "\n")
            for r in results:
                if r["mean"] > 0:
                    ratio = r["mean"] / ref["mean"]
                    loss = -10 * math.log10(ratio) if ratio > 0 else float("inf")
                    f.write(
                        f"{r['snr']:>+8d} {r['mean']:>14.3e} {ratio:>14.4f} {loss:>10.2f}\n"
                    )
                else:
                    f.write(f"{r['snr']:>+8d} {0.0:>14.3e} {'N/A':>14} {'N/A':>10}\n")

    with open(OUT_CSV, "w", encoding="utf-8") as f:
        f.write(
            "snr_dB,n_total,n_nonzero,non_zero_pct,mean,median,stdev,max,corr_legacy\n"
        )
        for r in results:
            pct = 100.0 * r["n_nonzero"] / r["n_total"] if r["n_total"] > 0 else 0.0
            f.write(
                f"{r['snr']},{r['n_total']},{r['n_nonzero']},{pct:.2f},"
                f"{r['mean']:.3e},{r['median']:.3e},{r['std']:.3e},"
                f"{r['max']:.3e},{r['corr_legacy']:.3f}\n"
            )


def main() -> int:
    data = parse_log()
    if not data:
        print("ERROR: S3 데이터 0건.", file=sys.stderr)
        return 1
    results = analyze(data)
    save(results)
    print(f"\n완료: {OUT_TXT}, {OUT_CSV}")
    print("\n요약:")
    for r in results:
        pct = 100.0 * r["n_nonzero"] / r["n_total"] if r["n_total"] > 0 else 0.0
        print(
            f"  SNR {r['snr']:+3d} dB: n={r['n_total']:>4d}, "
            f"non_zero={r['n_nonzero']} ({pct:.1f}%), mean={r['mean']:.2e}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
