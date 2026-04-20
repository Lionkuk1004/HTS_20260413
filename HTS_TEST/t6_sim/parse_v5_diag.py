#!/usr/bin/env python3
"""Parse stepB6_harq_on.log STAGE4-V5-2B lines aligned with HARQ_Matrix_Results row order."""
import csv
import re
import statistics
import sys
from pathlib import Path

LOG = Path("stepB6_harq_on.log")
CSV = Path("HARQ_Matrix_Results_STEPB_ON.csv")
PAT = re.compile(
    r"\[STAGE4-V5-2B\] off=(-?\d+) v5_score=(\d+) legacy_seed=\((-?\d+),(-?\d+)\)"
)


def main() -> int:
    if not LOG.is_file() or not CSV.is_file():
        print("missing log or csv", file=sys.stderr)
        return 1
    text = LOG.read_text(encoding="utf-8", errors="replace")
    matches = PAT.findall(text)
    scores = [int(m[1]) for m in matches]
    if not scores:
        print("no STAGE4-V5-2B lines", file=sys.stderr)
        return 1

    rows = []
    with CSV.open("r", encoding="utf-8-sig", newline="") as f:
        r = csv.DictReader(f)
        for row in r:
            rows.append(row)

    trials_per_cell = 100
    n_expected = len(rows) * trials_per_cell
    if len(scores) != n_expected:
        print(
            f"warn: score lines {len(scores)} vs csv_rows*100={n_expected}",
            file=sys.stderr,
        )

    def cell_scores(i: int) -> list[int]:
        base = i * trials_per_cell
        return scores[base : base + trials_per_cell]

    def summarize(name: str, pred) -> None:
        sub: list[int] = []
        for i, row in enumerate(rows):
            if pred(row):
                sub.extend(cell_scores(i))
        if not sub:
            print(f"## {name}\n(no rows)\n")
            return
        print(f"## {name}")
        print(f"- trial 수: {len(sub)}")
        print(f"- 평균 v5_score: {statistics.mean(sub):.1f}")
        print(f"- 최소: {min(sub)}")
        print(f"- 최대: {max(sub)}")
        if len(sub) > 1:
            print(f"- 표준편차: {statistics.pstdev(sub):.1f}")
        print()

    print("# Step B — v5 Walsh-row DIAG (로그 정렬 가정: CSV 행 순서 == 셀 순서)\n")
    summarize("Clean (전체)", lambda r: r["channel"] == "Clean")
    summarize("Swept 22 dB (intensity 22.00)", lambda r: r["channel"] == "Swept" and r["intensity"] == "22.00")
    summarize("Multi 17 dB", lambda r: r["channel"] == "MultiTone" and r["intensity"] == "17.00")
    summarize("AWGN -16 dB", lambda r: r["channel"] == "AWGN" and r["intensity"] == "-16.00")
    summarize("Barrage 18 dB_JSR", lambda r: r["channel"] == "Barrage" and r["intensity"] == "18.00")

    # legacy magnitude correlation (per trial)
    offs = [int(m[0]) for m in matches]
    leg_i = [int(m[2]) for m in matches]
    leg_q = [int(m[3]) for m in matches]
    leg_mag2 = [float(li) * li + float(lq) * lq for li, lq in zip(leg_i, leg_q)]
    n = min(len(scores), len(leg_mag2))
    sx = statistics.pstdev(scores[:n]) or 1e-9
    sy = statistics.pstdev(leg_mag2[:n]) or 1e-9
    mx = statistics.mean(scores[:n])
    my = statistics.mean(leg_mag2[:n])
    cov = sum((scores[i] - mx) * (leg_mag2[i] - my) for i in range(n)) / n
    r_pearson = cov / (sx * sy) if sx and sy else float("nan")
    print("## Legacy seed vs v5_score (전체 trial)")
    print(f"- 피어슨 상관 (v5_score, |legacy|^2): {r_pearson:.4f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
