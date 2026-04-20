#!/usr/bin/env python3
"""
Step D-Pre-4: HARQ Matrix CSV — Chase vs IR by intensity band.

crc_rate 컬럼은 0.0~1.0 비율. 출력은 % 및 %p 기준.
셀 키: (mode, channel, intensity_unit, intensity)
"""
from __future__ import annotations

import csv
import sys
from collections import defaultdict

CSV_PATH = "HARQ_Matrix_Results.csv"
OUT_TXT = "stepDpre4_by_intensity.txt"


def cell_key(row: dict) -> tuple[str, str, str, str]:
    return (
        row["mode"],
        row["channel"],
        row["intensity_unit"],
        row["intensity"],
    )


def analyze() -> int:
    with open(CSV_PATH, "r", encoding="utf-8-sig", newline="") as f:
        rows = list(csv.DictReader(f))
    print(f"총 행: {len(rows)}")
    if not rows:
        print("CSV 비어있음", file=sys.stderr)
        return 1

    fec_col, crc_col = "fec_path", "crc_rate"
    cell_results: dict[tuple, dict[str, float]] = defaultdict(dict)
    chase_rates: list[float] = []
    ir_rates: list[float] = []

    for r in rows:
        fec = r[fec_col].strip().lower()
        rate = float(r[crc_col])
        key = cell_key(r)
        if fec == "chase":
            cell_results[key]["chase"] = rate
            chase_rates.append(rate)
        elif fec in ("ir_harq", "ir"):
            cell_results[key]["ir"] = rate
            ir_rates.append(rate)

    paired = [(k, v) for k, v in cell_results.items() if "chase" in v and "ir" in v]
    print(f"페어링된 cell 수: {len(paired)} (기대 176)")

    chase_mean = sum(chase_rates) / len(chase_rates) if chase_rates else 0.0
    ir_mean = sum(ir_rates) / len(ir_rates) if ir_rates else 0.0
    diff_pp = 100.0 * (ir_mean - chase_mean)

    def band_easy(v: dict[str, float]) -> bool:
        return min(v["chase"], v["ir"]) >= 0.9

    def band_hard(v: dict[str, float]) -> bool:
        return max(v["chase"], v["ir"]) < 0.3

    def band_moderate(v: dict[str, float]) -> bool:
        if band_easy(v) or band_hard(v):
            return False
        hi = max(v["chase"], v["ir"])
        lo = min(v["chase"], v["ir"])
        return hi <= 0.9 and lo >= 0.3

    def band_moderate_loose(v: dict[str, float]) -> bool:
        """30~90% 구간: 둘 중 하나라도 (0.3,0.9] 안에 있으면 moderate 후보."""
        if band_easy(v) or band_hard(v):
            return False
        c, i = v["chase"], v["ir"]
        def in_mid(x: float) -> bool:
            return 0.3 <= x <= 0.9
        return in_mid(c) or in_mid(i)

    def write_band(
        f,
        name: str,
        pred,
    ) -> None:
        cells = [(k, v) for k, v in paired if pred(v)]
        f.write(f"─── {name} ───\n")
        if not cells:
            f.write("  (없음)\n\n")
            return
        ca = sum(v["chase"] for _, v in cells) / len(cells)
        ia = sum(v["ir"] for _, v in cells) / len(cells)
        f.write(f"  cell 수: {len(cells)}\n")
        f.write(f"  Chase avg: {100.0 * ca:.2f}%\n")
        f.write(f"  IR avg:    {100.0 * ia:.2f}%\n")
        f.write(f"  Δ(IR-Chase): {100.0 * (ia - ca):+.2f}%p\n")
        thr = 0.02
        ir_wins = sum(1 for _, v in cells if v["ir"] > v["chase"] + thr)
        ch_wins = sum(1 for _, v in cells if v["chase"] > v["ir"] + thr)
        ties = len(cells) - ir_wins - ch_wins
        f.write(f"  IR 우세 (Δ>{100*thr:.0f}%p): {ir_wins}\n")
        f.write(f"  Chase 우세:            {ch_wins}\n")
        f.write(f"  동등 (±{100*thr:.0f}%p):         {ties}\n")
        top_ir = sorted(cells, key=lambda x: x[1]["ir"] - x[1]["chase"], reverse=True)[:5]
        f.write("\n  IR 유리 Top 5 (Δ%p):\n")
        for k, v in top_ir:
            d = 100.0 * (v["ir"] - v["chase"])
            f.write(
                f"    mode={k[0]} ch={k[1]} unit={k[2]} inten={k[3]}: "
                f"ch={100*v['chase']:.1f}% ir={100*v['ir']:.1f}% Δ={d:+.1f}%p\n"
            )
        top_ch = sorted(cells, key=lambda x: x[1]["chase"] - x[1]["ir"], reverse=True)[:5]
        f.write("\n  Chase 유리 Top 5 (Δ%p):\n")
        for k, v in top_ch:
            d = 100.0 * (v["chase"] - v["ir"])
            f.write(
                f"    mode={k[0]} ch={k[1]} unit={k[2]} inten={k[3]}: "
                f"ch={100*v['chase']:.1f}% ir={100*v['ir']:.1f}% Δ={d:+.1f}%p\n"
            )
        f.write("\n")

    with open(OUT_TXT, "w", encoding="utf-8") as f:
        f.write("=" * 80 + "\n")
        f.write("Step D-Pre-4: HARQ Matrix Chase vs IR (강도 밴드)\n")
        f.write("=" * 80 + "\n\n")
        f.write(f"입력: {CSV_PATH}\n")
        f.write(f"페어 cell: {len(paired)}\n\n")
        f.write("━━━ 전체 평균 (행 단순 평균) ━━━\n")
        f.write(f"  Chase:   {100.0 * chase_mean:.2f}% ({len(chase_rates)}행)\n")
        f.write(f"  IR:      {100.0 * ir_mean:.2f}% ({len(ir_rates)}행)\n")
        f.write(f"  Δ(IR-Chase): {diff_pp:+.2f}%p\n\n")

        f.write("밴드 정의:\n")
        f.write("  Easy:     min(chase,ir) >= 90%\n")
        f.write("  Hard:     max(chase,ir) < 30%\n")
        f.write("  Moderate: 그 외 + min>=30% and max<=90% (둘 다 중간대)\n")
        f.write("  ModerateB: 그 외 + chase 또는 ir 이 [30%,90%] 안\n\n")

        write_band(f, "Easy (min>=90%)", band_easy)
        write_band(f, "Moderate (30~90% 박스, easy/hard 제외)", band_moderate)
        write_band(f, "Moderate-B (한 축이라도 30~90%)", band_moderate_loose)
        write_band(f, "Hard (max<30%)", band_hard)

        diffs_pp = [100.0 * (v["chase"] - v["ir"]) for _, v in paired]
        f.write("=" * 80 + "\n")
        f.write("히스토그램: (Chase−IR) %p, 페어 cell 전체\n")
        f.write("=" * 80 + "\n\n")
        buckets = [
            (-100.0, -20.0),
            (-20.0, -10.0),
            (-10.0, -5.0),
            (-5.0, -2.0),
            (-2.0, 2.0),
            (2.0, 5.0),
            (5.0, 10.0),
            (10.0, 20.0),
            (20.0, 100.0),
        ]
        for lo, hi in buckets:
            cnt = sum(1 for d in diffs_pp if lo <= d < hi)
            bar = "#" * min(cnt, 120)
            f.write(f"  [{lo:>+6.1f}, {hi:>+6.1f}) %p: {cnt:>4d}  {bar}\n")

        # Q4-style: mid 30~70% by max rate
        mid = [
            (k, v)
            for k, v in paired
            if 0.3 <= max(v["chase"], v["ir"]) <= 0.7
        ]
        f.write("\n" + "=" * 80 + "\n")
        f.write("샘플: max(crc) in [30%, 70%] (중간 구간 cell 최대 8개)\n")
        f.write("=" * 80 + "\n")
        for k, v in sorted(mid, key=lambda x: max(x[1]["chase"], x[1]["ir"]))[:8]:
            f.write(
                f"  mode={k[0]} ch={k[1]} inten={k[3]}: "
                f"ch={100*v['chase']:.1f}% ir={100*v['ir']:.1f}%\n"
            )

    print(f"완료: {OUT_TXT}")
    return 0


if __name__ == "__main__":
    raise SystemExit(analyze())
