"""Parse t6_step3l_diag.log for Phase 3.L CUBE statistics."""
from __future__ import annotations

import re
import statistics
import sys
from pathlib import Path

DETAIL_RE = re.compile(r"([A-F]):(-?\d+)/(\d+)\((\d)\)")


def main() -> None:
    path = Path(__file__).with_name("t6_step3l_diag.log")
    if len(sys.argv) > 1:
        path = Path(sys.argv[1])

    cube_lines: list[str] = []
    detail_lines: list[str] = []
    with path.open(encoding="utf-8", errors="replace") as f:
        for line in f:
            if "CMYK-DIAG-CUBE-PSLTE" in line:
                cube_lines.append(line.strip())
            elif "[CUBE-DETAIL]" in line and "AMI" not in line:
                detail_lines.append(line.strip())

    n_pass1 = sum(1 for L in cube_lines if "pass=1" in L)
    n_pass0 = sum(1 for L in cube_lines if "pass=0" in L)
    print("CMYK-DIAG-CUBE-PSLTE lines:", len(cube_lines))
    print("pass=1:", n_pass1)
    print("pass=0:", n_pass0)

    fail_counts = {k: 0 for k in "ABCDEF"}
    vals_all = {k: [] for k in "ABCDEF"}
    vals_on_fail = {k: [] for k in "ABCDEF"}

    for L in detail_lines:
        for m in DETAIL_RE.finditer(L):
            face = m.group(1)
            val = int(m.group(2))
            thr = int(m.group(3))
            ok = int(m.group(4))
            vals_all[face].append(val)
            if ok == 0:
                fail_counts[face] += 1
                vals_on_fail[face].append(val)

    print("per-face (0) fail count across CUBE-DETAIL rows:", fail_counts)
    mx = max(fail_counts.values())
    worst = [k for k, v in fail_counts.items() if v == mx]
    print("most_failing_face(s):", ",".join(worst), "count:", mx)

    thr_map = {"A": 200, "B": 45, "C": 80, "D": 1200, "E": 400, "F": 700}
    for F in worst[:1]:
        arr = sorted(vals_on_fail[F])
        if arr:
            med = int(statistics.median(arr))
            print(
                f"  {F}: median metric when {F} fails = {med}, "
                f"threshold GRAVITY_THR_{F}_Q10 = {thr_map[F]}"
            )
        else:
            print(f"  {F}: no isolated {F}-only parse (check regex)")

    # Global median per face (all samples)
    print("median (all detail rows), threshold:")
    for F in "ABCDEF":
        v = vals_all[F]
        if v:
            print(
                f"  {F}: median={int(statistics.median(sorted(v)))} thr={thr_map[F]}"
            )


if __name__ == "__main__":
    main()
