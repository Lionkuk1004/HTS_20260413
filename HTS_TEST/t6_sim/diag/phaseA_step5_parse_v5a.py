# Phase A-2 Step 5: parse [V5A-S5H-TRIAL] true=%d est=%d valid=%d
# Usage: python phaseA_step5_parse_v5a.py <log_or_txt>
from __future__ import annotations

import re
import statistics
import sys
from collections import defaultdict

LINE_RE = re.compile(
    r"\[V5A-S5H-TRIAL\]\s*true=(-?\d+)\s+est=(-?\d+)\s+valid=([01])"
)

# User report subset (Hz)
REPORT_CFOS = (
    0, 50, 100, 200, 500, 1000, 2000, 3500, 5000, 7500, 10000, 15000, 20000, 25000
)


def random_walk_y(true_hz: int, mean_est: float, std_est: float,
                  mean_abs_err: float, valid_pct: float) -> bool:
    if true_hz == 0:
        return abs(mean_est) > 100.0 or std_est > 200.0
    return (
        mean_abs_err > 0.5 * true_hz
        or std_est > 0.5 * true_hz
        or valid_pct < 70.0
    )


def main() -> int:
    path = sys.argv[1] if len(sys.argv) > 1 else "out_lab/phaseA_step5_s5h_full.log"
    by_true: dict[int, list[tuple[int, bool]]] = defaultdict(list)
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            m = LINE_RE.search(line)
            if not m:
                continue
            t = int(m.group(1))
            e = int(m.group(2))
            v = m.group(3) == "1"
            by_true[t].append((e, v))

    if not by_true:
        print("No V5A-S5H-TRIAL lines parsed.", file=sys.stderr)
        return 1

    rows = []
    for true_hz in sorted(by_true.keys()):
        samples = by_true[true_hz]
        ests = [e for e, _ in samples]
        valids = sum(1 for _, v in samples if v)
        n = len(samples)
        mean_est = statistics.fmean(ests)
        std_est = statistics.pstdev(ests) if n > 1 else 0.0
        mean_abs_err = statistics.fmean(abs(e - true_hz) for e in ests)
        valid_pct = 100.0 * valids / n
        rw = random_walk_y(true_hz, mean_est, std_est, mean_abs_err, valid_pct)
        rows.append(
            {
                "true": true_hz,
                "n": n,
                "mean_est": mean_est,
                "std_est": std_est,
                "mean_abs_err": mean_abs_err,
                "valid_pct": valid_pct,
                "random_walk": rw,
            }
        )

    # deadzone (literal spec): smallest true CFO (Hz) where random_walk is False ("N")
    stable = [r for r in rows if not r["random_walk"]]
    deadzone_cfo = min(r["true"] for r in stable) if stable else None
    unstable = [r for r in rows if r["random_walk"]]
    first_unstable = min(r["true"] for r in unstable) if unstable else None

    print("## Phase A-2 Step 5 — V5a per-true-CFO (all S5H sweep points in log)\n")
    print(
        "| true CFO (Hz) | n (samples) | est mean | est std | mean |est-true| | valid % | random_walk |"
    )
    print("|---:|---:|---:|---:|---:|---:|:---:|")
    for r in rows:
        yn = "Y" if r["random_walk"] else "N"
        print(
            f"| {r['true']} | {r['n']} | {r['mean_est']:.4g} | {r['std_est']:.4g} | "
            f"{r['mean_abs_err']:.4g} | {r['valid_pct']:.2f} | {yn} |"
        )

    print("\n### Subset (requested report rows)\n")
    print(
        "| true CFO | est mean | est std | mean abs err | valid % | random_walk |"
    )
    print("|---:|---:|---:|---:|---:|:---:|")
    row_by_true = {r["true"]: r for r in rows}
    for hz in REPORT_CFOS:
        r = row_by_true.get(hz)
        if not r:
            print(f"| {hz} | (no samples) | — | — | — | — |")
            continue
        yn = "Y" if r["random_walk"] else "N"
        print(
            f"| {r['true']} | {r['mean_est']:.4g} | {r['std_est']:.4g} | "
            f"{r['mean_abs_err']:.4g} | {r['valid_pct']:.2f} | {yn} |"
        )

    print("\n### Decision helpers\n")
    print(
        f"- **Smallest true CFO with random_walk=N (stable, literal spec)**: "
        f"{deadzone_cfo} Hz"
        if deadzone_cfo is not None
        else "- **No stable CFO in sweep (all Y)**"
    )
    print(
        f"- **First ascending true CFO with random_walk=Y (pathological)**: "
        f"{first_unstable} Hz"
        if first_unstable is not None
        else "- **None**"
    )
    print(
        "- **Note**: Some CFO rows show `n=200` because two holo `Estimate` sites "
        "can each emit one `[V5A-S5H-TRIAL]` line per trial (same `true=`)."
    )
    print(
        "- **Metrics used for random_walk**: "
        "true=0 → |est mean|>100 OR est std>200; "
        "true>0 → mean|est-true|>0.5·true OR std(est)>0.5·true OR valid%<70."
    )

    # Sample lines: first 5 est per requested CFO
    print("\n### Sample lines (first 5 parsed samples per requested CFO)\n")
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        lines = f.readlines()
    for hz in REPORT_CFOS:
        got = []
        for line in lines:
            m = LINE_RE.search(line)
            if m and int(m.group(1)) == hz:
                got.append(line.strip())
                if len(got) >= 5:
                    break
        print(f"CFO={hz} Hz:")
        if not got:
            print("  (none)")
        else:
            for g in got:
                print(f"  {g}")
        print()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
