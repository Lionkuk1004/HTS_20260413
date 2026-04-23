"""Phase 3.N: Parse t6_step3n.log (S1-TRIAL + CMYK-DIAG-CONTRACT-PSLTE)."""
from __future__ import annotations

import re
import statistics
import sys
from pathlib import Path

S1_RE = re.compile(
    r"\[S1-TRIAL\] t=(\d+) pass=(\d+) crc=(\d+) len_ok=(\d+) be=(\d+) grav_any=(\d+)"
)
CONTRACT_RE = re.compile(
    r"\[CMYK-DIAG-CONTRACT-PSLTE\] best_off=(-?\d+) chip_start=(-?\d+) "
    r"psal_off=(-?\d+) psal_e63=(-?\d+) est_I=(-?\d+) est_Q=(-?\d+) agc_sh=(-?\d+)"
)


def main() -> None:
    path = Path(__file__).with_name("t6_step3n.log")
    if len(sys.argv) > 1:
        path = Path(sys.argv[1])
    if not path.is_file():
        print("missing:", path)
        sys.exit(1)

    s1_pass = s1_fail = 0
    grav_any_on_fail = 0
    grav_any_on_pass = 0

    psal_e63s: list[int] = []
    est_is: list[int] = []
    est_qs: list[int] = []
    agc_shs: list[int] = []
    best_offs: list[int] = []

    with path.open(encoding="utf-8", errors="replace") as f:
        for line in f:
            m = S1_RE.search(line)
            if m:
                t, p, crc, lok, be, g = map(int, m.groups())
                if p:
                    s1_pass += 1
                    if g:
                        grav_any_on_pass += 1
                else:
                    s1_fail += 1
                    if g:
                        grav_any_on_fail += 1
                continue
            m = CONTRACT_RE.search(line)
            if m:
                bo, cs, po, e63, eI, eQ, ag = map(int, m.groups())
                psal_e63s.append(e63)
                est_is.append(eI)
                est_qs.append(eQ)
                agc_shs.append(ag)
                best_offs.append(bo)

    print("=== S1 (from [S1-TRIAL] lines) ===")
    print("pass=1:", s1_pass, " pass=0:", s1_fail)
    if s1_fail:
        print(
            "  among pass=0: grav_any=1 count:",
            grav_any_on_fail,
            f"({100.0 * grav_any_on_fail / s1_fail:.1f}%)",
        )
    if s1_pass:
        print(
            "  among pass=1: grav_any=1 count:",
            grav_any_on_pass,
            f"({100.0 * grav_any_on_pass / s1_pass:.1f}%)",
        )

    print("\n=== CMYK-DIAG-CONTRACT-PSLTE (all lines in log) ===")
    print("contract lines:", len(psal_e63s))
    if not psal_e63s:
        return

    def med(xs: list[int]) -> float:
        return float(statistics.median(xs))

    print("psal_e63 median:", med(psal_e63s), " min/max:", min(psal_e63s), max(psal_e63s))
    print("est_I median:", med(est_is), " min/max:", min(est_is), max(est_is))
    print("est_Q median:", med(est_qs), " min/max:", min(est_qs), max(est_qs))
    print("agc_sh median:", med(agc_shs), " unique:", sorted(set(agc_shs)))
    print(
        "best_off median:",
        med(best_offs),
        " min/median/max:",
        min(best_offs),
        int(med(best_offs)),
        max(best_offs),
    )


if __name__ == "__main__":
    main()
