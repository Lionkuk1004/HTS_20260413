#!/usr/bin/env python3
"""Compare T6 baseline vs HTS_V5A_DISABLE_LR logs (S5 / S5F CFO rows + grand total)."""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path
from typing import Dict, List, Tuple


def read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="replace")


def parse_grand_total(text: str) -> Tuple[int, int] | None:
    m = re.search(r"\uc815\ub7c9\s*\ud569\uacc4:\s*(\d+)\s*/\s*(\d+)", text)
    if not m:
        return None
    return int(m.group(1)), int(m.group(2))


def _hdr_s5(line: str) -> bool:
    pipe = "\u2502"
    return re.search(pipe + r"\s*S5\s+", line) is not None and re.search(
        pipe + r"\s*S5F\s+", line) is None


def _hdr_s5f(line: str) -> bool:
    return re.search("\u2502" + r"\s*S5F\s+", line) is not None


def _hdr_s6(line: str) -> bool:
    return re.search("\u2502" + r"\s*S6\s+", line) is not None


def _first_idx(lines: List[str], pred, start: int = 0) -> int:
    for i in range(start, len(lines)):
        if pred(lines[i]):
            return i
    return -1


def parse_cfo_rows_for_sections(text: str) -> Tuple[Dict[int, Tuple[int, int]], Dict[int, Tuple[int, int]]]:
    s5: Dict[int, Tuple[int, int]] = {}
    s5f: Dict[int, Tuple[int, int]] = {}
    lines = text.splitlines()
    row_re = re.compile(r"^\s+CFO\s+([+-]?\d+)\s*Hz\s+(\d+)/(\d+)\s")

    i5 = _first_idx(lines, _hdr_s5)
    i5f = _first_idx(lines, _hdr_s5f, start=i5 + 1 if i5 >= 0 else 0)
    i6 = _first_idx(lines, _hdr_s6, start=i5f + 1 if i5f >= 0 else 0)

    if i5 >= 0 and i5f > i5:
        for line in lines[i5 + 1: i5f]:
            m = row_re.match(line)
            if m:
                s5[int(m.group(1))] = (int(m.group(2)), int(m.group(3)))

    if i5f >= 0 and i6 > i5f:
        for line in lines[i5f + 1: i6]:
            m = row_re.match(line)
            if m:
                s5f[int(m.group(1))] = (int(m.group(2)), int(m.group(3)))

    return s5, s5f


def fmt_cell(t: Tuple[int, int] | None) -> str:
    if t is None:
        return "(n/a)"
    return "%d/%d" % (t[0], t[1])


def fmt_delta(base: Tuple[int, int] | None, cur: Tuple[int, int] | None) -> str:
    if base is None or cur is None:
        return "?"
    dp = cur[0] - base[0]
    if dp == 0:
        return "0"
    return "%+d" % dp


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("baseline", type=Path)
    ap.add_argument("nolr", type=Path)
    ap.add_argument("-o", "--output", type=Path, default=None)
    args = ap.parse_args()

    tb = read_text(args.baseline)
    tn = read_text(args.nolr)

    gb = parse_grand_total(tb)
    gn = parse_grand_total(tn)
    s5b, s5fb = parse_cfo_rows_for_sections(tb)
    s5n, s5fn = parse_cfo_rows_for_sections(tn)

    want_s5 = [0, 5000, 7500]
    want_s5f = [12500, 15000, 17500, 25000]

    lines_out: List[str] = []
    lines_out.append(
        "V5a L&R compare (baseline = LR ON, DISABLE_LR = HTS_V5A_DISABLE_LR)")
    lines_out.append("")
    lines_out.append("+-------------+--------------+--------------+--------+")
    lines_out.append("| Test        | baseline     | DISABLE_LR   | change |")
    lines_out.append("+-------------+--------------+--------------+--------+")
    lines_out.append("| T6 total    | %12s | %12s | %6s |" % (
        fmt_cell(gb), fmt_cell(gn), fmt_delta(gb, gn)))
    for hz in want_s5:
        cb = s5b.get(hz)
        cn = s5n.get(hz)
        label = "S5 %dHz" % hz
        lines_out.append("| %-11s | %12s | %12s | %6s |" % (
            label, fmt_cell(cb), fmt_cell(cn), fmt_delta(cb, cn)))
    for hz in want_s5f:
        cb = s5fb.get(hz)
        cn = s5fn.get(hz)
        label = "S5F %dHz" % hz
        lines_out.append("| %-11s | %12s | %12s | %6s |" % (
            label, fmt_cell(cb), fmt_cell(cn), fmt_delta(cb, cn)))
    lines_out.append("+-------------+--------------+--------------+--------+")
    lines_out.append("")
    lines_out.append("Criteria (manual):")
    lines_out.append("  - T6 total: regression < 50 trials vs baseline => safe")
    lines_out.append("  - S5 0..5000 Hz: maintain or improve vs baseline")
    lines_out.append("  - S5F high-CFO rows: any improvement => LR effect signal")

    text = "\n".join(lines_out) + "\n"
    sys.stdout.write(text)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8", newline="\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())