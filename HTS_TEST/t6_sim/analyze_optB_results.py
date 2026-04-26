# -*- coding: utf-8 -*-
"""
optB_matrix 결과 자동 비교 + SUMMARY.md 생성.

입력: out_lab/optB_matrix/result_{A,B,C}.log (run_optB_matrix.cmd 가 생성)
출력: SUMMARY.md, scores_{A,B,C}.txt

T6 종합 보고서 테이블 형식(HTS_T6_SIM_Test.cpp):
  ║ name │ param │ pass │ crc_only │ ber │ tag ║
"""
from __future__ import annotations

import os
import re
import sys

SEP = "\u2502"  # │
BOX_L = "\u2551"  # ║


def _script_dir() -> str:
    return os.path.dirname(os.path.abspath(__file__))


def log_dir() -> str:
    return os.path.join(_script_dir(), "out_lab", "optB_matrix")


def extract_summary_block(text: str) -> str:
    if "종합 보고서" not in text:
        return ""
    i = text.index("종합 보고서")
    j = text.find("\u255a", i)  # ╚
    if j < 0:
        return text[i:]
    return text[i:j]


def parse_table_rows(text: str) -> dict[str, tuple[int, int, str]]:
    """
    key: 'S1|...' or 'S5H|200Hz'
    value: (pass_count, crc_only, tag)  — total trials는 kTrials=100 고정 가정
    """
    block = extract_summary_block(text)
    if not block.strip():
        block = text
    rows: dict[str, tuple[int, int, str]] = {}
    for line in block.splitlines():
        if BOX_L not in line or SEP not in line:
            continue
        if "시나리오" in line and "조건" in line:
            continue
        if line.strip().startswith("\u2560") or line.strip().startswith("\u2554"):
            continue
        parts = line.split(SEP)
        if len(parts) < 6:
            continue
        raw0 = parts[0].strip().replace(BOX_L, "").replace("\u2551", "").strip()
        name = raw0.split()[0] if raw0 else ""
        if not name or name.startswith("═") or name in ("시나리오", "정량"):
            continue
        if not re.match(r"^[A-Za-z0-9]+$", name):
            continue
        param = parts[1].strip()
        try:
            p0 = int(parts[2].strip())
            p1 = int(parts[3].strip())
        except ValueError:
            continue
        tail_raw = parts[5].replace(BOX_L, "").replace("\u2551", "").strip()
        tag_parts = tail_raw.split()
        tag = tag_parts[0] if tag_parts else ""
        key = f"{name}|{param}"
        rows[key] = (p0, p1, tag)
    return rows


def fmt_pass(v: tuple[int, int, str] | None) -> str:
    if v is None:
        return "?"
    p, _, tag = v
    return f"{p}/100 ({tag.strip()})"


def pass_score(v: tuple[int, int, str] | None) -> int | None:
    if v is None:
        return None
    return v[0]


def main() -> int:
    base = log_dir()
    os.makedirs(base, exist_ok=True)

    def load(tag: str) -> tuple[str, dict[str, tuple[int, int, str]]]:
        path = os.path.join(base, f"result_{tag}.log")
        if not os.path.isfile(path):
            return "", {}
        with open(path, "r", encoding="utf-8", errors="replace") as f:
            raw = f.read()
        snippet = extract_summary_block(raw)
        scores_path = os.path.join(base, f"scores_{tag}.txt")
        with open(scores_path, "w", encoding="utf-8") as out:
            out.write(f"--- {tag} : summary excerpt ---\n")
            out.write(snippet if snippet else "(no summary block found)\n")
        return raw, parse_table_rows(raw)

    _, ra = load("A")
    _, rb = load("B")
    _, rc = load("C")

    # 회귀 게이트용 행 (T6 S5/S5H CFO 목록: 최고 25000Hz — 30000Hz 행 없음)
    focus_keys = [
        "S1",
        "S5|200Hz",
        "S5H|200Hz",
        "S5H|500Hz",
        "S5H|2000Hz",
        "S5H|25000Hz",
    ]

    def find_s1(rows: dict[str, tuple[int, int, str]]) -> tuple[int, int, str] | None:
        for k, v in rows.items():
            if k.startswith("S1|"):
                return v
        return None

    def get_row(rows: dict[str, tuple[int, int, str]], compound: str) -> tuple[int, int, str] | None:
        if compound == "S1":
            return find_s1(rows)
        return rows.get(compound)

    lines: list[str] = []
    lines.append("# Phase B 옵션 B — 자동 매트릭스 결과\n")
    lines.append("")
    lines.append("| 시나리오 | Build A (baseline) | Build B (REF only) | Build C (REF+APPLY) | 비고 |")
    lines.append("|----------|--------------------|--------------------|---------------------|------|")

    notes_b: list[str] = []

    for compound in focus_keys:
        label = compound.replace("|", " ")
        va = get_row(ra, compound)
        vb = get_row(rb, compound)
        vc = get_row(rc, compound)
        lines.append(
            f"| {label} | {fmt_pass(va)} | {fmt_pass(vb)} | {fmt_pass(vc)} | |"
        )

        pa, pb, pc = pass_score(va), pass_score(vb), pass_score(vc)
        if pa is not None and pb is not None and pb != pa:
            notes_b.append(f"- **Step1 회귀**: `{label}` A={pa} B={pb}")

    lines.append("")
    lines.append("## 자동 판정\n")

    s1_a = pass_score(find_s1(ra))
    s1_b = pass_score(find_s1(rb))
    s1_c = pass_score(find_s1(rc))
    h200_a = pass_score(get_row(ra, "S5H|200Hz"))
    h200_b = pass_score(get_row(rb, "S5H|200Hz"))
    h200_c = pass_score(get_row(rc, "S5H|200Hz"))
    h500_a = pass_score(get_row(ra, "S5H|500Hz"))
    h500_c = pass_score(get_row(rc, "S5H|500Hz"))

    if s1_a is not None and s1_b is not None and s1_b != s1_a:
        lines.append("- **Step1 (B vs A)**: S1 점수 차이 → REF 추출 경로가 baseline과 어긋남 가능.")
    else:
        lines.append("- **Step1 (B vs A)**: S1 동일 또는 로그 누락.")

    gate_fail = (
        (s1_a is not None and s1_a == 100 and s1_c is not None and s1_c < 100)
        or (
            h200_a is not None
            and h200_a == 100
            and h200_c is not None
            and h200_c < 100
        )
    )
    if gate_fail:
        lines.append("- **회귀 게이트 ★**: Build C 에서 S1 또는 S5H 200Hz 가 baseline 100 미만.")
    else:
        lines.append("- **회귀 게이트 (C vs A)**: S1 / S5H 200Hz 유지 (또는 로그 없음).")

    if (
        h500_a is not None
        and h500_c is not None
        and isinstance(h500_a, int)
        and isinstance(h500_c, int)
        and h500_c > h500_a
    ):
        lines.append("- **S5H 500Hz**: C 가 A 보다 개선 (회복 신호).")

    if notes_b:
        lines.extend(notes_b)

    lines.append("")
    lines.append("## 로그\n")
    lines.append(f"- `{os.path.join(base, 'result_A.log')}`")
    lines.append(f"- `{os.path.join(base, 'result_B.log')}`")
    lines.append(f"- `{os.path.join(base, 'result_C.log')}`")
    lines.append("")
    lines.append("※ 현재 T6 `test_S5_holographic` CFO 상한은 **25000Hz** (30000Hz 행 없음).")

    out_path = os.path.join(base, "SUMMARY.md")
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))

    print("\n".join(lines))
    print(f"\nWrote: {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
