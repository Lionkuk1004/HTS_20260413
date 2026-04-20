#!/usr/bin/env python3
"""
Step C-1.5 off 분포: STAGE4-V5-2B 의 best_off 값별 빈도 + v5_score 관계.

참고: 펌웨어에서 skip 조건(best_off+128>192)에 걸리면 STAGE4 라인 자체가
안 찍힐 수 있어, cat_b 는 '추정 skip 계열'이 아니라 로그에 남는 패턴 분류다.
"""
import re
import statistics
import sys
from collections import Counter, defaultdict

LOG_PATH = "stepC1_4_t6_v5.log"
LOG_ENCODING = "utf-16"
OUT_TXT = "stepC1_5_off_distribution.txt"

STAGE4 = re.compile(
    r"\[STAGE4-V5-2B\]\s+off=(\d+)\s+v5_score=(\d+)\s+legacy_seed=\(([+-]?\d+),([+-]?\d+)\)"
)


def main() -> int:
    records: list[tuple[int, int, int, int]] = []
    with open(LOG_PATH, "r", encoding=LOG_ENCODING, errors="replace") as f:
        for line in f:
            m = STAGE4.search(line)
            if m:
                off = int(m.group(1))
                score = int(m.group(2))
                si = int(m.group(3))
                sq = int(m.group(4))
                records.append((off, score, si, sq))

    print(f"총 STAGE4 레코드: {len(records)}")
    if not records:
        print("ERROR: STAGE4 레코드 0건. 로그 인코딩(utf-16) 또는 경로 확인.", file=sys.stderr)
        return 1

    off_count = Counter(r[0] for r in records)
    print("\n=== off 값 분포 (상위 20개) ===")
    for off, cnt in off_count.most_common(20):
        pct = 100.0 * cnt / len(records) if records else 0.0
        print(f"  off={off:>3d}: {cnt:>6d} ({pct:5.2f}%)")

    cat_a: list[tuple[int, int, int, int]] = []
    cat_b: list[tuple[int, int, int, int]] = []
    cat_c: list[tuple[int, int, int, int]] = []
    cat_other: list[tuple[int, int, int, int]] = []

    for off, score, si, sq in records:
        if off == 0 and si == 0 and sq == 0 and score == 0:
            cat_a.append((off, score, si, sq))
        elif off > 0 and (si != 0 or sq != 0) and score == 0:
            cat_b.append((off, score, si, sq))
        elif score > 0:
            cat_c.append((off, score, si, sq))
        else:
            cat_other.append((off, score, si, sq))

    total = len(records)
    zero_total = len(cat_a) + len(cat_b)

    with open(OUT_TXT, "w", encoding="utf-8") as f:
        f.write("=" * 90 + "\n")
        f.write("Step C-1.5 STAGE4-V5-2B off 분포 + v5_score=0 원인 분석\n")
        f.write("=" * 90 + "\n\n")
        f.write(f"총 레코드: {total}\n\n")

        f.write("--- off 값 분포 ---\n")
        for off, cnt in sorted(off_count.items())[:30]:
            pct = 100.0 * cnt / total if total else 0.0
            bar = "█" * int(pct / 2)
            f.write(f"  off={off:>3d}: {cnt:>6d} ({pct:6.2f}%)  {bar}\n")

        f.write("\n--- 카테고리 분류 ---\n")
        f.write("  A. off=0, legacy=(0,0), score=0  (Phase 0 실패 or 미진입):\n")
        f.write(f"     개수: {len(cat_a):>6d} ({100.0 * len(cat_a) / total:5.2f}%)\n\n")

        f.write(
            "  B. off>0, legacy≠(0,0), score=0  (v5=0 이지만 legacy 존재; skip은 로그 미출력 가능):\n"
        )
        f.write(f"     개수: {len(cat_b):>6d} ({100.0 * len(cat_b) / total:5.2f}%)\n")
        if cat_b:
            off_b = [r[0] for r in cat_b]
            f.write(
                f"     off 범위: min={min(off_b)}, max={max(off_b)}, "
                f"mean={statistics.mean(off_b):.1f}\n"
            )
            over_64 = sum(1 for o in off_b if o > 64)
            f.write(f"     off>64 개수: {over_64} ({100.0 * over_64 / len(cat_b):.1f}% of B)\n\n")

        f.write("  C. score>0 (정상 v5 계산 완료):\n")
        f.write(f"     개수: {len(cat_c):>6d} ({100.0 * len(cat_c) / total:5.2f}%)\n")
        if cat_c:
            off_c = [r[0] for r in cat_c]
            scores_c = [r[1] for r in cat_c]
            f.write(
                f"     off 범위: min={min(off_c)}, max={max(off_c)}, "
                f"mean={statistics.mean(off_c):.1f}\n"
            )
            f.write(
                f"     score 범위: min={min(scores_c):.2e}, max={max(scores_c):.2e}, "
                f"mean={statistics.mean(scores_c):.2e}\n\n"
            )

        if cat_other:
            f.write(f"  D. 기타 (분류 불가): {len(cat_other)}\n")
            for r in cat_other[:20]:
                f.write(f"     off={r[0]}, score={r[1]}, legacy=({r[2]},{r[3]})\n")

        f.write("\n--- v5_score=0 원인 종합 (A+B 대비) ---\n")
        if zero_total > 0:
            f.write(
                f"  v5_score=0 중 Phase 0 실패 (A): {len(cat_a)} "
                f"({100.0 * len(cat_a) / zero_total:.1f}%)\n"
            )
            f.write(
                f"  v5_score=0 중 legacy≠0 패턴 (B):   {len(cat_b)} "
                f"({100.0 * len(cat_b) / zero_total:.1f}%)\n"
            )
        else:
            f.write("  v5_score=0 in A+B: 0\n")

    print(f"\n완료: {OUT_TXT}")
    print(f"카테고리 A (Phase 0 실패): {len(cat_a)}")
    print(f"카테고리 B (legacy≠0, score=0):   {len(cat_b)}")
    print(f"카테고리 C (정상):         {len(cat_c)}")
    print(f"기타:                      {len(cat_other)}")
    if len(cat_a) + len(cat_b) + len(cat_c) + len(cat_other) != total:
        print("ERROR: 카테고리 합이 total 과 불일치", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
