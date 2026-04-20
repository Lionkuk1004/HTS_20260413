#!/usr/bin/env python3
"""S3 구간만 대상으로 한 off 값 분포 + 카테고리 분석."""
import re
import statistics
import sys
from collections import Counter

LOG_PATH = "stepC1_4_t6_v5.log"
LOG_ENCODING = "utf-16"
OUT_TXT = "stepC1_5_s3_off_distribution.txt"
S3_START_LINE = 47037
S3_END_LINE = 131311

STAGE4 = re.compile(
    r"\[STAGE4-V5-2B\]\s+off=(\d+)\s+v5_score=(\d+)\s+legacy_seed=\(([+-]?\d+),([+-]?\d+)\)"
)


def main() -> int:
    records: list[tuple[int, int, int, int]] = []
    with open(LOG_PATH, "r", encoding=LOG_ENCODING, errors="replace") as f:
        for i, line in enumerate(f, 1):
            if i < S3_START_LINE or i >= S3_END_LINE:
                continue
            m = STAGE4.search(line)
            if m:
                records.append(
                    (
                        int(m.group(1)),
                        int(m.group(2)),
                        int(m.group(3)),
                        int(m.group(4)),
                    )
                )

    total = len(records)
    print(f"S3 구간 STAGE4 레코드: {total}")

    cat_a = [r for r in records if r[0] == 0 and r[2] == 0 and r[3] == 0 and r[1] == 0]
    cat_b = [r for r in records if r[0] > 0 and (r[2] != 0 or r[3] != 0) and r[1] == 0]
    cat_c = [r for r in records if r[1] > 0]
    cat_other = [r for r in records if r not in cat_a and r not in cat_b and r not in cat_c]

    off_count = Counter(r[0] for r in records)

    with open(OUT_TXT, "w", encoding="utf-8") as f:
        f.write("=" * 90 + "\n")
        f.write(f"Step C-1.5 S3 구간 (lines {S3_START_LINE}~{S3_END_LINE}) off 분포\n")
        f.write("=" * 90 + "\n\n")
        f.write(f"S3 STAGE4 레코드: {total}\n\n")
        pct = lambda n: 100 * n / total if total else 0.0
        f.write(f"A. off=0/legacy=0/score=0 (P0 실패): {len(cat_a)} ({pct(len(cat_a)):.1f}%)\n")
        f.write(f"B. off>0/legacy≠0/score=0:           {len(cat_b)} ({pct(len(cat_b)):.1f}%)\n")
        f.write(f"C. score>0 (정상):                  {len(cat_c)} ({pct(len(cat_c)):.1f}%)\n")
        f.write(f"D. 기타:                             {len(cat_other)}\n\n")

        f.write("--- off 값 분포 (S3) ---\n")
        for off, cnt in sorted(off_count.items())[:30]:
            p = 100.0 * cnt / total if total else 0.0
            f.write(f"  off={off:>3d}: {cnt:>6d} ({p:6.2f}%)\n")

        if cat_b:
            off_b = [r[0] for r in cat_b]
            f.write(
                f"\nB off: min={min(off_b)}, max={max(off_b)}, mean={statistics.mean(off_b):.1f}\n"
            )

    print(f"완료: {OUT_TXT}")
    if len(cat_a) + len(cat_b) + len(cat_c) + len(cat_other) != total:
        print("ERROR: 카테고리 합 불일치", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
