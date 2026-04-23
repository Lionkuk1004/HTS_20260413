"""Parse HTS CMYK DIAG lines: stage=CUBE pass=... A=... B=... ..."""
import re
import sys

pat = re.compile(
    r"stage=CUBE pass=(\d+)\s+A=(-?\d+)\s+B=(-?\d+)\s+C=(-?\d+)\s+"
    r"D=(-?\d+)\s+E=(-?\d+)\s+F=(-?\d+)"
)
faces = {"A": [], "B": [], "C": [], "D": [], "E": [], "F": []}
pass_cnt = 0
fail_cnt = 0
n_lines = 0

path = sys.argv[1] if len(sys.argv) > 1 else "t6_cmyk_diag_full.log"


def open_log(p: str):
    with open(p, "rb") as bf:
        head = bf.read(4)
    if head.startswith(b"\xff\xfe") or head.startswith(b"\xfe\xff"):
        return open(p, encoding="utf-16", errors="replace")
    return open(p, encoding="utf-8", errors="replace")


with open_log(path) as f:
    for line in f:
        m = pat.search(line)
        if not m:
            continue
        n_lines += 1
        p = int(m.group(1))
        if p:
            pass_cnt += 1
        else:
            fail_cnt += 1
        keys = ["A", "B", "C", "D", "E", "F"]
        for i, k in enumerate(keys):
            faces[k].append(int(m.group(i + 2)))

print(f"stage=CUBE lines matched: {n_lines}")
print(f"PASS: {pass_cnt}, FAIL: {fail_cnt}")
thresholds = {
    "A": 512,
    "B": 409,
    "C": 409,
    "D": 10240,
    "E": 614,
    "F": 15360,
}
for k, vs in faces.items():
    if not vs:
        continue
    vs_sorted = sorted(vs)
    n = len(vs_sorted)
    p50 = vs_sorted[n // 2]
    p10 = vs_sorted[max(0, n // 10)]
    p90 = vs_sorted[min(n - 1, (9 * n) // 10)]
    thr = thresholds[k]
    below = sum(1 for v in vs if v < thr)
    print(
        f"{k}: min={vs_sorted[0]:>7} p10={p10:>7} p50={p50:>7} p90={p90:>7} "
        f"max={vs_sorted[-1]:>7} (thr={thr}, below={below}/{n}="
        f"{100.0 * below / n:.1f}%)"
    )
