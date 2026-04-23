import re
import sys

path = sys.argv[1]
th = {
    "A": int(sys.argv[2]),
    "B": int(sys.argv[3]),
    "C": int(sys.argv[4]),
    "D": int(sys.argv[5]),
    "E": int(sys.argv[6]),
    "F": int(sys.argv[7]),
}
pat = re.compile(
    r"stage=CUBE pass=(\d+)\s+A=(-?\d+)\s+B=(-?\d+)\s+C=(-?\d+)\s+"
    r"D=(-?\d+)\s+E=(-?\d+)\s+F=(-?\d+)"
)
keys = "ABCDEF"


def open_log(p: str):
    with open(p, "rb") as bf:
        head = bf.read(4)
    if head.startswith(b"\xff\xfe") or head.startswith(b"\xfe\xff"):
        return open(p, encoding="utf-16", errors="replace")
    return open(p, encoding="utf-8", errors="replace")


ok = 0
n = 0
bottleneck = {k: 0 for k in keys}
with open_log(path) as f:
    for line in f:
        m = pat.search(line)
        if not m:
            continue
        n += 1
        vals = {keys[i]: int(m.group(i + 2)) for i in range(6)}
        passes = all(vals[k] >= th[k] for k in keys)
        if passes:
            ok += 1
        else:
            for k in keys:
                if vals[k] < th[k]:
                    bottleneck[k] += 1

print("thr", th)
print("n=", n, "pass_all=", ok, "rate", 100.0 * ok / n if n else 0)
print("bottleneck fail counts:", bottleneck)
