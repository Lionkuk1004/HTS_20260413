import pathlib, re
root = pathlib.Path(r"D:/HTS_ARM11_Firmware/HTS_LIM/pipeline_audit_raw")
def read_lines(p):
    try:
        return pathlib.Path(p).read_text(encoding="utf-8", errors="replace").splitlines()
    except Exception as e:
        return [f"<<read_error {p}: {e}>>"]

src_paths = []
for name in ["audit_cpp_hts_lim_hts_lim_only.txt", "audit_cpp_test_paths.txt"]:
    src_paths.extend(read_lines(root / name))
srcs = set()
for line in src_paths:
    line = line.strip()
    if not line: continue
    if line.startswith("<<"): continue
    p = pathlib.Path(line)
    if p.suffix.lower() == ".cpp":
        srcs.add(p.name)

refs = ""
for name in ["audit_bat_sources.txt", "audit_vcxproj_sources.txt"]:
    refs += pathlib.Path(root / name).read_text(encoding="utf-8", errors="replace")

orphans = [s for s in sorted(srcs) if s not in refs]
out = root / "audit_orphans.txt"
with out.open("w", encoding="utf-8") as f:
    f.write("=== ORPHAN CANDIDATES (%d files) ===\n" % len(orphans))
    for o in orphans:
        f.write("  " + o + "\n")
print("orphans", len(orphans), "src_basenames", len(srcs))
