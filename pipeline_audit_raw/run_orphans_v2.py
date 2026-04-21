import pathlib
root = pathlib.Path(r"D:/HTS_ARM11_Firmware/HTS_LIM/pipeline_audit_raw")
def read(p):
    return pathlib.Path(p).read_text(encoding="utf-8", errors="replace")

src_paths = []
for name in ["audit_cpp_hts_lim_hts_lim_only.txt", "audit_cpp_test_paths.txt"]:
    src_paths.extend(read(root / name).splitlines())
srcs = set()
for line in src_paths:
    line = line.strip()
    if not line or line.startswith("<<"): continue
    p = pathlib.Path(line)
    if p.suffix.lower() == ".cpp":
        srcs.add(p.name)

refs = read(root / "audit_build_refs_full.txt")
refs += read(root / "audit_vcxproj_sources.txt")

orphans = [s for s in sorted(srcs) if s not in refs]
out = root / "audit_orphans_v2.txt"
out.write_text("=== ORPHAN CANDIDATES (%d files) ===\n" % len(orphans) + "".join("  %s\n" % o for o in orphans), encoding="utf-8")
print(len(orphans), len(srcs))
