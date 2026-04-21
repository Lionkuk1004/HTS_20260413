# Read-only: list header basenames under HTS_LIM tree with no #include match in scanned sources.
import os, re, pathlib
root = pathlib.Path(r"D:/HTS_ARM11_Firmware/HTS_LIM")
out_dir = root / "pipeline_audit_raw"
headers = []
for dp, _, fns in os.walk(root):
    for fn in fns:
        if fn.endswith((".hpp", ".h")):
            headers.append(pathlib.Path(dp) / fn)

cpp_files = []
for sub in [root / "HTS_LIM", root / "HTS_TEST", root / "HTS_Jammer_STD"]:
    if sub.is_dir():
        for dp, _, fns in os.walk(sub):
            for fn in fns:
                if fn.endswith((".cpp", ".c", ".hpp", ".h")):
                    cpp_files.append(pathlib.Path(dp) / fn)

orphan_headers = []
for hp in sorted(headers, key=lambda p: p.name.lower()):
    h = hp.name
    pattern = re.compile(r"#include\s*[\"<][^\">]*" + re.escape(h) + r"[\">]")
    found = False
    for cpp in cpp_files:
        try:
            txt = cpp.read_text(encoding="utf-8", errors="replace")
        except Exception:
            try:
                txt = cpp.read_text(encoding="cp949", errors="replace")
            except Exception:
                continue
        if pattern.search(txt):
            found = True
            break
    if not found:
        orphan_headers.append(str(hp))

out = out_dir / "audit_orphan_headers.txt"
out.write_text(
    "=== ORPHAN HEADER PATHS (%d) ===\n" % len(orphan_headers)
    + "\n".join(orphan_headers)
    + "\n",
    encoding="utf-8",
)
print(len(headers), len(orphan_headers))
