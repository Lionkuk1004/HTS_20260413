#!/usr/bin/env python3
"""Step C-1: delegate to analyze_s3_v5.ps1 (Windows-friendly)."""
import subprocess
import sys


def main() -> int:
    return subprocess.call(
        ["powershell", "-NoProfile", "-File", "analyze_s3_v5.ps1"],
        cwd=__import__("pathlib").Path(__file__).resolve().parent,
    )


if __name__ == "__main__":
    raise SystemExit(main())
