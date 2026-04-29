# -*- coding: utf-8 -*-
"""Emit UTF-8 CRLF cmd helpers for CFO Bank v4 lab (Cursor/Write may emit UTF-16; run this after edits)."""
import os

ROOT = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(ROOT, "cursor_lab_v4_batch6_13_bug_random_unified.cmd")
RUN = os.path.join(ROOT, "cursor_lab_v4_run_cfo_sweep_20260428.cmd")
DIAG = os.path.normpath(os.path.join(ROOT, "..", "diag_results", "cfo_sweep_20260428"))

LINES = [
    "@echo off",
    "REM Build CFO Bank v4 exes: BATCH 6..13, BUG_FOCUS, RANDOM, UNIFIED (same TU as cursor_lab_v4_batch1.cmd).",
    "REM Double-click: pause at end. Automation: ...cmd nopause",
    "setlocal",
    'call "C:\\Program Files\\Microsoft Visual Studio\\18\\Community\\VC\\Auxiliary\\Build\\vcvars64.bat" || (',
    "  echo [ERROR] vcvars64.bat failed.",
    "  pause",
    "  exit /b 1",
    ")",
    "cd /d \"%~dp0\"",
    "if not exist \"HTS_CFO_Bank_Test_v4.cpp\" (",
    "  echo [ERROR] HTS_CFO_Bank_Test_v4.cpp not found in \"%CD%\"",
    "  pause",
    "  exit /b 1",
    ")",
    "set \"HTS_LIM=%~dp0..\\HTS_LIM\"",
    'set "CLBASE=/nologo /O2 /std:c++20 /EHsc /MD /W3 /DNDEBUG /D_CRT_SECURE_NO_WARNINGS /DHTS_USE_HOLO_TENSOR_4D /DHTS_ALLOW_HOST_BUILD /DHTS_CFO_V5A_ENABLE=1 /DNOGDI /I"%HTS_LIM%""',
    "",
]

SRC = [
    "HTS_CFO_Bank_Test_v4.cpp",
    '"%HTS_LIM%\\HTS_CFO_V5a.cpp"',
    '"%HTS_LIM%\\HTS_Rx_CFO_SinCos_Table.cpp"',
    '"%HTS_LIM%\\HTS_Holo_Tensor_4D_Common.cpp"',
    '"%HTS_LIM%\\HTS_Holo_Tensor_4D_TX.cpp"',
    '"%HTS_LIM%\\HTS_Holo_Tensor_4D_RX.cpp"',
    '"%HTS_LIM%\\HTS_Secure_Memory.cpp"',
    '"%HTS_LIM%\\HTS_Preamble_Holographic.cpp"',
]

BUILDS = [
    ("/DHTS_BATCH_6", "HTS_CFO_Bank_Test_v4_b6.exe"),
    ("/DHTS_BATCH_7", "HTS_CFO_Bank_Test_v4_b7.exe"),
    ("/DHTS_BATCH_8", "HTS_CFO_Bank_Test_v4_b8.exe"),
    ("/DHTS_BATCH_9", "HTS_CFO_Bank_Test_v4_b9.exe"),
    ("/DHTS_BATCH_10", "HTS_CFO_Bank_Test_v4_b10.exe"),
    ("/DHTS_BATCH_11", "HTS_CFO_Bank_Test_v4_b11.exe"),
    ("/DHTS_BATCH_12", "HTS_CFO_Bank_Test_v4_b12.exe"),
    ("/DHTS_BATCH_13", "HTS_CFO_Bank_Test_v4_b13.exe"),
    ("/DHTS_BATCH_BUG_FOCUS", "HTS_CFO_Bank_Test_v4_bug.exe"),
    ("/DHTS_BATCH_RANDOM", "HTS_CFO_Bank_Test_v4_random.exe"),
    ("/DHTS_BATCH_UNIFIED", "HTS_CFO_Bank_Test_v4_unified.exe"),
]

for dflag, exe in BUILDS:
    LINES.append("cl %CLBASE% {} /Fe:{} ^".format(dflag, exe))
    for s in SRC:
        LINES.append("   {} ^".format(s))
    LINES.append("   /link /nologo")
    LINES.append("if errorlevel 1 (")
    LINES.append("  echo BUILD FAILED (see messages above)")
    LINES.append("  pause")
    LINES.append("  exit /b 1")
    LINES.append(")")
    LINES.append("")

LINES.extend(
    [
        "echo OK: b6..b13, bug, random, unified built in %CD%",
        "set RC=0",
        "if /i not \"%~1\"==\"nopause\" pause",
        "exit /b %RC%",
        "",
    ]
)

text = "\r\n".join(LINES)
with open(OUT, "wb") as f:
    f.write(text.encode("utf-8"))
print("Wrote", OUT, "bytes", len(text.encode("utf-8")))

RUN_LINES = [
    "@echo off",
    "REM Run CFO Bank v4 sweeps (b1..b13, bug, random, unified) into diag_results\\cfo_sweep_20260428",
    "REM Prereq: build b1..b5 via cursor_lab_v4_batch1.cmd .. batch5.cmd; b6+ via cursor_lab_v4_batch6_13_bug_random_unified.cmd",
    "REM Double-click: pause at end. Automation: ...cmd nopause",
    "setlocal",
    "cd /d \"%~dp0\"",
    "set \"OUT=%~dp0..\\diag_results\\cfo_sweep_20260428\"",
    "mkdir \"%OUT%\" 2>nul",
    "for %%E in (1 2 3 4 5 6 7 8 9 10 11 12 13) do (",
    "  echo Running batch %%E ...",
    "  \"%~dp0HTS_CFO_Bank_Test_v4_b%%E.exe\" > \"%OUT%\\batch_%%E_2026-04-28.txt\" 2>&1",
    ")",
    "echo Running BUG_FOCUS ...",
    "\"%~dp0HTS_CFO_Bank_Test_v4_bug.exe\" > \"%OUT%\\batch_bug_2026-04-28.txt\" 2>&1",
    "echo Running RANDOM ...",
    "\"%~dp0HTS_CFO_Bank_Test_v4_random.exe\" > \"%OUT%\\batch_random_2026-04-28.txt\" 2>&1",
    "echo Running UNIFIED (long) ...",
    "\"%~dp0HTS_CFO_Bank_Test_v4_unified.exe\" > \"%OUT%\\batch_unified_2026-04-28.txt\" 2>&1",
    "echo Done. Logs in \"%OUT%\"",
    "set RC=0",
    "if /i not \"%~1\"==\"nopause\" pause",
    "exit /b %RC%",
    "",
]
run_text = "\r\n".join(RUN_LINES)
with open(RUN, "wb") as f:
    f.write(run_text.encode("utf-8"))
print("Wrote", RUN, "bytes", len(run_text.encode("utf-8")))

os.makedirs(DIAG, exist_ok=True)
SUMMARY = os.path.join(DIAG, "summary_2026-04-28.txt")
summary_text = "\r\n".join(
    [
        "=== CFO Bank Test v4 통합 분석 (2026-04-28) ===",
        "",
        "[1] 정규 grid 결과 (BATCH 1~13)",
        "  0~10000Hz @50Hz : 200점 (배치 1~10, 경계 중복)",
        "  10000~25000Hz @100Hz : 151점 (배치 11~13)",
        "",
        "[2] BUG 영역 정밀 (BUG_FOCUS)",
        "  7500/12500/15000/17500/25000Hz +-500Hz @50Hz : 105점",
        "",
        "[3] 무작위 grid 회피 (RANDOM)",
        "  1000~25000Hz 비정규 offset : 약 106점",
        "",
        "=== 핵심 비교 ===",
        "",
        "(a) 정규 grid PASS rate @ 5000Hz 배수",
        "(b) 정규 grid PASS rate @ 2500Hz shift",
        "(c) 무작위 grid PASS rate",
        "",
        "만약 (a) >> (c): 알고리즘이 정규 grid 우연 fit",
        "만약 (a) ~ (c): 진짜 알고리즘 한계 = (a) 가 진실",
        "",
        "[A] fact-tier: 로그 원문 수치",
        "[B] fact-tier: 패턴 추정",
        "[C] fact-tier: 추가 분석 필요",
        "",
        "Note: 이 파일은 템플릿이다. batch_* 로그를 모은 뒤 sync/dec/BER 집계를 채운다.",
        "",
    ]
)
with open(SUMMARY, "wb") as f:
    f.write(summary_text.encode("utf-8"))
print("Wrote", SUMMARY)

PAT = os.path.join(DIAG, "pattern_analysis_2026-04-28.txt")
pat_text = "\r\n".join(
    [
        "=== CFO sweep pattern analysis (2026-04-28) ===",
        "",
        "1. 5000Hz 배수 PASS/FAIL 경계 (batch 로그에서 sync/dec/BER)",
        "2. 2500Hz shift alias 경계 (7000~8000 batch_8 등)",
        "3. RANDOM vs 정규 grid PASS% (구간별: 1000~5000, 5000~10000, 10000~25000)",
        "4. SNR 30dB 한계 — 필요 시 SNR 스윕 별도 TU",
        "",
        "정규 vs 무작위 비교 표(수동/스크립트 채움):",
        "| 영역          | 정규 grid PASS% | 무작위 grid PASS% | 차이 |",
        "|--------------:|----------------:|------------------:|-----:|",
        "| 1000~5000Hz   | ?               | ?                 | ?    |",
        "| 5000~10000Hz  | ?               | ?                 | ?    |",
        "| 10000~25000Hz | ?               | ?                 | ?    |",
        "",
    ]
)
with open(PAT, "wb") as f:
    f.write(pat_text.encode("utf-8"))
print("Wrote", PAT)
