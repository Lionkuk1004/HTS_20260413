# PN-masked Preamble 양산 적용 — 최종 보고서 (Step 7-2)

`HTS_USE_PN_MASKED` (기본 **미정의**): Walsh 프리앰블을 디바이스 행 LUT로 치환하고, RX에서 기대 행과의 BPTE 검증 플래그·PaCD TX preamble 정합을 제공한다.

## 적용 범위

| 영역 | 내용 |
|------|------|
| 빌드 | `/DHTS_USE_PN_MASKED` |
| TX | `HTS_V400_Dispatcher_TX.cpp` — 첫 64칩 `walsh_enc(device_row, …)` |
| RX | `Sync_PSLTE.cpp` / `Sync_AMI.cpp` — `dominant_row_` vs `pn_masked_get_device_row()`, `pn_masked_row_verify_ok_` (동기 거부 없음) |
| PaCD | `HTS_V400_Dispatcher_Payload.cpp` — `HTS_USE_PACD` 시 TX preamble을 `GetPnMaskedPreambleI/Q(dev_row)` |
| LUT | `HTS_V400_Dispatcher_PNMasked.hpp` — 64×128칩 I-only ~16 KiB `.rodata` |

## 성능 (KCMVP / Phase0)

- PC(MSVC x64) `pn_masked_phase0_scan` 평균 **~127 ns/call** (입력 3종 분산 **< 5%**, Step 7-1).
- Coarse **32 offset** 가정 시 **~4 µs** (1 ms 예산 대비 여유).
- 상세: `HTS_V400_Dispatcher_PNMasked_KCMVP_TIMING.md`.

## 보안 (KCMVP)

- `pn_masked_phase0_scan`, `pn_masked_pte_subchip`, `pn_masked_device_id_to_row`, `pn_masked_phase0_subchip_refine`: Step 7-1 **constant-time PASS** (편차 < 5%).
- BPTE max / clamp 패턴, 비밀 의존 분기 없음 설계 유지.
- **LPI**: 행 공간 64 — 디바이스별 행으로 탐지 비용 구조적 증가 (설계 의도).

## 메모리 (#13)

| 항목 | 증가 |
|------|------|
| Flash | ~**16 KiB** (PN TX preamble LUT, I-only) |
| 디스패처 RAM (`HTS_USE_PN_MASKED`) | `pn_masked_best_row_` + `pn_masked_subchip_q14_` + `pn_masked_row_verify_ok_` (기존 `sizeof` static_assert 경로 유지) |

## T6 정량 매트릭스 (2026-04-27, MSVC x64 Release)

동일 `HTS_T6_SIM_Test.cpp` 유니티 빌드; Walsh 경로 **18400** 셀, Holo 경로 **20400** 셀 (S5-HOLO 블록 포함).

| 모드 | 스크립트 | 실행 파일 | Walsh (pass/total) | Holo (pass/total) |
|------|------------|-------------|--------------------|-------------------|
| Default | `cursor_t6_build.cmd` | `HTS_T6_SIM_Test.exe` | **12596 / 18400** | — |
| Default Holo | `cursor_t6_build_holo.cmd` | `HTS_T6_SIM_Test_holo.exe` | — | **12996 / 20400** |
| PaCD | `cursor_t6_build_pacd.cmd` | `HTS_T6_SIM_Test_pacd.exe` | **12596 / 18400** | — |
| Holo + PaCD | `cursor_t6_build_holo_pacd.cmd` | `HTS_T6_SIM_Test_holo_pacd.exe` | — | **12996 / 20400** |
| PN-masked | `cursor_t6_build_pn_masked.cmd` | `HTS_T6_SIM_Test_pn.exe` | **12596 / 18400** | — |
| Holo + PN-masked | `cursor_t6_build_holo_pn_masked.cmd` | `HTS_T6_SIM_Test_holo_pn.exe` | — | **12996 / 20400** |
| PaCD + PN-masked | `cursor_t6_build_pacd_pn_masked.cmd` | `HTS_T6_SIM_Test_pacd_pn.exe` | **12596 / 18400** | — |
| Holo + PaCD + PN-masked | `cursor_t6_build_holo_pacd_pn_masked.cmd` | `HTS_T6_SIM_Test_holo_pacd_pn.exe` | — | **12996 / 20400** |

**결론**: PN-masked 및 PaCD 조합은 **Default / Holo 대비 정량 동일** — 회귀 게이트 충족.

## 빌드 스크립트 (`HTS_TEST/t6_sim/`)

| 스크립트 | 정의 플래그 |
|----------|-------------|
| `cursor_t6_build.cmd` | baseline Walsh |
| `cursor_t6_build_holo.cmd` | `HTS_USE_HOLOGRAPHIC_SYNC` |
| `cursor_t6_build_pacd.cmd` | `HTS_USE_PACD` |
| `cursor_t6_build_holo_pacd.cmd` | Holo + PaCD |
| `cursor_t6_build_pn_masked.cmd` | `HTS_USE_PN_MASKED` |
| `cursor_t6_build_holo_pn_masked.cmd` | Holo + PN (**Step 7-2**) |
| `cursor_t6_build_pacd_pn_masked.cmd` | PaCD + PN (**Step 7-2**) |
| `cursor_t6_build_holo_pacd_pn_masked.cmd` | Holo + PaCD + PN (**Step 7-2**) |

## Commit 흐름 (저장소 기록)

| Step | 해시 | 요약 |
|------|------|------|
| 1 | ae02a8c | 16KiB I-only LUT, BPTE row clamp |
| 2 | 32e7b1e | HNS FWHT 분석, Phase0·유닛테스트 |
| 3-2 | 8811170 | `pn_masked_phase0_scan` + fwht_raw |
| 3-3 | 5add2ec | Sync_PSLTE 통합 |
| 3-4 | 608663a | Sync_AMI 통합 |
| 4-1 | a4e6d39 | PTE sub-chip 함수 |
| 4-2 | 450104b | PTE 호출 통합 |
| 5-1 | e570cda | ARM SIMD FWHT wrapper (PC fallback) |
| 5-2 | 4a17f3b | PC 프로파일 |
| 6-1 | 68c867a | `device_id_to_row` |
| 6-2 | 6fdb176 | TX/RX/PaCD device 통합 |
| 7-1 | 3bfa960 | KCMVP constant-time 측정 |
| **7-2** | *(저장소 `feat(PN-masked): final T6 measurement + build scripts + report`)* | T6 매트릭스 + 빌드 스크립트 + 본 문서 |

*(위 해시는 본 브랜치 `git log` 기준; 로컬 merge 히스토리에 따라 표시 순서만 다를 수 있음.)*

## 양산 적합성 (체크리스트)

- **AMI**: PSLTE와 동일 PN 가드 경로; `HTS_TARGET_AMI && !HTS_PHASE0_WALSH_BANK` 시 검증 플래그 스킵 분기 문서화됨 (`Sync_*`).
- **PS-LTE / 재난망 / NIS**: Walsh + 선택 Holo/PaCD/PN 조합 T6 동일 점수.
- **군용 LPI**: 디바이스별 행 + KCMVP 타이밍 증빙(Step 7-1).

## 미결 (양산 직전 옵션)

- Cortex-M4 **SMUAD 등 SIMD 본체** (`e570cda` wrapper 자리) — 칩별 검증 후 치환.
- **FIPS 140-3 / KCMVP** 제출용: 본 문서 + `HTS_V400_Dispatcher_PNMasked_KCMVP_TIMING.md` + 유닛 테스트 로그 아카이브.

## 재현

```bat
cd HTS_TEST\t6_sim
cursor_t6_build.cmd && HTS_T6_SIM_Test.exe
cursor_t6_build_holo_pn_masked.cmd && HTS_T6_SIM_Test_holo_pn.exe
```

정량 줄은 출력 하단 `정량 합계:` 행에서 확인.
