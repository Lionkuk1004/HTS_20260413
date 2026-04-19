# Waterfall Matrix v2 — 이중 통계 50×50

일자: 2026-04-19  
구현: `HTS_TEST/t6_waterfall/` (`WaterfallSpec.hpp`, `HTS_T6_Waterfall_Harness.cpp`, `build_waterfall.bat`, `run_waterfall.bat`, `build_waterfall_smoke.bat`)

## 빌드

| 항목 | 결과 |
|------|------|
| `build_waterfall.bat` | **PASS** → `HTS_T6_Waterfall.exe` (meta=50, inner=50) |
| `build_waterfall_smoke.bat` | **PASS** → `HTS_T6_Waterfall_Smoke.exe` (meta=2, inner=2, `/DHTS_WATERFALL_FAST_SMOKE`) |

## 실행 구성 (본측정 exe)

- Meta loops: **50**
- Inner trials: **50**
- Per sweep point: **2 500** trials  
- Sweep points: **61** (S1: 8, S3: 5, S7–S9 각 12)  
- **Grand total: 152 500** trials (예상 수 시간)

`outer_seed` (meta cloud):

`base_seed ⊕ (meta · 0x9E3779B9) ⊕ scenario_tag ⊕ sweep_tag ⊕ FNV-1a(spec.scenario_name)`  
여기서 `sweep_tag = round(sweep_value_dB × 100) × 131`, `scenario_tag = splitmix32(spec_index · 0x1A2B3C4D)`.

## 회귀 (T6 SIM)

- 본 하네스는 **별도 exe**이며 `HTS_T6_SIM_Test`를 링크하지 않음 → T6 **10927 / 11200** 수치는 기존 회귀 바이너리로 확인 (엔진·MC 하네스 **비변경**).

## 출력 파일

| 파일 | 내용 |
|------|------|
| `waterfall_stdout.log` | 사람 읽기 진행 로그 (`run_waterfall.bat`에서 stdout 리다이렉트) |
| `waterfall_stderr.log` | FEC 등 `fprintf(stderr, …)` 진단 |
| `waterfall_results.csv` | 스윕 포인트별 집계 (Grand + Meta 통계, **하네스가 직접 기록**) |
| `waterfall_per_meta.csv` | meta 루프별 상세 (본측정 시 **데이터 행 3,050** = 61×50, +헤더 1행) |

**주의:** 집계 CSV는 FEC stderr와 섞이지 않도록 **`fopen` 전용 파일**로 기록한다.

## 스모크 검증 (2×2, 244 trials, seed `0xABCDEF01`)

에이전트 세션에서 **전체 152500 trial 미실행**. 스모크로 로직·CSV·CI 경로만 검증.

- **실측 wall time:** 약 **3–4 s** (244 trials)  
- **WF_S1_Clean (스모크):** SNR ≤ −10 dB 구간 `grand_pass=0/4`, −5 dB 이상 `4/4` — 5 dB 스텝에서 **거친 cliff** 형태만 확인 (N=4이므로 CI는 매우 넓음).  
- **WF_S7_Barrage (스모크):** JSR=5 dB에서만 `4/4`, 그 이상 0/4 — 동일하게 **거친 샘플**만 의미 있음.

### Grand vs Meta (스모크 예: S1, SNR=+5 dB)

- `GrandPassRate=1`, `MetaStdRate=0` → 두 meta cloud 모두 동일 pass rate (**Case A** 성향).

전 시나리오·전 스윕에 대한 **PER 10% cliff**, **Meta std 큰 지점** 표는 **50×50 본 실행 후** `waterfall_results.csv` / `waterfall_per_meta.csv`로 채울 것.

## 실행 방법 (선택 / 본측정)

스모크(재검증, 선택):

```bat
cd /d D:\HTS_ARM11_Firmware\HTS_LIM\HTS_TEST\t6_waterfall
HTS_T6_Waterfall_Smoke.exe 0xABCDEF01
```

본 실행 (152,500 trials):

```bat
cd /d D:\HTS_ARM11_Firmware\HTS_LIM\HTS_TEST\t6_waterfall
build_waterfall.bat
run_waterfall.bat 0xABCDEF01
```

산출물 (작업 디렉터리 = `t6_waterfall`):

- `waterfall_stdout.log` — 진행·요약 (stdout)
- `waterfall_stderr.log` — FEC 등 진단 (stderr)
- `waterfall_results.csv` — **61행** 집계(헤더 제외 시나리오×스윕 포인트당 1행) + 헤더 1행
- `waterfall_per_meta.csv` — **3,050행** 상세(61×50 meta; 헤더 제외) + 헤더 1행

`run_waterfall.bat`은 `%~dp0`로 **스크립트 폴더로 이동**한 뒤 exe를 실행하므로, 저장소 루트에서 호출해도 동일하게 동작한다.

직접 실행 예:

`HTS_T6_Waterfall.exe 0xABCDEF01 1>waterfall_stdout.log 2>waterfall_stderr.log`

## 통계 해석 가이드 (요약)

- **Meta std ≈ 0**, min≈max: meta cloud 간 pass rate 일관 (**Case A**).  
- **Meta std 크고** min–max 벌어짐: 일부 `outer_seed` cloud에서만 취약 (**Case B**).  
- 본 실행 후 **Grand CI 반폭**과 **meta_std**를 스윕 값별로 비교해 “가짜 평균” 여부를 판단 (**Case C**는 추가 분석).

## 자진 고지

- `WF_S9_Combined`의 CW 주파수 범위는 MC `MC_S9`와 동일하게 **100 kHz–2 MHz**로 두었음 (지시서 원문 `135, 100000`은 필드 정렬 모호 → **rotate=135°**, freq 대역은 MC와 정합).  
- 스모크는 **분산 추정에 부적합**(meta=2).  
- `fopen` MSVC C4996 경고는 기존 테스트 스타일과 동일 계열; 필요 시 `_CRT_SECURE_NO_WARNINGS` 추가 가능.

## 다음 단계

- `HTS_T6_Waterfall.exe` **본 실행**으로 cliff 표·양산 사양 초안 채움.  
- Cliff 주변 ±2.5 dB / 1 dB 재스윕 시 `WaterfallSpec.hpp`만 조정하면 됨.
