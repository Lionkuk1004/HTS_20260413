# AMI Sync 분리 결과

## 요약

- **AMI T6**는 `HTS_V400_Dispatcher_Sync_AMI.cpp` TU에서 본문 직전에 `HTS_TARGET_AMI` 를 `#undef` 하여, 기존 `Sync.cpp` 안의 **PS-LTE `#else` 경로만** 타도록 했다.
- **PS-LTE T6**는 기존과 같이 `HTS_V400_Dispatcher_Sync.cpp` 만 include 한다 (`Sync.cpp` **미수정**).
- 실측 결과, **AMI 합계가 PS-LTE와 동일(13839/15200)** 해졌다. (이전 32×2 실험 경로: 230/15200)

## 커밋

| 단계 | hash | 메시지 |
|------|------|--------|
| Sync_AMI 신설 | `f502e0e` | `ami_split: Sync_AMI.cpp 신설 (PS-LTE Sync 복사 + TU 내 HTS_TARGET_AMI undef)` |
| TU 링크·스크립트 주석 | `36d10fd` | `ami_split: AMI 빌드에서 Sync_AMI.cpp TU 링크 (T6 include + cmd 주석)` |

## 빌드 스크립트·소스 링크 (중요)

- `cursor_t6_build.cmd` / `cursor_t6_build_ami.cmd` 의 `cl` 인자에는 **Dispatcher .cpp 가 직접 나열되지 않는다.**
- `HTS_T6_SIM_Test.cpp` 하단에서 `../../HTS_LIM/*.cpp` 를 **단일 TU로 include** 하므로, AMI 전용 Sync 전환은 **`#if defined(HTS_TARGET_AMI)` 로 `HTS_V400_Dispatcher_Sync_AMI.cpp` vs `Sync.cpp` 분기**로 구현했다.
- `cursor_t6_build_ami.cmd` 에는 위 동작을 설명하는 **REM 주석**만 추가했다.

## 빌드

| 대상 | 스크립트 | 결과 |
|------|-----------|------|
| AMI | `cursor_t6_build_ami.cmd` | **성공** (`build_ami_split.log`) |
| PS-LTE | `cursor_t6_build.cmd` | **성공** (`build_baseline_split.log`, 기존과 동일하게 REM 줄로 인한 PowerShell 경고 1줄 가능) |

## PS-LTE 회귀

- 로그: `HTS_TEST/t6_sim/baseline_after_split.log`
- **정량 합계: 13839 / 15200 (91.0%) — 전체 BER: 0.08954**
- **13839 유지: Yes**

## AMI 3회 (Split 후)

| Run | 로그 | 정량 합계 | BER |
|-----|------|-----------|-----|
| 1 | `ami_split_run1.log` | **13839 / 15200 (91.0%)** | **0.08954** |
| 2 | `ami_split_run2.log` | **13839 / 15200 (91.0%)** | **0.08954** |
| 3 | `ami_split_run3.log` | **13839 / 15200 (91.0%)** | **0.08954** |

- **이전 (Sync.cpp 내 32×2 AMI 분기)**: 230 / 15200 (약 1.5%, BER 약 0.966 참고: `PHASE_B1_AMI_STEP2_RESULT.md`)
- **개선**: 통과 trial **+13609** (동일 BER 숫자는 전체 집계 기준으로 PS-LTE와 일치)

## 시나리오별 (AMI split run1 요약)

`ami_split_run1.log` 상단 요약 테이블 기준으로는 **PS-LTE baseline과 동일한 PASS/PART/FAIL 패턴**이다 (예: S3 저 SNR FAIL, S5 5000Hz FAIL, S7 일부 PART 등).

| S | 비고 (run1 샘플) |
|---|------------------|
| S1 | PASS 100/100 |
| S2 | 전 위상 PASS |
| S3 | -30~-15dB FAIL, -10dB PART, -5dB PART, 0~10dB PASS |
| S4 | +0~+127 PASS |
| S5 | 0~2000Hz PASS, **5000Hz FAIL** |
| S6 | 전 케이스 PASS |
| S7 | 0dB PASS, 5/10dB PART, 15~30dB FAIL (baseline과 동일 계열) |
| S8/S9 | FAIL (baseline과 동일) |

(Step 3 직후 AMI 로그와 셀 단위 숫자 비교는 별도 diff 미수행.)

## `walsh_shift` 분포 (`ami_split_run1.log`, `[PAYLOAD-SHIFT]`)

- **총 라인**: **15919**
- **상위 값** (shift: 건수): **0: 14288**, 16: 268, 32: 153, 63: 53, … (비-zero 다수; Step 2 이후 “전부 0”이 아님)

## 판정

- **[x] 성공 (10000+)** — 실제 **13839**로 PS-LTE와 동일 구간.
- **[ ] 부분 성공 (5000~10000)**
- **[ ] 재검토 필요 (5000 미만)**

## 금지 사항 준수

- `HTS_V400_Dispatcher_Sync.cpp` **미변경** (이번 단계).
- `Payload` / `HTS_FEC_HARQ` **미터치**.
- `cursor_t6_build.cmd` (PS-LTE) **플래그·소스 목록 미변경**.

## 다음 단계

1. **리포 정리**: `Sync.cpp` 에 남아 있는 `#if HTS_TARGET_AMI` 블록은 AMI T6에서 **더 이상 컴파일되지 않음**. 원하면 별도 커밋으로 **PS-LTE 전용 정리(삭제)** 하여 가독성·유지보수 비용을 줄인다.
2. **사양 반영**: 공식 AMI(64-chip, 1200 bps, LEA)는 **`Sync_AMI.cpp`만** 수정하고, `HTS_TARGET_AMI` undef 전략을 **제거·대체**한 뒤 AMI 전용 분기·임계를 넣는다.
3. **임베디드 빌드**: `HTS_LIM.vcxproj` 등은 여전히 `HTS_V400_Dispatcher_Sync.cpp` 만 나열할 수 있으므로, 펌웨어 AMI 타깃 시 **`Sync_AMI.cpp` 추가·조건부 컴파일**을 별도로 맞춘다.
