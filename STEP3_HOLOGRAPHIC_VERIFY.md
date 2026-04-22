# Step 3 Holographic Verify 보고 (v3 실측 기반 최종판)

## 사전 상태

- 기준 태그: `T_HOLO_SYNC_OFF_OK` (`21c7f9a`)
- 브랜치: `opt/holographic_sync`
- Step 2 baseline: AMI **14939/18400**, PS-LTE **16139/18400**

## v3에서 반영한 코드 정리 (테스트 TU만)

| 항목 | 내용 |
|------|------|
| `feed_raw_ext_holo` | 기존과 동일 — `setup` 직후 `Set_Holographic_Sync(true)` (변경 없음) |
| `test_S5_holographic` | **소스 순서**: `test_S5_seed_fixed()` **닫는 `}` 직후**로 이동 (지시서 v3). `static void test_S5_holographic() noexcept` 및 `hdr` 부제 문자열을 v3에 맞게 정리. |
| `main` | 기존과 동일: `test_S5` → `test_S5_seed_fixed` → `#ifdef` `test_S5_holographic` → `test_S6` |
| `feed_raw_ext` / `setup` / `test_S5` / `test_S5_seed_fixed` | **미수정** |
| 빌드 스크립트 | 실측 시 `HOLO_FLAG=/DHTS_USE_HOLOGRAPHIC_SYNC` 로 빌드함. **저장소 기본값은 다시 OFF** (`set HOLO_FLAG=`) 로 두어 Step2 기본 빌드와 동일. 재측정 시 주석 swap. |

## 빌드 (HOLO 매크로 ON, `cmd /c cursor_t6_build*.cmd`)

| 타깃 | 결과 |
|------|------|
| AMI | OK (`build_ami_step3v3.log`) |
| PS-LTE | OK (`build_pslte_step3v3.log`) |

## 교차 점검 1 — Determinism

| 대상 | 전체 로그 `fc /b` | S5H 테이블 행만 추출 후 `fc` |
|------|-------------------|------------------------------|
| AMI run1 vs run2 | **불일치** (소요 시간 등 비결정적 출력 포함으로 추정) | **일치** (`ami_s5h_only1.txt` vs `ami_s5h_only2.txt`, FC: no differences) |
| PS-LTE run1 vs run2 | (전체 비교 생략) | **일치** (`pslte_s5h_only1/2.txt`, FC: no differences) |

**판정**: 종합 테이블·S5H **수치**는 2회 동일. 전 로그 바이트 동일을 determinism 정의로 쓰지 않는 것이 타당.

## 교차 점검 2 — 기존 시나리오 회귀 (HOLO ON 빌드)

| 지표 | 기대 (Step2) | 실측 (run1) |
|------|----------------|-------------|
| AMI 종합 | 14939/18400 에 해당하는 pass + S5H 분모 +2000 | **14939 / 20400** |
| PS-LTE 종합 | 16139/18400 + 2000 | **16139 / 20400** |

기존 18400 행에 대한 pass 합: **14939 / 16139 유지** (S5H가 20×100 trial 모두 0 기여).

## 교차 점검 3 — S5 vs S5-HOLO (`ami_step3v3_run1.log` / `pslte_step3v3_run1.log`)

### AMI (200 kcps): pass / 100

| CFO | S5 | S5H |
|-----|-----|-----|
| 0Hz | 100 | 0 |
| 50Hz | 100 | 0 |
| 100Hz | 100 | 0 |
| 200Hz | 100 | 0 |
| 500Hz | 100 | 0 |
| 1000Hz | 100 | 0 |
| 2000Hz | 100 | 0 |
| 2500Hz | 0 | 0 |
| 3000Hz | 0 | 0 |
| 3500Hz | 0 | 0 |
| 4000Hz | 100 | 0 |
| 4500Hz | 100 | 0 |
| 5000Hz | 0 | 0 |
| 7500Hz | 0 | 0 |
| 10000Hz | 0 | 0 |
| 12500Hz | 0 | 0 |
| 15000Hz | 0 | 0 |
| 17500Hz | 0 | 0 |
| 20000Hz | 0 | 0 |
| 25000Hz | 0 | 0 |

- **S5** CFO 행 합계(통과 행만): 7+2 = **9/20** 행이 전 trial PASS (저역 7 + 4000/4500).  
- **S5H**: **0/20** 행 전부 0/100.

### PS-LTE (1 Mcps)

| CFO | S5 | S5H |
|-----|-----|-----|
| 0Hz ~ 5000Hz (13점) | 모두 100 | 모두 0 |
| 7500Hz | 0 | 0 |
| 10000Hz | 100 | 0 |
| 12500Hz | 0 | 0 |
| 15000Hz | 0 | 0 |
| 17500Hz | 0 | 0 |
| 20000Hz | 100 | 0 |
| 25000Hz | 0 | 0 |

- **S5**: 13+1+1 = **15/20** 행 PASS (표 위 grep 기준).  
- **S5H**: **0/20**.

## 교차 점검 4 — Phase 6 예측 vs 실측

| 항목 | 시뮬 기대(지시서 서술) | 실측 S5H |
|------|------------------------|----------|
| AMI CFO 구간 고 pass | 80~95%급 | **0%** (전 구간 0/100) |
| PS-LTE | 95~98%급 | **0%** |

→ **모델 예측과 실측 괴리 큼.** 원인은 이전 분석대로 **`phase0_scan_holographic_` 게이트·스케일·변환 도메인** 이슈로 보는 것이 타당 (Dispatcher/유틸 미수정 전제).

## 판정 (지시서 Step 3-10)

**시나리오: C (실패)** — S5H가 S5보다 나을 구간이 없고, CRC-only도 0.

- **Determinism (수치)**: S5H 행 기준 **OK**  
- **회귀 (18400 구간)**: **없음**  
- **성능 목표**: **미달** (A/B 아님)

## 커밋 / 태그

- **`T_HOLO_VERIFIED` 생성 안 함**, Step 3-11 커밋 **보류** (시나리오 C).

## 로그 파일

- `HTS_TEST/t6_sim/build_ami_step3v3.log`, `build_pslte_step3v3.log`  
- `HTS_TEST/t6_sim/ami_step3v3_run1.log`, `ami_step3v3_run2.log`  
- `HTS_TEST/t6_sim/pslte_step3v3_run1.log`, `pslte_step3v3_run2.log`  
- S5H-only 비교용: `ami_s5h_only1.txt` / `ami_s5h_only2.txt`, `pslte_s5h_only1.txt` / `pslte_s5h_only2.txt`

## 다음 단계 (영준님 결정)

- [ ] **Dispatcher 쪽** 임계·AMI `amp×19` 반영·L3 변환 정합 등 **튜닝/수정 지시서** (지시서 금지 범위 해제 여부 포함)  
- [ ] `HTS_DIAG_PRINTF`로 `[P0-HOLO-*]` 스캔 1회 캡처 후 **어느 게이트에서 막히는지** 확정  
- [ ] 동일 시드(`0x500000u`) **공정 비교** 스윕 (테스트만 변경 시나리오) 여부

---

*본 보고는 Step 3 v3 지시서의 실측·교차 점검 절차를 수행한 결과이며, Phase 6 “예측 달성”은 확인되지 않았습니다.*
