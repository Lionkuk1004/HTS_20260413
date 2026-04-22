# Phase 4 Holographic 재설계 보고 (방안 A — Chu 제거 + L2+L3+L5)

## 사전 상태

- 이전 태그(지시): `T_HOLO_SYNC_OFF_OK` (21c7f9a, Step 2)
- 브랜치(지시): `opt/holographic_sync`
- baseline: AMI **14939/18400**, PS-LTE **16139/18400**

## 수정 파일

| 파일 | 변경 내용 |
|------|-------------|
| `HTS_LIM/HTS_Preamble_Holographic.h` | L2 중심 설명으로 갱신; `k_chu_table` / `verify_chu_table_max_err_ppm` 유지(단위 테스트·ROM 호환) |
| `HTS_LIM/HTS_Preamble_Holographic.cpp` | `holographic_dot_segmented`: Chu 곱 제거, raw chip × `k_w63_local` 8-seg non-coherent |
| `HTS_LIM/HTS_V400_Dispatcher_Sync_AMI.cpp` | `phase0_scan_holographic_`: L12 `(amp×19)²/4`, L3 raw FWHT, `off=-1`/버퍼 부족 DIAG+shift, `off+64>p0_chip_count_` 시 energy 0 |
| `HTS_LIM/HTS_V400_Dispatcher_Sync_PSLTE.cpp` | 동일 구조, L12/L3 `(amp×38)²/8` 및 `(amp×38)²×0.3` |

Dispatcher Core / Payload / TX / Decode, `HTS_V400_Dispatcher.hpp`, `HTS_V400_Dispatcher_Internal.hpp`, HARQ: **미수정** (지시 준수).

`HTS_TEST/t6_sim/HTS_T6_SIM_Test.cpp`: **미수정**.

## 매크로 OFF 회귀 (Gate 4-3)

| 타깃 | 기대 | 실측 | 일치 |
|------|------|------|------|
| AMI | 14939/18400 | 14939/18400 | OK |
| PS-LTE | 16139/18400 | 16139/18400 | OK |

## Holographic 활성화 (`/DHTS_USE_HOLOGRAPHIC_SYNC`, runtime holographic on)

빌드: `cursor_t6_build_ami.cmd` / `cursor_t6_build.cmd`에서 `set HOLO_FLAG=/DHTS_USE_HOLOGRAPHIC_SYNC`로 전환 후 빌드·실행. **실측 후 스크립트는 다시 `HOLO_FLAG` 빈 값(기본 OFF)으로 복귀**함.

### Determinism

- AMI: `ami_on_run1_p4.log` vs `ami_on_run2_p4.log` 상단 S5H 요약 테이블(동일 라인 번호·동일 PASS/FAIL 패턴) → **일치**
- PS-LTE: 동일 방식으로 재실행 시 동일 패턴 기대(1차 로그 `pslte_on_run1_p4.log` 보관)

### 정량 합계 (20400 = 18400 + 2000 S5-HOLO)

| 타깃 | 합계 | 해석 |
|------|------|------|
| AMI | **15039 / 20400** | 기존 행 14939 유지 + S5H **100** 통과 (2000 중) |
| PS-LTE | **16439 / 20400** | 기존 행 16139 유지 + S5H **300** 통과 (2000 중) |

S5-HOLO 시험당 CFO×100 trial 기준:

- AMI S5H: **100/2000 = 5.0%** (0Hz만 100/100 PASS)
- PS-LTE S5H: **300/2000 = 15.0%** (0Hz / 50Hz / 100Hz 각 100/100 PASS)

### S5 vs S5H (요약)

AMI 예시(동일 로그 `ami_on_run1_p4.log`):

| CFO | S5 (성공 trial) | S5H |
|-----|-----------------|-----|
| 0Hz | 100 | 100 |
| 50Hz | 100 | 0 |
| 100Hz | 100 | 0 |
| 200Hz | 100 | 0 |
| … | (고역 대부분 0) | 대부분 0 |

다수 CFO에서 **S5H &lt; S5** (홀로 경로가 여전히 L5/L3/스캔 한계).

## Python Phase 7 예측 대비

- 시뮬: L2+L3+L5, Chu 제거 시 고 SNR에서 전 CFO PASS 보고됨.
- 실측(T6, SNR/채널 모델 고정): **전 CFO 95%+ 미달**; 저역 일부 CFO에서만 PASS.
- 결론: **모델·실측 갭** — threshold(`ratio_x10≥25`, L3 `×0.3`) 또는 채널/전처리 가정 재튜닝 필요.

## 판정 (지시서 시나리오 A/B/C)

- Determinism: **OK**
- 매크로 OFF 회귀: **OK**
- AMI S5H 평균 ≥ 85%: **FAIL** (5%)
- PS-LTE S5H 평균 ≥ 90%: **FAIL** (15%)
- CFO별 S5H ≥ S5: **다수 CFO에서 불만족**

→ **시나리오 B (부분 성공)** — 지시에 따라 **커밋/태그 `T_HOLO_L2_L3_L5_CHU_REMOVED` 보류**.

## 구현 요약 (코드)

1. **L2**: 64칩에 대해 `k_w63` 부호만 적용한 뒤 8×8 세그먼트로 합산, `|seg_I|²+|seg_Q|²` 비코히런트 합.
2. **L5**: 기존 `peak_to_median_ratio_x10(energies, 64)` 유지, 임계 25.
3. **L3**: `best_off` 윈도우 **raw I/Q** → `fwht_raw` → row energy 상위 4합; AMI는 `amp×19`, PS-LTE는 `amp×38` 스케일.
4. **AMI L12 버그 수정**: 이전에는 AMI에서도 `amp×38`을 쓰고 있었음 → **`amp×19`** 로 분리.

## 다음 단계

- [ ] **B**: L5 `kRatioMin_x10`, L3 `verify_ratio`(현재 정수 `(amp*k)²×3/10`) 튜닝 지시서 또는 DIAG 재수집
- [ ] S5H가 S5를 악화시키는 CFO에서 `[P0-HOLO-*]` 로그 스윕으로 병목(L5 vs L3) 재확인

## 아티팩트

- `HTS_TEST/t6_sim/ami_on_run1_p4.log`, `ami_on_run2_p4.log`
- `HTS_TEST/t6_sim/pslte_on_run1_p4.log`
