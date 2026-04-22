# Phase 4.2 — CFO 2-Pass Sync (클러치) 보고

> **재확인 (Cursor, 2026-04-22)**  
> `HTS_Preamble_Holographic.{h,cpp}` 의 `derotate_buffer_q8` 및 `Sync_AMI`/`Sync_PSLTE` 의 Pass 2·가설 그리드가 **현재 트리에 존재**함. 지시서 스니펫과의 차이: `phase0_scan_holographic_`는 **`void`** + `psal_commit_align_()` / 실패 시 shift; `holographic_dot_segmented`는 **2인자**; 네임스페이스는 **`ProtectedEngine::Holographic`**.

## 수정

| 파일 | 변경 |
|------|------|
| `HTS_LIM/HTS_Preamble_Holographic.h` | `derotate_buffer_q8(...)` 선언 |
| `HTS_LIM/HTS_Preamble_Holographic.cpp` | `k_sin_q14_256[256]` (Python 생성 Q14 sin) + per-chip 누적 회전 `derotate_buffer_q8` |
| `HTS_LIM/HTS_V400_Dispatcher_Sync_AMI.cpp` | Pass 1 유지; 실패 시 Pass 2 — `k_cfo_hyp_ami[] = ±{3,6,9,12}` Q8 step; 최고 `ratio_x10` 가설 선택; 성공 시 `p0_buf128_*`에 derotate 반영 후 `psal_commit_align_` |
| `HTS_LIM/HTS_V400_Dispatcher_Sync_PSLTE.cpp` | 동일 패턴; `k_cfo_hyp_pslte[] = ±{1,2,4,6}`; `[P0-HOLO-PSLTE]` DIAG |

**미수정**: HARQ, `Internal.hpp`, `Dispatcher.hpp`, Core/Payload/TX/Decode, T6 테스트 소스.

**실제 API**: `phase0_scan_holographic_`는 계속 **`void`** + `psal_commit_align_()` / 실패 시 128칩 shift (지시서의 `bool`/`psal_pending_` 스니펫과는 다름).

## 매크로 OFF

| 타깃 | 기대 | 실측 |
|------|------|------|
| AMI | 14939/18400 | 14939/18400 OK |
| PS-LTE | 16139/18400 | 16139/18400 OK |

## 매크로 ON (20400)

| 타깃 | Phase 4.1 | Phase 4.2 |
|------|-----------|-------------|
| AMI | 15039 / 20400 (S5H 5%) | **15039 / 20400** (동일) |
| PS-LTE | 16639 / 20400 (S5H 25%) | **16639 / 20400** (동일) |

### Determinism

- AMI `ami_on_run1_p4_2.log` vs `ami_on_run2_p4_2.log`: S5H 요약 20행 **일치**
- PS-LTE run1 vs run2: S5H 요약 **일치**

### S5H (CFO별 성공 trial / 100)

Phase 4.2 실측은 Phase 4.1과 **동일 패턴** (AMI: 0Hz만 PASS; PS-LTE: 0/50/100/200/3500Hz PASS).

### DIAG 관찰 (Pass 2 동작)

Pass 1 실패 후 Pass 2가 자주 실행되며, **L12·L5는 통과하지만 L3=0**으로 `pass=0`인 사례가 다수:

- AMI 예: `[P0-HOLO-AMI] ... ratio_x10=118 l12=1 l5=1 l3=0 pass=0 pass2=1 hyp=-3`
- PS-LTE 예: `[P0-HOLO-PSLTE] ... ratio_x10=120 l12=1 l5=1 l3=0 pass=0 pass2=1 hyp=-1`

→ CFO 가설로 **peak/ratio는 개선**되나, **FWHT top-4 임계**가 여전히 병목으로 남는 경우가 많음.

## 판정 (지시서 A/B/C)

- **A** (AMI S5H≥80%, PS≥90%): **미달**
- **Phase 4.1 대비 S5H 정량**: **동일** → “목표 대비 개선” 관점에서는 **C (개선 미미)** 에 가깝음  
- 다만 Pass 2 경로·DIAG로 **클러치 로직은 동작**함 (B와 C 중간 서술 가능)

**커밋 / 태그 `T_HOLO_L2_L3_L5_CFO_2PASS`**: 시나리오 A 아님 → **보류**.

## 다음 단계 (참고)

- L3 임계·또는 FWHT 입력 구간(블록0만 vs 2블록)·가설 그리드 해상도 재검토  
- Pass 2에서 선택한 `hyp`가 실제 채널 CFO와 정렬되는지 오프라인 상관 분석

## 아티팩트

- `HTS_TEST/t6_sim/ami_on_run1_p4_2.log`, `ami_on_run2_p4_2.log`
- `HTS_TEST/t6_sim/pslte_on_run1_p4_2.log`, `pslte_on_run2_p4_2.log`
