# CFO Apply 복원 지시 — 실행 보고

## 조사 결과 (코드)

- **`cfo_.Apply(chip_I, chip_Q)`는 이미 `Feed_Chip`에 존재**했습니다.  
  - `HTS_LIM/HTS_V400_Dispatcher_Sync_AMI.cpp`  
  - `HTS_LIM/HTS_V400_Dispatcher_Sync_PSLTE.cpp`  
- **`HTS_V400_Dispatcher_Sync.cpp` 단일 파일은 없음** (AMI/PSLTE TU 분리 후 구조).
- **`Core/Src/HTS_V400_Dispatcher.cpp`** 에도 DC 직후 `cfo_.Apply` 호출이 있음 (주석: CFO 보정).

지시서 배경의 “Apply 호출 안 됨”은 **현재 트리와 불일치**입니다. 다만 Sync 쪽에 **“시간도메인 Apply 철거”와 실제 `Apply` 호출이 함께 있어 주석이 모순**이었음.

## 이번에 한 수정

| 파일 | 내용 |
|------|------|
| `Sync_AMI.cpp` / `Sync_PSLTE.cpp` | `Feed_Chip`: DC 이후 블록 주석을 **“CFO 역회전 … no-op” + DC→CFO→AGC** 로 정리 (`cfo_.Apply` 줄은 **변경 없음**) |

`HTS_CFO_Compensator.h` 및 `Estimate_From_Preamble` 호출부: **미수정** (지시 준수).

## baseline (매크로 OFF)

| 타깃 | 기존(참고) | 이번 run1 | run2(AMI) |
|------|------------|-----------|-----------|
| AMI | 14939/18400 | **14939/18400** | **14939/18400** |
| PS-LTE | 16139/18400 | **16139/18400** | — |

Determinism: AMI `정량 합계` run1/run2 **동일**.

## S5 (AMI, run1)

| CFO | S5 (성공 trial/100) |
|-----|---------------------|
| 0Hz | 100 |
| 2500Hz | 0 |
| 3500Hz | 0 |
| 5000Hz | 0 |
| 10000Hz | 0 |
| 15000Hz | 0 |
| 25000Hz | 0 |

→ **주석 정리만으로 수치 변화 없음** (동작은 복원 전과 동일).

## 판정

- 지시서의 “**Apply 1줄 추가**” 작업: **해당 없음** (이미 반영됨).
- 실측: baseline **유지** → 시나리오 **A의 ‘복원으로 인한 개선’** 은 해당 없음; **회귀 없음(C에 해당하는 악화도 없음)**.

## 커밋 / 태그

- **선택**: 주석 정리만 반영하는 커밋은 가능(히스토리 명확화).  
- **`T_CFO_APPLY_RESTORE` 태그**: 기능 복원이 아니므로 **부여하지 않음**을 권장 (오해 방지).

로그: `HTS_TEST/t6_sim/ami_cfo_run1.log`, `ami_cfo_run2.log`, `pslte_cfo_run1.log`.
