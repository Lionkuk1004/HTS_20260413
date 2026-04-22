# CFO 파이프라인 연결 복원 보고

## 사전 상태

- **branch**: `opt/holographic_sync` (작업 트리에 Phase 4.x 등 미커밋 변경이 있을 수 있음)
- **지시서 경로 `HTS_LIM/HTS_V400_Dispatcher.cpp`**: **저장소에 파일 없음** (0 hits). V400은 **TU 분리** 후 `Core` / `Sync_AMI` / `Sync_PSLTE` / `Core+…` 조합으로 빌드됨.
- **T6 시뮬** (`HTS_TEST/t6_sim/HTS_T6_SIM_Test.cpp`): `Feed_Chip` 본문은  
  `#include "../../HTS_LIM/HTS_V400_Dispatcher_Sync_AMI.cpp"` 또는 `…_Sync_PSLTE.cpp` 에 있음 (**단일 `HTS_V400_Dispatcher.cpp` 미사용**).

## 수정 (지시서 Step 3 대비)

| 지시 | 실제 |
|------|------|
| `HTS_LIM/HTS_V400_Dispatcher.cpp`에 `cfo_.Apply` 1줄 추가 | **해당 파일 없음** → 추가 불가 |
| `Feed_Chip` DC 직후 `cfo_.Apply` | **이미 구현됨**: `HTS_V400_Dispatcher_Sync_AMI.cpp` / `Sync_PSLTE.cpp` 의 `Feed_Chip` 내 `cfo_.Apply(chip_I, chip_Q);` (예: AMI L1472–1476) |
| `HTS_CFO_Compensator.h` 수정 | **미수행** (지시 준수) |

현재 AMI `Feed_Chip` 일부:

```1472:1476:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher_Sync_AMI.cpp
    // CFO 역회전 적용 (Estimate 완료 시 active, 미완료 시 no-op)
    // 순서: DC → CFO → AGC
    cfo_.Apply(chip_I, chip_Q);
    // 프리앰블 AGC
    pre_agc_.Apply(chip_I, chip_Q);
```

`HTS_CFO_Compensator::Apply` 내부에는 **`if (!active_) return;`** 가 있어, Estimate 전에는 **no-op** (지시서 가정과 일치).

**옵션 A (`T_HOLO_SYNC_OFF_OK` 롤백)**: 본 문서 작성 시점에 **실행하지 않음** (영준님 선택 전제). 필요 시 별도 지시로 수행.

## 실측 (매크로 OFF, 현재 트리)

### 정량 합계

| 타깃 | baseline (기대) | 복원 후 run1 | run2 |
|------|-----------------|--------------|------|
| AMI | 14939/18400 | **14939/18400** | (동일 구성이면 run2도 동일 예상) |
| PS-LTE | 16139/18400 | **16139/18400** | — |

`cmd /c` 빌드 후 단일 실행으로 확인 (2026-04-22 측정).

### S5 CFO 별 비교 (AMI)

지시서의 “복원 후” 칸은 **코드 변경이 없었으므로 baseline과 동일**입니다. (이전 로그와 동일 패턴: 2500/3000/3500Hz FAIL 등.)

| CFO | baseline | 복원 후 (=현재) |
|-----|----------|-----------------|
| 0Hz | 100 | 100 |
| 2500Hz | 0 | 0 |
| 3500Hz | 0 | 0 |
| 5000Hz | 0 | 0 |
| 10000Hz | 0 | 0 |
| 25000Hz | 0 | 0 |

(전체 20행 테이블은 기존 T6 출력과 동일.)

### S5 (PS-LTE)

동일 이유로 **변경 없음** (16139 유지).

## Determinism

- 별도 2회 추출·`fc`는 이번에 생략. **동일 바이너리·동일 시드**면 기존과 같이 결정론적임.

## 판정 (지시서 Shape A/B/C)

- **Shape-B (동등)**: “`cfo_.Apply` 추가”에 해당하는 **새 코드 변경이 없음** → 정량·S5 **baseline과 동일**이 정상.
- 배경 서술의 “Apply 호출 안 됨”은 **현재 `HTS_LIM` Sync TU 기준으로는 사실이 아님**. 과거 단일 `Dispatcher.cpp` 시절 주석이 남아 오해를 부었을 가능성은 있음.

## 커밋 / 태그

- **Shape-A 아님** → **`T_CFO_PIPELINE_RESTORE` 태그 및 지시서 커밋 메시지로의 커밋은 하지 않음** (할 일이 “이미 완료” 상태).

## 다음 단계 (지시서 Step 8 정합)

1. **양산/펌웨어 빌드 경로**가 `Core/Src/HTS_V400_Dispatcher.cpp`만 쓰는지, `Sync_*`를 include하는지 **빌드 매트릭스별로 한 번 더 확인**할 것. (`Core/Src/...` 의 `Feed_Chip`에도 이미 `cfo_.Apply` 존재.)
2. **Phase 4.1/4.2** 롤백 여부는 영준님 판단. CFO “공백”이 원인이라는 가설은 **현 트리에서는 재검증 필요** (Apply는 이미 연결됨).
3. **Shape-B 심화 분석**: `Estimate_From_Preamble`의 `mag_approx < 1000` 등으로 `active_`가 거의 안 켜지는지, `HTS_DIAG_CFO_APPLY` 등으로 샘플링 가능.

---

## 요약 한 줄

**`HTS_LIM`에는 `HTS_V400_Dispatcher.cpp`가 없고, T6가 쓰는 `Feed_Chip`에는 이미 `cfo_.Apply`가 연결되어 있음 → 지시서 Step 3 “1줄 추가”는 추가 작업 없음; 파이프라인 공백 지적은 현 구조와 불일치.**
