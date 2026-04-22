# Step 0+1 Holographic Util 보고

## Step 0 결과

- **기준 태그**: `T_SEED_ABLATION` (브랜치 분기 시점 `ami_dry_run` HEAD)
- **작업 브랜치**: **`opt/holographic_sync`** (신규 생성)
- **지시서 문구 AMI/PS-LTE 13839·13939 / 15200**: *CFO sweep 확장 + `S5F` 이전* baseline. 현재 `ami_dry_run` 트리에서는 S5 20점 + `test_S5_seed_fixed` 로 **분모 18400** 이 표준이다.

### 파일 위치 (정적 확인)

| 항목 | 위치 |
|------|------|
| AMI `phase0_scan_()` 정의 | `HTS_LIM/HTS_V400_Dispatcher_Sync_AMI.cpp` **L62** |
| AMI `phase0_scan_()` 호출 예 | 동 파일 **L1265** |
| PS-LTE `phase0_scan_()` 정의 | `HTS_LIM/HTS_V400_Dispatcher_Sync_PSLTE.cpp` **L52** |
| PS-LTE `phase0_scan_()` 호출 예 | 동 파일 **L1255** |
| `k_w63[64]` | `HTS_LIM/HTS_V400_Dispatcher_Internal.hpp` **L69~77** (`inline constexpr int8_t k_w63[64]`) |
| `fwht_raw` | 동 헤더 **L173~** (`inline void fwht_raw(int32_t* d, int n)`) — 호출은 `HTS_V400_Dispatcher_Payload.cpp`, `HTS_V400_Dispatcher_Decode.cpp` 등 |

### 분할 Sync TU

- `HTS_LIM/HTS_V400_Dispatcher_Sync_AMI.cpp` — 존재
- `HTS_LIM/HTS_V400_Dispatcher_Sync_PSLTE.cpp` — 존재

**→ Step 1 진행 완료**

---

## Step 1 결과

### 신규 파일

| 파일 | 크기 (B) | 용도 |
|------|----------|------|
| `HTS_LIM/HTS_Preamble_Holographic.h` | 4310 | API·주석 |
| `HTS_LIM/HTS_Preamble_Holographic.cpp` | 5486 | L1+L2+L5 구현, ROM Chu, `k_w63_local` |
| `HTS_TEST/t6_sim/HTS_Holographic_Unit_Test.cpp` | 7503 | 단위 테스트 7건 |
| `HTS_TEST/t6_sim/build_holo_ut.cmd` | 587 | `cl` 독립 빌드 |

**HTS 본 경로 Dispatcher/Sync/HARQ TU: 변경 없음** (신규 추가만).

### 구현 메모

- `peak_to_median_ratio_x10` selection-swap: 지시서 초안의 swap이 값 유실 가능 → **표준 swap(`t = buf[k]; buf[k]=buf[min_idx]; buf[min_idx]=t`)** 로 수정.
- `k_w63_local[]` 는 `HTS_V400_Dispatcher_Internal.hpp` 의 `k_w63` 과 **동일 패턴** (링크 ODR 충돌 방지).

### 빌드 결과

- **단위 테스트 빌드**: **OK** (`build_holo_ut.log`)
- **cl warning (문자열 `warning`)**: **0** (로그 기준)

### 단위 테스트 결과 (`holo_ut_run.log`)

| # | 항목 | 결과 | 비고 |
|---|------|------|------|
| 1 | Chu table accuracy | **PASS** | max err **63 ppm** |
| 2 | `holo_dot` clean | **PASS** | energy **511968055** |
| 3 | `holo_dot` noise only | **PASS** | energy **237280** |
| 4 | ratio single peak | **PASS** | ratio_x10 **500** |
| 5 | ratio flat | **PASS** | ratio_x10 **10** |
| 6 | ratio median=0 | **PASS** | clamp **9999** |
| 7 | `holo_dot` + CFO 5 kHz @200 kcps | **PASS** | energy **448471770** |

**총: 7 / 7 PASS**

### T6 baseline 회귀 (신규 파일만 추가, T6 소스 무변)

| 빌드 | 정량 합계 | 비고 |
|------|-----------|------|
| AMI | **14939 / 18400 (81.2%)** | `ami_baseline_check.log` — 이전 실측과 동일 |
| PS-LTE | **16139 / 18400 (87.7%)** | `pslte_baseline_check.log` — 동일 |

- **15200 분모·13839/13939 수치**: 현재 트리와 불일치(스윕/S5F 반영). **회귀 판정**: 엔진 TU 미수정이므로 **수치 동일 유지 = 회귀 없음**.

---

## 커밋·태그

- **커밋 메시지**: `holographic_util: L1+L2+L5 유틸 모듈 신규 (시뮬 Phase6 기반, FTO safe)`
- **태그**: **`T_HOLO_L1_L2_L5_UTIL`**
- **커밋 ID**: **`20e5ff6`** (`git rev-parse T_HOLO_L1_L2_L5_UTIL^{}` 와 일치)

---

## 다음 단계

- [ ] Step 2 지시서: `phase0_scan_` / Payload 경로에 유틸 **통합**(본 트리 수정)
- [ ] Step 1 실패 시 분기: 해당 없음 (UT 전부 PASS)
