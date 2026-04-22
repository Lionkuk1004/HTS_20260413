# PHASE_B1_AMI_STEP3_MAIN.md

**조건**: 읽기 전용 · `ami_dry_run` · **`HTS_FEC_HARQ.cpp/.hpp` 미개편** · 로그: `HTS_TEST/t6_sim/ami_step2_run1.log`

---

## Step 0. 작업 트리 정리

```text
git update-index --refresh
→ HTS_LIM/HTS_CFO_Compensator.h, HTS_FEC_HARQ.cpp, HTS_V400_Dispatcher_Sync.cpp, HTS_TEST/t6_sim/HTS_T6_SIM_Test.exe : needs update

git status (요약)
→ 위 4개 modified, 다수 untracked (exe·md·cmd 등)
```

**참고 (이전 세션과 동일 현상)**: `HTS_LIM` 소스 3개는 **인덱스 blob 해시 = 작업트리 해시**로 내용 동일인데도 `M`으로 남는 **racy/stat 캐시** 성격이 강함. **실질 바이너리 diff**는 주로 **`HTS_T6_SIM_Test.exe`** (재빌드로 크기 변화).

---

## Q1. AMI TX payload chip 생성 단위

### Q1-A. `HTS_V400_Dispatcher_TX.cpp` — `HTS_TARGET_AMI`

- **`Select-String` 결과: `HTS_TARGET_AMI` 문자열 0건** — **TX.cpp 에 AMI 전용 `#if` 분기 없음**.

### Q1-B. `Build_Packet` — DATA 페이로드 칩 수

- **프리앰블/헤더**: `pre_reps_+1`회 **64칩** `walsh_enc` + 헤더 2심볼 각 **64칩** (`L57~L161`).
- **DATA (`PayloadMode::DATA`)**: `nsym64_live = cur_nsym64_()` 심볼 수만큼, **심볼당 `walsh_enc(..., 64, ...)` 또는 `walsh_enc_split(..., 64, ...)`** (`L192~L221`).  
  → **에어 인터페이스 상 Walsh 심볼 1개 = 64 chip** (AMI/PS-LTE 공통 TX 경로).

### Q1-C. FEC 상수 (`HTS_FEC_HARQ.hpp`)

| 상수 | 값 / 의미 |
|------|-----------|
| **`C64`** | **64** (`L130`) |
| **`NSYM64`** | `HTS_FEC_M4_RAM_LAYOUT`(또는 PC에서 `HTS_FEC_SIMULATE_M4_RAM_LAYOUT`) 시 **`BPS64=4` → `(TOTAL_CODED+3)/4` = 172** (`L146~L148`) |
| **`C32` / `NSYM32`** | **본 헤더에서 정의 없음** (검색 0건) |

**정리**: **TX는 AMI라도 DATA payload 를 “32-chip/symbol”로 내지 않음.** AMI는 **Sync 쪽 Phase0 검출 메트릭**에서 32×2 등을 쓰는 것과 **별개**이며, **FEC·Walsh 송신 심볼 단위는 64 chip 유지**다.

---

## Q2. AMI RX `on_sym_` 동작

### 함수 위치

- `HTS_V400_Dispatcher_Payload.cpp` **`on_sym_()`** — **`L110`** 부근.

### 심볼당 칩 수

- `L122`: `const int nc = (cur_mode_ == PayloadMode::DATA) ? 64 : 16;`  
  → **DATA 모드는 항상 `nc == 64`**.

### `buf_idx_` / 64칩 임계

- `buf_idx_`는 `Feed_Chip` 경로에서 **64칩 모일 때** `on_sym_()` 호출 (`Payload.cpp` **`L656`** 근처에서 `on_sym_()` — 본 조사에서 상위 흐름만 확인).  
- **`on_sym_` 본문**: `buf_I_/buf_Q_`를 **`nc`칩**으로 `orig_*`에 복사 후, IR이면 `sym_idx_ * FEC_HARQ::C64`에 `ir_chip_*` 기록 (`L242` 등).

### AMI와의 “칩 단위 불일치” 가능성

- **Payload 경로 상으로는 32-chip/symbol 모드가 없음** → **“AMI TX=32 / RX=64” 식의 구조적 chip-unit mismatch 는 DATA 페이로드에 대해 성립하지 않음.**  
- AMI 붕괴는 **Phase0 정렬·CFO·LLR·HARQ 내부 결합** 등 **다른 층**에서 설명하는 편이 타당.

---

## Q3. HDR shift XOR — AMI에서 동작·로그 분산

### 코드 위치

- **첫 헤더 Walsh 심볼** (`hdr_count_ == 0`): `HTS_V400_Dispatcher_Sync.cpp` **`L1622~L1636`**  
  - `walsh_shift = current_walsh_shift_()` (= `dominant_row_ ^ 63`)  
  - `corrected_sym = (raw_sym & 63) ^ walsh_shift`  
  - **`[HDR-SHIFT]`** printf (`L1631~L1636`).
- **두 번째 헤더 템플릿 매칭**: **`L1690~L1692`** 부근에서 동일하게 `current_walsh_shift_()` 사용.

### `parse_hdr_`

- `HTS_V400_Dispatcher_Payload.cpp` **`parse_hdr_()`** — **`L67~L108`**.  
  - **XOR 없음**: `hdr_syms_[0/1]`에서 비트 조합으로 `mb`, `plen`, `iq` 복원 후 `FEC_HARQ::bps_from_nsym(plen)` 등.  
  - **HDR shift는 Sync 의 Walsh 디코드 단계에서 이미 적용된 심볼**이 `hdr_syms_[]`에 들어온다는 구조.

### 로그 (`ami_step2_run1.log`)

| 항목 | 수치 |
|------|------|
| **`[HDR-SHIFT]`** | **183684**건 |
| **`shift=` 분포** | **`shift=0` → 139926**건 다수 + **`shift=32` → 19403** 등 **0~63 광범위** |
| 샘플 | `shift=0 dom=63` … / `shift=32 dom=31` … (Step 2 이후 **`dom` 시변**이 **HDR까지 전파**) |

**결론**: **HDR XOR 경로는 실행되고 있으며**, Step 2 이후 **`current_walsh_shift_()` 분산이 `[HDR-SHIFT]`에 반영됨** → **“AMI에서 HDR XOR 자체가 죽어 있다”는 가설은 로그상 기각**에 가깝다.

---

## Q4. HDR / Decode / 최종 격차

| 태그 / 지표 | `ami_step2_run1.log` |
|-------------|----------------------|
| **`[HDR-` 전체** | **350790**건 (부분 문자열; `HDR-SHIFT`, `HDR-FAIL` 등 포함) |
| **`[HDR-SHIFT]`** | **183684** |
| **`[PAYLOAD-SHIFT]`** | **12510** |
| **`Decode64_IR entry`** (부분 문자열) | **4** (FEC 내부 printf가 **제한적**이거나 stderr 병합으로 **본 로그에 적게 잡힘**) |
| **`harq_round` / `retx` 등** | 동일 로그에서 **유의미한 패턴 없음** (짧은 grep) |
| **최종 PASS** | **230 / 15200** |

**격차 해석 (정성)**  

1. **HDR 단계**: 라인 수가 매우 많아 “전부 실패”라기보다 **칩 스트림 전반에서 반복 시도·로그 폭주**에 가깝다.  
2. **`[PAYLOAD-SHIFT]` 12510** → **Decode 진입(또는 직전 IR 경로)은 상당 수** 존재.  
3. **최종 230** → **Decode 이후(memcmp/CRC 등) 대량 탈락** — Step 3 Precheck와 정합.  
4. `Decode64_IR entry` **4건**만 잡힌 것은 **로그 샘플링/조건 printf** 한계로 보이며, **`[PAYLOAD-SHIFT]`와 동일하게 “Decode 미진입”으로 단정할 수는 없음**.

---

## Q5. 과거 CFO / Stage 6 이력 주석 (검색 샘플)

| 위치 | 내용 |
|------|------|
| `HTS_FEC_HARQ.cpp` **L555, L640** | `Stage 6B` — RX FWHT bin / `Decode64_IR` conv path dump |
| `HTS_V400_Dispatcher_Sync.cpp` **L593~L594** | **Fix A1 (V28)**, **Fix A3 (V34)** — PS-LTE Phase0 dominant 후보 |
| `Sync.cpp` **L1690** | **Stage 6A+** — CFO 시 RX chip 열 `k_w_{sym XOR shift}` |
| `Sync.cpp` **L1748~L1750** | **V29b** — HDR 템플릿 linear floor / `[HDR-TMPL]` 관련 |
| `HTS_V400_Dispatcher.hpp` **L339** | **Stage 6A** — `current_walsh_shift_` 주석 (`63 ⊕ dom`) |
| `HTS_FEC_HARQ.hpp` **L169** | `NF_HEAVY_JAM = **2000u**` — 재밍 단계 NF 임계 (CFO 2000Hz S5와 **직접 동일 개념은 아님**) |

---

## 종합 체크

| 항목 | 판단 |
|------|------|
| **[ ] chip unit mismatch (32 vs 64) — 페이로드** | **구조적으로 “아님”** — TX·`on_sym_`·`C64` 모두 **64 chip / DATA 심볼**. AMI 32-chip은 **Phase0 검출** 영역. |
| **[ ] HDR 단계만 탈락** | **부분적 기각** — `[HDR-SHIFT]`에 **shift 분산** 확인; 다만 **350k+ `HDR*` 로그**는 실패/재시도 혼재 가능. |
| **[x] Decode 단계 탈락** | **`[PAYLOAD-SHIFT]` ≫ 최종 PASS** 패턴과 **Step 2에서도 PASS 불변** → **FEC/IR 결합 이후**가 주 전장. |
| **[x] 기타** | **HARQ 내부**(`walsh_shift` 소비, `fec_ir_fwht_bin_unshift` 등) — **파일 미터치** 전제로 **원인 후보로 유지**. |

---

## Step 4 권고 (구현 전)

1. **`[PAYLOAD-SHIFT]` 직후 `pkt.success_mask`**를 **기존 DIAG 한도 내**에서 시나리오별로 상관 (호출부만).  
2. **`dominant_row_` / `cur_bps64_` / `hdr_syms_`**이 **동일 패킷 경계에서 일관되는지** (리셋·`full_reset_` 타이밍).  
3. **HARQ.cpp 미터치** 유지 시: **호스트 단일 TU**에서만 가능한 **외부 계측**(예: 성공 마스크를 Dispatcher에서 이미 갖는지)으로 **Decode 성공률**을 분리.

---

**금지 준수**: 코드 수정·빌드 없음.
