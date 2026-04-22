# PHASE_B1_AMI_STEP3_PRECHECK.md

**조건**: 읽기 전용 · `ami_dry_run` 기준 코드/로그 조사 · **`HTS_FEC_HARQ.cpp/.hpp` 본문 미개편** · 호출부/소비 관점만 정리.

---

## Q1. `Decode*_IR` 호출

| 호출 | 파일:라인 | 인자 `walsh_shift` |
|------|-------------|-------------------|
| `FEC_HARQ::Decode16_IR(...)` | `HTS_LIM/HTS_V400_Dispatcher_Payload.cpp` **L389~L392** | **`walsh_shift_payload`** (지역 `const uint8_t`) |
| `FEC_HARQ::Decode64_IR(...)` | 동일 파일 **L472~L475** | 동일 **`walsh_shift_payload`** |

**정의부(참고만, 본 파일 수정 금지)**  
- `FEC_HARQ::Decode64_IR` / `Decode16_IR` 선언·구현: `HTS_LIM/HTS_FEC_HARQ.cpp` **L1551**, **L2112** 부근.

**`walsh_shift`에 들어가는 변수**  
- `try_decode_()` 진입 직후 **L342**: `const uint8_t walsh_shift_payload = current_walsh_shift_();`  
- 위 표의 IR 호출에 **그대로** 넘김 (추가 가공 없음).

**AMI / PS-LTE 분기**  
- **`HTS_V400_Dispatcher_Payload.cpp`에는 `#if defined(HTS_TARGET_AMI)` 문자열이 없음** (`Select-String` 0건).  
- 즉 **동일 `try_decode_` 경로**에서 AMI/PS-LTE를 나누지 않음; 차이는 **컴파일 타깃(AMI 정의)에 따른 타 모듈** 및 **런타임 `dominant_row_` 값** 등으로 간접 전달.

---

## Q2. `walsh_shift` 전달 — `current_walsh_shift_()` → `Decode*_IR`

### 식 (`HTS_V400_Dispatcher.hpp`)

```340:342:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher.hpp
        [[nodiscard]] uint8_t current_walsh_shift_() const noexcept {
            return static_cast<uint8_t>(dominant_row_ ^ 63u);
        }
```

- **의미**: `walsh_shift = dominant_row_ ⊕ 63` (6비트 전제).

### `try_decode_` 내 호출 시점

- **L342**: 디코드 시도 **마다 1회** 읽어 `walsh_shift_payload`에 저장 후, `Decode16_IR` / `Decode64_IR`에 전달 (**L392**, **L475**).

### 디코드 **이전** 가공 여부 (호출부)

- **Payload 쪽**: `current_walsh_shift_()` 결과를 **다시 XOR/클램프하지 않고** 그대로 전달.  
- **헤더 경로** (`HTS_V400_Dispatcher_Sync.cpp`): `READ_HEADER`에서 심볼 보정 등에 **`current_walsh_shift_()`** 재사용 (**L1626**, **L1691** 등) — 페이로드 IR 디코드와 **동일 `dominant_row_` 소스**를 공유.

---

## Q3. IR 누적 버퍼 (`harq_*`, `ir_chip_*`) — AMI 전용 크기 분기 여부

### 레이아웃 (`HTS_V400_Dispatcher_Internal.hpp`)

```36:40:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher_Internal.hpp
struct V400HarqCcmIr {
    int16_t chip_I[FEC_HARQ::NSYM64][FEC_HARQ::C64];
    int16_t chip_Q[FEC_HARQ::NSYM64][FEC_HARQ::C64];
    FEC_HARQ::IR_RxState ir_state;
};
```

- **크기**: `FEC_HARQ::NSYM64` × `FEC_HARQ::C64` **고정 2D 배열** (AMI/PS-LTE **조건부 크기 변경 없음**).

### 포인터 연결 (`HTS_V400_Dispatcher_Core.cpp` ctor)

```70:72:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher_Core.cpp
      harq_Q_(g_harq_ccm_union.chase.harq_Q),
      ir_chip_I_(&g_harq_ccm_union.ir.chip_I[0][0]),
      ir_chip_Q_(&g_harq_ccm_union.ir.chip_Q[0][0]),
```

- **`ir_chip_I_` / `ir_chip_Q_`**: `g_harq_ccm_union.ir.chip_*` 선두 요소 주소 — **AMI 전용 축소 버퍼 없음**.

### `Payload.cpp`에서의 누적

- `ir_chip_I_[base + c]` 등 (**예: L180, L290**) — `base`는 심볼·칩 인덱스 기반; **버퍼 물리 크기는 HARQ 상수**에 의존 (조사 범위에서 **AMI 32-chip에 맞춘 별도 IR 배열 분기는 없음**).

---

## Q4. `try_decode_` 내 AMI 분기

- **`HTS_V400_Dispatcher_Payload.cpp` 전체에 `HTS_TARGET_AMI` 없음** → **`try_decode_`에 AMI 전용 `#if` 분기 없음**.  
- DATA 모드에서 **`ir_mode_`가 참**이면 **항상** `Decode64_IR` 경로 (**L448~L475**); 거짓이면 Chase 쪽 (`rx_.m64_I` 등, L503 이후).

---

## Q5. Payload 실패 지점 추적 (`ami_step2_run1.log`)

| 패턴 | 관측 |
|------|------|
| **`[HDR-`** | **350790**건 — 헤더 단계 로그 대량 존재. |
| **`[Decode64_IR entry]`** | 존재 (예: L837 근처; PowerShell이 exe stderr를 한 줄로 감싼 형태도 혼재). |
| **`CRC` / `crc_ok` / `[DECODE`** | **전용 태그는 거의 없음** — 상위 요약에 `CRC+` 열은 있으나 per-trial CRC printf는 제한적. |

**탈락 지점 추정 (로그 + 코드)**  
1. **`Decode64_IR`까지는 진입** (`[Decode64_IR entry]`·`[PAYLOAD-SHIFT]` 다수).  
2. **`success_mask`/memcmp 최종 PASS**에서 대부분 탈락 — Step 2에서도 **230/15200 불변**이므로, **`walsh_shift` 전달 후에도 FEC+비트 복원 또는 라운드 결합 조건이 AMI 채널과 여전히 불일치**하는 쪽이 유력.  
3. **HARQ 내부**(`walsh_shift` 기반 `fec_ir_fwht_bin_unshift` 등)는 **`HTS_FEC_HARQ.cpp`에 집중** — 지시서상 **수정 불가**이면, 다음 단계는 **호출 인자 일관성**(예: 헤더 vs 페이로드 타이밍의 `dominant_row_` 스냅샷), **`nsym_ir`/`bps`/`ir_state_` 준비**, **AMI `on_sym_` → `ir_chip_*` 적재 패턴** 등 **호출부·준비부** 검증이 맞음.

---

## 종합 체크리스트

| 항목 | 판단 |
|------|------|
| **A. HARQ 내부 문제** | **가능성 유지** — `walsh_shift` 실제 소비·IR FWHT unshift는 `HTS_FEC_HARQ.cpp` 내부. **파일 미터치** 전제면 “원인 후보”로만 기록. |
| **B. 호출부 문제** | **조사 가능** — `try_decode_`는 AMI 분기 없이 동일 경로; **`dominant_row_`/`walsh_shift_payload`/헤더·페이로드 순서**·**`ir_chip_*` 채움**은 전부 **Dispatcher 측**. |
| **C. 복합** | **가장 그럴듯** — Step 2로 **shift 분산은 확인**되었으나 **통과율 불변** → 내부 결합 + 호출부 준비 **동시 의심**. |

---

## 다음 Step 권고 (구현 지시 전)

1. **`dominant_row_` 스냅샷 시각**: `try_decode_()`의 **L342** 시점과 `READ_HEADER` **L1626/L1691** 시점이 **동일 패킷 기준으로 일관되는지** (경쟁 업데이트·리셋 여부).  
2. **`on_sym_` IR 분기** (`Payload.cpp` **L104~L324**): `ir_chip_*`에 쓰는 **`nc` / `base` / FWHT pre-path**가 **AMI 32-chip 전제**와 정합하는지 (버퍼 크기는 64칩 스펙 고정).  
3. **로그**: `[PAYLOAD-SHIFT]` 다음 **성공/실패를 가르는 기존 DIAG**만 추가 활용 (HARQ 파일 printf는 지시서 범위 밖이면 grep으로만).

---

**금지 준수**: 코드 수정·빌드·실측 없음.
