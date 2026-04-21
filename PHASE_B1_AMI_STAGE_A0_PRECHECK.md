# AMI Stage A-0 사전 조사

**조건**: 읽기 전용. 코드 수정 없음.  
**기준 브랜치**: **`main`** (`git checkout main` 후 조사).  
**소스 형태**: `main` 은 **`HTS_LIM/HTS_V400_Dispatcher.cpp` 모놀리식** (`HTS_V400_Dispatcher_Sync.cpp` 없음). **분할 트리**(예: `ami_dry_run`)에서는 동일 블록이 `HTS_V400_Dispatcher_Sync.cpp` 로 옮겨졌으며 **전처리 구조는 동등**함(이전 세션에서 `Sync.cpp` 라인이 `Dispatcher.cpp` 와 대응됨).

---

## Q1. `dominant_row_` AMI 영향

### Q1-A: `dominant_row_` 사용 지점 (관측)

`HTS_LIM` 내 `*.cpp`/`*.hpp` grep **`dominant_row_`**: **`HTS_V400_Dispatcher.hpp` 2회**, **`HTS_V400_Dispatcher.cpp` 18회**(합계 **20**).

**`HTS_V400_Dispatcher.cpp` 주요 라인(용도)**:

| 라인(대략) | 용도 |
|------------|------|
| `1017` | `full_reset_` 등에서 `dominant_row_ = 63u` |
| `1494`, `1576` | `try_decode_` 내 DIAG: `dominant_row_` 출력 |
| `3099` | `phase0_scan_` 성공 시 `dominant_row_ = static_cast<uint8_t>(best_dom_row)` |
| `3104`, `3159` | P0 확정 후 DIAG |
| `3355`–`3370` | Phase1 **PS-LTE/WALSH** 경로: `dominant_row_`로 `sym1_row` 등 |
| `3439`–`3490` | PS-LTE FWHT Phase1 및 WALSH DIAG (`sym1_row_u` XOR `k_W63_FWHT_ROW`) |
| `3472` | `[P1-E]` DIAG — **AMI 빌드에서도** `dominant_row_` 출력 인자로 사용 |
| `3535`, `3677`, `3826`, `3851`, `3892` | 헤더/심볼 처리·`[HDR-SHIFT]` 등에서 `dominant_row_` 또는 XOR 체인 |

### Q1-B: `current_walsh_shift_()`

- **정의**: `HTS_V400_Dispatcher.hpp` **`328`–`329`** (`dominant_row_ ^ 63u`).
- **호출(관측)**: `HTS_V400_Dispatcher.cpp` **`1438`**, **`3667`**, **`3732`**, **`3882`** (`walsh_shift_payload` / 지역 `walsh_shift`).

### Q1-C: Payload decode (`try_decode_`)

- **`1438`**: `const uint8_t walsh_shift_payload = current_walsh_shift_();`
- **`1485`–`1488`**: `FEC_HARQ::Decode16_IR(..., walsh_shift_payload)`
- **`1568`–`1571`**: `FEC_HARQ::Decode64_IR(..., walsh_shift_payload)`
- **DIAG**: `1492`–`1494`, `1574`–`1576` 에 `walsh_shift`·`dom` 동시 출력.

**관측**: IR 페이로드 디코드 경로는 **타깃(AMI/PS-LTE) 분기 없이** 동일하게 `walsh_shift_payload` 를 넘김.

### Q1-D: `Decode*_IR` 에 전달되는 값

- **식**: `current_walsh_shift_() = (uint8_t)(dominant_row_ ^ 63u)`.
- **`dominant_row_ == 63`** 이면 **XOR 결과 `0`** (이전 조사와 동일).

### Q1 판정

- [x] **AMI에서 `dominant_row_` 값은 Payload IR decode 및 HDR XOR 경로에 의미 있음** (`walsh_shift` 가 0이 아닐 때 `fec_ir_fwht_bin_unshift` 등 Stage 6B 경로가 의미 있음).
- [ ] 하위에서 전혀 사용 안 됨 — **부정**(위 참조).
- [ ] 불명확 — **부정**.

**추정:** Phase1 **AMI 전용 에너지 블록**(`3381`–`3425`)은 **`dominant_row_`를 읽지 않음** — P1 게이트 에너지는 **32+32 비코히런트**로만 계산. 그러나 **동일 심볼 처리 이후 공통 경로**에서 `dominant_row_`·`walsh_shift` 는 여전히 사용 가능.

---

## Q2. `#elif !defined(HTS_TARGET_AMI)` 블록 의미

### Q2-A: 블록 전문 (복붙)

**파일**: `HTS_LIM/HTS_V400_Dispatcher.cpp` **`2715`–`2816`** (`#elif !defined(HTS_TARGET_AMI)` … `#endif` 직전까지).

```cpp
#elif !defined(HTS_TARGET_AMI)
            // Fix A1 (V28): 3-way 일치 시 r0, 아니면 r2.
            // Fix A3 (V34): block-2 FWHT 에너지에서 coherent pair 순차 excision
            // (bin63 하드코딩 없음). pair1=(top1,top2) → pair2=(top3,top4) 검사,
            // 2-level jam이면 top5, 1-level이면 top3를 preamble 후보로 채택.
            const bool full3blk_dom = ((off + 128 + 63) < 192);
            int best_dom_pick = (r0 == r1 && r1 == r2) ? r0 : r2;
            if (full3blk_dom) {
                int64_t e_b2[64];
                for (int m = 0; m < 64; ++m) {
                    e_b2[m] = static_cast<int64_t>(T2_I[m]) * T2_I[m] +
                              static_cast<int64_t>(T2_Q[m]) * T2_Q[m];
                }
                int t1_bin = 0;
                int64_t t1_e = 0;
                find_argmax_64(e_b2, &t1_bin, &t1_e);
                int t2_bin = 0;
                int64_t t2_e = 0;
                int final_pick = best_dom_pick;
                int a3_level = 0;
                // ... pair1_coherent / pair2_coherent 분기 ...
                best_dom_pick = final_pick;
            }
            best_dom_row = best_dom_pick;
            // ... DIAG printf ...
#endif
```

(중간 생략은 위와 동일 파일 참조.)

### Q2-B: 기능

- **3-block consistency**: 주석상 Fix A1 — `r0`,`r1`,`r2` 3-way 일치 시 `r0`, 아니면 `r2` 기본.
- **`best_dom_pick`**: 블록2(`T2_*`) FWHT 에너지 `e_b2` 상에서 **다중 argmax·coherent pair** 로 후보 행 확장(Fix A3).
- **`best_dom_row`**: 최종 `best_dom_pick` 대입.

### Q2-C: AMI vs PS-LTE

| 구분 | 내용 |
|------|------|
| **AMI** (`2394`–`2420`) | 32-chip×2 half, `k_w63` 가중 누적 **`e_nc`**, `accum = e_nc>>16`. **FWHT 없음**. **`r0`,`T2_I` 등 없음**. |
| **PS-LTE** (`2421`–`2698` 일대) | 3×64 FWHT, `r0`,`r1`,`r2`, `T0/T1/T2`, `accum` 다중 스케일. |
| **공통** | `accum > best_e63` 시 `best_off`/`best_e63` 갱신; 그 안에서만 `best_dom_row` 설정. |

**`#elif` 제거만으로 AMI에 위 PS-LTE 블록을 붙이는 효과**를 내려면: **AMI 번역 단위에 없는 `r0`,`r1`,`r2`,`T2_I`,`T2_Q` 참조** → **AMI 빌드에서 컴파일 오류**(관측).

---

## Q3. AMI 구조와 Stage 6 호환

### Q3-A: AMI Phase0 블록 (요약)

`2394`–`2420`: 블록 2개 × half 2 × 32칩, **`k_w63`** XOR 누적 후 **`dI²+dQ²`** 합산 → **`accum`**.

### Q3-B: 에너지·`k_w63`

- **비코히런트**: half 단위 32-chip Hadamard-style 합 `dI,dQ` 후 제곱합.
- **`k_w63`**: 인덱스 `widx = (half<<5)+j`.

### Q3-C: FWHT

- **Phase0 AMI**: **`fwht_64_complex_inplace_` 호출 없음**(위 블록).
- **Phase1 AMI** (`3381`–`3425`): 역시 **FWHT 없음**, 시간도메인 칩에 **`k_w63`** 직접 적용.

### Q3 판정

- [ ] AMI가 **FWHT max-row 의미의 `dominant_row`** 와 1:1 대응 — **부정**.
- [x] **혼합**: P0/P1 AMI는 **자체 비코히런트 지표**; **그러나** P0 끝에서 **`dominant_row_`는 여전히 IR/HDR에 전달**(Q1).

**추정:** Stage 6의 **FWHT-bin unshift**는 `walsh_shift≠0`일 때 의미가 큼. AMI에서 **shift=0 고정**이면 **Stage 6B 효과가 상쇄**될 수 있음(**추정**).

---

## Q4. `#elif` 제거(단순) 안전성

### Q4-A: 변수 의존성

`#elif` 블록은 **`r0`,`r1`,`r2`,`T2_I`,`T2_Q`,`max_row`,`stage4_b1_row`** 등을 사용. 이들은 **동일 `for (off…)` 루프 안의 PS-LTE `#else` 분기**(`2421`~)에서만 설정됨.

**AMI 빌드**에서는 해당 `#else` **전처리 제외** → 식별자 **미정의**.

### Q4-B: 판정

- [ ] `#elif` 직접 제거만으로 AMI에 PS-LTE 로직 이식 **안전** — **부정**(컴파일 실패).
- [x] **AMI 전용 `best_dom_row` 산출**(예: 비코히런트 피크에서 행 유사량 정의) 또는 **조건부로 PS-LTE FWHT 전체를 AMI에서도 수행**하는 등 **별도 설계** 필요 — **관측 기반**.

---

## Q5. AMI 붕괴 시나리오별 분포 (로그)

**출처**: `HTS_TEST/t6_sim/ami_run1.log` (ami_dry_run 실측 산출물; `main`과 무관한 **워크스페이스 파일**).

**정량 합계 (관측)**: **`230 / 15200 (1.5%)`**, BER `0.97171` (이전 dry-run 보고와 동일).

| 구간 | Pass / 100 | 판정 요약 |
|------|------------|-----------|
| S1 | 0 | FAIL |
| S2 (8행) | 각 0 | 전부 FAIL |
| S3 | -15~-20dB: 0; -10:4; -5:8; 0dB:3; 5dB:1; 10dB:0 | 일부 PART |
| S4 | +0~+31: 0; **+63:100**; **+127:100** | PASS 2행 |
| S5 | **0Hz~5000Hz 전부 0** | 전 FAIL |
| S6 | 전부 0 | FAIL |
| S7 | 0dB:4, 5dB:7, 10dB:2; 15dB~:0 | 일부 PART |

**CFO vs 비-CFO**: S5 **전 구간 FAIL** + S2/S6 등 **CFO 무관 FAIL 다수** → **CFO 단독 원인으로는 설명 불충분**(관측).

---

## Q6. Stage A-0 실험 범위·롤백

### Q6-A: `HTS_TARGET_AMI` / `!defined(HTS_TARGET_AMI)` (일부)

`HTS_V400_Dispatcher.cpp` 에서 `!defined(HTS_TARGET_AMI)` 는 **다수**(diag, `best_dom_row` 블록 등). **`#elif !defined(HTS_TARGET_AMI)`** 는 **`2715` 한 곳**(grep).

### Q6-B: 최소 수정 단위 (조사 결론)

- **단일 `#elif` 제거**만으로는 **불충분** — Q4 참조.
- **실질 최소안(설계 측면)**: `best_dom_row` 를 AMI에서도 **의미 있는 uint8_t**로 채우는 **소수 라인 추가** 또는 **AMI 전용 픽 규칙** 분리.

### Q6-C: 롤백

- **한 커밋 revert** 가능 여부는 **변경 산출물 크기**에 따름 — 본 문서는 **구현 전**이라 미확정.

---

## 종합 Stage A-0 실험 설계 권고

### 실험 목적

AMI에서 **`best_dom_row`/`dominant_row_`/`current_walsh_shift_()`** 가 Payload·FEC IR 경로에 **일관된 위상 정보**를 주도록 함.

### 실험 변경 (구현은 별도 지시서)

1. **방향 α (`#elif` 단순 제거)**: **비권장** — Q4로 **컴파일 불가·논리 불일치**.
2. **방향 β (권장)**: **AMI 전용 `best_dom_row`** — 예: P0 비코히런트 에너지에서 **대표 half/블록 인덱스**를 0..63 맵으로 투영하는 규칙을 **명시적으로 새로 정의**(PS-LTE의 `r0/r1/r2` 재사용 아님).
3. **방향 γ**: AMI P0/P1에 **FWHT 기반 행**을 **옵션으로 추가** — 스펙·부하 영향 큼.

### 예상 효과

- **수치 예측**: 본 문서에서 **하지 않음**(실측 필요).

### 위험

- [x] **PS-LTE 회귀**: `main` / 통합 브랜치에서 **`#elif` 블록 변경 시 PS-LTE 경로 오염** 가능.
- [x] **런타임/스펙 리스크**: AMI **32-chip** 모델과 **64-bin FWHT dominant** 의 **1:1 대응 가정**은 코드상 **성립하지 않음**(Q3).

### 권고 체크

- [ ] 방향 α  
- [x] **방향 β**  
- [ ] 방향 γ (필요 시 2단계)

### 대체 실험

- **`walsh_shift`를 `dominant_row_`와 독립된 AMI 전용 레지스터로 분리**하는 설계 검토(장기).

---

## 분할 트리 참고

`#elif !defined(HTS_TARGET_AMI)` 및 인접 AMI/PS-LTE 블록은 **`HTS_V400_Dispatcher_Sync.cpp`** 에서 **동일 패턴**으로 유지됨(`main` 대비 파일만 분리).

---

**브랜치 복귀 (관측)**: 조사 종료 후 **`git checkout ami_dry_run`** 실행함.
