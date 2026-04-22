# AMI 시나리오별 정밀 분석 (모든 파이프라인 활성)

**전제 (지시서 준수)**  
- 분석만 수행: 소스/빌드 스크립트 변경 없음, `ami_run*.log` / `baseline_run1.log` / 하네스 코드 재검증.  
- AMI 정식 빌드: `HTS_TEST/t6_sim/cursor_t6_build_ami.cmd` — `/DHTS_DIAG_PRINTF`, `/DHTS_TARGET_AMI` 사용, **`/DHTS_FEC_POLAR_DISABLE` 없음** (Polar 비활성 빌드 아님).  
- 브랜치: `ami_dry_run` 가정.  
- 로그: `ami_run1.log`, `ami_run2.log`, `ami_run3.log`, `baseline_run1.log` (이미 존재하는 산출물).

---

## Q1. T6 시나리오 매트릭스

**요약**

- 하네스에는 **S0가 없음**. `main()`에서 **`test_S1` … `test_S10`** 순으로 실행된다.  
- **시나리오 “행”(요약 테이블 한 줄)** 개수: **53행** (`S1` 1 + `S2` 8 + `S3` 9 + `S4` 7 + `S5` 8 + `S6` 4 + `S7` 7 + `S8` 5 + `S9` 1 + `S10a` 1 + `S10b` 1 + `S10c` 1 = 53).  
- **트라이얼 수**: 대부분의 행에서 **`kTrials = 100`**; `S10a`만 **`N_ENDURE = 10000`**.  
- **총 트라이얼 수 (grand_total 검증)**  

  `100 + 800 + 900 + 700 + 800 + 400 + 700 + 500 + 100 + 10000 + 100 + 100 = 15200` → **Yes** (요약 로그 `15200`과 일치).

**시나리오 표 (코드 기준)**

| # | ID | 의미 | CFO(Hz) | SNR / 채널 | LPI | Offset / 기타 |
|---|-----|------|---------|------------|-----|----------------|
| — | (없음) | S0 미정의 | — | — | — | — |
| 1 | S1 | 클린 채널 무결성 | 0 | 이상 없음 | 없음 | `feed_raw_ext` 기본 가드 `kGuard=256`만 |
| 2 | S2 | 위상 맹 획득 (호스트 회전) | 0 | 클린 + `ch_rotate` 0°…315° (8단) | 없음 | 없음 |
| 3 | S3 | 심해 SNR 워터폴 | 0 | AWGN `-30 … +10` dB (9점) | 없음 | 없음 |
| 4 | S4 | 비동기 타이밍 오프셋 | 0 | 클린 | 없음 | **`kGuard + off`**, `off ∈ {0,1,5,17,31,63,127}` 칩 선행 0 패딩 |
| 5 | S5 | 극한 CFO (블라인드) | **`ch_cfo`**: 0, 50, 100, 200, 500, 1000, 2000, 5000 | 클린 | 없음 | 위상 `ph = 2π·cfo·i / kChipRate` (샘플 인덱스 `i`) |
| 6 | S6 | 3-tap 다중경로 | 0 | ISI 4케이스 | 없음 | 탭 지연/진폭 표 참고 |
| 7 | S7 | 바라지 재밍 | 0 | `ch_barrage`, JSR **0…30 dB** (7점) | 없음 | 없음 |
| 8 | S8 | CW 톤 재밍 | 0 | JSR/주기 5케이스 | 없음 | 라벨: JSR+10 P=8 등 |
| 9 | S9 | 복합 스트레스 | 호스트 `ch_cfo` 1500 Hz + multipath + AWGN -15 dB | 복합 | 없음 | `kGuard + (rng % 31)` |
| 10 | S10a | 내구 10000회 | 0 | 클린 | 없음 | `N_ENDURE = 10000` |
| 11 | S10b | Holo LPI OFF | 0 | 클린 | **OFF** | `kTrials` |
| 12 | S10c | Holo LPI ON | 0 | 클린 | **ON** (TX/RX 시드) | `kTrials` |

**코드 근거 (발췌)**

- `kGuard`, S4 오프셋, S5 CFO 배열:

```72:72:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_TEST\t6_sim\HTS_T6_SIM_Test.cpp
static constexpr int      kGuard     = 256;
```

```648:673:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_TEST\t6_sim\HTS_T6_SIM_Test.cpp
    const int offsets[] = {0, 1, 5, 17, 31, 63, 127};
    for (int off : offsets) {
        ...
            const TrialMetrics m =
                feed_raw_ext(ds, tx.I, tx.Q, tx.n, tx.info, kGuard + off);
```

```717:745:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_TEST\t6_sim\HTS_T6_SIM_Test.cpp
    const double cfos[] = {0, 50, 100, 200, 500, 1000, 2000, 5000};
    for (double cfo : cfos) {
        ...
            ch_cfo(rI, rQ, tx.n, cfo);
            const TrialMetrics m = feed_raw_ext(ds, rI, rQ, tx.n, tx.info);
```

```335:342:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_TEST\t6_sim\HTS_T6_SIM_Test.cpp
static void ch_cfo(int16_t* I, int16_t* Q, int n, double cfo_hz) noexcept {
    for (int i = 0; i < n; ++i) {
        const double ph = 2.0 * kPi * cfo_hz * i / kChipRate;
```

- 칩레이트: 하네스 상수 **`kChipRate = 200000.0`** (200 kcps) — `ch_cfo` 위 식과 일치.

---

## Q2. AMI 시나리오별 분포 (`ami_run1.log`)

**정량 합계 (3회 재현)**  
- `ami_run1.log` / `ami_run2.log` / `ami_run3.log` 모두: **`230 / 15200 (1.5%)`**, 전체 BER **`0.97171`** — **동일** (재현성: 편차 없음).

**시나리오별 PASS / TOTAL (요약 테이블 라인 기준, FAIL=total−pass, PART는 pass가 0과 total 사이)**

| Scenario | 조건 (로그 param) | trial | PASS | FAIL | PART | 비율 |
|----------|-------------------|------:|-----:|-----:|:----:|------|
| S1 | B0/OK0/CRC+0/BE6400 | 100 | 0 | 100 | 0 | 0% |
| S2 | 각 위상 0°…315° | 100×8 | 0 | 800 | 0 | 0% |
| S3 | -30…-20 dB | 100×5 | 0 | 500 | 0 | 0% |
| S3 | -10 dB | 100 | 4 | 96 | 0 | 4% |
| S3 | -5 dB | 100 | 8 | 92 | 0 | 8% |
| S3 | 0 dB | 100 | 3 | 97 | 0 | 3% |
| S3 | 5 dB | 100 | 1 | 99 | 0 | 1% |
| S3 | 10 dB | 100 | 0 | 100 | 0 | 0% |
| S4 | +0,+1,+5,+17,+31 | 100×5 | 0 | 500 | 0 | 0% |
| S4 | **+63** | 100 | **100** | 0 | 0 | **100%** |
| S4 | **+127** | 100 | **100** | 0 | 0 | **100%** |
| S5 | **0Hz … 5000Hz 전 구간** | 100×8 | **0** | **800** | 0 | **0%** |
| S6 | 4케이스 | 100×4 | 0 | 400 | 0 | 0% |
| S7 | 0,5,10 dB JSR | 100×3 | 4+7+2 | 나머지 | 0 | PART 구간 |
| S7 | 15…30 dB | 100×4 | 0 | 400 | 0 | 0% |
| S8 | 5케이스 | 100×5 | 0 | 500 | 0 | 0% |
| S9 | Full | 100 | 0 | 100 | 0 | 0% |
| S10a | Endurance | 10000 | 0 | 9998 | (crc_only 2) | FAIL |
| S10b | LPI-OFF | 100 | 0 | 100 | 0 | 0% |
| S10c | LPI-ON | 100 | 1 | 99 | 0 | 1% |

**230 구성 검증**: S3 합 16 + S4 200 + S7 합 13 + S10c 1 = **230**.

---

## Q3. Phase 0 vs Payload 분리

### 관측 가능한 DIAG 제약 (중요)

`HTS_DIAG_PRINTF`가 켜져 있어도,**`HTS_TARGET_AMI`가 정의되면 `[STAGE4-JUDGE]` / `[STAGE4-SCAN]` printf 경로가 컴파일에서 제외**된다.

```801:819:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher_Sync.cpp
#if defined(HTS_DIAG_PRINTF) && !defined(HTS_PHASE0_WALSH_BANK) && \
    !defined(HTS_TARGET_AMI)
    if (diag_this_scan) {
        std::printf(
            "[STAGE4-SCAN#%d] ...
        std::printf(
            "[STAGE4-JUDGE#%d] best_off=%d best_e63=%d e63_min=%d "
            "r_avg_ok=%d sep_ok=%d pass=%d sum_others=%lld\n",
```

따라서 지시서에 있던 **`pass=0` / `pass=1` 카운트 기반 Phase0 통계는 AMI 로그로는 산출 불가** (로그에 문자열 부재).

### 로그에서 확인되는 대체 신호 (기존 printf만)

- **`[P0-SCAN]`**: `ami_run1.log`에 대량 존재 (grep 카운트 수만 위급; **시나리오/트라이얼과 1:1 매핑 없음**).  
- **`[PAYLOAD-SHIFT] walsh_shift=0 dom=63`**: 샘플 및 전체 grep 기준 **동일 패턴 13458회** — 페이로드 IR 경로가 실행되며 **walsh_shift는 항상 0, dominant는 63으로 고정 출력**되는 경향.  
- **`[Decode64_IR entry]`** 등: 디코더 진입은 있으나, **“CRC PASS 전용” 한 줄 태그**는 지시 범위에서 추가 검색하지 않음 (기존 패턴만 사용).

### Phase0 vs Payload 표 (추정 한계 명시)

| Scenario | Phase0 (로그 직접) | Payload (로그 직접) | 주 실패 (근거) |
|----------|-------------------|---------------------|----------------|
| 전체 | STAGE4 불가 → **미집계** | CRC 한 줄 태그 부재 → **미집계** | **엔드투엔드 `m.pass`만 확정** (표 Q2) |
| S1 | 동일 | 동일 | 클린에서도 0% → **동기/복조 전 구간** 실패 가능성 큼 (CFO만으로 설명 불가) |
| S4 +63/+127 | (추정) P0 정렬이 Walsh 주기와 공명 | 이후 FEC/memcmp까지 성공 | **타이밍 오프셋이 k_w63 구조와 정합한 경우만** 전체 통과 |
| S5 0Hz | (추정) CFO 외 요인 | 0/100 | **2000Hz 특수 현상 아님** — **0Hz 포함 전 CFO에서 실패** |

---

## Q4. S4 성공 분석 (+63 / +127)

**하네스 파라미터**

- 채널: 클린 (`build_tx` 직통).  
- 수신기 앞 선행 0 패딩: **`kGuard + off`** 칩 (`kGuard=256`).  
- `off`는 **추가 칩 지연**으로, P0 스캔 윈도우/128칩 프리앰블 정렬과 **상대 위상·边지**를 바꿈.

**+63 / +127 의미**

- 코드상 단순히 **“양의 타이밍 오프셋 칩 수”**이며, 별도 CFO/SNR/LPI 없음.  
- **`k_w63`**: H_64의 **63번 행**에 해당하는 **64칩** 시퀀스이며, ROM은 길이 64의 `int8_t k_w63[64]`로 정의됨. 주석상 **8칩 단위 자기유사**가 있어, 짧은 구간 CFO/정렬에서 **row 7 누적** 등 부작용이 문서화되어 있음.

```67:78:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher_Internal.hpp
/// @brief Walsh-Hadamard 행렬 H_64의 63번 행.
///        w63[j] = (-1)^popcount(j).  ROM 배치.
inline constexpr int8_t k_w63[64] = {
    +1, -1, -1, +1, -1, +1, +1, -1,
    ...
};
```

**성공 이유 (설계 관점 추정)**

- **+63, +127만 100%**인 것은, P0 에너지 스캔이 **`best_off`·128칩 버퍼 정렬**에서 **k_w63 기반 비코히런트 에너지 피크**를 안정적으로 찍는 **“정렬 해상도의 특이점”**과 일치할 가능성이 큼.  
- **127 = 64 + 63**으로, **64칩 블록 경계와 63칩 Walsh 주기가 겹치는** 조합이 AMI P0 버퍼링(128/192 칩 등)과 맞물릴 여지가 있음 (정량 증명은 별도 실험 필요).

**AMI Phase0 이후 CFO 추정 (코드)**

- `best_off` 확정 후 **블록0/블록1 각 64칩**에 대해 `walsh63_dot_` → **`cfo_.Estimate_From_Preamble(..., 64)`** → **`Advance_Phase_Only(192)`**.

```1038:1050:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher_Sync.cpp
                walsh63_dot_(&p0_buf128_I_[best_off + 0],
                             &p0_buf128_Q_[best_off + 0],
                             d0I, d0Q);
                walsh63_dot_(&p0_buf128_I_[best_off + 64],
                             &p0_buf128_Q_[best_off + 64],
                             d1I, d1Q);
                cfo_.Estimate_From_Preamble(d0I, d0Q, d1I, d1Q, 64);
                ...
                cfo_.Advance_Phase_Only(192);
```

---

## Q5. S5 CFO / “2000Hz 완전 실패” 정밀화

**S5 파라미터**

- 위 **Q1 표**와 동일. **사전 보정 없이** `ch_cfo`만 적용.

**칩레이트**

- **`kChipRate = 200000` Hz** → 칩 주기 **5 µs**.

**CFO 2000 Hz일 때 위상 회전 (관측식과 동일)**

- 칩당: \( \Delta\phi = 2\pi \cdot 2000 \cdot (1/200000) = 2\pi/100 \approx 0.0628 \) rad ≈ **3.6°**  
- **32칩(한 블록)**: \( 32 \times 3.6^\circ \approx 115.2^\circ \)  
- **64칩(두 블록)**: 약 **230.4°** (코드에서 CFO 추정에 **64칩 간격** 사용)

**CFO 처리 경로 (요약)**

- 위치: `HTS_V400_Dispatcher_Sync.cpp` 내 P0 시드 확정 직후 블록 (상기 인용).  
- **`block_chips` 인자 = 64** (`Estimate_From_Preamble` 호출) — **AMI/공통 경로에서 64칩 기준 CFO 추정**임이 코드로 확정.

**“2000Hz만”이 아닌 점 (로그 사실)**

- AMI에서 **S5는 0 Hz를 포함한 모든 CFO 행이 0/100**이다.  
- 따라서 본 장의 제목이 “2000Hz”여도, **실패의 특이점은 2000Hz가 아니라 “AMI에서 CFO 스윕 전체가 클린 채널에서도 엔드투엔드 불통”**으로 요약하는 것이 로그와 모순 없음.  
- 다만 **2000Hz**는 위상 누적이 커져 **CFO 추정/디로테이션 오차가 체감되기 쉬운 운영점**으로, PS-LTE 대비 여유가 없을 때 **먼저 악화되는 지점**으로는 여전히 유효한 분석 축이다.

**Phase0 vs Payload (S5)**

- Q3과 동일하게 **STAGE4 불가** → 행 단위로는 **“전 구간 0%”**만 확정.

---

## Q6. S3 / S7 부분 성공

**S3 (심해 SNR)**

- 조건: AWGN만, SNR -30…+10 dB.  
- AMI: **-10~-5~0~5 dB에서만** 소수 PASS (합 16/900 ≈ 1.78%).  
- 패턴: **중간 SNR에서만 간헐 통과** — 지나치게 낮으면 에너지 부족, 높으면(10 dB) 오히려 0%로 떨어짐 → **단순 단조 “SNR↑=성공”이 아님**; **정렬/게이트/AGC/정수 포화** 등 비단조 효과 가능성.

**S7 (바라지)**

- 조건: JSR 0…30 dB.  
- AMI: **0/5/10 dB에서만** 일부 PASS (4, 7, 2 / 각 100).  
- PS-LTE는 저JSR에서 거의 완주(예: 0 dB 100%)인 반면 AMI는 크게 붕괴 → **재밍이 아니라도( S1 실패) 바닥이 낮고**, 약한 재밍에서만 **우연적 정렬**이 나오는 형태에 가깝다.

---

## Q7. PS-LTE vs AMI (같은 시나리오 요약)

**대표 비교 (로그에서 직접 대조)**

| Scenario | PS-LTE (`baseline_run1.log`) | AMI (`ami_run1.log`) | 차이 요약 |
|----------|------------------------------|----------------------|-----------|
| S1 | 100% PASS | 0% | **근본 갭** |
| S2 각 행 | 100% | 0% | 위상 전수에서 전멸 |
| S3 중간 SNR | 고득점(예: 0 dB 100%) | 수 % 이하 | 큰 격차 |
| S4 +63,+127 | 100% | 100% | **유일하게 동일 만점** |
| S4 기타 오프셋 | 100% | 0% | AMI만 오프셋 민감 |
| S5 전체 | 각 100% | 전부 0% | **완전 역전** |
| S7 저JSR | 고득점 | 수 % 이하 | 역전 |
| S10a | 10000/10000 | 0/10000 | 내구 붕괴 |
| S10b/c | 100% | 0% / 1% | LPI 포함 열세 |

**관측 한 줄**

- PS-LTE가 압도하는 구간: **거의 전부** (특히 S1, S5, S10).  
- AMI가 PS-LTE와 **같은 수준**인 곳: **S4의 +63, +127 두 행뿐**.  
- 격차 최대: **S1, S5(0Hz 포함), S10a** 등 “클린에 가까운” 조건에서의 AMI 0%.

---

## 종합 — β 방향 권고

### 붕괴 원인 분류 (증거 기반)

**원인 A — Phase0 / 타이밍 정렬 (피크·best_off·가드 소비)**  
- 영향: **S4**에서 오프셋 민감도 극대화(+63/+127만 성공), **S1~S2** 전멸.  
- 근거: 클린·무회전에서도 AMI 0%; S4는 Walsh 주기성과 맞는 오프셋만 통과.

**원인 B — CFO 추정·위상 연속성 (`Estimate_From_Preamble` 64칩, `Advance_Phase_Only(192)`)**  
- 영향: **S5 전역** 및 이후 페이로드 디로테이션.  
- 근거: 코드 경로 확정; 2000Hz는 위상 누적이 큰 스트레스 포인트. 다만 **S5 0Hz도 AMI 0%**이므로 CFO만의 단일 근원으로는 부족.

**원인 C — Payload IR / `walsh_shift` 고정 (`[PAYLOAD-SHIFT] walsh_shift=0 dom=63` 대량)**  
- 영향: **S3/S7 간헐 성공** 포함 전 시나리오.  
- 근거: 로그상 디코더는 도는데 **shift가 0으로만 관측**; PS-LTE 대비 적응이 막혀 있을 가능성.

**원인 D — 기타 (정수 포화, AGC, 게이트 임계, barrage/CW 별도 경로)**  
- 영향: S8/S9/S6 등에서 추가 악화 요인.

### β 방향 재선택 (체크 권고)

- **[x] β-3 (AMI 전용 dominant / best_half·Walsh bank 등 “AMI에 맞는” 정렬 지표)**  
  - 근거: **S4의 Walsh-주기 정렬 특이점** + **payload 쪽 dom=63 고정**.  
  - 예상 개선: 오프셋 일반화, 저SNR/재밍에서의 간헐 성공을 **체계적 PASS**로 끌어올릴 여지.

- **[x] β-2 (`walsh_shift` 고정이 아닌 CFO·위상 연속성 / 64 vs 실제 P0 윈도우 정합)**  
  - 근거: `Estimate_From_Preamble(...,64)` 및 `Advance_Phase_Only(192)`는 **강한 가정**; S5 전역 실패와 정합 가능.  
  - 예상 개선: CFO 스윕에서 **최소 0Hz 근처부터** 회복되면 S5·S10이 함께 움직일 가능성.

- **[ ] β-1 (PS-LTE와 동일한 FWHT/dominant_row를 AMI에 그대로 이식)**  
  - 근거: AMI는 **의도적으로 다른 P0 경로**를 타며, STAGE4 DIAG도 분기됨 — “그대로 이식”은 충돌/중복 위험. **β-3가 먼저**일 수 있음.

- **[x] β-복합 (β-3 정렬 지표 + β-2 CFO 연속성 순)**  
  - 순서: **(1) 클린 S1 회복 목표** → **(2) S4 일반 오프셋** → **(3) S5 0Hz→고Hz** → **(4) S10 내구**.

### Stage A-0 실험 권고 (측정만 가능한 현재 단계 이후)

- **실험 변경**: (코드 변경은 지시서 범위 밖이므로 권고만) 최소 단위로 **β-2 또는 β-3 중 하나**를 선택해 A/B.  
- **측정 기준**: `cursor_t6_build_ami.cmd` 동일, **S1→S5→S4 순**으로 회복 판정.  
- **예상 결과**: S1이 먼저 0→비0으로 움직이면 원인 A/B 쪽이 맞음; S1 정체되고 S4만 개선되면 β-3 쪽.  
- **롤백 기준**: S4 +63/+127 **회귀(100% 유지)** 깨질 때 즉시 롤백 검토.

---

## 부록: 빌드 플래그 (파이프라인 활성 확인)

```9:18:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_TEST\t6_sim\cursor_t6_build_ami.cmd
cl /nologo /O2 /std:c++17 /EHsc /MD /W3 ^
   ...
   /DHTS_DIAG_PRINTF ^
   /DHTS_TARGET_AMI ^
```

**끝.**
