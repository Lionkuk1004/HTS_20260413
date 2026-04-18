# RESULT — PROMPT 47.1 Phase 1 (SPEC / Math_Defense v1.1)

**날짜**: 2026-04-19  
**기준 문서**: `HTS_SPEC_002_Math_Defense_v1.1` (사용자 제공), v1.0 폐기  
**구현 경로**: `HTS_TEST/t6_sim/HTS_Jammer_STD.{hpp,cpp}`, `HTS_Jammer_STD_UnitTest.cpp`

---

## 1. 구현 변경 사항 (v1.0 → v1.1)

| 항목 | 내용 |
|------|------|
| `ch_j3_pulse` | **§7.2**: `period_chips`를 실수 `T_period`로 유지, `std::fmod((double)n, T_period)`로 envelope. 정수 `lround`·`max(1, on_len)` 제거. `duty_cycle`은 `[0,1]` 클램프. |
| `ch_j6_swept` | **§10.5**: 위상을 **주기(cycle)** 누적 `phase_cycles += f_inst/f_chip` 후 `p = 2π·phase_cycles + φ₀` — 수학적으로 기존 라디안 누적과 동일. |
| `ch_j2_cw` / `ch_j5` | **§6.1 / §9.3**: `A = sqrt(P_j)`, `f_center = 0` 주석만 v1.1 문구로 정리. CW 진폭 변경 없음. |
| 단위 시험 | **§14.2**: 전력·σ² **1%**, Parseval **1e-6**, J2 인접(±2, 피크 제외) **≥40 dB**, J3 duty **±0.01**, SatStats **nullptr 금지**, **`[TRIAL-SEED]`** 출력, Parseval **J1~J6** (각 첫 `2^k` 샘플, radix-2 FFT), J5 **정확 f_k DFT**로 톤당 전력, J6 **wrap 관측** 및 **위상 미분 vs f_inst** 검증. |

---

## 2. 각 jammer 수치 (로컬 빌드·실행 후 원문으로 채울 것)

> **자진 고지**: 본 Cursor 작업 환경(Windows)에 **`cl` / `g++` / WSL 컴파일러가 PATH에 없어** `HTS_Jammer_UnitTest.exe`를 여기서 실행하지 못했습니다. 아래 표는 **코드·시험 조건에서 기대되는 항목**이며, **실측 숫자는 사용자 PC에서 빌드 후 로그 원문으로 대체**해야 Phase 2 게이트에 적합합니다.

### 2.1 빌드·로그 생성 (로컬)

```bat
call "%ProgramFiles%\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat"
cd /d D:\HTS_ARM11_Firmware\HTS_LIM\HTS_TEST\t6_sim
cl /nologo /O2 /std:c++17 /EHsc /MD /W3 /FeHTS_Jammer_UnitTest.exe HTS_Jammer_STD_UnitTest.cpp HTS_Jammer_STD.cpp /link /nologo
HTS_Jammer_UnitTest.exe > t6_phase1_v11_jammer_unit.log 2>&1
HTS_Jammer_UnitTest.exe > t6_phase1_v11_parseval.log 2>&1
```

회귀(`10927/11200` 유지)는 기존과 동일하게 `HTS_T6_SIM_Test_V34_FIXA3.exe` 출력을 `t6_phase1_v11_regression.log`로 저장.

### 2.2 수치 템플릿 (실행 후 로그에서 복사)

**J1 AWGN**

- N = `100000` (코드), Parseval용 접두 = `65536`
- P_s = `10000`, SNR = `10` dB → 기대 σ² = `1000`
- 측정 평균 전력·오차(%): **`t6_phase1_v11_jammer_unit.log`**
- Parseval: **`[PARSEVAL] J1:`** 줄
- Saturation: **`[SATURATION] jammer=J1`** 줄
- Seed: **`[TRIAL-SEED] ... tag=J1`** 줄

**J2 CW**

- f_offset = `50000` Hz, N = `2048`
- Peak bin, peak/adj dB, sep: **`J2 peak_bin=...`** 줄 (인접 ±2, 피크 제외 평균)
- 평균 전력 vs P_j (%): 동일 로그
- Parseval / Sat / Seed: `J2` 태그 줄들

**J3 Pulse**

- 케이스 A: D=`0.25`, T_period=`100`, mode=`0`, JSR_peak=`0`, P_s=`10000`
- 케이스 B: D=`0.5`, T_period=`12.5` (이산 envelope 기대값과 측정 duty 비교)
- ON 구간 전력 vs P_j: **`[J3-PULSE]`** 줄
- Parseval / Sat / Seed: `J3`, `J3b`

**J4 Barrage**

- N=`100000`, P_s=`10000`, JSR=`10` dB → P_j=`100000`
- σ²(평균 전력) 오차, Parseval, Sat, Seed

**J5 Multi-tone**

- tones=`4`, bw=`200000`, P_s=`1e6`, JSR=`3` dB, DFT 길이=`8192`
- 총 전력, **`[J5-MT] tone=k`** 톤별 `terr`
- Parseval, Sat, Seed

**J6 Swept**

- P_j=`1e8`, f_start=`-50000`, f_end=`50000`, rate=`2e8` Hz/s, N=`200000`
- 평균 전력, **`[J6-SWEPT] T_sweep`**, **`wraps_obs`**, **`phase_step max|rel err|`**
- Parseval, Sat, Seed

---

## 3. 회귀 시험 (V34 Fix A3)

- **목표**: `10927 / 11200` (97.6%) 및 기존 `t6_phase1_regression.log`와 동일 경향 유지.  
- **상태**: 본 환경에서 재실행 불가 — 사용자 측 `t6_phase1_v11_regression.log`로 확인 필요.

---

## 4. 모호점 해소 (v1.1 기준)

- **CW √2**: v1.1 **§6.1** `A = sqrt(P_j)` 만 사용 — 구현 일치.
- **Swept**: v1.1 **§10** sawtooth `f_inst` + 연속 위상 누적 — 구현 일치(주기 변수만 명시적 분리).
- **Pulse T_period**: v1.1 **§7.2** 실수 `fmod` — 구현 반영 완료.
- **J5 f_center**: v1.1 **§9.3** `0` Hz 고정, 별도 API 불필요 — 구현 유지.

---

## 5. 발견·주의 (추가 자진 고지)

1. **컴파일 미실행**: 위 표의 수치는 **로그 미첨부**이며, tol 1%·J2 40 dB·J6 위상 검증은 **실측으로만** 최종 확정 가능.
2. **J3 duty vs D (±0.01)**: `T=100`, `D=0.25`처럼 **이산 기대 duty = D**인 경우에만 직접 비교. `T=12.5` 등은 **이산 envelope 기대값**(`expected_pulse_on_fraction`)과 측정을 비교하는 보조 케이스를 추가함.
3. **J5 톤 전력**: FFT 빈 에너지 분할 대신 **정확 f_k 상관(DFT)** 으로 `P_k ≈ P_j/N` 검증 — 스펙 §9.2 의도에 맞춤.
4. **J6 순간 주파수**: `atan2` 기반 unwrap은 **진폭이 작은 구간**에서 불안정할 수 있음. 구현에서 **f_inst가 칩 간에 wrap 되는 구간**은 검증 루프에서 스킵함.

---

## 6. 완료 체크리스트 (코드 반영 상태)

- [x] `ch_j3` 실수 `T_period` + `fmod` (§7.2, §7.4 ON=0 허용)
- [x] `ch_j6` v1.1 §10.5 위상 누적 형태
- [x] 단위 시험 tol 1%, J2 ≥40 dB (인접), Parseval 6/6, SatStats, `[TRIAL-SEED]`
- [x] J3 ON 전력, J5 톤별 DFT, J6 wrap·위상-미분 검증
- [ ] **실행 로그 3종** (`t6_phase1_v11_*.log`) — **로컬 빌드 필요**
- [ ] **회귀 로그** — **로컬 재실행 필요**

---

**v1.0 폐기 준수**: 이 문서는 v1.1 기준 결과이며, 구 `RESULT_PROMPT_47_PHASE1.md`의 수치는 **재검증 전** 참조용으로만 취급.
