# Monte Carlo Harness 실측 결과

일자: 2026-04-19  
실행: `HTS_TEST/t6_mc/HTS_T6_MC.exe` (기본 시드 `0xABCDEF01`, trial마다 `sample_realization`에서 채널·RNG 분기)

## 빌드

- 스크립트: `HTS_TEST/t6_mc/build_mc_harness.bat`
- 결과: **PASS** (MSVC x64, `/std:c++17`, `HTS_FEC_POLAR_DISABLE`, `HTS_LLR_DIAG`, `HTS_ROW_CONSISTENCY_DIAG`)
- 링크: `HTS_T6_MC_Harness.cpp`, `../t6_sim/HTS_Session_Derive_Stub.cpp`, `../t6_sim/HTS_BER_PER_Measure.cpp` (Clopper–Pearson), `../t6_sim/HTS_Jammer_STD.cpp`, `HTS_Walsh_Row_Converter.cpp`, `HTS_LLR_Diag.cpp`, `HTS_Row_Consistency_Diag.cpp`, 코어 `.cpp`는 하네스 TU末尾 `#include` (T6와 동일 패턴)

## 기존 T6 회귀

- `build_and_test_row_consistency.bat` 재실행: **정량 합계 10927 / 11200 (97.6%)** — 기존 T6 그랜드 합계 유지 확인
- Monte Carlo 실행은 T6 바이너리와 별도 (`HTS_T6_MC.exe`)

## 시나리오별 결과 (N = 500)

원시 로그: 동일 인자로 생성한 `mc_stdout_500.log` (작업 디렉터리 `t6_mc`).

### MC_S1_Clean

- Pass rate: 311 / 500 = 0.6220
- 95% CI (Clopper–Pearson): [0.5779, 0.6647]
- BER (비트): 0.3780
- LLR_BIN: E=30.95, Std=16.56
- LLR_IR: E=1652.82, Std=1102.96
- Row entropy H: E=4.11, Std=0.53
- Row concentration: E=0.331, Std=0.075

### MC_S3_LowSNR

- Pass rate: 11 / 500 = 0.0220
- 95% CI: [0.0110, 0.0390]
- LLR_BIN: E=19.17, Std=11.69
- Row entropy H: E=4.53, Std=0.85

### MC_S7_Barrage (JSR·CFO trial별 샘플, barrage RNG trial별)

- Pass rate: 139 / 500 = 0.2780
- 95% CI: [0.2391, 0.3195]
- BER: 0.7220
- LLR / Row: 표에 상기 (MC harness stdout과 동일)

**T6 S7와의 질적 차이**: T6 S7은 JSR 고정 격자(0~30 dB) × trial 20회이며 barrage용 `std::mt19937`이 **시나리오 루프마다 고정 시드**로 재사용된다. 본 MC 시나리오는 **동일 JSR 구간에서 JSR·CFO·barrage RNG를 trial마다 독립 샘플**하므로 pass rate는 “단일 realization 반복”이 아니라 **조건 분포 위의 기대값 추정**에 가깝다. 수치 비교는 조건 정의가 1:1이 아니므로(예: MC_S7에 Normal CFO σ=200 Hz 추가) 직접 대등 비교는 권장하지 않는다.

### MC_S8_CW_Freq

- Pass rate: 0 / 500
- 95% CI: [0.0000, 0.0074]
- LLR/Row 집계: 복조 실패 위주 trial에서 관측 스냅샷이 쌓이지 않아 **평균 0**에 가깝게 나올 수 있음 (하네스는 “성공 패킷만”이 아니라 trial 끝 시점의 diag 값을 수집)

### MC_S8_CW_Multi

- Pass rate: 8 / 500 = 0.0160
- 95% CI: [0.0069, 0.0313]
- Row concentration: E≈1.0, Std=0 (T6 row consistency에서 본 CW 지문과 동일 계열)

### MC_S9_Combined

- Pass rate: 0 / 500
- 95% CI: [0.0000, 0.0074]

## 해석

### A. 기존 T6 고정 vs Monte Carlo

- MC_S1·MC_S7 등에서 pass rate가 **trial마다 샘플링된 채널**에 대해 0과 1 사이로 분산되며, Clopper–Pearson 구간이 **0 또는 1에 붙지 않는** 케이스가 나온다 → “단일 realization만 반복할 때의 분산 0” 문제를 완화하는 방향과 일치한다.
- 동일 엔진이라도 **스펙이 다르면** 절대 pass rate는 T6 표의 숫자와 직접 비교되지 않는다(의도된 차이).

### B. Realization variance

- MC_S1 (SNR·CFO 랜덤): pass rate 약 0.62, CI 폭이 중간 — 저·중 SNR 혼합에서 **조건 민감도**가 반영된다.
- MC_S7: pass rate 약 0.28, CI 폭 유사 — **JSR·잡음 RNG·CFO** 변동에 따른 분산이 관측된다.
- MC_S8_CW_Multi: pass는 매우 낮지만 row concentration≈1로 **특정 간섭 지문**이 안정적으로 드러난다.

### C. 3축 지표 (pass + LLR + entropy)

- pass rate는 운용 기준, LLR reliability·row entropy는 **물리적/알고리즘적 스트레스 지문** 보조 지표로 함께 보면, “망가졌지만 우연히 CRC만 통과”류와 구분하는 데 도움이 된다 (이미 T6 보조 진단과 동일 철학).

## 자진 고지

1. **Realization 샘플링**: `McScenarioKind`별로 `MC_Scenario_Specs.hpp`에 명시된 범위에서 균등·정규 분포를 사용한다. Combined는 T9에 가깝게 회전·ISI·타이밍 지터·AWGN·Barrage·CW를 겹친다.
2. **500 trials**: pass rate CI의 반폭은 대략 `O(1/√N)` 스케일. 1000으로 늘리면 CI는 좁아지나, pass rate가 매우 낮은 구간에서는 여전히 상대 오차가 크다.
3. **구현 범위**: T6 `HTS_T6_SIM_Test.cpp`의 `feed_raw_ext`·채널 블록을 하네스 내에 **복제**하여 단일 TU로 링크한다. 엔진 동작 변경은 없다.
4. **Clopper–Pearson**: `HTS_Phase2::clopper_pearson_ci(n, k, 0.05)` — `n` 시행 중 `k`회 pass (Beta exact, Phase 2 구현 재사용).
5. **MC_S8_CW_Freq** 전량 실패는 스펙(고 JSR + 넓은 주파수 스윕)이 매우 공격적이라는 해석이 가능하며, 완화 스펙은 `kMcSpecs`에서 범위를 줄여 재측정하면 된다.
