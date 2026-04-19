# AGC gain sweep 실험 결과

## AGC Discovery (지시 0)

### === AGC Discovery ===

- **File:** `D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_Preamble_AGC.h`
- **`pre_agc_` 클래스명:** `ProtectedEngine::HTS_Preamble_AGC` (`HTS_V400_Dispatcher.hpp` 멤버, 주석: P0 피크 기반 디지털 AGC, shift만)
- **`Apply` 시그니처 (원문):**

```30:33:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_Preamble_AGC.h
    /// @brief 칩에 gain 적용 (Feed_Chip 내, DC/CFO 후)
    /// @param[in,out] chipI, chipQ
    /// @note shift만 사용, 곱셈 0
    void Apply(int16_t& chipI, int16_t& chipQ) const noexcept;
```

- **Gain 변수:** `gain_shift_` (`int32_t`, private), 의미: **0=×1, 1=×2, 2=×4, 3=×8, 4=×16** (비트 시프트).
- **설정 경로:** `Set_From_Peak(int32_t peak_mag)` — P0에서 측정한 `|I|+|Q|` 평균 `peak_mag`와 `kTargetAmp`(1000) 구간 비교로 `gain_shift_`만 결정 (나눗셈 없음).

**Gain 적용 방식 (실측용 스케일 추가 후 원문 요지):**

- DC 제거 후 `pre_agc_.Apply(chip_I, chip_Q)` (`HTS_V400_Dispatcher.cpp` `Feed_Chip` 내).
- `gain_shift_ > 0`이면 `vi = chip << gain_shift_` 후 **`HTS_AGC_GAIN_SCALE_NUM/DEN` 정수 스케일**(기본 256/256)을 `int64` 곱으로 적용, `int16` 클램프.
- `gain_shift_ == 0`이면 시프트 없이 **스케일만** 적용 (NUM/DEN=256/256이면 입력과 동일).

**주의:** 기존 설계는 “shift만, 곱셈 0”. 이번 실험에서만 **NUM/DEN 스케일 곱셈**을 `Apply`에 추가함 (baseline 256/256은 기존 시프트 경로와 동치).

---

## 빌드 (5 구성)

| Config | NUM/DEN | Scale | 빌드 |
|--------|---------|-------|------|
| baseline | 256/256 | 1.0 | PASS |
| half | 128/256 | 0.5 | PASS |
| quarter | 64/256 | 0.25 | PASS |
| double | 512/256 | 2.0 | PASS |
| small | 32/256 | 0.125 | PASS |

(C4996 `strncpy` 경고만 기존과 동일.)

---

## 비교표 (T6 종합)

| Config | Scale | grand_pass | Δ vs baseline | Saturation % (진단) | RMS \|I\| (진단) |
|--------|-------|-------------|-----------------|---------------------|------------------|
| baseline | 1.0 | **10927 / 11200** | 0 | 0.2188% | 2128.64 |
| half | 0.5 | **153 / 11200** | −10774 | 0.2188% | 2128.64 |
| quarter | 0.25 | **41 / 11200** | −10886 | 0.2188% | 2128.64 |
| double | 2.0 | **10907 / 11200** | −20 | 0.2188% | 2128.64 |
| small | 0.125 | **0 / 11200** | −10927 | 0.2188% | 2128.64 |

**포화·RMS가 모든 구성에서 동일한 이유:** `HTS_AMP_DIAG`는 **`Feed_Chip` 진입 직후**(DC·`pre_agc_.Apply` **이전**) `rx_I`/`rx_Q`를 기록함. AGC 스케일은 **그 이후**에만 적용되므로, 본 스윕으로는 **“AGC 후 포화 감소”는 관측되지 않음**. AGC 효과는 **pass**에만 나타남.

---

## 시나리오별 (요지)

| 시나리오 | baseline | half | quarter | double | small |
|----------|----------|------|---------|--------|-------|
| S1 (표 1행) | 20 / OK20 | **0 / FAIL** (조건 문자열·BER 붕괴) | **0** | 20 | **0** |
| S10a Endurance | 10000/10000 | **0** | **0** | 10000/10000 | **0** |
| S7 10dB (샘플) | 6 / PART | (전면 FAIL 계열) | 동左 | **6 / PART** | 동左 |

- **half / quarter / small:** 종합 표 **전 행**이 사실상 baseline과 달리 붕괴(대표: S1 `PASS`→`FAIL`, S10a `10000`→`0`).
- **double:** baseline과 **대부분 동일**, `grand`만 **−20** (일부 조건에서만 pass 차이 — 예: S7 10dB 행은 baseline·double **동일** 확인).

전 행 53개 전부 표로 붙이지 않음(로그: `agc_*_stdout.log`).

---

## 판정

### C) **gain을 낮추면 pass 급락** (가설 “낮추면 유리” **반증**)

- **0.5 / 0.25 / 0.125** 스케일은 **양자화·SNR 붕괴**가 지배적이어서 **실용 불가** 수준.
- **2.0**은 소폭 악화(−20) — **이미 포화 꼬리가 있는 상태에서 추가 증폭**이 일부 조건에 해로울 수 있음.

### A) “포화만 줄이면 pass↑” — **본 측정만으로는 미검증**

- 포화 지표는 **AGC 전 raw**라 스케일과 무관; **AGC 후** 재계측이 필요.

### B) “변화 없음” — 해당 없음 (double은 소변화).

---

## Phase 3 cliff

- 본 작업에서는 **Phase 3 cliff 배치 미실행**. 동일 `HTS_AGC_GAIN_SCALE_*`를 Phase3 `cl`에 넣으면 연동 가능.

---

## 자진 고지

- **실제 gain:** 연속 값이 아니라 **`gain_shift_`(2의 거듭제곱)** + 실험용 **NUM/DEN 스케일**.
- **진폭 진단 위치:** AGC **전**이라 **포화%·RMS는 스윕과 무관** — 다음 단계는 **`Apply` 직후** 또는 `orig_I_` 저장 직전 등 **AGC 출력** 계측 권장.
- **의심:** `half`에서 S1 조건 문자열이 `B0/OK0/...`로 바뀐 것은 **상위 trial 실패로 인한 파생 표시**일 수 있음(코어 미분석).

---

## 변경 파일

- `HTS_LIM/HTS_Preamble_AGC.h` — `HTS_AGC_GAIN_SCALE_NUM/DEN`, `Apply` 스케일·`gain_shift_==0` 경로 정리
- `HTS_TEST/t6_sim/build_and_test_agc_sweep.bat` — 5구성 빌드·실행·`findstr` 요약

`NUM/DEN` 미지정 시 기본 **256/256**으로 **이전 `Apply` 동작과 동치**(시프트만 있던 경우).
