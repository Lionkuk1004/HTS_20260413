# S5 / CFO “초정밀 현미경” 점검 보고 (코드만 근거, **수정 없음**)

**범위:** T6 `test_S5` → `ch_cfo` → `feed_raw_ext` → `Feed_Chip` → `phase0_scan_` → (통과 시) `Estimate_From_Preamble` / `Advance_Phase_Only` / 이후 `Apply`.
**금지 준수:** 본 작업에서 소스·빌드 스크립트 **일절 변경하지 않음**.

---

## 1. 이전 원인 설명에 대한 정정 (왜 그 분석이 틀렸는가)

### 1.1 잘못 세운 가설

이전에 “`sin_block >> 6` 은 큰 \(|\Delta\phi|\) 에서 \(\sin(\Delta\phi)/N \not\approx \sin(\Delta\phi/N)\) 이므로, `atan2` 후 \(\Delta\phi/N\) 으로 바꾸면 S5 구멍이 메워질 것”이라고 말한 것은 **물리 모델을 잘못 식별한 것**에 해당합니다.

### 1.2 코드가 말해 주는 사실

`Estimate_From_Preamble` 의 입력 `dI0,dQ0` / `dI1,dQ1` 은 **시간도메인 칩 스트림**이 아니라, **각 64칩 블록에 대해 `walsh63_dot_` 로 압축된 복소 스칼라** 두 개입니다.

```155:171:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher_Internal.hpp
inline void walsh63_dot_(const int16_t *chip_I, const int16_t *chip_Q,
                         int32_t &dot_I, int32_t &dot_Q) noexcept {
    int32_t dI = 0;
    int32_t dQ = 0;
    for (int j = 0; j < 64; ++j) {
        const int32_t sI = static_cast<int32_t>(chip_I[j]);
        const int32_t sQ = static_cast<int32_t>(chip_Q[j]);
        if (k_w63[static_cast<std::size_t>(j)] > 0) {
            dI += sI;
            dQ += sQ;
        } else {
            dI -= sI;
            dQ -= sQ;
        }
    }
    dot_I = dI;
    dot_Q = dQ;
}
```

`cos_delta` / `sin_delta` 는 그 두 스칼라의 **쌍곡 내적**으로, 기하적으로는 \(\arg(\overline{z_0}\,z_1)\) 에 해당합니다.

```81:85:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_CFO_Compensator.h
    // conj(z0)·z1 의 실수/허수: cos = dI0*dI1+dQ0*dQ1, sin = dI0*dQ1−dQ0*dI1
    const int64_t cos_delta = static_cast<int64_t>(dI0) * dI1 +
                              static_cast<int64_t>(dQ0) * dQ1;
    const int64_t sin_delta = static_cast<int64_t>(dI0) * dQ1 -
                              static_cast<int64_t>(dQ0) * dI1;
```

**정정:** 이 각은 “칩당 위상 \(\omega\) 의 64배”와 **동치가 아닙니다**. Walsh 가중 합은 **선형 위상 누적**과 교환하지 않으므로, \(\mathrm{atan2}(\sin\Delta,\cos\Delta)/64\) 를 **시간도메인 `Apply` 의 단위칩 회전각**으로 바꿔 끼우는 것은 **검증되지 않은 식별(동치 가정)** 이었고, 실제로 그 패치는 S5 전역을 붕괴시켜 가정이 틀렸음을 **실험적으로 반증**했습니다.

### 1.3 레거시 `sin_block>>6` + `cos = sqrt(1−sin²)` 의 의미

이 경로는 “물리 CFO 한 칩”에 대한 엄밀한 MLE가 아니라, **Walsh-도메인 통계량 → Q14 단위원에 가까운 per-chip 스텝**으로 사상하는 **경험적·정합적** 연결입니다. `Apply` 는 **칩마다** 그 스텝을 곱합니다. 따라서 “근사 오류”라기보다 **다른 정의의 \(\omega\)** 를 쓰는 것에 가깝고, `atan2` 경로는 그 정의를 **바꿔버린** 것입니다.

---

## 2. 채널 CFO가 테스트에 주입되는 방식 (단일 진실)

```390:398:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_TEST\t6_sim\HTS_T6_SIM_Test.cpp
static void ch_cfo(int16_t* I, int16_t* Q, int n, double cfo_hz) noexcept {
    for (int i = 0; i < n; ++i) {
        const double ph = 2.0 * kPi * cfo_hz * i / kChipRate;
        const double c = std::cos(ph), s = std::sin(ph);
        const double di = I[i], dq = Q[i];
        I[i] = sat16(di * c - dq * s);
        Q[i] = sat16(di * s + dq * c);
    }
}
```

- 위상은 **버퍼 인덱스 `i=0..n-1`** 기준으로 선형 증가합니다.
- AMI 빌드에서 `kChipRate = 200000.0` Hz 입니다 (`HTS_T6_SIM_Test.cpp` 상단 상수).

**사실:** 여기서의 CFO는 **연속 회전**이며, 수신기 내부의 “dot 기반 CFO 추정”과 **1:1 선형 대응**이라는 가정은 코드만으로는 성립하지 않습니다.

---

## 3. S5 실패가 코드 상으로 “어디서” 가능한가 (부검 순서)

### 3.1 `feed_raw_ext` — 성공 정의

```203:241:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_TEST\t6_sim\HTS_T6_SIM_Test.cpp
    for (int i = 0; i < pre_guard; ++i) rx.Feed_Chip(0, 0);
    for (int i = 0; i < n; ++i)         rx.Feed_Chip(rxI[i], rxQ[i]);
    ...
    if (m.crc_passed && m.length_correct) {
        ...
        m.pass = (bit_err == 0);
    }
```

S5 표의 `0/100` 은 **`pass` 거짓**이며, 그 원인은 (1) CRC/길이 실패, (2) 비트 오류, (3) **애초 동기·헤더 단계에서 복구 불가** 등 **모두 가능**합니다. “CFO `Apply` 한 줄”만의 문제로 단정할 수 없습니다.

### 3.2 AMI `phase0_scan_` — **CFO 추정 이전**에 걸리는 하드 게이트

Phase0 최종 판정:

```807:808:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher_Sync_AMI.cpp
    const bool pass = (best_off >= 0 && r_avg_ok &&
                       best_e63 >= e63_min && sep_ok);
```

- `best_e63` / `second_e63` / `r_avg_ok` / `sep_ok` / `e63_min` 은 **버퍼 내 64오프셋 스캔**과 **에너지·분리도**에 의존합니다.
- AMI 분기에서는 **32칩×2 non-coherent** 로 `accum` 을 만들고 `best_e63` 을 갱신합니다 (`Sync_AMI.cpp` 293–319 인근).

**사실:** 특정 CFO에서 위상이 빠르게 돌면, 이 **에너지형 지표**가 약해지거나 `r_sep` 이 나빠져 **`pass==0`** 이 될 수 있습니다. 이 경우 **`Estimate_From_Preamble` 은 호출되지 않습니다**(`cfo_.active_` 도 false 유지).

### 3.3 `pass==true` 일 때만 이어지는 CFO 경로

락 성공 후에야 `walsh63_dot_` 두 번으로 `d0`, `d1` 을 다시 구하고 `Estimate_From_Preamble` + `Advance_Phase_Only(192)` 가 실행됩니다 (`Sync_AMI.cpp` 1087–1100 인근, 이전 세션에서 확인한 구조).

**사실:** S5 구멍이 “추정식 오차”인지 “애초 락 실패”인지는 **로그로 단계를 찍기 전에는 분리되지 않습니다.** (본 보고서는 코드 수정 없이 분리 증거를 추가로 생성하지 않음.)

### 3.4 `Advance_Phase_Only(192)` — 가설 수준

주석 의도: “P0 스캔 구간 192 chip 위상 누적 전진 → payload 첫 `Apply` 시 위상이 스캔 구간 끝과 정합”.

**가설(미증명):** `best_off` 가 0이 아닐 때, **물리 시간축**과 “버퍼 인덱스 0부터 192칩 전진”이 항상 일치하는지는 **주석만으로는 증명되지 않습니다.** 다만 이는 “`sin` 근사 오류”와는 **별도 축**의 정합성 문제입니다.

---

## 4. “이전 분석도 오류냐?”에 대한 답

| 주장 | 판정 |
|------|------|
| “S5 AMI 의 특정 CFO 구간 실패는 **반드시** `sin_block>>6` 의 Taylor 오류 때문이다” | **근거 불충분·실제로는 반증됨** (`atan2` 치환이 전역 붕괴). |
| “Walsh-dot 위상과 시간도메인 `Apply` 의 \(\omega\) 는 **동일 식별**이 아니다” | **코드상 사실** (§1.2–1.3). |
| “실패는 phase0 게이트·헤더·FEC·비트검증 등 **복수 단계** 중 어디에서도 발생 가능하다” | **코드상 사실** (§3). |

즉, **“근본 물리가 sin(Δφ)/N 이라서”** 를 전제로 한 설명은 틀렸고, **“Walsh-dot 통계와 시간도메인 CFO 보정은 동치가 아니다”** 는 여전히 맞습니다.

---

## 5. 수정 없이 할 수 있는 다음 증거 (권장)

1. **단일 CFO·단일 시드**로 `HTS_DIAG_PRINTF` 빌드를 유지한 채, 한 패킷에 대해
   - `[P0-SCAN] ...` 의 `pass` / `e63` / `r_sep`
   - 이후 `[P1-GATE]` / 헤더 관련 로그
   를 **같은 타임라인**으로 묶어 저장합니다.
2. **성공 CFO(예: 4000 Hz) vs 실패 CFO(예: 3000 Hz)** 를 동일 `t` 시드로 비교해, 실패가 **phase0** 에서 끊기는지 **락 이후** 에서 끊기는지 먼저 분리합니다.
3. 기존 대형 로그(`ami_diag_full.log` 등)는 **CFO 라벨이 라인에 없어** 인과 추적에 한계가 있으므로, 위 1–2처럼 **좁힌 재현 로그**가 필요합니다.

---

## 6. 결론 (한 문장)

S5 AMI 의 “비단조” CFO 구간은 **`ch_cfo` 가 만든 연속 회전**과 **`phase0_scan_` 의 AMI 전용 에너지·분리 게이트**, 그리고 **(락 후) Walsh-dot 기반 CFO 추정 + 시간도메인 `Apply` 의 비동치 연결**이 겹쳐 생길 수 있는 현상이며, **“sin(Δφ)/N 근사 버그” 단정은 코드·실험 모두에 의해 지지되지 않습니다.**

---

## 7. 제2차 초정밀 조사 (추가 사실·호출 그래프·실패 분기)

**전제:** 소스·빌드 **미변경**. 아래는 `HTS_V400_Dispatcher_Sync_AMI.cpp` / `HTS_T6_SIM_Test.cpp` / `Internal.hpp` 만 재독한 결과입니다.

### 7.1 T6 S5 기본 경로에서 `phase0_scan_holographic_` 는 **없다**

- `cursor_t6_build_ami.cmd` 기본은 `HTS_USE_HOLOGRAPHIC_SYNC` **미정의**이므로, `phase0_scan_()` 내부의 홀로 분기는 **전처리 단계에서 제거**된다.

```65:71:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher_Sync_AMI.cpp
void HTS_V400_Dispatcher::phase0_scan_() noexcept {
#ifdef HTS_USE_HOLOGRAPHIC_SYNC
    if (use_holographic_sync_) {
        phase0_scan_holographic_();
        return;
    }
#endif
    int32_t best_e63 = 0, second_e63 = 0;
```

- `feed_raw_ext` → `setup(rx, ds)` 는 **홀로 플래그를 켜지 않는다** (`Set_Holographic_Sync` 호출 없음).

```169:177:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_TEST\t6_sim\HTS_T6_SIM_Test.cpp
static void setup(HTS_V400_Dispatcher& d, uint32_t seed) noexcept {
    d.Set_Seed(seed);
    d.Set_IR_Mode(true);
    d.Set_Preamble_Boost(kPreBoost);
    d.Set_Preamble_Reps(kPreReps);
    d.Set_Packet_Callback(on_pkt);
    d.Update_Adaptive_BPS(1000);
    d.Set_Lab_IQ_Mode_Jam_Harness();
}
```

**사실:** 기본 AMI T6 S5 에서 동기 문제는 **`phase0_scan_holographic_` 가 아니라** 전통 `phase0_scan_` 본문(72행 이후) + 이후 Phase1 로직으로만 재현된다.

### 7.2 `phase0_scan_()` 호출 위치는 **단 한 곳** (192칩 버퍼 채운 직후)

`WAIT_SYNC` 이고 `pre_phase_==0` 일 때 칩을 `p0_buf128_*` 에 쌓고, `p0_chip_count_` 가 192가 되는 순간 **곧바로** `phase0_scan_()` 호출 후 `return` 한다.

```1489:1534:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher_Sync_AMI.cpp
    if (phase_ == RxPhase::WAIT_SYNC) {
        if (pre_phase_ == 0) {
            ...
            p0_buf128_I_[p0_chip_count_] = chip_I;
            p0_buf128_Q_[p0_chip_count_] = chip_Q;
            ++p0_chip_count_;
            if (p0_chip_count_ < 192)
                return;
            ...
            phase0_scan_();
            return;
        }
```

**사실:** “192칩 찼는데 스캔이 안 돌았다”류는 이 조건·상태기계와 **모순**된다.

### 7.3 Phase0 실패 시 **정확히** 하는 일 (버퍼 슬라이드)

`pass==false` 이면 `psal_commit_align_` 없이, **128칩 오프셋 복사로 윈도우를 밀고** `p0_chip_count_=64` 로 둔다.

```1134:1156:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher_Sync_AMI.cpp
    } else {
        ...
        std::memcpy(p0_buf128_I_, p0_buf128_I_ + 128,
                    static_cast<size_t>(64) * sizeof(int16_t));
        std::memcpy(p0_buf128_Q_, p0_buf128_Q_ + 128,
                    static_cast<size_t>(64) * sizeof(int16_t));
        p0_chip_count_ = 64;
    }
}
```

**사실:** 이 경로에서는 §3.3의 **CFO 추정 블록에 진입하지 않는다**(`if (pass) { ... }` 가 상위 구조이므로). 즉 S5 실패 원인 후보에 **“CFO 추정 자체가 안 돌았다”** 를 코드 차원에서 넣을 수 있다.

### 7.4 Phase1(프리앰블 2번째 심볼) — **별도 에너지 하한**과 “침묵 리셋”

Phase1에서 AMI 는 다시 **32칩×2 non-coherent** 로 `e63_64`/`e0_64` 를 만들고, `>>16` 한 뒤 `max_e_sh` 와 `k_P1_MIN_E` 를 비교한다.

```1683:1691:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher_Sync_AMI.cpp
        const int32_t e63_sh = static_cast<int32_t>(e63_64 >> 16);
        const int32_t e0_sh  = static_cast<int32_t>(e0_64 >> 16);
#if defined(HTS_PHASE0_WALSH_BANK)
        static constexpr int32_t k_P1_MIN_E = 1000;
#elif defined(HTS_TARGET_AMI)
        static constexpr int32_t k_P1_MIN_E = 500;
#else
        static constexpr int32_t k_P1_MIN_E = 1000;
#endif
```

`max_e_sh < k_P1_MIN_E` 이면 **대량 상태 클리어** 후 `pre_phase_=0` 으로 돌아가며, 조건에 따라 **현재 칩 한 개만** P0 버퍼에 다시 심는 분기까지 있다.

```1756:1794:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher_Sync_AMI.cpp
        if (max_e_sh < k_P1_MIN_E) {
            ...
            pre_phase_ = 0;
            ...
            p0_chip_count_ = 0; p0_carry_count_ = 0;
            ...
            if (carry_only_full && !p1_rx_in_buf) {
                p0_buf128_I_[0] = chip_I; p0_buf128_Q_[0] = chip_Q;
                p0_chip_count_ = 1;
            }
            return;
        }
```

**사실:** Phase0 를 통과해도 **Phase1 에너지가 CFO·위상에 의해 깎이면** 여기서 루프가 풀리지 않을 수 있다. S5 의 `0/100` 은 이 경로와도 **양립**한다.

### 7.5 `READ_HEADER` 진입 조건 (프리앰블 심볼 판별)

`PRE_SYM1` 로 판정될 때만 `set_phase_(READ_HEADER)` 가 호출된다.

```1840:1857:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher_Sync_AMI.cpp
        if (sym == static_cast<int8_t>(PRE_SYM1)) {
            ...
            set_phase_(RxPhase::READ_HEADER);
            ...
            return;
        }
```

`[P1-GATE]` 로그의 `enter_hdr` 필드는 **이름과 달리** `gate_sym_pre1` 을 그대로 넣은 것으로, **「이번 프레임에서 `sym` 이 `PRE_SYM1` 로 결정되었는가」** 이다 (`sym==PRE_SYM1` 이면 곧바로 아래 `if (sym==PRE_SYM1)` 블록에서 `READ_HEADER` 로 전이).

```1829:1836:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher_Sync_AMI.cpp
                const int enter_hdr = gate_sym_pre1;
                std::printf(
                    "[P1-GATE] #%d n=%d e63_sh=%d e0_sh=%d max_e=%d "
                    "k_P1_MIN_E=%d gate_e_ok=%d ge_dom=%d "
                    "sym_pre1=%d sym_pre0=%d carry_pend=%d enter_hdr=%d\n",
                    ...
                    gate_sym_pre0, p1_carry_pending_, enter_hdr);
```

**로그 해석 주의:** 필드명 `enter_hdr` 가 “헤더 단계 진입 완료”를 뜻하는 것처럼 읽히나, 코드상으로는 **`sym==PRE_SYM1` 여부**다. (에너지 게이트를 통과한 뒤 같은 블록에서 출력됨.)

### 7.6 실패 원인을 코드만으로 쪼갠 **판정 표** (증거 수집 순서)

| 단계 | 코드 근거 | 실패 시 의미 |
|------|------------|----------------|
| P0 | `pass` (807–808행) | 동기 스캔 자체 실패 → CFO 추정 **미실행** (§7.3) |
| P1 에너지 | `max_e_sh < k_P1_MIN_E` (1756행) | 프리앰블 2nd 심볼 에너지 부족 → **침묵 리셋** |
| P1 심볼 | `sym` vs `PRE_SYM1` (1840행) | 헤더 진입 **불가** |
| 상위 | `feed_raw_ext` 의 CRC/len/bit (203–241행) | 동기 이후 디코드/비트 불일치 |

**미확정(로그 필요):** 특정 CFO(예 2500 Hz)에서 위 표의 **어느 행이 지배적인지**는, 본 조사처럼 **실행 로그 없이는 단정 불가**이다.

### 7.7 제1차 보고서 대비 보강 요약

- **추가로 확정한 사실:** 기본 T6 AMI 에서는 홀로 스캔이 **링크되지 않는다**(§7.1). `phase0_scan_` 호출 지점이 **192칩 직후 한 곳**임이 코드로 확정(§7.2). P0 실패 시 CFO 추정 경로가 **구조적으로 스킵**됨(§7.3).
- **추가로 드러난 실패 축:** Phase1 **500 단위** 에너지 게이트 및 침묵 리셋(§7.4).
- **로그 해석 주의:** `[P1-GATE] enter_hdr` 의미(§7.5).

---

*보고 일자: 2026-04-22 (제1차) + 제2차 갱신 동일일 · 분석 대상: `HTS_LIM` · **코드/빌드 변경 없음**.*
