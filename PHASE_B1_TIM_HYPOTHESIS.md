# S5 시나리오 Timing 조사 (Step 4-A)

**범위**: `HTS_TEST/t6_sim/HTS_T6_SIM_Test.cpp` 정적 읽기만. 빌드·실행 없음.  
**기준 태그**: `T_CFO_SWEEP_EXT` (현재 `test_S5`에 확장 CFO 배열 포함)

---

## Q1. `test_S5` 구조

- **라인 범위**: 약 **699~772** (`static void test_S5()` 선언 ~ 함수 끝 `}`).
- **CFO sweep 방식**: `const double cfos[] = { ... }` (20점)에 대해 **이중 루프** — 외부 `for (double cfo : cfos)`, 내부 `for (int t = 0; t < kTrials; ++t)` (`kTrials == 100`).
- **trial당 채널 처리**:
  1. `ds = mk_seed(0x500000u, t)`
  2. `tx = build_tx(ds, t)` — 디스패처 시드·페이로드 생성
  3. `rI/rQ`에 TX 칩 복사 후 **`ch_cfo(rI, rQ, tx.n, cfo)` 만 적용**
  4. **`feed_raw_ext(ds, rI, rQ, tx.n, tx.info)`** — `pre_guard` 인자 **미지정** → 기본값 사용 (Q4)
- **trial당 timing 변동 (S4/S9 의미의 pre_guard 가산)**: **없음**. `timing_off`·`off` 변수 없음.

---

## Q2. 채널 생성 방식

- **호출 순서 (S5 전용)**: `build_tx` → `memcpy` → **`ch_cfo`** → **`feed_raw_ext`**
- **별도 `apply_channel` 류 함수**: 없음. 채널은 **`ch_cfo`** 가 전부.
- **CFO 주입**: **Y** — `ch_cfo` (아래 Q5).
- **timing offset 주입 (명시적 chip offset / `kGuard + off`)**: **N**
- **관련 파라미터**: CFO 값 `cfo` (Hz), `tx.n`, `kChipRate` (`ch_cfo` 내부).

---

## Q3. `timing_offset` / 타이밍 가산 사용처

파일 내 **`timing_off`** 명칭은 **S9** 에만 등장 (`rng() % 31` → `feed_raw_ext(..., kGuard + timing_off)`).

| 위치 | 요약 |
|------|------|
| **S4** (`test_S4`, 약 652~678) | `offsets[]` 루프에서 `feed_raw_ext(ds, tx.I, tx.Q, tx.n, tx.info, **kGuard + off**)` — **의도적 타이밍 오프셋** (양의 chip 만큼 pre_guard 확장). |
| **S9** (`test_S9`, 약 1049~1051) | 복합 스트레스: `timing_off = rng() % 31`, `kGuard + timing_off`. |
| **S5** | `feed_raw_ext(...)` **기본 인자**만 사용 → **S4/S9 와 달리 `kGuard + off` 없음**. |
| **그 외** | `feed_raw_ext` / `feed_raw` 대부분 기본 `pre_guard`; S4·S9 외 명시 가산 없음. |

**`timing_offset` / `chip_offset` / `tim_off` 문자열**: 본 파일에서 **검색되지 않음** (대신 `timing_off`, S4의 `off` 사용).

**S5 vs S4 (채널 생성 차이)**:

- **S4**: IQ는 **원본** (`ch_cfo` 등 없음). 수신 측 정렬만 **`pre_guard = kGuard + off`** 로 변화.
- **S5**: IQ에 **`ch_cfo`만** 적용. **`pre_guard`는 항상 기본 `kGuard`**.

---

## Q4. chip alignment / guard

- **`kGuard`**: **256** chip (`static constexpr int kGuard = 256;`, 약 72행).
- **`feed_raw_ext` 동작** (약 203~220행):
  - `pre_guard` 만큼 `Feed_Chip(0, 0)` 선행
  - `n` 칩 데이터 `Feed_Chip(rxI[i], rxQ[i])`
  - 이후 **`kGuard` 고정** 만큼 후행 `Feed_Chip(0, 0)` (인자로 받지 않음 — 항상 `kGuard`)
- **S5에서 guard 변동**: **N** — CFO 루프·trial 루프 모두에서 `pre_guard`·후행 guard **변경 없음**.

---

## Q5. CFO 수식 (정확한 형태)

- **함수**: `ch_cfo` — 약 **338~347**행.
- **수식** (칩 인덱스 `i = 0 .. n-1`):

  \[
  \phi_i = 2\pi \cdot f_{\mathrm{CFO}} \cdot i / R_{\mathrm{chip}},\quad
  (I,Q)_i \leftarrow \mathrm{rotate}_{\phi_i}(I_i, Q_i)
  \]

  코드: `ph = 2.0 * kPi * cfo_hz * i / kChipRate`; `cos/sin`으로 복소 회전 후 `sat16`.

- **chip index 기준**: **수신 버퍼의 첫 칩 `i=0`** 부터 **페이로드 직전이 아니라 `ch_cfo`에 넘긴 전체 `n` 칩**에 동일 공식 적용 (guard 칩은 `ch_cfo` 입력에 포함되지 않음 — `rI/rQ` 길이는 `tx.n`).

---

## 통신공학·코드 기반 판정 (시나리오 A / B / C)

### 시나리오 A — **S5는 “명시적 TIM(칩 타이밍 오프셋)” 없이 CFO 위주**

- S5 본문에는 **S4/S9 형태의 `kGuard + off` / `timing_off` 가 없음** → **측정 목적상 “순수 CFO + 고정 256칩 선행 제로 가드”**에 가깝다.
- CFO sweep 비단조 패턴은 **이 코드만으로는 “TIM 가설(칩 오프셋 변조)”으로 설명할 근거가 없음** — 다른 요인(코드·Walsh·동기 추정 비선형, trial별 페이로드/시드 변화와 CFO의 결합 등)을 의심하는 편이 타당.

### 시나리오 B — **S5에 timing 변동 포함**

- **해당 없음 (본 파일 기준)**. 타이밍 가산은 **S4·S9** 에만 명시적으로 존재.

### 시나리오 C — **S5는 CFO만 고정 guard이나 trial마다 seed 변동**

- **해당함**: `mk_seed(0x500000u, t)` 및 `build_tx(ds, t)` → **trial마다 디스패처 시드·8바이트 `info` 변화** → IQ 시퀀스·동기 경계 조건가 CFO와 **비단조적으로 결합**할 수 있음.
- 이는 **LPI 실험의 “TIM”(±15 chip 등)** 과는 별개이나, “trial 간 임의성”이 **간접적으로 동기/경계 민감도**를 바꿀 여지는 있다 (추가 실험으로 분리 가능).

### 종합 (TIM 가설 vs 현재 S5)

| 질문 | 결론 |
|------|------|
| S5 내부에 **S4형 timing offset** 있는가? | **아니오** |
| 비단조 CFO 패턴이 **현재 S5 코드만으로 TIM 때문**이라고 말할 수 있는가? | **아니오** (명시적 TIM 미주입) |
| 후속으로 **CFO + TIM 복합**을 보고 싶다면? | **새 시나리오** (예: S5 고정 CFO + `kGuard + off` sweep, 또는 S9 확장) 권장 — Step 4-B 지시서 **시나리오 A 후속** 과 일치 |

---

## Step 4-B 후속 지시 (조사 결과 반영)

1. **(권장) TIM 실증**: 지시서의 **“S5_TIM” 류** — CFO 몇 점 고정 + **`feed_raw_ext(..., kGuard + off)`** 로 `off` 그리드 스윕 (S4와 동일 메커니즘 재사용).
2. **CFO-only 정밀화**: **단일 seed 고정** (`t=0` 고정 또는 `mk_seed` 상수) 후 CFO만 스윕 → **시나리오 C** 분리 (비단조가 seed 민감인지 확인).
3. **TIM 가설 “부정”이 아니라 “범위 외”**: 실제 현장/다른 계층에서 TIM이 크다는 **과거 LPI 결론**과 모순되지 않음 — **본 T6_SIM의 S5 정의만** TIM-free.

---

## 코드 인용 (핵심만)

`test_S5` 핵심 루프:

```729:743:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_TEST\t6_sim\HTS_T6_SIM_Test.cpp
    for (double cfo : cfos) {
        int ok = 0, crc_only = 0;
        long long total_bits = 0;
        for (int t = 0; t < kTrials; ++t) {
            const uint32_t ds = mk_seed(0x500000u, t);
            auto tx = build_tx(ds, t);
            if (tx.n <= 0) {
                total_bits += 64;
                continue;
            }
            int16_t rI[kMaxC], rQ[kMaxC];
            std::memcpy(rI, tx.I, sizeof(int16_t) * static_cast<size_t>(tx.n));
            std::memcpy(rQ, tx.Q, sizeof(int16_t) * static_cast<size_t>(tx.n));
            ch_cfo(rI, rQ, tx.n, cfo);
            const TrialMetrics m = feed_raw_ext(ds, rI, rQ, tx.n, tx.info);
```

`ch_cfo`:

```338:347:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_TEST\t6_sim\HTS_T6_SIM_Test.cpp
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

S4 vs S5 `feed_raw_ext` 인자 차이:

```664:665:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_TEST\t6_sim\HTS_T6_SIM_Test.cpp
            const TrialMetrics m =
                feed_raw_ext(ds, tx.I, tx.Q, tx.n, tx.info, kGuard + off);
```

```742:743:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_TEST\t6_sim\HTS_T6_SIM_Test.cpp
            ch_cfo(rI, rQ, tx.n, cfo);
            const TrialMetrics m = feed_raw_ext(ds, rI, rQ, tx.n, tx.info);
```
