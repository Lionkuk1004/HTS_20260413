# Step 3 보충 조사

근거: `HTS_TEST/t6_sim/HTS_T6_SIM_Test.cpp` 읽기만 (수정·빌드 없음).

---

## Q1. CFO → `param` 문자열

`test_S5` 내부 (CFO 루프 끝부분):

```803:807:HTS_TEST/t6_sim/HTS_T6_SIM_Test.cpp
        char label[32], param[16];
        std::snprintf(label, sizeof(label), "CFO %+6.0f Hz", cfo);
        std::snprintf(param, sizeof(param), "%.0fHz", cfo);
        row(label, ok, kTrials);
        record_ext("S5", param, ok, crc_only, kTrials, total_bits);
```

| 버퍼 | 포맷 | 예시 출력 |
|------|------|-----------|
| `label` (진행 바용) | `"CFO %+6.0f Hz"` | `CFO     0 Hz`, `CFO  2500 Hz` 등 |
| `param` (`record_ext` 두 번째 인자) | `"%.0fHz"` | `0Hz`, `50Hz`, `2500Hz` — **숫자와 `Hz` 사이 공백 없음** |

`record_ext` 호출 (실제 코드):

```cpp
record_ext("S5", param, ok, crc_only, kTrials, total_bits);
```

---

## Q2. `TrialMetrics` + `pass` 판정

### 구조체 (전체 필드)

```88:94:HTS_TEST/t6_sim/HTS_T6_SIM_Test.cpp
struct TrialMetrics {
    bool pass;              // memcmp 완전 일치 (기존 feed_raw 동등)
    bool crc_passed;        // success_mask == DECODE_MASK_OK
    bool length_correct;    // data_len == 8
    int  bit_errors;        // 64 bit 중 틀린 수 (0~64)
    int  byte_errors;       // 8 byte 중 틀린 수 (0~8)
};
```

- `bit_errors` / `byte_errors` 타입은 **`int`** (지시서 초안의 `long long` 아님).

### `feed_raw_ext`에서 `m.pass` 설정 (L207–L242)

```207:242:HTS_TEST/t6_sim/HTS_T6_SIM_Test.cpp
    TrialMetrics m{};
    m.pass = false;
    m.crc_passed = false;
    m.length_correct = false;
    m.bit_errors = 64;
    m.byte_errors = 8;
    // ... Feed_Chip ...
    m.crc_passed = (g_last.success_mask == DecodedPacket::DECODE_MASK_OK);
    m.length_correct = (g_last.data_len == 8);

    if (m.crc_passed && m.length_correct) {
        int bit_err = 0;
        int byte_err = 0;
        // ... 8바이트 XOR popcount ...
        m.bit_errors = bit_err;
        m.byte_errors = byte_err;
        m.pass = (bit_err == 0);
    }
```

| 질문 | 답 |
|------|-----|
| `m.pass`를 콜백 내부에서 세팅하는가? | **No** — `on_pkt`는 `g_last`만 갱신; `pass`는 **`feed_raw_ext` 반환 직전**에만 계산 |
| `crc_passed` / `length_correct`와 `pass` 관계 | `m.pass == true`이려면 **`crc_passed && length_correct`가 참**이고, 그 안에서 **`bit_err == 0`**일 때만 `m.pass = true`. |
| CRC 실패 또는 길이 불일치 시 | `if` 블록 미진입 → **`m.pass`는 초기값 `false` 유지**, `bit_errors`/`byte_errors`는 초기 64/8 유지 |

---

## Q3. `test_S5` CFO 배열 (정확한 20점)

```775:780:HTS_TEST/t6_sim/HTS_T6_SIM_Test.cpp
    const double cfos[] = {
        0, 50, 100, 200, 500, 1000, 2000,
        2500, 3000, 3500, 4000, 4500,
        5000,
        7500, 10000,
        12500, 15000, 17500, 20000, 25000};
```

순서대로 (Hz):

`0`, `50`, `100`, `200`, `500`, `1000`, `2000`, `2500`, `3000`, `3500`, `4000`, `4500`, `5000`, `7500`, `10000`, `12500`, `15000`, `17500`, `20000`, `25000` — **총 20개**.

### 반복문

```781:781:HTS_TEST/t6_sim/HTS_T6_SIM_Test.cpp
    for (double cfo : cfos) {
```

- 형식: **`for (double cfo : cfos)`** (`const auto&` 아님).
