# HTS RX_LINE_TEST 프레임워크 설계서 v2
## (양산 평시 모드 + 무작위 변동 환경)

저장 인코딩: UTF-8 (LF). 편집기에서 한글이 깨지면 UTF-8로 다시 열기.

| 항목 | 내용 |
|---|---|
| 문서 ID | HTS_RX_LINE_TEST_v2.md |
| 작성일 | 2026-04-29 |
| 버전 | v2 (무작위 변동 환경, v1 시나리오 폐기) |
| 위치 | `D:\HTS_ARM11_Firmware\HTS_LIM\HTS_TEST\RX_LINE_TEST\` |
| 목표 | 양산 평시 모드 + 모든 파라미터 random 변동 환경에서 RX 라인 종합 검증 |
| 사용자 | 영준님 (VS 직접 빌드/실행) + Cursor (작업지시서로 코드 작성) |

---

## 0. 핵심 원칙 (절대 준수)

```
1. 모든 파라미터 random sweep (고정값 0)
2. 양산 평시 모드 (jam_harness 호출 X)
3. 매 패킷마다 환경 재생성 (캐싱 X)
4. 꼼수 / 변칙 / bypass 매크로 금지
5. 영준님 / Cursor 모두 이 문서를 단일 출처로 사용
6. baseline 갱신 시 별도 commit + 이력 기록
7. 양산 영향 0 (별도 폴더 RX_LINE_TEST/ 안에서만 작업)
```

---

## 1. 변경 사항 (v1 → v2)

| 항목 | v1 | v2 |
|---|---|---|
| 시나리오 | M1~M10 고정값 10 개 | 5 묶음 (random range) |
| CFO | 0/500/2k/5k/10k 고정 | Uniform(-12k, +12k) random |
| SNR | 5/15/30 고정 | Uniform(-15, +40) random |
| Multipath | 0 또는 2-tap 고정 | Random(0~4 tap) random |
| JSR | -∞ 또는 10dB | Uniform(-∞, +20dB) 10% 확률 |
| 패킷 수 | 시나리오당 명시 | 시나리오당 1000~10000 |
| 평가 | 시나리오별 BER threshold | 환경 bin 별 통계 |

---

## 2. 양산 모드 강제 (setup() 표준)

```cpp
HTS_V400_Dispatcher rx;
rx.Set_Seed(seed);
rx.Set_IR_Mode(true);                  // 양산 default (HARQ IR 모드)
// 적응형 IQ 동작은 Tick_Adaptive_BPS 자동 (강제 X)

// 절대 호출 금지:
// rx.Set_Lab_IQ_Mode_Jam_Harness();   ❌
// rx.Set_Lab_IQ_Mode_Independent();   ❌
```

추가 옵션 (시나리오별 평가용):

```cpp
// LPI 검증 시 (필요 시):
//   특수 모드 활성화는 시나리오별 정의에서 명시
```

---

## 3. 폴더 구조

```
D:\HTS_ARM11_Firmware\HTS_LIM\HTS_TEST\RX_LINE_TEST\
├── RX_LINE_TEST.cpp                  # 메인 (이미 영준님 생성)
├── RX_LINE_TEST.vcxproj              # MSBuild
├── HTS_RX_Channel.hpp                # 채널 시뮬 (random)
├── HTS_RX_Scenarios.hpp              # 시나리오 정의
├── HTS_RX_Metrics.hpp                # 메트릭 + 통계
├── HTS_RX_Random.hpp                 # mt19937 wrapper, 분포 helper
├── README.md                         # 이 문서 요약 + 사용법
├── baseline\
│   ├── baseline_v1_<YYYY-MM-DD>.json # baseline 점수
│   └── baseline_history.md           # 이력
└── results\
    ├── run_<timestamp>_full.txt
    ├── run_<timestamp>_metrics.txt
    └── run_<timestamp>_failed.txt    # worst case 패킷 환경 기록
```

---

## 4. 무작위 변동 환경 — 파라미터 분포

### 4-1. 채널 파라미터 (매 패킷 random)

| 파라미터 | 분포 | 범위 | 활성 확률 |
|---|---|---|---|
| CFO | Uniform | -12000 ~ +12000 Hz | 100% |
| 초기 phase | Uniform | 0 ~ 2π | 100% |
| Symbol time offset | Uniform | 0 ~ 64 chip | 100% |
| SNR | Uniform | -15 ~ +40 dB | 100% |
| DC offset I | Uniform | -200 ~ +200 | 100% |
| DC offset Q | Uniform | -200 ~ +200 | 100% |
| IQ amp imbalance | Uniform | -0.5 ~ +0.5 dB | 100% |
| IQ phase imbalance | Uniform | -3 ~ +3 ° | 100% |
| Multipath taps | Random | 0~4 (이산) | 100% |
| Multipath delay | Uniform | 1 ~ 20 chip | tap당 |
| Multipath gain | Uniform | 0.1 ~ 0.7 | tap당 |
| Multipath phase | Uniform | 0 ~ 2π | tap당 |
| Jammer 활성 | Bernoulli | - | 10% |
| JSR (활성 시) | Uniform | -10 ~ +20 dB | jammer 활성 시 |
| Jammer freq offset | Uniform | -50k ~ +50k Hz | jammer 활성 시 |
| Jammer 종류 | Random | CW / NB / Sweep | jammer 활성 시 |

### 4-2. TX 파라미터 (매 패킷 random)

| 파라미터 | 분포 | 비고 |
|---|---|---|
| seed | mt19937 random | 패킷 ID + 외부 seed |
| Payload 모드 | Random | DATA / VIDEO_1 / VIDEO_16 / VOICE 중 |
| Payload 데이터 | mt19937 random byte | fill_info random |
| Pre/Post guard | Random | 32~256 chip |

---

## 5. 시나리오 (5 개)

각 시나리오 = SNR / 환경 강도의 range 묶음. 시나리오 안에서도 모든 파라미터 random.

### 5-1. S_LIGHT (일반 양산 환경)

```
SNR : Uniform(20, 40) dB
CFO : Uniform(-3000, +3000) Hz       (TCXO ±5ppm 가정)
Multipath : 0~1 tap (50% 확률 1 tap)
Jammer : 0% 활성
패킷 수 : 5000
목표 BER : < 1e-4 (양산 정상)
```

### 5-2. S_NORMAL (양산 가혹)

```
SNR : Uniform(5, 25) dB
CFO : Uniform(-8000, +8000) Hz
Multipath : 0~2 tap
Jammer : 5% 활성, JSR -5~+10 dB
패킷 수 : 5000
목표 BER : < 1e-3
```

### 5-3. S_HARSH (재난안전망 worst)

```
SNR : Uniform(-10, +15) dB
CFO : Uniform(-12000, +12000) Hz
Multipath : 0~3 tap
Jammer : 20% 활성, JSR -5~+15 dB
패킷 수 : 5000
목표 BER : < 1e-2
```

### 5-4. S_LPI (저피탐 검증)

```
SNR : Uniform(-15, 0) dB              (잡음 레벨 이하)
CFO : Uniform(-5000, +5000) Hz
Multipath : 0~2 tap
Jammer : 0% (LPI 측정 방해 X)
패킷 수 : 10000                        (통계 안정성 위해 더 많이)
목표 BER : < 5e-2 (LPI 한계선 검증)
```

### 5-5. S_BARRAGE (jamming 100%)

```
SNR : Uniform(10, 30) dB
CFO : Uniform(-3000, +3000) Hz
Multipath : 0~1 tap
Jammer : 100% 활성, JSR Uniform(0, +20) dB
Jammer 종류 : Barrage (광대역 noise)
패킷 수 : 3000
목표 BER : < 1e-1 (64-chip 한계 ~14dB AJ 가정)
```

### 5-6. 시나리오 확장 정책

```
새 시나리오 = S_<NAME> 명시적 이름 (M1 등 인덱스 X)
시나리오 추가 시 baseline_history.md 기록
시나리오 삭제 금지 (기존 baseline 보존 위해)
파라미터 분포 변경 = 새 시나리오 (예: S_NORMAL_v2)
```

### 5-7. S_CFO_SWEEP (Phase 1 — CFO 단독, TASK-015b)

```
CFO          : -15000 ~ +15000 Hz, 100 Hz step (301 step)
SNR          : 30 dB 고정
Multipath    : 0
Jammer       : 0
Phase init   : Uniform(0, 2π) per packet
Symbol time  : Uniform(0, 63) chips per packet (RX 버퍼 정렬)
IQ / DC      : 0
패킷 수      : 각 CFO 100 packets → 총 30,100 packets

목적: CFO 추정·sync/decode vs CFO, AMI/LTE 한계 윤곽 (PASS/FAIL 기준 없음)
구현: HTS_TEST/RX_LINE_TEST/ (baseline_phase1_*.json)
```

---

## 6. RX 파이프라인 측정 포인트 (19 단계)

| ID | 단계 | 카운터 | 의미 |
|---|---|---|---|
| C01 | RAW | rx_feed_count | Feed_Chip 호출 |
| C02 | DC | dc_active_count | DC 제거 적용 |
| C03 | AGC | agc_active_count | AGC 적용 |
| C04 | CFO_APPLY | cfo_apply_count | Apply_Per_Chip 호출 |
| C05 | HOLO_LPI | holo_lpi_count | apply_holo_lpi_inverse |
| C06 | P0_BUF | p0_buf_count | preamble 누적 |
| C07 | FWHT_SYNC | fwht_sync_count | sync FWHT |
| C08 | V5_SCORE | v5_score_count | preamble v5 score |
| C09 | SEED_LOCK | seed_lock_count | sync 성공 |
| C10 | HEADER_BUF | hdr_buf_count | header 누적 |
| C11 | HDR_PARSE_OK | hdr_parse_ok | parse_hdr_ 성공 |
| C12 | PAYLOAD_BUF | payload_buf_count | payload 누적 |
| C13 | ON_SYM_ENTER | on_sym_enter | on_sym_ 진입 |
| C14 | PERMUTE | permute_count | Walsh permute |
| C15 | SYM_DECODE | sym_decode_count | sym 비트 결정 |
| C16 | HARQ_ACCUM | harq_accum_count | HARQ 누적 |
| C17 | DECODE_ENTER | decode_enter_count | try_decode_ 진입 |
| C18 | CRC_OK | crc_ok_count | CRC 통과 |
| C19 | BIT_MATCH | bit_match_count | memcmp 통과 |

각 시나리오 종료 시 19 단계 카운트 출력.

---

## 7. 메트릭 (통계 기반)

### 7-1. 패킷 단위 (PacketResult)

```cpp
struct PacketResult {
    // 환경
    int32_t cfo_hz;
    double  snr_db;
    int     n_taps;
    bool    jammer_on;
    double  jsr_db;

    // RX 결과
    bool    sync_ok;
    bool    hdr_ok;
    int     payload_recv;
    int     payload_total;
    bool    decode_ok;
    int     bit_errors;
    int     total_bits;
};
```

### 7-2. SNR / CFO bin 통계

```
SNR bin (5 dB 단위, -15~+40 = 11 bin)
CFO bin (1 kHz 단위, ±12 = 25 bin)

각 bin 별:
  - n_packets
  - sync_rate (%)
  - decode_rate (%)
  - BER
```

### 7-3. Worst case 기록

```
실패 패킷 (decode_ok=false 또는 BER>0.1) 의 환경 파라미터 누적 저장
→ failed_packets.csv 생성
→ 영준님이 실패 환경 분석 가능
```

### 7-4. 종합 출력

```
========== HTS RX_LINE_TEST v2 ==========
환경: ir_mode=true, jam_harness=OFF, 적응형 IQ default
시각: 2026-04-29 HH:MM:SS
빌드: Release (LEGACY)

[S_LIGHT]   n=5000   sync=98.2%  decode=96.1%  BER=2.3e-5  PASS
[S_NORMAL]  n=5000   sync=85.3%  decode=78.1%  BER=4.5e-4  PASS
[S_HARSH]   n=5000   sync=42.1%  decode=31.2%  BER=8.2e-3  PASS
[S_LPI]     n=10000  sync=18.4%  decode=12.5%  BER=2.1e-2  PASS
[S_BARRAGE] n=3000   sync=12.1%  decode= 8.3%  BER=6.4e-2  PASS

SNR bin 통계 (S_NORMAL):
  -15..-10  sync=  0.5%  BER= 0.495
  -10..-5   sync=  3.2%  BER= 0.432
  ...
   30..40   sync=100.0%  BER= 1.2e-6

CFO bin 통계 (S_NORMAL):
  -12k..-10k  sync= 65%  BER= 1.5e-3
  ...
   -1k..+1k   sync= 95%  BER= 5.0e-5
  ...

Worst case 환경 (5000개 중 fail 247개):
  → results\run_<ts>_failed.csv
========================================
```

---

## 8. 채널 시뮬 구현 (random)

```cpp
// HTS_RX_Channel.hpp
struct ChannelParams {
    int32_t cfo_hz;
    double  initial_phase_rad;
    int     time_offset_chips;
    double  snr_db;
    int32_t dc_i;
    int32_t dc_q;
    double  iq_amp_imb_db;
    double  iq_phase_imb_deg;
    int     n_taps;
    int     tap_delay[4];
    double  tap_gain[4];
    double  tap_phase[4];
    bool    jammer_on;
    double  jsr_db;
    int32_t jam_freq_offset_hz;
    int     jam_kind;        // 0=CW, 1=NB, 2=Sweep, 3=Barrage
};

void apply_channel(const int16_t* tx_I, const int16_t* tx_Q, int n,
                   ChannelParams& p, mt19937& rng,
                   int16_t* rx_I, int16_t* rx_Q) noexcept;

// 적용 순서:
//   1. multipath (FIR)
//   2. CFO + initial_phase
//   3. IQ imbalance
//   4. DC offset
//   5. AWGN (SNR 기반)
//   6. Jammer (활성 시)
//   7. Time offset (앞뒤 zero-pad)
```

---

## 9. baseline 관리

### 9-1. JSON 형식

```json
{
  "version": "v1",
  "date": "2026-04-29",
  "git_commit": "<hash>",
  "build": "Release",
  "scenarios": {
    "S_LIGHT":   { "n": 5000,  "sync_rate": 0.982, "decode_rate": 0.961, "ber": 2.3e-5 },
    "S_NORMAL":  { "n": 5000,  "sync_rate": 0.853, "decode_rate": 0.781, "ber": 4.5e-4 },
    "S_HARSH":   { "n": 5000,  "sync_rate": 0.421, "decode_rate": 0.312, "ber": 8.2e-3 },
    "S_LPI":     { "n": 10000, "sync_rate": 0.184, "decode_rate": 0.125, "ber": 2.1e-2 },
    "S_BARRAGE": { "n": 3000,  "sync_rate": 0.121, "decode_rate": 0.083, "ber": 6.4e-2 }
  },
  "snr_bins": { "S_NORMAL": { "-10..-5": {}, "...": {} } }
}
```

### 9-2. 비교 명령

```cmd
RX_LINE_TEST.exe --compare baseline\baseline_v1_2026-04-29.json
   → 시나리오별 BER/sync/decode 비교
   → 회귀 시 exit code 1
```

### 9-3. 갱신

```cmd
RX_LINE_TEST.exe --save-baseline baseline\baseline_v2_<date>.json
git add baseline\baseline_v2_<date>.json baseline\baseline_history.md
git commit --trailer "Made-with: Cursor" -m "[baseline] v2 <date> <reason>"
```

baseline_history.md 표 형식:

```
| 버전 | 날짜 | git commit --trailer "Made-with: Cursor" | 변경 사유 | S_LIGHT BER | S_NORMAL BER | ... |
```

---

## 10. 빌드 / 실행

### 10-1. 빌드 (VS 또는 명령줄)

```cmd
cd /d D:\HTS_ARM11_Firmware\HTS_LIM\HTS_TEST\RX_LINE_TEST
msbuild RX_LINE_TEST.vcxproj /p:Configuration=Release /p:Platform=x64 /t:Rebuild
```

### 10-2. 실행

```cmd
REM 기본
x64\Release\RX_LINE_TEST.exe

REM seed 지정 (재현 가능)
x64\Release\RX_LINE_TEST.exe --seed 12345

REM 시나리오 지정
x64\Release\RX_LINE_TEST.exe --scenario S_NORMAL

REM baseline 비교
x64\Release\RX_LINE_TEST.exe --compare baseline\baseline_v1_2026-04-29.json

REM 결과 저장
x64\Release\RX_LINE_TEST.exe > results\run_<ts>_full.txt
```

---

## 11. 영준님 / Cursor 협업

### 11-1. 영준님 (VS)

```
- VS 솔루션에서 RX_LINE_TEST 프로젝트 빌드
- 직접 실행 + baseline 측정
- 시나리오 / 분포 / 메트릭 정의 변경 결정권
- 양산 baseline 승인
```

### 11-2. Cursor (작업지시서)

```
- 영준님 결정 따라 코드 작성 / 수정
- 자동 빌드 / 회귀 측정
- baseline 비교 자동
- 시나리오 추가 자율 X (영준님 승인 후만)
```

### 11-3. 공동 업데이트

```
- 이 설계서 변경 시 양쪽 commit
- 시나리오 / 메트릭 추가 시 이 문서 갱신 우선
- baseline 회귀 발견 시 즉시 양쪽 보고
```

---

## 12. 구현 단계

### Phase 1 (TASK-015b) — 골격 + S_LIGHT 만

```
- HTS_RX_Random.hpp (mt19937 wrapper)
- HTS_RX_Channel.hpp (CFO + AWGN 만, 단순)
- HTS_RX_Scenarios.hpp (S_LIGHT 1 개)
- RX_LINE_TEST.cpp (메인 + setup 양산 모드 + 단계별 카운터 19)
- 첫 baseline 측정 (S_LIGHT 1000 패킷)
```

### Phase 2 (TASK-016) — 채널 확장

```
- multipath / IQ imbalance / DC offset 추가
- S_NORMAL, S_HARSH 시나리오 추가
```

### Phase 3 (TASK-017) — Jammer + LPI

```
- Jammer 4 종류 추가
- S_LPI, S_BARRAGE 시나리오
- LPI 메트릭 (sync rate vs SNR)
```

### Phase 4 (TASK-018) — 메트릭 + baseline JSON

```
- PacketResult 누적
- SNR/CFO bin 통계
- baseline JSON 저장 / 비교
- worst case CSV
```

### Phase 5 (TASK-019~) — 본진 추적

```
- baseline 분석
- failed 패킷 환경 분석
- 본진 식별 + fix
- 회귀 검증 + baseline 갱신
```

---

## 13. 양산 영향 보장

```
RX_LINE_TEST/ 폴더 안에서만 작업
HTS_LIM/ 양산 코드 0 수정 (모든 측정은 외부 wrapper)
다른 테스트 (HTS_T6_SIM_Test 등) 0 영향
baseline JSON 외 양산 commit X
```

---

## 14. 영준님 검토 항목 (이 문서)

검토 후 수정 의견 부탁드립니다:

```
[ ] 시나리오 5 개 (S_LIGHT/NORMAL/HARSH/LPI/BARRAGE) 적절한가?
[ ] 파라미터 분포 (4-1) 합리적인가?
[ ] 패킷 수 (1000~10000) 충분한가?
[ ] BER 목표값 적절한가?
[ ] 19 단계 카운터 다 필요한가?
[ ] SNR / CFO bin 단위 적절한가?
[ ] worst case CSV 필요한가?
[ ] Phase 1~5 분할 적절한가?
[ ] 추가/삭제할 항목?
```

---

## 15. 다음 작업

영준님 검토 → 승인 / 수정 → 설계서 v3 (필요 시) → TASK-015b (Phase 1 골격) 작업지시서 발행.

---

## 부록 — 문서 정합성 (구현 시)

| 항목 | 메모 |
|---|---|
| §4-1 vs §5 | 글로벌 분포와 시나리오 제약이 겹칠 때 **시나리오가 샘플링 범위를 클립**하는 규칙을 명시 권장. |
| §1 vs §4-1 Jammer | 표는 10% 언급, §5는 시나리오별 0~100% — **시나리오 우선**. |
| §12 Phase 1 | S_LIGHT 최종 n=5000(§5-1)과 Phase 1 n=1000은 **스모크/CI vs 공식 baseline**으로 구분 권장. |
| §7-3 vs §3 | 실패 로그 파일명 failed.txt vs failed.csv — **확장자 통일** 권장. |
