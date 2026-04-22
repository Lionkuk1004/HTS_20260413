# T6 chip rate 매크로 분기 결과

## 커밋·태그

- **커밋 ID**: `git rev-parse T_CHIPRATE_PARAM^{}` (annotated tag이면 이 명령으로 **구현 커밋** 확인)
- **커밋 메시지**: `t6_chiprate: kChipRate 매크로 분기 (AMI 200k, PS-LTE 1M)`
- **태그**: **`T_CHIPRATE_PARAM`** — *T6 chip_rate 매크로 분기. AMI 200k 유지, PS-LTE 1M 첫 측정*

## 변경

- **파일**: `HTS_TEST/t6_sim/HTS_T6_SIM_Test.cpp`
- **`kChipRate` 선언** (기존 단일 `constexpr` → 조건부 분기):
  - **`#if defined(HTS_TARGET_AMI)`** → `200000.0` (200 kcps)
  - **`#else`** → `1000000.0` (1 Mcps)
- **`ch_cfo` (L341 근처)**: 수식 변경 없음 — `kChipRate` 참조만으로 자동 반영

## 빌드

| 대상 | 결과 | 로그 |
|------|------|------|
| AMI | **성공** | `HTS_TEST/t6_sim/build_ami_chiprate.log` |
| PS-LTE | **성공** | `HTS_TEST/t6_sim/build_pslte_chiprate.log` |

## 실측 (1회)

### AMI (200 kcps, 회귀)

- **로그**: `HTS_TEST/t6_sim/ami_chiprate_run1.log`
- **정량 합계**: **13839 / 15200 (91.0%)** — BER **0.08954**
- **13839 유지**: **Y**

### PS-LTE (1 Mcps, 첫 측정)

- **로그**: `HTS_TEST/t6_sim/pslte_chiprate_run1.log`
- **정량 합계**: **13939 / 15200 (91.7%)** — BER **0.08296**
- **이전 (동일 Sync·200 kcps 채널 모델 시)**: 13839 / 15200, BER 0.08954
- **변화**: 통과 trial **+100** (비율 **+0.7%p**), BER **감소**

## 시나리오 비교

### S5 CFO sweep (`test_S5` 행 + 요약 테이블)

| CFO (Hz) | AMI (200 kcps) `row` | PS-LTE (1 Mcps) `row` | 비고 |
|----------|----------------------|-------------------------|------|
| 0 | 100/100 PASS | 100/100 PASS | 동일 |
| 50 | 100/100 PASS | 100/100 PASS | 동일 |
| 100 | 100/100 PASS | 100/100 PASS | 동일 |
| 200 | 100/100 PASS | 100/100 PASS | 동일 |
| 500 | 100/100 PASS | 100/100 PASS | 동일 |
| 1000 | 100/100 PASS | 100/100 PASS | 동일 |
| 2000 | 100/100 PASS | 100/100 PASS | 동일 |
| **5000** | **0/100 FAIL** | **100/100 PASS** | **1 Mcps에서만 S5 최고 CFO 통과** |

요약 테이블 (`║ S5 │ …Hz`)도 동일하게 **5000 Hz**: AMI **FAIL**, PS-LTE **PASS**.

### S1 / S2 / S3 / S4 (요약 테이블 상단)

- **S1 Clean, S2 위상, S3/S4**: AMI·PS-LTE **동일 패턴** (로그 상단 `║ S1`…`║ S4` 줄 비교 시 수치 일치).

## 통신공학 평가

- **동일 Hz·동일 칩 인덱스**에서 위상 누적은 **\(\propto 1/R_\mathrm{chip}\)** → **1 Mcps 시 CFO에 의한 칩당 회전이 약 1/5** → **S5 @5000 Hz 가 PS-LTE에서만 PASS**로 나타남 → **CFO 완화 효과: Y**
- **“AMI의 CFO 난이도 ≈ PS-LTE에서 5× Hz”** 같은 스케일링은 **본 로그의 단일 포인트(5000 Hz)** 에서 **정성적으로 일치**; 전 구간에 대한 엄밀 검증은 **추가 측정** 필요.

## 다음 단계 후보

- **[ ]** PS-LTE 1 Mcps에서 **CFO 스윕을 Hz 상향**(예: 25k–100k)해 **동등 난이도** 재현
- **[ ]** AMI 200 kcps에서 **S5 세분 측정**(한계 Hz 근처)
- **[ ]** AMI PHY 실험(32×2 등)은 **Sync_AMI** 측과 별도 정책
