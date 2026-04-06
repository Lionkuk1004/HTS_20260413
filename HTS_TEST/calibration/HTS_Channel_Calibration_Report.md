# HTS 채널 모델 A/B 진단 보고서 (코드 조사)

**위치:** `HTS_TEST/calibration/` (PC 전용 — ARM·`hts_cs_core` 미포함)  
**작성:** 통신 시뮬레이션 진단용  
**전제:** `apply_channel`(A) 및 `LTE_Channel`(B) **원본 소스는 변경하지 않음.**

---

## 1단계 — 비교 표

| 항목 | apply_channel (A) | LTE_Channel (B) | 불일치 여부 |
|------|-------------------|------------------|------------|
| **파일 경로** | `HTS_TEST/종합재밍_종합_테스트.cpp` 내 `apply_channel()` (익명 `namespace`) | `HTS_LIM/HTS_3D_Tensor_FEC.cpp` `LTE_Channel::Transmit` / `Transmit_To`, 상수는 `HTS_LIM/HTS_3D_Tensor_FEC.h` | **링크 범위:** A는 TU 내부 전용 |
| **스프레드 이득 계산식** | `spread_gain = kNumChips = 128`; 수신항 `tx[i] * spread_gain` (선형 진폭 ×128) | `tensor[i] * NUM_CHIPS`, `NUM_CHIPS = 128` | **형태 일치** (둘 다 ×128) |
| **AWGN σ 공식** | `signal_power = spread_gain²`; `noise_sigma = sqrt(signal_power / SNR_linear)`; `SNR_linear = 10^(intensity_db/10)`; 잡음 `N(0, noise_sigma)` | **별도 AWGN 모드 없음.** 잡음은 `JS_DB`로부터 `sigma_chip = sqrt(10^(JS_DB/10))`, `sigma_total = sigma_chip * sqrt(NUM_CHIPS)` 인 **가우시안**만 존재 | **불일치:** A는 SNR 스윕 채널, B는 고정 J/S 기반 σ |
| **바라지 J/S → σ 변환** | `js_linear = 10^(J/10)`; `jam_sigma = sqrt(js_linear) * spread_gain`; 샘플마다 `N(0,jam_sigma)` + `base_noise` | J/S를 **샘플별 독립 가우시안**으로 쓰지 않고, **σ를 JS_DB 한 점에서 결정** 후 `noise(rng)`에 사용 | **불일치:** 스케일·의미(고정 vs 스윕) 모두 상이 |
| **CW 톤 주입 대역** | **부분 대역:** 중심 `N/4`, 반폭 `N/16` → 길이 `N/8` 구간에만 `sin` | **CW 경로 없음** | **불일치:** A만 CW; B는 해당 없음 |
| **EMP 파괴 단위** | 텐서 **요소(샘플) 인덱스 `i`** 단위로 `u01 < destroy_rate` 판단 | 동일하게 **텐서 요소 인덱스** 단위 (`for i < N`) | **단위 일치(샘플/노드)** — “칩”이라는 별도 레이어 없음 |
| **EMP 비율** | `destroy_rate = intensity_db / 100` → **스윕 가변** (0~1) | `EMP_RATE = 0.03` **컴파일 타임 고정** | **불일치:** 가변 vs 고정 |
| **base_noise 값** | `base_noise_sigma = 0.01` — **바라지·CW·EMP(비파괴 샘플)** 에 사용; AWGN 경로에는 미사용 | **명시적 `base_noise` 없음** (EMP 아닌 샘플은 `tensor[i]*NUM_CHIPS + noise`) | **불일치:** A만 추가 미소 잡음 |
| **시드 관리** | 호출자 `std::mt19937& rng` — **재현 가능** (시드만 고정하면 됨) | 동일 `std::mt19937& rng` | **동일 패턴** — 다만 B는 강도 스윕이 없어 **동일 매트릭스 비교 불가** |

---

## 2단계 — 불일치 항목별 근거 / [요검토]

(이전 버전과 동일 — 본 파일은 `HTS_TEST/calibration/` 으로만 이동됨.)

---

## 3단계 — 구현 위치 (격리 후)

- `HTS_TEST/calibration/HTS_Channel_Calibration.h` / `.cpp`
- `HTS_TEST/calibration/HTS_Channel_Calibration_Test.cpp` (`HTS_PC_CALIBRATION_RUN` 정의 시에만 CSV `main` 빌드)
- 빌드: **`HTS_TEST/CMakeLists.txt`** → `hts_pc_layer2_tensor_fec` **정적** + `HTS_Calibration_Run` (**캘리브레이션 CSV만** — `HTS_Fractal_Channel_Compare.cpp` 는 별도 타깃)
- 프랙탈 와이어 비교 + **FEC_HARQ V400 Chase/IR CSV**: CMake 타깃 **`HTS_Fractal_Channel_Compare_Run`** (`benchmark/HTS_Fractal_Channel_Compare.cpp`)

---

## 4단계 — 빌드 (HTS_TEST 단독)

```text
cmake -S HTS_TEST -B HTS_TEST/build-calibration -G "Visual Studio 18 2026" -A x64
cmake --build HTS_TEST/build-calibration --config Release
```

실행 후 작업 디렉터리에 `HTS_Channel_Calibration_Out.csv` 생성.

`HTS_Fractal_Channel_Compare_Run` 실행 시 `HTS_Fractal_Channel_Compare_Out.csv`(텐서·프랙탈) 및 `HTS_FEC_V400_IR_Compare_Out.csv`(**FEC_HARQ `Encode64_IR` / `Decode64_IR`**) 가 생성된다.

**주의:** `HTS_검증_종합재밍.vcxproj` → `JamCombo_TensorHARQ.exe` 는 **캘리브레이션 `main` 미포함**; 프랙탈+FEC 벤치는 동 프로젝트에 포함. 캘리브레이션 전용은 CMake `HTS_Calibration_Run` 사용.
