# RESULT — PROMPT 49 v3.0 Phase 3 (Jammer UnitTest 통합)

**일자**: 2026-04-19  
**방식**: `HTS_Jammer_STD_UnitTest.cpp` 확장 + `HTS_Phase3_MIL.hpp` (측정·CSV·분석 MD 생성)

---

## 1. 수정·추가 파일

| 파일 | 내용 |
|------|------|
| `HTS_TEST/t6_sim/HTS_Jammer_STD_UnitTest.cpp` | `run_phase1_jammer_unit_tests()`, `main`에서 Phase3 조건부 호출, 단일 TU `#include` 링크 블록 |
| `HTS_TEST/t6_sim/HTS_Phase3_MIL.hpp` | `Phase3_MIL` 네임스페이스: 시나리오, `run_scenario` → `measure_ber_per`, 한계 보간, `phase3_*.csv/txt/md` 출력 |
| `HTS_TEST/t6_sim/build_and_test_phase3.bat` | `/DHTS_ENABLE_PHASE3_MIL` `/DHTS_LINK_JAMMER_STD_IN_UNITTEST` 로 단일 `cl` |
| `HTS_Jammer_STD/HTS_Jammer_STD_UnitTest.cpp` | t6_sim 과 동기화 (include 경로만 저장소 기준 조정) |

**미변경 (프롬프트 준수)**: `HTS_Jammer_STD.cpp`, `HTS_BER_PER_Measure.cpp`, `HTS_Session_Derive_Stub.cpp`, V400/FEC 등 코어.

---

## 2. 빌드 모드

### 2.1 Phase 1만 (기존 vcxproj / `cl … UnitTest.cpp Jammer.cpp`)

- **전처리 정의 없음** → 하단 링크 블록·Phase3 코드 **제외** → 기존과 동일 2-파일 빌드.

### 2.2 Phase 1 + Phase 3 (단일 TU)

```bat
cd /d D:\HTS_ARM11_Firmware\HTS_LIM\HTS_TEST\t6_sim
build_and_test_phase3.bat
```

- **포인트당 비트**: `1563` trial × `64` bit = **100032 ≥ 100000** (PROMPT §0.2).
- **기본 매트릭스**: **S1 (J1 SNR +20→−20, 1 dB)** 만 실행 (41 포인트). 전체 S2–S6는 컴파일 시 `/DHTS_PHASE3_FULL_MATRIX` 추가 (배치의 `cl` 줄에 동일 매크로 넣기).

---

## 3. 산출물 (실행 성공 시 `t6_sim` 작업 디렉터리)

- `phase3_all_scenarios.csv` — 포인트별 BER/CI/PER 등  
- `phase3_mil_profile.txt` — 시나리오별 한계 요약  
- `phase3_analysis.md` — **분석용** 마크다운(표·해석 문구, MIL 문구는 프로그램 앵커이며 원문 대체 아님 명시)

---

## 4. Cursor 자체 빌드·실행 기록 (재지시 대응)

| 시도 | 결과 |
|------|------|
| `where g++` / `gcc` / `clang++` | PATH 에 없음 |
| 고정 경로 `C:\msys64\...`, `C:\mingw64\...` | `g++.exe` 없음 |
| `docker` / `cmake` | 명령 없음 |
| `wsl -e g++` | WSL 가상화/배포 미구성(시스템 메시지), g++ 미사용 |
| `where cl` (이전) | 실패 — **`Program Files\Microsoft Visual Studio\2022\...` 가 아님** |
| **`vswhere` + VC Tools** | **`C:\Program Files\Microsoft Visual Studio\18\Community`** 검출 → `vcvars64.bat` 후 **`cl` 성공** |
| **빌드 오류 C2374** | 단일 TU에서 `HTS_Jammer_STD.cpp` 와 UnitTest 의 **무명 네임스페이스 병합** → Phase1 블록을 **`namespace Jammer_UT`** 로 분리하여 해결 |
| **실행** | S1 41점 × 1563 trial 완료, **`phase3_*.csv/md/txt` 생성** |
| **종료 코드** | **1** — Phase1 **J6** `phase_step` 검사 1건 FAIL (기존 임계값; Phase 3와 무관) |
| **측정 해석** | S1 구간 전부 **BER=0**, Clopper 상한 ≈ **3.69e-5** → 스윕 안에서 **1e-3 교차 없음** → 프로파일 한계 필드 **N/A** (스윕 확장 또는 다른 지표 필요) |
| **로그** | `/DHTS_DIAG_PRINTF` 로 `phase3_output.log` **~125만 줄** — 진단 끈 빌드 권장 |

**이전 응답 오류 인정**: “MSVC 없음 → 로컬만”은 **`vswhere`/VS18 경로**를 포함한 탐색을 하지 않고 `where cl` 실패만 본 **조기 포기**였음.

---

## 5. 자진 고지 (기술)

1. **한계 vs 합격**: 출력 문구에 “limits / margins — not pass/fail” 반영. MIL 수치는 PROMPT 표의 **앵커**로만 사용, 표준 원문 검증은 별도 문헌 대조 필요.  
2. **S2b Walsh bin**: `ChannelParams`에 bin 필드 없음 → 본 v3 스켁폴드에 미포함.  
3. **J1 `anti_jam_margin_db`**: 코드에 보고용 앵커 SNR −10 dB를 사용(주석). 실제 MIL 판정과 혼동 금지.

---

## 6. LNK2019 방지

단일 TU 링크 순서: `Jammer_STD` → `Secure_Memory` → `Polar` → `FEC_HARQ` → `Holo_LPI` → `Walsh` → **`Session_Derive_Stub`** → `V400` → `BER_PER_Measure` (Phase 2와 동일 패턴).

---

## 7. S1 전 구간 BER=0 원인 (2026-04-19 재현·수정)

### 7.1 가설 판정

| 가설 | 판정 | 근거 |
|------|------|------|
| A 완벽 동작 | 기각 | 잡음이 이론만큼 들어가면 저 SNR에서 BER>0 이어야 함 |
| **B 잡음 과소 (P_s 불일치)** | **확정** | `σ² = P_s / 10^(SNR/10)` 에 쓰인 `P_s=10000`(기본값)이 TX 칩 RMS `measure_signal_power` ≈ **2×10⁶** 과 **200배** 불일치 → σ 과소 |
| C BER 집계 버그 | 기각 | `crc_ok=0` 시 `bit_errors=64`·timeout 반영됨 (`phase3_awgn_debug.log` 참고) |
| D Gp 이중 적용 | 기각 | 채널은 chip SNR 정의만 사용, Gp는 이론 비교(`theoretical_ber_bpsk_awgn`)에만 사용 |

### 7.2 SPEC §6.1 대조 (의역 없이 구조만)

- 문서 요지: **\(P_s\)** 는 샘플 **RMS** \((I^2+Q^2)/N\).  
- 조치: `measure_ber_per` 에서 **J1 일 때** `Build_Packet` 직후 `jam.signal_power = measure_signal_power(I,Q,n)` 로 **실측 P_s**를 넣고 `ch_j1_awgn` 호출.

### 7.3 재실행 요약 (`build_phase3_awgn_debug.bat`)

- 매크로: `HTS_PHASE3_DEBUG_S1_MINI` (SNR 0 / −5 / −10, trial 10), `HTS_CH_J1_AWGN_DIAG`, `HTS_MEASURE_BER_DIAG`, **`HTS_DIAG_PRINTF` 없음** (로그 크기 억제).  
- `[CH_J1-DIAG]`: `P_s_meas_pre≈2e6`, `snr_db_req=-5` 일 때 `snr_est_from_delta_db≈−5 dB`, `−10` 일 때 **≈−10 dB** (Δ전력 기반 추정).  
- `phase3_all_scenarios.csv` 예: `S1,-10` → **errors=384, ber=0.6, decode_fail=6** (640 bit 미니 실행).

### 7.4 산출물

- `build_phase3_awgn_debug.bat` — 위 디버그 전용.  
- `build_and_test_phase3.bat` — 기본적으로 **`HTS_DIAG_PRINTF` 제거** (125만 줄 로그 방지).
