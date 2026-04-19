# RESULT — Prompt 48 Phase 2 (BER/PER + Clopper-Pearson + 이론 비교)

**날짜**: 2026-04-19  
**범위**: `HTS_BER_PER_Measure.hpp/.cpp`, `HTS_BER_PER_UnitTest.cpp` 신규. `HTS_Jammer_STD.hpp/cpp`, `HTS_T6_SIM_Test.cpp`, V400 코어 **미변경**.

### 1.0 재작업 — LNK2019 (`Session_Gateway::Derive_Session_Material`)

- **원인**: `HTS_Walsh_Row_Permuter.cpp`가 `Session_Gateway::Derive_Session_Material`을 참조하는데, 단일 TU 빌드에 구현 TU가 없었음.  
- **근거**: `HTS_T6_SIM_Test.vcxproj`는 동일 목적으로 `HTS_TEST\t6_sim\HTS_Session_Derive_Stub.cpp`를 **별도 ClCompile**로 링크함.  
- **조치**: `HTS_BER_PER_UnitTest.cpp` 하단에서 `HTS_Walsh_Row_Permuter.cpp` 직후 `#include "HTS_Session_Derive_Stub.cpp"` 추가.  
- **비고**: `HTS_Session_Gateway.cpp` 전체를 넣지 않음(PC 하네스·의존성 폭발 방지). 스텁은 `HTS_WALSH_ROW_PERM` 도메인만 고정 시드 반환(파일 내 경고 참고).

---

## 1. 구현 요약

| 항목 | 내용 |
|------|------|
| Clopper-Pearson | `boost::math` **미사용**. 불완전 베타 `I_x(a,b)` = NR 스타일 `betacf` + `lgamma` 기반, `x=0`/`x=n` 경계는 SPEC §11.1 폐쇄식. 내부 역분위는 이분 탐색. |
| BER/PER | V400 `Build_Packet` → `ch_j1`…`ch_j6` → `Feed_Chip` 복조, 페이로드 **64비트/패킷**(8바이트), T6 `feed_raw_ext`와 동일 계열. |
| CI 판정 | `pass_ber` = `ber_ci_upper < target_ber`, `pass_per` = `per_ci_upper < target_per` (α=0.05 고정). |
| 이론 BER | `0.5 * erfc(sqrt(Eb/N0_linear))`, `Eb/N0_dB = snr_db + gp_db` (기본 Gp=18.06 dB). |
| `ber_matches_theory` | **단방향 상한**: 시뮬이 이론보다 좋으면 통과; 나쁠 때는 `10·log10(sim/theory) ≤ tol_db`. (확산·FEC로 이론보다 수 dB 좋은 경우 양방향 ±1 dB는 물리적으로 성립하지 않음 — 자진 고지 §3 참고.) |
| 시드 | `derive_seed(base, trial, "ber_per")` (TX/RX), `derive_seed(..., "jammer_ch")` (잼머 RNG). |

---

## 2. Clopper-Pearson 정확성 (단위 시험 설계)

`HTS_BER_PER_UnitTest.cpp` 에 포함:

1. **문헌 예** (n=107, x=85, α=0.05) → 하한 ≈0.7044, 상한 ≈0.8586 (각 ±0.01).
2. **경계** x=0 (n=1000), x=n (n=100).
3. **중간** n=10000, x=10 → 하한 ≈4.79×10⁻⁴, 상한 ≈1.84×10⁻³ (문헌/SciPy 기준, 완화 공차).
4. **무효 입력** n≤0, x<0, x>n → `[0,1]`.

**본 Cursor 실행 환경**: MSVC / Python(SciPy) 미탑재로 **여기서는 `HTS_BER_PER_UnitTest.exe` 를 실행하지 못함**. 로컬에서 아래 빌드 후 로그를 남겨 주십시오.

```bat
cd /d D:\HTS_ARM11_Firmware\HTS_LIM\HTS_TEST\t6_sim
build_and_test_phase2.bat
```

수동 `cl` 한 줄은 `HTS_BER_PER_UnitTest.cpp` 상단 주석 참고. Cursor 터미널에서 `cl` 이 안 뜨면 `CURSOR_BUILD_ENV_PHASE2.md` 의 `vcvars64` 프로필 설정을 적용한다.

권장 추가 로그 파일명(프롬프트 §7): `t6_phase2_ber_per.log`, `t6_phase2_theory_compare.log` — 동일 실행으로 통합 가능.

---

## 3. UT1–UT7 (인프라 검증)

| ID | 조건 | 단위 시험에서의 판정 요지 |
|----|------|---------------------------|
| UT1 | J1, SNR=-10 dB | `ber_matches_theory` (상한 1 dB) + 측정 완료 |
| UT2 | J1, SNR=0 dB | `ber_ci_upper < 1e-2` |
| UT3 | J1, SNR=+10 dB | `ber_ci_upper < 1e-3` (무에러 근접 시 CP 상한으로 검증) |
| UT4–UT7 | J2/J4/J5/J6 표 파라미터 | 총 비트·BER/PER·timeout·sat 집계 정상 |

각 시나리오 **trial=160** → **10,240 bit ≥ 10,000** (Phase 2 UT 최소). Phase 3 정식 시험은 SPEC대로 **≥100,000 bit** 유지.

---

## 4. 회귀 (V34 Fix A3)

- **소스**: `HTS_T6_SIM_Test.cpp` **비수정** 유지.
- **기대**: 기존과 동일 **10927 / 11200** — Phase 1 보고(`RESULT_PROMPT_47_PHASE1_v1.1.md`)와 동일하게 로컬에서 `HTS_T6_SIM_Test_V34_FIXA3.exe` 재실행 후 `t6_phase2_regression.log` 저장 권장.

---

## 5. 자진 고지

1. **Pre-FEC vs Post-FEC**: 본 측정은 V400 **복조 후 8바이트 페이로드** 기준 **Post-FEC(또는 디코더 출력) BER**에 해당. Pre-FEC 칩/심볼 BER은 본 Phase 2 인프라에 포함하지 않음.
2. **`ber_matches_theory`**: 프롬프트의 “\|sim−theory\| ≤ 1 dB”를 문자 그대로 `fabs` 로 쓰면, FEC/확산으로 **이론보다 훨씬 좋은** BER에서 항상 실패한다. 그래서 **이론보다 나쁘지 않은 경우 통과**, 나쁜 경우에만 +1 dB 상한을 적용했다.
3. **Boost**: 요청 시 권장이었으나, 툴체인 무관 **헤더만** 동작을 위해 Beta/CP는 **자체 수치**로 구현. 이후 Boost 도입 시 `clopper_pearson_ci` 만 교체 가능.
4. **실행 로그**: 이 환경에서 `.exe` 미생성 → 위 §2 명령으로 사용자 측에서 로그 확보 필요.

---

## 6. 산출물 체크리스트

- [x] `HTS_BER_PER_Measure.hpp` / `.cpp`
- [x] `HTS_BER_PER_UnitTest.cpp`
- [ ] `HTS_BER_PER_UnitTest.exe` + `t6_phase2_*.log` (**로컬 빌드·실행**)
- [x] `RESULT_PROMPT_48_PHASE2.md` (본 문서)
- [ ] 회귀 10927/11200 (**로컬 재실행**)

Phase 3 진행 전, 위 미체크 항목을 로컬에서 통과시키는 것을 권장한다.
