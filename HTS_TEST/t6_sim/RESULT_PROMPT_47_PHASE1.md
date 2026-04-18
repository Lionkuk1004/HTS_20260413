# RESULT — PROMPT 47 Phase 1 (Jammer ch_j1…ch_j6)

**날짜**: 2026-04-18  
**문서**: `HTS_SPEC_001` / `HTS_SPEC_002` **docx는 본 워크스페이스에서 읽지 못함** — 수식·금지사항은 **프롬프트 47 본문**만 반영. docx와 불일치 시 사용자 검토 필요.

## 산출물

| 파일 | 설명 |
|------|------|
| `HTS_Jammer_STD.hpp` | 선언 + `SatStats`, `measure_signal_power`, `derive_seed` |
| `HTS_Jammer_STD.cpp` | `ch_j1`…`ch_j6` 구현 |
| `HTS_Jammer_STD_UnitTest.cpp` | 단위 시험 + Parseval (FFT radix-2) |
| `HTS_Jammer_UnitTest.exe` | 빌드 산출 |
| `t6_phase1_jammer_unit.log` | 단위 시험 로그 |
| `t6_phase1_parseval.log` | 동일 로그 복사 (Parseval 포함) |
| `t6_phase1_regression.log` | 기존 T6 V34 재실행 |

## 구현 요지

- **인터페이스**: 기존 `ch_*` 와 동일하게 `int16_t` I/Q **in-place 가산**, `std::mt19937` (`rand()` 미사용).
- **`measure_signal_power`**: \(\frac{1}{N}\sum(I^2+Q^2)\).
- **`ch_j1`**: \(\sigma^2 = P_s/10^{\mathrm{SNR}/10}\), \(n_I,n_Q\sim\mathcal N(0,\sigma^2/2)\).
- **`ch_j2`**: \(A=\sqrt{P_s 10^{\mathrm{JSR}/10}}\), 위상 \(2\pi f_{\mathrm{off}} n/f_{\mathrm{chip}}+\phi\), \(\phi\sim U[0,2\pi)\).
- **`ch_j3`**: `period_chips` 반올림 정수 주기, `duty`로 ON 길이; 위상은 **OFF에서도 시간 진행**; mode0=CW, mode1=AWGN(ON만 \(A/\sqrt{2}\) 분산).
- **`ch_j4`**: \(P_j=P_s 10^{\mathrm{JSR}/10}\), 가우시안 분산 \(P_j/2\) per branch.
- **`ch_j5`**: \(1/\sqrt{N}\) 스케일 유지; **\(f_{\mathrm{center}}\)** 는 SPEC에 파라미터 없어 **0 Hz** 고정 (docx 확인 후 변경 가능).
- **`ch_j6`**: \(f_{\mathrm{inst}}=f_{\mathrm{start}}+\mathrm{fmod}(r t,\,f_{\mathrm{end}}-f_{\mathrm{start}})\), 위상 누적 \(2\pi f_{\mathrm{inst}}/f_{\mathrm{chip}}\); **smooth turnaround 없음**.

## 단위 시험 (측정값)

실행: `HTS_Jammer_UnitTest.exe` (exit 0)

| 항목 | 기대 | 측정 | 판정 |
|------|------|------|------|
| J1 평균 전력 | 1000 | 1008.84 | OK (±2%) |
| J4 평균 전력 | \(10^5\) | 100244.55 | OK (±2%) |
| J2 FFT 피크 분리 | >25 dB | ~286 dB | OK |
| J2 평균 전력 | \(P_j=10^6\) | ~1.00009e6 | OK (±3%) |
| J3 duty | 0.25 | 0.2500 | OK |
| J5 평균 전력 | \(P_j\approx1.995\times10^6\) | ~1.99534e6 | OK (±3%) |
| J6 평균 전력 | \(10^8\) | ~1.00e8 | OK (±5%) |
| Parseval (J4, N=1024) | rel < 1e-3 | ~1.38e-14 | OK |

## Saturation

- 기본 단위 시험: 포인터 `nullptr` — 별도 수치 없음. 필요 시 `SatStats` 전달 후 `sat_rate_pct()` 로그 추가 가능.

## 회귀 (기존 T6 미변경)

- `HTS_T6_SIM_Test.cpp` **비수정** — 기존 `ch_awgn` / `ch_barrage` / `ch_cw` 유지.
- `HTS_T6_SIM_Test_V34_FIXA3.exe` 재실행: **10927 / 11200** (로그 `t6_phase1_regression.log`).

## 빌드 명령 (참고)

```bat
call "%ProgramFiles%\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat"
cd /d D:\HTS_ARM11_Firmware\HTS_LIM\HTS_TEST\t6_sim
cl /nologo /O2 /std:c++17 /EHsc /MD /W3 /FeHTS_Jammer_UnitTest.exe HTS_Jammer_STD_UnitTest.cpp HTS_Jammer_STD.cpp /link /nologo
```

## 모호·후속

1. **SPEC_002 docx 미검증** — §7 pulse ON 길이 정수화, §9 `f_center`, §10 chirp `f(t)` 정의가 docx와 다르면 수정 필요.
2. **boost::math**: Phase 1 미사용; Phase 2에서 요구 시 링크 추가.
3. **단위 시험**: J6은 “순간 주파수 선형” 미세 검증은 생략(평균 전력만).

## 금지 조항 대응

- `rand()` 미사용, `1/\sqrt{N}\)` 유지, 위상 RNG, `P_s` 기반 \(A\) 정의, 회귀 T6 미변경.
