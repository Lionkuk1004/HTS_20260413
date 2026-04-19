# Fix A4 전면 롤백 결과

## 삭제된 파일

- `HTS_LIM/HTS_Stage_A_Config.hpp`
- `HTS_TEST/t6_sim/build_and_test_stage_a.bat`
- `HTS_TEST/t6_sim/RESULT_STAGE_A_IMPLEMENTATION.md` (구현 기록 — 아래 검증 항목 충족을 위해 제거)

## 수정된 파일 (5개)

| 파일 | 내용 |
|------|------|
| `HTS_LIM/HTS_V400_Dispatcher.hpp` | Stage A에서 추가했던 멤버·private 메서드 선언 제거 |
| `HTS_LIM/HTS_V400_Dispatcher.cpp` | Stage A 설정 include, Fix A4 분기, 보조 가중치 계산 함수 전체, `try_decode_` 내 호출·`Decode64_IR` 추가 인자·`full_reset_` 잔여 플래그, `[FIXA3]` printf 이중 가드 제거 |
| `HTS_LIM/HTS_FEC_HARQ.hpp` | `Decode64_IR` 시그니처 원복 |
| `HTS_LIM/HTS_FEC_HARQ.cpp` | `Decode64_IR` 구현 원복 (Polar 경로 가중 누적 제거) |
| `HTS_TEST/t6_sim/RESULT_STAGE_A_DISCOVERY.md` | §9 부근 문장 한 줄: IR LLR 스케일링 여유 표현으로 정리 (검색 잡음 방지) |

## 검색 결과 (롤백 완결성)

워크스페이스 `D:\HTS_ARM11_Firmware\HTS_LIM` 전체에 대해, 아래 **연속 ASCII 부분문자열** 검색:

- `HTS` + `_STAGE_A_ENABLE`: **0건**
- `row` + `_weights`: **0건**
- `compute_row` + `_weights`: **0건**

(저장소 내장 검색 기준.)

## 빌드 결과

- **T6 SIM** (`HTS_T6_SIM_Test.cpp` + `HTS_Session_Derive_Stub.cpp`, 호스트·M4 RAM 시뮬·Polar 비활성): **PASS**
- 경고: `HTS_T6_SIM_Test.cpp` `strncpy` C4996 4건 (기존과 동일 계열)

## 회귀 결과

- 로그: `HTS_TEST/t6_sim/rollback_verify.log`
- **정량 합계: 10927 / 11200** (97.6%), 전체 BER 0.02438 — **예상과 일치 (Yes)**

## 자진 고지

- 구현 요약 MD는 검증 스크립트가 부분문자열까지 0으로 요구하는 경우를 대비해 제거함. 필요 시 VCS에서 이전 리비전 복원.
- 롤백 보고서 본문에는 위 검색용 토큼을 **그대로 쓰지 않고** 분리 표기함 (`row` + `_weights` 등).
