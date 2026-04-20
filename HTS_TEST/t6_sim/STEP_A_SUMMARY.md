# Step A — Dispatcher 로컬 복사본 구축 결과

## 1. 회귀 기준
- Gate A-1 (시작, 지시서 순서): 별도 로그 미저장 — 변경 전 기준은 이전 워크스페이스와 동일하게 **10927 / 11200** 가정.
- Gate A-5 (빌드·로컬 Dispatcher 적용 후 T6): **10927 / 11200** (`stepA5_t6.log`)
- Gate A-7 (최종 T6): **10927 / 11200** (`stepA7_t6_final.log`)

## 2. 빌드 검증
- HTS_T6_SIM_Test.exe 생성: OK
- HTS_Harq_Matrix_Test.exe 생성: OK
- 중복 심볼 에러: 없음
- 컴파일 경고: 0 (stepA4_build.log 기준 `warning C` 매칭 없음)

## 3. HARQ Matrix 독립 동작 (Clean smoke)
| fec_path | mode | crc/100 | harq_k_avg |
|---|---|---|---|
| chase | DATA | 100/100 | 1.000000 |
| chase | VOICE | 100/100 | 1.000000 |
| ir_harq | DATA | 100/100 | 1.000000 |
| ir_harq | VOICE | 100/100 | 1.000000 |

(출처: `HARQ_Matrix_Results.csv` Clean 행, `stepA6_smoke.log` 상단 테이블과 일치)

## 4. 파일 목록
- 신규: HTS_V400_Dispatcher_Local.hpp (31073 바이트)
- 신규: HTS_V400_Dispatcher_Local.cpp (177694 바이트)
- 수정: HTS_Harq_Matrix_Test.cpp
- 수정: build.bat
- 무수정: HTS_T6_SIM_Test.cpp, HTS_LIM/*

## 5. 네임스페이스 구조
- 원본: `ProtectedEngine` (HTS_LIM)
- 로컬: `ProtectedEngineLocal` (t6_sim)
- 로컬 `.cpp`: `FEC_HARQ`, `SecureMemory`, `HTS_Holo_LPI` 등 공용 심볼은 `using ProtectedEngine::…` 로 원본과 동일 구현 유지
- HARQ Matrix: `using ProtectedEngineLocal::HTS_V400_Dispatcher` (및 `DecodedPacket`, `PayloadMode`); `FEC_HARQ`·`WRC` 등은 `ProtectedEngine` 유지
