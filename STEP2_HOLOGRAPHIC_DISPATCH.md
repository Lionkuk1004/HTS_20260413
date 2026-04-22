# Step 2 Holographic Dispatch 보고

## 사전 상태

- 이전 태그: T_HOLO_L1_L2_L5_UTIL (20e5ff6)
- 브랜치: opt/holographic_sync
- baseline: AMI 14939/18400, PS-LTE 16139/18400
- 미스테이징 처리: CFO/HARQ 등은 대화 전제에 따라 `git checkout --` 로 정리 시도됨. Step 2 커밋에는 아래 5개 파일만 포함 (빌드 산출물·기타 문서 untracked 는 미포함).

## 파일 위치 재확인

- AMI `phase0_scan_`: L65
- PS-LTE `phase0_scan_`: L55
- Internal.hpp `k_w63`: L69~77
- Internal.hpp `fwht_raw`: L173~

## 수정 파일

| 파일 | 변경 | 라인 수 |
|---|---|---|
| HTS_V400_Dispatcher.hpp | API + member + include | +17 |
| HTS_V400_Dispatcher_Sync_AMI.cpp | 분기 + 신규 함수 | +120 |
| HTS_V400_Dispatcher_Sync_PSLTE.cpp | 분기 + 신규 함수 | +120 |
| cursor_t6_build_ami.cmd | HOLO_FLAG + Holographic.cpp TU | +6 |
| cursor_t6_build.cmd | HOLO_FLAG + Holographic.cpp TU (신규 추적) | +24 (파일 전체) |

기존 `phase0_scan_()` 본문 변경: **0 줄** (진입부 `#ifdef` 분기 4줄만 추가).

## 빌드 결과

### 매크로 OFF (`HOLO_FLAG` 빈 문자열, `cmd /c cursor_t6_build*.cmd`)

- AMI: OK, warning 0개 (build 로그 기준)
- PS-LTE: OK, warning 0개

### 매크로 ON + runtime false (기본값, 별도 `cl` 빌드 산출물 `HTS_T6_SIM_Test_*_holo.exe`)

- AMI: OK
- PS-LTE: OK

## 회귀 검증

### 매크로 OFF (필수: bit-exact)

| 빌드 | 기대 | 실측 | 일치 |
|---|---|---|---|
| AMI | 14939/18400 | 14939/18400 | OK |
| PS-LTE | 16139/18400 | 16139/18400 | OK |

### 매크로 ON + runtime false (필수: bit-exact)

| 빌드 | 기대 | 실측 | 일치 |
|---|---|---|---|
| AMI | 14939/18400 | 14939/18400 | OK |
| PS-LTE | 16139/18400 | 16139/18400 | OK |

## 커밋

- 태그: T_HOLO_SYNC_OFF_OK (annotated; `git rev-parse T_HOLO_SYNC_OFF_OK^{commit}` 로 커밋 해시 확인)

## 판정

- [x] 모든 회귀 통과 → Step 3 진행 가능
- [ ] 회귀 실패 → 원인 분석 필요

## 다음 단계

- [ ] Step 3 지시서: `Set_Holographic_Sync(true)` + `test_S5_holographic` 실측

## 참고

- `cursor_t6_build.cmd` 는 기존에 미추적(또는 로컬 전용)이었을 수 있음. PowerShell에서 직접 `.\cursor_t6_build.cmd` 호출 시 인코딩/줄바꿈 이슈가 있으면 `cmd /c "...\cursor_t6_build.cmd"` 로 실행 권장.
- 실패 경로는 지시서 예시의 `memmove` 대신 기존 `phase0_scan_` 과 동일하게 `std::memcpy` 유지.
