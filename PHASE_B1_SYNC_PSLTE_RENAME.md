# Sync.cpp → Sync_PSLTE.cpp rename

## 요약

- `git mv` 로 **`HTS_LIM/HTS_V400_Dispatcher_Sync.cpp` → `HTS_V400_Dispatcher_Sync_PSLTE.cpp`** (이력 보존).
- PS-LTE TU include: **`HTS_TEST/t6_sim/HTS_T6_SIM_Test.cpp`** `#else` 분기 **1줄**을 `Sync_PSLTE.cpp` 로 변경.
- **동작·로직 변경 없음** (파일명·include·프로젝트/CMake/보조 스크립트 경로만).

## 커밋·태그

- **커밋**: 메시지 `rename: Sync.cpp -> Sync_PSLTE.cpp + T6 include 경로 보정` — 해시는 `git log -1 --oneline` 로 확인
- **태그**: `T_FILES_RENAMED` — *Sync.cpp → Sync_PSLTE.cpp 완료. AMI/PS-LTE 양쪽 13839 유지*

## 파일 변경

| 구분 | 내용 |
|------|------|
| **rename** | `HTS_LIM/HTS_V400_Dispatcher_Sync.cpp` → `HTS_V400_Dispatcher_Sync_PSLTE.cpp` |
| **T6 SIM** | `HTS_TEST/t6_sim/HTS_T6_SIM_Test.cpp` — `#include ...Sync_PSLTE.cpp` |
| **VS/CMake** | `HTS_LIM/HTS_LIM.vcxproj`, `HTS_LIM.vcxproj.filters`, `HTS_TEST/CMakeLists.txt`, 검증·Jammer 관련 `*.vcxproj` / `*.filters` |
| **기타 TU include** | `HTS_T6_Waterfall_Harness.cpp`, `HTS_Jammer_STD_UnitTest.cpp`, `HTS_BER_PER_UnitTest.cpp`, `HTS_T6_MC_Harness.cpp` |
| **도구** | `HTS_LIM/_split_v400_dispatcher.mjs`, `stepS_split_regression.ps1`, `stepS_t6_post_split_includes.snippet` |

## 빌드

| 대상 | 결과 | 로그 |
|------|------|------|
| PS-LTE | **성공** | `HTS_TEST/t6_sim/build_pslte_rename.log` |
| AMI | **성공** | `HTS_TEST/t6_sim/build_ami_rename.log` |

## 회귀 (1회 실측)

| 빌드 | 로그 | 정량 합계 |
|------|------|-----------|
| PS-LTE | `pslte_rename_run1.log` | **13839 / 15200 (91.0%), BER 0.08954** |
| AMI | `ami_rename_run1.log` | **13839 / 15200 (91.0%), BER 0.08954** |

- **13839 유지**: **Y / Y**

## 사전 확인 메모 (Step 1)

- **T6 include** (변경 전): L1308–1313 — AMI → `Sync_AMI.cpp`, 그 외 → ~~`Sync.cpp`~~ → **`Sync_PSLTE.cpp`**.
- **파일 크기** (rename 직후 디렉터리): `Sync_PSLTE.cpp` **94333 B**, `Sync_AMI.cpp` **94940 B** (AMI 파일은 상단 undef 블록 포함).

## 발견

- **`HTS_V400_Dispatcher_Sync.cpp` 문자열**은 `Sync_PSLTE.cpp` / `Sync_AMI.cpp` **주석** 및 `pipeline_audit_raw/*` 등 **감사 스냅샷**에 잔존 — **빌드 경로와 무관**.
- **`HTS_LIM/HTS_CFO_Compensator.h`**, **`HTS_LIM/HTS_FEC_HARQ.cpp`** 등 **본 작업과 무관한 로컬 변경**은 **이 커밋에 포함하지 않음**.

## 다음 단계 (지시서 예고)

- **Step 2**: T6 `kChipRate` 매크로 분기 (AMI 200 kcps / PS-LTE 1 Mcps) — `PHASE_B1_T6_CHIP_RATE_PARAM.md` 참고.
