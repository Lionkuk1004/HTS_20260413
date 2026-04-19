# Cursor 터미널 — Phase 2 빌드 환경 (VS IDE 없이 `cl`)

저장소에는 **사용자 전역 `settings.json`을 자동으로 수정하지 않습니다.** 아래를 Cursor에 그대로 적용하세요.

## 1. 통합 터미널 프로필 (권장)

1. `Ctrl+Shift+P` → `Preferences: Open User Settings (JSON)`
2. Windows 프로필 예시 (에디션에 맞게 `vcvars64.bat` 경로만 수정):

```json
{
  "terminal.integrated.profiles.windows": {
    "Developer Command Prompt": {
      "path": "C:\\Windows\\System32\\cmd.exe",
      "args": [
        "/k",
        "C:\\Program Files\\Microsoft Visual Studio\\2022\\Community\\VC\\Auxiliary\\Build\\vcvars64.bat"
      ],
      "icon": "terminal-cmd"
    }
  },
  "terminal.integrated.defaultProfile.windows": "Developer Command Prompt"
}
```

| 에디션 | `vcvars64.bat` 위치 |
|--------|---------------------|
| Community | `...\2022\Community\VC\Auxiliary\Build\` |
| Professional | `...\2022\Professional\...` |
| Enterprise | `...\2022\Enterprise\...` |
| Build Tools | `...\2022\BuildTools\...` |

경로 확인:

```cmd
dir /s /b "C:\Program Files\Microsoft Visual Studio\2022\*vcvars64.bat"
dir /s /b "C:\Program Files (x86)\Microsoft Visual Studio\2022\*vcvars64.bat"
```

저장 후 **Cursor 완전 종료 → 재실행**. 새 터미널에서 `cl` 입력 시 컴파일러 사용법이 출력되면 성공입니다.

## 2. Phase 2 빌드/실행 (이 저장소)

- 소스 위치: `HTS_TEST\t6_sim\` (프롬프트 예시의 `HTS_Jammer_STD` 폴더가 아님)
- 빌드는 **단일 TU** `HTS_BER_PER_UnitTest.cpp` 한 개만 `cl`에 넘기면 됨 (하단 `#include`로 V400·잼머·Measure 링크).
- **`HTS_Session_Derive_Stub.cpp` 필수**: `Walsh_Row_Permuter` → `Session_Gateway::Derive_Session_Material` 링크용. `HTS_T6_SIM_Test.vcxproj`와 동일 역할. 누락 시 **LNK2019**.
- 현재 Clopper-Pearson 구현은 **Boost 불필요**. 나중에 `boost::math`로 바꿀 경우에만:

```cmd
set BOOST_ROOT=D:\boost\boost_1_85_0
```

## 3. 한 번에 빌드 + 로그

`HTS_TEST\t6_sim`에서:

```cmd
build_and_test_phase2.bat
```

생성/갱신:

- `t6_phase2_cp_unittest.log`
- `t6_phase2_ber_per.log` / `t6_phase2_theory_compare.log` (동일 내용 복사, 프롬프트 파일명 규칙 충족)
- `t6_phase2_regression.log` — 같은 폴더에 `HTS_T6_SIM_Test_V34_FIXA3.exe`가 있을 때만 실제 회귀 실행

## 4. MinGW 대안

MSVC가 없으면 사용자 가이드 §8대로 MSYS2/MinGW `g++`를 PATH에 두고, 동일 소스에 맞는 `g++` 명령을 별도 스크립트로 추가하면 됩니다 (현재 배치는 MSVC `cl` 전용).
