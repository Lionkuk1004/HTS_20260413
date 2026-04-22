# PHASE_B1_RUNTIME_MODE.md

**조사 범위**: `D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM` 및 상위 `HTS_TEST` 일부(문서 검색). 읽기 전용, 빌드·코드 변경 없음. 기준일: 대화 시점 워크스페이스 스냅샷.

---

## Q1. DeviceMode 실제 사용

| 항목 | 내용 |
|------|------|
| **정의 위치** | `HTS_Console_Manager_Defs.h` — `enum class DeviceMode : uint8_t` (`SENSOR_GATEWAY` … `ETHERNET_BRIDGE`, `MODE_COUNT`). |
| **프리셋·채널과 결합** | `HTS_Device_Profile_Defs.h` — `ChannelConfig` 안에 `DeviceMode device_mode` 및 `uint8_t spread_chips` 필드, 모드별 `constexpr DevicePreset k_device_presets[6]` (문서상 **하나의 펌웨어·설정 전환** 명시). |
| **읽는 코드 (Dispatcher 제외)** | `HTS_Device_Profile.cpp` — 모드 전환 시 `k_device_presets[idx]` 로드, `HTS_Console_Manager::Set_Channel_Config` 로 `ChannelConfig` 전달, `Get_Current_Mode` 등. `HTS_Console_Manager.cpp` — TLV/리포트로 `device_mode`·`spread_chips` 읽기·쓰기. **`.cpp`에서 `DeviceMode` 심볼 매칭은 주로 위 2파일(합 ~13회 수준)**. |
| **Sync 에서 참조** | **N** — `HTS_V400_Dispatcher_Sync.cpp` / `Sync_AMI.cpp` 에 `DeviceMode` / `device_mode` / `spread_chips` 문자열 **없음** (`Select-String`/`rg` 기준). |
| **V400 전체** | `HTS_V400_Dispatcher*.cpp` / `.hpp` 에서 `DeviceMode` / `spread_chips` **미사용** (검색 기준). |
| **실제 동작에 영향 (모뎀 경로)** | Device/Console 측 **런타임 설정·주변기기·채널 파라미터**에는 영향. **V400 Sync 알고리즘 직결은 현재 코드만으로는 확인되지 않음** → **Dispatcher 동작: N(직접), Y(간접 가능성은 상위 계층 연동 시에 한함)**. |

---

## Q2. 매크로와 런타임 관계

| 항목 | 내용 |
|------|------|
| **`HTS_TARGET_AMI` ↔ `DeviceMode::AMI_METER` 연결** | **N** — 동일 표현을 묶는 `#if`/`if` 패턴 **미검출**. `HTS_TARGET_AMI`는 `HTS_V400_Dispatcher_Internal.hpp`, `HTS_V400_Dispatcher_Sync*.cpp` 등 **모뎀 구현 분기**; `AMI_METER`는 **프로파일/콘솔 시나리오** 축. |
| **이중 분기** | **부분(현실적)** — **컴파일 타임**: `/DHTS_TARGET_AMI` 등으로 Sync·내부 정책 분기. **런타임**: `DeviceMode`·`ChannelConfig.spread_chips`는 **콘솔/프로파일**에서 유지되나 **V400에 전달되는 코드 경로가 본 리포지토리 스냅샷에서는 비어 있음**. |
| **관계 요약** | 설계 문서(`HTS_Device_Profile_Defs.h`)는 **단일 펌웨어 + 모드 테이블**을 전제로 하나, **B-CDMA V400 레이어는 별도 매크로 타깃**으로 빌드되는 **하이브리드** 상태. `HTS_KT_DSN_Adapter.cpp` 등에 `force_spread_chips` **콜백**이 있어, 향후 `spread_chips`를 RF/모뎀에 실어 나를 **확장점**은 존재. |

---

## Q3. 사용자 설정 주입

| 항목 | 내용 |
|------|------|
| **API** | `HTS_Device_Profile::Switch_Mode(DeviceMode mode)` (`HTS_Device_Profile.h`), 내부 `Execute_Switch` — 프리셋 로드·주변기기·`console->Set_Channel_Config(...)`. 초기화: `HTS_Device_Profile::Initialize(HTS_Console_Manager*)`. |
| **호출 시점** | 부팅 후 `Initialize` 성공 시 `Impl` 구성; 모드 변경은 **`Switch_Mode` 호출 시**(외부 IPC/콘솔 명령 가정). `HTS_Console_Manager`는 TLV로 `device_mode` 등 갱신. |
| **저장 위치** | **프리셋 테이블**: `k_device_presets` — **`constexpr` ROM(플래시 상수)**. **현재 모드/채널**: `HTS_Device_Profile::Impl` 및 콘솔의 **`ChannelConfig` 등 — RAM**(구조체 필드). OTP/EEEPROM 전용 API는 본 조사 범위의 V400 경로에서 **확인하지 않음**. |

---

## Q4. Sync 런타임 반응

| 항목 | 내용 |
|------|------|
| **Sync.cpp 의 DeviceMode 참조** | **N** (Q1과 동일). |
| **`spread_chips` 동적 사용 (V400)** | **N** — Dispatcher 멤버는 `pay_cps_` 등으로 **1/16/64 페이로드**를 다루나, **`ChannelConfig::spread_chips`를 읽는 코드 없음**. |
| **런타임 전환 (동일 바이너리 내 AMI↔PS-LTE PHY)** | **현 코드만으로는 N** — PHY 차이는 **`HTS_TARGET_AMI` 빌드 플래그**로 고정. **모드 전환 API만으로 V400 분기를 바꾸는 경로는 미구현**으로 판단. |

---

## Q5. 메모리

| 항목 | 값·근거 |
|------|---------|
| **`p0_buf128_I_` / `p0_buf128_Q_`** | 각 `int16_t[192]` → **192×2 = 384 byte** × 2버퍼 = **768 byte** (`HTS_V400_Dispatcher.hpp`). |
| **`p0_carry_*`** | `int16_t[64]` ×2 → **256 byte**. (P0 링 관련 추가 정적 버퍼.) |
| **`HTS_V400_Dispatcher` 총 정적 크기** | 주석 **약 120 KiB (SRAM 상주분)**; `static_assert(sizeof(HTS_V400_Dispatcher) < 128*1024)` (`HTS_V400_Dispatcher.hpp`). **HARQ Q·일부 IR 버퍼는 클래스 밖(CCM union)이라 sizeof에 미포함** — 설계 주석 참조. |
| **2종(AMI+PS-LTE) 동시 탑재 시** | **RAM**: 인스턴스는 통상 **1개**이므로 **버퍼 중복 없음**(동일 `HTS_V400_Dispatcher` 레이아웃). **Flash/ROM**: **Sync 로직을 두 경로 모두 링크**하면 **코드 세그먼트 증가**(대략 Sync TU 크기만큼, 중복 정도는 링커/ dead strip에 따름). **T6 단일 TU**는 `Sync.cpp` **또는** `Sync_AMI.cpp` **하나만** include 하므로 **호스트 시뮼은 중복 링크 없음**. |

---

## Q6. 양산 시나리오 문서

| 항목 | 내용 |
|------|------|
| **README 수준** | `HTS_Device_Profile_Defs.h` 상단: **「하나의 펌웨어로 설정만으로 전환」**, **6종 시나리오**, **constexpr ROM 테이블** — **양산 단일 이미지 + 런타임 모드** 의도가 명시됨. |
| **`DeviceMode` 설계 의도** | `HTS_Console_Manager_Defs.h` — **「하나의 펌웨어로 아래 모든 시나리오를 설정만으로 전환」**, ASIC 3비트 모드 셀렉터 언급. |
| **기타** | `HTS_LIM` 루트 `*.md`에서 「AMI/PS-LTE 사용자 선택」 직접 문구는 **짧은 검색으로는 미확인**; 통합 기준서류(`# HTS B-CDMA 전수검사…`) 등에 **양산** 일반 논의는 존재. |

---

## 종합 판정

- **[ ] A. 단일 펌웨어 + 런타임 선택 구조 존재 (V400까지 완결)**  
  - **프로파일/콘솔 축**에서는 **단일 이미지 + `DeviceMode`** 설계가 **문서·코드에 존재**.  
  - **V400 Sync/PHY**는 **`HTS_TARGET_AMI` 컴파일 분기**로 떨어져 있어 **동일 바이너리에서 AMI vs PS-LTE PHY를 런타임으로 고르는 구조는 현재 미완**.

- **[x] B. 2종 펌웨어 빌드 (현재 매크로 방식) — 사실상 V400 측 현황**  
  - T6: `/DHTS_TARGET_AMI` + `Sync_AMI.cpp` vs baseline `Sync.cpp`.  
  - **SRAM 인스턴스 1개** 유지 가능.

- **[x] C. 하이브리드 (일부 런타임, 일부 컴파일) — 상위 vs V400**  
  - **정비 필요**: `spread_chips`·`DeviceMode`를 **Feed_Chip 이전 계층**에서 V400 정책과 **한 줄로 연결**하거나, **의도적으로 B 유지**하고 프로파일에서 AMI_meter 시 **별도 펌**만 배포.

---

## 권고 (메모리 증가 없이, 요지)

- **[ ] A 방식 가능** — **조건**: Sync·임계를 **`DeviceMode`/`spread_chips`(또는 단일 `PhyProfile` enum)** 기준 **런타임 분기**로 모으고, **단일 Sync TU**로 유지. **RAM**: 기존 `p0_buf128` 등 **그대로 공용**(이미 192칩 링 설계). **Flash**: 분기 증가는 있으나 **이중 전체 TU 복제보다 작을 수 있음**.

- **[x] B 방식 권장 (단기·저위험)** — **현재 검증된 상태**와 일치. AMI_meter 전용 SKU는 AMI 빌드, 재난망 등은 PS-LTE 빌드. **SRAM 예산**: 바이너리당 Dispatcher 1개.

- **[ ] C 정비 후 A** — 장기적으로 **프로파일의 `AMI_METER`(64칩·1200bps·LEA)** 과 **V400 동작**을 **명시적 API**(예: `Set_Phy_Profile` / `Apply_Channel_Config`)로 묶은 뒤 **`HTS_TARGET_AMI` 제거**를 목표로 할 때.

---

## Sync 분리 작업에 대한 근거 (요청 사항 연결)

- **런타임 `DeviceMode`가 Sync를 타지 않는 한**, `Sync_AMI.cpp` vs `Sync.cpp` 분리는 **「빌드 타깃별 PHY」** 문제와 정합.  
- **향후 A로 수렴**할 경우: **파일 분리보다 단일 `Sync.cpp` + 런타임 `phy_mode_`** 가 Flash·유지보수에 유리할 수 있음(지금은 **실측·회귀 분리** 목적이면 **B + TU 분리**가 안전).

---

## 조사 시 사용한 검색 요약

- `DeviceMode` / `spread_chips` / `HTS_TARGET_AMI` / `AMI_METER`: `HTS_LIM` 트리 `*.cpp`/`*.hpp`/`*.h`.  
- `HTS_V400_Dispatcher_Sync.cpp` / `Sync_AMI.cpp`: `DeviceMode`·`spread_chips` **0건**.  
- `sizeof(HTS_V400_Dispatcher)`: `HTS_V400_Dispatcher.hpp` `static_assert` 및 주석.
