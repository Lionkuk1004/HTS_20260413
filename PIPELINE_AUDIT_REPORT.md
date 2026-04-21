# Pipeline Audit Report (CFO 4-3 진입 전)

조사 범위: `D:\HTS_ARM11_Firmware` (하위 `HTS_LIM`, `CORE` 존재 시).  
원시 출력 파일 디렉터리: `HTS_LIM/pipeline_audit_raw/`  
본 문서는 지시서 요구에 따라 **명령 출력·grep 결과·파일 목록을 그대로** 옮겼으며, 해석·추정 문장은 넣지 않음.

---

## A. 고아 소스 파일

### A-1: 사용한 PowerShell 명령 (cmd `dir /s /b` 대응)

```text
Get-ChildItem -Path "D:\HTS_ARM11_Firmware\HTS_LIM" -Recurse -Include *.cpp,*.c,*.hpp,*.h -File → audit_all_sources_hts_lim_tree.txt
Get-ChildItem -Path "D:\HTS_ARM11_Firmware\HTS_LIM\HTS_TEST" -Recurse -Include *.cpp,*.c,*.hpp,*.h -File → audit_all_sources_test.txt
Get-ChildItem -Path "D:\HTS_ARM11_Firmware\CORE" -Recurse -Include *.cpp,*.c,*.hpp,*.h -File → audit_all_sources_core.txt
```

### A-1: 줄 수 (파일 경로 한 줄 = 1)

```text
audit_all_sources_hts_lim_tree.txt lines: 486
audit_all_sources_test.txt lines: 123
audit_all_sources_core.txt lines: 0
```

### A-1: `HTS_LIM\HTS_LIM` 이하 `.cpp` 전용 목록 줄 수

```text
audit_cpp_hts_lim_hts_lim_only.txt lines: 115
```

### A-1: `HTS_TEST` 이하 `.cpp` 전용 목록 줄 수

```text
audit_cpp_test_paths.txt lines: 74
```

### A-1: `HTS_LIM\HTS_LIM` 이하 `.hpp` 경로 줄 수

```text
audit_hpp_mainline_paths.txt lines: 63
```

### A-2: 빌드 참조 텍스트 생성

```text
Get-ChildItem -Recurse -Include *.bat,*.cmd,*.ps1,*.vcxproj,*.vcxproj.filters → audit_build_refs_full.txt (bytes: 546010)
별도: audit_bat_sources.txt (각 .bat에서 Select-String '\.cpp')
별도: audit_vcxproj_sources.txt (각 .vcxproj에서 Select-String 'ClCompile|Include')
```

### A-2: `.cpp` 문자열 출현 횟수 (중복·다중 매치 포함, 라인 단위 아님)

파일: `pipeline_audit_raw/audit_cpp_ref_line_counts.txt`

```text
127 206 690
```

순서: `audit_bat_sources.txt` 매치수, `audit_vcxproj_sources.txt` 매치수, `audit_build_refs_full.txt` 매치수.

### A-2: Makefile / CMakeLists

파일: `pipeline_audit_raw/audit_make_files.txt` (원문 참조).

### A-3: 고아 후보 (basename이 `audit_build_refs_full.txt` + `audit_vcxproj_sources.txt` 텍스트에 **부분 문자열로도 미출현**)

스크립트: `pipeline_audit_raw/run_orphans_v2.py`  
입력 basename 집합: `audit_cpp_hts_lim_hts_lim_only.txt` ∪ `audit_cpp_test_paths.txt`  
참조 문자열: `audit_build_refs_full.txt` + `audit_vcxproj_sources.txt`

파일: `pipeline_audit_raw/audit_orphans_v2.txt`

```text
=== ORPHAN CANDIDATES (20 files) ===
  HTS_AMI_Barrage30_Test_V2.cpp
  HTS_AMI_CommSpec_Barrage30_Matrix_Test.cpp
  HTS_Extended_Endurance_Test.cpp
  HTS_FEC_Layer_Test.cpp
  HTS_Fractal_Channel_Compare.cpp
  HTS_LDPC.cpp
  HTS_LPI_Measurement.cpp
  HTS_Layer10111213_Sequential_Audit_PC_Test.cpp
  HTS_Layer1417_Host_IPC_Tick_API_Stub.cpp
  HTS_Layer14_17_Sequential_Audit_PC_Test.cpp
  HTS_Layer345_Sequential_Audit_PC_Test.cpp
  HTS_Layer6789_Sequential_Audit_PC_Test.cpp
  HTS_Pipeline_Sequential_Audit_PC_Test.cpp
  HTS_Pipeline_V2_Compare.cpp
  HTS_RS_FEC_Sequential_Audit_PC_Test.cpp
  HTS_RS_GF16.cpp
  HTS_TPC_LPI_Test.cpp
  HTS_V400_SIC_PC_Test.cpp
  toolchain_smoke_main.cpp
  종합재밍_종합_테스트.cpp
```

### A-3: 고아 후보 목록과 `HTS_TEST/CMakeLists.txt` 교차 (원시 grep)

`HTS_RS_GF16.cpp` 는 `audit_orphans_v2.txt` 에 포함되어 있음.

`HTS_TEST/CMakeLists.txt` 내 문자열:

```
80:  "${CMAKE_CURRENT_SOURCE_DIR}/HTS_RS_GF16.cpp"
153:  "${CMAKE_CURRENT_SOURCE_DIR}/HTS_RS_GF16.cpp"
```

### A-3: `.bak` 경로 목록 (원시)

파일: `pipeline_audit_raw/audit_bak_files.txt`

```text
D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher.cpp.bak
D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher.hpp.bak
D:\HTS_ARM11_Firmware\HTS_LIM\HTS_TEST\t6_sim\t6_phase2_regression.log.bak
D:\HTS_ARM11_Firmware\HTS_LIM\HTS_TEST\t6_sim\_backup_before_step_c0\build.bat.bak
D:\HTS_ARM11_Firmware\HTS_LIM\HTS_TEST\t6_sim\_backup_before_step_c0\HTS_V400_Dispatcher.cpp.bak
```

---

## B. RX Matched Filter 이중화 매핑

### B-1: `dir /s /b HTS_Rx_Matched_Filter*` 대응

파일: `pipeline_audit_raw/stepB_mf_dir.txt` (전체 경로 원문).

### B-2: 파일 크기 (바이트)

파일: `pipeline_audit_raw/stepB_mf_sizes.txt`

```text
D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_Rx_Matched_Filter.h 4948
D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_Rx_Matched_Filter.cpp 9712
```

### B-2: `HTS_Rx_Matched_Filter.h` 전문 (공개 API 원문)

원문 파일: `HTS_LIM/HTS_LIM/HTS_Rx_Matched_Filter.h` (바이트 크기: 4948, `stepB_mf_sizes.txt` 참조).

```cpp
// =========================================================================
// HTS_Rx_Matched_Filter.h
// B-CDMA 교차 상관 정합 필터 — 공개 인터페이스
// Target: STM32F407 (Cortex-M4, 168MHz, SRAM 192KB)
//
// ─────────────────────────────────────────────────────────────────────────
//  외주 업체 통합 가이드
// ─────────────────────────────────────────────────────────────────────────
//
//  이 모듈은 B-CDMA 수신단의 교차 상관(Cross-Correlation) 정합 필터입니다.
//
//  [정합 필터 원리]
//   out[i] = Σ(j=0..N-1) ((rx[i+j]×ref[j] + Q16_ROUND_BIAS) >> 16)  (탭마다 즉시 Q16 다운스케일 후 누적)
//
//  [메모리 요구량]
//   sizeof(HTS_Rx_Matched_Filter) ≈ IMPL_BUF_SIZE + impl_valid_ + op_busy_ + padding
//   Impl: HTS_Sys_Config(32B) + int32_t[64](256B) + ref_len(4B) ≈ 296B → IMPL_BUF_SIZE = 320B
//   기준 시퀀스 데이터: int32_t[64] 정적 배열 (256B, 힙 0)
//
//  [동시성]
//   Set_Reference_Sequence / Apply_Filter 는 atomic_flag(op_busy_) 로 상호 배제.
//   소멸자: op_busy_ 짧은 시도 후 미획득이면 AIRCR 시스템 리셋(강제 파쇄·~Impl 경로 없음, UAF 방지). PC 시뮬은 abort.
//   ISR 에서 Apply_Filter 호출 시 메인과 스핀 경합 가능 — 호출 컨텍스트 설계 필수.
//
//  [보안 설계]
//   기준 시퀀스: Set 교체 시 기존 소거 + 소멸자 소거
//   impl_buf_: 소멸자에서 SecWipe — 전체 이중 소거
//   복사/이동: = delete (PN 시퀀스 복제 방지)
//
// ─────────────────────────────────────────────────────────────────────────
#pragma once

#include <cstdint>
#include <cstddef>
#include <atomic>

namespace ProtectedEngine {

    // 전방 선언 (HTS_Dynamic_Config.h include 제거)
    enum class HTS_Sys_Tier : uint8_t;

    class HTS_Rx_Matched_Filter {
    public:
        /// @brief 교차 상관 정합 필터 생성
        /// @param tier  시스템 체급 (향후 필터 크기 자동 조정 예약)
        /// @note  초기화 실패(OOM) 시 impl_valid_=false → 모든 함수 false
        explicit HTS_Rx_Matched_Filter(HTS_Sys_Tier tier) noexcept;

        /// @brief 소멸자 — Impl 소멸자 호출 후 impl_buf_ SecWipe
        ~HTS_Rx_Matched_Filter() noexcept;

        /// 기준 시퀀스 = 보안 자산 → 복사 경로 차단
        HTS_Rx_Matched_Filter(const HTS_Rx_Matched_Filter&) = delete;
        HTS_Rx_Matched_Filter& operator=(const HTS_Rx_Matched_Filter&) = delete;
        HTS_Rx_Matched_Filter(HTS_Rx_Matched_Filter&&) = delete;
        HTS_Rx_Matched_Filter& operator=(HTS_Rx_Matched_Filter&&) = delete;

        /// @brief 기준 시퀀스(대역 확산 코드) 설정
        /// @param seq_data  Q16 기준 시퀀스 배열 (nullptr 불가)
        /// @param size      요소 수 (0 불가)
        /// @return true=성공, false=nullptr/0/OOM
        /// @post   기존 시퀀스는 교체 전 보안 소거
        [[nodiscard]] bool Set_Reference_Sequence(
            const int32_t* seq_data, size_t size) noexcept;

        /// @brief Q16 교차 상관 연산
        /// @param rx_q16_data      수신 Q16 데이터 (nullptr 불가)
        /// @param rx_size          수신 데이터 요소 수 (>= ref_len 필수)
        /// @param out_correlation  출력 버퍼 (호출자 할당, 최소 rx_size - ref_len + 1)
        /// @return true=성공, false=파라미터 오류 또는 기준 시퀀스 미설정
        [[nodiscard]] bool Apply_Filter(
            const int32_t* __restrict rx_q16_data, size_t rx_size,
            int32_t* __restrict out_correlation) noexcept;

    private:
        // ── Pimpl In-Place Storage (zero-heap) ───────────────────
        // Impl = HTS_Sys_Config(≈32B) + int32_t[64](256B) + ref_len(4B) ≈ 296B
        // 정적 impl_buf_, 힙 0
        static constexpr size_t IMPL_BUF_SIZE = 320u;
        static constexpr size_t IMPL_BUF_ALIGN = 8u;

        struct Impl;  ///< 기준 시퀀스 + 체급 설정 은닉

        alignas(IMPL_BUF_ALIGN) uint8_t impl_buf_[IMPL_BUF_SIZE];
        std::atomic<bool> impl_valid_{ false };
        /// reference_sequence / ref_len 갱신·교차 상관 연산 상호 배제
        std::atomic_flag op_busy_ = ATOMIC_FLAG_INIT;

        Impl* get_impl() noexcept;
        const Impl* get_impl() const noexcept;
    };

} // namespace ProtectedEngine
```

### B-2: `HTS_Rx_Matched_Filter.cpp` 내 `HTS_Rx_Matched_Filter::` 정의 줄 (grep)

```
125:    HTS_Rx_Matched_Filter::HTS_Rx_Matched_Filter(
136:    HTS_Rx_Matched_Filter::~HTS_Rx_Matched_Filter() noexcept {
158:    bool HTS_Rx_Matched_Filter::Set_Reference_Sequence(
194:    bool HTS_Rx_Matched_Filter::Apply_Filter(
```

### B-3: `HTS_Rx_Matched_Filter` 문자열이 한 번이라도 등장하는 파일 경로 (List)

파일: `pipeline_audit_raw/stepB_mf_independent_users_paths.txt` (전체).

### B-3: `#include` 가 아닌 줄에서의 등장 (ripgrep, 일부)

명령: `rg -n "HTS_Rx_Matched_Filter|HTS_RX_MATCHED_FILTER" --glob "*.{cpp,hpp,h}"` 후 `include` 제외는 별도 필터 미적용·원시는 `stepB_mf_dir.txt` 및 소스 파일 직접 참조.

### B-4: `HTS_V400_Dispatcher_Sync.cpp` 내 `mf_*` 줄 (ripgrep)

```
1183:                if (mf_feed_chip_(chip_I, chip_Q)) {
1473:            mf_reset_();
1566:        mf_reset_();
1923:    static inline void mf_prng_init_(
1931:    static inline uint32_t mf_prng_next_(MF_PRNG_State& st) noexcept {
1943:    static inline int mf_popcount_u32_(uint32_t x) noexcept {
1951:    static inline int8_t mf_walsh_bit_(uint32_t row, uint32_t col) noexcept {
1957:    static void mf_shuffle_u16_perm_(
1976:        mf_reset_();
1988:void HTS_V400_Dispatcher::mf_reset_() noexcept {
1996:uint32_t HTS_V400_Dispatcher::mf_envelope_combine_(
2016:void HTS_V400_Dispatcher::mf_generate_reference_(uint32_t rx_seq) noexcept {
2079:bool HTS_V400_Dispatcher::mf_feed_chip_(int16_t rx_I, int16_t rx_Q) noexcept {
2154:int32_t HTS_V400_Dispatcher::mf_pte_refine_(int peak_idx) const noexcept {
2074:    (void)mf_engine_.Set_Reference_Sequence(mf_ref_q16_, kN);
2109:    const bool ok_I = mf_engine_.Apply_Filter(
2114:    const bool ok_Q = mf_engine_.Apply_Filter(
```

### B-4: `HTS_V400_Dispatcher.hpp` 내 MF 관련 줄 (ripgrep)

```
74:#if defined(HTS_SYNC_USE_MATCHED_FILTER)
75:#include "HTS_Rx_Matched_Filter.h"
299:        void Set_Matched_Filter_Sync(bool enable) noexcept;
300:        [[nodiscard]] bool Get_Matched_Filter_Sync() const noexcept;
302:        [[nodiscard]] int32_t Get_Estimated_Timing_Offset_Q16() const noexcept;
551:#if defined(HTS_SYNC_USE_MATCHED_FILTER)
560:        HTS_Rx_Matched_Filter mf_engine_;
561:        alignas(8) int32_t mf_ref_q16_[kMF_RefChips] = {};
...
```

### B-4: `HTS_SYNC_USE_MATCHED_FILTER` 문자열 출현 (ripgrep, 일부 경로)

```
HTS_LIM/HTS_V400_Dispatcher_Sync.cpp
HTS_TEST/t6_sim/HTS_V400_Dispatcher_Local.cpp
HTS_TEST/t6_sim/HTS_FEC_Layer_Real_Sync_Test.cpp
HTS_LIM/HTS_V400_Dispatcher_Core.cpp
HTS_TEST/t6_sim/HTS_V400_Dispatcher_Local.hpp
HTS_TEST/t6_sim/HTS_FEC_Layer_LPI_Isolation_Test.cpp
HTS_TEST/t6_sim/build_layer_lpi_isolation.bat
HTS_LIM/HTS_V400_Dispatcher.hpp
```

### B-5: `HTS_TEST/t6_sim/HTS_V400_Dispatcher_Local.cpp` 줄 21 (원문)

```21:21:d:/HTS_ARM11_Firmware/HTS_LIM/HTS_TEST/t6_sim/HTS_V400_Dispatcher_Local.cpp
#include "../../HTS_LIM/HTS_Rx_Matched_Filter.cpp"
```

### B-5: `HTS_LIM.vcxproj` ClCompile 줄 (ripgrep)

```
276:    <ClCompile Include="HTS_Rx_Matched_Filter.cpp" />
310:    <ClCompile Include="HTS_V400_Dispatcher_Core.cpp" Condition="'$(ExcludeV400DispatcherFromStaticLib)'!='true'" />
311:    <ClCompile Include="HTS_V400_Dispatcher_Sync.cpp" Condition="'$(ExcludeV400DispatcherFromStaticLib)'!='true'" />
```

---

## C. Dispatcher 내부 MF 보안 연결

### C-1: `HTS_V400_Dispatcher_Sync.cpp` 내 `SecureMemory::secureWipe` 줄 번호

```
1864:                                SecureMemory::secureWipe(
1876:                            SecureMemory::secureWipe(
1885:                                SecureMemory::secureWipe(
```

### C-1: `HTS_V400_Dispatcher_Sync.cpp` 내 `std::memset` 줄 번호 (mf_reset_ 구간)

```
1989:    std::memset(mf_recv_I_q16_, 0, sizeof(mf_recv_I_q16_));
1990:    std::memset(mf_recv_Q_q16_, 0, sizeof(mf_recv_Q_q16_));
```

### C-1: `HTS_V400_Dispatcher_Core.cpp` 내 `mf_reset_` / `full_reset_` 줄 번호

```
110:    full_reset_();
333:void HTS_V400_Dispatcher::full_reset_() noexcept {
405:    mf_reset_();
497:    mf_reset_();
```

### C-2: `HTS_Rx_Matched_Filter.cpp` 내 `SecureMemory::secureWipe` 줄 번호

```
94:            SecureMemory::secureWipe(
129:        SecureMemory::secureWipe(static_cast<void*>(impl_buf_), sizeof(impl_buf_));
151:        SecureMemory::secureWipe(static_cast<void*>(impl_buf_), sizeof(impl_buf_));
174:        SecureMemory::secureWipe(
180:        SecureMemory::secureWipe(static_cast<void*>(shadow), sizeof(shadow));
```

---

## D. 빌드 시스템 인벤토리

### D-1

파일: `pipeline_audit_raw/audit_bat_targets.txt`  
파일: `pipeline_audit_raw/audit_bat_sources.txt`  
파일: `pipeline_audit_raw/audit_build_refs_full.txt`

### D-2

파일: `pipeline_audit_raw/audit_vcxproj_sources.txt`  
파일: `pipeline_audit_raw/audit_vcxproj_targets.txt`

### D-3

```text
dir /s /b *.cproject *.project *.ioc *.uvprojx *.ewp *.ewd → audit_arm_project_files.txt (0 lines, 빈 파일 생성됨)
```

---

## E. 헤더 참조 고아

스크립트: `pipeline_audit_raw/run_orphan_headers.py`  
스캔 루트: `D:/HTS_ARM11_Firmware/HTS_LIM` 내 `HTS_LIM`, `HTS_TEST`, `HTS_Jammer_STD`  
매칭: `#include` 줄에 헤더 **basename** 정확 일치 regex.

표준 출력:

```text
247 18
```

파일: `pipeline_audit_raw/audit_orphan_headers.txt`

```text
=== ORPHAN HEADER PATHS (18) ===
D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\arm_arch.h
D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_Data_Loader.h
D:\HTS_ARM11_Firmware\HTS_LIM\arm_firmware\include\hts_gcc_arm_force_include.h
D:\HTS_ARM11_Firmware\HTS_LIM\arm_firmware\include\HTS_HAL.h
D:\HTS_ARM11_Firmware\HTS_LIM\arm_firmware\include\HTS_Interface_Manager.h
D:\HTS_ARM11_Firmware\HTS_LIM\arm_firmware\include\HTS_Interface_StubLayout.h
D:\HTS_ARM11_Firmware\HTS_LIM\arm_firmware\include\HTS_LockFree_Ring.h
D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_MAC_RateController.h
D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_PHY_Config.h
D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_Secure_Memory_Manager.hpp
D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_Sensor_ADC_Guard.h
D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_Server_Stress_Test.h
D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_Storage_Adapter.hpp
D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_TimeSpace_Guard.h
D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_Types.h
D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_Universal_Adapter.hpp
D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_Universal_API.hpp
D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\lea_key.h
```

---

## F. 테스트 exe ↔ Dispatcher (`HTS_TEST/t6_sim/*.cpp` ripgrep)

명령: `rg "include.*Dispatcher" HTS_TEST/t6_sim -g*.cpp`

```
HTS_TEST\t6_sim\HTS_V400_Dispatcher_Local.cpp
11:#include "HTS_V400_Dispatcher_Local.hpp"

HTS_TEST\t6_sim\HTS_FEC_Layer_Real_Sync_Test.cpp
20:#include "HTS_V400_Dispatcher_Local.hpp"
1553:#include "HTS_V400_Dispatcher_Local.cpp"

HTS_TEST\t6_sim\HTS_T6_SIM_Test.cpp
26:#include "HTS_V400_Dispatcher.hpp"
1307:#include "../../HTS_LIM/HTS_V400_Dispatcher_Core.cpp"
1308:#include "../../HTS_LIM/HTS_V400_Dispatcher_Sync.cpp"
1309:#include "../../HTS_LIM/HTS_V400_Dispatcher_Payload.cpp"
1310:#include "../../HTS_LIM/HTS_V400_Dispatcher_TX.cpp"
1311:#include "../../HTS_LIM/HTS_V400_Dispatcher_Decode.cpp"

HTS_TEST\t6_sim\HTS_BER_PER_UnitTest.cpp
188:#include "../../HTS_LIM/HTS_V400_Dispatcher_Core.cpp"
189:#include "../../HTS_LIM/HTS_V400_Dispatcher_Sync.cpp"
190:#include "../../HTS_LIM/HTS_V400_Dispatcher_Payload.cpp"
191:#include "../../HTS_LIM/HTS_V400_Dispatcher_TX.cpp"
192:#include "../../HTS_LIM/HTS_V400_Dispatcher_Decode.cpp"

HTS_TEST\t6_sim\HTS_Jammer_STD_UnitTest.cpp
482:#include "../../HTS_LIM/HTS_V400_Dispatcher_Core.cpp"
483:#include "../../HTS_LIM/HTS_V400_Dispatcher_Sync.cpp"
484:#include "../../HTS_LIM/HTS_V400_Dispatcher_Payload.cpp"
485:#include "../../HTS_LIM/HTS_V400_Dispatcher_TX.cpp"
486:#include "../../HTS_LIM/HTS_V400_Dispatcher_Decode.cpp"

HTS_TEST\t6_sim\HTS_FEC_Layer_LPI_Isolation_Test.cpp
18:#include "HTS_V400_Dispatcher_Local.hpp"
1155:#include "HTS_V400_Dispatcher_Local.cpp"

HTS_TEST\t6_sim\HTS_FEC_Layer_Partial_Test.cpp
18:#include "HTS_V400_Dispatcher_Local.hpp"
1102:#include "HTS_V400_Dispatcher_Local.cpp"

HTS_TEST\t6_sim\HTS_Block_Message_Test.cpp
14:#include "HTS_V400_Dispatcher_Local.hpp"
520:#include "HTS_V400_Dispatcher_Local.cpp"

HTS_TEST\t6_sim\HTS_Harq_Matrix_Test.cpp
14:#include "HTS_V400_Dispatcher_Local.hpp"

HTS_TEST\t6_sim\_backup_before_step_b\HTS_V400_Dispatcher_Local.cpp
11:#include "HTS_V400_Dispatcher_Local.hpp"

HTS_TEST\t6_sim\_backup_before_step_b\HTS_Harq_Matrix_Test.cpp
14:#include "HTS_V400_Dispatcher_Local.hpp"

HTS_TEST\t6_sim\_backup_before_step_a\HTS_Harq_Matrix_Test.cpp
12:#include "HTS_V400_Dispatcher.hpp"
511:#include "../../HTS_LIM/HTS_V400_Dispatcher.cpp"

HTS_TEST\t6_sim\_backup_before_kamp\HTS_Harq_Matrix_Test.cpp
11:#include "HTS_V400_Dispatcher.hpp"
470:#include "../../HTS_LIM/HTS_V400_Dispatcher.cpp"

HTS_TEST\t6_sim\_backup_before_diag\HTS_Harq_Matrix_Test.cpp
11:#include "HTS_V400_Dispatcher.hpp"
470:#include "../../HTS_LIM/HTS_V400_Dispatcher.cpp"

HTS_TEST\t6_sim\_backup_before_step_c1\HTS_T6_SIM_Test.cpp
26:#include "HTS_V400_Dispatcher.hpp"
1307:#include "../../HTS_LIM/HTS_V400_Dispatcher.cpp"

HTS_TEST\t6_sim\HTS_BER_PER_Measure.cpp
3:#include "HTS_V400_Dispatcher.hpp"
```

`HTS_Harq_Matrix_Test.cpp` 파일 말미 `#include` 블록 (508행 이후, Dispatcher 문자열 없음):

```508:516:d:/HTS_ARM11_Firmware/HTS_LIM/HTS_TEST/t6_sim/HTS_Harq_Matrix_Test.cpp
#include "../../HTS_LIM/HTS_Secure_Memory.cpp"
#include "../../HTS_LIM/HTS_Polar_Codec.cpp"
#include "../../HTS_LIM/HTS_FEC_HARQ.cpp"
#include "../../HTS_LIM/HTS_Walsh_Row_Converter.cpp"
#include "../../HTS_LIM/HTS_Holo_LPI.cpp"
#include "../../HTS_LIM/HTS_Walsh_Row_Permuter.cpp"
#include "HTS_Session_Derive_Stub.cpp"
#include "HTS_Jammer_STD.cpp"
```

---

## G. 즉시 보고 이상 항목 (지시서 체크박스 문구 그대로, 관측만)

1. `HTS_LIM.vcxproj` 한 파일에 `HTS_Rx_Matched_Filter.cpp` 와 `HTS_V400_Dispatcher_Sync.cpp` 가 동시에 `ClCompile Include` 로 존재 (B절 grep 원문 참조).

2. `HTS_V400_Dispatcher_Sync.cpp` 의 `SecureMemory::secureWipe` 호출 줄 번호가 MF 블록(`#if defined(HTS_SYNC_USE_MATCHED_FILTER)`) 밖에 존재 (C절 grep 원문 참조).

3. `class HTS_V400_Dispatcher` 문자열이 여러 `.hpp` 경로에 존재 (ripgrep 결과):

```
HTS_TEST\t6_sim\HTS_V400_Dispatcher_Local.hpp:167
HTS_LIM\HTS_V400_Dispatcher.hpp:157
HTS_TEST\t6_sim\_backup_before_step_b\HTS_V400_Dispatcher_Local.hpp:164
HTS_Jammer_STD\HTS_V400_Dispatcher.hpp:154
HTS_TEST\pipeline_experiment\HTS_V400_Dispatcher.hpp:153
HTS_TEST\pipeline_experiment_6515323\HTS_V400_Dispatcher.hpp:162
HTS_TEST\pipeline_experiment_stage0_baseline\HTS_V400_Dispatcher.hpp:162
Core\Inc\HTS_V400_Dispatcher.hpp:162
HTS_TEST\HTS_V400_Dispatcher.hpp:162
HTS_TEST\PLUTO_SDR\HTS_V400_Dispatcher.hpp:162
```

---

## 부록: `pipeline_audit_raw` 파일 목록

디렉터리: `D:\HTS_ARM11_Firmware\HTS_LIM\pipeline_audit_raw\`

생성·갱신된 주요 파일:

- `audit_all_sources_hts_lim_tree.txt`
- `audit_all_sources_test.txt`
- `audit_all_sources_core.txt`
- `audit_cpp_hts_lim_hts_lim_only.txt`
- `audit_cpp_test_paths.txt`
- `audit_hpp_mainline_paths.txt`
- `audit_bat_sources.txt`
- `audit_vcxproj_sources.txt`
- `audit_build_refs_full.txt`
- `audit_make_files.txt`
- `audit_orphans_v2.txt` / `run_orphans_v2.py`
- `audit_cpp_ref_line_counts.txt`
- `audit_bat_targets.txt`
- `audit_vcxproj_targets.txt`
- `audit_arm_project_files.txt`
- `audit_bak_files.txt`
- `stepB_mf_dir.txt`
- `stepB_mf_sizes.txt`
- `stepB_mf_independent_users_paths.txt`
- `audit_orphan_headers.txt` / `run_orphan_headers.py`

---

END OF REPORT
