# =============================================================================
# HTS_TEST — PC 전용 검증 타깃 (ARM 펌웨어·hts_cs_core 와 분리)
#
# - 본 CMake는 HTS_LIM/arm_firmware 등 크로스 빌드 트리에 add_subdirectory 되지 않음.
# - 산출: HTS_Calibration_Run(.exe) + HTS_Fractal_Channel_Compare_Run(.exe)
#         + HTS_V400_SIC_PC_Test_Run(.exe) + HTS_RS_FEC_Sequential_Audit_Run(.exe)
#         + HTS_Pipeline_Sequential_Audit_Run(.exe)
#         + HTS_Microbench_Run(.exe)
#         + HTS_Fractal_Mapper_Run(.exe) + libhts_pc_layer2_tensor_fec
#         + HTS_Layer345_Sequential_Audit_Run(.exe) ← Layer3–5 KCMVP+CondSelfTest+KeyRotator
#         + HTS_Layer6789_Sequential_Audit_Run(.exe) ← Layer6–9 (MSVC x64만 — Anti_Debug.cpp 제약)
#         + HTS_Layer10111213_Sequential_Audit_Run(.exe) ← Layer10–13 FEC/V400/Pipeline/Role (MSVC)
#         + HTS_Layer1417_Sequential_Audit_Run(.exe) ← Layer14–17 IPC/OTA/Modbus/Mesh/IoT/Beacon/Scheduler (MSVC)
#
# 구성 예 (저장소 루트 = HTS_LIM):
#   cmake -S HTS_TEST -B HTS_TEST/build-calibration -G "Visual Studio 18 2026" -A x64
#   cmake --build HTS_TEST/build-calibration --config Release
#   Release\HTS_Calibration_Run.exe
#   Release\HTS_Fractal_Channel_Compare_Run.exe ← Tensor 프랙탈 CSV + FEC_HARQ IR CSV
#   Release\HTS_V400_SIC_PC_Test_Run.exe ← V400 디스패처 IR-HARQ + SIC 루프백 스모크
#   Release\HTS_RS_FEC_Sequential_Audit_Run.exe ← RS+FEC_HARQ 널/경계 스모크
#   Release\HTS_Pipeline_Sequential_Audit_Run.exe ← Pipeline V2 + BPS + RF_Metrics 스모크
#   Release\HTS_Microbench_Run.exe   ← 핫패스 ns/호출 (Release 필수)
#   Release\HTS_Fractal_Mapper_Run.exe ← Dynamic_Fractal_Mapper 전수/벤치 (PC)
# =============================================================================
cmake_minimum_required(VERSION 3.20)
project(HTS_PC_TEST_Calibration LANGUAGES CXX)
enable_language(C)

set(HTS_LIM_CORE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../HTS_LIM")

# ---- 정적 라이브러리: HTS_3D_Tensor_FEC (PC Layer 2 단일 TU) ----
add_library(hts_pc_layer2_tensor_fec STATIC
  "${HTS_LIM_CORE_DIR}/HTS_Channel_Physics.cpp"
  "${HTS_LIM_CORE_DIR}/HTS_AntiJam_Engine.cpp"
  "${HTS_LIM_CORE_DIR}/HTS_Dynamic_Fractal_Mapper.cpp"
  "${HTS_LIM_CORE_DIR}/HTS_3D_Tensor_FEC.cpp"
  "${HTS_LIM_CORE_DIR}/HTS_FEC_HARQ.cpp"
  "${HTS_LIM_CORE_DIR}/HTS_RS_GF16.cpp"
  "${HTS_LIM_CORE_DIR}/HTS_Secure_Memory.cpp"
  "${HTS_LIM_CORE_DIR}/HTS_Holo_LPI.cpp"
)
target_include_directories(hts_pc_layer2_tensor_fec PUBLIC
  "${HTS_LIM_CORE_DIR}"
)
target_compile_features(hts_pc_layer2_tensor_fec PUBLIC cxx_std_20)
target_compile_definitions(hts_pc_layer2_tensor_fec PRIVATE
  HTS_FEC_SIMULATE_M4_RAM_LAYOUT
)

if(MSVC)
  target_compile_options(hts_pc_layer2_tensor_fec PRIVATE /W3 /EHsc /permissive-)
  target_compile_definitions(hts_pc_layer2_tensor_fec PRIVATE _CRT_SECURE_NO_WARNINGS)
else()
  target_compile_options(hts_pc_layer2_tensor_fec PRIVATE -Wall -Wextra -O2)
endif()

# ---- 실행 파일: 채널 캘리브레이션 CSV (main 은 Calibration_Test TU 단독) ----
add_executable(HTS_Calibration_Run
  calibration/HTS_Channel_Calibration.cpp
  calibration/HTS_Channel_Calibration_Test.cpp
)
target_link_libraries(HTS_Calibration_Run PRIVATE hts_pc_layer2_tensor_fec)
target_include_directories(HTS_Calibration_Run PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/calibration"
  "${HTS_LIM_CORE_DIR}"
)
target_compile_features(HTS_Calibration_Run PRIVATE cxx_std_20)
target_compile_definitions(HTS_Calibration_Run PRIVATE HTS_PC_CALIBRATION_RUN=1)

if(MSVC)
  target_compile_options(HTS_Calibration_Run PRIVATE /W3 /EHsc /permissive-)
  target_compile_definitions(HTS_Calibration_Run PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()

# ---- 실행 파일: 프랙탈 와이어 비교 CSV + FEC_HARQ V400 Chase/IR CSV ----
add_executable(HTS_Fractal_Channel_Compare_Run
  benchmark/HTS_Fractal_Channel_Compare.cpp
  benchmark/HTS_Pipeline_V2_Compare.cpp
  "${HTS_LIM_CORE_DIR}/HTS_Pipeline_V2_Dispatcher.cpp"
  "${HTS_LIM_CORE_DIR}/HTS_Adaptive_BPS_Controller.cpp"
)
target_link_libraries(HTS_Fractal_Channel_Compare_Run PRIVATE hts_pc_layer2_tensor_fec)
target_include_directories(HTS_Fractal_Channel_Compare_Run PRIVATE
  "${HTS_LIM_CORE_DIR}"
)
target_compile_features(HTS_Fractal_Channel_Compare_Run PRIVATE cxx_std_20)
target_compile_definitions(HTS_Fractal_Channel_Compare_Run PRIVATE
  HTS_FEC_SIMULATE_M4_RAM_LAYOUT
)
if(MSVC)
  target_compile_options(HTS_Fractal_Channel_Compare_Run PRIVATE /W3 /EHsc /permissive-)
  target_compile_definitions(HTS_Fractal_Channel_Compare_Run PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()

# ---- 실행 파일: V400 디스패처 IR-HARQ + SIC 루프백 (PC 스모크) ----
add_executable(HTS_V400_SIC_PC_Test_Run
  benchmark/HTS_V400_SIC_PC_Test.cpp
  "${HTS_LIM_CORE_DIR}/HTS_V400_Dispatcher.cpp"
)
target_link_libraries(HTS_V400_SIC_PC_Test_Run PRIVATE hts_pc_layer2_tensor_fec)
target_include_directories(HTS_V400_SIC_PC_Test_Run PRIVATE
  "${HTS_LIM_CORE_DIR}"
)
target_compile_features(HTS_V400_SIC_PC_Test_Run PRIVATE cxx_std_20)
target_compile_definitions(HTS_V400_SIC_PC_Test_Run PRIVATE
  HTS_FEC_SIMULATE_M4_RAM_LAYOUT
)
if(MSVC)
  target_compile_options(HTS_V400_SIC_PC_Test_Run PRIVATE /W3 /EHsc /permissive-)
  target_compile_definitions(HTS_V400_SIC_PC_Test_Run PRIVATE _CRT_SECURE_NO_WARNINGS)
else()
  target_compile_options(HTS_V400_SIC_PC_Test_Run PRIVATE -Wall -Wextra -O2)
endif()

# ---- 실행 파일: L3 실측 스펙트럼 (Build_Packet NRZ 기준선 PSD) ----
add_executable(HTS_L3_Spectrum_Measured_Test
  benchmark/HTS_L3_Spectrum_Measured_Test.cpp
  "${HTS_LIM_CORE_DIR}/HTS_V400_Dispatcher.cpp"
)
target_link_libraries(HTS_L3_Spectrum_Measured_Test PRIVATE hts_pc_layer2_tensor_fec)
target_include_directories(HTS_L3_Spectrum_Measured_Test PRIVATE
  "${HTS_LIM_CORE_DIR}"
)
target_compile_features(HTS_L3_Spectrum_Measured_Test PRIVATE cxx_std_20)
target_compile_definitions(HTS_L3_Spectrum_Measured_Test PRIVATE
  HTS_FEC_SIMULATE_M4_RAM_LAYOUT
)
if(MSVC)
  target_compile_options(HTS_L3_Spectrum_Measured_Test PRIVATE /W3 /EHsc /permissive-)
  target_compile_definitions(HTS_L3_Spectrum_Measured_Test PRIVATE _CRT_SECURE_NO_WARNINGS)
else()
  target_compile_options(HTS_L3_Spectrum_Measured_Test PRIVATE -Wall -Wextra -O2)
endif()

# ---- 실행 파일: RS + FEC_HARQ 순차 감사 (널·경계·상태 재초기화) ----
add_executable(HTS_RS_FEC_Sequential_Audit_Run
  benchmark/HTS_RS_FEC_Sequential_Audit_PC_Test.cpp
)
target_link_libraries(HTS_RS_FEC_Sequential_Audit_Run PRIVATE hts_pc_layer2_tensor_fec)
target_include_directories(HTS_RS_FEC_Sequential_Audit_Run PRIVATE "${HTS_LIM_CORE_DIR}")
target_compile_features(HTS_RS_FEC_Sequential_Audit_Run PRIVATE cxx_std_20)
target_compile_definitions(HTS_RS_FEC_Sequential_Audit_Run PRIVATE
  HTS_FEC_SIMULATE_M4_RAM_LAYOUT
)
if(MSVC)
  target_compile_options(HTS_RS_FEC_Sequential_Audit_Run PRIVATE /W3 /EHsc /permissive-)
  target_compile_definitions(HTS_RS_FEC_Sequential_Audit_Run PRIVATE _CRT_SECURE_NO_WARNINGS)
else()
  target_compile_options(HTS_RS_FEC_Sequential_Audit_Run PRIVATE -Wall -Wextra -O2)
endif()

# ---- 실행 파일: Pipeline V2 + Adaptive BPS + RF_Metrics (경량 TU 묶음) ----
add_executable(HTS_Pipeline_Sequential_Audit_Run
  benchmark/HTS_Pipeline_Sequential_Audit_PC_Test.cpp
  "${HTS_LIM_CORE_DIR}/HTS_Pipeline_V2_Dispatcher.cpp"
  "${HTS_LIM_CORE_DIR}/HTS_Adaptive_BPS_Controller.cpp"
  "${HTS_LIM_CORE_DIR}/HTS_Dynamic_Fractal_Mapper.cpp"
)
target_include_directories(HTS_Pipeline_Sequential_Audit_Run PRIVATE "${HTS_LIM_CORE_DIR}")
target_compile_features(HTS_Pipeline_Sequential_Audit_Run PRIVATE cxx_std_20)
if(MSVC)
  target_compile_options(HTS_Pipeline_Sequential_Audit_Run PRIVATE /W3 /EHsc /permissive-)
  target_compile_definitions(HTS_Pipeline_Sequential_Audit_Run PRIVATE _CRT_SECURE_NO_WARNINGS)
else()
  target_compile_options(HTS_Pipeline_Sequential_Audit_Run PRIVATE -Wall -Wextra -O2)
endif()

# ---- 실행 파일: Dynamic_Fractal_Mapper (18비트 Feistel) PC 검증 ----
add_executable(HTS_Fractal_Mapper_Run
  calibration/HTS_Dynamic_Fractal_Mapper_PC_Test.cpp
  "${HTS_LIM_CORE_DIR}/HTS_Dynamic_Fractal_Mapper.cpp"
)
target_include_directories(HTS_Fractal_Mapper_Run PRIVATE "${HTS_LIM_CORE_DIR}")
target_compile_features(HTS_Fractal_Mapper_Run PRIVATE cxx_std_20)
target_compile_definitions(HTS_Fractal_Mapper_Run PRIVATE HTS_PC_FRACTAL_MAPPER_TEST=1)
if(MSVC)
  target_compile_options(HTS_Fractal_Mapper_Run PRIVATE /W3 /EHsc /permissive-)
  target_compile_definitions(HTS_Fractal_Mapper_Run PRIVATE _CRT_SECURE_NO_WARNINGS)
else()
  target_compile_options(HTS_Fractal_Mapper_Run PRIVATE -Wall -Wextra -O2)
endif()

# ---- 실행 파일: 핫패스 wall-clock 마이크로벤치 (Release 권장) ----
add_executable(HTS_Microbench_Run
  benchmark/HTS_Microbench_Run.cpp
  "${HTS_LIM_CORE_DIR}/HTS_Adaptive_BPS_Controller.cpp"
)
target_link_libraries(HTS_Microbench_Run PRIVATE hts_pc_layer2_tensor_fec)
target_include_directories(HTS_Microbench_Run PRIVATE "${HTS_LIM_CORE_DIR}")
target_compile_features(HTS_Microbench_Run PRIVATE cxx_std_20)
if(MSVC)
  target_compile_options(HTS_Microbench_Run PRIVATE /W3 /EHsc /permissive-)
  target_compile_definitions(HTS_Microbench_Run PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()

# ---- 실행 파일: Layer 3→4→5 순차 감사 (KCMVP TU + 조건부 자가진단 + 키 로테이터) ----
add_executable(HTS_Layer345_Sequential_Audit_Run
  benchmark/HTS_Layer345_Sequential_Audit_PC_Test.cpp
  "${HTS_LIM_CORE_DIR}/HTS_ARIA_Bridge.cpp"
  "${HTS_LIM_CORE_DIR}/HTS_HMAC_Bridge.cpp"
  "${HTS_LIM_CORE_DIR}/HTS_LEA_Bridge.cpp"
  "${HTS_LIM_CORE_DIR}/HTS_LSH256_Bridge.cpp"
  "${HTS_LIM_CORE_DIR}/HTS_ConstantTimeUtil.cpp"
  "${HTS_LIM_CORE_DIR}/HTS_Secure_Memory.cpp"
  "${HTS_LIM_CORE_DIR}/HTS_Conditional_SelfTest.cpp"
  "${HTS_LIM_CORE_DIR}/HTS_Key_Rotator.cpp"
  "${HTS_LIM_CORE_DIR}/aria050117.c"
  "${HTS_LIM_CORE_DIR}/KISA_SHA256.c"
  "${HTS_LIM_CORE_DIR}/lea_base.c"
  "${HTS_LIM_CORE_DIR}/lea_core.c"
  "${HTS_LIM_CORE_DIR}/lea_gcm_generic.c"
  "${HTS_LIM_CORE_DIR}/lea_online.c"
  "${HTS_LIM_CORE_DIR}/lea_t_fallback.c"
  "${HTS_LIM_CORE_DIR}/lea_t_generic.c"
  "${HTS_LIM_CORE_DIR}/lsh.c"
  "${HTS_LIM_CORE_DIR}/lsh256.c"
  "${HTS_LIM_CORE_DIR}/lsh512.c"
)
target_include_directories(HTS_Layer345_Sequential_Audit_Run PRIVATE "${HTS_LIM_CORE_DIR}")
target_compile_features(HTS_Layer345_Sequential_Audit_Run PRIVATE cxx_std_20)
target_compile_definitions(HTS_Layer345_Sequential_Audit_Run PRIVATE HTS_ALLOW_HOST_BUILD=1)
if(MSVC)
  target_compile_options(HTS_Layer345_Sequential_Audit_Run PRIVATE /W3 /EHsc /permissive-)
  target_compile_definitions(HTS_Layer345_Sequential_Audit_Run PRIVATE _CRT_SECURE_NO_WARNINGS)
else()
  target_compile_options(HTS_Layer345_Sequential_Audit_Run PRIVATE -Wall -Wextra -O2)
endif()

# ---- 실행 파일: Layer 6→9 (AntiGlitch/PAC/Logger/DSP/Tensor) — MSVC 전용 ----
# HTS_Anti_Debug.cpp 는 비-ARM에서 _MSC_VER 일 때만 빌드 허용(GCC/Clang Linux 타깃 제외).
if(MSVC)
  add_executable(HTS_Layer6789_Sequential_Audit_Run
    benchmark/HTS_Layer6789_Sequential_Audit_PC_Test.cpp
    "${HTS_LIM_CORE_DIR}/HTS_Anti_Glitch.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_Anti_Debug.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_Pointer_Auth.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_Secure_Logger.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_Auto_Rollback_Manager.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_Hardware_Bridge.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_Hardware_Init.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_Secure_Memory.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_Crc32Util.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_Rx_Matched_Filter.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_Dynamic_Config.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_AntiJam_Engine.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_Holo_Tensor_4D.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_Sparse_Recovery.cpp"
  )
  target_include_directories(HTS_Layer6789_Sequential_Audit_Run PRIVATE "${HTS_LIM_CORE_DIR}")
  target_compile_features(HTS_Layer6789_Sequential_Audit_Run PRIVATE cxx_std_20)
  target_compile_definitions(HTS_Layer6789_Sequential_Audit_Run PRIVATE
    HTS_ALLOW_HOST_BUILD=1
    _CRT_SECURE_NO_WARNINGS
  )
  target_compile_options(HTS_Layer6789_Sequential_Audit_Run PRIVATE /W3 /EHsc /permissive-)
endif()

# ---- 실행 파일: Layer 10→13 (TxScheduler/FEC/V400/SecurityPipeline/RoleAuth/Universal_API) ----
if(MSVC)
  add_executable(HTS_Layer10111213_Sequential_Audit_Run
    benchmark/HTS_Layer10111213_Sequential_Audit_PC_Test.cpp
    "${HTS_LIM_CORE_DIR}/HTS_Tx_Scheduler.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_Dynamic_Config.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_Secure_Memory.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_FEC_HARQ.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_RS_GF16.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_V400_Dispatcher.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_Holo_LPI.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_AntiJam_Engine.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_Security_Pipeline.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_Universal_API.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_Gyro_Engine.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_Sparse_Recovery.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_AntiAnalysis_Shield.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_Physical_Entropy_Engine.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_Secure_Logger.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_Crc32Util.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_Hardware_Bridge.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_Hardware_Init.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_Anti_Debug.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_Anti_Glitch.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_Auto_Rollback_Manager.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_Pointer_Auth.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_Role_Auth.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_HMAC_Bridge.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_ConstantTimeUtil.cpp"
    "${HTS_LIM_CORE_DIR}/KISA_SHA256.c"
  )
  target_include_directories(HTS_Layer10111213_Sequential_Audit_Run PRIVATE "${HTS_LIM_CORE_DIR}")
  target_compile_features(HTS_Layer10111213_Sequential_Audit_Run PRIVATE cxx_std_20)
  target_compile_definitions(HTS_Layer10111213_Sequential_Audit_Run PRIVATE
    HTS_ALLOW_HOST_BUILD=1
    HTS_FEC_SIMULATE_M4_RAM_LAYOUT=1
    _CRT_SECURE_NO_WARNINGS
  )
  target_compile_options(HTS_Layer10111213_Sequential_Audit_Run PRIVATE /W3 /EHsc /permissive-)
endif()

# ---- 실행 파일: Layer 14→17 (IPC/Console/OTA/Modbus/Mesh/IoT/Emergency/Priority/Sensor/Universal_API) ----
if(MSVC)
  add_executable(HTS_Layer1417_Sequential_Audit_Run
    benchmark/HTS_Layer14_17_Sequential_Audit_PC_Test.cpp
    benchmark/HTS_Layer1417_Host_IPC_Tick_API_Stub.cpp
    "${HTS_LIM_CORE_DIR}/HTS_IPC_Protocol.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_Console_Manager.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_OTA_Manager.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_Modbus_Gateway.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_Mesh_Sync.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_IoT_Codec.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_Emergency_Beacon.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_Priority_Scheduler.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_Sensor_Fusion.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_Tx_Scheduler.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_Dynamic_Config.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_Secure_Memory.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_FEC_HARQ.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_RS_GF16.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_V400_Dispatcher.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_Holo_LPI.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_AntiJam_Engine.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_Security_Pipeline.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_Universal_API.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_Gyro_Engine.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_Sparse_Recovery.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_AntiAnalysis_Shield.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_Physical_Entropy_Engine.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_Secure_Logger.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_Crc32Util.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_Hardware_Bridge.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_Hardware_Init.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_Anti_Debug.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_Anti_Glitch.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_Auto_Rollback_Manager.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_Pointer_Auth.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_Role_Auth.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_HMAC_Bridge.cpp"
    "${HTS_LIM_CORE_DIR}/HTS_ConstantTimeUtil.cpp"
    "${HTS_LIM_CORE_DIR}/KISA_SHA256.c"
  )
  target_include_directories(HTS_Layer1417_Sequential_Audit_Run PRIVATE "${HTS_LIM_CORE_DIR}")
  target_compile_features(HTS_Layer1417_Sequential_Audit_Run PRIVATE cxx_std_20)
  target_compile_definitions(HTS_Layer1417_Sequential_Audit_Run PRIVATE
    HTS_ALLOW_HOST_BUILD=1
    HTS_FEC_SIMULATE_M4_RAM_LAYOUT=1
    _CRT_SECURE_NO_WARNINGS
  )
  target_compile_options(HTS_Layer1417_Sequential_Audit_Run PRIVATE /W3 /EHsc /permissive-)
endif()

# Windows: 정적 링크는 기본적으로 CRT + 위 정적 lib 조합 (별도 DLL 불필요)
