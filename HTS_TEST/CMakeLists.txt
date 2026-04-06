# =============================================================================
# HTS_TEST — PC 전용 검증 타깃 (ARM 펌웨어·hts_cs_core 와 분리)
#
# - 본 CMake는 HTS_LIM/arm_firmware 등 크로스 빌드 트리에 add_subdirectory 되지 않음.
# - 산출: HTS_Calibration_Run(.exe) + HTS_Fractal_Channel_Compare_Run(.exe)
#         + HTS_Microbench_Run(.exe) + HTS_Fractal_Mapper_Run(.exe)
#         + libhts_pc_layer2_tensor_fec
#
# 구성 예 (저장소 루트 = HTS_LIM):
#   cmake -S HTS_TEST -B HTS_TEST/build-calibration -G "Visual Studio 18 2026" -A x64
#   cmake --build HTS_TEST/build-calibration --config Release
#   Release\HTS_Calibration_Run.exe
#   Release\HTS_Fractal_Channel_Compare_Run.exe ← Tensor 프랙탈 CSV + FEC_HARQ IR CSV
#   Release\HTS_Microbench_Run.exe   ← 핫패스 ns/호출 (Release 필수)
#   Release\HTS_Fractal_Mapper_Run.exe ← Dynamic_Fractal_Mapper 전수/벤치 (PC)
# =============================================================================
cmake_minimum_required(VERSION 3.20)
project(HTS_PC_TEST_Calibration LANGUAGES CXX)

set(HTS_LIM_CORE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../HTS_LIM")

# ---- 정적 라이브러리: HTS_3D_Tensor_FEC (PC Layer 2 단일 TU) ----
add_library(hts_pc_layer2_tensor_fec STATIC
  "${HTS_LIM_CORE_DIR}/HTS_Channel_Physics.cpp"
  "${HTS_LIM_CORE_DIR}/HTS_AntiJam_Engine.cpp"
  "${HTS_LIM_CORE_DIR}/HTS_Dynamic_Fractal_Mapper.cpp"
  "${HTS_LIM_CORE_DIR}/HTS_3D_Tensor_FEC.cpp"
  "${HTS_LIM_CORE_DIR}/HTS_FEC_HARQ.cpp"
  "${HTS_LIM_CORE_DIR}/HTS_Secure_Memory.cpp"
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

# Windows: 정적 링크는 기본적으로 CRT + 위 정적 lib 조합 (별도 DLL 불필요)
