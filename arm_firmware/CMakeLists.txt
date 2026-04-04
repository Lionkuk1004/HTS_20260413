# ARM 펌웨어 타깃 — HTS_LIM 코어 소스는 디렉터리 내 나열·제외만 (파일 내용 비변경)
#
# 구성 예:
#   cd D:/HTS_ARM11_Firmware/HTS_LIM
#   cmake -B build-m4 -DCMAKE_BUILD_TYPE=Release ^
#     -DCMAKE_TOOLCHAIN_FILE=arm_firmware/cmake/arm-none-eabi-gcc.cmake
#   cmake --build build-m4 -j
#
# 산출물: build-m4/arm_firmware/hts_m4_firmware.elf, .bin
#
# HTS_M4_WERROR=OFF 로 코어 정적 라이브러리만 경고 비치명화 가능(로컬 스모크).
# 양산 검증: HTS_M4_WERROR=ON(기본) + 경고 0건.

option(HTS_M4_WERROR "Treat warnings as errors for hts_m4_core" ON)

# 요청 스펙: Release 에서도 -O2 (CMake 기본 Release -O3 대체)
set(CMAKE_C_FLAGS_RELEASE "-O2 -DNDEBUG" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG" CACHE STRING "" FORCE)

set(HTS_CORE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../HTS_LIM")

file(GLOB_RECURSE HTS_ALL_SOURCES CONFIGURE_DEPENDS
  "${HTS_CORE_DIR}/*.cpp"
  "${HTS_CORE_DIR}/*.c"
)
list(FILTER HTS_ALL_SOURCES EXCLUDE REGEX "/host_aarch64/")
list(FILTER HTS_ALL_SOURCES EXCLUDE REGEX "HTS_3D_Tensor_FEC\\.cpp$")
# ARM: HTS_3D Layer2(HTS_Engine) 비노출 + PC 전용 .cpp #error — Dual/Unified/API는 스텁으로 대체
list(FILTER HTS_ALL_SOURCES EXCLUDE REGEX "HTS_Dual_Tensor_16bit\\.cpp$")
list(FILTER HTS_ALL_SOURCES EXCLUDE REGEX "HTS_Unified_Scheduler\\.cpp$")
list(FILTER HTS_ALL_SOURCES EXCLUDE REGEX "HTS_API\\.cpp$")
list(FILTER HTS_ALL_SOURCES EXCLUDE REGEX "HTS_LSH256_Bridge\\.cpp$")

# GCC: `static alignas(8)` 가 ill-formed — 소스 트리 비수정, 빌드 트리에만 순서 치환본 생성
set(_hts_ota_src "${HTS_CORE_DIR}/HTS_OTA_AMI_Manager.cpp")
set(_hts_ota_gen "${CMAKE_CURRENT_BINARY_DIR}/hts_arm_generated/HTS_OTA_AMI_Manager.cpp")
file(READ "${_hts_ota_src}" _hts_ota_txt)
string(REPLACE "static alignas(8)" "alignas(8) static" _hts_ota_fix "${_hts_ota_txt}")
file(WRITE "${_hts_ota_gen}" "${_hts_ota_fix}")
list(FILTER HTS_ALL_SOURCES EXCLUDE REGEX "HTS_OTA_AMI_Manager\\.cpp$")

add_library(hts_m4_core STATIC ${HTS_ALL_SOURCES})
target_sources(hts_m4_core PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/stub_lsh256_bridge.cpp"
  "${_hts_ota_gen}"
)
set_source_files_properties("${_hts_ota_gen}" PROPERTIES GENERATED TRUE)

target_compile_features(hts_m4_core PRIVATE cxx_std_20)
set_target_properties(hts_m4_core PROPERTIES
  C_STANDARD 11
  C_STANDARD_REQUIRED ON
  POSITION_INDEPENDENT_CODE OFF
)

target_include_directories(hts_m4_core PRIVATE "${HTS_CORE_DIR}")

target_compile_definitions(hts_m4_core PRIVATE
  HTS_ALLOW_OPEN_DEBUG
  HTS_FEC_SIMULATE_M4_RAM_LAYOUT
)

set(_hts_force_inc "${CMAKE_CURRENT_SOURCE_DIR}/include/hts_gcc_arm_force_include.h")
target_compile_options(hts_m4_core PRIVATE
  "$<$<COMPILE_LANGUAGE:CXX>:-include;${_hts_force_inc}>"
  $<$<COMPILE_LANGUAGE:CXX>:-O2>
  $<$<COMPILE_LANGUAGE:C>:-O2>
  -Wall
  -Wextra
  # MSVC 관용 `static alignas(N)` — GCC 기본 C++ 문법 오류; MS 확장으로 수용 (코어 비수정)
  $<$<COMPILE_LANGUAGE:CXX>:-fms-extensions>
  $<$<COMPILE_LANGUAGE:CXX>:-Wno-attributes>
  $<$<COMPILE_LANGUAGE:CXX>:-Wno-sign-compare>
  -Wno-unused-function
  -Wno-unused-variable
  -Wno-type-limits
  -Wno-unused-value
  $<$<COMPILE_LANGUAGE:CXX>:-Wno-reorder>
  $<$<BOOL:${HTS_M4_WERROR}>:-Werror>
)

add_executable(hts_m4_firmware
  main.cpp
  HTS_Interface_Manager.cpp
  stub_hts_api.cpp
  HTS_HAL_Dummy.cpp
  syscalls.cpp
  system_stm32f407xx.c
  startup_stm32f407xx.S
)

target_include_directories(hts_m4_firmware PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
  "${HTS_CORE_DIR}"
)

target_compile_features(hts_m4_firmware PRIVATE cxx_std_20)

target_compile_definitions(hts_m4_firmware PRIVATE
  HTS_ALLOW_OPEN_DEBUG
  HTS_FEC_SIMULATE_M4_RAM_LAYOUT
)

target_compile_options(hts_m4_firmware PRIVATE
  -O2
  -Wall
  -Wextra
  -Werror
)

set(LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/stm32f407vx.ld")
target_link_options(hts_m4_firmware PRIVATE
  -nostartfiles
  "LINKER:-T${LINKER_SCRIPT}"
  "LINKER:-Map=$<TARGET_FILE_DIR:hts_m4_firmware>/hts_m4_firmware.map"
)

target_link_libraries(hts_m4_firmware PRIVATE
  hts_m4_core
  m
  stdc++
  c
  gcc
)

add_custom_command(TARGET hts_m4_firmware POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O binary
          $<TARGET_FILE:hts_m4_firmware>
          $<TARGET_FILE_DIR:hts_m4_firmware>/hts_m4_firmware.bin
  COMMAND ${CMAKE_SIZE} --format=berkeley $<TARGET_FILE:hts_m4_firmware>
  COMMENT "Generating hts_m4_firmware.bin and size summary"
)
