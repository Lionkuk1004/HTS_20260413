# Toolchain: GNU Arm Embedded (arm-none-eabi-g++)
# 사용: cmake -B build -S . -DCMAKE_TOOLCHAIN_FILE=arm_firmware/cmake/arm-none-eabi-gcc.cmake
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR ARM)

set(TOOLCHAIN_PREFIX arm-none-eabi-)
find_program(CMAKE_C_COMPILER   ${TOOLCHAIN_PREFIX}gcc   REQUIRED)
find_program(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}g++   REQUIRED)
find_program(CMAKE_ASM_COMPILER ${TOOLCHAIN_PREFIX}gcc   REQUIRED)
find_program(CMAKE_OBJCOPY      ${TOOLCHAIN_PREFIX}objcopy REQUIRED)
find_program(CMAKE_SIZE         ${TOOLCHAIN_PREFIX}size   REQUIRED)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

set(MCPU cortex-m4)
set(FPU_FLAGS "-mfpu=fpv4-sp-d16 -mfloat-abi=hard")

set(CMAKE_C_FLAGS_INIT
    "-mcpu=${MCPU} -mthumb ${FPU_FLAGS} -ffunction-sections -fdata-sections")
set(CMAKE_CXX_FLAGS_INIT
    "${CMAKE_C_FLAGS_INIT} -fno-exceptions -fno-rtti")
set(CMAKE_ASM_FLAGS_INIT "-mcpu=${MCPU} -mthumb ${FPU_FLAGS}")

set(CMAKE_EXE_LINKER_FLAGS_INIT
    "-specs=nano.specs -specs=nosys.specs -Wl,--gc-sections -Wl,-print-memory-usage")
