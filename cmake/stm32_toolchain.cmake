# STM32 Cross-Compilation Toolchain File
# Usage: cmake -DCMAKE_TOOLCHAIN_FILE=../cmake/stm32_toolchain.cmake -DFW_UAV_STM32_TARGET=F4 ..

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR ARM)

# Find ARM toolchain
find_program(ARM_CC arm-none-eabi-gcc)
find_program(ARM_CXX arm-none-eabi-g++)
find_program(ARM_OBJCOPY arm-none-eabi-objcopy)
find_program(ARM_SIZE arm-none-eabi-size)

if(NOT ARM_CC)
    message(FATAL_ERROR "arm-none-eabi-gcc not found. Install ARM GCC toolchain.")
endif()

set(CMAKE_C_COMPILER ${ARM_CC})
set(CMAKE_CXX_COMPILER ${ARM_CXX})
set(CMAKE_ASM_COMPILER ${ARM_CC})
set(CMAKE_OBJCOPY ${ARM_OBJCOPY})
set(CMAKE_SIZE ${ARM_SIZE})

# Don't run test programs on cross-compile
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# Target-specific flags
if(NOT DEFINED FW_UAV_STM32_TARGET)
    set(FW_UAV_STM32_TARGET "F4")
endif()

# CPU flags based on target
if(FW_UAV_STM32_TARGET STREQUAL "F4")
    set(CPU_FLAGS "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")
    set(STM32_DEFINES "-DSTM32F4xx -DSTM32F407xx")
elseif(FW_UAV_STM32_TARGET STREQUAL "H7")
    set(CPU_FLAGS "-mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard")
    set(STM32_DEFINES "-DSTM32H7xx -DSTM32H743xx")
else()
    message(FATAL_ERROR "Unknown STM32 target: ${FW_UAV_STM32_TARGET}")
endif()

# Common embedded flags
set(COMMON_FLAGS "${CPU_FLAGS} -ffunction-sections -fdata-sections -fno-common -fno-exceptions -fno-rtti")

set(CMAKE_C_FLAGS_INIT "${COMMON_FLAGS} ${STM32_DEFINES}")
set(CMAKE_CXX_FLAGS_INIT "${COMMON_FLAGS} ${STM32_DEFINES}")
set(CMAKE_ASM_FLAGS_INIT "${CPU_FLAGS}")

# Optimization flags
set(CMAKE_C_FLAGS_DEBUG "-Og -g3")
set(CMAKE_C_FLAGS_RELEASE "-O2 -DNDEBUG")
set(CMAKE_C_FLAGS_MINSIZEREL "-Os -DNDEBUG")
set(CMAKE_C_FLAGS_RELWITHDEBINFO "-O2 -g3")

set(CMAKE_CXX_FLAGS_DEBUG "-Og -g3")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG")
set(CMAKE_CXX_FLAGS_MINSIZEREL "-Os -DNDEBUG")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O2 -g3")

# Linker flags
set(CMAKE_EXE_LINKER_FLAGS_INIT "${CPU_FLAGS} -Wl,--gc-sections -specs=nosys.specs -specs=nano.specs")

# Search paths for libraries and includes
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
