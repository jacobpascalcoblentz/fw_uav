cmake_minimum_required(VERSION 3.16)
project(fw_uav VERSION 0.1.0 LANGUAGES CXX C)

# C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Options
option(FW_UAV_BUILD_TESTS "Build unit tests" ON)
option(FW_UAV_BUILD_SITL "Build software-in-the-loop simulation" ON)
option(FW_UAV_BUILD_EXAMPLES "Build example applications" ON)
option(FW_UAV_STM32_TARGET "Target STM32 family (F4, H7, or OFF for host)" OFF)

# Include custom CMake modules
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Detect if cross-compiling for STM32
if(FW_UAV_STM32_TARGET)
    message(STATUS "Cross-compiling for STM32${FW_UAV_STM32_TARGET}")
    set(FW_UAV_EMBEDDED ON)
    add_compile_definitions(FW_UAV_EMBEDDED=1)
    add_compile_definitions(FW_UAV_STM32_${FW_UAV_STM32_TARGET}=1)
else()
    message(STATUS "Building for host platform")
    set(FW_UAV_EMBEDDED OFF)
    add_compile_definitions(FW_UAV_HOST=1)
endif()

# Compiler warnings
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(
        -Wall
        -Wextra
        -Wpedantic
        -Werror=return-type
        -Wno-unused-parameter
    )
    if(NOT FW_UAV_EMBEDDED)
        add_compile_options(-fsanitize=address,undefined)
        add_link_options(-fsanitize=address,undefined)
    endif()
endif()

# Main library
add_library(fw_uav STATIC
    # Core
    src/control/pid.cpp

    # Control (to be added)
    # src/control/attitude_controller.cpp
    # src/control/tecs.cpp
    # src/control/l1_controller.cpp

    # Estimation (to be added)
    # src/estimation/attitude_estimator.cpp
    # src/estimation/position_estimator.cpp

    # Navigation (to be added)
    # src/navigation/navigator.cpp
    # src/navigation/mission.cpp
    # src/navigation/geofence.cpp

    # Safety (to be added)
    # src/safety/failsafe.cpp
    # src/safety/envelope_protection.cpp

    # Modes (to be added)
    # src/modes/mode_manual.cpp
    # src/modes/mode_stabilize.cpp
    # src/modes/mode_fbw.cpp
    # src/modes/mode_auto.cpp
)

target_include_directories(fw_uav PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Platform-specific HAL implementation
if(FW_UAV_EMBEDDED)
    target_sources(fw_uav PRIVATE
        # STM32 HAL implementations (to be added)
        # platform/stm32/imu_spi.cpp
        # platform/stm32/gps_uart.cpp
        # platform/stm32/barometer.cpp
        # platform/stm32/pwm_output.cpp
        # platform/stm32/pwm_input.cpp
    )

    # Link STM32 HAL libraries
    # target_link_libraries(fw_uav PRIVATE stm32_hal cmsis)
endif()

# Tests
if(FW_UAV_BUILD_TESTS AND NOT FW_UAV_EMBEDDED)
    enable_testing()

    # Fetch GoogleTest
    include(FetchContent)
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG v1.14.0
    )
    FetchContent_MakeAvailable(googletest)

    # Unit tests
    add_executable(fw_uav_tests
        tests/unit/test_types.cpp
        tests/unit/test_result.cpp
        tests/unit/test_pid.cpp
        tests/unit/test_units.cpp
    )

    target_link_libraries(fw_uav_tests PRIVATE
        fw_uav
        GTest::gtest_main
    )

    include(GoogleTest)
    gtest_discover_tests(fw_uav_tests)
endif()

# SITL Simulation
if(FW_UAV_BUILD_SITL AND NOT FW_UAV_EMBEDDED)
    add_executable(sitl_main
        tests/simulation/sitl_main.cpp
        # aircraft_model.cpp and sensor_sim.cpp are included directly in sitl_main.cpp
    )

    target_include_directories(sitl_main PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests/simulation)
    target_link_libraries(sitl_main PRIVATE fw_uav)
endif()

# Examples
if(FW_UAV_BUILD_EXAMPLES AND NOT FW_UAV_EMBEDDED)
    add_executable(example_minimal_stabilize
        examples/minimal_stabilize/main.cpp
    )
    target_link_libraries(example_minimal_stabilize PRIVATE fw_uav)

    add_executable(example_full_autopilot
        examples/full_autopilot/main.cpp
    )
    target_link_libraries(example_full_autopilot PRIVATE fw_uav)
endif()

# Installation
install(TARGETS fw_uav
    EXPORT fw_uav_targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(DIRECTORY include/fw_uav
    DESTINATION include
    FILES_MATCHING PATTERN "*.h"
)

install(EXPORT fw_uav_targets
    FILE fw_uav_targets.cmake
    NAMESPACE fw_uav::
    DESTINATION lib/cmake/fw_uav
)

# Package configuration
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/fw_uav_config_version.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/fw_uav_config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/fw_uav_config.cmake"
    INSTALL_DESTINATION lib/cmake/fw_uav
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/fw_uav_config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/fw_uav_config_version.cmake"
    DESTINATION lib/cmake/fw_uav
)
