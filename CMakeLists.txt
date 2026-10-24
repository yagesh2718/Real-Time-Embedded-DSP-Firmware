cmake_minimum_required(VERSION 3.12)
project(RealTimeEmbeddedDspFirmware VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler optimization and warning options
if(MSVC)
    add_compile_options(/W4 /O2 /permissive-)
else()
    add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

# Sanitizer options (supported on Linux/macOS and Clang/GCC with libasan installed)
option(ENABLE_SANITIZERS "Enable AddressSanitizer and UndefinedBehaviorSanitizer" OFF)
if(ENABLE_SANITIZERS AND NOT MSVC)
    message(STATUS "Enabling AddressSanitizer (ASan) and UndefinedBehaviorSanitizer (UBSan)")
    add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
    add_link_options(-fsanitize=address,undefined)
endif()

# Core DSP firmware library (pure portable algorithms, zero heap allocation in sample path)
add_library(dsp_firmware_core STATIC
    src/simulated_adc.cpp
    src/fir_filter.cpp
    src/iir_biquad.cpp
    src/fixed_point_dsp.cpp
    src/dsp_pipeline.cpp
    src/benchmark.cpp
)

target_include_directories(dsp_firmware_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# CLI firmware simulator and benchmark runner executable
add_executable(dsp_firmware_sim
    src/main.cpp
)
target_link_libraries(dsp_firmware_sim PRIVATE dsp_firmware_core)

# Automated unit test suite executable
add_executable(dsp_firmware_tests
    tests/test_main.cpp
    tests/test_circular_buffer.cpp
    tests/test_fir_filter.cpp
    tests/test_iir_filter.cpp
    tests/test_fixed_point.cpp
    tests/test_simulated_adc.cpp
)
target_include_directories(dsp_firmware_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/tests
)
target_link_libraries(dsp_firmware_tests PRIVATE dsp_firmware_core)

# CTest integration
enable_testing()
add_test(NAME DspUnitTests COMMAND dsp_firmware_tests)
