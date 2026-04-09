# Minimum CMake version required for Pico SDK
cmake_minimum_required(VERSION 3.13)

# Select target board
# Options:
#   pico   -> RP2040 (original Pico)
#   pico2  -> RP2350 (Pico 2)
# NOTE: Must be set BEFORE pico_sdk_import.cmake
set(PICO_BOARD pico2 CACHE STRING "Board type")

# Import the Pico SDK (path comes from PICO_SDK_PATH env variable)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Project name and supported languages
project(bmp390 C CXX ASM)

# Set language standards
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the Pico SDK (sets up toolchain, libraries, etc.)
pico_sdk_init()

# Define the executable and source files
add_executable(${PROJECT_NAME}
    src/main.cpp
)

# Link against Pico standard library
# Provides GPIO, timing (sleep_ms), stdio, etc.
target_link_libraries(${PROJECT_NAME}
    pico_stdlib
    hardware_i2c
)

# Enable USB serial output (printf over USB)
pico_enable_stdio_usb(${PROJECT_NAME} 1)

# Disable UART serial output
pico_enable_stdio_uart(${PROJECT_NAME} 0)

# Generate additional output files:
#   .uf2 -> for drag-and-drop flashing
#   .bin -> raw binary
#   .hex -> Intel HEX format
#   .map -> memory map (useful for debugging)
pico_add_extra_outputs(${PROJECT_NAME})