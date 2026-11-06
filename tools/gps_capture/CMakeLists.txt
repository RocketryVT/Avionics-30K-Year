cmake_minimum_required(VERSION 3.18)

# Pico 2 W board and SDK/toolchain selection must precede project().
set(PICO_BOARD pico2_w CACHE STRING "Target board")
include("${CMAKE_CURRENT_LIST_DIR}/../../motor/firmware/pico_sdk_import.cmake")

project(gps_capture LANGUAGES C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

pico_sdk_init()

# Reuse the portable parser and collector without building desktop programs.
set(TRACKER_BUILD_DESKTOP OFF)
add_subdirectory(
    "${CMAKE_CURRENT_SOURCE_DIR}/../../tracker"
    "${CMAKE_CURRENT_BINARY_DIR}/tracker"
    EXCLUDE_FROM_ALL
)

add_executable(gps_capture main.cpp)
target_link_libraries(gps_capture PRIVATE
    pico_stdlib
    hardware_uart
    gps_parser
)
target_compile_options(gps_capture PRIVATE -Wall -Wextra)

# Debug output goes over USB, leaving UART0 dedicated to the GPS.
pico_enable_stdio_usb(gps_capture 1)
pico_enable_stdio_uart(gps_capture 0)
pico_add_extra_outputs(gps_capture)
