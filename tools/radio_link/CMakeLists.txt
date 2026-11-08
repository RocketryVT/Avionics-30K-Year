cmake_minimum_required(VERSION 3.18)

# Select the Pico and import the SDK before project().
set(PICO_BOARD pico2_w CACHE STRING "Target board")

include(
    "${CMAKE_CURRENT_LIST_DIR}/../../motor/firmware/pico_sdk_import.cmake"
)

project(radio_link LANGUAGES C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

pico_sdk_init()

# Download a fixed RadioLib release during configuration.
include(FetchContent)

FetchContent_Declare(radiolib
    GIT_REPOSITORY https://github.com/jgromes/RadioLib.git
    GIT_TAG 7.8.1
    GIT_SHALLOW TRUE
)

FetchContent_MakeAvailable(radiolib)

# Reuse tracker libraries without its desktop programs.
set(TRACKER_BUILD_DESKTOP OFF)

add_subdirectory(
    "${CMAKE_CURRENT_SOURCE_DIR}/../../tracker"
    "${CMAKE_CURRENT_BINARY_DIR}/tracker"
    EXCLUDE_FROM_ALL
)

# Build our program with RadioLib's native Pico hardware layer.
add_executable(radio_link
    main.cpp
    "${CMAKE_CURRENT_SOURCE_DIR}/../../tracker/src/radio_link.cpp"
    "${radiolib_SOURCE_DIR}/src/hal/RPiPico/PicoHal.cpp"
)

option(RADIO_LINK_TRANSMITTER "Build the transmitting radio" OFF)

if(RADIO_LINK_TRANSMITTER)
    target_compile_definitions(radio_link PRIVATE
        RADIO_LINK_TRANSMITTER=1
    )
endif()

target_compile_definitions(radio_link PRIVATE
    RADIOLIB_BUILD_RPI_PICO
)

target_link_libraries(radio_link PRIVATE
    RadioLib
    position_packet
    gps_parser
    pico_stdlib
    hardware_spi
    hardware_gpio
    hardware_timer
    hardware_pwm
    pico_multicore
)

set(REPLAY_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/../../tracker/tests/bench_tests/replay"
)

target_sources(radio_link PRIVATE
    "${REPLAY_DIR}/replay_position.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../motor/firmware/src/line_reader.cpp"
)

target_include_directories(radio_link PRIVATE
    "${REPLAY_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../motor/firmware/src"
)

# Print diagnostic messages over USB.
pico_enable_stdio_usb(radio_link 1)
pico_enable_stdio_uart(radio_link 0)

pico_add_extra_outputs(radio_link)
