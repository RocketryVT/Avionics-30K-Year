# Builds gs_motor, the program for the Pico that drives the two motors.
#
# Build inside WSL, from the repo folder (see docs/SETUP.md):
#   cmake -S firmware -B ~/build/gs -G Ninja     once, and after adding a file
#   ninja -C ~/build/gs                          every time the code changes
#
# The build itself stays in WSL, out of OneDrive. Only the finished
# gs_motor.uf2 is copied back into firmware/uf2/, where Windows can see it.
# ----------------------------------------------------------------------------
# Language settings
# ----------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.18)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ----------------------------------------------------------------------------
# Pico board, toolchain, and SDK initialization
# ----------------------------------------------------------------------------

# Pico 2 W (RP2350). Must be set before the SDK is pulled in, because that is
# where the SDK picks the cross-compiler for this chip.
set(PICO_BOARD pico2_w CACHE STRING "Target board")

# Must come before project(): this swaps in the ARM toolchain.
include(pico_sdk_import.cmake)

project(gs_pico_link C CXX ASM)

pico_sdk_init()

# ----------------------------------------------------------------------------
# Version stamp for the startup banner and the PING reply
# ----------------------------------------------------------------------------
# So a Pico found in a drawer can say which build is on it.
#
# Two things worth knowing:
#
#  - safe.directory: the repo lives on the Windows drive (/mnt/c), so git in
#    WSL sees it owned by a different user and refuses to read it. Passing
#    safe.directory for this one command fixes that without touching anyone's
#    git config.
#  - It is read when cmake runs, not when ninja builds. Re-run cmake to
#    refresh it.
get_filename_component(GS_REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}" DIRECTORY)

execute_process(
    COMMAND git -c "safe.directory=${GS_REPO_ROOT}" -C "${GS_REPO_ROOT}"
            describe --always --dirty --tags
    OUTPUT_VARIABLE GS_VERSION
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_VARIABLE GS_VERSION_ERR
    RESULT_VARIABLE GS_VERSION_RESULT)

if(NOT GS_VERSION_RESULT EQUAL 0 OR NOT GS_VERSION)
    set(GS_VERSION "unknown")
    message(WARNING
        "Could not read a version from git, banner will say \"unknown\".\n"
        "  git said: ${GS_VERSION_ERR}")
endif()

message(STATUS "gs_motor version stamp: ${GS_VERSION}")

# ----------------------------------------------------------------------------
# Motor firmware executable and source files
# ----------------------------------------------------------------------------
add_executable(gs_motor
    src/main.cpp
    src/line_reader.cpp
    src/command_parser.cpp
    src/command_rules.cpp
    src/replies.cpp
    src/motion_profile.cpp
    src/stepper.cpp
    src/settings.cpp
    src/settings_flash.cpp)

# ----------------------------------------------------------------------------
# Pointing and motor command adapter integration
# ----------------------------------------------------------------------------
# Include the tracker libraries without its desktop executable or tests.
set(TRACKER_BUILD_DESKTOP OFF)

add_subdirectory(
    "${CMAKE_CURRENT_SOURCE_DIR}/../../tracker"
    "${CMAKE_CURRENT_BINARY_DIR}/tracker"
)

target_link_libraries(gs_motor PRIVATE motor_interface)

# RF69 support, using the same RadioLib release as the standalone build.
include(FetchContent)

FetchContent_Declare(radiolib
    GIT_REPOSITORY https://github.com/jgromes/RadioLib.git
    GIT_TAG 7.8.1
    GIT_SHALLOW TRUE
)

FetchContent_MakeAvailable(radiolib)

target_sources(gs_motor PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../../tracker/src/radio_link.cpp"
    "${radiolib_SOURCE_DIR}/src/hal/RPiPico/PicoHal.cpp"
)

target_compile_definitions(gs_motor PRIVATE RADIOLIB_BUILD_RPI_PICO)

target_link_libraries(gs_motor PRIVATE
    RadioLib
    position_packet
    hardware_spi
    hardware_gpio
    hardware_timer
    hardware_pwm
    pico_multicore
)

# The simulated GPS flight for bench tests. It is always built in, and only
# TRACK REPLAY plays it, so there is one firmware for the bench and the field.
# It costs about 80 KB of the Pico 2 W's 4 MB of flash.
set(REPLAY_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/../../tracker/tests/bench_tests/replay"
)

target_sources(gs_motor PRIVATE
    "${REPLAY_DIR}/replay_position.cpp"
)

target_include_directories(gs_motor PRIVATE "${REPLAY_DIR}")

# GS_ENABLE_REPLAY used to be a switch here. A build folder configured with it
# keeps it in its cache, where it now does nothing - drop it so nobody thinks
# it still matters.
unset(GS_ENABLE_REPLAY CACHE)

# ----------------------------------------------------------------------------
# Firmware identity and compiler warnings
# ----------------------------------------------------------------------------

target_compile_definitions(gs_motor PRIVATE
    GS_TARGET_NAME=\"gs_motor\"
    GS_VERSION=\"${GS_VERSION}\")

target_compile_options(gs_motor PRIVATE -Wall -Wextra)

# ----------------------------------------------------------------------------
# Pico hardware dependencies
# ----------------------------------------------------------------------------

target_link_libraries(gs_motor PRIVATE
    pico_stdlib
    hardware_flash
    pico_flash)

# ----------------------------------------------------------------------------
# USB console configuration
# ----------------------------------------------------------------------------

# The laptop talks to the Pico over USB. The UART console stays off.
pico_enable_stdio_usb(gs_motor 1)
pico_enable_stdio_uart(gs_motor 0)

# ----------------------------------------------------------------------------
# Firmware output files
# ----------------------------------------------------------------------------

# gs_motor.uf2 for drag-and-drop flashing, plus .elf/.map for debugging.
pico_add_extra_outputs(gs_motor)

# ----------------------------------------------------------------------------
# Copy the finished UF2 to the firmware output folder
# ----------------------------------------------------------------------------

# Copy the .uf2 into firmware/uf2/. This has to come after
# pico_add_extra_outputs, whose last step is writing the .uf2.
add_custom_command(TARGET gs_motor POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory
            "${CMAKE_CURRENT_SOURCE_DIR}/uf2"
    COMMAND ${CMAKE_COMMAND} -E copy
            "${CMAKE_CURRENT_BINARY_DIR}/gs_motor.uf2"
            "${CMAKE_CURRENT_SOURCE_DIR}/uf2/"
    VERBATIM)
