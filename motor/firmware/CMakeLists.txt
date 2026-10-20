# Builds gs_motor, the program for the Pico that drives the two motors.
#
# Build inside WSL, from the repo folder (see docs/SETUP.md):
#   cmake -S firmware -B ~/build/gs -G Ninja     once, and after adding a file
#   ninja -C ~/build/gs                          every time the code changes
#
# The build itself stays in WSL, out of OneDrive. Only the finished
# gs_motor.uf2 is copied back into firmware/uf2/, where Windows can see it.
cmake_minimum_required(VERSION 3.18)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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
# gs_motor
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

target_compile_definitions(gs_motor PRIVATE
    GS_TARGET_NAME=\"gs_motor\"
    GS_VERSION=\"${GS_VERSION}\")

target_compile_options(gs_motor PRIVATE -Wall -Wextra)

target_link_libraries(gs_motor PRIVATE
    pico_stdlib
    hardware_flash
    pico_flash)

# The laptop talks to the Pico over USB. The UART console stays off.
pico_enable_stdio_usb(gs_motor 1)
pico_enable_stdio_uart(gs_motor 0)

# gs_motor.uf2 for drag-and-drop flashing, plus .elf/.map for debugging.
pico_add_extra_outputs(gs_motor)

# Copy the .uf2 into firmware/uf2/. This has to come after
# pico_add_extra_outputs, whose last step is writing the .uf2.
add_custom_command(TARGET gs_motor POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory
            "${CMAKE_CURRENT_SOURCE_DIR}/uf2"
    COMMAND ${CMAKE_COMMAND} -E copy
            "${CMAKE_CURRENT_BINARY_DIR}/gs_motor.uf2"
            "${CMAKE_CURRENT_SOURCE_DIR}/uf2/"
    VERBATIM)
