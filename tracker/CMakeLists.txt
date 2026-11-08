# ---------------------------------------------------------------------------
# Project and C++ settings
# ---------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.18)
project(rocket_antenna_tracker LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---------------------------------------------------------------------------
# Pointing library
# ---------------------------------------------------------------------------

# Portable coordinate conversion and pointing calculations.
add_library(pointing
    src/coordinate_utils.cpp
    src/pointing.cpp
)

target_include_directories(pointing PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

# ---------------------------------------------------------------------------
# Motor command adapter library
# ---------------------------------------------------------------------------

# Converts pointing results into the existing motor Command type.
add_library(motor_interface
    src/motor_interface.cpp
)

target_link_libraries(motor_interface PUBLIC pointing)

target_include_directories(motor_interface PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/../motor/firmware/src"
)

# ---------------------------------------------------------------------------
# GPS parser library
# ---------------------------------------------------------------------------

add_library(gps_parser
    src/gps_parser.cpp
    src/sentence_collector.cpp
)

target_include_directories(gps_parser PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

# ---------------------------------------------------------------------------
# Position packet library
# ---------------------------------------------------------------------------

add_library(position_packet
    src/position_packet.cpp
)

target_include_directories(position_packet PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

# ---------------------------------------------------------------------------
# Desktop build switch
# The Pico build turns this OFF; the libraries above remain available.
# ---------------------------------------------------------------------------

option(TRACKER_BUILD_DESKTOP "Build desktop tools and tests" ON)

if(TRACKER_BUILD_DESKTOP)

    # ---------------------------------------------------------------------------
    # Desktop tests
    # ---------------------------------------------------------------------------

    include(CTest)

    if(BUILD_TESTING)
        # Shared assertions and test result reporting.
        add_library(test_helpers
            tests/utils/test_helpers.cpp
        )

        target_include_directories(test_helpers PUBLIC
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/utils"
        )

        # Coordinate conversion tests.
        add_executable(test_coordinate_utils
            tests/cases/test_coordinate_utils.cpp
        )
        target_link_libraries(test_coordinate_utils PRIVATE pointing test_helpers)
        add_test(NAME coordinate_utils COMMAND test_coordinate_utils)

        # GPS-to-pointing calculation tests.
        add_executable(test_pointing
            tests/cases/test_pointing.cpp
        )
        target_link_libraries(test_pointing PRIVATE pointing test_helpers)
        add_test(NAME pointing COMMAND test_pointing)

        # Angular velocity and position error tests.
        add_executable(test_motor_targeting
            tests/cases/test_motor_targeting.cpp
            src/motor_targeting.cpp
        )
        target_link_libraries(test_motor_targeting PRIVATE pointing test_helpers)
        add_test(NAME motor_targeting COMMAND test_motor_targeting)

        # Pointing-to-motor-command adapter tests.
        add_executable(test_motor_interface
            tests/cases/test_motor_interface.cpp
        )
        target_link_libraries(test_motor_interface PRIVATE motor_interface test_helpers)
        add_test(NAME motor_interface COMMAND test_motor_interface)

        # Captured GPS fixes, checksum validation, and malformed inputs.
        add_executable(test_gps_parser
            tests/cases/test_gps_parser.cpp
        )
        target_link_libraries(test_gps_parser PRIVATE gps_parser test_helpers)
        add_test(NAME gps_parser COMMAND test_gps_parser)

        # UART sentence framing, overflow recovery, and GGA parser integration.
        add_executable(test_sentence_collector
            tests/cases/test_sentence_collector.cpp
        )
        target_link_libraries(test_sentence_collector PRIVATE gps_parser test_helpers)
        add_test(NAME sentence_collector COMMAND test_sentence_collector)

        # Simulation interpolation, altitude references, and replay time bounds.
        add_executable(test_replay_position
            tests/cases/test_replay_position.cpp
            tests/bench_tests/replay/replay_position.cpp
        )
        target_include_directories(test_replay_position PRIVATE
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/bench_tests/replay"
        )
        target_link_libraries(test_replay_position PRIVATE pointing test_helpers)
        add_test(NAME replay_position COMMAND test_replay_position)

        # Radio payload layout, signed values, rounding, and malformed packets.
        add_executable(test_position_packet
            tests/cases/test_position_packet.cpp
        )
        target_link_libraries(test_position_packet PRIVATE position_packet test_helpers)
        add_test(NAME position_packet COMMAND test_position_packet)
    endif() # BUILD_TESTING
endif() # TRACKER_BUILD_DESKTOP
