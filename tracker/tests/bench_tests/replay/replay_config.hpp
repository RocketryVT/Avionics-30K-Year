#pragma once

#include <cstdint>
#include <optional>

#include "gps_position.hpp"

/**
 * @brief Ground-station position for the simulated launch geometry.
 * @note Fill in latitude, longitude, and WGS84 ellipsoidal altitude later.
 */
inline constexpr std::optional<GPSPosition> REPLAY_STATION =
    GPSPosition{28.5954790666, -80.6, 0.0};

/**
 * @brief Geoid separation added to the CSV's sea-level altitude, in meters.
 * @note Ellipsoidal altitude = sea-level altitude + geoid separation.
 */
inline constexpr std::optional<double> REPLAY_GEOID_SEPARATION_M =
    0.0;

/**
 * @brief Time between replay updates: 200 milliseconds gives 5 Hz.
 */
inline constexpr std::uint32_t REPLAY_UPDATE_PERIOD_MS = 200;