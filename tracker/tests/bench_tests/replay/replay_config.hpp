#pragma once

#include <cstdint>
#include <optional>

#include "gps_position.hpp"

/**
 * @brief Ground-station position for the simulated launch geometry.
 * @note Fill in latitude, longitude, and WGS84 ellipsoidal altitude later.
 * @note 250 ft (76.20 m) due south of the CSV launch point (28.61, -80.6),
 *       solved on the WGS84 ellipsoid. Earlier: one mile south
 *       (28.5954790666), 1,000 ft (28.6072498256), 500 ft (28.6086249129).
 */
inline constexpr std::optional<GPSPosition> REPLAY_STATION =
    GPSPosition{28.6093124565, -80.6, 0.0};

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