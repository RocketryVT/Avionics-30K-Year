#pragma once

#include <optional>
#include "gps_position.hpp"

/** @brief Selects live GPS or the existing simulated trajectory as the radio source. */
enum class RadioPositionSource
{
    Gps,
    Replay
};

/**
 * @brief Selects the transmitter source and matching ground-station reference.
 * @note This is a compile-time choice. Rebuild and flash both boards together.
 */
inline constexpr RadioPositionSource RADIO_POSITION_SOURCE =
    RadioPositionSource::Replay;

/**
 * @brief Defines the ground-station reference used only in live GPS mode.
 *
 * @note Latitude/longitude are decimal degrees; altitude is WGS84 ellipsoidal
 *       meters. std::nullopt refuses radio arming in GPS mode. Replay uses
 *       REPLAY_STATION from replay_config.hpp instead.
 */
inline constexpr std::optional<GPSPosition> RADIO_LIVE_STATION =
    std::nullopt;
