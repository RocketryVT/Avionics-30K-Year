#pragma once

#include <optional>
#include "gps_position.hpp"

enum class RadioPositionSource
{
    Gps,
    Replay
};

// Build and flash both boards after changing this selection.
inline constexpr RadioPositionSource RADIO_POSITION_SOURCE =
    RadioPositionSource::Replay;

// Set this to the verified station position before selecting live GPS.
inline constexpr std::optional<GPSPosition> RADIO_LIVE_STATION =
    std::nullopt;
