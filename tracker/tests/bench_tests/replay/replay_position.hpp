#pragma once

#include <optional>
#include "gps_position.hpp"

std::optional<GPSPosition> replayPositionAt(
    double timeSeconds,
    double geoidSeparationMeters);