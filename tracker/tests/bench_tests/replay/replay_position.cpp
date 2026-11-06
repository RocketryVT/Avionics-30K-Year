#include "replay_position.hpp"
#include "replay_data.hpp"
#include <cmath>
#include <cstddef>

namespace
{
    double interpolate(double start, double end, double fraction)
    {
        return start + (end - start) * fraction;
    }
}

std::optional<GPSPosition> replayPositionAt(
    double timeSeconds,
    double geoidSeparationMeters)
{
    if (!std::isfinite(timeSeconds) ||
        !std::isfinite(geoidSeparationMeters) ||
        REPLAY_SAMPLES.size() < 2)
    {
        return std::nullopt;
    }

    if (timeSeconds < REPLAY_SAMPLES.front().timeSeconds ||
        timeSeconds > REPLAY_SAMPLES.back().timeSeconds)
    {
        return std::nullopt;
    }

    for (std::size_t i = 1; i < REPLAY_SAMPLES.size(); ++i)
    {
        const auto &before = REPLAY_SAMPLES[i - 1];
        const auto &after = REPLAY_SAMPLES[i];

        if (timeSeconds > after.timeSeconds)
            continue;

        const double fraction =
            (timeSeconds - before.timeSeconds) /
            (after.timeSeconds - before.timeSeconds);

        GPSPosition position{
            interpolate(before.latitude, after.latitude, fraction),
            interpolate(before.longitude, after.longitude, fraction),
            interpolate(
                before.altitudeMslMeters,
                after.altitudeMslMeters,
                fraction) +
                geoidSeparationMeters};

        if (!std::isfinite(position.altitude))
            return std::nullopt;

        return position;
    }

    return std::nullopt;
}