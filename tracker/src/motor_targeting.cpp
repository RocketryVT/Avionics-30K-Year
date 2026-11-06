#include "motor_targeting.hpp"

#include <cmath>

std::optional<AngularVelocity> calcAngularVelocity(
    const PointingAngles &current,
    const PointingAngles &previous,
    float elapsedTime)
{
    if (!std::isfinite(elapsedTime) ||
        elapsedTime <= 0.0 ||
        !std::isfinite(current.azimuth) ||
        !std::isfinite(current.elevation) ||
        !std::isfinite(previous.azimuth) ||
        !std::isfinite(previous.elevation))
    {
        return std::nullopt;
    }

    AngularVelocity angularVelocity;

    angularVelocity.azimuth = std::remainder(current.azimuth - previous.azimuth, 360.0) / elapsedTime;
    angularVelocity.elevation = (current.elevation - previous.elevation) / elapsedTime;

    if (!std::isfinite(angularVelocity.azimuth) ||
        !std::isfinite(angularVelocity.elevation))
    {
        return std::nullopt;
    }

    return angularVelocity;
}

std::optional<PositionError> calcPositionError(
    const PointingAngles &target,
    const PointingAngles &antennaPosition)
{
    if (!std::isfinite(target.azimuth) ||
        !std::isfinite(target.elevation) ||
        !std::isfinite(antennaPosition.azimuth) ||
        !std::isfinite(antennaPosition.elevation))
    {
        return std::nullopt;
    }

    PositionError positionError;

    positionError.azimuth = std::remainder(target.azimuth - antennaPosition.azimuth, 360.0);
    positionError.elevation = target.elevation - antennaPosition.elevation;

    if (!std::isfinite(positionError.azimuth) ||
        !std::isfinite(positionError.elevation))
    {
        return std::nullopt;
    }

    return positionError;
}

std::optional<AngularVelocity> calcDesiredVelocity(
    const PointingAngles &currentTarget,
    const PointingAngles &previousTarget,
    const PointingAngles &antennaPosition,
    double elapsedTime,
    double kp)
{
}