#include "motor_interface.hpp"

#include <cmath>

std::optional<Command> makeMotorTarget(
    const PointingAngles &angles,
    uint16_t commandId)
{
    if (!std::isfinite(angles.azimuth) ||
        !std::isfinite(angles.elevation) ||
        angles.azimuth < 0 || angles.azimuth >= 360 ||
        angles.elevation < -90 || angles.elevation > 90)
    {
        return std::nullopt;
    }

    Command command;

    command.verb = Verb::Target;
    command.id = commandId;

    command.az_mdeg = std::round(angles.azimuth * 1000);
    command.el_mdeg = std::round(angles.elevation * 1000);

    if (angles.azimuth >= 180)
    {
        command.az_mdeg -= 360 * 1000;
    }

    return command;
}