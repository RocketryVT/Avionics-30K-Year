#pragma once

#include <cstdint>
#include <optional>

#include "pointing.hpp"
#include "command_parser.hpp"

/**
 * @brief Converts absolute pointing angles into an internal motor TARGET command.
 *
 * @param angles Azimuth in degrees [0, 360) and elevation in degrees [-90, 90].
 * @param commandId Identifier copied into the command for reply correlation.
 * @return A Command with Verb::Target and rounded millidegree angles, or
 *         std::nullopt for non-finite angles or angles outside those ranges.
 *
 * @note Azimuth inputs at or above 180 degrees are shifted by -360000
 *       millidegrees after rounding, matching the motor's signed angle frame.
 * @note This function only constructs a command. The motor controller handles
 *       tracking state, active settings, travel limits, and movement execution.
 *       Its zero and gear mode must match the antenna's pointing frame.
 */
std::optional<Command> makeMotorTarget(
    const PointingAngles &angles,
    uint16_t commandId);
