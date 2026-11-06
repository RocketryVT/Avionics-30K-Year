#include "pointing.hpp"
#include "coordinate_utils.hpp"
#include <cmath>

// Helper Functions
namespace
{
    // Constants
    constexpr double PI = 3.14159265358979323846;

    // Checks if GPSPostion is in valid ranges
    bool checkRanges(const GPSPosition &pos)
    {
        if (!std::isfinite(pos.latitude) || pos.latitude < -90 || pos.latitude > 90)
            return false;

        if (!std::isfinite(pos.longitude) || pos.longitude < -180 || pos.longitude > 180)
            return false;

        if (!std::isfinite(pos.altitude))
            return false;

        return true;
    }

    // Checks if Relative Postion is valid (not overhead or directly ontop)
    bool checkRelativePosition(const ENUPosition relativePos)
    {
        // Provisional tolerance of 5 meters
        constexpr double POSITION_TOLERANCE = 5.0;
        constexpr double HORIZONTAL_TOLERANCE = 5.0;

        double horizontalDistance = std::hypot(
            relativePos.east,
            relativePos.north);

        double totalDistance = std::hypot(
            horizontalDistance,
            relativePos.up);

        // Check if rocket and ground station are approximately coincident
        if (totalDistance < POSITION_TOLERANCE)
        {
            return false;
        }
        // Check if rocket is approximately above the ground station
        if (horizontalDistance < HORIZONTAL_TOLERANCE)
        {
            return false;
        }

        return true;
    }
}

// Main Pointing Calculation
std::optional<PointingAngles> calculatePointingAngles(
    const GPSPosition &rPos,
    const GPSPosition &gsPos)
{
    if (!checkRanges(rPos) || !checkRanges(gsPos))
    {
        return std::nullopt;
    }

    PointingAngles result;

    ECEFPosition recef = gpsToECEF(rPos);
    ECEFPosition gsecef = gpsToECEF(gsPos);

    ENUPosition relativePos = ecefToENU(recef, gsecef, gsPos);

    if (!checkRelativePosition(relativePos))
    {
        return std::nullopt;
    }

    double horizontalDistance = std::hypot(relativePos.east, relativePos.north);

    result.azimuth = std::atan2(relativePos.east, relativePos.north) * (180.0 / PI);
    result.elevation = std::atan2(relativePos.up, horizontalDistance) * (180.0 / PI);

    // Normalize azimuth to [0, 360)
    if (result.azimuth < 0.0)
        result.azimuth += 360.0;

    if (result.azimuth >= 360.0 - 1e-10)
        result.azimuth = 0.0;

    return result;
}