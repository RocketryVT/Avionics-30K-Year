#include "coordinate_utils.hpp"
#include <cmath>

namespace
{
    // Constants
    constexpr double PI = 3.14159265358979323846;
    constexpr double SEMI_MAJOR_AXIS = 6378137;
    constexpr double ECCENTRICITY_SQUARED = 0.00669437999014;
}

// Convert GPS coordinates to ECEF Cartesian coordinates (meters)
ECEFPosition gpsToECEF(const GPSPosition &pos)
{
    ECEFPosition result;
    double latRadians = pos.latitude * (PI / 180);
    double longRadians = pos.longitude * (PI / 180);

    double sinLat = std::sin(latRadians);
    double cosLat = std::cos(latRadians);

    double N = SEMI_MAJOR_AXIS / std::sqrt(1.0 - ECCENTRICITY_SQUARED * sinLat * sinLat);

    result.x = (N + pos.altitude) * cosLat * std::cos(longRadians);
    result.y = (N + pos.altitude) * cosLat * std::sin(longRadians);
    result.z = (N * (1.0 - ECCENTRICITY_SQUARED) + pos.altitude) * sinLat;

    return result;
}

// Converts given ECEF coordinates to ENU with the Ground Station as the reference point.
ENUPosition ecefToENU(
    const ECEFPosition &rocket,
    const ECEFPosition &station,
    const GPSPosition &stationGPS)
{
    ENUPosition result;

    double relativePosition[3] = {0, 0, 0};

    double latRadians = stationGPS.latitude * (PI / 180);
    double longRadians = stationGPS.longitude * (PI / 180);

    double sinLat = std::sin(latRadians);
    double cosLat = std::cos(latRadians);
    double sinLong = std::sin(longRadians);
    double cosLong = std::cos(longRadians);

    relativePosition[0] = rocket.x - station.x;
    relativePosition[1] = rocket.y - station.y;
    relativePosition[2] = rocket.z - station.z;

    result.east = -sinLong * relativePosition[0] + cosLong * relativePosition[1];
    result.north = -sinLat * cosLong * relativePosition[0] - sinLat * sinLong * relativePosition[1] + cosLat * relativePosition[2];
    result.up = cosLat * cosLong * relativePosition[0] + cosLat * sinLong * relativePosition[1] + sinLat * relativePosition[2];

    return result;
}