#pragma once

#include "gps_position.hpp"

/**
 * @brief Stores a position in the Earth-Centered, Earth-Fixed (ECEF)
 *        coordinate system.
 *
 * @note X, Y, and Z are Cartesian coordinates measured in meters
 *       relative to Earth's center.
 *       X points toward latitude 0, longitude 0; Y toward latitude 0,
 *       longitude 90 degrees east; Z toward the north pole.
 */
struct ECEFPosition
{
    double x, y, z; ///< Earth-centered Cartesian coordinates in meters.
};

/**
 * @brief Stores a position relative to a reference point using
 *        the local East-North-Up (ENU) coordinate system.
 *
 * @note East, North, and Up represent displacement in meters
 *       along their respective local axes.
 */
struct ENUPosition
{
    double east, north, up; ///< Displacements along the station's local axes, in meters.
};

/**
 * @brief Converts geographic GPS coordinates to Earth-Centered,
 *        Earth-Fixed (ECEF) Cartesian coordinates using WGS84.
 *
 * @param pos GPS position containing latitude and longitude
 *            in decimal degrees and altitude in meters.
 *
 * @return ECEFPosition
 *         Returns the calculated X, Y, and Z coordinates
 *         in meters relative to Earth's center.
 *
 * @note Altitude must be measured relative to the WGS84
 *       ellipsoid, not mean sea level.
 * @note This conversion does not validate inputs. The caller must provide
 *       finite coordinates with valid latitude and longitude ranges.
 */
ECEFPosition gpsToECEF(const GPSPosition &pos);

/**
 * @brief Converts the rocket's position from ECEF coordinates
 *        into a local East-North-Up (ENU) coordinate system
 *        relative to the ground station.
 *
 * @param rocket ECEF coordinates (X, Y, Z) of the rocket in meters.
 * @param station ECEF coordinates (X, Y, Z) of the ground station in meters.
 * @param stationGPS Geographic coordinates (latitude, longitude, altitude)
 *                   of the ground station. Its latitude and longitude orient
 *                   the ENU axes; its altitude is not used by this function.
 *
 * @return ENUPosition
 *         Returns the rocket's East, North, and Up displacement
 *         relative to the ground station, in meters.
 *
 * @note Latitude and longitude are provided in decimal degrees. The station's
 *       ECEF position and stationGPS must describe the same reference point.
 * @note This conversion does not validate inputs. ECEF positions should be
 *       calculated from WGS84 positions with ellipsoidal altitude in meters.
 */
ENUPosition ecefToENU(const ECEFPosition &rocket, const ECEFPosition &station, const GPSPosition &stationGPS);
