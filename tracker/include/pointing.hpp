#pragma once

#include <optional>
#include "gps_position.hpp"

/**
 * @brief Stores the angles required to point an antenna toward a specific location.
 */
struct PointingAngles
{
    double azimuth;   ///< Degrees [0, 360), measured clockwise from true north.
    double elevation; ///< Degrees [-90, 90], measured relative to the horizon.
};

/**
 * @brief Calculates the azimuth and elevation angles required
 *        to point an antenna from a ground station to a rocket.
 *
 * @param rPos Rocket position in decimal degrees and ellipsoidal meters.
 * @param gsPos Ground-station position in the same units and reference system.
 *
 * @return std::optional<PointingAngles>
 *         Returns a PointingAngles struct containing the calculated
 *         azimuth and elevation when the calculation is valid.
 *
 *         Returns std::nullopt when:
 *         - Latitude or longitude is outside its valid range.
 *         - Any coordinate contains NaN or infinity.
 *         - The relative distance is less than the provisional 5-meter limit.
 *         - The horizontal displacement is less than the provisional
 *           5-meter limit, including directions directly above or below
 *           the station where azimuth is undefined.
 *
 * @note Both altitudes must be meters above the WGS84 ellipsoid.
 *       The calculation converts WGS84 to ECEF, then to the station's ENU frame.
 *
 * @note Azimuth is measured clockwise from true north:
 *       0 = North, 90 = East, 180 = South, 270 = West.
 *       Elevation is positive above the horizon and negative below it.
 */
std::optional<PointingAngles> calculatePointingAngles(
    const GPSPosition &rPos,
    const GPSPosition &gsPos);
