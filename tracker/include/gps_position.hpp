#pragma once

/**
 * @brief Stores a geographic position using WGS84 latitude, longitude, and height.
 *
 * @note Altitude is measured above the WGS84 ellipsoid. Convert mean-sea-level
 *       altitude or height above launch into this reference before using it.
 * @note This structure stores values without validating them.
 */
struct GPSPosition
{
    double latitude;  ///< Decimal degrees [-90, 90]; north is positive.
    double longitude; ///< Decimal degrees [-180, 180]; east is positive.
    double altitude;  ///< Meters above the WGS84 ellipsoid.
};
