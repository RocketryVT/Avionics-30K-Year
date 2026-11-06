#pragma once

#include <optional>
#include <string_view>
#include "gps_position.hpp"

/**
 * @brief Parses one complete NMEA GPGGA or GNGGA sentence into a GPS position.
 *
 * @param sentence Text beginning with '$' and ending with '*HH', optionally
 *                 followed by CR, LF, or CRLF. Leading noise is not accepted.
 * @return Decimal-degree coordinates and ellipsoidal altitude in meters, or
 *         std::nullopt if the checksum, message type, field count, fix quality,
 *         coordinates, or required altitude fields are invalid.
 *
 * @note The current parser requires exactly 15 fields and fix quality 1 or 2.
 *       Both altitude unit fields must be 'M'. Ellipsoidal altitude is the
 *       mean-sea-level altitude plus the signed geoid separation.
 * @note UTC time, satellite count, and HDOP are not validated or returned.
 *       The caller must track fix freshness separately.
 */
std::optional<GPSPosition> parseGGA(std::string_view sentence);
