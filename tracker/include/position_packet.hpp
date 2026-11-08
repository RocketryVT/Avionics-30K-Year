#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "gps_position.hpp"

/** @brief Version identifier reserved for the first position packet format. */
constexpr std::uint8_t POSITION_PACKET_VERSION = 1;
/** @brief Size of the defined encoded payload in bytes, independent of struct size. */
constexpr std::size_t POSITION_PACKET_SIZE = 22;

/** @brief Bit 0 indicates a usable position; remaining unspecified bits must be zero. */
constexpr std::uint8_t POSITION_FLAG_VALID = 0x01;
/**
 * @brief Bit 1 identifies altitude derived from a barometer.
 * @note This flag does not change units or the required ellipsoidal reference.
 */
constexpr std::uint8_t POSITION_FLAG_BAROMETRIC_ALTITUDE = 0x02;

/**
 * @brief Stores a position and its telemetry metadata before or after encoding.
 *
 * @note This is an in-memory structure, not the transmitted byte layout.
 *       Setting POSITION_FLAG_VALID is the producer's responsibility after
 *       validating the position. Reserved flag bits must remain zero.
 */
struct PositionPacket
{
    /// Validity and altitude-source bits; zero does not mark a position valid.
    std::uint8_t flags = 0;
    /// Transmitter-assigned sequence number for identifying packet order.
    std::uint32_t sequence = 0;

    /// Transmitter uptime when this fix was accepted, in milliseconds.
    std::uint32_t timestamp = 0;

    /// Decimal-degree latitude/longitude and meters above the WGS84 ellipsoid.
    GPSPosition position{};
};

/**
 * @brief Holds the fixed 22-byte payload defined for radio transmission.
 *
 * @details The layout is version at byte 0, flags at byte 1, sequence
 *          at bytes 2-5, transmitter timestamp at bytes 6-9, latitude at bytes
 *          10-13, longitude at bytes 14-17, and altitude at bytes 18-21.
 *          Multi-byte fields use big-endian order. Latitude and longitude are
 *          signed 32-bit degrees times 10^7; altitude is signed 32-bit
 *          ellipsoidal meters times 100. Sequence and timestamp are unsigned.
 * @note The transmitter timestamp uses a different clock from the receiver.
 *       Reception timeouts require the receiver's own arrival timestamps.
 */
using PositionPacketBytes =
    std::array<std::uint8_t, POSITION_PACKET_SIZE>;

/**
 * @brief Validates and encodes a position packet into the defined wire format.
 *
 * @param packet Position and metadata to validate and encode field by field.
 * @return Encoded bytes, or std::nullopt for invalid flags, coordinates,
 *         non-finite values, or values outside the encoded numeric range.
 */
std::optional<PositionPacketBytes> encodePositionPacket(
    const PositionPacket &packet);

/**
 * @brief Decodes and validates a position packet from its transmitted bytes.
 *
 * @param data Pointer to the received payload bytes.
 * @param length Number of received bytes available at data.
 * @return A validated packet, or std::nullopt for invalid input.
 * @note This checks the wire format and position ranges, not packet ordering
 *       or whether the application should submit a motor target.
 */
std::optional<PositionPacket> decodePositionPacket(
    const std::uint8_t *data,
    std::size_t length);
