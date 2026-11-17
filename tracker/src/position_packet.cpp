#include "position_packet.hpp"

#include <limits>
#include <cmath>

namespace
{
    std::optional<std::int32_t> scaleToInt32(
        double value,
        double scale)
    {
        if (!std::isfinite(value) ||
            !std::isfinite(scale) ||
            scale <= 0)
        {
            return std::nullopt;
        }

        const double rounded = std::round(value * scale);

        if (!std::isfinite(rounded) ||
            rounded < std::numeric_limits<std::int32_t>::min() ||
            rounded > std::numeric_limits<std::int32_t>::max())
        {
            return std::nullopt;
        }

        return static_cast<std::int32_t>(rounded);
    }

    void writeUint32BE(
        PositionPacketBytes &bytes,
        std::size_t offset,
        std::uint32_t value)
    {
        bytes[offset] =
            static_cast<std::uint8_t>(value >> 24);

        bytes[offset + 1] =
            static_cast<std::uint8_t>(value >> 16);

        bytes[offset + 2] =
            static_cast<std::uint8_t>(value >> 8);

        bytes[offset + 3] =
            static_cast<std::uint8_t>(value);
    }

    std::uint32_t readUint32BE(
        const std::uint8_t *data,
        std::size_t offset)
    {
        return (static_cast<std::uint32_t>(data[offset]) << 24) |
               (static_cast<std::uint32_t>(data[offset + 1]) << 16) |
               (static_cast<std::uint32_t>(data[offset + 2]) << 8) |
               static_cast<std::uint32_t>(data[offset + 3]);
    }

    std::int32_t readInt32BE(
        const std::uint8_t *data,
        std::size_t offset)
    {
        std::int64_t value = readUint32BE(data, offset);

        // Recover a negative value when the 32-bit sign bit is set.
        if (value >= 0x80000000LL)
        {
            value -= 0x100000000LL;
        }

        return static_cast<std::int32_t>(value);
    }
}

std::optional<PositionPacketBytes> encodePositionPacket(
    const PositionPacket &packet)
{
    constexpr std::uint8_t allowedFlags =
        POSITION_FLAG_VALID | POSITION_FLAG_BAROMETRIC_ALTITUDE;

    if ((packet.flags & POSITION_FLAG_VALID) == 0 ||
        (packet.flags & ~allowedFlags) != 0 ||
        packet.position.latitude > 90 ||
        packet.position.latitude < -90 ||
        packet.position.longitude > 180 ||
        packet.position.longitude < -180)
    {
        return std::nullopt;
    }

    auto latInt32 = scaleToInt32(packet.position.latitude, 10000000.0);
    auto longInt32 = scaleToInt32(packet.position.longitude, 10000000.0);
    auto altInt32 = scaleToInt32(packet.position.altitude, 100.0);

    if (!latInt32 || !longInt32 || !altInt32)
    {
        return std::nullopt;
    }

    PositionPacketBytes bytes{};

    bytes[0] = POSITION_PACKET_VERSION;
    bytes[1] = packet.flags;

    writeUint32BE(bytes, 2, packet.sequence);
    writeUint32BE(bytes, 6, packet.timestamp);
    writeUint32BE(bytes, 10, static_cast<std::uint32_t>(*latInt32));
    writeUint32BE(bytes, 14, static_cast<std::uint32_t>(*longInt32));
    writeUint32BE(bytes, 18, static_cast<std::uint32_t>(*altInt32));

    return bytes;
}

std::optional<PositionPacket> decodePositionPacket(
    const uint8_t *data,
    size_t length)
{
    constexpr std::uint8_t allowedFlags =
        POSITION_FLAG_VALID | POSITION_FLAG_BAROMETRIC_ALTITUDE;

    if (!data ||
        length != POSITION_PACKET_SIZE ||
        (data[1] & POSITION_FLAG_VALID) == 0 ||
        (data[1] & ~allowedFlags) != 0 ||
        data[0] != POSITION_PACKET_VERSION)
    {
        return std::nullopt;
    }

    PositionPacket packet;

    packet.flags = data[1];

    packet.sequence = readUint32BE(data, 2);
    packet.timestamp = readUint32BE(data, 6);
    packet.position.latitude = readInt32BE(data, 10) / 10000000.0;
    packet.position.longitude = readInt32BE(data, 14) / 10000000.0;
    packet.position.altitude = readInt32BE(data, 18) / 100.0;

    if (packet.position.latitude < -90 ||
        packet.position.latitude > 90 ||
        packet.position.longitude < -180 ||
        packet.position.longitude > 180)
    {
        return std::nullopt;
    }

    return packet;
}