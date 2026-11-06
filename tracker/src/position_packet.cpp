#include "position_packet.hpp"

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
}

std::optional<PositionPacketBytes> encodePositionPacket(
    const PositionPacket &packet)
{
}

std::optional<PositionPacket> decodePositionPacket(
    const std::uint8_t *data,
    std::size_t length)
{
}