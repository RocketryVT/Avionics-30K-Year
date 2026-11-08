// Unit tests for the position packet encoder and decoder.
// No third-party C++ test framework is required.

#include "test_helpers.hpp"
#include "position_packet.hpp"

using test::expectEquals;
using test::expectNear;

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <limits>
#include <string>

namespace
{
    void expectValidPositionPacketBytes(
        const std::string &name,
        const PositionPacket &packet,
        const PositionPacketBytes &bytes)
    {
        const auto result = encodePositionPacket(packet);
        test::expectTrue(name + " returns Position Packet Bytes", result.has_value());
        if (result)
        {
            for (std::size_t i = 0; i < bytes.size(); ++i)
            {
                expectEquals(name + " byte " + std::to_string(i),
                             static_cast<unsigned>((*result)[i]),
                             static_cast<unsigned>(bytes[i]));
            }
        }
    }

    void expectValidPositionPacket(
        const std::string &name,
        const std::uint8_t *data,
        std::size_t length,
        const PositionPacket &packet,
        double coordinateTolerance = 1e-7,
        double altitudeTolerance = 0.01)
    {
        const auto result = decodePositionPacket(data, length);
        test::expectTrue(name + " returns Position Packet", result.has_value());
        if (result)
        {
            expectEquals(name + " flags", static_cast<unsigned>(result->flags),
                         static_cast<unsigned>(packet.flags));
            expectEquals(name + " sequence", result->sequence, packet.sequence);
            expectEquals(name + " timestamp", result->timestamp, packet.timestamp);
            expectNear(name + " altitude (m)", result->position.altitude,
                       packet.position.altitude, altitudeTolerance);
            expectNear(name + " latitude (deg)", result->position.latitude,
                       packet.position.latitude, coordinateTolerance);
            expectNear(name + " longitude (deg)", result->position.longitude,
                       packet.position.longitude, coordinateTolerance);
        }
    }

    void expectInvalidPositionPacketBytes(
        const std::string &name,
        const PositionPacket &packet)
    {
        const auto result = encodePositionPacket(packet);
        test::expectTrue(name + " returns nullopt", !result.has_value());
    }

    void expectInvalidPositionPacket(
        const std::string &name,
        const std::uint8_t *data,
        std::size_t length)
    {
        const auto result = decodePositionPacket(data, length);
        test::expectTrue(name + " returns nullopt", !result.has_value());
    }
    const PositionPacket KNOWN_PACKET{
        POSITION_FLAG_VALID, 0x12345678u, 0x89ABCDEFu, {12.3456789, -98.7654321, -123.45}};

    // Fixed wire references are independent of the production encoder/decoder.
    // The four-byte groups are sequence, timestamp, latitude, longitude, altitude.
    constexpr PositionPacketBytes KNOWN_BYTES{
        0x01, 0x01,
        0x12, 0x34, 0x56, 0x78,
        0x89, 0xAB, 0xCD, 0xEF,
        0x07, 0x5B, 0xCD, 0x15,
        0xC5, 0x21, 0x97, 0x4F,
        0xFF, 0xFF, 0xCF, 0xC7};

    void testKnownWireFormat()
    {
        expectValidPositionPacketBytes("Known mixed-sign encoding", KNOWN_PACKET, KNOWN_BYTES);
        expectValidPositionPacket("Known mixed-sign decoding", KNOWN_BYTES.data(),
                                  KNOWN_BYTES.size(), KNOWN_PACKET, 1e-12, 1e-9);

        const PositionPacket oppositeSigns{
            POSITION_FLAG_VALID | POSITION_FLAG_BAROMETRIC_ALTITUDE,
            0x12345678u,
            0x89ABCDEFu,
            {-12.3456789, 98.7654321, 123.45}};
        constexpr PositionPacketBytes oppositeBytes{
            0x01, 0x03,
            0x12, 0x34, 0x56, 0x78,
            0x89, 0xAB, 0xCD, 0xEF,
            0xF8, 0xA4, 0x32, 0xEB,
            0x3A, 0xDE, 0x68, 0xB1,
            0x00, 0x00, 0x30, 0x39};
        expectValidPositionPacketBytes("Opposite signs and barometer encoding",
                                       oppositeSigns, oppositeBytes);
        expectValidPositionPacket("Opposite signs and barometer decoding",
                                  oppositeBytes.data(), oppositeBytes.size(),
                                  oppositeSigns, 1e-12, 1e-9);
    }

    void testCoordinateAndIntegerLimits()
    {
        const PositionPacket maximum{
            POSITION_FLAG_VALID,
            std::numeric_limits<std::uint32_t>::max(),
            std::numeric_limits<std::uint32_t>::max(),
            {90.0, 180.0, 21474836.47}};
        constexpr PositionPacketBytes maximumBytes{
            0x01, 0x01,
            0xFF, 0xFF, 0xFF, 0xFF,
            0xFF, 0xFF, 0xFF, 0xFF,
            0x35, 0xA4, 0xE9, 0x00,
            0x6B, 0x49, 0xD2, 0x00,
            0x7F, 0xFF, 0xFF, 0xFF};
        expectValidPositionPacketBytes("Maximum coordinates and integer fields",
                                       maximum, maximumBytes);
        expectValidPositionPacket("Decode maximum coordinates and integer fields",
                                  maximumBytes.data(), maximumBytes.size(), maximum, 1e-12, 1e-9);

        const PositionPacket minimum{
            POSITION_FLAG_VALID, 0, 0, {-90.0, -180.0, -21474836.48}};
        constexpr PositionPacketBytes minimumBytes{
            0x01, 0x01,
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
            0xCA, 0x5B, 0x17, 0x00,
            0x94, 0xB6, 0x2E, 0x00,
            0x80, 0x00, 0x00, 0x00};
        expectValidPositionPacketBytes("Minimum coordinates and integer fields",
                                       minimum, minimumBytes);
        expectValidPositionPacket("Decode minimum coordinates and integer fields",
                                  minimumBytes.data(), minimumBytes.size(), minimum, 1e-12, 1e-9);
    }

    void testRounding()
    {
        // These binary-exact inputs produce exact half-unit ties when scaled.
        constexpr PositionPacketBytes positiveBytes{
            0x01, 0x01, 0, 0, 0, 0, 0, 0, 0, 0,
            0x00, 0x00, 0x98, 0x97,
            0x00, 0x00, 0x98, 0x97,
            0x00, 0x00, 0x00, 0x0D};
        constexpr PositionPacketBytes negativeBytes{
            0x01, 0x01, 0, 0, 0, 0, 0, 0, 0, 0,
            0xFF, 0xFF, 0x67, 0x69,
            0xFF, 0xFF, 0x67, 0x69,
            0xFF, 0xFF, 0xFF, 0xF3};

        for (int sign : {1, -1})
        {
            const std::string name = sign > 0 ? "Positive rounding tie" : "Negative rounding tie";
            const auto &bytes = sign > 0 ? positiveBytes : negativeBytes;
            const PositionPacket input{
                POSITION_FLAG_VALID, 0, 0, {sign * 0.00390625, sign * 0.00390625, sign * 0.125}};
            const PositionPacket rounded{
                POSITION_FLAG_VALID, 0, 0, {sign * 0.0039063, sign * 0.0039063, sign * 0.13}};

            expectValidPositionPacketBytes(name + " encodes away from zero", input, bytes);
            expectValidPositionPacket(name + " decodes the rounded value", bytes.data(),
                                      bytes.size(), rounded, 1e-12, 1e-12);
        }
    }

    void testRoundTrips()
    {
        const GPSPosition positions[] = {
            {0.0, 0.0, 0.0},
            {37.2235, -80.4421893, 614.9},
            {-37.2235, 80.4421893, -33.4},
            {-0.0000001, -0.0000001, -0.01},
            {12.34567894, -98.76543216, 123.456}};

        for (std::size_t i = 0; i < std::size(positions); ++i)
        {
            auto packet = KNOWN_PACKET;
            packet.position = positions[i];
            const std::string name = "Round trip " + std::to_string(i);
            const auto bytes = encodePositionPacket(packet);
            test::expectTrue(name + " encodes", bytes.has_value());
            if (bytes)
                expectValidPositionPacket(name, bytes->data(), bytes->size(), packet);
        }
    }

    void testInvalidEncodedPositions()
    {
        const double infinity = std::numeric_limits<double>::infinity();
        const double nan = std::numeric_limits<double>::quiet_NaN();
        struct Case
        {
            const char *name;
            GPSPosition position;
        };
        const Case cases[] = {
            {"Latitude above 90", {91, 0, 0}},
            {"Latitude below -90", {-91, 0, 0}},
            {"Longitude above 180", {0, 181, 0}},
            {"Longitude below -180", {0, -181, 0}},
            {"Latitude just above 90", {std::nextafter(90.0, infinity), 0, 0}},
            {"Latitude just below -90", {std::nextafter(-90.0, -infinity), 0, 0}},
            {"Longitude just above 180", {0, std::nextafter(180.0, infinity), 0}},
            {"Longitude just below -180", {0, std::nextafter(-180.0, -infinity), 0}},
            {"NaN latitude", {nan, 0, 0}},
            {"NaN longitude", {0, nan, 0}},
            {"NaN altitude", {0, 0, nan}},
            {"Positive infinite latitude", {infinity, 0, 0}},
            {"Negative infinite latitude", {-infinity, 0, 0}},
            {"Positive infinite longitude", {0, infinity, 0}},
            {"Negative infinite longitude", {0, -infinity, 0}},
            {"Positive infinite altitude", {0, 0, infinity}},
            {"Negative infinite altitude", {0, 0, -infinity}},
            {"Altitude one encoded unit above int32 maximum", {0, 0, 21474836.48}},
            {"Altitude one encoded unit below int32 minimum", {0, 0, -21474836.49}},
            {"Altitude multiplication overflows positively", {0, 0, std::numeric_limits<double>::max()}},
            {"Altitude multiplication overflows negatively", {0, 0, -std::numeric_limits<double>::max()}}};

        for (const auto &item : cases)
        {
            auto packet = KNOWN_PACKET;
            packet.position = item.position;
            expectInvalidPositionPacketBytes(item.name, packet);
        }
    }

    void testInvalidFlagsAndVersions()
    {
        for (unsigned flags : {0u, 2u, 0x05u, 0x09u, 0x11u, 0x21u, 0x41u, 0x81u, 0xFFu})
        {
            auto packet = KNOWN_PACKET;
            packet.flags = static_cast<std::uint8_t>(flags);
            auto bytes = KNOWN_BYTES;
            bytes[1] = packet.flags;
            const std::string name = "Invalid flags " + std::to_string(flags);
            expectInvalidPositionPacketBytes(name + " encoding", packet);
            expectInvalidPositionPacket(name + " decoding", bytes.data(), bytes.size());
        }

        for (unsigned version : {0u, 2u, 0xFFu})
        {
            auto bytes = KNOWN_BYTES;
            bytes[0] = static_cast<std::uint8_t>(version);
            expectInvalidPositionPacket("Unsupported version " + std::to_string(version),
                                        bytes.data(), bytes.size());
        }
    }

    void testInvalidBuffers()
    {
        expectInvalidPositionPacket("Null pointer with zero length", nullptr, 0);
        expectInvalidPositionPacket("Null pointer with correct length", nullptr, POSITION_PACKET_SIZE);
        for (std::size_t length = 0; length < KNOWN_BYTES.size(); ++length)
        {
            expectInvalidPositionPacket("Truncated length " + std::to_string(length),
                                        KNOWN_BYTES.data(), length);
        }

        // The actual allocation is also short: useful with address sanitization.
        const std::uint8_t oneByte[] = {0x01};
        expectInvalidPositionPacket("One-byte allocation", oneByte, sizeof(oneByte));
        std::array<std::uint8_t, POSITION_PACKET_SIZE + 1> oversized{};
        std::copy(KNOWN_BYTES.begin(), KNOWN_BYTES.end(), oversized.begin());
        expectInvalidPositionPacket("Valid packet with an extra byte", oversized.data(), oversized.size());
    }

    void testInvalidDecodedCoordinates()
    {
        struct Case
        {
            const char *name;
            std::size_t offset;
            std::array<std::uint8_t, 4> encodedCoordinate;
        };
        const Case cases[] = {
            {"Latitude 90 plus one encoded unit", 10, {0x35, 0xA4, 0xE9, 0x01}},
            {"Latitude -90 minus one encoded unit", 10, {0xCA, 0x5B, 0x16, 0xFF}},
            {"Longitude 180 plus one encoded unit", 14, {0x6B, 0x49, 0xD2, 0x01}},
            {"Longitude -180 minus one encoded unit", 14, {0x94, 0xB6, 0x2D, 0xFF}},
            {"Decoded latitude 100", 10, {0x3B, 0x9A, 0xCA, 0x00}},
            {"Decoded latitude -100", 10, {0xC4, 0x65, 0x36, 0x00}},
            {"Decoded longitude 181", 14, {0x6B, 0xE2, 0x68, 0x80}},
            {"Decoded longitude -181", 14, {0x94, 0x1D, 0x97, 0x80}}};

        for (const auto &item : cases)
        {
            auto bytes = KNOWN_BYTES;
            for (std::size_t i = 0; i < item.encodedCoordinate.size(); ++i)
                bytes[item.offset + i] = item.encodedCoordinate[i];
            expectInvalidPositionPacket(item.name, bytes.data(), bytes.size());
        }
    }
}

int main()
{
    testKnownWireFormat();
    testCoordinateAndIntegerLimits();
    testRounding();
    testRoundTrips();
    testInvalidEncodedPositions();
    testInvalidFlagsAndVersions();
    testInvalidBuffers();
    testInvalidDecodedCoordinates();

    return test::summary();
}
