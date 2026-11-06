#include "gps_parser.hpp"

#include <array>
#include <string_view>
#include <charconv>
#include <cstdint>
#include <cmath>

namespace
{
    // Fields 0–14, including empty fields.
    using GGAFields = std::array<std::string_view, 15>;

    // Converts hex to decimal.
    int hexDigitValue(char c)
    {
        if (c >= '0' && c <= '9')
            return c - '0';

        if (c >= 'A' && c <= 'F')
            return c - 'A' + 10;

        if (c >= 'a' && c <= 'f')
            return c - 'a' + 10;

        return -1;
    }

    // Validate $...*HH and its checksum.
    // Return the contents between $ and *.
    std::optional<std::string_view> checkedPayload(
        std::string_view sentence)
    {
        // Accept a sentence with or without its CR/LF terminator.
        if (!sentence.empty() && sentence.back() == '\n')
            sentence.remove_suffix(1);

        if (!sentence.empty() && sentence.back() == '\r')
            sentence.remove_suffix(1);

        // The sentence collector should already have removed leading noise.
        if (sentence.size() < 4 || sentence.front() != '$')
            return std::nullopt;

        const auto star = sentence.find('*');

        // Require '*' followed by exactly two checksum characters.
        if (star == std::string_view::npos ||
            star + 3 != sentence.size())
        {
            return std::nullopt;
        }

        // Start after '$'; stop before '*'.
        const auto payload = sentence.substr(1, star - 1);

        std::uint8_t calculated = 0;

        for (char c : payload)
        {
            calculated ^= static_cast<std::uint8_t>(c);
        }

        const int high = hexDigitValue(sentence[star + 1]);
        const int low = hexDigitValue(sentence[star + 2]);

        if (high < 0 || low < 0)
            return std::nullopt;

        const int expected = high * 16 + low;

        if (calculated != expected)
            return std::nullopt;

        return payload;
    }

    // Split on commas, keeping empty fields.
    // This first version requires exactly 15 fields.
    std::optional<GGAFields> splitGGAFields(
        std::string_view payload)
    {
        GGAFields fields{};
        int index = 0;

        size_t start = 0;

        while (index < fields.size())
        {
            const auto end = payload.find(',', start);

            if (end == std::string_view::npos)
            {
                // No comma remains: this must be the final field.
                if (index != fields.size() - 1)
                    return std::nullopt;

                fields[index] = payload.substr(start);
                return fields;
            }

            fields[index] = payload.substr(start, end - start);
            start = end + 1;
            index++;
        }

        return std::nullopt;
    }

    // Parse a complete finite decimal number.
    // Reject empty text, trailing garbage, NaN, and infinity.
    std::optional<double> parseDecimal(
        std::string_view text)
    {
        if (text.empty())
            return std::nullopt;

        double value = 0.0;

        const auto result = std::from_chars(
            text.data(),
            text.data() + text.size(),
            value,
            std::chars_format::fixed);

        if (result.ec != std::errc{} ||
            result.ptr != text.data() + text.size() ||
            !std::isfinite(value))
        {
            return std::nullopt;
        }

        return value;
    }

    // Convert ddmm.mmmm / dddmm.mmmm to decimal degrees.
    // Validate degree/minute ranges and hemisphere.
    std::optional<double> parseCoordinate(
        std::string_view value,
        std::string_view hemisphere,
        bool isLatitude)
    {
        if (value.empty() || hemisphere.empty())
            return std::nullopt;

        const size_t degreeDigits = isLatitude ? 2 : 3;

        if (value.size() < degreeDigits + 2)
            return std::nullopt;

        const size_t wholeDigits = degreeDigits + 2;

        // Require degree digits followed by exactly two whole-minute digits.
        for (size_t i = 0; i < wholeDigits; ++i)
        {
            if (value[i] < '0' || value[i] > '9')
                return std::nullopt;
        }

        // Any remaining characters must begin with a decimal point,
        // followed by at least one fractional digit.
        if (value.size() > wholeDigits)
        {
            if (value[wholeDigits] != '.' ||
                value.size() == wholeDigits + 1)
            {
                return std::nullopt;
            }
        }

        auto decimal_degrees = parseDecimal(value.substr(0, degreeDigits));
        auto decimal_minutes = parseDecimal(value.substr(degreeDigits));

        if (!decimal_degrees || !decimal_minutes)
            return std::nullopt;

        if (*decimal_minutes < 0 || *decimal_minutes >= 60)
            return std::nullopt;

        const double maxDegrees = isLatitude ? 90.0 : 180.0;

        if (*decimal_degrees < 0 ||
            *decimal_degrees > maxDegrees ||
            (*decimal_degrees == maxDegrees && *decimal_minutes != 0))
        {
            return std::nullopt;
        }

        if (isLatitude)
        {
            if (hemisphere != "N" && hemisphere != "S")
                return std::nullopt;
        }
        else
        {
            if (hemisphere != "E" && hemisphere != "W")
                return std::nullopt;
        }

        *decimal_degrees += *decimal_minutes / 60.0;

        if (hemisphere == "S" || hemisphere == "W")
        {
            *decimal_degrees = -(*decimal_degrees);
        }
        return decimal_degrees;
    }

    // Require meter units, parse both heights, then return MSL + geoid.
    std::optional<double> parseEllipsoidAltitude(
        std::string_view altitude,
        std::string_view altitudeUnits,
        std::string_view geoid,
        std::string_view geoidUnits)
    {
        if (geoidUnits != "M" || altitudeUnits != "M")
        {
            return std::nullopt;
        }

        auto decimal_alt = parseDecimal(altitude);
        auto decimal_geo = parseDecimal(geoid);

        if (!decimal_alt.has_value() || !decimal_geo.has_value())
        {
            return std::nullopt;
        }

        double ellipsoidAltitude = *decimal_alt + *decimal_geo;

        if (!std::isfinite(ellipsoidAltitude))
        {
            return std::nullopt;
        }

        return ellipsoidAltitude;
    }
}

std::optional<GPSPosition> parseGGA(std::string_view sentence)
{
    const auto payload = checkedPayload(sentence);

    if (!payload)
        return std::nullopt;

    const auto fields = splitGGAFields(*payload);

    if (!fields)
        return std::nullopt;

    const auto &f = *fields;

    // Accept GPS-only and combined-GNSS GGA messages.
    if (f[0] != "GPGGA" && f[0] != "GNGGA")
        return std::nullopt;

    // Initially accept GPS and differential GPS fixes.
    if (f[6] != "1" && f[6] != "2")
        return std::nullopt;

    const auto latitude = parseCoordinate(f[2], f[3], true);
    const auto longitude = parseCoordinate(f[4], f[5], false);

    const auto altitude = parseEllipsoidAltitude(
        f[9], f[10], f[11], f[12]);

    if (!latitude || !longitude || !altitude)
        return std::nullopt;

    return GPSPosition{*latitude, *longitude, *altitude};
}
