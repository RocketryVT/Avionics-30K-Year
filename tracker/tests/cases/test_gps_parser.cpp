// Unit tests for the gps parser.
// No third-party C++ test framework is required.

#include "test_helpers.hpp"
#include "gps_parser.hpp"

using test::expectNear;

#include <array>
#include <string>

namespace
{
    void expectValidPosition(
        const std::string &name,
        const std::string_view sentence,
        double lat,
        double lon,
        double alt,
        double tolerance)
    {
        const auto result = parseGGA(sentence);
        test::expectTrue(name + " returns a GPSPosition", result.has_value());
        if (result)
        {
            expectNear(name + " latitude", result->latitude, lat, tolerance);
            expectNear(name + " longitude", result->longitude, lon, tolerance);
            expectNear(name + " ellipsoidal altitude", result->altitude, alt, tolerance);
        }
    }

    void expectInvalidPosition(
        const std::string &name,
        const std::string_view sentence)
    {
        const auto result = parseGGA(sentence);

        test::expectTrue(name + " returns nullopt", !result.has_value());
    }

    constexpr double POSITION_TOLERANCE = 1e-8;
    constexpr double CAPTURED_LATITUDE = 37.22350016666667;
    constexpr double CAPTURED_LONGITUDE = -80.44218933333333;
    constexpr std::string_view CAPTURED_BODY =
        "GNGGA,152653.50,3713.41001,N,08026.53136,W,2,12,0.62,648.3,M,-33.4,M,,";
    constexpr std::string_view CAPTURED_SENTENCE =
        "$GNGGA,152653.50,3713.41001,N,08026.53136,W,2,12,0.62,648.3,M,-33.4,M,,*7B";

    // Synthetic cases receive a correct checksum, so failures exercise the
    // changed field rather than being rejected by the checksum first.
    std::string makeSentence(std::string_view body)
    {
        unsigned checksum = 0;
        for (unsigned char c : body)
            checksum ^= c;

        constexpr char HEX[] = "0123456789ABCDEF";
        std::string sentence = "$";
        sentence.append(body);
        sentence += '*';
        sentence += HEX[checksum >> 4];
        sentence += HEX[checksum & 0x0F];
        return sentence;
    }

    std::string withField(size_t field, std::string_view value)
    {
        std::array<std::string_view, 15> fields = {
            "GNGGA", "152653.50", "3713.41001", "N", "08026.53136",
            "W", "2", "12", "0.62", "648.3", "M", "-33.4", "M", "", ""};
        fields[field] = value;

        std::string body;
        for (size_t i = 0; i < fields.size(); ++i)
        {
            if (i != 0)
                body += ',';
            body.append(fields[i]);
        }
        return makeSentence(body);
    }

    void testCapturedFixes()
    {
        // These literal checksums came from the actual module capture,
        // independently of the synthetic fixture builder above.
        expectValidPosition("Captured fix at .50", CAPTURED_SENTENCE,
                            CAPTURED_LATITUDE, CAPTURED_LONGITUDE, 614.9, POSITION_TOLERANCE);
        expectValidPosition("Captured fix at .60",
                            "$GNGGA,152653.60,3713.41001,N,08026.53136,W,2,12,0.60,648.3,M,-33.4,M,,*7A",
                            CAPTURED_LATITUDE, CAPTURED_LONGITUDE, 614.9, POSITION_TOLERANCE);
        expectValidPosition("Captured fix at .70",
                            "$GNGGA,152653.70,3713.41001,N,08026.53137,W,2,12,0.60,648.3,M,-33.4,M,,*7A",
                            CAPTURED_LATITUDE, -80.4421895, 614.9, POSITION_TOLERANCE);
        expectValidPosition("Captured sentence with CRLF", std::string(CAPTURED_SENTENCE) + "\r\n",
                            CAPTURED_LATITUDE, CAPTURED_LONGITUDE, 614.9, POSITION_TOLERANCE);
        expectValidPosition("Captured sentence with LF", std::string(CAPTURED_SENTENCE) + "\n",
                            CAPTURED_LATITUDE, CAPTURED_LONGITUDE, 614.9, POSITION_TOLERANCE);
        expectValidPosition("Lowercase checksum digits", std::string(CAPTURED_SENTENCE.substr(0, CAPTURED_SENTENCE.size() - 1)) + "b",
                            CAPTURED_LATITUDE, CAPTURED_LONGITUDE, 614.9, POSITION_TOLERANCE);
    }

    void testCoordinateAndAltitudeConversion()
    {
        expectValidPosition("Standard GP reference with positive geoid",
                            "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47",
                            48.1173, 11.516666666666667, 592.3, POSITION_TOLERANCE);
        expectValidPosition("Southern latitude", withField(3, "S"),
                            -CAPTURED_LATITUDE, CAPTURED_LONGITUDE, 614.9, POSITION_TOLERANCE);
        expectValidPosition("Eastern longitude", withField(5, "E"),
                            CAPTURED_LATITUDE, -CAPTURED_LONGITUDE, 614.9, POSITION_TOLERANCE);
        expectValidPosition("Ordinary GPS fix quality", withField(6, "1"),
                            CAPTURED_LATITUDE, CAPTURED_LONGITUDE, 614.9, POSITION_TOLERANCE);
        expectValidPosition("Latitude exactly 90 degrees", withField(2, "9000.000"),
                            90, CAPTURED_LONGITUDE, 614.9, POSITION_TOLERANCE);
        expectValidPosition("Longitude exactly 180 degrees", withField(4, "18000.000"),
                            CAPTURED_LATITUDE, -180, 614.9, POSITION_TOLERANCE);
        expectValidPosition("Whole-minute latitude", withField(2, "3713"),
                            37.21666666666667, CAPTURED_LONGITUDE, 614.9, POSITION_TOLERANCE);
        expectValidPosition("Whole-minute longitude", withField(4, "08026"),
                            CAPTURED_LATITUDE, -80.43333333333334, 614.9, POSITION_TOLERANCE);
        expectValidPosition("Zero altitude MSL", withField(9, "0"),
                            CAPTURED_LATITUDE, CAPTURED_LONGITUDE, -33.4, POSITION_TOLERANCE);
        expectValidPosition("Negative altitude MSL", withField(9, "-50.0"),
                            CAPTURED_LATITUDE, CAPTURED_LONGITUDE, -83.4, POSITION_TOLERANCE);
        expectValidPosition("Zero geoid separation", withField(11, "0"),
                            CAPTURED_LATITUDE, CAPTURED_LONGITUDE, 648.3, POSITION_TOLERANCE);
        expectValidPosition("Positive geoid separation", withField(11, "33.4"),
                            CAPTURED_LATITUDE, CAPTURED_LONGITUDE, 681.7, POSITION_TOLERANCE);
        expectValidPosition("Optional differential age is empty", withField(14, "0042"),
                            CAPTURED_LATITUDE, CAPTURED_LONGITUDE, 614.9, POSITION_TOLERANCE);
    }

    void testInvalidEnvelope()
    {
        expectInvalidPosition("Empty input", "");
        expectInvalidPosition("Truncated input", "$GNGGA");
        expectInvalidPosition("Missing start marker", CAPTURED_SENTENCE.substr(1));
        expectInvalidPosition("Missing checksum", "$" + std::string(CAPTURED_BODY));
        expectInvalidPosition("One checksum digit", std::string(CAPTURED_SENTENCE.substr(0, CAPTURED_SENTENCE.size() - 1)));
        expectInvalidPosition("Three checksum digits", std::string(CAPTURED_SENTENCE) + "0");
        expectInvalidPosition("Leading noise", "noise" + std::string(CAPTURED_SENTENCE));
        expectInvalidPosition("Trailing garbage", std::string(CAPTURED_SENTENCE) + "junk");

        std::string changed(CAPTURED_SENTENCE);
        changed[changed.find("648.3")] = '7';
        expectInvalidPosition("Data changed without recomputing checksum", changed);
        changed = CAPTURED_SENTENCE;
        changed[changed.size() - 2] = 'Z';
        expectInvalidPosition("Invalid high checksum digit", changed);
        changed = CAPTURED_SENTENCE;
        changed.back() = 'Z';
        expectInvalidPosition("Invalid low checksum digit", changed);
    }

    void testInvalidFields()
    {
        struct InvalidFieldCase
        {
            const char *name;
            size_t field;
            const char *value;
        };

        const InvalidFieldCase cases[] = {
            {"Unsupported message type", 0, "GNRMC"},
            {"Unsupported talker", 0, "ZZGGA"},
            {"No fix", 6, "0"},
            {"Missing fix quality", 6, ""},
            {"Dead-reckoning fix is not accepted", 6, "6"},
            {"RTK is outside current accepted-quality policy", 6, "4"},
            {"Malformed fix quality", 6, "2x"},
            {"Missing latitude", 2, ""},
            {"Missing longitude", 4, ""},
            {"Latitude too short", 2, "371"},
            {"Longitude too short", 4, "0802"},
            {"Latitude decimal point too early", 2, "37.123"},
            {"Longitude decimal point too early", 4, "080.123"},
            {"Extra latitude whole-minute digit", 2, "37001.0"},
            {"Extra longitude whole-minute digit", 4, "080001.0"},
            {"Latitude missing fractional digits", 2, "3713."},
            {"Longitude missing fractional digits", 4, "08026."},
            {"Latitude minutes equal 60", 2, "3760.000"},
            {"Longitude minutes equal 60", 4, "08060.000"},
            {"Latitude exceeds 90 degrees", 2, "9100.000"},
            {"Longitude exceeds 180 degrees", 4, "18100.000"},
            {"Nonzero minutes at 90 degrees", 2, "9000.001"},
            {"Nonzero minutes at 180 degrees", 4, "18000.001"},
            {"Latitude has trailing garbage", 2, "3713.41001x"},
            {"Longitude has trailing garbage", 4, "08026.53136x"},
            {"Signed raw latitude", 2, "-3713.41001"},
            {"Missing latitude hemisphere", 3, ""},
            {"Missing longitude hemisphere", 5, ""},
            {"Longitude hemisphere used for latitude", 3, "W"},
            {"Latitude hemisphere used for longitude", 5, "N"},
            {"Multiple hemisphere characters", 3, "NS"},
            {"Missing MSL altitude", 9, ""},
            {"Missing geoid separation", 11, ""},
            {"Altitude units are not meters", 10, "F"},
            {"Geoid units are not meters", 12, "F"},
            {"Missing altitude units", 10, ""},
            {"Missing geoid units", 12, ""},
            {"Altitude has trailing garbage", 9, "648.3junk"},
            {"Geoid has trailing garbage", 11, "-33.4junk"},
            {"Altitude in scientific notation", 9, "1e3"},
            {"Altitude has leading whitespace", 9, " 648.3"},
            {"Altitude has trailing whitespace", 9, "648.3 "},
            {"Altitude is NaN", 9, "nan"},
            {"Altitude is positive infinity", 9, "inf"},
            {"Altitude is negative infinity", 9, "-inf"},
            {"Geoid is NaN", 11, "nan"},
            {"Geoid is infinite", 11, "inf"},
        };

        for (const auto &item : cases)
            expectInvalidPosition(item.name, withField(item.field, item.value));

        expectInvalidPosition("Too few fields", makeSentence(CAPTURED_BODY.substr(0, CAPTURED_BODY.size() - 1)));
        expectInvalidPosition("Extra empty field", makeSentence(std::string(CAPTURED_BODY) + ","));
        expectInvalidPosition("Extra populated field", makeSentence(std::string(CAPTURED_BODY) + ",extra"));

        // Both terms can be finite doubles while their sum overflows.
        const std::string largeNumber = "1" + std::string(308, '0');
        expectInvalidPosition("Altitude outside double range", withField(9, "1" + std::string(400, '0')));
        expectInvalidPosition("Ellipsoidal altitude sum overflows",
                              makeSentence("GNGGA,152653.50,3713.41001,N,08026.53136,W,2,12,0.62," +
                                           largeNumber + ",M," + largeNumber + ",M,,"));
    }
}

int main()
{
    testCapturedFixes();
    testCoordinateAndAltitudeConversion();
    testInvalidEnvelope();
    testInvalidFields();

    return test::summary();
}
