// Unit tests for the antenna pointing algorithm.
// Reference WGS84 values were computed independently using pyproj
// (EPSG:4979 -> EPSG:4978 for ECEF, and a WGS84 topocentric transform for ENU).
// No third-party C++ test framework is required.

#include "coordinate_utils.hpp"
#include "pointing.hpp"
#include "test_helpers.hpp"

using test::expectNear;
using test::expectTrue;

#include <cmath>
#include <iostream>
#include <limits>
#include <string>

namespace
{
    constexpr double ANGLE_TOLERANCE = 0.001; // degrees

    void expectNoAngles(const std::string &name, const GPSPosition &rocket,
                        const GPSPosition &station)
    {
        auto result = calculatePointingAngles(rocket, station);
        expectTrue(name + " returns nullopt", !result.has_value());
    }

    void expectAngles(const std::string &name, const GPSPosition &rocket,
                      const GPSPosition &station,
                      double expectedAzimuth, double expectedElevation)
    {
        auto result = calculatePointingAngles(rocket, station);
        expectTrue(name + " returns angles", result.has_value());
        if (!result)
            return;

        expectTrue(name + " azimuth is in [0, 360)",
                   std::isfinite(result->azimuth) &&
                       result->azimuth >= 0.0 && result->azimuth < 360.0);

        // Account for 359.999... degrees being near 0 degrees.
        double azimuthError = std::remainder(
            result->azimuth - expectedAzimuth, 360.0);
        expectNear(name + " azimuth (degrees)",
                   azimuthError, 0.0, ANGLE_TOLERANCE);
        expectNear(name + " elevation (degrees)",
                   result->elevation, expectedElevation, ANGLE_TOLERANCE);
    }

    // ---------------- Invalid and undefined inputs ----------------
    void testInvalidPointing()
    {
        GPSPosition station = {0.0, 0.0, 0.0};
        GPSPosition rocket = {0.01, 0.0, 1000.0};
        const double nan = std::numeric_limits<double>::quiet_NaN();
        const double inf = std::numeric_limits<double>::infinity();

        expectNoAngles("Latitude above 90", {91.0, 0.0, 100.0}, station);
        expectNoAngles("Latitude below -90", {-91.0, 0.0, 100.0}, station);
        expectNoAngles("Longitude above 180", {0.0, 181.0, 100.0}, station);
        expectNoAngles("Longitude below -180", {0.0, -181.0, 100.0}, station);
        expectNoAngles("NaN latitude", {nan, 0.0, 100.0}, station);
        expectNoAngles("NaN longitude", {0.0, nan, 100.0}, station);
        expectNoAngles("NaN altitude", {0.0, 0.0, nan}, station);
        expectNoAngles("Infinite altitude", {0.0, 0.0, inf}, station);
        expectNoAngles("Invalid station latitude", rocket, {100.0, 0.0, 0.0});
        expectNoAngles("NaN station longitude", rocket, {0.0, nan, 0.0});
        expectNoAngles("Identical positions", station, station);
        expectNoAngles("Only 2 m overhead (near coincident)",
                       {0.0, 0.0, 2.0}, station);
        expectNoAngles("Directly overhead", {0.0, 0.0, 1000.0}, station);
        expectNoAngles("Overhead with <5 m horizontal offset",
                       {0.0, 0.00001, 1000.0}, station);
    }

    // ---------------- Pointing angles ----------------
    void testPointingAngles()
    {
        const GPSPosition equatorStation = {0.0, 0.0, 0.0};
        // Equal ellipsoidal heights at different locations produce a
        // slightly negative elevation due to Earth curvature.
        expectAngles("North", {0.01, 0.0, 0.0}, equatorStation,
                     0.0, -0.005);
        expectAngles("East", {0.0, 0.01, 0.0}, equatorStation,
                     90.0, -0.005);
        expectAngles("South", {-0.01, 0.0, 0.0}, equatorStation,
                     180.0, -0.005);
        expectAngles("West", {0.0, -0.01, 0.0}, equatorStation,
                     270.0, -0.005);
        expectAngles("East and 1000 m higher", {0.0, 0.01, 1000.0},
                     equatorStation, 90.0, 41.926605);
        // Negative ellipsoidal altitude is valid and can point below horizon.
        expectTrue("Negative altitude can produce valid angles",
                   calculatePointingAngles({0.0, 0.01, -100.0},
                                           equatorStation)
                       .has_value());

        const GPSPosition station = {37.22, -80.42, 600.0};
        expectAngles("Original rocket example", {37.23, -80.42, 1000.0},
                     station, 0.0, 19.812840);
        expectAngles("Northwest and higher", {37.24, -80.44, 2500.0},
                     station, 321.359738, 33.745297);
        expectAngles("Azimuth near 360 wraparound",
                     {37.23, -80.4200001, 1000.0}, station,
                     359.999542, 19.812840);
        expectAngles("Southern hemisphere",
                     {-33.85, 151.23, 1700.0},
                     {-33.865143, 151.209900, 42.0},
                     47.922615, 33.472300);
        expectAngles("Date line crossing", {0.0, -179.999, 500.0},
                     {0.0, 179.999, 0.0}, 90.0, 65.995809);
        expectTrue("More than 5 m horizontal offset returns angles",
                   calculatePointingAngles({0.0, 0.001, 1000.0}, equatorStation).has_value());
    }
}

int main()
{
    testInvalidPointing();
    testPointingAngles();

    return test::summary();
}
