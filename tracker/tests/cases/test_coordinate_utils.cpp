// Unit tests for the coordinate conversions.
// Reference WGS84 values were computed independently using pyproj
// (EPSG:4979 -> EPSG:4978 for ECEF, and a WGS84 topocentric transform for ENU).
// No third-party C++ test framework is required.

#include "coordinate_utils.hpp"
#include "test_helpers.hpp"

using test::expectNear;

#include <cmath>
#include <iostream>
#include <string>

namespace
{
    constexpr double ECEF_TOLERANCE = 0.01; // 1 cm
    constexpr double ENU_TOLERANCE = 0.01;  // 1 cm

    void expectECEF(const std::string &name, const GPSPosition &gps,
                    double x, double y, double z)
    {
        ECEFPosition result = gpsToECEF(gps);
        expectNear(name + " X", result.x, x, ECEF_TOLERANCE);
        expectNear(name + " Y", result.y, y, ECEF_TOLERANCE);
        expectNear(name + " Z", result.z, z, ECEF_TOLERANCE);
    }

    void expectENU(const std::string &name, const ENUPosition &actual,
                   double east, double north, double up,
                   double tolerance = ENU_TOLERANCE)
    {
        expectNear(name + " East", actual.east, east, tolerance);
        expectNear(name + " North", actual.north, north, tolerance);
        expectNear(name + " Up", actual.up, up, tolerance);
    }

    // ---------------- GPS -> ECEF ----------------
    void testECEF()
    {
        expectECEF("Equator, prime meridian", {0.0, 0.0, 0.0},
                   6378137.0, 0.0, 0.0);
        expectECEF("Equator, 90 deg east", {0.0, 90.0, 0.0},
                   0.0, 6378137.0, 0.0);
        expectECEF("Equator, altitude 1000 m", {0.0, 0.0, 1000.0},
                   6379137.0, 0.0, 0.0);
        expectECEF("Equator, altitude -100 m", {0.0, 0.0, -100.0},
                   6378037.0, 0.0, 0.0);
        expectECEF("North pole", {90.0, 0.0, 0.0},
                   0.0, 0.0, 6356752.314245);
        expectECEF("45 deg latitude/longitude, 1000 m", {45.0, 45.0, 1000.0},
                   3194919.145061, 3194919.145061, 4488055.515647);
        expectECEF("Southern hemisphere", {-33.865143, 151.209900, 42.0},
                   -4646306.677663, 2553283.348999, -3534058.980110);
    }

    // ---------------- ECEF -> ENU ----------------
    void testENU()
    {
        GPSPosition stationGPS = {0.0, 0.0, 0.0};
        ECEFPosition station = gpsToECEF(stationGPS);

        // Identical ECEF coordinates must produce zero displacement.
        expectENU("Identical positions",
                  ecefToENU(station, station, stationGPS),
                  0.0, 0.0, 0.0, 1e-6);

        // At lat=0, lon=0: +X is Up, +Y is East, +Z is North.
        ECEFPosition rocket = {station.x + 30.0, 100.0, 200.0};
        expectENU("Known East/North/Up axes",
                  ecefToENU(rocket, station, stationGPS),
                  100.0, 200.0, 30.0, 1e-6);

        // Real geographic positions, compared to independent WGS84 references.
        GPSPosition stationGPS2 = {37.22, -80.42, 600.0};
        GPSPosition rocketGPS2 = {37.23, -80.42, 1000.0};
        ECEFPosition station2 = gpsToECEF(stationGPS2);
        ECEFPosition rocket2 = gpsToECEF(rocketGPS2);
        ENUPosition actual = ecefToENU(rocket2, station2, stationGPS2);
        expectENU("Blacksburg-area example", actual,
                  0.0, 1109.993167, 399.903135);

        // ENU is a rotation: distance must be preserved.
        double ecefDistance = std::hypot(
            std::hypot(rocket2.x - station2.x, rocket2.y - station2.y),
            rocket2.z - station2.z);
        double enuDistance = std::hypot(
            std::hypot(actual.east, actual.north), actual.up);
        expectNear("ECEF and ENU lengths match", enuDistance,
                   ecefDistance, 1e-6);

        GPSPosition dateLineStation = {0.0, 179.999, 0.0};
        GPSPosition dateLineRocket = {0.0, -179.999, 500.0};
        expectENU("International date line",
                  ecefToENU(gpsToECEF(dateLineRocket),
                            gpsToECEF(dateLineStation), dateLineStation),
                  222.656435, 0.0, 499.996114);
    }
}

int main()
{
    testECEF();
    testENU();

    return test::summary();
}