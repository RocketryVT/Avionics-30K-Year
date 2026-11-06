// Unit tests for the motor interface
// No third-party C++ test framework is required.

#include "test_helpers.hpp"
#include "motor_interface.hpp"

using test::expectEquals;

#include <cmath>
#include <limits>
#include <string>

namespace
{
    void expectValidCommand(const std::string &name,
                            const PointingAngles &angles,
                            uint16_t id,
                            int abs_az,
                            int abs_el)
    {
        const auto result = makeMotorTarget(angles, id);

        test::expectTrue(name + " returns a command", result.has_value());

        if (result)
        {
            expectEquals(name + " azimuth (mdeg)", result->az_mdeg, abs_az);
            expectEquals(name + " elevation (mdeg)", result->el_mdeg, abs_el);
            expectEquals(name + " preserves command ID", result->id, id);
            test::expectTrue(name + " uses TARGET verb", result->verb == Verb::Target);
            test::expectTrue(name + " has no command error", result->err == ParseError::None);
        }
    }

    void expectInvalidCommand(const std::string &name,
                              const PointingAngles &angles,
                              uint16_t id = 42)
    {
        const auto result = makeMotorTarget(angles, id);

        test::expectTrue(name + " returns nullopt", !result.has_value());
    }

    void testAngleConversion()
    {
        expectValidCommand("North", {0, 0}, 42, 0, 0);
        expectValidCommand("East", {90, 15}, 42, 90000, 15000);
        expectValidCommand("South maps to negative seam", {180, 15}, 42, -180000, 15000);
        expectValidCommand("West", {270, 15}, 42, -90000, 15000);
        expectValidCommand("359 degrees", {359, -15}, 42, -1000, -15000);
        expectValidCommand("Before south seam", {179.999, 0}, 42, 179999, 0);
        expectValidCommand("After south seam", {180.001, 0}, 42, -179999, 0);
        expectValidCommand("One millidegree west of north", {359.999, 0}, 42, -1, 0);
    }

    void testRounding()
    {
        expectValidCommand("Round down", {12.3454, 23.4564}, 42, 12345, 23456);
        expectValidCommand("Round up", {12.3456, 23.4566}, 42, 12346, 23457);
        expectValidCommand("Negative elevation toward zero", {0, -12.3454}, 42, 0, -12345);
        expectValidCommand("Negative elevation away from zero", {0, -12.3456}, 42, 0, -12346);
        expectValidCommand("Wrapped azimuth rounding", {270.1234, 0}, 42, -89877, 0);
        // 0.0625 is exactly representable in binary, so these are exact
        // half-millidegree ties rather than approximate decimal inputs.
        expectValidCommand("Positive half rounds away from zero", {0.0625, 0.0625}, 42, 63, 63);
        expectValidCommand("Negative elevation half rounds away from zero", {0, -0.0625}, 42, 0, -63);
    }

    void testValidBoundaries()
    {
        expectValidCommand("Minimum elevation", {0, -90}, 42, 0, -90000);
        expectValidCommand("Maximum elevation", {0, 90}, 42, 0, 90000);
        expectValidCommand("Just below 360 rounds to north",
                           {std::nextafter(360.0, 0.0), 0}, 42, 0, 0);
        expectValidCommand("Zero command ID", {45, 10}, 0, 45000, 10000);
        expectValidCommand("Maximum command ID", {45, 10},
                           std::numeric_limits<uint16_t>::max(), 45000, 10000);
    }

    void testInvalidAngles()
    {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        const double inf = std::numeric_limits<double>::infinity();

        expectInvalidCommand("Negative azimuth", {-1, 0});
        expectInvalidCommand("Azimuth just below zero", {std::nextafter(0.0, -1.0), 0});
        expectInvalidCommand("Azimuth exactly 360", {360, 0});
        expectInvalidCommand("Azimuth above 360", {361, 0});
        expectInvalidCommand("Elevation below minimum", {0, -91});
        expectInvalidCommand("Elevation just below minimum", {0, std::nextafter(-90.0, -inf)});
        expectInvalidCommand("Elevation above maximum", {0, 91});
        expectInvalidCommand("Elevation just above maximum", {0, std::nextafter(90.0, inf)});
        expectInvalidCommand("NaN azimuth", {nan, 0});
        expectInvalidCommand("Positive infinite azimuth", {inf, 0});
        expectInvalidCommand("Negative infinite azimuth", {-inf, 0});
        expectInvalidCommand("NaN elevation", {0, nan});
        expectInvalidCommand("Positive infinite elevation", {0, inf});
        expectInvalidCommand("Negative infinite elevation", {0, -inf});
    }

    void testIntegration()
    {
        const GPSPosition station{0.0, 0.0, 0.0};
        GPSPosition rocket{0.0, 1.0, 0.0};

        // At the equator, equal-altitude positions one degree apart have
        // an elevation of -0.5 degrees because of Earth's curvature.
        auto angles = calculatePointingAngles(rocket, station);
        test::expectTrue("East GPS position returns pointing angles", angles.has_value());
        if (angles)
        {
            expectValidCommand("East GPS position to motor target", *angles, 0,
                               90000, -500);
        }

        // Pointing returns 270 degrees west; the adapter must express it
        // as -90 degrees for the motor controller.
        rocket.longitude = -1.0;
        angles = calculatePointingAngles(rocket, station);
        test::expectTrue("West GPS position returns pointing angles", angles.has_value());
        if (angles)
        {
            expectValidCommand("West GPS position to motor target", *angles, 42,
                               -90000, -500);
        }
    }
}

int main()
{
    testAngleConversion();
    testRounding();
    testValidBoundaries();
    testInvalidAngles();
    testIntegration();

    return test::summary();
}
