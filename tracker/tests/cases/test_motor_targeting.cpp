// Unit tests for the motor targeting
// No third-party C++ test framework is required.

#include "test_helpers.hpp"
#include "motor_targeting.hpp"

using test::expectNear;

#include <cmath>
#include <iostream>
#include <string>

namespace
{
    constexpr float VELOCITY_TOLERANCE = 0.01;

    void expectAngularVelocity(const std::string &name,
                               const PointingAngles &current,
                               const PointingAngles &previous,
                               float elapsedTime,
                               double azimuth,
                               double elevation)
    {
        const auto result = calcAngularVelocity(current, previous, elapsedTime);

        test::expectTrue("Valid samples return velocity", result.has_value());

        if (result)
        {
            expectNear(name + " Azimuth", result->azimuth, azimuth, VELOCITY_TOLERANCE);
            expectNear(name + " Elevation", result->elevation, elevation, VELOCITY_TOLERANCE);
        }
    }

    void expectInvalidVelocity(const std::string &name,
                               const PointingAngles &current,
                               const PointingAngles &previous,
                               float elapsedTime)
    {
        const auto result = calcAngularVelocity(current, previous, elapsedTime);

        test::expectTrue("Valid samples return nullopt", !result.has_value());
    }

    void testAngularVelocity()
    {
        expectAngularVelocity("AZ 40 → 42, EL 20 → 21",
                              {42, 21}, {40, 20}, 0.2, 10, 5);
        expectAngularVelocity("AZ 42 → 40, EL 21 → 20",
                              {40, 20}, {42, 21}, 0.2, -10, -5);
        expectAngularVelocity("AZ 359 → 1",
                              {1, 20}, {359, 20}, 0.2, 10, 0);
        expectAngularVelocity("AZ 1 → 359",
                              {359, 20}, {1, 20}, 0.2, -10, 0);
        expectAngularVelocity("Both angles unchanged",
                              {40, 20}, {40, 20}, 0.2, 0, 0);
        expectAngularVelocity("AZ unchanged, EL 20 → 19",
                              {40, 19}, {40, 20}, 0.2, 0, -5);
    }
    void testInvalidAngularVelocity()
    {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        const double inf = std::numeric_limits<double>::infinity();
        const float nanTime = std::numeric_limits<float>::quiet_NaN();
        const float infTime = std::numeric_limits<float>::infinity();

        expectInvalidVelocity("Zero elapsed time",
                              {42, 21}, {40, 20}, 0.0f);

        expectInvalidVelocity("Negative elapsed time",
                              {42, 21}, {40, 20}, -0.2f);

        expectInvalidVelocity("NaN elapsed time",
                              {42, 21}, {40, 20}, nanTime);
        expectInvalidVelocity("Infinite elapsed time",
                              {42, 21}, {40, 20}, infTime);

        expectInvalidVelocity("Infinite current azimuth",
                              {inf, 21}, {40, 20}, 0.2f);
        expectInvalidVelocity("NaN current azimuth",
                              {nan, 21}, {40, 20}, 0.2f);

        expectInvalidVelocity("Infinite current elevation",
                              {42, inf}, {40, 20}, 0.2f);
        expectInvalidVelocity("NaN current elevation",
                              {42, nan}, {40, 20}, 0.2f);

        expectInvalidVelocity("Infinite previous azimuth",
                              {42, 21}, {inf, 20}, 0.2f);
        expectInvalidVelocity("NaN previous azimuth",
                              {42, 21}, {nan, 20}, 0.2f);

        expectInvalidVelocity("Infinite previous elevation",
                              {42, 21}, {40, inf}, 0.2f);
        expectInvalidVelocity("NaN previous elevation",
                              {42, 21}, {40, nan}, 0.2f);
    }
}

int main()
{
    testAngularVelocity();
    testInvalidAngularVelocity();

    return test::summary();
}