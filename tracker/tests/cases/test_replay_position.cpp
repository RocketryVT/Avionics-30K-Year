// Desktop tests for positions interpolated from the generated simulation table.

#include "replay_position.hpp"
#include "test_helpers.hpp"

#include <cmath>
#include <limits>
#include <string>

namespace
{
    constexpr double COORDINATE_TOLERANCE_DEG = 1e-12;
    constexpr double ALTITUDE_TOLERANCE_M = 1e-8;
    constexpr double FIRST_TIME_S = 0.0;
    constexpr double LAST_TIME_S = 156.029625211;

    void expectValidReplayPosition(
        const std::string &name,
        double timeSeconds,
        double geoidSeparationMeters,
        const GPSPosition &expected)
    {
        const auto result = replayPositionAt(timeSeconds, geoidSeparationMeters);
        test::expectTrue(name + " returns a position", result.has_value());

        if (result)
        {
            test::expectNear(name + " latitude", result->latitude,
                             expected.latitude, COORDINATE_TOLERANCE_DEG);
            test::expectNear(name + " longitude", result->longitude,
                             expected.longitude, COORDINATE_TOLERANCE_DEG);
            test::expectNear(name + " ellipsoidal altitude", result->altitude,
                             expected.altitude, ALTITUDE_TOLERANCE_M);
        }
    }

    void expectInvalidReplayPosition(
        const std::string &name,
        double timeSeconds,
        double geoidSeparationMeters)
    {
        test::expectTrue(name + " returns nullopt",
                         !replayPositionAt(timeSeconds, geoidSeparationMeters));
    }

    void testExactSamples()
    {
        // Zero geoid separation is a synthetic fixture, not a site correction.
        expectValidReplayPosition("First timestamp", FIRST_TIME_S, 0.0,
                                  {28.61, -80.6, 0.0});
        expectValidReplayPosition("Second timestamp", 0.011, 0.0,
                                  {28.61, -80.6, 0.0009228440268});
        expectValidReplayPosition("Exact half-second sample", 0.5, 0.0,
                                  {28.61, -80.6000000785, 15.353698000309921});
        expectValidReplayPosition("Interior timestamp", 15.8061914063, 0.0,
                                  {28.6100003902, -80.5993538192, 1923.8101654299878});
        expectValidReplayPosition("Last timestamp", LAST_TIME_S, 0.0,
                                  {28.6100004101, -80.6019054024, 0.0});

        // Looking up a later time must not prevent restarting from the beginning.
        expectValidReplayPosition("Restart at the beginning", FIRST_TIME_S, 0.0,
                                  {28.61, -80.6, 0.0});
    }

    void testInterpolation()
    {
        expectValidReplayPosition("Halfway through the first interval", 0.0055, 0.0,
                                  {28.61, -80.6, 0.0004614220134});

        // Between 15.7561914063 and 15.8061914063 seconds, all three fields change.
        // These expected values are fixed reference calculations for that pair.
        expectValidReplayPosition("Interior midpoint", 15.7811914063, 0.0,
                                  {28.6100003893, -80.59935455695, 1923.0560587381127});
        expectValidReplayPosition("Interior quarter interval", 15.7686914063, 0.0,
                                  {28.61000038885, -80.599354925825, 1922.6790053921752});

        // The final interval has a different duration and descending altitude.
        expectValidReplayPosition("Midpoint before touchdown", 155.7897621695, 0.0,
                                  {28.61000041025, -80.6019007743, 1.0273906808203202});
    }

    void testGeoidCorrection()
    {
        // Synthetic signed corrections verify h_ellipsoid = H_MSL + N_geoid.
        expectValidReplayPosition("Positive geoid at launch", FIRST_TIME_S, 33.4,
                                  {28.61, -80.6, 33.4});
        expectValidReplayPosition("Negative geoid at launch", FIRST_TIME_S, -33.4,
                                  {28.61, -80.6, -33.4});
        expectValidReplayPosition("Positive geoid after interpolation", 15.7811914063, 33.4,
                                  {28.6100003893, -80.59935455695, 1956.4560587381127});
        expectValidReplayPosition("Negative geoid after interpolation", 15.7811914063, -33.4,
                                  {28.6100003893, -80.59935455695, 1889.6560587381127});
        expectValidReplayPosition("Geoid correction at touchdown", LAST_TIME_S, -33.4,
                                  {28.6100004101, -80.6019054024, -33.4});
    }

    void testInvalidInputs()
    {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        const double infinity = std::numeric_limits<double>::infinity();

        expectInvalidReplayPosition("Time before launch", -1.0, 0.0);
        expectInvalidReplayPosition("Time just before the first sample",
                                    std::nextafter(FIRST_TIME_S, -infinity), 0.0);
        expectInvalidReplayPosition("Time after touchdown", LAST_TIME_S + 1.0, 0.0);
        expectInvalidReplayPosition("Time just after the last sample",
                                    std::nextafter(LAST_TIME_S, infinity), 0.0);
        expectInvalidReplayPosition("NaN time", nan, 0.0);
        expectInvalidReplayPosition("Positive infinite time", infinity, 0.0);
        expectInvalidReplayPosition("Negative infinite time", -infinity, 0.0);
        expectInvalidReplayPosition("NaN geoid separation", 1.0, nan);
        expectInvalidReplayPosition("Positive infinite geoid separation", 1.0, infinity);
        expectInvalidReplayPosition("Negative infinite geoid separation", 1.0, -infinity);
    }
}

int main()
{
    testExactSamples();
    testInterpolation();
    testGeoidCorrection();
    testInvalidInputs();

    return test::summary();
}
