#pragma once

#include <sstream>
#include <string>

namespace test
{
    /**
     * @brief Records one Boolean assertion and prints its pass/fail result.
     * @param name Label identifying the check in the test output.
     * @param condition True for a passing check, false for a failure.
     */
    void expectTrue(const std::string &name, bool condition);

    /**
     * @brief Records whether two values compare equal and prints mismatched values.
     *
     * @tparam Actual Type supporting comparison with Expected and stream output.
     * @tparam Expected Type supporting comparison with Actual and stream output.
     * @param name Label identifying the check in the test output.
     * @param actual Value produced by the code under test.
     * @param expected Reference value required by the test.
     *
     * @note Updates the shared counters through expectTrue(). The template is
     *       defined in this header so each test can instantiate its needed types.
     *       Floating-point calculations generally require expectNear() instead.
     */
    template <typename Actual, typename Expected>
    void expectEquals(
        const std::string &name,
        const Actual &actual,
        const Expected &expected)
    {
        if (actual == expected)
        {
            expectTrue(name, true);
        }
        else
        {
            std::ostringstream message;

            message << name
                    << ": expected " << expected
                    << ", got " << actual;

            expectTrue(message.str(), false);
        }
    }

    /**
     * @brief Records whether a finite value is within an absolute error tolerance.
     * @param name Label identifying the check in the test output.
     * @param actual Calculated value, which must be finite to pass.
     * @param expected Reference value required by the test.
     * @param tolerance Maximum allowed absolute difference, in the values' units.
     * @pre The caller supplies a finite, non-negative tolerance.
     */
    void expectNear(const std::string &name, double actual,
                    double expected, double tolerance);

    /**
     * @brief Prints the accumulated passing and total assertion counts.
     * @return Zero if every recorded check passed, otherwise one.
     * @note The counters are not reset. The return value can be used by main().
     */
    int summary();
}
