#include "test_helpers.hpp"

#include <cmath>
#include <iostream>

namespace test
{
    namespace
    {
        int checks = 0;
        int failures = 0;
    }

    void expectTrue(const std::string &name, bool condition)
    {
        ++checks;

        if (condition)
        {
            std::cout << "[PASS] " << name << '\n';
        }
        else
        {
            ++failures;
            std::cout << "[FAIL] " << name << '\n';
        }
    }

    void expectNear(const std::string &name, double actual,
                    double expected, double tolerance)
    {
        ++checks;

        if (std::isfinite(actual) &&
            std::abs(actual - expected) <= tolerance)
        {
            std::cout << "[PASS] " << name << '\n';
        }
        else
        {
            ++failures;
            std::cout << "[FAIL] " << name
                      << ": expected " << expected
                      << ", got " << actual << '\n';
        }
    }

    int summary()
    {
        std::cout << '\n'
                  << (checks - failures) << '/' << checks
                  << " checks passed.\n";

        return failures == 0 ? 0 : 1;
    }
}
