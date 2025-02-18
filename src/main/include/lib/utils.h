//from T-nor
#pragma once

namespace utils
{
    static const double kEpsilon = 1e-9;

    inline bool epsilonEquals(double a, double b, double epsilon)
    {
        return ((a - epsilon <= b) and (a + epsilon >= b));
    }

    inline bool epsilonEquals(double a, double b)
    {
        return epsilonEquals(a, b, kEpsilon);
    }

    inline double limit(double value, double low, double high)
    {
        return (value < low) ? low : ((value > high) ? high : value);
    }
}