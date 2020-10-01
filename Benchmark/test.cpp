#include <algorithm>
#include <boost/units/systems/si/length.hpp>
#include <boost/units/cmath.hpp>

using length = boost::units::quantity<boost::units::si::length, float>;

bool less(float a, float b)
{
    return a < b;
}

bool less_equal(float a, float b)
{
    return a <= b;
}

float min(float a, float b)
{
    return a < b? a: b;
}

float min2(float a, float b)
{
    return a <= b? a: b;
}

float min_std(float a, float b)
{
    return std::min(a, b);
}

bool less(length a, length b)
{
    return a < b;
}

bool less_equal(length a, length b)
{
    return a <= b;
}

length min(length a, length b)
{
    return a < b? a: b;
}

length min2(length a, length b)
{
    return a <= b? a: b;
}

length min_std(length a, length b)
{
    return std::min(a, b);
}

