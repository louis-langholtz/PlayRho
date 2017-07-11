/*
 * Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

#include "gtest/gtest.h"
#include <PlayRho/Common/Fixed.hpp>
#include <PlayRho/Common/Math.hpp>

using namespace box2d;

TEST(Fixed32, ByteSizeIs4)
{
    EXPECT_EQ(sizeof(Fixed32), std::size_t(4));
}

#ifndef _WIN32
TEST(Fixed64, ByteSizeIs8)
{
    EXPECT_EQ(sizeof(Fixed64), std::size_t(8));
}
#endif

// Tests of Fixed<T>::GetFromUnsignedInt(v)

#define DECL_GET_FROM_UNSIGNED_INT_TEST(type) \
TEST(type, GetFromUnsignedInt) \
{ \
    EXPECT_EQ(type::GetFromUnsignedInt(0u),  0 * type::ScaleFactor); \
    EXPECT_EQ(type::GetFromUnsignedInt(1u),  1 * type::ScaleFactor); \
    EXPECT_EQ(type::GetFromUnsignedInt(2u),  2 * type::ScaleFactor); \
}

DECL_GET_FROM_UNSIGNED_INT_TEST(Fixed32)
#ifndef _WIN32
DECL_GET_FROM_UNSIGNED_INT_TEST(Fixed64)
#endif

// Tests of Fixed<T>::GetFromSignedInt(v)

#define DECL_GET_FROM_SIGNED_INT_TEST(type) \
TEST(type, GetFromSignedInt) \
{ \
    EXPECT_EQ(type::GetFromSignedInt( 0),  0 * type::ScaleFactor); \
    EXPECT_EQ(type::GetFromSignedInt( 1),  1 * type::ScaleFactor); \
    EXPECT_EQ(type::GetFromSignedInt( 2),  2 * type::ScaleFactor); \
    EXPECT_EQ(type::GetFromSignedInt(-1), -1 * type::ScaleFactor); \
    EXPECT_EQ(type::GetFromSignedInt(-2), -2 * type::ScaleFactor); \
}

DECL_GET_FROM_SIGNED_INT_TEST(Fixed32)
#ifndef _WIN32
DECL_GET_FROM_SIGNED_INT_TEST(Fixed64)
#endif

// Tests of Fixed<T>::GetFromFloat(v)

#define DECL_GET_FROM_FLOAT_TEST(type) \
TEST(type, GetFromFloat) \
{ \
    EXPECT_EQ(type::GetFromFloat( 0.0),  0 * type::ScaleFactor); \
    EXPECT_EQ(type::GetFromFloat( 1.0),  1 * type::ScaleFactor); \
    EXPECT_EQ(type::GetFromFloat( 2.0),  2 * type::ScaleFactor); \
    EXPECT_EQ(type::GetFromFloat(-1.0), -1 * type::ScaleFactor); \
    EXPECT_EQ(type::GetFromFloat(-2.0), -2 * type::ScaleFactor); \
    EXPECT_EQ(type::GetFromFloat(-4.7), static_cast<type::value_type>(-4.7 * type::ScaleFactor)); \
    const auto long_double_max = std::numeric_limits<long double>::max(); \
    const auto fixed_infinity = type::GetInfinity(); \
    const auto fixed_infinity_as_vt = *reinterpret_cast<const type::value_type*>(&fixed_infinity); \
    EXPECT_EQ(type::GetFromFloat( long_double_max),  fixed_infinity_as_vt); \
    EXPECT_EQ(type::GetFromFloat(-long_double_max), -fixed_infinity_as_vt); \
    EXPECT_EQ(type::GetFromFloat( std::numeric_limits<float>::infinity()),  fixed_infinity_as_vt); \
    EXPECT_EQ(type::GetFromFloat(-std::numeric_limits<float>::infinity()), -fixed_infinity_as_vt); \
}

DECL_GET_FROM_FLOAT_TEST(Fixed32)
#ifndef _WIN32
DECL_GET_FROM_FLOAT_TEST(Fixed64)
#endif

// Tests of Fixed<T>::Fixed(v) and comparisons

#define DECL_INT_CONSTRUCTION_AND_COMPARE_TEST(type) \
TEST(type, IntConstructionAndCompare) \
{ \
    EXPECT_EQ(type( 0), type( 0)); \
    EXPECT_LT(type( 0), type( 1)); \
    EXPECT_GT(type( 0), type(-1)); \
    EXPECT_EQ(type(-10), type(-10)); \
    EXPECT_LT(type(-10), type( -9)); \
    EXPECT_GT(type(-10), type(-11)); \
}

DECL_INT_CONSTRUCTION_AND_COMPARE_TEST(Fixed32)
#ifndef _WIN32
DECL_INT_CONSTRUCTION_AND_COMPARE_TEST(Fixed64)
#endif

// Tests of std::isfinite(Fixed<T>)

#define DECL_ISFINITE_TEST(type) \
TEST(type, isfinite) \
{ \
    EXPECT_TRUE(std::isfinite(type(0))); \
    EXPECT_FALSE(std::isfinite( type::GetInfinity())); \
    EXPECT_FALSE(std::isfinite(-type::GetInfinity())); \
    EXPECT_FALSE(std::isfinite(type::GetNaN())); \
}

DECL_ISFINITE_TEST(Fixed32)
#ifndef _WIN32
DECL_ISFINITE_TEST(Fixed64)
#endif

// Tests of std::isnan(Fixed<T>)

#define DECL_ISNAN_TEST(type) \
TEST(type, isnan) \
{ \
    EXPECT_FALSE(std::isnan(type( 0))); \
    EXPECT_FALSE(std::isnan(type( 1))); \
    EXPECT_FALSE(std::isnan(type(-1))); \
    EXPECT_FALSE(std::isnan( type::GetInfinity())); \
    EXPECT_FALSE(std::isnan(-type::GetInfinity())); \
    EXPECT_FALSE(std::isnan( type::GetNegativeInfinity())); \
    EXPECT_TRUE(std::isnan(type::GetNaN())); \
    EXPECT_TRUE(std::isnan(type(std::numeric_limits<float>::quiet_NaN()))); \
    EXPECT_TRUE(std::isnan(type(std::numeric_limits<float>::signaling_NaN()))); \
    EXPECT_TRUE(std::isnan(type(std::numeric_limits<double>::quiet_NaN()))); \
    EXPECT_TRUE(std::isnan(type(std::numeric_limits<double>::signaling_NaN()))); \
    EXPECT_TRUE(std::isnan(type(std::numeric_limits<long double>::quiet_NaN()))); \
    EXPECT_TRUE(std::isnan(type(std::numeric_limits<long double>::signaling_NaN()))); \
}

DECL_ISNAN_TEST(Fixed32)
#ifndef _WIN32
DECL_ISNAN_TEST(Fixed64)
#endif

// Regular tests

TEST(Fixed32, IntCast)
{
    EXPECT_EQ(static_cast<int>(Fixed32( 0)),  0);
    EXPECT_EQ(static_cast<int>(Fixed32(-1)), -1);
    EXPECT_EQ(static_cast<int>(Fixed32(-2)), -2);
    EXPECT_EQ(static_cast<int>(Fixed32(+1)), +1);
    EXPECT_EQ(static_cast<int>(Fixed32(+2)), +2);
}

#ifndef _WIN32
TEST(Fixed64, IntCast)
{
    EXPECT_EQ(static_cast<int>(Fixed64( 0)),  0);
    EXPECT_EQ(static_cast<int>(Fixed64(-1)), -1);
    EXPECT_EQ(static_cast<int>(Fixed64(-2)), -2);
    EXPECT_EQ(static_cast<int>(Fixed64(+1)), +1);
    EXPECT_EQ(static_cast<int>(Fixed64(+2)), +2);
}
#endif

TEST(Fixed32, FloatCast)
{
    EXPECT_EQ(static_cast<float>(Fixed32( 0)),  0.0f);
    EXPECT_EQ(static_cast<float>(Fixed32(-1)), -1.0f);
    EXPECT_EQ(static_cast<float>(Fixed32(+1)), +1.0f);
}

#ifndef _WIN32
TEST(Fixed64, FloatCast)
{
    EXPECT_EQ(static_cast<float>(Fixed64( 0)),  0.0f);
    EXPECT_EQ(static_cast<float>(Fixed64(-1)), -1.0f);
    EXPECT_EQ(static_cast<float>(Fixed64(+1)), +1.0f);
}
#endif

TEST(Fixed32, DoubleCast)
{
    EXPECT_EQ(static_cast<double>(Fixed32( 0)),  0.0);
    EXPECT_EQ(static_cast<double>(Fixed32(-1)), -1.0);
    EXPECT_EQ(static_cast<double>(Fixed32(+1)), +1.0);
}

#ifndef _WIN32
TEST(Fixed64, DoubleCast)
{
    EXPECT_EQ(static_cast<double>(Fixed64( 0)),  0.0);
    EXPECT_EQ(static_cast<double>(Fixed64(-1)), -1.0);
    EXPECT_EQ(static_cast<double>(Fixed64(+1)), +1.0);
}
#endif

TEST(Fixed32, FloatConstruction)
{
    EXPECT_EQ(Fixed32(0.0), 0.0);
    EXPECT_EQ(Fixed32(-1.0), -1.0);
    EXPECT_EQ(Fixed32(+1.0), +1.0);

    EXPECT_EQ(Fixed32(std::numeric_limits<float>::infinity()), Fixed32::GetInfinity());
    EXPECT_EQ(Fixed32(-std::numeric_limits<float>::infinity()), -Fixed32::GetInfinity());
    EXPECT_EQ(Fixed32(-std::numeric_limits<float>::infinity()), Fixed32::GetNegativeInfinity());
    EXPECT_TRUE(std::isnan(Fixed32(std::numeric_limits<float>::quiet_NaN())));
    EXPECT_TRUE(std::isnan(Fixed32(std::numeric_limits<float>::signaling_NaN())));

    const auto range = 30000;
    for (auto i = -range; i < range; ++i)
    {
        EXPECT_EQ(Fixed32(static_cast<float>(i)), i);
        EXPECT_EQ(Fixed32(float(i)), i);
        EXPECT_EQ(Fixed32(float(i)), Fixed32(i));
    }
}

TEST(Fixed32, Equals)
{
    EXPECT_TRUE(Fixed32(12) == Fixed32(12.0f));
    EXPECT_FALSE(std::numeric_limits<Fixed32>::quiet_NaN() == std::numeric_limits<Fixed32>::quiet_NaN());
}

TEST(Fixed32, NotEquals)
{
    EXPECT_TRUE(Fixed32(-302) != Fixed32(12.0f));
    EXPECT_FALSE(Fixed32(-302) != Fixed32(-302));
    EXPECT_TRUE(std::numeric_limits<Fixed32>::quiet_NaN() != std::numeric_limits<Fixed32>::quiet_NaN());
}

TEST(Fixed32, LessThan)
{
    EXPECT_TRUE(Fixed32(-302) < Fixed32(12.0f));
    EXPECT_TRUE(Fixed32(40) < Fixed32(44));
    EXPECT_FALSE(Fixed32(76) < Fixed32(31));
    EXPECT_TRUE(Fixed32(0.001) < Fixed32(0.002));
    EXPECT_TRUE(Fixed32(0.000) < Fixed32(0.01));
}

TEST(Fixed32, GreaterThan)
{
    EXPECT_FALSE(Fixed32(-302) > Fixed32(12.0f));
    EXPECT_FALSE(Fixed32(40) > Fixed32(44));
    EXPECT_TRUE(Fixed32(76) > Fixed32(31));
}

TEST(Fixed32, Addition)
{
    for (auto val = 0; val < 100; ++val)
    {
        Fixed32 a{val};
        Fixed32 b{val};
        EXPECT_EQ(a + b, Fixed32(val * 2));
    }
}

TEST(Fixed32, InfinityPlusValidIsInfinity)
{
    EXPECT_EQ(Fixed32::GetInfinity() + 0, Fixed32::GetInfinity());
    EXPECT_EQ(Fixed32::GetInfinity() + 1, Fixed32::GetInfinity());
    EXPECT_EQ(Fixed32::GetInfinity() + 100, Fixed32::GetInfinity());
    EXPECT_EQ(Fixed32::GetInfinity() + -1, Fixed32::GetInfinity());
    EXPECT_EQ(Fixed32::GetInfinity() + -100, Fixed32::GetInfinity());
    EXPECT_EQ(Fixed32::GetInfinity() + Fixed32::GetInfinity(), Fixed32::GetInfinity());
}

TEST(Fixed32, EqualSubtraction)
{
    for (auto val = 0; val < 100; ++val)
    {
        Fixed32 a{val};
        Fixed32 b{val};
        EXPECT_EQ(a - b, Fixed32(0));
    }
}

TEST(Fixed32, OppositeSubtraction)
{
    for (auto val = 0; val < 100; ++val)
    {
        Fixed32 a{-val};
        Fixed32 b{val};
        EXPECT_EQ(a - b, Fixed32(val * -2));
    }
}

TEST(Fixed32, Multiplication)
{
    for (auto val = 0; val < 100; ++val)
    {
        Fixed32 a{val};
        EXPECT_EQ(a * a, Fixed32(val * val));
    }
    EXPECT_EQ(Fixed32(9) * Fixed32(3), Fixed32(27));
    EXPECT_EQ(Fixed32(-5) * Fixed32(-4), Fixed32(20));
    EXPECT_EQ(Fixed32(0.5) * Fixed32(0.5), Fixed32(0.25));
    EXPECT_EQ(round(Fixed32(-0.05) * Fixed32(0.05), 1000), round(Fixed32(-0.0025), 1000));
    EXPECT_EQ(round(Fixed32(Pi) * 2, 100), round(Fixed32(Pi * 2), 100));
    EXPECT_EQ(Fixed32(181) * Fixed32(181), Fixed32(32761));
}

TEST(Fixed32, Division)
{
    for (auto val = 1; val < 100; ++val)
    {
        Fixed32 a{val};
        EXPECT_EQ(a / a, Fixed32(1));
    }
    EXPECT_EQ(Fixed32(9) / Fixed32(3), Fixed32(3));
    EXPECT_EQ(Fixed32(81) / Fixed32(9), Fixed32(9));
    EXPECT_EQ(Fixed32(-10) / Fixed32(2), Fixed32(-5));
    EXPECT_EQ(Fixed32(1) / Fixed32(2), Fixed32(0.5));
    EXPECT_EQ(Fixed32(7) / Fixed32(3), Fixed32(7.0/3.0));

    // Confirm int divided by Fixed32 gets promoted to Fixed32 divided by Fixed32
    EXPECT_EQ(1 / Fixed32(2), Fixed32(0.5));
    EXPECT_EQ(2 / Fixed32(2), Fixed32(1));
    EXPECT_EQ(3 / Fixed32(2), Fixed32(1.5));
}

TEST(Fixed32, Sin)
{
    EXPECT_FLOAT_EQ(std::sin(Fixed32(0)), 0.0f);
    EXPECT_FLOAT_EQ(std::sin(Fixed32(1)), std::sin(1.0f));
    EXPECT_FLOAT_EQ(std::sin(Fixed32(2)), std::sin(2.0f));
    EXPECT_FLOAT_EQ(std::sin(Fixed32(Pi/2)), 1.0f);
}

TEST(Fixed32, Cos)
{
    EXPECT_FLOAT_EQ(std::cos(Fixed32(0)), float(1));
    EXPECT_FLOAT_EQ(std::cos(Fixed32(1)), std::cos(1.0f));
    EXPECT_FLOAT_EQ(std::cos(Fixed32(2)), std::cos(2.0f));
    EXPECT_LT(std::cos(Fixed32(Pi/2)), +0.001f);
    EXPECT_GT(std::cos(Fixed32(Pi/2)), -0.001f);
}

TEST(Fixed32, Max)
{
    const auto max_internal_val = std::numeric_limits<int32_t>::max() - 1;
    const auto max_fixed32 = *reinterpret_cast<const Fixed32*>(&max_internal_val);
    
    EXPECT_EQ(Fixed32::GetMax(), Fixed32::GetMax());
    EXPECT_EQ(Fixed32::GetMax(), max_fixed32);
    //EXPECT_EQ(static_cast<long double>(Fixed32::GetMax()), 131071.99993896484375000000L);
    //std::cout << std::setprecision(22) << static_cast<long double>(Fixed32::GetMax()) << std::endl;
    switch (Fixed32::FractionBits)
    {
        case  9:
            EXPECT_NEAR(static_cast<long double>(Fixed32::GetMax()), 4.1943e+06, 4.0);
            break;
        case 14:
            EXPECT_EQ(static_cast<long double>(Fixed32::GetMax()), 131071.9998779296875L);
            break;
    }

    EXPECT_GT(Fixed32::GetMax(), Fixed32(0));
    EXPECT_GT(Fixed32::GetMax(), Fixed32::GetMin());
    EXPECT_GT(Fixed32::GetMax(), Fixed32::GetLowest());
    EXPECT_GT(Fixed32::GetMax(), Fixed32((1 << (31u - Fixed32::FractionBits)) - 1));
}

TEST(Fixed32, Min)
{
    EXPECT_EQ(Fixed32::GetMin(), Fixed32::GetMin());
    EXPECT_EQ(Fixed32::GetMin(), Fixed32(0, 1));
    switch (Fixed32::FractionBits)
    {
        case  9:
            EXPECT_NEAR(static_cast<long double>(Fixed32::GetMin()), 0.00195312, 0.0000001);
            break;
        case 14:
            EXPECT_EQ(static_cast<long double>(Fixed32::GetMin()), 0.00006103515625000000L);
            break;
    }

    EXPECT_LT(Fixed32::GetMin(), Fixed32::GetMax());
    
    EXPECT_GT(Fixed32::GetMin(), Fixed32(0));
    EXPECT_GT(Fixed32::GetMin(), Fixed32::GetLowest());
}

TEST(Fixed32, Lowest)
{
    const auto lowest_internal_val = std::numeric_limits<int32_t>::min() + 2;
    const auto lowest_fixed32 = *reinterpret_cast<const Fixed32*>(&lowest_internal_val);

    EXPECT_EQ(Fixed32::GetLowest(), Fixed32::GetLowest());
    EXPECT_EQ(Fixed32::GetLowest(), lowest_fixed32);
    //EXPECT_EQ(static_cast<long double>(Fixed32::GetLowest()), -131072.00000000000000000000L);
    //std::cout << std::setprecision(22) << static_cast<long double>(Fixed32::GetLowest()) << std::endl;
    switch (Fixed32::FractionBits)
    {
        case  9:
            EXPECT_NEAR(static_cast<long double>(Fixed32::GetLowest()), -4.1943e+06, 4.0);
            break;
        case 14:
            EXPECT_EQ(static_cast<long double>(Fixed32::GetLowest()), -131071.9998779296875L);
            break;
    }
    EXPECT_LT(Fixed32::GetLowest(), Fixed32(0));
    EXPECT_LT(Fixed32::GetLowest(), Fixed32(-((1 << (31u - Fixed32::FractionBits)) - 1), 0u));
    EXPECT_LT(Fixed32::GetLowest(), Fixed32(-((1 << (31u - Fixed32::FractionBits)) - 1), (1u << Fixed32::FractionBits) - 1u));
    EXPECT_EQ(Fixed32::GetLowest(), -Fixed32::GetMax());
}

TEST(Fixed32, SubtractingFromLowestGetsNegativeInfinity)
{
    EXPECT_EQ(Fixed32::GetLowest() - Fixed32::GetMin(), Fixed32::GetNegativeInfinity());
    EXPECT_EQ(Fixed32::GetLowest() - 1, Fixed32::GetNegativeInfinity());
}

TEST(Fixed32, AddingToMaxGetsInfinity)
{
    EXPECT_EQ(Fixed32::GetMax() + Fixed32::GetMin(), Fixed32::GetInfinity());
    EXPECT_EQ(Fixed32::GetMax() + 1, Fixed32::GetInfinity());
}

TEST(Fixed32, MinusInfinityEqualsNegativeInfinity)
{
    EXPECT_EQ(-Fixed32::GetInfinity(), Fixed32::GetNegativeInfinity());
}

TEST(Fixed32, InfinityEqualsMinusNegativeInfinity)
{
    EXPECT_EQ(Fixed32::GetInfinity(), -Fixed32::GetNegativeInfinity());
}

TEST(Fixed32, InifnityTimesPositiveIsInfinity)
{
    EXPECT_EQ(Fixed32::GetInfinity() * 1, Fixed32::GetInfinity());
    EXPECT_EQ(Fixed32::GetInfinity() * 2, Fixed32::GetInfinity());
    EXPECT_EQ(Fixed32::GetInfinity() * 0.5, Fixed32::GetInfinity());
}

TEST(Fixed32, InifnityDividedByPositiveIsInfinity)
{
    EXPECT_EQ(Fixed32::GetInfinity() / 1, Fixed32::GetInfinity());
    EXPECT_EQ(Fixed32::GetInfinity() / 2, Fixed32::GetInfinity());
    EXPECT_EQ(Fixed32::GetInfinity() / 0.5, Fixed32::GetInfinity());
}

TEST(Fixed32, InfinityDividedByInfinityIsNaN)
{
    EXPECT_TRUE(std::isnan(Fixed32::GetInfinity() / Fixed32::GetInfinity()));
}

TEST(Fixed32, InifnityTimesNegativeIsNegativeInfinity)
{
    EXPECT_EQ(Fixed32::GetInfinity() * -1, -Fixed32::GetInfinity());
    EXPECT_EQ(Fixed32::GetInfinity() * -2, -Fixed32::GetInfinity());
    EXPECT_EQ(Fixed32::GetInfinity() * -0.5, -Fixed32::GetInfinity());
}

TEST(Fixed32, InifnityDividedByNegativeIsNegativeInfinity)
{
    EXPECT_EQ(Fixed32::GetInfinity() / -1, -Fixed32::GetInfinity());
    EXPECT_EQ(Fixed32::GetInfinity() / -2, -Fixed32::GetInfinity());
    EXPECT_EQ(Fixed32::GetInfinity() / -0.5, -Fixed32::GetInfinity());
}

TEST(Fixed32, InfinityMinusNegativeInfinityIsInfinity)
{
    EXPECT_EQ(Fixed32::GetInfinity() - -Fixed32::GetInfinity(), Fixed32::GetInfinity());
}

TEST(Fixed32, NegativeInfinityMinusInfinityIsNegativeInfinity)
{
    EXPECT_EQ(-Fixed32::GetInfinity() - Fixed32::GetInfinity(), -Fixed32::GetInfinity());
}

TEST(Fixed32, NaN)
{
    EXPECT_TRUE(std::isnan(Fixed32::GetNaN()));
    EXPECT_TRUE(std::isnan(Fixed32::GetInfinity() / Fixed32::GetInfinity()));
    EXPECT_TRUE(std::isnan(Fixed32::GetInfinity() - Fixed32::GetInfinity()));
    EXPECT_TRUE(std::isnan(-Fixed32::GetInfinity() - -Fixed32::GetInfinity()));
    EXPECT_TRUE(std::isnan(-Fixed32::GetInfinity() + Fixed32::GetInfinity()));

    EXPECT_FALSE(std::isnan(Fixed32{0}));
    EXPECT_FALSE(std::isnan(Fixed32{10.0f}));
    EXPECT_FALSE(std::isnan(Fixed32{-10.0f}));
    EXPECT_FALSE(std::isnan(Fixed32::GetInfinity()));
    EXPECT_FALSE(std::isnan(Fixed32::GetNegativeInfinity()));
    EXPECT_FALSE(std::isnan(Fixed32::GetMax()));
    EXPECT_FALSE(std::isnan(Fixed32::GetMin()));
    EXPECT_FALSE(std::isnan(Fixed32::GetLowest()));
}

TEST(Fixed32, InfinityTimesZeroIsNaN)
{
    EXPECT_TRUE(std::isnan(Fixed32::GetInfinity() * 0));
}

TEST(Fixed32, Comparators)
{
    EXPECT_FALSE(Fixed32::GetNaN() > 0.0f);
    EXPECT_FALSE(Fixed32::GetNaN() < 0.0f);
    EXPECT_FALSE(Fixed32::GetNaN() == 0.0f);
    EXPECT_TRUE(Fixed32::GetNaN() != 0.0f);
    EXPECT_FALSE(Fixed32::GetNaN() == Fixed32::GetNaN());
}

TEST(Fixed32, BiggerValsIdenticallyInaccurate)
{
    // Check that Real doesn't suffer from inconstent inaccuracy (like float has depending on
    // the float's value).
    auto last_delta = float(0);
    auto val = Fixed32{1};
    const auto max = sizeof(Fixed32) * 8 - Fixed32::FractionBits - 1;
    for (auto i = decltype(max){0}; i < max; ++i)
    {
        const auto next = std::nextafter(val, std::numeric_limits<Fixed32>::max());
        const auto delta = static_cast<float>(next - val);
        ASSERT_EQ(val + (delta / 2.0f), val);
#if 0
        std::cout << std::hexfloat;
        std::cout << "For " << std::setw(7) << val << ", delta of next value is " << std::setw(7) << delta;
        std::cout << std::defaultfloat;
        std::cout << ": ie. at " << std::setw(6) << val;
        std::cout << std::fixed;
        std::cout << ", delta is " << delta;
        std::cout << std::endl;
#endif
        val *= 2;
        if (last_delta != 0)
        {
            ASSERT_EQ(delta, last_delta);
        }
        last_delta = delta;
    }
}

TEST(Fixed, Int32TypeAnd0bits)
{
    using fixed = Fixed<std::int32_t, 0>;
    
    const auto zero = fixed(0);
    EXPECT_TRUE(zero == zero);
    EXPECT_EQ(zero, zero);

    const auto one = fixed(1);
    EXPECT_TRUE(one == one);
    EXPECT_EQ(one, one);

    EXPECT_NE(one, zero);
    EXPECT_NE(zero, one);
    EXPECT_GT(one, zero);
    EXPECT_GE(one, zero);
    EXPECT_GE(one, one);
    EXPECT_LT(zero, one);
    EXPECT_LE(zero, one);
    
    const auto two = one + one;
    EXPECT_NE(one, two);
    EXPECT_GT(two, one);
    EXPECT_GT(two, zero);
    
    EXPECT_EQ(one * one, one);
    EXPECT_EQ(one * two, two);
    EXPECT_EQ(two / two, one);
    EXPECT_EQ(two - two, zero);
}
