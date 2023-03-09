/*
 * Copyright (c) 2023 Louis Langholtz https://github.com/louis-langholtz/PlayRho
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

#include "UnitTests.hpp"

#include <PlayRho/Common/TypeInfo.hpp> // for GetTypeName

#include <cmath>
#include <algorithm>
#include <limits>
#include <iostream>
#include <cstring>
#include <type_traits>

TEST(float, BiggerValsIncreasinglyInaccurate)
{
    // This test is meant to demonstrate the increasing inaccuracy of the float type and help
    // recognize the problems that using this type can cause. Note that the double suffers the
    // same way except more slowly. This increasing inaccuracy is inherent to how floating point
    // types are designed.
    //
    // A way to avoid this problem, is to use fixed-point calculations (instead of floating-point
    // calculations).
    
    auto last_delta = float(0);
    auto val = float(1);
    for (auto i = 0; i < 24; ++i)
    {
        const auto next = std::nextafter(val, std::numeric_limits<float>::max());
        const auto delta = next - val;
        ASSERT_EQ(val + (delta / 2), val);
        
        // For 0x1p+00, delta of next value is 0x1p-23: ie. at      1, delta is 0.0000001192092895508
        // For 0x1p+01, delta of next value is 0x1p-22: ie. at      2, delta is 0.0000002384185791016
        // For 0x1p+02, delta of next value is 0x1p-21: ie. at      4, delta is 0.0000004768371582031
        // For 0x1p+03, delta of next value is 0x1p-20: ie. at      8, delta is 0.0000009536743164062
        // For 0x1p+04, delta of next value is 0x1p-19: ie. at     16, delta is 0.0000019073486328125
        // For 0x1p+05, delta of next value is 0x1p-18: ie. at     32, delta is 0.0000038146972656250
        // For 0x1p+06, delta of next value is 0x1p-17: ie. at     64, delta is 0.0000076293945312500
        // For 0x1p+07, delta of next value is 0x1p-16: ie. at    128, delta is 0.0000152587890625000
        // For 0x1p+08, delta of next value is 0x1p-15: ie. at    256, delta is 0.0000305175781250000
        // For 0x1p+09, delta of next value is 0x1p-14: ie. at    512, delta is 0.0000610351562500000
        // For 0x1p+10, delta of next value is 0x1p-13: ie. at   1024, delta is 0.0001220703125000000
        // For 0x1p+11, delta of next value is 0x1p-12: ie. at   2048, delta is 0.0002441406250000000
        // For 0x1p+12, delta of next value is 0x1p-11: ie. at   4096, delta is 0.0004882812500000000
        // For 0x1p+13, delta of next value is 0x1p-10: ie. at   8192, delta is 0.0009765625000000000
        // For 0x1p+14, delta of next value is 0x1p-09: ie. at  16384, delta is 0.0019531250000000000
        // For 0x1p+15, delta of next value is 0x1p-08: ie. at  32768, delta is 0.0039062500000000000
        // For 0x1p+16, delta of next value is 0x1p-07: ie. at  65536, delta is 0.0078125000000000000
        // For 0x1p+17, delta of next value is 0x1p-06: ie. at 131072, delta is 0.0156250000000000000
        // For 0x1p+18, delta of next value is 0x1p-05: ie. at 262144, delta is 0.0312500000000000000
        // For 0x1p+19, delta of next value is 0x1p-04: ie. at 524288, delta is 0.0625000000000000000
        //
        // If a floating-point type is used in the implementation of the simulation then, these
        // deltas mean that:
        // - The farther bodies get out from the origin (0, 0) the less accurately they can be moved.
        // - The larger shape vertex radiuses get, the less accurately time of impact can be
        //   calculated for those shapes.
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
        EXPECT_GT(delta, last_delta);
        last_delta = delta;
    }
}

TEST(float, max)
{
    // Use pragmas to quiet MS Visual Studio re: "overflow in constant arithmetic".
#pragma warning( push )
#pragma warning( disable: 4756 )
    EXPECT_EQ(std::numeric_limits<float>::max() * 2, std::numeric_limits<float>::infinity());
    EXPECT_EQ(std::numeric_limits<float>::max() + std::numeric_limits<float>::max(), std::numeric_limits<float>::infinity());
    
    // Compared to float-max, 1 is insignificant.
    // So adding 1 to float-max effectively results in adding 0 to float-max.
    EXPECT_EQ(std::numeric_limits<float>::max() + 1, std::numeric_limits<float>::max());

    EXPECT_LT(std::numeric_limits<float>::max() / 2, std::numeric_limits<float>::max());
    EXPECT_GT(std::numeric_limits<float>::max() / 2, 0.0f);

    EXPECT_EQ(std::numeric_limits<float>::max() + std::numeric_limits<float>::max() / 2, std::numeric_limits<float>::infinity());

    EXPECT_NEAR(          std::numeric_limits<float>::max(),  3.4028234663852886e+38, 0.0);
    EXPECT_NEAR(std::sqrt(std::numeric_limits<float>::max()), 1.8446742974197924e+19, 0.0);
    EXPECT_LT(  std::sqrt(std::numeric_limits<float>::max()), std::numeric_limits<float>::max());
    EXPECT_GT(            std::numeric_limits<float>::max(),  std::sqrt(std::numeric_limits<float>::max()));
#pragma warning( pop )
}

TEST(float, infinity)
{
    ASSERT_TRUE(std::numeric_limits<float>::has_infinity);
    EXPECT_TRUE(std::numeric_limits<float>::is_iec559);

    EXPECT_EQ(std::numeric_limits<float>::infinity() * 2, std::numeric_limits<float>::infinity());
    EXPECT_EQ(std::numeric_limits<float>::infinity() * 0.5f, std::numeric_limits<float>::infinity());
    EXPECT_EQ(std::numeric_limits<float>::infinity() * -1, -std::numeric_limits<float>::infinity());
    EXPECT_EQ(std::numeric_limits<float>::infinity() * -0.5f, -std::numeric_limits<float>::infinity());
    EXPECT_EQ(std::numeric_limits<float>::infinity() * std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity());
    EXPECT_EQ(std::numeric_limits<float>::infinity() * -std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity());

    EXPECT_EQ(std::numeric_limits<float>::infinity() / 2, std::numeric_limits<float>::infinity());
    EXPECT_EQ(std::numeric_limits<float>::infinity() / 0.5f, std::numeric_limits<float>::infinity());
    EXPECT_EQ(std::numeric_limits<float>::infinity() / -1, -std::numeric_limits<float>::infinity());
    EXPECT_EQ(std::numeric_limits<float>::infinity() / -0.5f, -std::numeric_limits<float>::infinity());

    EXPECT_EQ(std::numeric_limits<float>::infinity() - 0, std::numeric_limits<float>::infinity());
    EXPECT_EQ(std::numeric_limits<float>::infinity() - 1000, std::numeric_limits<float>::infinity());
    EXPECT_EQ(std::numeric_limits<float>::infinity() - -std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity());
    EXPECT_EQ(0 - std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity());
    EXPECT_EQ(0.5 - std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity());
    EXPECT_EQ(1000 - std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity());

    EXPECT_EQ(0.0f / std::numeric_limits<float>::infinity(), 0.0f);
    EXPECT_EQ(1.0f / std::numeric_limits<float>::infinity(), 0.0f);
    EXPECT_EQ(-1.0f / std::numeric_limits<float>::infinity(), 0.0f);

    EXPECT_TRUE(std::numeric_limits<float>::has_quiet_NaN);

#ifndef __FAST_MATH__
    EXPECT_TRUE(std::isnan(std::numeric_limits<float>::infinity() * 0));
    EXPECT_TRUE(std::isnan(std::numeric_limits<float>::infinity() / std::numeric_limits<float>::infinity()));
    EXPECT_FALSE((std::numeric_limits<float>::infinity() * 0) == 0.0f);
    EXPECT_FALSE((std::numeric_limits<float>::infinity() / std::numeric_limits<float>::infinity()) == 1.0f);
#else
    EXPECT_FALSE(std::isnan(std::numeric_limits<float>::infinity() * 0));
    EXPECT_FALSE(std::isnan(std::numeric_limits<float>::infinity() / std::numeric_limits<float>::infinity()));
    EXPECT_TRUE((std::numeric_limits<float>::infinity() * 0) == 0.0f);
    EXPECT_TRUE((std::numeric_limits<float>::infinity() / std::numeric_limits<float>::infinity()) == 1.0f);
#endif
    EXPECT_FALSE(std::isnan(std::numeric_limits<float>::infinity()));
    EXPECT_FALSE(std::isnan(std::numeric_limits<float>::infinity() * 1.0f));
    EXPECT_FALSE(std::isnan(std::numeric_limits<float>::infinity() * 0.2f));
    EXPECT_TRUE (std::isnan(std::numeric_limits<float>::infinity() / -std::numeric_limits<float>::infinity()));
    EXPECT_TRUE (std::isnan(std::numeric_limits<float>::infinity() - std::numeric_limits<float>::infinity()));
    EXPECT_TRUE (std::isnan(-std::numeric_limits<float>::infinity() - -std::numeric_limits<float>::infinity()));
    
    {
        const auto foo = std::numeric_limits<float>::infinity();
        EXPECT_NE(foo - foo, 0.0f);
    }

    EXPECT_GT(+std::numeric_limits<float>::infinity(), 0.0f);
    EXPECT_LT(-std::numeric_limits<float>::infinity(), 0.0f);
    EXPECT_LT(0.0f, +std::numeric_limits<float>::infinity());
    EXPECT_GT(0.0f, -std::numeric_limits<float>::infinity());

    EXPECT_EQ(+std::numeric_limits<float>::infinity(), +std::numeric_limits<float>::infinity());
    EXPECT_EQ(-std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity());

    // Note: Behavior of casting float infinity to a type that doesn't have an infinity is
    //   undefined! Same is true for -infinity.
    // EXPECT_EQ(static_cast<int>(std::numeric_limits<float>::infinity()), 0);
    // EXPECT_EQ(static_cast<int>(-std::numeric_limits<float>::infinity()), 0);
    EXPECT_EQ(static_cast<double>(+std::numeric_limits<float>::infinity()), +std::numeric_limits<double>::infinity());
    EXPECT_EQ(static_cast<double>(-std::numeric_limits<float>::infinity()), -std::numeric_limits<double>::infinity());
}

TEST(float, AsFloat_7fe00000)
{
    const auto asUint = std::uint32_t{0x7fe00000};
    const auto asFloat = *reinterpret_cast<const float*>(&asUint);
    ASSERT_TRUE(std::isnan(asFloat));
#ifndef __FAST_MATH__
    EXPECT_FALSE(asFloat == asFloat);
    EXPECT_NE(asFloat, asFloat);
#else
    EXPECT_TRUE(asFloat == asFloat);
    EXPECT_EQ(asFloat, asFloat);
#endif
}

TEST(float, AsFloat_7fc00000)
{
    const auto asUint = std::uint32_t{0x7fc00000};
    const auto asFloat = *reinterpret_cast<const float*>(&asUint);
    ASSERT_TRUE(std::isnan(asFloat));
#ifndef __FAST_MATH__
    EXPECT_FALSE(asFloat == asFloat);
    EXPECT_NE(asFloat, asFloat);
#else
    EXPECT_TRUE(asFloat == asFloat);
    EXPECT_EQ(asFloat, asFloat);
#endif
}

TEST(float, AsFloat_7f800000)
{
    const auto asUint = std::uint32_t{0x7f800000};
    const auto asFloat = *reinterpret_cast<const float*>(&asUint);
    EXPECT_FALSE(std::isnan(asFloat));
    EXPECT_TRUE(std::isinf(asFloat));
    EXPECT_TRUE(asFloat == asFloat);
    EXPECT_EQ(asFloat, asFloat);
}

TEST(float, quiet_NaN)
{
    ASSERT_TRUE(std::numeric_limits<float>::has_quiet_NaN);
    ASSERT_TRUE(std::isnan(std::numeric_limits<float>::quiet_NaN()));

    EXPECT_TRUE(std::numeric_limits<float>::quiet_NaN() != std::numeric_limits<float>::quiet_NaN());
    const auto foo = std::numeric_limits<float>::quiet_NaN();
    //const auto asUint = *reinterpret_cast<const std::uint32_t *>(&foo);
    //std::cout << "quiet_NaN=" << std::hex << asUint << std::dec << "\n";
    EXPECT_FALSE(foo == foo);
    EXPECT_TRUE(foo != foo);
    EXPECT_NE(std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN());

#ifndef __FAST_MATH__
    EXPECT_TRUE(std::isnan(std::numeric_limits<float>::quiet_NaN() * 0));
    EXPECT_TRUE(std::isnan(0.0f / std::numeric_limits<float>::quiet_NaN()));
    EXPECT_FALSE(std::numeric_limits<float>::quiet_NaN() * 0 == 0.0f);
    EXPECT_FALSE(0.0f / std::numeric_limits<float>::quiet_NaN() == 0.0f);
#else
    EXPECT_FALSE(std::isnan(std::numeric_limits<float>::quiet_NaN() * 0));
    EXPECT_FALSE(std::isnan(0.0f / std::numeric_limits<float>::quiet_NaN()));
    EXPECT_TRUE(std::numeric_limits<float>::quiet_NaN() * 0 == 0.0f);
    EXPECT_TRUE(0.0f / std::numeric_limits<float>::quiet_NaN() == 0.0f);
#endif

    EXPECT_TRUE(std::isnan(std::numeric_limits<float>::quiet_NaN() * 1));
    EXPECT_TRUE(std::isnan(std::numeric_limits<float>::quiet_NaN() * std::numeric_limits<float>::infinity()));
    EXPECT_TRUE(std::isnan(std::numeric_limits<float>::quiet_NaN() / 1));
    EXPECT_TRUE(std::isnan(1.0f / std::numeric_limits<float>::quiet_NaN()));
    EXPECT_TRUE(std::isnan(std::numeric_limits<float>::quiet_NaN() / std::numeric_limits<float>::infinity()));
    EXPECT_FALSE(std::numeric_limits<float>::quiet_NaN() > 0.0f);
    EXPECT_FALSE(std::numeric_limits<float>::quiet_NaN() < 0.0f);
    EXPECT_TRUE(std::numeric_limits<float>::quiet_NaN() != std::numeric_limits<float>::quiet_NaN());
    EXPECT_NE(std::numeric_limits<float>::quiet_NaN(), 0.0f);
    EXPECT_TRUE(std::isnan(std::numeric_limits<float>::quiet_NaN() + 0.0f));
    EXPECT_TRUE(std::isnan(std::numeric_limits<float>::quiet_NaN() + 0));
    EXPECT_TRUE(std::isnan(0 + std::numeric_limits<float>::quiet_NaN()));
    float value = std::numeric_limits<float>::quiet_NaN();
    EXPECT_TRUE(std::isnan(value+0));
    
    EXPECT_TRUE(std::isnan(std::min (std::numeric_limits<float>::quiet_NaN(), 0.0f)));
    EXPECT_EQ(       0.0f, std::min (0.0f, std::numeric_limits<float>::quiet_NaN()));
    EXPECT_EQ(       0.0f, std::fmin(0.0f, std::numeric_limits<float>::quiet_NaN()));
    EXPECT_EQ(       0.0f, std::fmin(std::numeric_limits<float>::quiet_NaN(), 0.0f));
}

TEST(float, signaling_NaN)
{
    ASSERT_TRUE(std::numeric_limits<float>::has_signaling_NaN);
    ASSERT_TRUE(std::isnan(std::numeric_limits<float>::signaling_NaN()));

    EXPECT_TRUE(std::numeric_limits<float>::signaling_NaN() != std::numeric_limits<float>::signaling_NaN());
    const auto foo = std::numeric_limits<float>::signaling_NaN();
    EXPECT_TRUE(foo != foo);
    EXPECT_FALSE(foo == foo);
    EXPECT_NE(std::numeric_limits<float>::signaling_NaN(), std::numeric_limits<float>::signaling_NaN());

    EXPECT_TRUE(std::isnan(std::numeric_limits<float>::signaling_NaN() + 0.0f));
    EXPECT_TRUE(std::isnan(std::numeric_limits<float>::signaling_NaN() + 0));
    EXPECT_TRUE(std::isnan(0 + std::numeric_limits<float>::signaling_NaN()));
}

TEST(float, sqrt)
{
    EXPECT_EQ(std::sqrt(4.0f), 2.0f);
    EXPECT_EQ(std::sqrt(1.0f), 1.0f);
    EXPECT_EQ(std::sqrt(0.0f), 0.0f);
    EXPECT_EQ(std::sqrt(std::numeric_limits<float>::infinity()), std::numeric_limits<float>::infinity());
    EXPECT_EQ(std::sqrt(std::numeric_limits<float>::infinity() * 10.0f), std::numeric_limits<float>::infinity());
    
#ifndef __FAST_MATH__
    EXPECT_FALSE(std::isnan(std::sqrt(std::numeric_limits<float>::infinity())));
    EXPECT_TRUE (std::isnan(std::sqrt(std::numeric_limits<float>::quiet_NaN())));
    EXPECT_TRUE (std::isnan(std::sqrt(-1.0f)));
#else
    EXPECT_TRUE (std::isnan(std::sqrt(std::numeric_limits<float>::infinity())));
    EXPECT_FALSE(std::isnan(std::sqrt(std::numeric_limits<float>::quiet_NaN())));
    EXPECT_FALSE(std::isnan(std::sqrt(-1.0f)));
#endif
}

TEST(float, casting)
{
    EXPECT_EQ(int(0.0f), 0);
    EXPECT_EQ(int(1.0f), 1);
    EXPECT_EQ(int(-1.0f), -1);
    
    EXPECT_EQ(unsigned(1.0f), 1u);
    EXPECT_EQ(unsigned(4.7f), 4u);

    // Casting a negative float value to an unsigned integral type is not defined behavior.
    // EXPECT_EQ(unsigned(-1.0f), 1);
    // EXPECT_EQ(unsigned(-4.7f), 4);
}

TEST(float, beta0)
{
    {
        const auto beta = std::nextafter(0.0f, 1.0f);
        const auto coefficient0 = 1 - beta;
        const auto coefficient1 = beta;
        EXPECT_EQ(coefficient0 + coefficient1, 1.0f);
    }

    {
        const auto x = 2.587699890136719e-02f;
        const auto beta = 0.0866042823f;
        const auto coefficient0 = 1 - beta;
        const auto coefficient1 = beta;
        ASSERT_EQ(coefficient0 + coefficient1, 1.0f);
        EXPECT_NE(x * coefficient0 + x * coefficient1, x);
    }
    {
        const auto y = 5.515012264251709e+00f;
        const auto beta = 0.0866042823f;
        const auto coefficient0 = 1 - beta;
        const auto coefficient1 = beta;
        ASSERT_EQ(coefficient0 + coefficient1, 1.0f);
        EXPECT_NE(y * coefficient0 + y * coefficient1, y);
    }
}

TEST(float, beta1)
{
    const auto beta = std::nextafter(1.0f, 0.0f);
    const auto coefficient0 = 1 - beta;
    const auto coefficient1 = beta;
    EXPECT_EQ(coefficient0 + coefficient1, 1.0f);
}

TEST(float, zero)
{
    EXPECT_EQ(-0.0f, +0.0f);
    const auto a = -0.0f;
    const auto b = 1.0f;
    EXPECT_EQ(a / b, 0.0f);
    EXPECT_TRUE(-0.0f / 1.0f == +0.0f);
    EXPECT_EQ(-0.0f / 1.0f, 0.0f);
    EXPECT_TRUE(-3.0f * 0.0f == +0.0f);
    EXPECT_EQ(-3.0f * 0.0f, -0.0f);
}

TEST(float, GetInvalid)
{
    auto val = playrho::GetInvalid<float>();
    EXPECT_TRUE(std::isnan(val));
    const auto same = std::is_same<decltype(val), float>::value;
    EXPECT_TRUE(same);
}

TEST(float, GetTypeName)
{
    const auto name = playrho::GetTypeName<float>();
    EXPECT_STREQ(name, "float");
}

TEST(float, traits)
{
    EXPECT_TRUE((playrho::IsAddable<float>::value));
    EXPECT_TRUE((playrho::IsAddable<float,float>::value));
    EXPECT_TRUE((playrho::IsAddable<float,double>::value));
    EXPECT_TRUE((playrho::IsAddable<float,int>::value));
}
