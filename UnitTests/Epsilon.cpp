/*
 * Copyright (c) 2020 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#include "UnitTests.hpp"
#include <PlayRho/Common/Math.hpp>

using namespace playrho;

static inline bool ten_epsilon_equal(float x, float y)
{
    // Here's essentially algorthm originally used in b2Collision.cpp b2TestOverlap function
    // Pros: Probably faster and fine with larger DefaultLinearSlop settings.
    // Cons: Doesn't scale to magnitude of values used which becomes more problematic with smaller
    //   DefaultLinearSlop settings.
    return std::abs(x - y) < (std::numeric_limits<float>::epsilon() * 10);
}

TEST(Epsilon, AlmostEqual)
{
    {
        EXPECT_TRUE(AlmostEqual(0.0f, 0.0f));
        EXPECT_TRUE(AlmostEqual(1.0f, 1.0f));
        EXPECT_TRUE(AlmostEqual(-1.0f, -1.0f));
        EXPECT_TRUE(AlmostEqual(std::numeric_limits<float>::min(), std::numeric_limits<float>::min()));
        EXPECT_TRUE(AlmostEqual(std::numeric_limits<float>::min(), std::numeric_limits<float>::min(), 0));
        EXPECT_TRUE(AlmostEqual(std::numeric_limits<float>::min(), std::numeric_limits<float>::min(), 1));
        EXPECT_TRUE(AlmostEqual(std::numeric_limits<float>::min(), std::numeric_limits<float>::min(), 2));
        EXPECT_TRUE(AlmostEqual(std::numeric_limits<float>::min(), std::numeric_limits<float>::min(), 20));
        EXPECT_FALSE(AlmostEqual(std::numeric_limits<float>::min(), float(0), 0));
        EXPECT_FALSE(AlmostEqual(std::numeric_limits<float>::min(), float(0), 1));
        EXPECT_FALSE(AlmostEqual(std::numeric_limits<float>::min(), float(0), 2));
        EXPECT_FALSE(AlmostEqual(std::numeric_limits<float>::min(), -std::numeric_limits<float>::min()));
    }
    {
        const auto a = float(0);
        const auto b = float(0);
        EXPECT_FLOAT_EQ(a, b);
        EXPECT_TRUE(AlmostEqual(a, b, 1));
        EXPECT_TRUE(AlmostEqual(a, b, 2));
    }
    {
        const auto a = float(1000);
        const auto b = float(1000 + 0.0001);
        EXPECT_FLOAT_EQ(a, b);
        EXPECT_TRUE(AlmostEqual(a, b, 1));
        EXPECT_TRUE(AlmostEqual(a, b, 2));
        EXPECT_TRUE(AlmostEqual(a, b, 3));
    }
    {
        const auto a = float(0.000001);
        const auto b = float(0.000001 * 2);
        EXPECT_EQ(AlmostEqual(a, b), false);
    }
    {
        const auto Epsilon = std::numeric_limits<float>::epsilon();
        EXPECT_FALSE(AlmostEqual(float(1) + Epsilon, float(1), 0));
        EXPECT_TRUE(AlmostEqual(float(1) + Epsilon, float(1), 1));
        EXPECT_TRUE(AlmostEqual(float(1) + Epsilon, float(1), 2));
        EXPECT_TRUE(AlmostEqual(float(1) + Epsilon, float(1), 3));
        const auto a = std::numeric_limits<float>::min() * std::numeric_limits<float>::epsilon();
        EXPECT_LT(a, std::numeric_limits<float>::min());
        EXPECT_LT(a, std::numeric_limits<float>::epsilon());
        EXPECT_TRUE(AlmostZero(a));
        EXPECT_TRUE(AlmostEqual(std::numeric_limits<float>::min() * std::numeric_limits<float>::epsilon() * 2, 0.0f));
        EXPECT_TRUE(AlmostZero(std::numeric_limits<float>::min() * std::numeric_limits<float>::epsilon() * 2));
        EXPECT_FALSE(AlmostZero(std::numeric_limits<float>::min()));

        EXPECT_FALSE(AlmostEqual(std::numeric_limits<float>::min() * 2, std::numeric_limits<float>::min()));
        EXPECT_FALSE(AlmostEqual(std::numeric_limits<float>::min(), 0.0f));
        EXPECT_FALSE(AlmostEqual(std::numeric_limits<float>::min() * float(1.001), 0.0f));
        EXPECT_TRUE(AlmostEqual(std::numeric_limits<float>::min() * 0.5f, std::numeric_limits<float>::min()));
        EXPECT_TRUE(AlmostEqual(std::numeric_limits<float>::min() * 0.5f, 0.0f));
        EXPECT_TRUE(AlmostZero(std::numeric_limits<float>::min() * 0.5f));
        // (abs(x - y) < (std::numeric_limits<float>::epsilon() * abs(x + y) * ulp))
    }
    
    EXPECT_TRUE(AlmostEqual(50.0001373f, 50.0001564f));
}

TEST(Epsilon, ten_epsilon_equal)
{
    {
        const auto a = float(0);
        const auto b = float(0);
        EXPECT_FLOAT_EQ(a, b);
        EXPECT_EQ(ten_epsilon_equal(a, b), true);
    }
    {
        // Demonstrates the problem with not scaling...
        const auto a = float(1000);
        const auto b = float(1000 + 0.0001);
        EXPECT_FLOAT_EQ(a, b); // Google test code says almost equal (as did AlmostEqual)
        EXPECT_PRED2( [](float lhs, float rhs) { return !ten_epsilon_equal(lhs, rhs); }, a, b); // 10 Epsilon says not equal
    }
    {
        // Demonstrates the problem with not scaling...
        const auto a = float(0.000001);
        const auto b = float(0.000001 * 2);
        EXPECT_EQ(AlmostEqual(a, b), false);
        EXPECT_PRED2(ten_epsilon_equal, a, b);
    }
}
