/*
 * Copyright (c) 2016 Louis Langholtz https://github.com/louis-langholtz/Box2D
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
#include <Box2D/Common/Math.hpp>

using namespace box2d;

static inline bool ten_epsilon_equal(float x, float y)
{
	// Here's essentially algorthm originally used in b2Collision.cpp b2TestOverlap function
	// Pros: Probably faster and fine with larger LinearSlop settings.
	// Cons: Doesn't scale to magnitude of values used which becomes more problematic with smaller
	//   LinearSlop settings.
	return std::abs(x - y) < (std::numeric_limits<float>::epsilon() * 10);
}

TEST(Epsilon, AlmostEqual)
{
	{
		EXPECT_TRUE(almost_equal(0.0f, 0.0f));
		EXPECT_TRUE(almost_equal(1.0f, 1.0f));
		EXPECT_TRUE(almost_equal(-1.0f, -1.0f));
		EXPECT_TRUE(almost_equal(std::numeric_limits<float>::min(), std::numeric_limits<float>::min()));
		EXPECT_TRUE(almost_equal(std::numeric_limits<float>::min(), std::numeric_limits<float>::min(), 0));
		EXPECT_TRUE(almost_equal(std::numeric_limits<float>::min(), std::numeric_limits<float>::min(), 1));
		EXPECT_TRUE(almost_equal(std::numeric_limits<float>::min(), std::numeric_limits<float>::min(), 2));
		EXPECT_TRUE(almost_equal(std::numeric_limits<float>::min(), std::numeric_limits<float>::min(), 20));
		EXPECT_FALSE(almost_equal(std::numeric_limits<float>::min(), float(0), 0));
		EXPECT_FALSE(almost_equal(std::numeric_limits<float>::min(), float(0), 1));
		EXPECT_FALSE(almost_equal(std::numeric_limits<float>::min(), float(0), 2));
		EXPECT_FALSE(almost_equal(std::numeric_limits<float>::min(), -std::numeric_limits<float>::min()));
	}
	{
		const auto a = float(0);
		const auto b = float(0);
		EXPECT_FLOAT_EQ(a, b);
		EXPECT_TRUE(almost_equal(a, b, 1));
		EXPECT_TRUE(almost_equal(a, b, 2));
	}
	{
		const auto a = float(1000);
		const auto b = float(1000 + 0.0001);
		EXPECT_FLOAT_EQ(a, b);
		EXPECT_TRUE(almost_equal(a, b, 1));
		EXPECT_TRUE(almost_equal(a, b, 2));
		EXPECT_TRUE(almost_equal(a, b, 3));
	}
	{
		const auto a = float(0.000001);
		const auto b = float(0.000001 * 2);
		EXPECT_EQ(almost_equal(a, b), false);
	}
	{
		const auto Epsilon = std::numeric_limits<float>::epsilon();
		EXPECT_FALSE(almost_equal(float(1) + Epsilon, float(1), 0));
		EXPECT_TRUE(almost_equal(float(1) + Epsilon, float(1), 1));
		EXPECT_TRUE(almost_equal(float(1) + Epsilon, float(1), 2));
		EXPECT_TRUE(almost_equal(float(1) + Epsilon, float(1), 3));
		const auto a = std::numeric_limits<float>::min() * std::numeric_limits<float>::epsilon();
		EXPECT_LT(a, std::numeric_limits<float>::min());
		EXPECT_LT(a, std::numeric_limits<float>::epsilon());
		EXPECT_TRUE(almost_zero(a));
		EXPECT_TRUE(almost_equal(std::numeric_limits<float>::min() * std::numeric_limits<float>::epsilon() * 2, 0));
		EXPECT_TRUE(almost_zero(std::numeric_limits<float>::min() * std::numeric_limits<float>::epsilon() * 2));
		EXPECT_FALSE(almost_zero(std::numeric_limits<float>::min()));

		EXPECT_FALSE(almost_equal(std::numeric_limits<float>::min() * 2, std::numeric_limits<float>::min()));
		EXPECT_FALSE(almost_equal(std::numeric_limits<float>::min(), 0.0f));
		EXPECT_FALSE(almost_equal(std::numeric_limits<float>::min() * float(1.001), 0));
		EXPECT_TRUE(almost_equal(std::numeric_limits<float>::min() * 0.5f, std::numeric_limits<float>::min()));
		EXPECT_TRUE(almost_equal(std::numeric_limits<float>::min() * 0.5f, 0.0f));
		EXPECT_TRUE(almost_zero(std::numeric_limits<float>::min() * 0.5f));
		// (Abs(x - y) < (std::numeric_limits<float>::epsilon() * Abs(x + y) * ulp))
	}
	
	EXPECT_TRUE(almost_equal(50.0001373f, 50.0001564f));
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
		EXPECT_FLOAT_EQ(a, b); // Google test code says almost equal (as did almost_equal)
		EXPECT_PRED2( [](float lhs, float rhs) { return !ten_epsilon_equal(lhs, rhs); }, a, b); // 10 Epsilon says not equal
	}
	{
		// Demonstrates the problem with not scaling...
		const auto a = float(0.000001);
		const auto b = float(0.000001 * 2);
		EXPECT_EQ(almost_equal(a, b), false);
		EXPECT_PRED2(ten_epsilon_equal, a, b);
	}
}
