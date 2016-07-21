//
//  Epsilon.cpp
//  Box2D
//
//  Created by Louis D. Langholtz on 7/8/16.
//
//

#include "gtest/gtest.h"
#include <Box2D/Common/Math.h>

using namespace box2d;

static inline bool ten_epsilon_equal(float_t x, float_t y)
{
	// Here's essentially algorthm originally used in b2Collision.cpp b2TestOverlap function
	// Pros: Probably faster and fine with larger LinearSlop settings.
	// Cons: Doesn't scale to magnitude of values used which becomes more problematic with smaller
	//   LinearSlop settings.
	return std::abs(x - y) < (Epsilon * 10);
}

TEST(Epsilon, AlmostEqual)
{
	{
		const auto a = float_t(0);
		const auto b = float_t(0);
		EXPECT_FLOAT_EQ(a, b);
		EXPECT_PRED3(almost_equal, a, b, 1);
		EXPECT_PRED3(almost_equal, a, b, 2);
	}
	{
		const auto a = float_t(1000);
		const auto b = float_t(1000 + 0.0001);
		EXPECT_FLOAT_EQ(a, b);
		EXPECT_PRED3(almost_equal, a, b, 1);
		EXPECT_PRED3(almost_equal, a, b, 2);
		EXPECT_PRED3(almost_equal, a, b, 3);
	}
	{
		const auto a = float_t(0.000001);
		const auto b = float_t(0.000001 * 2);
		EXPECT_EQ(almost_equal(a, b), false);
	}
}

TEST(Epsilon, ten_epsilon_equal)
{
	{
		const auto a = float_t(0);
		const auto b = float_t(0);
		EXPECT_FLOAT_EQ(a, b);
		EXPECT_EQ(ten_epsilon_equal(a, b), true);
	}
	{
		// Demonstrates the problem with not scaling...
		const auto a = float_t(1000);
		const auto b = float_t(1000 + 0.0001);
		EXPECT_FLOAT_EQ(a, b); // Google test code says almost equal (as did almost_equal)
		EXPECT_PRED2( [](float_t lhs, float_t rhs) { return !ten_epsilon_equal(lhs, rhs); }, a, b); // 10 Epsilon says not equal
	}
	{
		// Demonstrates the problem with not scaling...
		const auto a = float_t(0.000001);
		const auto b = float_t(0.000001 * 2);
		EXPECT_EQ(almost_equal(a, b), false);
		EXPECT_PRED2(ten_epsilon_equal, a, b);
	}
}