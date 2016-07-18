//
//  AABB.cpp
//  Box2D
//
//  Created by Louis D. Langholtz on 7/16/16.
//
//

#include "gtest/gtest.h"
#include <Box2D/Collision/AABB.hpp>

using namespace box2d;

TEST(AABB, InitializingConstruction)
{
	const auto lower_x = float_t(-2);
	const auto lower_y = float_t(-3);
	const auto upper_x = float_t(+1.6);
	const auto upper_y = float_t(+1.9);
	
	const auto center_x = (lower_x + upper_x) / 2;
	const auto center_y = (lower_y + upper_y) / 2;

	const auto v0 = Vec2{upper_x, lower_y};
	const auto v1 = Vec2{lower_x, upper_y};
	
	{
		AABB foo{v0, v1};
		EXPECT_EQ(foo.GetCenter().x, center_x);
		EXPECT_EQ(foo.GetCenter().y, center_y);
		EXPECT_EQ(foo.GetLowerBound().x, lower_x);
		EXPECT_EQ(foo.GetLowerBound().y, lower_y);
		EXPECT_EQ(foo.GetUpperBound().x, upper_x);
		EXPECT_EQ(foo.GetUpperBound().y, upper_y);
	}
	{
		AABB foo{v1, v0};
		EXPECT_EQ(foo.GetCenter().x, center_x);
		EXPECT_EQ(foo.GetCenter().y, center_y);
		EXPECT_EQ(foo.GetLowerBound().x, lower_x);
		EXPECT_EQ(foo.GetLowerBound().y, lower_y);
		EXPECT_EQ(foo.GetUpperBound().x, upper_x);
		EXPECT_EQ(foo.GetUpperBound().y, upper_y);
	}
}

TEST(AABB, TestOverlap)
{
	{
		AABB bb1{Vec2{-2, -3}, Vec2{-1, 0}};
		EXPECT_TRUE(TestOverlap(bb1, bb1));
	}
	{
		const auto vec = Vec2{-2, -3};
		AABB bb1{vec, vec};
		EXPECT_TRUE(TestOverlap(bb1, bb1));
	}
	{
		AABB bb1{Vec2{-2, -3}, Vec2{-1, 0}};
		AABB bb2{Vec2{-1, -1}, Vec2{1, 2}};
		EXPECT_TRUE(TestOverlap(bb1, bb2));
	}
	{
		AABB bb1{Vec2{-99, -3}, Vec2{-1, 0}};
		AABB bb2{Vec2{76, -1}, Vec2{-2, 2}};
		EXPECT_TRUE(TestOverlap(bb1, bb2));
	}
	{
		AABB bb1{Vec2{-20, -3}, Vec2{-18, 0}};
		AABB bb2{Vec2{-1, -1}, Vec2{1, 2}};
		EXPECT_FALSE(TestOverlap(bb1, bb2));
	}
	{
		AABB bb1{Vec2{-2, -3}, Vec2{-1, 0}};
		AABB bb2{Vec2{-1, +1}, Vec2{1, 2}};
		EXPECT_FALSE(TestOverlap(bb1, bb2));
	}
	{
		AABB bb1{Vec2{-2, +3}, Vec2{-1, 0}};
		AABB bb2{Vec2{-1, -1}, Vec2{0, -2}};
		EXPECT_FALSE(TestOverlap(bb1, bb2));
	}
}
