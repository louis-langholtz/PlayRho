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
#include <Box2D/Collision/AABB.hpp>

using namespace box2d;

TEST(AABB, ByteSizeIs16)
{
	EXPECT_EQ(sizeof(AABB), size_t(16));
}

TEST(AABB, DefaultConstruction)
{
	const auto lb = Vec2{MaxFloat, MaxFloat};
	const auto ub = Vec2{-MaxFloat, -MaxFloat};
	const auto aabb = AABB{};
	EXPECT_EQ(aabb.GetLowerBound(), lb);
	EXPECT_EQ(aabb.GetUpperBound(), ub);
}

TEST(AABB, DefaultAabbAddsToOther)
{
	const auto default_aabb = AABB{};
	{
		const auto other_aabb = AABB{Vec2_zero, Vec2_zero};
		const auto sum_aabb = default_aabb + other_aabb;
		EXPECT_EQ(sum_aabb.GetLowerBound(), other_aabb.GetLowerBound());
		EXPECT_EQ(sum_aabb.GetUpperBound(), other_aabb.GetUpperBound());
	}
	{
		const auto other_aabb = AABB{Vec2_zero, Vec2_zero};
		const auto sum_aabb = other_aabb + default_aabb;
		EXPECT_EQ(sum_aabb.GetLowerBound(), other_aabb.GetLowerBound());
		EXPECT_EQ(sum_aabb.GetUpperBound(), other_aabb.GetUpperBound());
	}
	{
		const auto other_aabb = AABB{Vec2{-1, -2}, Vec2{+99, +3}};
		const auto sum_aabb = other_aabb + default_aabb;
		EXPECT_EQ(sum_aabb.GetLowerBound(), other_aabb.GetLowerBound());
		EXPECT_EQ(sum_aabb.GetUpperBound(), other_aabb.GetUpperBound());
	}
}

TEST(AABB, DefaultAabbIncrementsToOther)
{
	{
		auto default_aabb = AABB{};
		const auto other_aabb = AABB{Vec2_zero, Vec2_zero};
		default_aabb += other_aabb;
		EXPECT_EQ(default_aabb.GetLowerBound(), other_aabb.GetLowerBound());
		EXPECT_EQ(default_aabb.GetUpperBound(), other_aabb.GetUpperBound());
	}
	{
		auto default_aabb = AABB{};
		const auto other_aabb = AABB{Vec2{-1, -2}, Vec2{+99, +3}};
		default_aabb += other_aabb;
		EXPECT_EQ(default_aabb.GetLowerBound(), other_aabb.GetLowerBound());
		EXPECT_EQ(default_aabb.GetUpperBound(), other_aabb.GetUpperBound());
	}
}

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
