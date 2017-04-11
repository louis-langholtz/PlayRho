/*
 * Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

TEST(AABB, ByteSizeIsTwiceVec2)
{
	EXPECT_EQ(sizeof(AABB), sizeof(Vec2) * 2);
}

TEST(AABB, DefaultConstruction)
{
	const auto infinity = std::numeric_limits<RealNum>::infinity();
	const auto lb = Vec2{infinity, infinity} * Meter;
	const auto ub = Vec2{-infinity, -infinity} * Meter;
	const auto aabb = AABB{};
	EXPECT_EQ(aabb.GetLowerBound(), lb);
	EXPECT_EQ(aabb.GetUpperBound(), ub);
}

TEST(AABB, DefaultAabbAddsToOther)
{
	const auto default_aabb = AABB{};
	{
		const auto other_aabb = AABB{Vec2_zero * Meter, Vec2_zero * Meter};
		const auto sum_aabb = GetEnclosingAABB(default_aabb, other_aabb);
		EXPECT_EQ(sum_aabb.GetLowerBound(), other_aabb.GetLowerBound());
		EXPECT_EQ(sum_aabb.GetUpperBound(), other_aabb.GetUpperBound());
	}
	{
		const auto other_aabb = AABB{Vec2_zero * Meter, Vec2_zero * Meter};
		const auto sum_aabb = GetEnclosingAABB(other_aabb, default_aabb);
		EXPECT_EQ(sum_aabb.GetLowerBound(), other_aabb.GetLowerBound());
		EXPECT_EQ(sum_aabb.GetUpperBound(), other_aabb.GetUpperBound());
	}
	{
		const auto other_aabb = AABB{Vec2{-1, -2} * Meter, Vec2{+99, +3} * Meter};
		const auto sum_aabb = GetEnclosingAABB(other_aabb, default_aabb);
		EXPECT_EQ(sum_aabb.GetLowerBound(), other_aabb.GetLowerBound());
		EXPECT_EQ(sum_aabb.GetUpperBound(), other_aabb.GetUpperBound());
	}
}

TEST(AABB, DefaultAabbIncrementsToOther)
{
	{
		auto default_aabb = AABB{};
		const auto other_aabb = AABB{Vec2_zero * Meter, Vec2_zero * Meter};
		default_aabb += other_aabb;
		EXPECT_EQ(default_aabb.GetLowerBound(), other_aabb.GetLowerBound());
		EXPECT_EQ(default_aabb.GetUpperBound(), other_aabb.GetUpperBound());
	}
	{
		auto default_aabb = AABB{};
		const auto other_aabb = AABB{Vec2{-1, -2} * Meter, Vec2{+99, +3} * Meter};
		default_aabb += other_aabb;
		EXPECT_EQ(default_aabb.GetLowerBound(), other_aabb.GetLowerBound());
		EXPECT_EQ(default_aabb.GetUpperBound(), other_aabb.GetUpperBound());
	}
}

TEST(AABB, InitializingConstruction)
{
	const auto lower_x = RealNum(-2) * Meter;
	const auto lower_y = RealNum(-3) * Meter;
	const auto upper_x = RealNum(+1.6) * Meter;
	const auto upper_y = RealNum(+1.9) * Meter;
	
	const auto center_x = (lower_x + upper_x) / RealNum{2};
	const auto center_y = (lower_y + upper_y) / RealNum{2};

	const auto v0 = Length2D{upper_x, lower_y};
	const auto v1 = Length2D{lower_x, upper_y};
	
	{
		AABB foo{v0, v1};
		EXPECT_EQ(GetCenter(foo).x, center_x);
		EXPECT_EQ(GetCenter(foo).y, center_y);
		EXPECT_EQ(foo.GetLowerBound().x, lower_x);
		EXPECT_EQ(foo.GetLowerBound().y, lower_y);
		EXPECT_EQ(foo.GetUpperBound().x, upper_x);
		EXPECT_EQ(foo.GetUpperBound().y, upper_y);
	}
	{
		AABB foo{v1, v0};
		EXPECT_EQ(GetCenter(foo).x, center_x);
		EXPECT_EQ(GetCenter(foo).y, center_y);
		EXPECT_EQ(foo.GetLowerBound().x, lower_x);
		EXPECT_EQ(foo.GetLowerBound().y, lower_y);
		EXPECT_EQ(foo.GetUpperBound().x, upper_x);
		EXPECT_EQ(foo.GetUpperBound().y, upper_y);
	}
}

TEST(AABB, TestOverlap)
{
	{
		AABB bb1{Vec2{-2, -3} * Meter, Vec2{-1, 0} * Meter};
		EXPECT_TRUE(TestOverlap(bb1, bb1));
	}
	{
		const auto vec = Vec2{-2, -3} * Meter;
		AABB bb1{vec, vec};
		EXPECT_TRUE(TestOverlap(bb1, bb1));
	}
	{
		AABB bb1{Vec2{-2, -3} * Meter, Vec2{-1, 0} * Meter};
		AABB bb2{Vec2{-1, -1} * Meter, Vec2{1, 2} * Meter};
		EXPECT_TRUE(TestOverlap(bb1, bb2));
	}
	{
		AABB bb1{Vec2{-99, -3} * Meter, Vec2{-1, 0} * Meter};
		AABB bb2{Vec2{76, -1} * Meter, Vec2{-2, 2} * Meter};
		EXPECT_TRUE(TestOverlap(bb1, bb2));
	}
	{
		AABB bb1{Vec2{-20, -3} * Meter, Vec2{-18, 0} * Meter};
		AABB bb2{Vec2{-1, -1} * Meter, Vec2{1, 2} * Meter};
		EXPECT_FALSE(TestOverlap(bb1, bb2));
	}
	{
		AABB bb1{Vec2{-2, -3} * Meter, Vec2{-1, 0} * Meter};
		AABB bb2{Vec2{-1, +1} * Meter, Vec2{1, 2} * Meter};
		EXPECT_FALSE(TestOverlap(bb1, bb2));
	}
	{
		AABB bb1{Vec2{-2, +3} * Meter, Vec2{-1, 0} * Meter};
		AABB bb2{Vec2{-1, -1} * Meter, Vec2{0, -2} * Meter};
		EXPECT_FALSE(TestOverlap(bb1, bb2));
	}
}
