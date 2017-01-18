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
#include <Box2D/Collision/Shapes/CircleShape.hpp>
#include <Box2D/Collision/AABB.hpp>

using namespace box2d;

TEST(CircleShape, ByteSizeIs16)
{
	EXPECT_EQ(sizeof(CircleShape), size_t(16));
}

TEST(CircleShape, DefaultConstruction)
{
	CircleShape foo{};
	
	EXPECT_EQ(foo.GetType(), Shape::e_circle);
	EXPECT_EQ(GetChildCount(foo), child_count_t{1});
	EXPECT_EQ(foo.GetRadius(), CircleShape::GetDefaultRadius());
	EXPECT_EQ(foo.GetLocation().x, 0);
	EXPECT_EQ(foo.GetLocation().y, 0);
}

TEST(CircleShape, InitConstruction)
{
	const auto radius = realnum(1);
	const auto position = Vec2{-1, 1};
	CircleShape foo{radius, position};
	
	EXPECT_EQ(foo.GetType(), Shape::e_circle);
	EXPECT_EQ(GetChildCount(foo), child_count_t{1});
	EXPECT_EQ(foo.GetRadius(), radius);
	EXPECT_EQ(foo.GetLocation().x, position.x);
	EXPECT_EQ(foo.GetLocation().y, position.y);
}

TEST(CircleShape, TestPoint)
{
	const auto radius = realnum(1);
	const auto position = Vec2{0, 0};
	CircleShape foo{radius, position};
	EXPECT_TRUE(TestPoint(foo, Transform_identity, Vec2{ 0,  0}));
	EXPECT_TRUE(TestPoint(foo, Transform_identity, Vec2{+1,  0}));
	EXPECT_TRUE(TestPoint(foo, Transform_identity, Vec2{ 0, +1}));
	EXPECT_TRUE(TestPoint(foo, Transform_identity, Vec2{ 0, -1}));
	EXPECT_TRUE(TestPoint(foo, Transform_identity, Vec2{-1,  0}));
	EXPECT_FALSE(TestPoint(foo, Transform_identity, Vec2{-1,  -1}));
	EXPECT_FALSE(TestPoint(foo, Transform_identity, Vec2{+1,  +1}));
	EXPECT_FALSE(TestPoint(foo, Transform_identity, Vec2{+realnum(0.9),  +realnum(0.9)}));
}

TEST(CircleShape, ComputeAABB)
{
	const auto radius = realnum(2.4);
	const auto position = Vec2{2, 1};
	CircleShape foo{radius, position};
	const auto aabb = ComputeAABB(foo, Transform_identity);
	EXPECT_EQ(aabb.GetLowerBound().x, position.x - radius);
	EXPECT_EQ(aabb.GetLowerBound().y, position.y - radius);
	EXPECT_EQ(aabb.GetUpperBound().x, position.x + radius);
	EXPECT_EQ(aabb.GetUpperBound().y, position.y + radius);
	EXPECT_EQ(aabb.GetExtents().x, radius);
	EXPECT_EQ(aabb.GetExtents().y, radius);
	EXPECT_EQ(aabb.GetCenter().x, position.x);
	EXPECT_EQ(aabb.GetCenter().y, position.y);
}
