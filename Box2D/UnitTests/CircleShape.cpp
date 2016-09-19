//
//  CircleShape.cpp
//  Box2D
//
//  Created by Louis D. Langholtz on 7/14/16.
//
//

#include "gtest/gtest.h"
#include <Box2D/Collision/Shapes/CircleShape.h>

using namespace box2d;

TEST(CircleShape, DefaultConstruction)
{
	CircleShape foo{};
	
	EXPECT_EQ(foo.GetType(), Shape::e_circle);
	EXPECT_EQ(foo.GetChildCount(), child_count_t{1});
	EXPECT_EQ(foo.GetRadius(), 0);
	EXPECT_EQ(foo.GetPosition().x, 0);
	EXPECT_EQ(foo.GetPosition().y, 0);
	const auto mass_data = foo.ComputeMass(1);
	EXPECT_EQ(mass_data.mass, 0);
	EXPECT_EQ(mass_data.I, 0);
	EXPECT_EQ(mass_data.center.x, 0);
	EXPECT_EQ(mass_data.center.y, 0);
}

TEST(CircleShape, InitConstruction)
{
	const auto radius = float_t(1);
	const auto position = Vec2{-1, 1};
	CircleShape foo{radius, position};
	
	EXPECT_EQ(foo.GetType(), Shape::e_circle);
	EXPECT_EQ(foo.GetChildCount(), child_count_t{1});
	EXPECT_EQ(foo.GetRadius(), radius);
	EXPECT_EQ(foo.GetPosition().x, position.x);
	EXPECT_EQ(foo.GetPosition().y, position.y);
	const auto mass_data = foo.ComputeMass(1);
	EXPECT_EQ(mass_data.mass, Pi);
	EXPECT_FLOAT_EQ(mass_data.I, float_t(7.85398));
	EXPECT_EQ(mass_data.center.x, -1);
	EXPECT_EQ(mass_data.center.y, 1);
}

TEST(CircleShape, TestPoint)
{
	const auto radius = float_t(1);
	const auto position = Vec2{0, 0};
	CircleShape foo{radius, position};
	EXPECT_TRUE(foo.TestPoint(Transform_identity, Vec2{ 0,  0}));
	EXPECT_TRUE(foo.TestPoint(Transform_identity, Vec2{+1,  0}));
	EXPECT_TRUE(foo.TestPoint(Transform_identity, Vec2{ 0, +1}));
	EXPECT_TRUE(foo.TestPoint(Transform_identity, Vec2{ 0, -1}));
	EXPECT_TRUE(foo.TestPoint(Transform_identity, Vec2{-1,  0}));
	EXPECT_FALSE(foo.TestPoint(Transform_identity, Vec2{-1,  -1}));
	EXPECT_FALSE(foo.TestPoint(Transform_identity, Vec2{+1,  +1}));
	EXPECT_FALSE(foo.TestPoint(Transform_identity, Vec2{+float_t(0.9),  +float_t(0.9)}));
}

TEST(CircleShape, ComputeAABB)
{
	const auto radius = float_t(2.4);
	const auto position = Vec2{2, 1};
	CircleShape foo{radius, position};
	const auto aabb = foo.ComputeAABB(Transform_identity, 0);
	EXPECT_EQ(aabb.GetLowerBound().x, position.x - radius);
	EXPECT_EQ(aabb.GetLowerBound().y, position.y - radius);
	EXPECT_EQ(aabb.GetUpperBound().x, position.x + radius);
	EXPECT_EQ(aabb.GetUpperBound().y, position.y + radius);
	EXPECT_EQ(aabb.GetExtents().x, radius);
	EXPECT_EQ(aabb.GetExtents().y, radius);
	EXPECT_EQ(aabb.GetCenter().x, position.x);
	EXPECT_EQ(aabb.GetCenter().y, position.y);
}