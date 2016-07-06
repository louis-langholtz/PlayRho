//
//  DistanceProxy.cpp
//  Box2D
//
//  Created by Louis D. Langholtz on 7/5/16.
//
//

#include "gtest/gtest.h"
#include <Box2D/Collision/DistanceProxy.hpp>

using namespace box2d;

TEST(DistanceProxy, OneVecInitialization)
{
	const auto radius = float_t{1};
	const auto vec1 = Vec2{float_t(2), float_t(-3)};
	DistanceProxy foo{radius, vec1};
	EXPECT_EQ(radius, foo.GetRadius());
	EXPECT_EQ(1, foo.GetVertexCount());
	EXPECT_EQ(vec1, foo.GetVertex(0));
}

TEST(DistanceProxy, OneVecSupportIndex)
{
	const auto radius = float_t{1};
	const auto vec1 = Vec2{float_t(2), float_t(-3)};
	DistanceProxy foo{radius, vec1};
	EXPECT_EQ(0, foo.GetSupportIndex(vec1));
	EXPECT_EQ(0, foo.GetSupportIndex(Vec2_zero));
	EXPECT_EQ(0, foo.GetSupportIndex(Vec2{vec1.y, vec1.x}));
}

TEST(DistanceProxy, TwoVecInitialization)
{
	const auto radius = float_t{1};
	const auto vec1 = Vec2{float_t(2), float_t(3)};
	const auto vec2 = Vec2{float_t(-10), float_t(-1)};
	DistanceProxy foo{radius, vec1, vec2};
	EXPECT_EQ(radius, foo.GetRadius());
	EXPECT_EQ(2, foo.GetVertexCount());
	EXPECT_EQ(vec1, foo.GetVertex(0));
	EXPECT_EQ(vec2, foo.GetVertex(1));
}

TEST(DistanceProxy, TwoVecSupportIndex)
{
	const auto radius = float_t{1};
	const auto vec1 = Vec2{float_t(2), float_t(3)};
	const auto vec2 = Vec2{float_t(-10), float_t(-1)};
	DistanceProxy foo{radius, vec1, vec2};
	EXPECT_EQ(0, foo.GetSupportIndex(vec1));
	EXPECT_EQ(0, foo.GetSupportIndex(Vec2{vec1.y, vec1.x}));
	EXPECT_EQ(0, foo.GetSupportIndex(Vec2_zero));
	EXPECT_EQ(1, foo.GetSupportIndex(vec2));
	EXPECT_EQ(1, foo.GetSupportIndex(Vec2{vec2.y, vec2.x}));
}