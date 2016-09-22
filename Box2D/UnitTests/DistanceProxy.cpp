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

TEST(DistanceProxy, ByteSizeIs32)
{
	EXPECT_EQ(sizeof(DistanceProxy), size_t(32));
}

TEST(DistanceProxy, OneVecInitialization)
{
	const auto radius = float_t{1};
	const auto vertex0 = Vec2{float_t(2), float_t(-3)};
	DistanceProxy foo{radius, vertex0};
	EXPECT_EQ(radius, foo.GetRadius());
	EXPECT_EQ(1, foo.GetVertexCount());
	EXPECT_EQ(vertex0, foo.GetVertex(0));
}

TEST(DistanceProxy, OneVecSupportIndex)
{
	const auto radius = float_t{1};
	const auto vertex0 = Vec2{float_t(2), float_t(-3)};
	DistanceProxy foo{radius, vertex0};
	EXPECT_EQ(0, foo.GetSupportIndex(vertex0));
	EXPECT_EQ(0, foo.GetSupportIndex(Vec2_zero));
	EXPECT_EQ(0, foo.GetSupportIndex(Vec2{vertex0.y, vertex0.x}));
}

TEST(DistanceProxy, TwoVecInitialization)
{
	const auto radius = float_t{1};
	const auto vertex0 = Vec2{float_t(2), float_t(3)};
	const auto vertex1 = Vec2{float_t(-10), float_t(-1)};
	DistanceProxy foo{radius, vertex0, vertex1};
	EXPECT_EQ(radius, foo.GetRadius());
	EXPECT_EQ(2, foo.GetVertexCount());
	EXPECT_EQ(vertex0, foo.GetVertex(0));
	EXPECT_EQ(vertex1, foo.GetVertex(1));
}

TEST(DistanceProxy, TwoVecSupportIndex)
{
	const auto radius = float_t{1};
	const auto vertex0 = Vec2{float_t(2), float_t(3)};
	const auto vertex1 = Vec2{float_t(-10), float_t(-1)};
	DistanceProxy foo{radius, vertex0, vertex1};
	EXPECT_EQ(0, foo.GetSupportIndex(vertex0));
	EXPECT_EQ(0, foo.GetSupportIndex(Vec2{vertex0.y, vertex0.x}));
	EXPECT_EQ(0, foo.GetSupportIndex(Vec2_zero));
	EXPECT_EQ(1, foo.GetSupportIndex(vertex1));
	EXPECT_EQ(1, foo.GetSupportIndex(Vec2{vertex1.y, vertex1.x}));
}

TEST(DistanceProxy, ThreeVertices)
{
	const auto radius = float_t(33);
	const auto count = DistanceProxy::size_type(3);
	const auto v0 = Vec2{float_t(1), float_t(2)};
	const auto v1 = Vec2{float_t(-3), float_t(-4)};
	const auto v2 = Vec2{float_t(-6), float_t(5)};
	const auto vertices = std::array<Vec2, count>{{v0, v1, v2}};
	
	const DistanceProxy foo{radius, vertices.data(), count};
	
	EXPECT_EQ(foo.GetRadius(), radius);
	ASSERT_EQ(foo.GetVertexCount(), count);
	EXPECT_EQ(foo.GetVertex(0).x, v0.x);
	EXPECT_EQ(foo.GetVertex(0).y, v0.y);
	EXPECT_EQ(foo.GetVertex(1).x, v1.x);
	EXPECT_EQ(foo.GetVertex(1).y, v1.y);
	EXPECT_EQ(foo.GetVertex(2).x, v2.x);
	EXPECT_EQ(foo.GetVertex(2).y, v2.y);
}