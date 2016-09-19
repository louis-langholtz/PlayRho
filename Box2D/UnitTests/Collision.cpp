//
//  Collision.cpp
//  Box2D
//
//  Created by Louis D. Langholtz on 7/16/16.
//
//

#include "gtest/gtest.h"
#include <Box2D/Collision/Collision.h>
#include <Box2D/Collision/Manifold.hpp>
#include <Box2D/Collision/Shapes/CircleShape.h>
#include <Box2D/Collision/Shapes/PolygonShape.h>

using namespace box2d;

TEST(Collision, CollideCircleCircle)
{
	const auto r1 = float_t(1);
	const auto r2 = float_t(1);
	const auto s1 = CircleShape{r1};
	const auto s2 = CircleShape{r2};
	const auto p1 = Vec2{1, 0};
	const auto p2 = Vec2{3, 0};
	const auto t1 = Transformation{p1, Rot_identity};
	const auto t2 = Transformation{p2, Rot_identity};
	const auto manifold = CollideShapes(s1, t1, s2, t2);
	
	EXPECT_EQ(manifold.GetType(), Manifold::e_circles);
	
	//EXPECT_EQ(manifold.GetLocalNormal(), Vec2_zero);
	
	EXPECT_EQ(manifold.GetLocalPoint().x, 0);
	EXPECT_EQ(manifold.GetLocalPoint().y, 0);
	
	EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(1));

	EXPECT_EQ(manifold.GetPoint(0).localPoint.x, 0);
	EXPECT_EQ(manifold.GetPoint(0).localPoint.y, 0);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexA, 0);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 0);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_vertex);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_vertex);
}

TEST(Collision, CollidePolygonCircle)
{
	const auto r2 = float_t(1);
	auto s1 = PolygonShape{};
	const auto hx = float_t(2.2);
	const auto hy = float_t(4.8);
	s1.SetAsBox(hx, hy);
	const auto s2 = CircleShape{r2};
	const auto p1 = Vec2{-1, 0};
	const auto p2 = Vec2{3, 0};
	const auto t1 = Transformation{p1, Rot{DegreesToRadians(45)}};
	const auto t2 = Transformation{p2, Rot{0}};
	const auto manifold = CollideShapes(s1, t1, s2, t2);
	
	EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
	
	EXPECT_EQ(manifold.GetLocalNormal().x, 1);
	EXPECT_EQ(manifold.GetLocalNormal().y, 0);
	
	EXPECT_EQ(manifold.GetLocalPoint().x, hx);
	EXPECT_EQ(manifold.GetLocalPoint().y, 0);
	
	EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(1));
	
	EXPECT_EQ(manifold.GetPoint(0).localPoint.x, 0);
	EXPECT_EQ(manifold.GetPoint(0).localPoint.y, 0);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexA, 0);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.indexB, 0);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeA, ContactFeature::e_vertex);
	EXPECT_EQ(manifold.GetPoint(0).contactFeature.typeB, ContactFeature::e_vertex);
}