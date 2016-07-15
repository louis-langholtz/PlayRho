//
//  Manifold.cpp
//  Box2D
//
//  Created by Louis D. Langholtz on 7/14/16.
//
//

#include "gtest/gtest.h"
#include <Box2D/Collision/Manifold.hpp>

using namespace box2d;

TEST(Manifold, DefaultConstruction)
{
	Manifold foo;
	EXPECT_EQ(foo.GetType(), Manifold::e_unset);
	EXPECT_EQ(foo.GetPointCount(), 0);
}

TEST(Manifold, PointInitializingConstructor)
{
	const auto lp = Vec2{3, 4};
	const auto ni = float_t(1.2);
	const auto ti = float_t(2.4);
	const auto foo = Manifold::Point{lp, DefaultContactFeature, ni, ti};
	EXPECT_EQ(foo.localPoint.x, lp.x);
	EXPECT_EQ(foo.localPoint.y, lp.y);
	EXPECT_EQ(foo.contactFeature, DefaultContactFeature);
	EXPECT_EQ(foo.normalImpulse, ni);
	EXPECT_EQ(foo.tangentImpulse, ti);
}

TEST(Manifold, GetForCircles)
{
	Manifold foo = Manifold::GetForCircles(Vec2{0, 0}, Manifold::Point{});
	EXPECT_EQ(foo.GetType(), Manifold::e_circles);
	EXPECT_EQ(foo.GetPointCount(), 1);
}
