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

TEST(Manifold, ByteSizeIs60)
{
	EXPECT_EQ(sizeof(Manifold), size_t(60));
}

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
	EXPECT_EQ(foo.GetPointCount(), Manifold::size_type(1));
}

TEST(Manifold, GetForFaceA)
{
	const auto ln = Vec2{0, 0};
	const auto lp = Vec2{0, 0};
	{
		Manifold foo = Manifold::GetForFaceA(ln, lp);
		EXPECT_EQ(foo.GetType(), Manifold::e_faceA);
		EXPECT_EQ(foo.GetLocalNormal(), ln);
		EXPECT_EQ(foo.GetLocalPoint(), lp);
		EXPECT_EQ(foo.GetPointCount(), Manifold::size_type(0));
	}
	{
		const auto pl = Vec2{float_t(-0.12), float_t(0.34)};
		const auto cf = DefaultContactFeature;
		const auto ni = float_t(2.9);
		const auto ti = float_t(.7);
		Manifold foo = Manifold::GetForFaceA(ln, lp, Manifold::Point{pl, cf, ni, ti});
		EXPECT_EQ(foo.GetType(), Manifold::e_faceA);
		EXPECT_EQ(foo.GetLocalNormal(), ln);
		EXPECT_EQ(foo.GetLocalPoint(), lp);

		EXPECT_EQ(foo.GetPointCount(), Manifold::size_type(1));
		const auto p0 = foo.GetPoint(0);
		EXPECT_EQ(p0.localPoint, pl);
		EXPECT_EQ(p0.contactFeature, cf);
		EXPECT_EQ(p0.normalImpulse, ni);
		EXPECT_EQ(p0.tangentImpulse, ti);
	}
	{
		const auto pl = Vec2{float_t(-0.12), float_t(0.34)};
		const auto cf = DefaultContactFeature;
		const auto ni = float_t(2.9);
		const auto ti = float_t(.7);
		Manifold foo = Manifold::GetForFaceA(ln, lp, Manifold::Point{pl, cf, ni, ti}, Manifold::Point{-pl, Flip(cf), -ni, -ti});
		EXPECT_EQ(foo.GetType(), Manifold::e_faceA);
		EXPECT_EQ(foo.GetLocalNormal(), ln);
		EXPECT_EQ(foo.GetLocalPoint(), lp);

		EXPECT_EQ(foo.GetPointCount(), Manifold::size_type(2));
		const auto p0 = foo.GetPoint(0);
		EXPECT_EQ(p0.localPoint, pl);
		EXPECT_EQ(p0.contactFeature, cf);
		EXPECT_EQ(p0.normalImpulse, ni);
		EXPECT_EQ(p0.tangentImpulse, ti);
		const auto p1 = foo.GetPoint(1);
		EXPECT_EQ(p1.localPoint, -pl);
		EXPECT_EQ(p1.contactFeature, Flip(cf));
		EXPECT_EQ(p1.normalImpulse, -ni);
		EXPECT_EQ(p1.tangentImpulse, -ti);
	}
}

TEST(Manifold, GetForFaceB)
{
	const auto ln = Vec2{0, 0};
	const auto lp = Vec2{0, 0};
	{
		Manifold foo = Manifold::GetForFaceB(ln, lp);
		EXPECT_EQ(foo.GetType(), Manifold::e_faceB);
		EXPECT_EQ(foo.GetLocalNormal(), ln);
		EXPECT_EQ(foo.GetLocalPoint(), lp);
		EXPECT_EQ(foo.GetPointCount(), Manifold::size_type(0));
	}
	{
		const auto pl = Vec2{float_t(-0.12), float_t(0.34)};
		const auto cf = DefaultContactFeature;
		const auto ni = float_t(2.9);
		const auto ti = float_t(.7);
		Manifold foo = Manifold::GetForFaceB(ln, lp, Manifold::Point{pl, cf, ni, ti});
		EXPECT_EQ(foo.GetType(), Manifold::e_faceB);
		EXPECT_EQ(foo.GetLocalNormal(), ln);
		EXPECT_EQ(foo.GetLocalPoint(), lp);

		EXPECT_EQ(foo.GetPointCount(), Manifold::size_type(1));
		const auto p0 = foo.GetPoint(0);
		EXPECT_EQ(p0.localPoint, pl);
		EXPECT_EQ(p0.contactFeature, cf);
		EXPECT_EQ(p0.normalImpulse, ni);
		EXPECT_EQ(p0.tangentImpulse, ti);
	}
	{
		const auto pl = Vec2{float_t(-0.12), float_t(0.34)};
		const auto cf = DefaultContactFeature;
		const auto ni = float_t(2.9);
		const auto ti = float_t(.7);
		Manifold foo = Manifold::GetForFaceB(ln, lp, Manifold::Point{pl, cf, ni, ti}, Manifold::Point{-pl, Flip(cf), -ni, -ti});
		EXPECT_EQ(foo.GetType(), Manifold::e_faceB);
		EXPECT_EQ(foo.GetLocalNormal(), ln);
		EXPECT_EQ(foo.GetLocalPoint(), lp);
		
		EXPECT_EQ(foo.GetPointCount(), Manifold::size_type(2));
		const auto p0 = foo.GetPoint(0);
		EXPECT_EQ(p0.localPoint, pl);
		EXPECT_EQ(p0.contactFeature, cf);
		EXPECT_EQ(p0.normalImpulse, ni);
		EXPECT_EQ(p0.tangentImpulse, ti);
		const auto p1 = foo.GetPoint(1);
		EXPECT_EQ(p1.localPoint, -pl);
		EXPECT_EQ(p1.contactFeature, Flip(cf));
		EXPECT_EQ(p1.normalImpulse, -ni);
		EXPECT_EQ(p1.tangentImpulse, -ti);
	}
}
