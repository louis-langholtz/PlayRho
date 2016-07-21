//
//  Transformation.cpp
//  Box2D
//
//  Created by Louis D. Langholtz on 7/6/16.
//
//

#include "gtest/gtest.h"
#include <Box2D/Common/Math.h>

using namespace box2d;

TEST(Transformation, Initialize)
{
	const Vec2 translation{2, 4};
	const Rot rotation{Pi / 2};
	const Transformation xfm{translation, rotation};
	EXPECT_EQ(translation, xfm.p);
	EXPECT_EQ(rotation, xfm.q);
}

TEST(Transformation, Equality)
{
	const Vec2 translation{2, 4};
	const Rot rotation{Pi / 2};
	const Transformation xfm{translation, rotation};
	EXPECT_EQ(xfm, xfm);
}

TEST(Transformation, Inequality)
{
	const Vec2 translation1{2, 4};
	const Rot rotation1{Pi / 2};
	const Transformation xfm1{translation1, rotation1};

	const Vec2 translation2{-3, 37};
	const Rot rotation2{Pi * 2};
	const Transformation xfm2{translation2, rotation2};

	EXPECT_NE(xfm1, xfm2);
}

TEST(Transformation, Mul)
{
	const Vec2 translation1{2, 4};
	const Rot rotation1{Pi / 2};
	const Transformation xfm{translation1, rotation1};

	const auto xfm2 = Mul(xfm, xfm);
	const Vec2 translation2{4, 8};
	const Rot rotation2{Pi};

	const auto Ap = xfm.p;
	const auto Bp = xfm.p;
	const auto newP = Ap + Rotate(Bp, xfm.q);
	EXPECT_EQ(xfm2.p.x, newP.x);
	EXPECT_EQ(xfm2.p.y, newP.y);
	
	EXPECT_EQ(xfm2.q.cos(), rotation2.cos());
	EXPECT_EQ(xfm2.q.sin(), rotation2.sin());
}