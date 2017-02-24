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
#include <Box2D/Common/Math.hpp>

using namespace box2d;

TEST(Transformation, ByteSizeIs_16_32_or_64)
{
	switch (sizeof(RealNum))
	{
		case  4: EXPECT_EQ(sizeof(Transformation), size_t(16)); break;
		case  8: EXPECT_EQ(sizeof(Transformation), size_t(32)); break;
		case 16: EXPECT_EQ(sizeof(Transformation), size_t(64)); break;
		default: FAIL(); break;
	}
}

TEST(Transformation, Initialize)
{
	const Vec2 translation{2, 4};
	const UnitVec2 rotation{1_rad * Pi / 2};
	const Transformation xfm{translation, rotation};
	EXPECT_EQ(translation, xfm.p);
	EXPECT_EQ(rotation, xfm.q);
}

TEST(Transformation, Equality)
{
	const Vec2 translation{2, 4};
	const UnitVec2 rotation{1_rad * Pi / 2};
	const Transformation xfm{translation, rotation};
	EXPECT_EQ(xfm, xfm);
}

TEST(Transformation, Inequality)
{
	const Vec2 translation1{2, 4};
	const UnitVec2 rotation1{1_rad * Pi / 2};
	const Transformation xfm1{translation1, rotation1};

	const Vec2 translation2{-3, 37};
	const UnitVec2 rotation2{1_rad * Pi * 2};
	const Transformation xfm2{translation2, rotation2};

	EXPECT_NE(xfm1, xfm2);
}

TEST(Transformation, Mul)
{
	const Vec2 translation1{2, 4};
	const UnitVec2 rotation1{1_rad * Pi / 2};
	const Transformation xfm{translation1, rotation1};

	const auto xfm2 = Mul(xfm, xfm);
	const Vec2 translation2{4, 8};
	const UnitVec2 rotation2{1_rad * Pi};

	const auto Ap = xfm.p;
	const auto Bp = xfm.p;
	const auto newP = Ap + Rotate(Bp, xfm.q);
	EXPECT_EQ(xfm2.p.x, newP.x);
	EXPECT_EQ(xfm2.p.y, newP.y);
	
	EXPECT_NEAR(double(xfm2.q.cos()), double(rotation2.cos()), 0.0001);
	EXPECT_NEAR(double(xfm2.q.sin()), double(rotation2.sin()), 0.0001);
}
