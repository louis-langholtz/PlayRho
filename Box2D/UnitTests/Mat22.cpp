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
#include <Box2D/Common/Math.hpp>

using namespace box2d;

TEST(Mat22, ByteSizeIs16)
{
	EXPECT_EQ(sizeof(Mat22), size_t(16));
}

TEST(Mat22, Init)
{
	Vec2 c1{1, 1};
	Vec2 c2{2, 2};
	const Mat22 foo{c1, c2};
	EXPECT_EQ(c1, foo.ex);
	EXPECT_EQ(c2, foo.ey);
}

TEST(Mat22, Invert)
{
	Vec2 ex{1, 2};
	Vec2 ey{3, 4};
	const Mat22 foo{ex, ey};
	ASSERT_EQ(foo.ex, ex);
	ASSERT_EQ(foo.ey, ey);

	const auto inverted = Invert(foo);
	const auto cp = Cross(ex, ey);
	ASSERT_EQ(cp, float_t(-2));
	const auto det = (cp != 0)? float_t(1)/cp : float_t(0);
	
	EXPECT_EQ(inverted.ex.x, det * foo.ey.y);
	EXPECT_EQ(inverted.ex.y, -det * foo.ex.y);
	EXPECT_EQ(inverted.ey.x, -det * foo.ey.x);
	EXPECT_EQ(inverted.ey.y, det * foo.ex.x);
	
	EXPECT_EQ(inverted.ex.x, float_t(-2));
	EXPECT_EQ(inverted.ex.y, float_t(1));
	EXPECT_EQ(inverted.ey.x, float_t(1.5));
	EXPECT_EQ(inverted.ey.y, float_t(-0.5));
}

TEST(Mat22, InvertInvertedIsOriginal)
{
	Vec2 c1{1, 2};
	Vec2 c2{3, 4};
	const Mat22 foo{c1, c2};
	const auto inverted = Invert(foo);
	const auto inverted2 = Invert(inverted);
	EXPECT_EQ(foo.ex.x, inverted2.ex.x);
	EXPECT_EQ(foo.ex.y, inverted2.ex.y);
	EXPECT_EQ(foo.ey.x, inverted2.ey.x);
	EXPECT_EQ(foo.ey.y, inverted2.ey.y);
}
