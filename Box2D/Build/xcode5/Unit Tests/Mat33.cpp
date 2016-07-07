//
//  Mat33.cpp
//  Box2D
//
//  Created by Louis D. Langholtz on 7/6/16.
//
//

#include "gtest/gtest.h"
#include <Box2D/Common/Math.h>

using namespace box2d;

TEST(Mat33, Init)
{
	Vec3 c1{1, 1, 1};
	Vec3 c2{2, 2, 2};
	Vec3 c3{3, 3, 3};
	Mat33 foo{c1, c2, c3};
	EXPECT_EQ(c1, foo.ex);
	EXPECT_EQ(c2, foo.ey);
	EXPECT_EQ(c3, foo.ez);
}

TEST(Mat33, GetInverse)
{
	Vec3 c1{1, 1, 1};
	Vec3 c2{2, 2, 2};
	Vec3 c3{3, 3, 3};
	const Mat33 foo{c1, c2, c3};
	Mat33 boo{c1, c2, c3};
	boo = foo.GetInverse22();

	EXPECT_EQ(0, boo.ez.x);
	EXPECT_EQ(0, boo.ez.y);
	EXPECT_EQ(0, boo.ez.z);
	
	EXPECT_EQ(0, boo.ey.z);
	EXPECT_EQ(0, boo.ex.z);
	
	const auto a = boo.ex.x, b = boo.ey.x, c = boo.ex.y, d = boo.ey.y;
	auto det = (a * d) - (b * c);
	if (det != float_t{0})
	{
		det = float_t{1} / det;
	}
	EXPECT_EQ(boo.ex.x, det * d);
	EXPECT_EQ(boo.ex.y, -det * c);
	EXPECT_EQ(boo.ey.x, -det * b);
	EXPECT_EQ(boo.ey.y, det * a);
}