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
	
	const auto a = foo.ex.x, b = foo.ey.x, c = foo.ex.y, d = foo.ey.y;
	auto det = (a * d) - (b * c);
	if (det != float_t{0})
	{
		det = float_t{1} / det;
	}

	Mat33 boo{c1, c2, c3};
	boo = GetInverse22(foo);

	EXPECT_EQ(0, boo.ez.x);
	EXPECT_EQ(0, boo.ez.y);
	EXPECT_EQ(0, boo.ez.z);
	
	EXPECT_EQ(0, boo.ey.z);
	EXPECT_EQ(0, boo.ex.z);
	
	EXPECT_EQ(boo.ex.x, det * d);
	EXPECT_EQ(boo.ex.y, -det * c);
	EXPECT_EQ(boo.ey.x, -det * b);
	EXPECT_EQ(boo.ey.y, det * a);
}

TEST(Mat33, GetSymInverse33)
{
	Vec3 c1{1, 1, 1};
	Vec3 c2{2, 2, 2};
	Vec3 c3{3, 3, 3};
	const Mat33 foo{c1, c2, c3};
	
	auto det = Dot(foo.ex, Cross(foo.ey, foo.ez));
	if (det != float_t{0})
	{
		det = float_t{1} / det;
	}
	
	const auto a11 = foo.ex.x, a12 = foo.ey.x, a13 = foo.ez.x;
	const auto a22 = foo.ey.y, a23 = foo.ez.y;
	const auto a33 = foo.ez.z;
	const auto ex_y = det * (a13 * a23 - a12 * a33);
	const auto ex_z = det * (a12 * a23 - a13 * a22);
	const auto ey_z = det * (a13 * a12 - a11 * a23);

	Mat33 boo{c1, c2, c3};
	boo = GetSymInverse33(foo);

	EXPECT_EQ(boo.ex.x, det * (a22 * a33 - a23 * a23));
	EXPECT_EQ(boo.ex.y, ex_y);
	EXPECT_EQ(boo.ex.z, ex_z);
	
	EXPECT_EQ(boo.ey.x, ex_y);
	EXPECT_EQ(boo.ey.y, det * (a11 * a33 - a13 * a13));
	EXPECT_EQ(boo.ey.z, ey_z);
	
	EXPECT_EQ(boo.ez.x, ex_z);
	EXPECT_EQ(boo.ez.y, ey_z);
	EXPECT_EQ(boo.ez.z, det * (a11 * a22 - a12 * a12));
}