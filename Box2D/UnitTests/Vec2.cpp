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
#include <Box2D/Common/Math.h>

using namespace box2d;

TEST(Vec2, ByteSizeIs8)
{
	EXPECT_EQ(sizeof(Vec2), size_t(8));
}

TEST(Vec2, max_size) {
	Vec2 vector;
	EXPECT_EQ(Vec2::size_type{2}, vector.max_size());
}

TEST(Vec2, Constructor) {
	Vec2 vector{float_t{5}, float_t{-3}};
	EXPECT_EQ(float_t{5}, vector.x);
	EXPECT_EQ(float_t{-3}, vector.y);
}

TEST(Vec2, Indexing) {
	Vec2 vector{float_t{5}, float_t{-3}};
	EXPECT_EQ(float_t{5}, vector[0]);
	EXPECT_EQ(float_t{-3}, vector[1]);
	vector[0] = float_t{4};
	EXPECT_EQ(float_t{4}, vector[0]);
	vector[1] = float_t{-2};
	EXPECT_EQ(float_t{-2}, vector[1]);
}

TEST(Vec2, Equality)
{	
	Vec2 vector{float_t{5}, float_t{-3}};
	EXPECT_EQ(vector.x, vector.x);
	EXPECT_EQ(vector.y, vector.y);
	EXPECT_EQ(vector, vector);
}

TEST(Vec2, Inequality)
{	
	Vec2 vector1{float_t{5}, float_t{-3}};
	Vec2 vector2{float_t{-5}, float_t{+3}};
	EXPECT_NE(vector1.x, vector2.x);
	EXPECT_NE(vector1.y, vector2.y);
	EXPECT_NE(vector1, vector2);
}

TEST(Vec2, Negate)
{
	Vec2 v10{1, 0};
	Vec2 n10 = -v10;
	Vec2 v01{0, 1};
	Vec2 n01 = -v01;
	EXPECT_EQ(-v10.x, n10.x);
	EXPECT_EQ(-v10.y, n10.y);
	EXPECT_EQ(-v01.x, n01.x);
	EXPECT_EQ(-v01.y, n01.y);
	
	EXPECT_EQ(-22, (-Vec2{22, 0}).x);
	EXPECT_EQ(-3, (-Vec2{0, 3}).y);
}

TEST(Vec2, Rotate)
{
	Vec2 v10{1, 0};
	Vec2 v01{0, 1};

	EXPECT_EQ(round(v01), round(Rotate(v10, Rot(DegreesToRadians(90)))));

	EXPECT_EQ(round(Vec2{22, 30}), round(Rotate(Vec2{22, 30}, Rot(DegreesToRadians(0)))));
	EXPECT_EQ(round(Vec2{22, 30}, 1000), round(Rotate(Vec2{22, 30}, Rot(DegreesToRadians(360))), 1000));
	EXPECT_EQ(round(-Vec2{22, 30}, 1000), round(Rotate(Vec2{22, 30}, Rot(DegreesToRadians(180))), 1000));
}