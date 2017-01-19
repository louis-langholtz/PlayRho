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
#include <sstream>

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
	Vec2 vector{RealNum{5}, RealNum{-3}};
	EXPECT_EQ(RealNum{5}, vector.x);
	EXPECT_EQ(RealNum{-3}, vector.y);
}

TEST(Vec2, OutputOperator)
{
	std::stringstream os;
	const Vec2 value{RealNum(1.5), RealNum(-2.3)};
	os << value;
	EXPECT_EQ(os.str(), "Vec2(1.5,-2.3)");
}

TEST(Vec2, Indexing) {
	Vec2 vector{RealNum{5}, RealNum{-3}};
	EXPECT_EQ(RealNum{5}, vector[0]);
	EXPECT_EQ(RealNum{-3}, vector[1]);
	vector[0] = RealNum{4};
	EXPECT_EQ(RealNum{4}, vector[0]);
	vector[1] = RealNum{-2};
	EXPECT_EQ(RealNum{-2}, vector[1]);
}

TEST(Vec2, Equality)
{	
	Vec2 vector{RealNum{5}, RealNum{-3}};
	EXPECT_EQ(vector.x, vector.x);
	EXPECT_EQ(vector.y, vector.y);
	EXPECT_EQ(vector, vector);
}

TEST(Vec2, Inequality)
{	
	Vec2 vector1{RealNum{5}, RealNum{-3}};
	Vec2 vector2{RealNum{-5}, RealNum{+3}};
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

	EXPECT_EQ(round(v01), round(Rotate(v10, UnitVec2{90_deg})));

	EXPECT_EQ(round(Vec2{22, 30}), round(Rotate(Vec2{22, 30}, UnitVec2{0_deg})));
	EXPECT_EQ(round(Vec2{22, 30}, 1000), round(Rotate(Vec2{22, 30}, UnitVec2{360_deg}), 1000));
	EXPECT_EQ(round(-Vec2{22, 30}, 1000), round(Rotate(Vec2{22, 30}, UnitVec2{180_deg}), 1000));
}

TEST(Vec2, IncrementOperator)
{
	auto a = Vec2{0, 0};
	ASSERT_EQ(a, Vec2(0, 0));
	const auto inc = Vec2{1, 1};
	
	a += inc;
	
	EXPECT_EQ(a, inc);
	
	a += inc;
	
	EXPECT_EQ(a, inc * 2);
}
