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

TEST(Math, InverseTransformTransformedIsOriginal)
{
	const auto vector = Vec2{float_t(19), float_t(-0.5)};
	const auto translation = Vec2{float_t(-3), float_t(+5)};
	const auto rotation = Rot{DegreesToRadians(90)};
	const auto transformation = Transformation{translation, rotation};

	const auto transformed_vector = Transform(vector, transformation);
	const auto inverse_transformed_vector = InverseTransform(transformed_vector, transformation);

	EXPECT_FLOAT_EQ(vector.x, inverse_transformed_vector.x);
	EXPECT_FLOAT_EQ(vector.y, inverse_transformed_vector.y);
}

TEST(Math, TransformInverseTransformedIsOriginal)
{
	const auto vector = Vec2{float_t(19), float_t(-0.5)};
	const auto translation = Vec2{float_t(-3), float_t(+5)};
	const auto rotation = Rot{DegreesToRadians(90)};
	const auto transformation = Transformation{translation, rotation};

	const auto inverse_transformed_vector = InverseTransform(vector, transformation);
	const auto transformed_inverse_vector = Transform(inverse_transformed_vector, transformation);
	
	EXPECT_FLOAT_EQ(vector.x, transformed_inverse_vector.x);
	EXPECT_FLOAT_EQ(vector.y, transformed_inverse_vector.y);
}

TEST(Math, ComputeCentroidCenteredR1)
{
	const auto hx = float_t(1);
	const auto hy = float_t(1);
	const auto real_center = Vec2{0, 0};
	const auto vertices = {
		Vec2{real_center.x + hx, real_center.y + hy},
		Vec2{real_center.x - hx, real_center.y + hy},
		Vec2{real_center.x - hx, real_center.y - hy},
		Vec2{real_center.x + hx, real_center.y - hy}
	};
	const auto center = ComputeCentroid(vertices.begin(), vertices.size());
	EXPECT_EQ(center.x, real_center.x);
	EXPECT_EQ(center.y, real_center.y);
	
	const auto average = Average(vertices.begin(), vertices.size());
	EXPECT_EQ(average.x, center.x);
	EXPECT_EQ(average.y, center.y);
}

TEST(Math, ComputeCentroidCentered0R1000)
{
	const auto hx = float_t(1000);
	const auto hy = float_t(1000);
	const auto real_center = Vec2{0, 0};
	const auto vertices = {
		Vec2{real_center.x + hx, real_center.y + hy},
		Vec2{real_center.x - hx, real_center.y + hy},
		Vec2{real_center.x - hx, real_center.y - hy},
		Vec2{real_center.x + hx, real_center.y - hy}
	};
	const auto center = ComputeCentroid(vertices.begin(), vertices.size());
	EXPECT_EQ(center.x, real_center.x);
	EXPECT_EQ(center.y, real_center.y);
	
	const auto average = Average(vertices.begin(), vertices.size());
	EXPECT_EQ(average.x, center.x);
	EXPECT_EQ(average.y, center.y);
}

TEST(Math, ComputeCentroidUpRight1000R1)
{
	const auto hx = float_t(1);
	const auto hy = float_t(1);
	const auto real_center = Vec2{1000, 1000};
	const auto vertices = {
		Vec2{real_center.x + hx, real_center.y + hy},
		Vec2{real_center.x - hx, real_center.y + hy},
		Vec2{real_center.x - hx, real_center.y - hy},
		Vec2{real_center.x + hx, real_center.y - hy}
	};
	const auto center = ComputeCentroid(vertices.begin(), vertices.size());
	EXPECT_EQ(center.x, real_center.x);
	EXPECT_EQ(center.y, real_center.y);
	
	const auto average = Average(vertices.begin(), vertices.size());
	EXPECT_EQ(average.x, center.x);
	EXPECT_EQ(average.y, center.y);
}

TEST(Math, ComputeCentroidUpRight1000R100)
{
	const auto hx = float_t(100);
	const auto hy = float_t(100);
	const auto real_center = Vec2{1000, 1000};
	const auto vertices = {
		Vec2{real_center.x + hx, real_center.y + hy},
		Vec2{real_center.x - hx, real_center.y + hy},
		Vec2{real_center.x - hx, real_center.y - hy},
		Vec2{real_center.x + hx, real_center.y - hy}
	};
	const auto center = ComputeCentroid(vertices.begin(), vertices.size());
	EXPECT_EQ(center.x, real_center.x);
	EXPECT_EQ(center.y, real_center.y);
	
	const auto average = Average(vertices.begin(), vertices.size());
	EXPECT_EQ(average.x, center.x);
	EXPECT_EQ(average.y, center.y);
}

TEST(Math, ComputeCentroidUpRight10000R01)
{
	const auto hx = float_t(0.1);
	const auto hy = float_t(0.1);
	const auto real_center = Vec2{10000, 10000};
	const auto vertices = {
		Vec2{real_center.x + hx, real_center.y + hy},
		Vec2{real_center.x - hx, real_center.y + hy},
		Vec2{real_center.x - hx, real_center.y - hy},
		Vec2{real_center.x + hx, real_center.y - hy}
	};
	const auto center = ComputeCentroid(vertices.begin(), vertices.size());
	EXPECT_EQ(center.x, real_center.x);
	EXPECT_EQ(center.y, real_center.y);
	
	const auto average = Average(vertices.begin(), vertices.size());
	EXPECT_EQ(average.x, center.x);
	EXPECT_EQ(average.y, center.y);
}

TEST(Math, ComputeCentroidDownLeft1000R1)
{
	const auto hx = float_t(1);
	const auto hy = float_t(1);
	const auto real_center = Vec2{-1000, -1000};
	const auto vertices = {
		Vec2{real_center.x + hx, real_center.y + hy},
		Vec2{real_center.x - hx, real_center.y + hy},
		Vec2{real_center.x - hx, real_center.y - hy},
		Vec2{real_center.x + hx, real_center.y - hy}
	};
	const auto center = ComputeCentroid(vertices.begin(), vertices.size());
	EXPECT_EQ(center.x, real_center.x);
	EXPECT_EQ(center.y, real_center.y);
	
	const auto average = Average(vertices.begin(), vertices.size());
	EXPECT_EQ(average.x, center.x);
	EXPECT_EQ(average.y, center.y);
}

TEST(Math, ComputeCentroidOfHexagonalVertices)
{
	const auto hx = float_t(1);
	const auto hy = float_t(1);
	const auto real_center = Vec2{-1000, -1000};
	const auto vertices = {
		Vec2{real_center.x + 00, real_center.y + 2 * hy},
		Vec2{real_center.x - hx, real_center.y + 1 * hy},
		Vec2{real_center.x - hx, real_center.y - 1 * hy},
		Vec2{real_center.x + 00, real_center.y - 2 * hy},
		Vec2{real_center.x + hx, real_center.y - 1 * hy},
		Vec2{real_center.x + hx, real_center.y + 1 * hy},
	};
	const auto center = ComputeCentroid(vertices.begin(), vertices.size());
	EXPECT_EQ(center.x, real_center.x);
	EXPECT_EQ(center.y, real_center.y);
	
	const auto average = Average(vertices.begin(), vertices.size());
	EXPECT_EQ(average.x, center.x);
	EXPECT_EQ(average.y, center.y);
}