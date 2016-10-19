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

TEST(Math, DotProductOfTwoVecTwoIsCommutative)
{
	const auto a = Vec2{float_t(-3.2), float_t(1.9)};
	const auto b = Vec2{float_t(4.01), float_t(-0.002)};
	EXPECT_EQ(Dot(a, b), Dot(b, a));
}

TEST(Math, DotProductOfTwoVecThreeIsCommutative)
{
	const auto a = Vec3{float_t(-3.2), float_t(1.9), float_t(36.01)};
	const auto b = Vec3{float_t(4.01), float_t(-0.002), float_t(1.2)};
	EXPECT_EQ(Dot(a, b), Dot(b, a));
}

TEST(Math, CrossProductOfTwoVecTwoIsAntiCommutative)
{
	const auto a = Vec2{float_t(-3.2), float_t(1.9)};
	const auto b = Vec2{float_t(4.01), float_t(-0.002)};
	EXPECT_EQ(Cross(a, b), -Cross(b, a));
}

TEST(Math, TransformIsRotatePlusTranslate)
{
	const auto vector = Vec2{float_t(19), float_t(-0.5)};
	const auto translation = Vec2{float_t(-3), float_t(+5)};
	const auto rotation = Rot{DegreesToRadians(90)};
	const auto transformation = Transformation{translation, rotation};
	
	const auto transformed_vector = Transform(vector, transformation);
	const auto alt = Rotate(vector, rotation) + translation;
	
	EXPECT_EQ(transformed_vector.x, alt.x);
	EXPECT_EQ(transformed_vector.y, alt.y);
}

TEST(Math, InverseTransformIsUntranslateAndInverseRotate)
{
	const auto vector = Vec2{float_t(19), float_t(-0.5)};
	const auto translation = Vec2{float_t(-3), float_t(+5)};
	const auto rotation = Rot{DegreesToRadians(90)};
	const auto transformation = Transformation{translation, rotation};
	
	const auto inv_vector = InverseTransform(vector, transformation);
	const auto alt = InverseRotate(vector - translation, rotation);
	
	EXPECT_EQ(inv_vector.x, alt.x);
	EXPECT_EQ(inv_vector.y, alt.y);
}

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

TEST(Math, GetContactRelVelocity)
{
	const auto velA = Velocity{Vec2(+1, +4), float_t(3.2)};
	const auto velB = Velocity{Vec2(+3, +1), float_t(0.4)};
	const auto relA = Vec2(0, 0);
	const auto relB = Vec2(0, 0);
	const auto result = GetContactRelVelocity(velA, relA, velB, relB);
	
	EXPECT_EQ(result, velB.v - velA.v);
}