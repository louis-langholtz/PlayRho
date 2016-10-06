//
//  Math.cpp
//  Box2D
//
//  Created by Louis D. Langholtz on 9/29/16.
//
//

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

TEST(Math, ComputeCentroid)
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
	EXPECT_FLOAT_EQ(center.x, real_center.x);
	EXPECT_FLOAT_EQ(center.y, real_center.y);
}