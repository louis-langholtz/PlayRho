//
//  PolygonShape.cpp
//  Box2D
//
//  Created by Louis D. Langholtz on 7/16/16.
//
//

#include "gtest/gtest.h"
#include <Box2D/Collision/Shapes/PolygonShape.h>

using namespace box2d;

TEST(PolygonShape, DefaultConstruction)
{
	PolygonShape shape{};
	EXPECT_EQ(shape.GetType(), Shape::e_polygon);
	EXPECT_EQ(shape.GetVertexCount(), 0);
	EXPECT_EQ(shape.GetChildCount(), 1);
	EXPECT_EQ(shape.GetRadius(), PolygonRadius);
}