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

TEST(PolygonShape, ByteSizeIs276)
{
	EXPECT_EQ(sizeof(PolygonShape), size_t(276));
}

TEST(PolygonShape, DefaultConstruction)
{
	PolygonShape shape{};
	EXPECT_EQ(shape.GetType(), Shape::e_polygon);
	EXPECT_EQ(shape.GetVertexCount(), 0);
	EXPECT_EQ(GetChildCount(shape), child_count_t(1));
	EXPECT_EQ(shape.GetRadius(), PolygonRadius);
}

TEST(PolygonShape, SetAsBox)
{
	PolygonShape shape{};
	const auto hx = float_t(2.3);
	const auto hy = float_t(54.1);
	shape.SetAsBox(hx, hy);
	EXPECT_EQ(shape.GetType(), Shape::e_polygon);
	EXPECT_EQ(GetChildCount(shape), child_count_t(1));
	EXPECT_EQ(shape.GetRadius(), PolygonRadius);

	EXPECT_EQ(shape.GetVertexCount(), PolygonShape::vertex_count_t(4));
	
	// vertices go counter-clockwise (and normals follow their edges)...

	EXPECT_EQ(shape.GetVertex(0).x, -hx);
	EXPECT_EQ(shape.GetVertex(0).y, -hy);
	EXPECT_EQ(shape.GetNormal(0).x, 0);
	EXPECT_EQ(shape.GetNormal(0).y, -1);
	
	EXPECT_EQ(shape.GetVertex(1).x, hx);
	EXPECT_EQ(shape.GetVertex(1).y, -hy);
	EXPECT_EQ(shape.GetNormal(1).x, +1);
	EXPECT_EQ(shape.GetNormal(1).y, 0);
	
	EXPECT_EQ(shape.GetVertex(2).x, hx);
	EXPECT_EQ(shape.GetVertex(2).y, hy);
	EXPECT_EQ(shape.GetNormal(2).x, 0);
	EXPECT_EQ(shape.GetNormal(2).y, +1);
	
	EXPECT_EQ(shape.GetVertex(3).x, -hx);
	EXPECT_EQ(shape.GetVertex(3).y, hy);
	EXPECT_EQ(shape.GetNormal(3).x, -1);
	EXPECT_EQ(shape.GetNormal(3).y, 0);
}