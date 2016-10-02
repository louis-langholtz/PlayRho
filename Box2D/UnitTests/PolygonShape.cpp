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

TEST(PolygonShape, ByteSizeIs272)
{
	EXPECT_EQ(sizeof(PolygonShape), size_t(272));
}

TEST(PolygonShape, DefaultConstruction)
{
	PolygonShape shape{};
	EXPECT_EQ(shape.GetType(), Shape::e_polygon);
	EXPECT_EQ(shape.GetVertexCount(), 0);
	EXPECT_EQ(GetChildCount(shape), child_count_t(1));
	EXPECT_EQ(GetRadius(shape), PolygonRadius);
}

TEST(PolygonShape, SetAsBox)
{
	PolygonShape shape{};
	const auto hx = float_t(2.3);
	const auto hy = float_t(54.1);
	shape.SetAsBox(hx, hy);
	EXPECT_EQ(shape.GetType(), Shape::e_polygon);
	EXPECT_EQ(GetChildCount(shape), child_count_t(1));
	EXPECT_EQ(GetRadius(shape), PolygonRadius);

	ASSERT_EQ(shape.GetVertexCount(), PolygonShape::vertex_count_t(4));
	
	// vertices go counter-clockwise (and normals follow their edges)...

	EXPECT_EQ(shape.GetVertex(0).x, -hx); // left
	EXPECT_EQ(shape.GetVertex(0).y, -hy); // bottom
	EXPECT_EQ(shape.GetNormal(0).x, 0);
	EXPECT_EQ(shape.GetNormal(0).y, -1);
	
	EXPECT_EQ(shape.GetVertex(1).x, hx); // right
	EXPECT_EQ(shape.GetVertex(1).y, -hy); // bottom
	EXPECT_EQ(shape.GetNormal(1).x, +1);
	EXPECT_EQ(shape.GetNormal(1).y, 0);
	
	EXPECT_EQ(shape.GetVertex(2).x, hx); // right
	EXPECT_EQ(shape.GetVertex(2).y, hy); // top
	EXPECT_EQ(shape.GetNormal(2).x, 0);
	EXPECT_EQ(shape.GetNormal(2).y, +1);
	
	EXPECT_EQ(shape.GetVertex(3).x, -hx); // left
	EXPECT_EQ(shape.GetVertex(3).y, hy); // top
	EXPECT_EQ(shape.GetNormal(3).x, -1);
	EXPECT_EQ(shape.GetNormal(3).y, 0);
}

TEST(PolygonShape, SetPoints)
{
	PolygonShape shape{};
	const auto points = std::vector<Vec2>{{ Vec2{-1, +2}, Vec2{+3, +3}, Vec2{+2, -1}, Vec2{-1, -2}, Vec2{-4, -1} }};
	shape.Set(&points[0], static_cast<PolygonShape::vertex_count_t>(points.size()));
	
	ASSERT_EQ(shape.GetVertexCount(), PolygonShape::vertex_count_t(5));

	// vertices go counter-clockwise...

	EXPECT_EQ(shape.GetVertex(0).x, points[1].x);
	EXPECT_EQ(shape.GetVertex(0).y, points[1].y);
	EXPECT_EQ(shape.GetVertex(1).x, points[0].x);
	EXPECT_EQ(shape.GetVertex(1).y, points[0].y);
	EXPECT_EQ(shape.GetVertex(2).x, points[4].x);
	EXPECT_EQ(shape.GetVertex(2).y, points[4].y);
	EXPECT_EQ(shape.GetVertex(3).x, points[3].x);
	EXPECT_EQ(shape.GetVertex(3).y, points[3].y);
	EXPECT_EQ(shape.GetVertex(4).x, points[2].x);
	EXPECT_EQ(shape.GetVertex(4).y, points[2].y);
}