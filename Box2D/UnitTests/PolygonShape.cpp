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
#include <Box2D/Collision/Shapes/PolygonShape.h>

using namespace box2d;

TEST(PolygonShape, ByteSizeIs272)
{
	EXPECT_EQ(sizeof(PolygonShape), size_t(272));
}

TEST(PolygonShape, DefaultConstruction)
{
	PolygonShape shape;
	EXPECT_EQ(shape.GetType(), Shape::e_polygon);
	EXPECT_EQ(shape.GetVertexCount(), 0);
	EXPECT_EQ(shape.GetCentroid().x, float_t(0));
	EXPECT_EQ(shape.GetCentroid().y, float_t(0));
	EXPECT_EQ(GetChildCount(shape), child_count_t(1));
	EXPECT_EQ(GetRadius(shape), PolygonRadius);
}

TEST(PolygonShape, FindLowestRightMostVertex)
{
	Vec2 vertices[4];
	
	vertices[0] = Vec2{0, +1};
	vertices[1] = Vec2{-1, -2};
	vertices[2] = Vec2{+3, -4};
	vertices[3] = Vec2{+2, +2};

	const auto index = FindLowestRightMostVertex(vertices);
	
	EXPECT_EQ(index, size_t(2));
}

TEST(PolygonShape, BoxConstruction)
{
	const auto hx = float_t(2.3);
	const auto hy = float_t(54.1);
	const auto shape = PolygonShape{hx, hy};
	EXPECT_EQ(shape.GetType(), Shape::e_polygon);
	EXPECT_EQ(shape.GetCentroid().x, float_t(0));
	EXPECT_EQ(shape.GetCentroid().y, float_t(0));
	EXPECT_EQ(GetChildCount(shape), child_count_t(1));
	EXPECT_EQ(GetRadius(shape), PolygonRadius);

	ASSERT_EQ(shape.GetVertexCount(), PolygonShape::vertex_count_t(4));
	
	// vertices go counter-clockwise from lowest right-most (and normals follow their edges)...

	EXPECT_EQ(shape.GetVertex(0).x, hx); // right
	EXPECT_EQ(shape.GetVertex(0).y, -hy); // bottom
	EXPECT_EQ(shape.GetVertex(1).x, hx); // right
	EXPECT_EQ(shape.GetVertex(1).y, hy); // top
	EXPECT_EQ(shape.GetVertex(2).x, -hx); // left
	EXPECT_EQ(shape.GetVertex(2).y, hy); // top
	EXPECT_EQ(shape.GetVertex(3).x, -hx); // left
	EXPECT_EQ(shape.GetVertex(3).y, -hy); // bottom

	EXPECT_EQ(shape.GetNormal(0).x, +1);
	EXPECT_EQ(shape.GetNormal(0).y, 0);
	EXPECT_EQ(shape.GetNormal(1).x, 0);
	EXPECT_EQ(shape.GetNormal(1).y, +1);
	EXPECT_EQ(shape.GetNormal(2).x, -1);
	EXPECT_EQ(shape.GetNormal(2).y, 0);
	EXPECT_EQ(shape.GetNormal(3).x, 0);
	EXPECT_EQ(shape.GetNormal(3).y, -1);
}

TEST(PolygonShape, SetAsBox)
{
	const auto hx = float_t(2.3);
	const auto hy = float_t(54.1);
	PolygonShape shape;
	shape.SetAsBox(hx, hy);
	EXPECT_EQ(shape.GetType(), Shape::e_polygon);
	EXPECT_EQ(shape.GetCentroid().x, float_t(0));
	EXPECT_EQ(shape.GetCentroid().y, float_t(0));
	EXPECT_EQ(GetChildCount(shape), child_count_t(1));
	EXPECT_EQ(GetRadius(shape), PolygonRadius);
	
	ASSERT_EQ(shape.GetVertexCount(), PolygonShape::vertex_count_t(4));
	
	// vertices go counter-clockwise from lowest right-most (and normals follow their edges)...
	
	EXPECT_EQ(shape.GetVertex(0).x, hx); // right
	EXPECT_EQ(shape.GetVertex(0).y, -hy); // bottom
	EXPECT_EQ(shape.GetVertex(1).x, hx); // right
	EXPECT_EQ(shape.GetVertex(1).y, hy); // top
	EXPECT_EQ(shape.GetVertex(2).x, -hx); // left
	EXPECT_EQ(shape.GetVertex(2).y, hy); // top
	EXPECT_EQ(shape.GetVertex(3).x, -hx); // left
	EXPECT_EQ(shape.GetVertex(3).y, -hy); // bottom
	
	EXPECT_EQ(shape.GetNormal(0).x, +1);
	EXPECT_EQ(shape.GetNormal(0).y, 0);
	EXPECT_EQ(shape.GetNormal(1).x, 0);
	EXPECT_EQ(shape.GetNormal(1).y, +1);
	EXPECT_EQ(shape.GetNormal(2).x, -1);
	EXPECT_EQ(shape.GetNormal(2).y, 0);
	EXPECT_EQ(shape.GetNormal(3).x, 0);
	EXPECT_EQ(shape.GetNormal(3).y, -1);
}

TEST(PolygonShape, SetAsZeroCenteredRotatedBox)
{
	const auto hx = float_t(2.3);
	const auto hy = float_t(54.1);
	PolygonShape shape;
	shape.SetAsBox(hx, hy, Vec2_zero, 0);
	EXPECT_EQ(shape.GetType(), Shape::e_polygon);
	EXPECT_EQ(shape.GetCentroid().x, float_t(0));
	EXPECT_EQ(shape.GetCentroid().y, float_t(0));
	EXPECT_EQ(GetChildCount(shape), child_count_t(1));
	EXPECT_EQ(GetRadius(shape), PolygonRadius);
	
	ASSERT_EQ(shape.GetVertexCount(), PolygonShape::vertex_count_t(4));
	
	// vertices go counter-clockwise from lowest right-most (and normals follow their edges)...
	
	EXPECT_EQ(shape.GetVertex(0).x, hx); // right
	EXPECT_EQ(shape.GetVertex(0).y, -hy); // bottom
	EXPECT_EQ(shape.GetVertex(1).x, hx); // right
	EXPECT_EQ(shape.GetVertex(1).y, hy); // top
	EXPECT_EQ(shape.GetVertex(2).x, -hx); // left
	EXPECT_EQ(shape.GetVertex(2).y, hy); // top
	EXPECT_EQ(shape.GetVertex(3).x, -hx); // left
	EXPECT_EQ(shape.GetVertex(3).y, -hy); // bottom
	
	EXPECT_EQ(shape.GetNormal(0).x, +1);
	EXPECT_EQ(shape.GetNormal(0).y, 0);
	EXPECT_EQ(shape.GetNormal(1).x, 0);
	EXPECT_EQ(shape.GetNormal(1).y, +1);
	EXPECT_EQ(shape.GetNormal(2).x, -1);
	EXPECT_EQ(shape.GetNormal(2).y, 0);
	EXPECT_EQ(shape.GetNormal(3).x, 0);
	EXPECT_EQ(shape.GetNormal(3).y, -1);
}

TEST(PolygonShape, SetAsCenteredBox)
{
	const auto hx = float_t(2.3);
	const auto hy = float_t(54.1);
	PolygonShape shape;
	const auto x_off = float_t(10.2);
	const auto y_off = float_t(-5);
	shape.SetAsBox(hx, hy, Vec2(x_off, y_off), 0);
	EXPECT_EQ(shape.GetType(), Shape::e_polygon);
	EXPECT_EQ(shape.GetCentroid().x, x_off);
	EXPECT_EQ(shape.GetCentroid().y, y_off);
	EXPECT_EQ(GetChildCount(shape), child_count_t(1));
	EXPECT_EQ(GetRadius(shape), PolygonRadius);
	
	ASSERT_EQ(shape.GetVertexCount(), PolygonShape::vertex_count_t(4));
	
	// vertices go counter-clockwise from lowest right-most (and normals follow their edges)...
	
	EXPECT_EQ(shape.GetVertex(0).x, hx + x_off); // right
	EXPECT_EQ(shape.GetVertex(0).y, -hy + y_off); // bottom
	EXPECT_EQ(shape.GetVertex(1).x, hx + x_off); // right
	EXPECT_EQ(shape.GetVertex(1).y, hy + y_off); // top
	EXPECT_EQ(shape.GetVertex(2).x, -hx + x_off); // left
	EXPECT_EQ(shape.GetVertex(2).y, hy + y_off); // top
	EXPECT_EQ(shape.GetVertex(3).x, -hx + x_off); // left
	EXPECT_EQ(shape.GetVertex(3).y, -hy + y_off); // bottom
	
	EXPECT_EQ(shape.GetNormal(0).x, +1);
	EXPECT_EQ(shape.GetNormal(0).y, 0);
	EXPECT_EQ(shape.GetNormal(1).x, 0);
	EXPECT_EQ(shape.GetNormal(1).y, +1);
	EXPECT_EQ(shape.GetNormal(2).x, -1);
	EXPECT_EQ(shape.GetNormal(2).y, 0);
	EXPECT_EQ(shape.GetNormal(3).x, 0);
	EXPECT_EQ(shape.GetNormal(3).y, -1);
}

TEST(PolygonShape, SetAsBoxAngledDegrees90)
{
	const auto hx = float_t(2.3);
	const auto hy = float_t(54.1);
	PolygonShape shape;
	const auto angle = DegreesToRadians(90);
	shape.SetAsBox(hx, hy, Vec2_zero, angle);

	EXPECT_EQ(shape.GetType(), Shape::e_polygon);
	EXPECT_EQ(shape.GetCentroid().x, float_t(0));
	EXPECT_EQ(shape.GetCentroid().y, float_t(0));
	EXPECT_EQ(GetChildCount(shape), child_count_t(1));
	EXPECT_EQ(GetRadius(shape), PolygonRadius);
	
	ASSERT_EQ(shape.GetVertexCount(), PolygonShape::vertex_count_t(4));
	
	// vertices go counter-clockwise from lowest right-most (and normals follow their edges)...
	
	const auto precision = 1000u;
	EXPECT_FLOAT_EQ(round(shape.GetVertex(0).x, precision), hy); // right
	EXPECT_FLOAT_EQ(round(shape.GetVertex(0).y, precision), -hx); // bottom
	EXPECT_FLOAT_EQ(round(shape.GetVertex(1).x, precision), hy); // right
	EXPECT_FLOAT_EQ(round(shape.GetVertex(1).y, precision), hx); // top
	EXPECT_FLOAT_EQ(round(shape.GetVertex(2).x, precision), -hy); // left
	EXPECT_FLOAT_EQ(round(shape.GetVertex(2).y, precision), hx); // top
	EXPECT_FLOAT_EQ(round(shape.GetVertex(3).x, precision), -hy); // left
	EXPECT_FLOAT_EQ(round(shape.GetVertex(3).y, precision), -hx); // bottom
	
	EXPECT_GE(shape.GetVertex(0).x, shape.GetVertex(3).x);
	if (shape.GetVertex(0).x == shape.GetVertex(3).x)
	{
		EXPECT_LT(shape.GetVertex(0).y, shape.GetVertex(3).y);
	}
	
	EXPECT_FLOAT_EQ(round(shape.GetNormal(0).x), +1);
	EXPECT_FLOAT_EQ(round(shape.GetNormal(0).y), 0);

	EXPECT_FLOAT_EQ(round(shape.GetNormal(1).x), 0);
	EXPECT_FLOAT_EQ(round(shape.GetNormal(1).y), +1);
	
	EXPECT_FLOAT_EQ(round(shape.GetNormal(2).x), -1);
	EXPECT_FLOAT_EQ(round(shape.GetNormal(2).y), 0);

	EXPECT_FLOAT_EQ(round(shape.GetNormal(3).x), 0);
	EXPECT_FLOAT_EQ(round(shape.GetNormal(3).y), -1);	
}

TEST(PolygonShape, SetPoints)
{
	PolygonShape shape;
	const auto points = Span<const Vec2>{ Vec2{-1, +2}, Vec2{+3, +3}, Vec2{+2, -1}, Vec2{-1, -2}, Vec2{-4, -1} };
	shape.Set(points);
	
	ASSERT_EQ(shape.GetVertexCount(), PolygonShape::vertex_count_t(5));

	// vertices go counter-clockwise from lowest right-most...

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