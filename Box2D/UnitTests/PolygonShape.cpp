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
#include <Box2D/Collision/Shapes/PolygonShape.hpp>

using namespace box2d;

TEST(PolygonShape, ByteSizeIs276)
{
	EXPECT_EQ(sizeof(PolygonShape), size_t(276));
}

TEST(PolygonShape, DefaultConstruction)
{
	PolygonShape shape;
	EXPECT_EQ(shape.GetType(), Shape::e_polygon);
	EXPECT_EQ(shape.GetVertexCount(), 0);
	EXPECT_EQ(shape.GetCentroid(), Vec2(0, 0));
	EXPECT_EQ(GetChildCount(shape), child_count_t(1));
	EXPECT_EQ(GetVertexRadius(shape), PolygonShape::GetDefaultVertexRadius());
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
	EXPECT_EQ(shape.GetCentroid(), Vec2(0, 0));
	EXPECT_EQ(GetChildCount(shape), child_count_t(1));
	EXPECT_EQ(GetVertexRadius(shape), PolygonShape::GetDefaultVertexRadius());

	ASSERT_EQ(shape.GetVertexCount(), PolygonShape::vertex_count_t(4));
	
	// vertices go counter-clockwise from lowest right-most (and normals follow their edges)...

	EXPECT_EQ(shape.GetVertex(0), Vec2(hx, -hy)); // bottom right
	EXPECT_EQ(shape.GetVertex(1), Vec2(hx, hy)); // top right
	EXPECT_EQ(shape.GetVertex(2), Vec2(-hx, hy)); // top left
	EXPECT_EQ(shape.GetVertex(3), Vec2(-hx, -hy)); // bottom left

	EXPECT_EQ(shape.GetNormal(0) * 1, Vec2(+1, 0));
	EXPECT_EQ(shape.GetNormal(1) * 1, Vec2(0, +1));
	EXPECT_EQ(shape.GetNormal(2) * 1, Vec2(-1, 0));
	EXPECT_EQ(shape.GetNormal(3) * 1, Vec2(0, -1));
}

TEST(PolygonShape, Copy)
{
	const auto hx = float_t(2.3);
	const auto hy = float_t(54.1);
	
	auto shape = PolygonShape{hx, hy};
	ASSERT_EQ(shape.GetType(), Shape::e_polygon);
	ASSERT_EQ(shape.GetCentroid(), Vec2(0, 0));
	ASSERT_EQ(GetChildCount(shape), child_count_t(1));
	ASSERT_EQ(GetVertexRadius(shape), PolygonShape::GetDefaultVertexRadius());
	ASSERT_EQ(shape.GetVertexCount(), PolygonShape::vertex_count_t(4));
	
	// vertices go counter-clockwise from lowest right-most (and normals follow their edges)...
	ASSERT_EQ(shape.GetVertex(0), Vec2(hx, -hy)); // bottom right
	ASSERT_EQ(shape.GetVertex(1), Vec2(hx, hy)); // top right
	ASSERT_EQ(shape.GetVertex(2), Vec2(-hx, hy)); // top left
	ASSERT_EQ(shape.GetVertex(3), Vec2(-hx, -hy)); // bottom left
	
	ASSERT_EQ(shape.GetNormal(0) * 1, Vec2(+1, 0));
	ASSERT_EQ(shape.GetNormal(1) * 1, Vec2(0, +1));
	ASSERT_EQ(shape.GetNormal(2) * 1, Vec2(-1, 0));
	ASSERT_EQ(shape.GetNormal(3) * 1, Vec2(0, -1));

	const auto copy = shape;
	
	EXPECT_EQ(copy.GetType(), Shape::e_polygon);
	EXPECT_EQ(copy.GetCentroid(), Vec2(0, 0));
	EXPECT_EQ(GetChildCount(copy), child_count_t(1));
	EXPECT_EQ(GetVertexRadius(copy), PolygonShape::GetDefaultVertexRadius());
	
	ASSERT_EQ(copy.GetVertexCount(), PolygonShape::vertex_count_t(4));
	
	// vertices go counter-clockwise from lowest right-most (and normals follow their edges)...
	
	EXPECT_EQ(copy.GetVertex(0), Vec2(hx, -hy)); // bottom right
	EXPECT_EQ(copy.GetVertex(1), Vec2(hx, hy)); // top right
	EXPECT_EQ(copy.GetVertex(2), Vec2(-hx, hy)); // top left
	EXPECT_EQ(copy.GetVertex(3), Vec2(-hx, -hy)); // bottom left
	
	EXPECT_EQ(copy.GetNormal(0) * 1, Vec2(+1, 0));
	EXPECT_EQ(copy.GetNormal(1) * 1, Vec2(0, +1));
	EXPECT_EQ(copy.GetNormal(2) * 1, Vec2(-1, 0));
	EXPECT_EQ(copy.GetNormal(3) * 1, Vec2(0, -1));
}

TEST(PolygonShape, Translate)
{
	const auto hx = float_t(2.3);
	const auto hy = float_t(54.1);
	
	auto shape = PolygonShape{hx, hy};
	ASSERT_EQ(shape.GetType(), Shape::e_polygon);
	ASSERT_EQ(shape.GetCentroid(), Vec2(0, 0));
	ASSERT_EQ(GetChildCount(shape), child_count_t(1));
	ASSERT_EQ(GetVertexRadius(shape), PolygonShape::GetDefaultVertexRadius());
	ASSERT_EQ(shape.GetVertexCount(), PolygonShape::vertex_count_t(4));
	
	// vertices go counter-clockwise from lowest right-most (and normals follow their edges)...
	ASSERT_EQ(shape.GetVertex(0), Vec2(hx, -hy)); // bottom right
	ASSERT_EQ(shape.GetVertex(1), Vec2(hx, hy)); // top right
	ASSERT_EQ(shape.GetVertex(2), Vec2(-hx, hy)); // top left
	ASSERT_EQ(shape.GetVertex(3), Vec2(-hx, -hy)); // bottom left
	
	ASSERT_EQ(shape.GetNormal(0) * 1, Vec2(+1, 0));
	ASSERT_EQ(shape.GetNormal(1) * 1, Vec2(0, +1));
	ASSERT_EQ(shape.GetNormal(2) * 1, Vec2(-1, 0));
	ASSERT_EQ(shape.GetNormal(3) * 1, Vec2(0, -1));
	
	const auto new_ctr = Vec2{-3, 67};
	shape.Transform(Transformation{new_ctr, UnitVec2{0_deg}});
	
	EXPECT_EQ(shape.GetType(), Shape::e_polygon);
	EXPECT_EQ(shape.GetCentroid(), new_ctr);
	EXPECT_EQ(GetChildCount(shape), child_count_t(1));
	EXPECT_EQ(GetVertexRadius(shape), PolygonShape::GetDefaultVertexRadius());

	ASSERT_EQ(shape.GetVertexCount(), PolygonShape::vertex_count_t(4));

	EXPECT_EQ(shape.GetVertex(0), Vec2(hx, -hy) + new_ctr); // bottom right
	EXPECT_EQ(shape.GetVertex(1), Vec2(hx, hy) + new_ctr); // top right
	EXPECT_EQ(shape.GetVertex(2), Vec2(-hx, hy) + new_ctr); // top left
	EXPECT_EQ(shape.GetVertex(3), Vec2(-hx, -hy) + new_ctr); // bottom left

	EXPECT_EQ(shape.GetNormal(0) * 1, Vec2(+1, 0));
	EXPECT_EQ(shape.GetNormal(1) * 1, Vec2(0, +1));
	EXPECT_EQ(shape.GetNormal(2) * 1, Vec2(-1, 0));
	EXPECT_EQ(shape.GetNormal(3) * 1, Vec2(0, -1));
}

TEST(PolygonShape, SetAsBox)
{
	const auto hx = float_t(2.3);
	const auto hy = float_t(54.1);
	PolygonShape shape;
	shape.SetAsBox(hx, hy);
	EXPECT_EQ(shape.GetType(), Shape::e_polygon);
	EXPECT_EQ(shape.GetCentroid(), Vec2(0, 0));
	EXPECT_EQ(GetChildCount(shape), child_count_t(1));
	EXPECT_EQ(GetVertexRadius(shape), PolygonShape::GetDefaultVertexRadius());
	
	ASSERT_EQ(shape.GetVertexCount(), PolygonShape::vertex_count_t(4));
	
	// vertices go counter-clockwise from lowest right-most (and normals follow their edges)...
	
	EXPECT_EQ(shape.GetVertex(0), Vec2(hx, -hy)); // bottom right
	EXPECT_EQ(shape.GetVertex(1), Vec2(hx, hy)); // top right
	EXPECT_EQ(shape.GetVertex(2), Vec2(-hx, hy)); // top left
	EXPECT_EQ(shape.GetVertex(3), Vec2(-hx, -hy)); // bottom left
	
	EXPECT_EQ(shape.GetNormal(0) * 1, Vec2(+1, 0));
	EXPECT_EQ(shape.GetNormal(1) * 1, Vec2(0, +1));
	EXPECT_EQ(shape.GetNormal(2) * 1, Vec2(-1, 0));
	EXPECT_EQ(shape.GetNormal(3) * 1, Vec2(0, -1));
}

TEST(PolygonShape, SetAsZeroCenteredRotatedBox)
{
	const auto hx = float_t(2.3);
	const auto hy = float_t(54.1);
	PolygonShape shape;
	SetAsBox(shape, hx, hy, Vec2_zero, 0_deg);
	EXPECT_EQ(shape.GetType(), Shape::e_polygon);
	EXPECT_EQ(shape.GetCentroid(), Vec2(0, 0));
	EXPECT_EQ(GetChildCount(shape), child_count_t(1));
	EXPECT_EQ(GetVertexRadius(shape), PolygonShape::GetDefaultVertexRadius());
	
	ASSERT_EQ(shape.GetVertexCount(), PolygonShape::vertex_count_t(4));
	
	// vertices go counter-clockwise from lowest right-most (and normals follow their edges)...
	
	EXPECT_EQ(shape.GetVertex(0), Vec2(hx, -hy)); // bottom right
	EXPECT_EQ(shape.GetVertex(1), Vec2(hx, hy)); // top right
	EXPECT_EQ(shape.GetVertex(2), Vec2(-hx, hy)); // top left
	EXPECT_EQ(shape.GetVertex(3), Vec2(-hx, -hy)); // bottom left
	
	EXPECT_EQ(shape.GetNormal(0) * 1, Vec2(+1, 0));
	EXPECT_EQ(shape.GetNormal(1) * 1, Vec2(0, +1));
	EXPECT_EQ(shape.GetNormal(2) * 1, Vec2(-1, 0));
	EXPECT_EQ(shape.GetNormal(3) * 1, Vec2(0, -1));
}

TEST(PolygonShape, SetAsCenteredBox)
{
	const auto hx = float_t(2.3);
	const auto hy = float_t(54.1);
	PolygonShape shape;
	const auto x_off = float_t(10.2);
	const auto y_off = float_t(-5);
	SetAsBox(shape, hx, hy, Vec2(x_off, y_off), 0_deg);
	EXPECT_EQ(shape.GetType(), Shape::e_polygon);
	EXPECT_EQ(shape.GetCentroid(), Vec2(x_off, y_off));
	EXPECT_EQ(GetChildCount(shape), child_count_t(1));
	EXPECT_EQ(GetVertexRadius(shape), PolygonShape::GetDefaultVertexRadius());
	
	ASSERT_EQ(shape.GetVertexCount(), PolygonShape::vertex_count_t(4));
	
	// vertices go counter-clockwise from lowest right-most (and normals follow their edges)...
	
	EXPECT_EQ(shape.GetVertex(0), Vec2(hx + x_off, -hy + y_off)); // bottom right
	EXPECT_EQ(shape.GetVertex(1), Vec2(hx + x_off, hy + y_off)); // top right
	EXPECT_EQ(shape.GetVertex(2), Vec2(-hx + x_off, hy + y_off)); // top left
	EXPECT_EQ(shape.GetVertex(3), Vec2(-hx + x_off, -hy + y_off)); // bottom left
	
	EXPECT_EQ(shape.GetNormal(0) * 1, Vec2(+1, 0));
	EXPECT_EQ(shape.GetNormal(1) * 1, Vec2(0, +1));
	EXPECT_EQ(shape.GetNormal(2) * 1, Vec2(-1, 0));
	EXPECT_EQ(shape.GetNormal(3) * 1, Vec2(0, -1));
}

TEST(PolygonShape, SetAsBoxAngledDegrees90)
{
	const auto hx = float_t(2.3);
	const auto hy = float_t(54.1);
	PolygonShape shape;
	const auto angle = 90_deg;
	SetAsBox(shape, hx, hy, Vec2_zero, angle);

	EXPECT_EQ(shape.GetType(), Shape::e_polygon);
	EXPECT_EQ(shape.GetCentroid().x, float_t(0));
	EXPECT_EQ(shape.GetCentroid().y, float_t(0));
	EXPECT_EQ(GetChildCount(shape), child_count_t(1));
	EXPECT_EQ(GetVertexRadius(shape), PolygonShape::GetDefaultVertexRadius());
	
	ASSERT_EQ(shape.GetVertexCount(), PolygonShape::vertex_count_t(4));
	
	// vertices go counter-clockwise (and normals follow their edges)...
	
	const auto precision = 1000u;
	EXPECT_FLOAT_EQ(round(shape.GetVertex(0).x, precision), hy); // right
	EXPECT_FLOAT_EQ(round(shape.GetVertex(0).y, precision), hx); // top
	EXPECT_FLOAT_EQ(round(shape.GetVertex(1).x, precision), -hy); // left
	EXPECT_FLOAT_EQ(round(shape.GetVertex(1).y, precision), hx); // top
	EXPECT_FLOAT_EQ(round(shape.GetVertex(2).x, precision), -hy); // left
	EXPECT_FLOAT_EQ(round(shape.GetVertex(2).y, precision), -hx); // bottom
	EXPECT_FLOAT_EQ(round(shape.GetVertex(3).x, precision), hy); // right
	EXPECT_FLOAT_EQ(round(shape.GetVertex(3).y, precision), -hx); // bottom
	
	EXPECT_FLOAT_EQ(round(shape.GetNormal(0).GetX()), 0);
	EXPECT_FLOAT_EQ(round(shape.GetNormal(0).GetY()), +1);
	
	EXPECT_FLOAT_EQ(round(shape.GetNormal(1).GetX()), -1);
	EXPECT_FLOAT_EQ(round(shape.GetNormal(1).GetY()), 0);

	EXPECT_FLOAT_EQ(round(shape.GetNormal(2).GetX()), 0);
	EXPECT_FLOAT_EQ(round(shape.GetNormal(2).GetY()), -1);
	
	EXPECT_FLOAT_EQ(round(shape.GetNormal(3).GetX()), +1);
	EXPECT_FLOAT_EQ(round(shape.GetNormal(3).GetY()), 0);
}

TEST(PolygonShape, SetPoints)
{
	PolygonShape shape;
	const auto points = Span<const Vec2>{ Vec2{-1, +2}, Vec2{+3, +3}, Vec2{+2, -1}, Vec2{-1, -2}, Vec2{-4, -1} };
	shape.Set(points);
	
	ASSERT_EQ(shape.GetVertexCount(), PolygonShape::vertex_count_t(5));

	// vertices go counter-clockwise from lowest right-most...

	EXPECT_EQ(shape.GetVertex(0), points[1]);
	EXPECT_EQ(shape.GetVertex(1), points[0]);
	EXPECT_EQ(shape.GetVertex(2), points[4]);
	EXPECT_EQ(shape.GetVertex(3), points[3]);
	EXPECT_EQ(shape.GetVertex(4), points[2]);
}

TEST(PolygonShape, CanSetTwoPoints)
{
	const auto points = Span<const Vec2>{Vec2{-1, +0}, Vec2{+1, +0}};
	const auto vertexRadius = float_t(2);
	PolygonShape shape(vertexRadius);
	shape.Set(points);
	EXPECT_EQ(shape.GetVertexCount(), static_cast<PolygonShape::vertex_count_t>(points.size()));
	EXPECT_EQ(shape.GetVertex(0), points[1]);
	EXPECT_EQ(shape.GetVertex(1), points[0]);
	EXPECT_EQ(Vec2(shape.GetNormal(0)), Vec2(0, +1));
	EXPECT_EQ(Vec2(shape.GetNormal(1)), Vec2(0, -1));
	EXPECT_EQ(shape.GetCentroid(), Average(points));
	EXPECT_EQ(shape.GetVertexRadius(), vertexRadius);
}

TEST(PolygonShape, CanSetOnePoint)
{
	const auto points = Span<const Vec2>{Vec2{0, 0}};
	const auto vertexRadius = float_t(2);
	PolygonShape shape(vertexRadius);
	shape.Set(points);
	EXPECT_EQ(shape.GetVertexCount(), static_cast<PolygonShape::vertex_count_t>(points.size()));
	EXPECT_EQ(shape.GetVertex(0), points[0]);
	EXPECT_FALSE(IsValid(shape.GetNormal(0)));
	EXPECT_EQ(shape.GetCentroid(), points[0]);
	EXPECT_EQ(shape.GetVertexRadius(), vertexRadius);
}
