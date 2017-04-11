/*
 * Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

TEST(PolygonShape, ByteSizeIs_64_80_or_112)
{
	switch (sizeof(RealNum))
	{
		case  4: EXPECT_EQ(sizeof(PolygonShape), size_t(80)); break;
		case  8: EXPECT_EQ(sizeof(PolygonShape), size_t(104)); break;
		case 16: EXPECT_EQ(sizeof(PolygonShape), size_t(160)); break;
		default: FAIL(); break;
	}
}

TEST(PolygonShape, DefaultConstruction)
{
	PolygonShape shape;
	EXPECT_EQ(shape.GetType(), Shape::e_polygon);
	EXPECT_EQ(shape.GetVertexCount(), 0);
	EXPECT_EQ(shape.GetCentroid(), Vec2(0, 0) * Meter);
	EXPECT_EQ(GetChildCount(shape), child_count_t(1));
	EXPECT_EQ(GetVertexRadius(shape), PolygonShape::GetDefaultVertexRadius());
}

TEST(PolygonShape, FindLowestRightMostVertex)
{
	Length2D vertices[4];
	
	vertices[0] = Vec2{0, +1} * Meter;
	vertices[1] = Vec2{-1, -2} * Meter;
	vertices[2] = Vec2{+3, -4} * Meter;
	vertices[3] = Vec2{+2, +2} * Meter;

	const auto index = FindLowestRightMostVertex(vertices);
	
	EXPECT_EQ(index, size_t(2));
}

TEST(PolygonShape, BoxConstruction)
{
	const auto hx = RealNum(2.3) * Meter;
	const auto hy = RealNum(54.1) * Meter;
	const auto shape = PolygonShape{hx, hy};
	EXPECT_EQ(shape.GetType(), Shape::e_polygon);
	EXPECT_EQ(shape.GetCentroid(), Vec2(0, 0) * Meter);
	EXPECT_EQ(GetChildCount(shape), child_count_t(1));
	EXPECT_EQ(GetVertexRadius(shape), PolygonShape::GetDefaultVertexRadius());

	ASSERT_EQ(shape.GetVertexCount(), PolygonShape::vertex_count_t(4));
	
	// vertices go counter-clockwise from lowest right-most (and normals follow their edges)...

	EXPECT_EQ(shape.GetVertex(0), Length2D(hx, -hy)); // bottom right
	EXPECT_EQ(shape.GetVertex(1), Length2D(hx, hy)); // top right
	EXPECT_EQ(shape.GetVertex(2), Length2D(-hx, hy)); // top left
	EXPECT_EQ(shape.GetVertex(3), Length2D(-hx, -hy)); // bottom left

	EXPECT_EQ(shape.GetNormal(0) * RealNum{1}, Vec2(+1, 0));
	EXPECT_EQ(shape.GetNormal(1) * RealNum{1}, Vec2(0, +1));
	EXPECT_EQ(shape.GetNormal(2) * RealNum{1}, Vec2(-1, 0));
	EXPECT_EQ(shape.GetNormal(3) * RealNum{1}, Vec2(0, -1));
}

TEST(PolygonShape, Copy)
{
	const auto hx = RealNum(2.3) * Meter;
	const auto hy = RealNum(54.1) * Meter;
	
	auto shape = PolygonShape{hx, hy};
	ASSERT_EQ(shape.GetType(), Shape::e_polygon);
	ASSERT_EQ(shape.GetCentroid(), Vec2(0, 0) * Meter);
	ASSERT_EQ(GetChildCount(shape), child_count_t(1));
	ASSERT_EQ(GetVertexRadius(shape), PolygonShape::GetDefaultVertexRadius());
	ASSERT_EQ(shape.GetVertexCount(), PolygonShape::vertex_count_t(4));
	
	// vertices go counter-clockwise from lowest right-most (and normals follow their edges)...
	ASSERT_EQ(shape.GetVertex(0), Length2D(hx, -hy)); // bottom right
	ASSERT_EQ(shape.GetVertex(1), Length2D(hx, hy)); // top right
	ASSERT_EQ(shape.GetVertex(2), Length2D(-hx, hy)); // top left
	ASSERT_EQ(shape.GetVertex(3), Length2D(-hx, -hy)); // bottom left
	
	ASSERT_EQ(shape.GetNormal(0) * RealNum{1}, Vec2(+1, 0));
	ASSERT_EQ(shape.GetNormal(1) * RealNum{1}, Vec2(0, +1));
	ASSERT_EQ(shape.GetNormal(2) * RealNum{1}, Vec2(-1, 0));
	ASSERT_EQ(shape.GetNormal(3) * RealNum{1}, Vec2(0, -1));

	const auto copy = shape;
	
	EXPECT_EQ(copy.GetType(), Shape::e_polygon);
	EXPECT_EQ(copy.GetCentroid(), Vec2(0, 0) * Meter);
	EXPECT_EQ(GetChildCount(copy), child_count_t(1));
	EXPECT_EQ(GetVertexRadius(copy), PolygonShape::GetDefaultVertexRadius());
	
	ASSERT_EQ(copy.GetVertexCount(), PolygonShape::vertex_count_t(4));
	
	// vertices go counter-clockwise from lowest right-most (and normals follow their edges)...
	
	EXPECT_EQ(copy.GetVertex(0), Length2D(hx, -hy)); // bottom right
	EXPECT_EQ(copy.GetVertex(1), Length2D(hx, hy)); // top right
	EXPECT_EQ(copy.GetVertex(2), Length2D(-hx, hy)); // top left
	EXPECT_EQ(copy.GetVertex(3), Length2D(-hx, -hy)); // bottom left
	
	EXPECT_EQ(copy.GetNormal(0) * RealNum{1}, Vec2(+1, 0));
	EXPECT_EQ(copy.GetNormal(1) * RealNum{1}, Vec2(0, +1));
	EXPECT_EQ(copy.GetNormal(2) * RealNum{1}, Vec2(-1, 0));
	EXPECT_EQ(copy.GetNormal(3) * RealNum{1}, Vec2(0, -1));
}

TEST(PolygonShape, Translate)
{
	const auto hx = RealNum(2.3) * Meter;
	const auto hy = RealNum(54.1) * Meter;
	
	auto shape = PolygonShape{hx, hy};
	ASSERT_EQ(shape.GetType(), Shape::e_polygon);
	ASSERT_EQ(shape.GetCentroid(), Vec2(0, 0) * Meter);
	ASSERT_EQ(GetChildCount(shape), child_count_t(1));
	ASSERT_EQ(GetVertexRadius(shape), PolygonShape::GetDefaultVertexRadius());
	ASSERT_EQ(shape.GetVertexCount(), PolygonShape::vertex_count_t(4));
	
	// vertices go counter-clockwise from lowest right-most (and normals follow their edges)...
	ASSERT_EQ(shape.GetVertex(0), Length2D(hx, -hy)); // bottom right
	ASSERT_EQ(shape.GetVertex(1), Length2D(hx, hy)); // top right
	ASSERT_EQ(shape.GetVertex(2), Length2D(-hx, hy)); // top left
	ASSERT_EQ(shape.GetVertex(3), Length2D(-hx, -hy)); // bottom left
	
	ASSERT_EQ(shape.GetNormal(0) * RealNum{1}, Vec2(+1, 0));
	ASSERT_EQ(shape.GetNormal(1) * RealNum{1}, Vec2(0, +1));
	ASSERT_EQ(shape.GetNormal(2) * RealNum{1}, Vec2(-1, 0));
	ASSERT_EQ(shape.GetNormal(3) * RealNum{1}, Vec2(0, -1));
	
	const auto new_ctr = Vec2{-3, 67} * Meter;
	shape.Transform(Transformation{new_ctr, UnitVec2{RealNum{0} * Degree}});
	
	EXPECT_EQ(shape.GetType(), Shape::e_polygon);
	EXPECT_EQ(shape.GetCentroid(), new_ctr);
	EXPECT_EQ(GetChildCount(shape), child_count_t(1));
	EXPECT_EQ(GetVertexRadius(shape), PolygonShape::GetDefaultVertexRadius());

	ASSERT_EQ(shape.GetVertexCount(), PolygonShape::vertex_count_t(4));

	EXPECT_EQ(shape.GetVertex(0), Length2D(hx, -hy) + new_ctr); // bottom right
	EXPECT_EQ(shape.GetVertex(1), Length2D(hx, hy) + new_ctr); // top right
	EXPECT_EQ(shape.GetVertex(2), Length2D(-hx, hy) + new_ctr); // top left
	EXPECT_EQ(shape.GetVertex(3), Length2D(-hx, -hy) + new_ctr); // bottom left

	EXPECT_EQ(shape.GetNormal(0) * RealNum{1}, Vec2(+1, 0));
	EXPECT_EQ(shape.GetNormal(1) * RealNum{1}, Vec2(0, +1));
	EXPECT_EQ(shape.GetNormal(2) * RealNum{1}, Vec2(-1, 0));
	EXPECT_EQ(shape.GetNormal(3) * RealNum{1}, Vec2(0, -1));
}

TEST(PolygonShape, SetAsBox)
{
	const auto hx = RealNum(2.3) * Meter;
	const auto hy = RealNum(54.1) * Meter;
	PolygonShape shape;
	shape.SetAsBox(hx, hy);
	EXPECT_EQ(shape.GetType(), Shape::e_polygon);
	EXPECT_EQ(shape.GetCentroid(), Vec2(0, 0) * Meter);
	EXPECT_EQ(GetChildCount(shape), child_count_t(1));
	EXPECT_EQ(GetVertexRadius(shape), PolygonShape::GetDefaultVertexRadius());
	
	ASSERT_EQ(shape.GetVertexCount(), PolygonShape::vertex_count_t(4));
	
	// vertices go counter-clockwise from lowest right-most (and normals follow their edges)...
	
	EXPECT_EQ(shape.GetVertex(0), Length2D(hx, -hy)); // bottom right
	EXPECT_EQ(shape.GetVertex(1), Length2D(hx, hy)); // top right
	EXPECT_EQ(shape.GetVertex(2), Length2D(-hx, hy)); // top left
	EXPECT_EQ(shape.GetVertex(3), Length2D(-hx, -hy)); // bottom left
	
	EXPECT_EQ(shape.GetNormal(0) * RealNum{1}, Vec2(+1, 0));
	EXPECT_EQ(shape.GetNormal(1) * RealNum{1}, Vec2(0, +1));
	EXPECT_EQ(shape.GetNormal(2) * RealNum{1}, Vec2(-1, 0));
	EXPECT_EQ(shape.GetNormal(3) * RealNum{1}, Vec2(0, -1));
}

TEST(PolygonShape, SetAsZeroCenteredRotatedBox)
{
	const auto hx = RealNum(2.3) * Meter;
	const auto hy = RealNum(54.1) * Meter;
	PolygonShape shape;
	SetAsBox(shape, hx, hy, Vec2_zero * Meter, RealNum{0} * Degree);
	EXPECT_EQ(shape.GetType(), Shape::e_polygon);
	EXPECT_EQ(shape.GetCentroid(), Vec2(0, 0) * Meter);
	EXPECT_EQ(GetChildCount(shape), child_count_t(1));
	EXPECT_EQ(GetVertexRadius(shape), PolygonShape::GetDefaultVertexRadius());
	
	ASSERT_EQ(shape.GetVertexCount(), PolygonShape::vertex_count_t(4));
	
	// vertices go counter-clockwise from lowest right-most (and normals follow their edges)...
	
	EXPECT_EQ(shape.GetVertex(0), Length2D(hx, -hy)); // bottom right
	EXPECT_EQ(shape.GetVertex(1), Length2D(hx, hy)); // top right
	EXPECT_EQ(shape.GetVertex(2), Length2D(-hx, hy)); // top left
	EXPECT_EQ(shape.GetVertex(3), Length2D(-hx, -hy)); // bottom left
	
	EXPECT_EQ(shape.GetNormal(0) * RealNum{1}, Vec2(+1, 0));
	EXPECT_EQ(shape.GetNormal(1) * RealNum{1}, Vec2(0, +1));
	EXPECT_EQ(shape.GetNormal(2) * RealNum{1}, Vec2(-1, 0));
	EXPECT_EQ(shape.GetNormal(3) * RealNum{1}, Vec2(0, -1));
}

TEST(PolygonShape, SetAsCenteredBox)
{
	const auto hx = RealNum(2.3) * Meter;
	const auto hy = RealNum(54.1) * Meter;
	PolygonShape shape;
	const auto x_off = RealNum(10.2) * Meter;
	const auto y_off = RealNum(-5) * Meter;
	SetAsBox(shape, hx, hy, Length2D(x_off, y_off), RealNum{0} * Degree);
	EXPECT_EQ(shape.GetType(), Shape::e_polygon);
	EXPECT_EQ(shape.GetCentroid(), Length2D(x_off, y_off));
	EXPECT_EQ(GetChildCount(shape), child_count_t(1));
	EXPECT_EQ(GetVertexRadius(shape), PolygonShape::GetDefaultVertexRadius());
	
	ASSERT_EQ(shape.GetVertexCount(), PolygonShape::vertex_count_t(4));
	
	// vertices go counter-clockwise from lowest right-most (and normals follow their edges)...
	
	EXPECT_EQ(shape.GetVertex(0), Length2D(hx + x_off, -hy + y_off)); // bottom right
	EXPECT_EQ(shape.GetVertex(1), Length2D(hx + x_off, hy + y_off)); // top right
	EXPECT_EQ(shape.GetVertex(2), Length2D(-hx + x_off, hy + y_off)); // top left
	EXPECT_EQ(shape.GetVertex(3), Length2D(-hx + x_off, -hy + y_off)); // bottom left
	
	EXPECT_EQ(shape.GetNormal(0) * RealNum{1}, Vec2(+1, 0));
	EXPECT_EQ(shape.GetNormal(1) * RealNum{1}, Vec2(0, +1));
	EXPECT_EQ(shape.GetNormal(2) * RealNum{1}, Vec2(-1, 0));
	EXPECT_EQ(shape.GetNormal(3) * RealNum{1}, Vec2(0, -1));
}

TEST(PolygonShape, SetAsBoxAngledDegrees90)
{
	const auto hx = RealNum(2.3);
	const auto hy = RealNum(54.1);
	PolygonShape shape;
	const auto angle = 90.0f * Degree;
	SetAsBox(shape, hx * Meter, hy * Meter, Vec2_zero * Meter, angle);

	EXPECT_EQ(shape.GetType(), Shape::e_polygon);
	EXPECT_EQ(shape.GetCentroid().x, RealNum(0) * Meter);
	EXPECT_EQ(shape.GetCentroid().y, RealNum(0) * Meter);
	EXPECT_EQ(GetChildCount(shape), child_count_t(1));
	EXPECT_EQ(GetVertexRadius(shape), PolygonShape::GetDefaultVertexRadius());
	
	ASSERT_EQ(shape.GetVertexCount(), PolygonShape::vertex_count_t(4));
	
	// vertices go counter-clockwise (and normals follow their edges)...
	
	EXPECT_NEAR(double(shape.GetVertex(0).x / Meter), double( hy), 0.0001); // right
	EXPECT_NEAR(double(shape.GetVertex(0).y / Meter), double( hx), 0.0001); // top
	EXPECT_NEAR(double(shape.GetVertex(1).x / Meter), double(-hy), 0.0001); // left
	EXPECT_NEAR(double(shape.GetVertex(1).y / Meter), double( hx), 0.0001); // top
	EXPECT_NEAR(double(shape.GetVertex(2).x / Meter), double(-hy), 0.0001); // left
	EXPECT_NEAR(double(shape.GetVertex(2).y / Meter), double(-hx), 0.0001); // bottom
	EXPECT_NEAR(double(shape.GetVertex(3).x / Meter), double( hy), 0.0001); // right
	EXPECT_NEAR(double(shape.GetVertex(3).y / Meter), double(-hx), 0.0001); // bottom
	
	EXPECT_NEAR(double(shape.GetNormal(0).GetX()),  0.0, 0.0001);
	EXPECT_NEAR(double(shape.GetNormal(0).GetY()), +1.0, 0.0001);
	
	EXPECT_NEAR(double(shape.GetNormal(1).GetX()), -1.0, 0.000001);
	EXPECT_NEAR(double(shape.GetNormal(1).GetY()),  0.0, 0.000001);

	EXPECT_NEAR(double(shape.GetNormal(2).GetX()),  0.0, 0.000001);
	EXPECT_NEAR(double(shape.GetNormal(2).GetY()), -1.0, 0.000001);
	
	EXPECT_NEAR(double(shape.GetNormal(3).GetX()), +1.0, 0.000001);
	EXPECT_NEAR(double(shape.GetNormal(3).GetY()),  0.0, 0.000001);
}

TEST(PolygonShape, SetPoints)
{
	PolygonShape shape;
	const auto points = Span<const Length2D>{
		Vec2{-1, +2} * Meter,
		Vec2{+3, +3} * Meter,
		Vec2{+2, -1} * Meter,
		Vec2{-1, -2} * Meter,
		Vec2{-4, -1} * Meter
	};
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
	const auto points = Span<const Length2D>{Vec2{-1, +0} * Meter, Vec2{+1, +0} * Meter};
	const auto vertexRadius = RealNum(2) * Meter;
	PolygonShape shape;
	shape.SetVertexRadius(vertexRadius);
	shape.Set(points);
	EXPECT_EQ(shape.GetVertexCount(), static_cast<PolygonShape::vertex_count_t>(points.size()));
	EXPECT_EQ(shape.GetVertex(0), points[1]);
	EXPECT_EQ(shape.GetVertex(1), points[0]);
	EXPECT_EQ(GetVec2(shape.GetNormal(0)), Vec2(0, +1));
	EXPECT_EQ(GetVec2(shape.GetNormal(1)), Vec2(0, -1));
	EXPECT_EQ(shape.GetCentroid(), Average(points));
	EXPECT_EQ(shape.GetVertexRadius(), vertexRadius);
}

TEST(PolygonShape, CanSetOnePoint)
{
	const auto points = Span<const Length2D>{Vec2{0, 0} * Meter};
	const auto vertexRadius = RealNum(2) * Meter;
	PolygonShape shape;
	shape.SetVertexRadius(vertexRadius);
	shape.Set(points);
	EXPECT_EQ(shape.GetVertexCount(), static_cast<PolygonShape::vertex_count_t>(points.size()));
	EXPECT_EQ(shape.GetVertex(0), points[0]);
	EXPECT_FALSE(IsValid(shape.GetNormal(0)));
	EXPECT_EQ(shape.GetCentroid(), points[0]);
	EXPECT_EQ(shape.GetVertexRadius(), vertexRadius);
}
