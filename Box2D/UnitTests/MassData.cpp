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
#include <Box2D/Collision/MassData.hpp>
#include <Box2D/Collision/Shapes/CircleShape.hpp>
#include <Box2D/Collision/Shapes/PolygonShape.hpp>
#include <Box2D/Collision/Shapes/EdgeShape.hpp>

using namespace box2d;

TEST(MassData, ByteSizeIs16)
{
	EXPECT_EQ(sizeof(MassData), size_t(16));
}

TEST(MassData, GetForZeroVertexRadiusCircle)
{
	const auto shape = CircleShape(0);
	const auto mass_data = GetMassData(shape, 1);
	EXPECT_EQ(mass_data.mass, 0);
	EXPECT_EQ(mass_data.I, 0);
	EXPECT_EQ(mass_data.center.x, 0);
	EXPECT_EQ(mass_data.center.y, 0);
}

TEST(MassData, GetForOriginCenteredCircle)
{
	const auto radius = float_t(1);
	const auto position = Vec2{0, 0};
	const auto foo = CircleShape{radius, position};
	const auto density = float_t(1);
	const auto mass_data = GetMassData(foo, density);
	EXPECT_EQ(mass_data.mass, Pi);
	EXPECT_FLOAT_EQ(mass_data.I, float_t(1.5707964));
	EXPECT_FLOAT_EQ(mass_data.I, density * (Pi * Square(radius) * Square(radius) / 2));
	EXPECT_EQ(mass_data.center, position);
}

TEST(MassData, GetForCircle)
{
	const auto radius = float_t(1);
	const auto position = Vec2{-1, 1};
	const auto foo = CircleShape{radius, position};
	const auto density = float_t(1);
	const auto mass_data = GetMassData(foo, density);
	EXPECT_EQ(mass_data.mass, Pi);
	EXPECT_FLOAT_EQ(mass_data.I, float_t(7.85398));
	EXPECT_EQ(mass_data.center, position);
}

TEST(MassData, GetForZeroVertexRadiusRectangle)
{
	auto shape = PolygonShape(0);
	shape.SetAsBox(4, 1);
	ASSERT_EQ(shape.GetCentroid().x, float_t(0));
	ASSERT_EQ(shape.GetCentroid().y, float_t(0));
	const auto density = float_t(2.1);
	const auto mass_data = GetMassData(shape, density);
	EXPECT_FLOAT_EQ(mass_data.mass, float_t((8 * 2) * density));
	EXPECT_FLOAT_EQ(mass_data.I, float_t(90.666664 * density));
	EXPECT_FLOAT_EQ(mass_data.center.x, shape.GetCentroid().x);
	EXPECT_FLOAT_EQ(mass_data.center.y, shape.GetCentroid().y);
	
	// Area moment of inertia (I) for a rectangle is Ix + Iy = (b * h^3) / 12 + (b^3 * h) / 12....
	const auto i = 8.0 * 2.0 * 2.0 * 2.0 / 12.0 + 8.0 * 8.0 * 8.0 * 2.0 / 12.0;
	EXPECT_FLOAT_EQ(mass_data.I, density * float_t(i));
	
	const auto i_z = GetPolarMomentOfPolygon(shape.GetVertices());
	EXPECT_FLOAT_EQ(mass_data.I, density * i_z);
	
	EXPECT_FLOAT_EQ(GetAreaOfPolygon(shape.GetVertices()), float_t(16));
}

TEST(MassData, GetForZeroVertexRadiusEdge)
{
	const auto v1 = Vec2{-1, 0};
	const auto v2 = Vec2{+1, 0};
	auto shape = EdgeShape(0);
	shape.Set(v1, v2);
	const auto density = float_t(2.1);
	const auto mass_data = GetMassData(shape, density);
	EXPECT_EQ(mass_data.mass, 0);
	EXPECT_EQ(mass_data.I, 0);
	EXPECT_EQ(mass_data.center.x, 0);
	EXPECT_EQ(mass_data.center.y, 0);
}

TEST(MassData, GetForSamePointedEdgeIsSameAsCircle)
{
	const auto v1 = Vec2{-1, 1};
	auto shape = EdgeShape(1);
	shape.Set(v1, v1);
	const auto density = float_t(1);
	const auto mass_data = GetMassData(shape, density);
	
	const auto circleMass = density * Pi * Square(shape.GetVertexRadius());

	EXPECT_FLOAT_EQ(mass_data.mass, circleMass);
	EXPECT_FLOAT_EQ(mass_data.I, float_t(7.85398));
	EXPECT_FLOAT_EQ(mass_data.center.x, v1.x);
	EXPECT_FLOAT_EQ(mass_data.center.y, v1.y);
}

TEST(MassData, GetForCenteredEdge)
{
	const auto v1 = Vec2{-2, 0};
	const auto v2 = Vec2{+2, 0};
	const auto radius = float_t(0.5);
	auto shape = EdgeShape(radius);
	shape.Set(v1, v2);
	const auto density = float_t(2.1);
	const auto mass_data = GetMassData(shape, density);
	
	const auto vertices = Span<const Vec2>{Vec2(-2, +0.5), Vec2(-2, -0.5), Vec2(+2, -0.5), Vec2(+2, +0.5)};
	const auto area = GetAreaOfPolygon(vertices) + GetAreaOfCircle(radius);
	EXPECT_EQ(mass_data.mass, density * area);

	EXPECT_FLOAT_EQ(mass_data.I, float_t(18.70351));
	EXPECT_FLOAT_EQ(GetPolarMomentOfPolygon(vertices), float_t(5.6666665));
	EXPECT_GT(mass_data.I, GetPolarMomentOfPolygon(vertices) * density);
	
	EXPECT_EQ(mass_data.center.x, 0);
	EXPECT_EQ(mass_data.center.y, 0);
}
