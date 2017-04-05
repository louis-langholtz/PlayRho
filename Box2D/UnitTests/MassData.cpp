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
#include <Box2D/Collision/MassData.hpp>
#include <Box2D/Collision/Shapes/CircleShape.hpp>
#include <Box2D/Collision/Shapes/PolygonShape.hpp>
#include <Box2D/Collision/Shapes/EdgeShape.hpp>

using namespace box2d;

TEST(MassData, ByteSizeIs_16_32_or_64)
{
	switch (sizeof(RealNum))
	{
		case  4: EXPECT_EQ(sizeof(MassData), size_t(16)); break;
		case  8: EXPECT_EQ(sizeof(MassData), size_t(32)); break;
		case 16: EXPECT_EQ(sizeof(MassData), size_t(64)); break;
		default: FAIL(); break;
	}
}

TEST(MassData, GetForZeroVertexRadiusCircle)
{
	const auto shape = CircleShape(0);
	const auto mass_data = GetMassData(shape, KilogramPerSquareMeter);
	EXPECT_EQ(mass_data.mass, Mass{0});
	EXPECT_EQ(mass_data.I, RotInertia{0});
	EXPECT_EQ(mass_data.center.x, 0);
	EXPECT_EQ(mass_data.center.y, 0);
}

TEST(MassData, GetForOriginCenteredCircle)
{
	auto conf = CircleShape::Conf{};
	conf.vertexRadius = 1;
	conf.location = Vec2{0, 0};
	conf.density = RealNum{1} * KilogramPerSquareMeter;
	const auto foo = CircleShape{conf};
	const auto mass_data = GetMassData(foo, conf.density);
	EXPECT_EQ(RealNum{mass_data.mass / Kilogram}, Pi);
	EXPECT_NEAR(double(mass_data.I / (SquareMeter * Kilogram / SquareRadian)), 1.5707964, 0.0001);
	EXPECT_TRUE(almost_equal(mass_data.I / (SquareMeter * Kilogram / SquareRadian),
							 (conf.density / KilogramPerSquareMeter) * (Square(conf.vertexRadius) * Square(conf.vertexRadius) * Pi / 2)));
	EXPECT_EQ(mass_data.center, conf.location);
}

TEST(MassData, GetForCircle)
{
	const auto radius = RealNum(1);
	const auto position = Vec2{-1, 1};
	const auto density = RealNum{1} * KilogramPerSquareMeter;
	auto conf = CircleShape::Conf{};
	conf.vertexRadius = radius;
	conf.location = position;
	conf.density = density;
	const auto foo = CircleShape{conf};
	const auto mass_data = GetMassData(foo, density);
	EXPECT_EQ(RealNum{mass_data.mass / Kilogram}, Pi);
	EXPECT_NEAR(double(mass_data.I / (SquareMeter * Kilogram / SquareRadian)), 7.85398, 0.0002);
	EXPECT_EQ(mass_data.center, position);
}

TEST(MassData, GetForZeroVertexRadiusRectangle)
{
	const auto density = RealNum{2.1f} * KilogramPerSquareMeter;
	auto conf = PolygonShape::Conf{};
	conf.vertexRadius = 0;
	conf.density = density;
	auto shape = PolygonShape(conf);
	shape.SetAsBox(4, 1);
	ASSERT_EQ(shape.GetCentroid().x, RealNum(0));
	ASSERT_EQ(shape.GetCentroid().y, RealNum(0));
	const auto mass_data = GetMassData(shape, density);
	EXPECT_TRUE(almost_equal(RealNum{mass_data.mass / Kilogram}, RealNum((density / KilogramPerSquareMeter) * (8 * 2))));
	EXPECT_NEAR(double(mass_data.I / (SquareMeter * Kilogram / SquareRadian)), 90.666664 * double(density / (KilogramPerSquareMeter)), 0.0004);
	EXPECT_TRUE(almost_equal(mass_data.center.x, shape.GetCentroid().x));
	EXPECT_TRUE(almost_equal(mass_data.center.y, shape.GetCentroid().y));
	
	// Area moment of inertia (I) for a rectangle is Ix + Iy = (b * h^3) / 12 + (b^3 * h) / 12....
	const auto i = 8.0 * 2.0 * 2.0 * 2.0 / 12.0 + 8.0 * 8.0 * 8.0 * 2.0 / 12.0;
	EXPECT_NEAR(double(mass_data.I / (SquareMeter * Kilogram / SquareRadian)), double((density / KilogramPerSquareMeter) * RealNum(i)), 0.0004);
	
	const auto i_z = GetPolarMoment(shape.GetVertices());
	EXPECT_NEAR(double(mass_data.I / (SquareMeter * Kilogram / SquareRadian)), double(density * i_z / (SquareMeter * Kilogram)), 0.0004);
	
	EXPECT_TRUE(almost_equal(RealNum{GetAreaOfPolygon(shape.GetVertices()) / SquareMeter}, RealNum(16)));
}

TEST(MassData, GetForZeroVertexRadiusEdge)
{
	const auto v1 = Vec2{-1, 0};
	const auto v2 = Vec2{+1, 0};
	const auto density = RealNum{2.1f} * KilogramPerSquareMeter;
	auto conf = EdgeShape::Conf{};
	conf.vertexRadius = 0;
	conf.density = density;
	auto shape = EdgeShape(conf);
	shape.Set(v1, v2);
	const auto mass_data = GetMassData(shape, density);
	EXPECT_EQ(RealNum{mass_data.mass / Kilogram}, RealNum{0});
	EXPECT_EQ(mass_data.I, RotInertia{0});
	EXPECT_EQ(mass_data.center.x, 0);
	EXPECT_EQ(mass_data.center.y, 0);
}

TEST(MassData, GetForSamePointedEdgeIsSameAsCircle)
{
	const auto v1 = Vec2{-1, 1};
	const auto density = RealNum{1} * KilogramPerSquareMeter;
	auto conf = EdgeShape::Conf{};
	conf.vertexRadius = 1;
	conf.density = density;
	auto shape = EdgeShape(conf);
	shape.Set(v1, v1);
	const auto mass_data = GetMassData(shape, density);
	
	const auto circleMass = RealNum{density / KilogramPerSquareMeter} * Pi * Square(shape.GetVertexRadius());

	EXPECT_TRUE(almost_equal(RealNum{mass_data.mass / Kilogram}, circleMass));
	EXPECT_NEAR(double(mass_data.I / (SquareMeter * Kilogram / SquareRadian)), 7.85398, 0.0004);
	EXPECT_TRUE(almost_equal(mass_data.center.x, v1.x));
	EXPECT_TRUE(almost_equal(mass_data.center.y, v1.y));
}

TEST(MassData, GetForCenteredEdge)
{
	const auto v1 = Vec2{-2, 0};
	const auto v2 = Vec2{+2, 0};
	const auto radius = RealNum(0.5);
	const auto density = RealNum{2.1f} * KilogramPerSquareMeter;
	auto conf = EdgeShape::Conf{};
	conf.vertexRadius = radius;
	conf.density = density;
	auto shape = EdgeShape(conf);
	shape.Set(v1, v2);
	const auto mass_data = GetMassData(shape, density);
	
	const auto vertices = Span<const Vec2>{Vec2(-2, +0.5), Vec2(-2, -0.5), Vec2(+2, -0.5), Vec2(+2, +0.5)};
	const auto area = GetAreaOfPolygon(vertices) + GetAreaOfCircle(radius * Meter);
	EXPECT_EQ(mass_data.mass, density * area);

	EXPECT_NEAR(double(mass_data.I / (SquareMeter * Kilogram / SquareRadian)), 18.70351, 0.002);
	EXPECT_NEAR(double(RealNum{GetPolarMoment(vertices) / (SquareMeter * SquareMeter)}), 5.6666665, 0.0001);
	EXPECT_GT(mass_data.I, (GetPolarMoment(vertices) * density) / SquareRadian);
	
	EXPECT_EQ(mass_data.center.x, 0);
	EXPECT_EQ(mass_data.center.y, 0);
}
