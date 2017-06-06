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
#include <Box2D/Collision/Shapes/DiskShape.hpp>
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
    auto shape = DiskShape(0);
    shape.SetDensity(KilogramPerSquareMeter);
    const auto mass_data = shape.GetMassData();
    EXPECT_EQ(mass_data.mass, Mass{0});
    EXPECT_EQ(mass_data.I, RotInertia{0});
    EXPECT_EQ(mass_data.center.x, RealNum{0} * Meter);
    EXPECT_EQ(mass_data.center.y, RealNum{0} * Meter);
}

TEST(MassData, GetForOriginCenteredCircle)
{
    auto conf = DiskShape::Conf{};
    conf.vertexRadius = RealNum{1} * Meter;
    conf.location = Vec2{0, 0} * Meter;
    conf.density = RealNum{1} * KilogramPerSquareMeter;
    const auto foo = DiskShape{conf};
    const auto mass_data = foo.GetMassData();
    EXPECT_EQ(RealNum{Mass{mass_data.mass} / Kilogram}, Pi);
    EXPECT_NEAR(double(StripUnit(mass_data.I)), 1.5707964, 0.0001);
    const auto squareVertexRadius = Square(Length{conf.vertexRadius});
    const auto expectedI = RotInertia{
        // L^2 M QP^-2
        Density{conf.density} * squareVertexRadius * squareVertexRadius * Pi / (RealNum{2} * Radian * Radian)
    };
    EXPECT_TRUE(almost_equal(StripUnit(mass_data.I), StripUnit(expectedI)));
    EXPECT_EQ(mass_data.center, conf.location);
}

TEST(MassData, GetForCircle)
{
    const auto radius = RealNum(1) * Meter;
    const auto position = Vec2{-1, 1} * Meter;
    const auto density = RealNum{1} * KilogramPerSquareMeter;
    auto conf = DiskShape::Conf{};
    conf.vertexRadius = radius;
    conf.location = position;
    conf.density = density;
    const auto foo = DiskShape{conf};
    const auto mass_data = foo.GetMassData();
    EXPECT_EQ(RealNum{Mass{mass_data.mass} / Kilogram}, Pi);
    EXPECT_NEAR(double(StripUnit(mass_data.I)), 7.85398, 0.0002);
    EXPECT_EQ(mass_data.center, position);
}

TEST(MassData, GetForZeroVertexRadiusRectangle)
{
    const auto density = RealNum{2.1f} * KilogramPerSquareMeter;
    auto conf = PolygonShape::Conf{};
    conf.vertexRadius = 0;
    conf.density = density;
    auto shape = PolygonShape(conf);
    shape.SetAsBox(RealNum{4} * Meter, RealNum{1} * Meter);
    ASSERT_EQ(shape.GetCentroid().x, RealNum(0) * Meter);
    ASSERT_EQ(shape.GetCentroid().y, RealNum(0) * Meter);
    const auto mass_data = shape.GetMassData();
    EXPECT_TRUE(almost_equal(RealNum(Mass{mass_data.mass} / Kilogram),
                             RealNum((density / KilogramPerSquareMeter) * (8 * 2))));
    EXPECT_NEAR(double(StripUnit(mass_data.I)),
                90.666664 * double(StripUnit(density)),
                0.0004);
    EXPECT_TRUE(almost_equal(mass_data.center.x / Meter, shape.GetCentroid().x / Meter));
    EXPECT_TRUE(almost_equal(mass_data.center.y / Meter, shape.GetCentroid().y / Meter));
    
    // Area moment of inertia (I) for a rectangle is Ix + Iy = (b * h^3) / 12 + (b^3 * h) / 12....
    const auto i = 8.0 * 2.0 * 2.0 * 2.0 / 12.0 + 8.0 * 8.0 * 8.0 * 2.0 / 12.0;
    EXPECT_NEAR(double(StripUnit(mass_data.I)),
                double(StripUnit(density) * RealNum(i)),
                0.0004);
    
    const auto i_z = GetPolarMoment(shape.GetVertices());
    EXPECT_NEAR(double(RealNum{RotInertia{mass_data.I} / (SquareMeter * Kilogram / SquareRadian)}),
                double(RealNum{density * i_z / (SquareMeter * Kilogram)}),
                0.0004);
    
    EXPECT_TRUE(almost_equal(RealNum(GetAreaOfPolygon(shape.GetVertices()) / SquareMeter), RealNum(16)));
}

TEST(MassData, GetForZeroVertexRadiusEdge)
{
    const auto v1 = Vec2{-1, 0} * Meter;
    const auto v2 = Vec2{+1, 0} * Meter;
    const auto density = RealNum{2.1f} * KilogramPerSquareMeter;
    auto conf = EdgeShape::Conf{};
    conf.vertexRadius = Length{0};
    conf.density = density;
    const auto shape = EdgeShape(v1, v2, conf);
    const auto mass_data = shape.GetMassData();
    EXPECT_EQ(RealNum(Mass{mass_data.mass} / Kilogram), RealNum(0));
    EXPECT_TRUE(IsValid(mass_data.I));
    if (IsValid(mass_data.I))
    {
        EXPECT_NEAR(double(RealNum{RotInertia{mass_data.I} / (SquareMeter * Kilogram / SquareRadian)}), 0.0, 0.00001);
    }
    EXPECT_EQ(mass_data.center.x, Length{0});
    EXPECT_EQ(mass_data.center.y, Length{0});
}

TEST(MassData, GetForSamePointedEdgeIsSameAsCircle)
{
    const auto v1 = Vec2{-1, 1} * Meter;
    const auto density = RealNum{1} * KilogramPerSquareMeter;
    auto conf = EdgeShape::Conf{};
    conf.vertexRadius = RealNum{1} * Meter;
    conf.density = density;
    auto shape = EdgeShape(conf);
    shape.Set(v1, v1);
    const auto mass_data = shape.GetMassData();
    
    const auto circleMass = density * Pi * Square(shape.GetVertexRadius());

    EXPECT_TRUE(almost_equal(StripUnit(mass_data.mass), StripUnit(circleMass)));
    EXPECT_TRUE(IsValid(mass_data.I));
    if (IsValid(mass_data.I))
    {
        EXPECT_NEAR(double(RealNum{RotInertia{mass_data.I} / (SquareMeter * Kilogram / SquareRadian)}),
                    7.85398, 0.0004);
    }
    EXPECT_TRUE(almost_equal(StripUnit(mass_data.center.x), StripUnit(v1.x)));
    EXPECT_TRUE(almost_equal(StripUnit(mass_data.center.y), StripUnit(v1.y)));
}

TEST(MassData, GetForCenteredEdge)
{
    const auto v1 = Vec2{-2, 0} * Meter;
    const auto v2 = Vec2{+2, 0} * Meter;
    const auto radius = RealNum{0.5f} * Meter;
    const auto density = RealNum{2.1f} * KilogramPerSquareMeter;
    
    const auto radiusSquared = Area{radius * radius};
    const auto circleArea = radiusSquared * Pi;
    EXPECT_EQ(double(RealNum{circleArea / SquareMeter}), 0.5f * 0.5f * Pi);

    auto conf = EdgeShape::Conf{};
    conf.vertexRadius = radius;
    conf.density = density;
    auto shape = EdgeShape(conf);
    shape.Set(v1, v2);
    ASSERT_EQ(shape.GetVertexRadius(), radius);
    ASSERT_EQ(shape.GetVertex1(), v1);
    ASSERT_EQ(shape.GetVertex2(), v2);
    ASSERT_EQ(shape.GetDensity(), density);
    
    const auto vertices = Span<const Length2D>{
        Vec2(-2, +0.5) * Meter,
        Vec2(-2, -0.5) * Meter,
        Vec2(+2, -0.5) * Meter,
        Vec2(+2, +0.5) * Meter
    };
    const auto polarMoment = GetPolarMoment(vertices);
    ASSERT_GE(polarMoment, SecondMomentOfArea(0));
    const auto area = GetAreaOfPolygon(vertices) + GetAreaOfCircle(radius);

    const auto halfCircleArea = circleArea / RealNum{2};
    const auto halfRSquared = radiusSquared / RealNum{2};
    const auto I1 = SecondMomentOfArea{halfCircleArea * (halfRSquared + GetLengthSquared(v1))};
    const auto I2 = SecondMomentOfArea{halfCircleArea * (halfRSquared + GetLengthSquared(v2))};
    EXPECT_GE(I1, SecondMomentOfArea(0));
    EXPECT_GE(I2, SecondMomentOfArea(0));

    const auto mass_data = shape.GetMassData();
    EXPECT_EQ(mass_data.mass, density * area);
    EXPECT_TRUE(IsValid(mass_data.I));
    if (IsValid(mass_data.I))
    {
        const auto I = (polarMoment + I1 + I2) * density / SquareRadian;
        EXPECT_EQ(double(RealNum{RotInertia{mass_data.I} / (SquareMeter * Kilogram / SquareRadian)}),
                  double(RealNum{I / (SquareMeter * Kilogram / SquareRadian)}));
        EXPECT_NEAR(double(RealNum{RotInertia{mass_data.I} / (SquareMeter * Kilogram / SquareRadian)}),
                    18.70351, 0.002);
        EXPECT_GE(mass_data.I, (polarMoment * density) / SquareRadian);
    }
    EXPECT_NEAR(double(RealNum(polarMoment / (SquareMeter * SquareMeter))), 5.6666665, 0.0001);
    EXPECT_EQ(mass_data.center.x, Length(0));
    EXPECT_EQ(mass_data.center.y, Length(0));
}
