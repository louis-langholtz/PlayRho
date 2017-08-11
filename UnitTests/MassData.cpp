/*
 * Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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
#include <PlayRho/Collision/MassData.hpp>
#include <PlayRho/Collision/Shapes/DiskShape.hpp>
#include <PlayRho/Collision/Shapes/PolygonShape.hpp>
#include <PlayRho/Collision/Shapes/EdgeShape.hpp>

using namespace playrho;

TEST(MassData, ByteSize)
{
    switch (sizeof(Real))
    {
        case  4: EXPECT_EQ(sizeof(MassData), std::size_t(16)); break;
        case  8: EXPECT_EQ(sizeof(MassData), std::size_t(32)); break;
        case 16: EXPECT_EQ(sizeof(MassData), std::size_t(64)); break;
        default: FAIL(); break;
    }
}

TEST(MassData, DefaultConstruct)
{
    MassData foo;
    EXPECT_EQ(foo.center, (Length2D{}));
    EXPECT_EQ(foo.mass, Mass(0));
    EXPECT_EQ(foo.I, RotInertia(0));
}

TEST(MassData, Traits)
{
    EXPECT_TRUE(std::is_default_constructible<MassData>::value);
    // EXPECT_FALSE(std::is_nothrow_default_constructible<MassData>::value); // clang-3.7 and 4.0
    // EXPECT_TRUE(std::is_nothrow_default_constructible<MassData>::value); // gcc 6.3
    EXPECT_FALSE(std::is_trivially_default_constructible<MassData>::value);
    
    EXPECT_FALSE((std::is_constructible<MassData, Length2D, Mass, RotInertia>::value));
    EXPECT_FALSE((std::is_constructible<MassData, Length2D, Mass>::value));
    EXPECT_FALSE((std::is_constructible<MassData, Length2D>::value));
    EXPECT_FALSE((std::is_constructible<MassData, Length2D, NonNegative<Mass>, NonNegative<RotInertia>>::value));
    EXPECT_FALSE((std::is_constructible<MassData, Length2D, NonNegative<Mass>>::value));
    EXPECT_FALSE((std::is_constructible<MassData, Length2D>::value));
    EXPECT_TRUE((std::is_constructible<MassData>::value));
    // EXPECT_FALSE(std::is_nothrow_constructible<MassData>::value); // clang 3.7 and 4.0
    // EXPECT_TRUE(std::is_nothrow_constructible<MassData>::value); // gcc 6.3
    EXPECT_FALSE((std::is_trivially_constructible<MassData, Length2D, Mass, RotInertia>::value));
    
    EXPECT_TRUE(std::is_copy_constructible<MassData>::value);
    // EXPECT_TRUE(std::is_nothrow_copy_constructible<MassData>::value); // with clang-4.0 gcc 6.3
    // EXPECT_FALSE(std::is_nothrow_copy_constructible<MassData>::value); // with clang-3.7
    // EXPECT_TRUE(std::is_trivially_copy_constructible<MassData>::value); // with clang-4.0
    // EXPECT_FALSE(std::is_trivially_copy_constructible<MassData>::value); // with clang-3.7
    
    EXPECT_TRUE(std::is_copy_assignable<MassData>::value);
    // EXPECT_TRUE(std::is_nothrow_copy_assignable<MassData>::value); // with clang-4.0 gcc 6.3
    // EXPECT_FALSE(std::is_nothrow_copy_assignable<MassData>::value); // with clang-3.7
    EXPECT_FALSE(std::is_trivially_copy_assignable<MassData>::value);
    
    EXPECT_TRUE(std::is_destructible<MassData>::value);
    EXPECT_TRUE(std::is_nothrow_destructible<MassData>::value);
    EXPECT_TRUE(std::is_trivially_destructible<MassData>::value);
}

TEST(MassData, GetAreaOfPolygon)
{
    {
        // empty
        const auto vertices = Span<const Length2D>{};
        const auto area = GetAreaOfPolygon(vertices);
        EXPECT_NEAR(static_cast<double>(Real{area / SquareMeter}), 0.0, 0.0);
    }
    {
        // point
        const auto vertices = Span<const Length2D>{
            Length2D(Real(-2) * Meter, Real(+0.5) * Meter),
        };
        const auto area = GetAreaOfPolygon(vertices);
        EXPECT_NEAR(static_cast<double>(Real{area / SquareMeter}), 0.0, 0.0);
    }
    {
        // edge
        const auto vertices = Span<const Length2D>{
            Length2D(Real(-2) * Meter, Real(+0.5) * Meter),
            Length2D(Real(+2) * Meter, Real(+0.5) * Meter),
        };
        const auto area = GetAreaOfPolygon(vertices);
        EXPECT_NEAR(static_cast<double>(Real{area / SquareMeter}), 0.0, 0.0);
    }
    {
        // CCW triangle
        const auto vertices = Span<const Length2D>{
            Length2D(Real(-2) * Meter, Real(+1.0) * Meter),
            Length2D(Real(-2) * Meter, Real(-1.0) * Meter),
            Length2D(Real(+2) * Meter, Real(+0.0) * Meter),
        };
        const auto area = GetAreaOfPolygon(vertices);
        EXPECT_NEAR(static_cast<double>(Real{area / SquareMeter}), 4.0, 0.0);
    }
    {
        // CCW triangle
        const auto vertices = Span<const Length2D>{
            Length2D(Real(-2) * Meter, Real(-1.0) * Meter),
            Length2D(Real(+2) * Meter, Real(+0.0) * Meter),
            Length2D(Real(-2) * Meter, Real(+1.0) * Meter),
        };
        const auto area = GetAreaOfPolygon(vertices);
        EXPECT_NEAR(static_cast<double>(Real{area / SquareMeter}), 4.0, 0.0);
    }
    {
        // CCW triangle
        const auto vertices = Span<const Length2D>{
            Length2D(Real(+2) * Meter, Real(+0.0) * Meter),
            Length2D(Real(-2) * Meter, Real(+1.0) * Meter),
            Length2D(Real(-2) * Meter, Real(-1.0) * Meter),
        };
        const auto area = GetAreaOfPolygon(vertices);
        EXPECT_NEAR(static_cast<double>(Real{area / SquareMeter}), 4.0, 0.0);
    }
    {
        // CW triangle
        const auto vertices = Span<const Length2D>{
            Length2D(Real(+2) * Meter, Real(+0.0) * Meter),
            Length2D(Real(-2) * Meter, Real(-1.0) * Meter),
            Length2D(Real(-2) * Meter, Real(+1.0) * Meter),
        };
        const auto area = GetAreaOfPolygon(vertices);
        EXPECT_NEAR(static_cast<double>(Real{area / SquareMeter}), 4.0, 0.0);
    }
    {
        // CCW triangle
        const auto vertices = Span<const Length2D>{
            Length2D(Real(+0) * Meter, Real(-2.0) * Meter),
            Length2D(Real(-4) * Meter, Real(-1.0) * Meter),
            Length2D(Real(-4) * Meter, Real(-3.0) * Meter),
        };
        const auto area = GetAreaOfPolygon(vertices);
        EXPECT_NEAR(static_cast<double>(Real{area / SquareMeter}), 4.0, 0.0);
    }
    {
        // CCW quadrilateral
        const auto vertices = Span<const Length2D>{
            Length2D(Real(-2) * Meter, Real(+0.5) * Meter),
            Length2D(Real(-2) * Meter, Real(-0.5) * Meter),
            Length2D(Real(+2) * Meter, Real(-0.5) * Meter),
            Length2D(Real(+2) * Meter, Real(+0.5) * Meter)
        };
        const auto area = GetAreaOfPolygon(vertices);
        EXPECT_NEAR(static_cast<double>(Real{area / SquareMeter}), 4.0, 0.0);
    }
}

TEST(MassData, GetMassDataFreeFunctionForNoVertices)
{
    const auto vertexRadius = Length{Real(1) * Meter};
    const auto density = NonNegative<Density>(Real(1) * KilogramPerSquareMeter);
    const auto vertices = Span<const Length2D>{
        static_cast<const Length2D*>(nullptr), Span<const Length2D>::size_type{0}
    };
    const auto massData = GetMassData(vertexRadius, density, vertices);
    EXPECT_EQ(massData.center, (Length2D{}));
    EXPECT_EQ(massData.mass, Mass(0));
    EXPECT_EQ(massData.I, RotInertia(0));
}

TEST(MassData, GetForZeroVertexRadiusCircle)
{
    auto shape = DiskShape(0);
    shape.SetDensity(Real(1) * KilogramPerSquareMeter);
    const auto mass_data = shape.GetMassData();
    EXPECT_EQ(mass_data.mass, NonNegative<Mass>(Real{0} * Kilogram));
    EXPECT_EQ(mass_data.I, RotInertia{0});
    EXPECT_EQ(GetX(mass_data.center), Real{0} * Meter);
    EXPECT_EQ(GetY(mass_data.center), Real{0} * Meter);
}

TEST(MassData, GetForOriginCenteredCircle)
{
    auto conf = DiskShape::Conf{};
    conf.vertexRadius = Real{1} * Meter;
    conf.location = Length2D{};
    conf.density = Real{1} * KilogramPerSquareMeter;
    const auto foo = DiskShape{conf};
    const auto mass_data = foo.GetMassData();
    EXPECT_EQ(Real{Mass{mass_data.mass} / Kilogram}, Pi);
    EXPECT_NEAR(double(StripUnit(mass_data.I)), 1.5707964, 0.0005);
    const auto squareVertexRadius = Square(Length{conf.vertexRadius});
    const auto expectedI = RotInertia{
        // L^2 M QP^-2
        Density{conf.density} * squareVertexRadius * squareVertexRadius * Pi / (Real{2} * Radian * Radian)
    };
    EXPECT_TRUE(almost_equal(StripUnit(mass_data.I), StripUnit(expectedI)));
    EXPECT_EQ(mass_data.center, conf.location);
}

TEST(MassData, GetForCircle)
{
    const auto radius = Real(1) * Meter;
    const auto position = Length2D{-Real(1) * Meter, Real(1) * Meter};
    const auto density = Real{1} * KilogramPerSquareMeter;
    auto conf = DiskShape::Conf{};
    conf.vertexRadius = radius;
    conf.location = position;
    conf.density = density;
    const auto foo = DiskShape{conf};
    const auto mass_data = foo.GetMassData();
    EXPECT_EQ(Real{Mass{mass_data.mass} / Kilogram}, Pi);
    EXPECT_NEAR(double(StripUnit(mass_data.I)), 7.85398, 0.003);
    EXPECT_EQ(mass_data.center, position);
}

TEST(MassData, GetForZeroVertexRadiusRectangle)
{
    const auto density = Real{2.1f} * KilogramPerSquareMeter;
    auto conf = PolygonShape::Conf{};
    conf.vertexRadius = 0;
    conf.density = density;
    auto shape = PolygonShape(conf);
    shape.SetAsBox(Real{4} * Meter, Real{1} * Meter);
    ASSERT_EQ(GetX(shape.GetCentroid()), Real(0) * Meter);
    ASSERT_EQ(GetY(shape.GetCentroid()), Real(0) * Meter);
    const auto mass_data = shape.GetMassData();
    EXPECT_TRUE(almost_equal(Real(Mass{mass_data.mass} / Kilogram),
                             Real((density / KilogramPerSquareMeter) * (8 * 2))));
    EXPECT_NEAR(double(StripUnit(mass_data.I)),
                90.666664 * double(StripUnit(density)),
                0.008);
    EXPECT_TRUE(almost_equal(GetX(mass_data.center) / Meter, GetX(shape.GetCentroid()) / Meter));
    EXPECT_TRUE(almost_equal(GetY(mass_data.center) / Meter, GetY(shape.GetCentroid()) / Meter));
    
    // Area moment of inertia (I) for a rectangle is Ix + Iy = (b * h^3) / 12 + (b^3 * h) / 12....
    const auto i = 8.0 * 2.0 * 2.0 * 2.0 / 12.0 + 8.0 * 8.0 * 8.0 * 2.0 / 12.0;
    EXPECT_NEAR(double(StripUnit(mass_data.I)),
                double(StripUnit(density) * Real(i)),
                0.004);
    
    const auto i_z = GetPolarMoment(shape.GetVertices());
    EXPECT_NEAR(double(Real{RotInertia{mass_data.I} / (SquareMeter * Kilogram / SquareRadian)}),
                double(Real{density * i_z / (SquareMeter * Kilogram)}),
                0.004);
    
    EXPECT_TRUE(almost_equal(Real(GetAreaOfPolygon(shape.GetVertices()) / SquareMeter), Real(16)));
}

TEST(MassData, GetForZeroVertexRadiusEdge)
{
    const auto v1 = Length2D{-Real(1) * Meter, Real(0) * Meter};
    const auto v2 = Length2D{+Real(1) * Meter, Real(0) * Meter};
    const auto density = Real{2.1f} * KilogramPerSquareMeter;
    auto conf = EdgeShape::Conf{};
    conf.vertexRadius = Length{0};
    conf.density = density;
    const auto shape = EdgeShape(v1, v2, conf);
    const auto mass_data = shape.GetMassData();
    EXPECT_EQ(Real(Mass{mass_data.mass} / Kilogram), Real(0));
    EXPECT_NEAR(double(Real{RotInertia{mass_data.I} / (SquareMeter * Kilogram / SquareRadian)}),
                0.0, 0.00001);
    EXPECT_EQ(GetX(mass_data.center), Length{0});
    EXPECT_EQ(GetY(mass_data.center), Length{0});
}

TEST(MassData, GetForSamePointedEdgeIsSameAsCircle)
{
    const auto v1 = Length2D{-Real(1) * Meter, Real(1) * Meter};
    const auto density = Real{1} * KilogramPerSquareMeter;
    auto conf = EdgeShape::Conf{};
    conf.vertexRadius = Real{1} * Meter;
    conf.density = density;
    auto shape = EdgeShape(conf);
    shape.Set(v1, v1);
    const auto mass_data = shape.GetMassData();
    
    const auto circleMass = density * Pi * Square(shape.GetVertexRadius());

    EXPECT_TRUE(almost_equal(StripUnit(mass_data.mass), StripUnit(circleMass)));
    EXPECT_TRUE(IsValid(mass_data.I));
    if (IsValid(mass_data.I))
    {
        EXPECT_NEAR(double(Real{RotInertia{mass_data.I} / (SquareMeter * Kilogram / SquareRadian)}),
                    7.85398, 0.003);
    }
    EXPECT_TRUE(almost_equal(StripUnit(GetX(mass_data.center)), StripUnit(GetX(v1))));
    EXPECT_TRUE(almost_equal(StripUnit(GetY(mass_data.center)), StripUnit(GetY(v1))));
}

TEST(MassData, GetForCenteredEdge)
{
    const auto v1 = Length2D{-Real(2) * Meter, Real(0) * Meter};
    const auto v2 = Length2D{+Real(2) * Meter, Real(0) * Meter};
    const auto radius = Real{0.5f} * Meter;
    const auto density = Real{2.1f} * KilogramPerSquareMeter;
    
    const auto radiusSquared = Area{radius * radius};
    const auto circleArea = radiusSquared * Pi;
    ASSERT_EQ(double(Real{circleArea / SquareMeter}), 0.5f * 0.5f * Pi);
    ASSERT_NEAR(static_cast<double>(Real{circleArea / SquareMeter}),
                0.78539818525314331, 0.78539818525314331 / 1000000.0);

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
        Length2D(Real(-2) * Meter, Real(+0.5) * Meter),
        Length2D(Real(-2) * Meter, Real(-0.5) * Meter),
        Length2D(Real(+2) * Meter, Real(-0.5) * Meter),
        Length2D(Real(+2) * Meter, Real(+0.5) * Meter)
    };
    const auto polarMoment = GetPolarMoment(vertices);
    EXPECT_NEAR(static_cast<double>(Real{polarMoment / (SquareMeter * SquareMeter)}),
                5.6666665077209473, 5.6666665077209473 / 1000000.0);

    const auto areaOfPolygon = GetAreaOfPolygon(vertices);
    EXPECT_NEAR(static_cast<double>(Real{areaOfPolygon / SquareMeter}), 4.0, 0.0);
    
    const auto areaOfCircle = GetAreaOfCircle(radius);
    EXPECT_NEAR(static_cast<double>(Real{areaOfCircle / SquareMeter}),
                0.78539818525314331, 0.78539818525314331 / 1000000.0);
    EXPECT_NEAR(static_cast<double>(Real{areaOfCircle / SquareMeter}),
                static_cast<double>(Real{circleArea / SquareMeter}),
                0.0);
    
    const auto area = areaOfPolygon + areaOfCircle;

    const auto halfCircleArea = circleArea / Real{2};
    const auto halfRSquared = radiusSquared / Real{2};
    const auto I1 = SecondMomentOfArea{halfCircleArea * (halfRSquared + GetLengthSquared(v1))};
    const auto I2 = SecondMomentOfArea{halfCircleArea * (halfRSquared + GetLengthSquared(v2))};
    EXPECT_NEAR(static_cast<double>(Real{I1 / (SquareMeter * SquareMeter)}),
                1.6198837757110596, 1.6198837757110596 / 1000000.0);
    EXPECT_NEAR(static_cast<double>(Real{I2 / (SquareMeter * SquareMeter)}),
                1.6198837757110596, 1.6198837757110596 / 1000000.0);

    const auto totalMoment = polarMoment + I1 + I2;
    const auto I = totalMoment * density / SquareRadian;
    EXPECT_NEAR(static_cast<double>(Real{I / (Kilogram * SquareMeter / SquareRadian)}),
                18.703510284423828, 18.703510284423828 / 1000000.0);

    const auto mass_data = shape.GetMassData();
    EXPECT_EQ(mass_data.mass, density * area);
    {
        EXPECT_EQ(double(Real{RotInertia{mass_data.I} / (SquareMeter * Kilogram / SquareRadian)}),
                  double(Real{I / (SquareMeter * Kilogram / SquareRadian)}));
        EXPECT_NEAR(double(Real{RotInertia{mass_data.I} / (SquareMeter * Kilogram / SquareRadian)}),
                    18.70351, 0.009);
        EXPECT_GE(mass_data.I, (polarMoment * density) / SquareRadian);
    }
    EXPECT_NEAR(double(Real(polarMoment / (SquareMeter * SquareMeter))), 5.6666665, 0.0007);
    EXPECT_EQ(GetX(mass_data.center), Length(0));
    EXPECT_EQ(GetY(mass_data.center), Length(0));
}

TEST(MassData, Equals)
{
    const auto foo = MassData{};
    EXPECT_TRUE(foo == foo);
    
    const auto boo = MassData{};
    EXPECT_TRUE(foo == boo);
    
    const auto poo = MassData{
        Length2D(Real(1) * Meter, Real(1) * Meter),
        Real(4) * Kilogram,
        RotInertia{Real(1) * Kilogram * Real(2) * SquareMeter / SquareRadian}};
    EXPECT_FALSE(foo == poo);
}

TEST(MassData, NotEquals)
{
    const auto foo = MassData{};
    EXPECT_FALSE(foo != foo);
    
    const auto boo = MassData{};
    EXPECT_FALSE(foo != boo);
    
    const auto poo = MassData{
        Length2D(Real(1) * Meter, Real(1) * Meter),
        Real(4) * Kilogram,
        RotInertia{Real(1) * Kilogram * Real(2) * SquareMeter / SquareRadian}};
    EXPECT_TRUE(foo != poo);
}
