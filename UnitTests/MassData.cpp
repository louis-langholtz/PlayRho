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
#include <PlayRho/Collision/Shapes/DiskShapeConf.hpp>
#include <PlayRho/Collision/Shapes/PolygonShapeConf.hpp>
#include <PlayRho/Collision/Shapes/EdgeShapeConf.hpp>

using namespace playrho;
using namespace playrho::d2;

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
    EXPECT_EQ(foo.center, (Length2{}));
    EXPECT_EQ(foo.mass, 0_kg);
    EXPECT_EQ(foo.I, RotInertia(0));
}

TEST(MassData, Traits)
{
    EXPECT_TRUE(std::is_default_constructible<MassData>::value);
    // EXPECT_FALSE(std::is_nothrow_default_constructible<MassData>::value); // clang-3.7 and 4.0
    // EXPECT_TRUE(std::is_nothrow_default_constructible<MassData>::value); // gcc 6.3
    EXPECT_FALSE(std::is_trivially_default_constructible<MassData>::value);
    
    EXPECT_FALSE((std::is_constructible<MassData, Length2, Mass, RotInertia>::value));
    EXPECT_FALSE((std::is_constructible<MassData, Length2, Mass>::value));
    EXPECT_FALSE((std::is_constructible<MassData, Length2>::value));
    EXPECT_FALSE((std::is_constructible<MassData, Length2, NonNegative<Mass>, NonNegative<RotInertia>>::value));
    EXPECT_FALSE((std::is_constructible<MassData, Length2, NonNegative<Mass>>::value));
    EXPECT_FALSE((std::is_constructible<MassData, Length2>::value));
    EXPECT_TRUE((std::is_constructible<MassData>::value));
    // EXPECT_FALSE(std::is_nothrow_constructible<MassData>::value); // clang 3.7 and 4.0
    // EXPECT_TRUE(std::is_nothrow_constructible<MassData>::value); // gcc 6.3
    EXPECT_FALSE((std::is_trivially_constructible<MassData, Length2, Mass, RotInertia>::value));
    
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
        const auto vertices = Span<const Length2>{};
        const auto area = GetAreaOfPolygon(vertices);
        EXPECT_NEAR(static_cast<double>(Real{area / SquareMeter}), 0.0, 0.0);
    }
    {
        // point
        const auto vertices = Vector<const Length2, 1>{
            Length2{-2_m, +0.5_m},
        };
        const auto area = GetAreaOfPolygon(vertices);
        EXPECT_NEAR(static_cast<double>(Real{area / SquareMeter}), 0.0, 0.0);
    }
    {
        // edge
        const auto vertices = Vector<const Length2, 2>{
            Length2{-2_m, +0.5_m},
            Length2{+2_m, +0.5_m},
        };
        const auto area = GetAreaOfPolygon(vertices);
        EXPECT_NEAR(static_cast<double>(Real{area / SquareMeter}), 0.0, 0.0);
    }
    {
        // CCW triangle
        const auto vertices = Vector<const Length2, 3>{
            Length2{-2_m, +1.0_m},
            Length2{-2_m, -1.0_m},
            Length2{+2_m, +0.0_m},
        };
        const auto area = GetAreaOfPolygon(vertices);
        EXPECT_NEAR(static_cast<double>(Real{area / SquareMeter}), 4.0, 0.0);
    }
    {
        // CCW triangle
        const auto vertices = Vector<const Length2, 3>{
            Length2{-2_m, -1.0_m},
            Length2{+2_m, +0.0_m},
            Length2{-2_m, +1.0_m},
        };
        const auto area = GetAreaOfPolygon(vertices);
        EXPECT_NEAR(static_cast<double>(Real{area / SquareMeter}), 4.0, 0.0);
    }
    {
        // CCW triangle
        const auto vertices = Vector<const Length2, 3>{
            Length2{+2_m, +0.0_m},
            Length2{-2_m, +1.0_m},
            Length2{-2_m, -1.0_m},
        };
        const auto area = GetAreaOfPolygon(vertices);
        EXPECT_NEAR(static_cast<double>(Real{area / SquareMeter}), 4.0, 0.0);
    }
    {
        // CW triangle
        const auto vertices = Vector<const Length2, 3>{
            Length2{+2_m, +0.0_m},
            Length2{-2_m, -1.0_m},
            Length2{-2_m, +1.0_m},
        };
        const auto area = GetAreaOfPolygon(vertices);
        EXPECT_NEAR(static_cast<double>(Real{area / SquareMeter}), 4.0, 0.0);
    }
    {
        // CCW triangle
        const auto vertices = Vector<const Length2, 3>{
            Length2{+0_m, -2.0_m},
            Length2{-4_m, -1.0_m},
            Length2{-4_m, -3.0_m},
        };
        const auto area = GetAreaOfPolygon(vertices);
        EXPECT_NEAR(static_cast<double>(Real{area / SquareMeter}), 4.0, 0.0);
    }
    {
        // CCW quadrilateral
        const auto vertices = Vector<const Length2, 4>{
            Length2{-2_m, +0.5_m},
            Length2{-2_m, -0.5_m},
            Length2{+2_m, -0.5_m},
            Length2{+2_m, +0.5_m}
        };
        const auto area = GetAreaOfPolygon(vertices);
        EXPECT_NEAR(static_cast<double>(Real{area / SquareMeter}), 4.0, 0.0);
    }
}

TEST(MassData, GetMassDataFreeFunctionForNoVertices)
{
    const auto vertexRadius = Length{1_m};
    const auto density = NonNegative<AreaDensity>(1_kgpm2);
    const auto vertices = Span<const Length2>{
        static_cast<const Length2*>(nullptr), Span<const Length2>::size_type{0}
    };
    const auto massData = GetMassData(vertexRadius, density, vertices);
    EXPECT_EQ(massData.center, (Length2{}));
    EXPECT_EQ(massData.mass, 0_kg);
    EXPECT_EQ(massData.I, RotInertia(0));
}

TEST(MassData, GetForZeroVertexRadiusCircle)
{
    const auto shape = DiskShapeConf{}.UseRadius(0_m).UseDensity(1_kgpm2);
    const auto mass_data = GetMassData(shape);
    EXPECT_EQ(mass_data.mass, NonNegative<Mass>(0_kg));
    EXPECT_EQ(mass_data.I, RotInertia{0});
    EXPECT_EQ(GetX(mass_data.center), 0_m);
    EXPECT_EQ(GetY(mass_data.center), 0_m);
}

TEST(MassData, GetForOriginCenteredCircle)
{
    auto conf = DiskShapeConf{};
    conf.vertexRadius = 1_m;
    conf.location = Length2{};
    conf.density = 1_kgpm2;
    const auto mass_data = GetMassData(conf);
    EXPECT_EQ(Real{Mass{mass_data.mass} / 1_kg}, Pi);
    EXPECT_NEAR(double(StripUnit(mass_data.I)), 1.5707964, 0.0005);
    const auto squareVertexRadius = Square(Length{conf.vertexRadius});
    const auto expectedI = RotInertia{
        // L^2 M QP^-2
        AreaDensity{conf.density} * squareVertexRadius * squareVertexRadius * Pi / (2 * 1_rad * 1_rad)
    };
    EXPECT_TRUE(AlmostEqual(StripUnit(mass_data.I), StripUnit(expectedI)));
    EXPECT_EQ(mass_data.center, conf.location);
}

TEST(MassData, GetForCircle)
{
    const auto radius = 1_m;
    const auto position = Length2{-1_m, 1_m};
    const auto density = 1_kgpm2;
    auto conf = DiskShapeConf{};
    conf.vertexRadius = radius;
    conf.location = position;
    conf.density = density;
    const auto mass_data = GetMassData(conf);
    EXPECT_EQ(Real{Mass{mass_data.mass} / 1_kg}, Pi);
    EXPECT_NEAR(double(StripUnit(mass_data.I)), 7.85398, 0.003);
    EXPECT_EQ(mass_data.center, position);
}

TEST(MassData, GetForZeroVertexRadiusRectangle)
{
    const auto density = 2.1_kgpm2;
    auto conf = PolygonShapeConf{};
    conf.vertexRadius = 0;
    conf.density = density;
    conf.SetAsBox(4_m, 1_m);
    auto shape = conf;
    ASSERT_EQ(GetX(shape.GetCentroid()), 0_m);
    ASSERT_EQ(GetY(shape.GetCentroid()), 0_m);
    const auto mass_data = GetMassData(shape);
    EXPECT_TRUE(AlmostEqual(Real(Mass{mass_data.mass} / 1_kg),
                             Real((density / 1_kgpm2) * (8 * 2))));
    EXPECT_NEAR(double(StripUnit(mass_data.I)),
                90.666664 * double(StripUnit(density)),
                0.008);
    EXPECT_TRUE(AlmostEqual(Real{GetX(mass_data.center) / Meter}, Real{GetX(shape.GetCentroid()) / Meter}));
    EXPECT_TRUE(AlmostEqual(Real{GetY(mass_data.center) / Meter}, Real{GetY(shape.GetCentroid()) / Meter}));
    
    // Area moment of inertia (I) for a rectangle is Ix + Iy = (b * h^3) / 12 + (b^3 * h) / 12....
    const auto i = 8.0 * 2.0 * 2.0 * 2.0 / 12.0 + 8.0 * 8.0 * 8.0 * 2.0 / 12.0;
    EXPECT_NEAR(double(StripUnit(mass_data.I)),
                double(StripUnit(density) * Real(i)),
                0.004);
    
    const auto i_z = GetPolarMoment(shape.GetVertices());
    EXPECT_NEAR(double(Real{RotInertia{mass_data.I} / (SquareMeter * 1_kg / SquareRadian)}),
                double(Real{density * i_z / (SquareMeter * 1_kg)}),
                0.004);
    
    EXPECT_TRUE(AlmostEqual(Real(GetAreaOfPolygon(shape.GetVertices()) / SquareMeter), Real(16)));
}

TEST(MassData, GetForZeroVertexRadiusEdge)
{
    const auto v1 = Length2{-1_m, 0_m};
    const auto v2 = Length2{+1_m, 0_m};
    const auto density = 2.1_kgpm2;
    auto conf = EdgeShapeConf{};
    conf.vertexRadius = 0_m;
    conf.density = density;
    const auto shape = EdgeShapeConf(v1, v2, conf);
    const auto mass_data = GetMassData(shape);
    EXPECT_EQ(Real(Mass{mass_data.mass} / 1_kg), Real(0));
    EXPECT_NEAR(double(Real{RotInertia{mass_data.I} / (SquareMeter * 1_kg / SquareRadian)}),
                0.0, 0.00001);
    EXPECT_EQ(GetX(mass_data.center), 0_m);
    EXPECT_EQ(GetY(mass_data.center), 0_m);
}

TEST(MassData, GetForSamePointedEdgeIsSameAsCircle)
{
    const auto v1 = Length2{-1_m, 1_m};
    const auto density = 1_kgpm2;
    const auto shape = EdgeShapeConf(EdgeShapeConf{}.UseVertexRadius(1_m).UseDensity(density).Set(v1, v1));
    const auto mass_data = GetMassData(shape);
    const auto circleMass = density * Pi * Square(GetVertexRadius(shape));

    EXPECT_TRUE(AlmostEqual(StripUnit(mass_data.mass), StripUnit(circleMass)));
    EXPECT_TRUE(IsValid(mass_data.I));
    if (IsValid(mass_data.I))
    {
        EXPECT_NEAR(double(Real{RotInertia{mass_data.I} / (SquareMeter * 1_kg / SquareRadian)}),
                    7.85398, 0.003);
    }
    EXPECT_TRUE(AlmostEqual(StripUnit(GetX(mass_data.center)), StripUnit(GetX(v1))));
    EXPECT_TRUE(AlmostEqual(StripUnit(GetY(mass_data.center)), StripUnit(GetY(v1))));
}

TEST(MassData, GetForCenteredEdge)
{
    const auto v1 = Length2{-2_m, 0_m};
    const auto v2 = Length2{+2_m, 0_m};
    const auto radius = 0.5_m;
    const auto density = 2.1_kgpm2;
    
    const auto radiusSquared = Area{radius * radius};
    const auto circleArea = radiusSquared * Pi;
    ASSERT_EQ(double(Real{circleArea / SquareMeter}), 0.5f * 0.5f * Pi);
    ASSERT_NEAR(static_cast<double>(Real{circleArea / SquareMeter}),
                0.78539818525314331, 0.78539818525314331 / 1000000.0);

    const auto shape = EdgeShapeConf(EdgeShapeConf{}.UseVertexRadius(radius).UseDensity(density).Set(v1, v2));
    ASSERT_EQ(GetVertexRadius(shape), radius);
    ASSERT_EQ(shape.GetVertexA(), v1);
    ASSERT_EQ(shape.GetVertexB(), v2);
    ASSERT_EQ(GetDensity(shape), density);
    
    const auto vertices = Vector<Length2, 4>{
        Length2(-2_m, +0.5_m),
        Length2(-2_m, -0.5_m),
        Length2(+2_m, -0.5_m),
        Length2(+2_m, +0.5_m)
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
    const auto I1 = SecondMomentOfArea{halfCircleArea * (halfRSquared + GetMagnitudeSquared(v1))};
    const auto I2 = SecondMomentOfArea{halfCircleArea * (halfRSquared + GetMagnitudeSquared(v2))};
    EXPECT_NEAR(static_cast<double>(Real{I1 / (SquareMeter * SquareMeter)}),
                1.6198837757110596, 1.6198837757110596 / 1000000.0);
    EXPECT_NEAR(static_cast<double>(Real{I2 / (SquareMeter * SquareMeter)}),
                1.6198837757110596, 1.6198837757110596 / 1000000.0);

    const auto totalMoment = polarMoment + I1 + I2;
    const auto I = totalMoment * density / SquareRadian;
    EXPECT_NEAR(static_cast<double>(Real{I / (1_kg * SquareMeter / SquareRadian)}),
                18.703510284423828, 18.703510284423828 / 1000000.0);

    const auto mass_data = GetMassData(shape);
    EXPECT_EQ(mass_data.mass, density * area);
    {
        EXPECT_NEAR(static_cast<double>(Real{RotInertia{mass_data.I} / (SquareMeter * 1_kg / SquareRadian)}),
                    static_cast<double>(Real{I / (SquareMeter * 1_kg / SquareRadian)}),
                    0.0001);
        EXPECT_NEAR(static_cast<double>(Real{RotInertia{mass_data.I} / (SquareMeter * 1_kg / SquareRadian)}),
                    18.70351, 0.009);
        EXPECT_GE(mass_data.I, (polarMoment * density) / SquareRadian);
    }
    EXPECT_NEAR(double(Real(polarMoment / (SquareMeter * SquareMeter))), 5.6666665, 0.0007);
    EXPECT_EQ(GetX(mass_data.center), 0_m);
    EXPECT_EQ(GetY(mass_data.center), 0_m);
}

TEST(MassData, Equals)
{
    const auto foo = MassData{};
    EXPECT_TRUE(foo == foo);
    
    const auto boo = MassData{};
    EXPECT_TRUE(foo == boo);
    
    const auto poo = MassData{
        Length2(1_m, 1_m), 4_kg, RotInertia{2_kg * SquareMeter / SquareRadian}
    };
    EXPECT_FALSE(foo == poo);
}

TEST(MassData, NotEquals)
{
    const auto foo = MassData{};
    EXPECT_FALSE(foo != foo);
    
    const auto boo = MassData{};
    EXPECT_FALSE(foo != boo);
    
    const auto poo = MassData{
        Length2(1_m, 1_m), 4_kg, RotInertia{2_kg * SquareMeter / SquareRadian}
    };
    EXPECT_TRUE(foo != poo);
}
