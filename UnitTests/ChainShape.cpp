/*
 * Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

#include "gtest/gtest.h"
#include <PlayRho/Collision/Shapes/ChainShape.hpp>
#include <PlayRho/Collision/Shapes/Shape.hpp>
#include <array>

using namespace playrho;

TEST(ChainShape, ByteSize)
{
    switch (sizeof(Real))
    {
        case  4:
#if defined(_WIN64)
#if !defined(NDEBUG)
            EXPECT_EQ(sizeof(ChainShape::Conf), std::size_t(88));
#else
            EXPECT_EQ(sizeof(ChainShape::Conf), std::size_t(72));
#endif
#elif defined(_WIN32)
#if !defined(NDEBUG)
            EXPECT_EQ(sizeof(ChainShape::Conf), std::size_t(52));
#else
            EXPECT_EQ(sizeof(ChainShape::Conf), std::size_t(44));
#endif
#else
            EXPECT_EQ(sizeof(ChainShape::Conf), std::size_t(64));
#endif
            break;
        case  8: EXPECT_EQ(sizeof(ChainShape::Conf), std::size_t(88)); break;
        case 16: EXPECT_EQ(sizeof(ChainShape::Conf), std::size_t(128)); break;
        default: FAIL(); break;
    }
}

TEST(ChainShape, DefaultConstruction)
{
    const auto foo = ChainShape::Conf{};
    const auto defaultMassData = MassData{};
    const auto defaultConf = ChainShape::Conf{};
    
    EXPECT_EQ(typeid(foo), typeid(ChainShape::Conf));
    EXPECT_EQ(GetChildCount(foo), ChildCounter{0});
    EXPECT_EQ(foo.GetVertexCount(), ChildCounter{0});
    EXPECT_EQ(GetMassData(foo), defaultMassData);
    
    EXPECT_EQ(GetVertexRadius(foo), ChainShape::GetDefaultVertexRadius());
    EXPECT_EQ(GetDensity(foo), defaultConf.density);
    EXPECT_EQ(GetFriction(foo), defaultConf.friction);
    EXPECT_EQ(GetRestitution(foo), defaultConf.restitution);
}

TEST(ChainShape, GetInvalidChildThrows)
{
    const auto foo = ChainShape::Conf{};
    
    ASSERT_EQ(GetChildCount(foo), ChildCounter{0});
    EXPECT_THROW(GetChild(foo, 0), InvalidArgument);
    EXPECT_THROW(GetChild(foo, 1), InvalidArgument);
}

TEST(ChainShape, Accept)
{
    auto visited = false;
    auto shapeVisited = false;
    const auto foo = ChainShape::Conf{};
    ASSERT_FALSE(visited);
    ASSERT_FALSE(shapeVisited);
    
    Accept(Shape(foo), [&](const std::type_info& ti, const void*) {
        visited = true;
        if (ti == typeid(ChainShape::Conf))
        {
            shapeVisited = true;
        }
    });
    EXPECT_TRUE(visited);
    EXPECT_TRUE(shapeVisited);
}

TEST(ChainShape, OneVertexLikeDisk)
{
    const auto vertexRadius = 1_m;
    const auto density = 1_kgpm2;
    const auto location = Length2{};
    const auto expectedMassData = ::GetMassData(vertexRadius, density, location);
    const auto expectedDistanceProxy = DistanceProxy{vertexRadius, 1, &location, nullptr};

    auto conf = ChainShape::Conf{};
    conf.density = density;
    conf.vertexRadius = vertexRadius;
    conf.Add(location);
    auto foo = ChainShape::Conf{conf};
    EXPECT_EQ(GetChildCount(foo), ChildCounter{1});
    EXPECT_EQ(foo.GetVertexCount(), ChildCounter{1});
    EXPECT_EQ(GetVertexRadius(foo), vertexRadius);
    EXPECT_EQ(GetMassData(foo), expectedMassData);
    
    const auto child = GetChild(foo, 0);
    EXPECT_EQ(child, expectedDistanceProxy);
}

TEST(ChainShape, TwoVertexLikeEdge)
{
    const auto vertexRadius = 1_m;
    const auto density = NonNegative<AreaDensity>(1_kgpm2);
    const auto locations = std::array<Length2, 2>{{
        Length2{0_m, 0_m}, Length2(4_m, 0_m)
    }};
    
    auto conf = ChainShape::Conf{};
    conf.density = density;
    conf.vertexRadius = vertexRadius;
    conf.Add(locations[0]);
    conf.Add(locations[1]);
    auto foo = ChainShape::Conf{conf};
    EXPECT_EQ(GetChildCount(foo), ChildCounter{1});
    EXPECT_EQ(foo.GetVertexCount(), ChildCounter{2});
    EXPECT_EQ(GetVertexRadius(foo), vertexRadius);
}

TEST(ChainShape, TwoVertexDpLikeEdgeDp)
{
    const auto vertexRadius = 1_m;
    const auto density = NonNegative<AreaDensity>(1_kgpm2);
    const auto locations = std::array<Length2, 2>{{
        Length2{0_m, 0_m}, Length2(4_m, 0_m)
    }};
    const auto normals = std::array<UnitVec2, 2>{{UnitVec2::GetTop(), UnitVec2::GetBottom()}};
    const auto expectedDistanceProxy = DistanceProxy{vertexRadius, 2, locations.data(), normals.data()};
    
    auto conf = ChainShape::Conf{};
    conf.density = density;
    conf.vertexRadius = vertexRadius;
    conf.Add(locations[0]);
    conf.Add(locations[1]);
    auto foo = ChainShape::Conf{conf};
    ASSERT_EQ(GetChildCount(foo), ChildCounter{1});
    
    const auto child = GetChild(foo, 0);
    EXPECT_EQ(child, expectedDistanceProxy);
}

TEST(ChainShape, TwoVertexMassLikeEdgeMass)
{
    const auto vertexRadius = 1_m;
    const auto density = NonNegative<AreaDensity>(1_kgpm2);
    const auto locations = std::array<Length2, 2>{{
        Length2{0_m, 0_m}, Length2(4_m, 0_m)
    }};
    const auto expectedMassData = ::GetMassData(vertexRadius, density, locations[0], locations[1]);
    
    auto conf = ChainShape::Conf{};
    conf.density = density;
    conf.vertexRadius = vertexRadius;
    conf.Add(locations[0]);
    conf.Add(locations[1]);
    auto foo = ChainShape::Conf{conf};
    
    const auto massData = GetMassData(foo);
    EXPECT_NEAR(static_cast<double>(Real{GetX(massData.center)/1_m}),
                static_cast<double>(Real{GetX(expectedMassData.center)/1_m}),
                0.000001);
    EXPECT_NEAR(static_cast<double>(Real{GetY(massData.center)/1_m}),
                static_cast<double>(Real{GetY(expectedMassData.center)/1_m}),
                0.000001);
    EXPECT_EQ(massData.mass, expectedMassData.mass);
    EXPECT_EQ(massData.I, expectedMassData.I);
}

TEST(ChainShape, FourVertex)
{
    const auto vertexRadius = 1_m;
    const auto density = 1_kgpm2;
    const auto locations = std::array<Length2, 5>{{
        Length2(-4_m, -4_m),
        Length2(-4_m, +4_m),
        Length2(+4_m, +4_m),
        Length2(+4_m, -4_m),
        Length2(-4_m, -4_m)
    }};
    const auto edgeMassData0 = ::GetMassData(vertexRadius, density, locations[0], locations[1]);

    auto conf = ChainShape::Conf{};
    conf.density = density;
    conf.vertexRadius = vertexRadius;
    conf.Set(std::vector<Length2>(std::begin(locations), std::end(locations)));
    auto foo = ChainShape::Conf{conf};
    EXPECT_EQ(GetChildCount(foo), ChildCounter{4});
    EXPECT_EQ(foo.GetVertexCount(), ChildCounter{5});
    EXPECT_EQ(GetVertexRadius(foo), vertexRadius);
    
    const auto massData = GetMassData(foo);
    EXPECT_EQ(massData.center, (Length2{}));
    const auto expectedMass = Mass{edgeMassData0.mass} * Real(4);
    EXPECT_EQ(massData.mass, NonNegative<Mass>{expectedMass});
}

TEST(ChainShape, WithCircleVertices)
{
    const auto circleRadius = 4_m;
    const auto vertices = GetCircleVertices(circleRadius, 4, 0_deg, Real(1) / Real(2));
    const auto density = 1_kgpm2;
    const auto vertexRadius = 1_m / 10;

    auto conf = ChainShape::Conf{};
    conf.density = density;
    conf.vertexRadius = vertexRadius;
    conf.Set(vertices);
    auto foo = ChainShape::Conf{conf};
    EXPECT_EQ(GetChildCount(foo), ChildCounter{4});
    EXPECT_EQ(foo.GetVertexCount(), ChildCounter{5});
    EXPECT_EQ(GetVertexRadius(foo), vertexRadius);
    
    const auto massData = GetMassData(foo);
    EXPECT_NEAR(static_cast<double>(Real(GetX(massData.center) / 1_m)), 0.0, 0.0001);
    EXPECT_NEAR(static_cast<double>(Real(GetY(massData.center) / 1_m)), 2.4142134189605713, 0.0001);
}

TEST(ChainShape, TooManyVertices)
{
    const auto density = 1_kgpm2;
    const auto vertexRadius = 1_m / 10;
    
    auto conf = ChainShape::Conf{};
    conf.density = density;
    conf.vertexRadius = vertexRadius;
    EXPECT_THROW(conf.Set(std::vector<Length2>(MaxChildCount + 1)), InvalidArgument);
}
