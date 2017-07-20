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
#include <array>

using namespace playrho;

TEST(ChainShape, ByteSize)
{
    switch (sizeof(Real))
    {
        case  4: EXPECT_EQ(sizeof(ChainShape), std::size_t(72)); break;
        case  8: EXPECT_EQ(sizeof(ChainShape), std::size_t(88)); break;
        case 16: EXPECT_EQ(sizeof(ChainShape), std::size_t(102)); break;
        default: FAIL(); break;
    }
}

TEST(ChainShape, DefaultConstruction)
{
    ChainShape foo{};
    const auto defaultMassData = MassData{};
    const auto defaultConf = ChainShape::Conf{};
    
    EXPECT_EQ(typeid(foo), typeid(ChainShape));
    EXPECT_EQ(foo.GetChildCount(), ChildCounter{0});
    EXPECT_EQ(foo.GetVertexCount(), ChildCounter{0});
    EXPECT_EQ(foo.GetMassData(), defaultMassData);
    
    EXPECT_EQ(foo.GetVertexRadius(), ChainShape::GetDefaultVertexRadius());
    EXPECT_EQ(foo.GetDensity(), defaultConf.density);
    EXPECT_EQ(foo.GetFriction(), defaultConf.friction);
    EXPECT_EQ(foo.GetRestitution(), defaultConf.restitution);
}

TEST(ChainShape, GetInvalidChildThrows)
{
    ChainShape foo{};
    
    ASSERT_EQ(foo.GetChildCount(), ChildCounter{0});
    EXPECT_THROW(foo.GetChild(0), InvalidArgument);
    EXPECT_THROW(foo.GetChild(1), InvalidArgument);
}

TEST(ChainShape, Accept)
{
    class Visitor: public Shape::Visitor
    {
    public:
        void Visit(const ChainShape&) override
        {
            visited = true;
        }
        bool visited = false;
    };
    
    ChainShape foo{};
    Visitor v;
    ASSERT_FALSE(v.visited);
    foo.Accept(v);
    EXPECT_TRUE(v.visited);
}

TEST(ChainShape, OneVertexLikeDisk)
{
    const auto vertexRadius = Real(1) * Meter;
    const auto density = Real(1) * KilogramPerSquareMeter;
    const auto location = Length2D(0, 0);
    const auto expectedMassData = ::GetMassData(vertexRadius, density, location);
    const auto expectedDistanceProxy = DistanceProxy{vertexRadius, 1, &location, nullptr};

    auto conf = ChainShape::Conf{};
    conf.density = density;
    conf.vertexRadius = vertexRadius;
    conf.vertices.push_back(location);
    auto foo = ChainShape{conf};
    EXPECT_EQ(foo.GetChildCount(), ChildCounter{1});
    EXPECT_EQ(foo.GetVertexCount(), ChildCounter{1});
    EXPECT_EQ(foo.GetVertexRadius(), vertexRadius);
    EXPECT_EQ(foo.GetMassData(), expectedMassData);
    
    const auto child = foo.GetChild(0);
    EXPECT_EQ(child, expectedDistanceProxy);
}

TEST(ChainShape, TwoVertexLikeEdge)
{
    const auto vertexRadius = Real(1) * Meter;
    const auto density = Real(1) * KilogramPerSquareMeter;
    const auto locations = std::array<Length2D, 2>{{
        Length2D(Real(0) * Meter, Real(0) * Meter), Length2D(Real(4) * Meter, Real(0) * Meter)
    }};
    const auto normals = std::array<UnitVec2, 2>{{UnitVec2::GetTop(), UnitVec2::GetBottom()}};
    const auto expectedMassData = ::GetMassData(vertexRadius, density, locations[0], locations[1]);
    const auto expectedDistanceProxy = DistanceProxy{vertexRadius, 2, locations.data(), normals.data()};
    
    auto conf = ChainShape::Conf{};
    conf.density = density;
    conf.vertexRadius = vertexRadius;
    conf.vertices.push_back(locations[0]);
    conf.vertices.push_back(locations[1]);
    auto foo = ChainShape{conf};
    EXPECT_EQ(foo.GetChildCount(), ChildCounter{1});
    EXPECT_EQ(foo.GetVertexCount(), ChildCounter{2});
    EXPECT_EQ(foo.GetVertexRadius(), vertexRadius);
    
    const auto massData = foo.GetMassData();
    EXPECT_EQ(massData, expectedMassData);
    
    const auto child = foo.GetChild(0);
    EXPECT_EQ(child, expectedDistanceProxy);
}

TEST(ChainShape, FourVertex)
{
    const auto vertexRadius = Real(1) * Meter;
    const auto density = Real(1) * KilogramPerSquareMeter;
    const auto locations = std::array<Length2D, 5>{{
        Length2D(Real(-4) * Meter, Real(-4) * Meter),
        Length2D(Real(-4) * Meter, Real(+4) * Meter),
        Length2D(Real(+4) * Meter, Real(+4) * Meter),
        Length2D(Real(+4) * Meter, Real(-4) * Meter),
        Length2D(Real(-4) * Meter, Real(-4) * Meter)
    }};
    const auto edgeMassData0 = ::GetMassData(vertexRadius, density, locations[0], locations[1]);

    auto conf = ChainShape::Conf{};
    conf.density = density;
    conf.vertexRadius = vertexRadius;
    conf.vertices = std::vector<Length2D>(std::begin(locations), std::end(locations));
    auto foo = ChainShape{conf};
    EXPECT_EQ(foo.GetChildCount(), ChildCounter{4});
    EXPECT_EQ(foo.GetVertexCount(), ChildCounter{5});
    EXPECT_EQ(foo.GetVertexRadius(), vertexRadius);
    
    const auto massData = foo.GetMassData();
    EXPECT_EQ(massData.center, Length2D(0, 0));
    const auto expectedMass = Mass{edgeMassData0.mass} * Real(4);
    EXPECT_EQ(massData.mass, NonNegative<Mass>{expectedMass});
}

TEST(ChainShape, WithCircleVertices)
{
    const auto circleRadius = Real(4) * Meter;
    const auto vertices = GetCircleVertices(circleRadius, 4, Angle(0), Real(1) / Real(2));
    const auto density = Real(1) * KilogramPerSquareMeter;
    const auto vertexRadius = Meter / Real(10);

    auto conf = ChainShape::Conf{};
    conf.density = density;
    conf.vertexRadius = vertexRadius;
    conf.vertices = vertices;
    auto foo = ChainShape{conf};
    EXPECT_EQ(foo.GetChildCount(), ChildCounter{4});
    EXPECT_EQ(foo.GetVertexCount(), ChildCounter{5});
    EXPECT_EQ(foo.GetVertexRadius(), vertexRadius);
    
    const auto massData = foo.GetMassData();
    EXPECT_NEAR(static_cast<double>(massData.center.GetX() / Meter), 0.0, 0.0001);
    EXPECT_NEAR(static_cast<double>(massData.center.GetY() / Meter), 2.4142134189605713, 0.0001);
}
