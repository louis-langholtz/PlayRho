/*
 * Copyright (c) 2023 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#include "UnitTests.hpp"
#include <PlayRho/Collision/Shapes/ChainShapeConf.hpp>
#include <PlayRho/Collision/Shapes/Shape.hpp>
#include <array>

using namespace playrho;
using namespace playrho::d2;

TEST(ChainShapeConf, ByteSize)
{
    // Check size at test runtime instead of compile-time via static_assert to avoid stopping
    // builds and to report actual size rather than just reporting that expected size is wrong.
    switch (sizeof(Real))
    {
        case  4:
#if defined(_WIN64)
#if !defined(NDEBUG)
            EXPECT_EQ(sizeof(ChainShapeConf), std::size_t(80));
#else
            EXPECT_EQ(sizeof(ChainShapeConf), std::size_t(64));
#endif
#elif defined(_WIN32)
#if !defined(NDEBUG)
	    EXPECT_EQ(sizeof(ChainShapeConf), std::size_t(48));
#else
            EXPECT_EQ(sizeof(ChainShapeConf), std::size_t(40));
#endif
#else
            EXPECT_EQ(sizeof(ChainShapeConf), std::size_t(64));
#endif
            break;
        case  8: EXPECT_EQ(sizeof(ChainShapeConf), std::size_t(80)); break;
        case 16: EXPECT_EQ(sizeof(ChainShapeConf), std::size_t(112)); break;
        default: FAIL(); break;
    }
}

TEST(ChainShapeConf, DefaultConstruction)
{
    const auto foo = ChainShapeConf{};
    const auto defaultMassData = MassData{};
    const auto defaultConf = ChainShapeConf{};
    
    EXPECT_EQ(GetTypeID(foo), GetTypeID<ChainShapeConf>());
    EXPECT_EQ(GetChildCount(foo), ChildCounter{0});
    EXPECT_EQ(foo.GetVertexCount(), ChildCounter{0});
    EXPECT_EQ(GetMassData(foo), defaultMassData);
    for (auto i = ChildCounter{0}; i < GetChildCount(foo); ++i)
    {
        EXPECT_EQ(GetVertexRadius(foo, i), ChainShapeConf::GetDefaultVertexRadius());
    }
    EXPECT_THROW(GetChild(foo, GetChildCount(foo)), InvalidArgument);
    EXPECT_EQ(GetVertexRadius(foo, GetChildCount(foo)), ChainShapeConf::GetDefaultVertexRadius());
    EXPECT_EQ(GetDensity(foo), defaultConf.density);
    EXPECT_EQ(GetFriction(foo), defaultConf.friction);
    EXPECT_EQ(GetRestitution(foo), defaultConf.restitution);
}

TEST(ChainShapeConf, GetInvalidChildThrows)
{
    const auto foo = ChainShapeConf{};
    
    ASSERT_EQ(GetChildCount(foo), ChildCounter{0});
    EXPECT_THROW(GetChild(foo, 0), InvalidArgument);
    EXPECT_THROW(GetChild(foo, 1), InvalidArgument);
}

TEST(ChainShapeConf, TypeInfo)
{
    const auto foo = ChainShapeConf{};
    const auto shape = Shape(foo);
    EXPECT_EQ(GetType(shape), GetTypeID<ChainShapeConf>());
    auto copy = ChainShapeConf{};
    EXPECT_NO_THROW(copy = TypeCast<ChainShapeConf>(shape));
    EXPECT_THROW(TypeCast<int>(shape), std::bad_cast);
}

TEST(ChainShapeConf, TransformFF)
{
    {
        auto foo = ChainShapeConf{};
        auto tmp = foo;
        Transform(foo, Mat22{});
        EXPECT_EQ(foo, tmp);
    }
    {
        auto foo = ChainShapeConf{};
        auto tmp = foo;
        Transform(foo, GetIdentity<Mat22>());
        EXPECT_EQ(foo, tmp);
    }
    {
        const auto v1 = Length2{1_m, 2_m};
        const auto v2 = Length2{3_m, 4_m};
        auto foo = ChainShapeConf{};
        foo.Add(v1);
        foo.Add(v2);
        auto tmp = foo;
        Transform(foo, GetIdentity<Mat22>() * 2);
        EXPECT_NE(foo, tmp);
        ASSERT_EQ(foo.GetVertexCount(), ChildCounter(2));
        EXPECT_EQ(foo.GetVertex(0), v1 * 2);
        EXPECT_EQ(foo.GetVertex(1), v2 * 2);
    }
}

TEST(ChainShapeConf, OneVertexLikeDisk)
{
    const auto vertexRadius = 1_m;
    const auto density = 1_kgpm2;
    const auto location = Length2{};
    const auto expectedMassData = ::GetMassData(vertexRadius, density, location);
    const auto expectedDistanceProxy = DistanceProxy{vertexRadius, 1, &location, nullptr};

    auto conf = ChainShapeConf{};
    conf.density = density;
    conf.vertexRadius = vertexRadius;
    conf.Add(location);
    auto foo = ChainShapeConf{conf};
    EXPECT_EQ(GetChildCount(foo), ChildCounter{1});
    EXPECT_EQ(foo.GetVertexCount(), ChildCounter{1});
    for (auto i = ChildCounter{0}; i < GetChildCount(foo); ++i)
    {
        EXPECT_EQ(GetVertexRadius(foo, i), vertexRadius);
    }
    EXPECT_EQ(GetMassData(foo), expectedMassData);
    
    const auto child = GetChild(foo, 0);
    EXPECT_EQ(child, expectedDistanceProxy);
}

TEST(ChainShapeConf, TwoVertexLikeEdge)
{
    const auto vertexRadius = 1_m;
    const auto density = NonNegative<AreaDensity>(1_kgpm2);
    const auto locations = std::array<Length2, 2>{{
        Length2{0_m, 0_m}, Length2(4_m, 0_m)
    }};
    
    auto conf = ChainShapeConf{};
    conf.density = density;
    conf.vertexRadius = vertexRadius;
    conf.Add(locations[0]);
    conf.Add(locations[1]);
    auto foo = ChainShapeConf{conf};
    EXPECT_EQ(GetChildCount(foo), ChildCounter{1});
    EXPECT_EQ(foo.GetVertexCount(), ChildCounter{2});
    for (auto i = ChildCounter{0}; i < GetChildCount(foo); ++i)
    {
        EXPECT_EQ(GetVertexRadius(foo, i), vertexRadius);
    }
}

TEST(ChainShapeConf, TwoVertexDpLikeEdgeDp)
{
    const auto vertexRadius = 1_m;
    const auto density = NonNegative<AreaDensity>(1_kgpm2);
    const auto locations = std::array<Length2, 2>{{
        Length2{0_m, 0_m}, Length2(4_m, 0_m)
    }};
    const auto normals = std::array<UnitVec, 2>{{UnitVec::GetTop(), UnitVec::GetBottom()}};
    const auto expectedDistanceProxy = DistanceProxy{vertexRadius, 2, data(locations), data(normals)};
    
    auto conf = ChainShapeConf{};
    conf.density = density;
    conf.vertexRadius = vertexRadius;
    conf.Add(locations[0]);
    conf.Add(locations[1]);
    auto foo = ChainShapeConf{conf};
    ASSERT_EQ(GetChildCount(foo), ChildCounter{1});
    
    const auto child = GetChild(foo, 0);
    EXPECT_EQ(child, expectedDistanceProxy);
}

TEST(ChainShapeConf, TwoVertexMassLikeEdgeMass)
{
    const auto vertexRadius = 1_m;
    const auto density = NonNegative<AreaDensity>(1_kgpm2);
    const auto locations = std::array<Length2, 2>{{
        Length2{0_m, 0_m}, Length2(4_m, 0_m)
    }};
    const auto expectedMassData = ::GetMassData(vertexRadius, density, locations[0], locations[1]);
    
    auto conf = ChainShapeConf{};
    conf.density = density;
    conf.vertexRadius = vertexRadius;
    conf.Add(locations[0]);
    conf.Add(locations[1]);
    auto foo = ChainShapeConf{conf};
    
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

TEST(ChainShapeConf, FourVertex)
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

    auto conf = ChainShapeConf{};
    conf.density = density;
    conf.vertexRadius = vertexRadius;
    conf.Set(std::vector<Length2>(begin(locations), end(locations)));
    auto foo = ChainShapeConf{conf};
    EXPECT_EQ(GetChildCount(foo), ChildCounter{4});
    EXPECT_EQ(foo.GetVertexCount(), ChildCounter{5});
    for (auto i = ChildCounter{0}; i < GetChildCount(foo); ++i)
    {
        EXPECT_EQ(GetVertexRadius(foo, i), vertexRadius);
    }
    const auto massData = GetMassData(foo);
    EXPECT_EQ(massData.center, (Length2{}));
    const auto expectedMass = Mass{edgeMassData0.mass} * Real(4);
    EXPECT_EQ(massData.mass, NonNegative<Mass>{expectedMass});
}

TEST(ChainShapeConf, WithCircleVertices)
{
    const auto circleRadius = 4_m;
    const auto vertices = GetCircleVertices(circleRadius, 4, 0_deg, Real(1) / Real(2));
    const auto density = 1_kgpm2;
    const auto vertexRadius = 1_m / 10;

    auto conf = ChainShapeConf{};
    conf.density = density;
    conf.vertexRadius = vertexRadius;
    conf.Set(vertices);
    auto foo = ChainShapeConf{conf};
    EXPECT_EQ(GetChildCount(foo), ChildCounter{4});
    EXPECT_EQ(foo.GetVertexCount(), ChildCounter{5});
    for (auto i = ChildCounter{0}; i < GetChildCount(foo); ++i)
    {
        EXPECT_EQ(GetVertexRadius(foo, i), vertexRadius);
    }
    const auto massData = GetMassData(foo);
    EXPECT_NEAR(static_cast<double>(Real(GetX(massData.center) / 1_m)), 0.0, 0.0001);
    EXPECT_NEAR(static_cast<double>(Real(GetY(massData.center) / 1_m)), 2.4142134189605713, 0.0001);
}

TEST(ChainShapeConf, TooManyVertices)
{
    const auto density = 1_kgpm2;
    const auto vertexRadius = 1_m / 10;
    
    auto conf = ChainShapeConf{};
    conf.density = density;
    conf.vertexRadius = vertexRadius;
    EXPECT_THROW(conf.Set(std::vector<Length2>(MaxChildCount + 1)), InvalidArgument);
}

TEST(ChainShapeConf, Equality)
{
    EXPECT_TRUE(ChainShapeConf() == ChainShapeConf());

    EXPECT_FALSE(ChainShapeConf().UseVertexRadius(10_m) == ChainShapeConf());
    EXPECT_TRUE(ChainShapeConf().UseVertexRadius(10_m) == ChainShapeConf().UseVertexRadius(10_m));
    
    EXPECT_FALSE(ChainShapeConf().UseDensity(10_kgpm2) == ChainShapeConf());
    EXPECT_TRUE(ChainShapeConf().UseDensity(10_kgpm2) == ChainShapeConf().UseDensity(10_kgpm2));
    
    EXPECT_FALSE(ChainShapeConf().UseFriction(Real(10)) == ChainShapeConf());
    EXPECT_TRUE(ChainShapeConf().UseFriction(Real(10)) == ChainShapeConf().UseFriction(Real(10)));
    
    EXPECT_FALSE(ChainShapeConf().UseRestitution(Real(10)) == ChainShapeConf());
    EXPECT_TRUE(ChainShapeConf().UseRestitution(Real(10)) == ChainShapeConf().UseRestitution(Real(10)));

    EXPECT_FALSE(ChainShapeConf().Add(Length2(1_m, 2_m)) == ChainShapeConf());
    EXPECT_TRUE(ChainShapeConf().Add(Length2(1_m, 2_m)) == ChainShapeConf().Add(Length2(1_m, 2_m)));
}

TEST(ChainShapeConf, Inequality)
{
    EXPECT_FALSE(ChainShapeConf() != ChainShapeConf());

    EXPECT_TRUE(ChainShapeConf().UseVertexRadius(10_m) != ChainShapeConf());
    EXPECT_FALSE(ChainShapeConf().UseVertexRadius(10_m) != ChainShapeConf().UseVertexRadius(10_m));
    
    EXPECT_TRUE(ChainShapeConf().UseDensity(10_kgpm2) != ChainShapeConf());
    EXPECT_FALSE(ChainShapeConf().UseDensity(10_kgpm2) != ChainShapeConf().UseDensity(10_kgpm2));
    
    EXPECT_TRUE(ChainShapeConf().UseFriction(Real(10)) != ChainShapeConf());
    EXPECT_FALSE(ChainShapeConf().UseFriction(Real(10)) != ChainShapeConf().UseFriction(Real(10)));
    
    EXPECT_TRUE(ChainShapeConf().UseRestitution(Real(10)) != ChainShapeConf());
    EXPECT_FALSE(ChainShapeConf().UseRestitution(Real(10)) != ChainShapeConf().UseRestitution(Real(10)));

    EXPECT_TRUE(ChainShapeConf().Add(Length2(1_m, 2_m)) != ChainShapeConf());
    EXPECT_FALSE(ChainShapeConf().Add(Length2(1_m, 2_m)) != ChainShapeConf().Add(Length2(1_m, 2_m)));
}

TEST(ChainShapeConf, GetSquareChainShapeConf)
{
    const auto conf = GetChainShapeConf(2_m);
    const auto childCount = GetChildCount(conf);
    EXPECT_EQ(childCount, decltype(childCount){4});
    for (auto i = ChildCounter{0}; i < childCount; ++i)
    {
        const auto childI = GetChild(conf, i);
        EXPECT_EQ(childI.GetVertexCount(), decltype(childI.GetVertexCount()){2});
        for (auto j = ChildCounter{0}; j < childCount; ++j)
        {
            const auto childJ = GetChild(conf, j);
            if (i != j)
            {
                EXPECT_NE(childI, childJ);
            }
        }
    }

    auto vertices = std::set<Length2, LexicographicalLess<Length2>>();
    for (auto i = ChildCounter{0}; i < childCount; ++i)
    {
        const auto child = GetChild(conf, i);
        const auto numVertices = child.GetVertexCount();
        for (auto j = decltype(numVertices){0}; j < numVertices; ++j)
        {
            vertices.insert(child.GetVertex(j));
        }
    }
    EXPECT_EQ(vertices.size(), decltype(vertices.size()){4});
}

TEST(ChainShapeConf, GetAabbChainShapeConf)
{
    const auto v0 = Length2{2_m, -3_m};
    const auto v1 = Length2{2_m,  4_m};
    const auto v2 = Length2{1_m,  4_m};
    const auto v3 = Length2{1_m, -3_m};
    auto aabb = AABB{};
    Include(aabb, v0);
    Include(aabb, v1);
    Include(aabb, v2);
    Include(aabb, v3);
    const auto conf = GetChainShapeConf(aabb);
    EXPECT_EQ(conf.GetChildCount(), ChildCounter(4));
    EXPECT_EQ(conf.GetVertexCount(), ChildCounter(5));
    EXPECT_EQ(conf.GetVertex(0), v0);
    EXPECT_EQ(conf.GetVertex(1), v1);
    EXPECT_EQ(conf.GetVertex(2), v2);
    EXPECT_EQ(conf.GetVertex(3), v3);
    EXPECT_EQ(conf.GetVertex(4), v0);
}
