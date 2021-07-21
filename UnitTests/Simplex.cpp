/*
 * Copyright (c) 2021 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#include <PlayRho/Collision/Simplex.hpp>

using namespace playrho;
using namespace playrho::d2;

TEST(SimplexCache, ByteSize)
{
    // Check size at test runtime instead of compile-time via static_assert to avoid stopping
    // builds and to report actual size rather than just reporting that expected size is wrong.
    switch (sizeof(Real))
    {
        case  4: EXPECT_EQ(sizeof(Simplex::Cache), std::size_t(12)); break;
        case  8: EXPECT_EQ(sizeof(Simplex::Cache), std::size_t(16)); break;
        case 16: EXPECT_EQ(sizeof(Simplex::Cache), std::size_t(32)); break;
        default: FAIL(); break;
    }
}

TEST(SimplexCache, IndexPairsByteSize)
{
    EXPECT_EQ(sizeof(IndexPair3), std::size_t(6));
}

TEST(SimplexCache, DefaultInit)
{
    {
        Simplex::Cache foo;
        EXPECT_EQ(std::uint8_t{0}, GetNumValidIndices(foo.indices));
        EXPECT_FALSE(IsValid(foo.metric));
    }
    {
        Simplex::Cache foo{};
        EXPECT_EQ(std::uint8_t{0}, GetNumValidIndices(foo.indices));
        EXPECT_FALSE(IsValid(foo.metric));
    }
}

TEST(SimplexCache, InitializingConstructor)
{
    {
        const auto metric = Real(.3);
        const auto indices = IndexPair3{{InvalidIndexPair, InvalidIndexPair, InvalidIndexPair}};
        Simplex::Cache foo{metric, indices};
        
        EXPECT_EQ(GetNumValidIndices(foo.indices), decltype(GetNumValidIndices(foo.indices)){0});
        EXPECT_EQ(foo.metric, metric);
    }
    {
        const auto ip0 = IndexPair{0, 0};
        const auto ip1 = IndexPair{1, 0};
        const auto metric = Real(-1.4);
        Simplex::Cache foo{metric, IndexPair3{{ip0, ip1, InvalidIndexPair}}};
        
        EXPECT_EQ(GetNumValidIndices(foo.indices), decltype(GetNumValidIndices(foo.indices)){2});
        EXPECT_EQ(std::get<0>(foo.indices), ip0);
        EXPECT_EQ(std::get<1>(foo.indices), ip1);
        EXPECT_EQ(foo.metric, metric);
    }
    {
        const auto ip0 = IndexPair{0, 0};
        const auto ip1 = IndexPair{1, 0};
        const auto ip2 = IndexPair{4, 3};
        const auto metric = Real(-1.4);
        Simplex::Cache foo{metric, IndexPair3{{ip0, ip1, ip2}}};
        
        EXPECT_EQ(GetNumValidIndices(foo.indices), decltype(GetNumValidIndices(foo.indices)){3});
        EXPECT_EQ(std::get<0>(foo.indices), ip0);
        EXPECT_EQ(std::get<1>(foo.indices), ip1);
        EXPECT_EQ(std::get<2>(foo.indices), ip2);
        EXPECT_EQ(foo.metric, metric);
    }
}

TEST(SimplexCache, Assignment)
{
    const auto metric = Real(.3);
    const auto indices = IndexPair3{{InvalidIndexPair, InvalidIndexPair, InvalidIndexPair}};
    Simplex::Cache foo{metric, indices};
    
    ASSERT_EQ(GetNumValidIndices(foo.indices), decltype(GetNumValidIndices(foo.indices)){0});
    ASSERT_EQ(foo.metric, metric);
    
    const auto ip0 = IndexPair{0, 0};
    const auto ip1 = IndexPair{1, 0};
    const auto ip2 = IndexPair{4, 3};
    const auto roo_metric = Real(-1.4);
    Simplex::Cache roo{roo_metric, IndexPair3{{ip0, ip1, ip2}}};
    
    foo = roo;
    
    EXPECT_EQ(GetNumValidIndices(foo.indices), decltype(GetNumValidIndices(foo.indices)){3});
    EXPECT_EQ(std::get<0>(foo.indices), ip0);
    EXPECT_EQ(std::get<1>(foo.indices), ip1);
    EXPECT_EQ(std::get<2>(foo.indices), ip2);
    EXPECT_EQ(foo.metric, roo_metric);
}

TEST(SimplexEdgeList, ByteSize)
{
    switch (sizeof(Real))
    {
        case  4: EXPECT_EQ(sizeof(SimplexEdges), std::size_t(64)); break;
        case  8: EXPECT_EQ(sizeof(SimplexEdges), std::size_t(128)); break;
        case 16: EXPECT_EQ(sizeof(SimplexEdges), std::size_t(256)); break;
        default: FAIL();
    }
}

TEST(Simplex, ByteSize)
{
    switch (sizeof(Real))
    {
        case  4: EXPECT_EQ(sizeof(Simplex), std::size_t(80)); break;
        case  8: EXPECT_EQ(sizeof(Simplex), std::size_t(160)); break;
        case 16: EXPECT_EQ(sizeof(Simplex), std::size_t(320)); break;
        default: FAIL();
    }
}

TEST(Simplex, DefaultConstruction)
{
    {
        Simplex foo;
        EXPECT_EQ(size(foo), decltype(size(foo)){0});
        EXPECT_EQ(foo.GetEdges().size(), decltype(foo.GetEdges().size()){0});
        EXPECT_EQ(foo.GetEdges().max_size(), decltype(foo.GetEdges().max_size()){3});
    }
    {
        Simplex foo{};
        EXPECT_EQ(size(foo), decltype(size(foo)){0});
        EXPECT_EQ(foo.GetEdges().size(), decltype(foo.GetEdges().size()){0});
        EXPECT_EQ(foo.GetEdges().max_size(), decltype(foo.GetEdges().max_size()){3});
    }
}

TEST(Simplex, Get1)
{
    const auto va = Length2{-4_m, 33_m};
    const auto vb = Length2{901.5_m, 0.06_m};
    const auto ia = VertexCounter{2};
    const auto ib = VertexCounter{7};
    const auto sv = SimplexEdge{va, ia, vb, ib};
    
    const auto simplex = Simplex::Get(sv);
    EXPECT_EQ(size(simplex), decltype(size(simplex)){1});

    ASSERT_GT(size(simplex), decltype(size(simplex)){0});
    const auto sv_new = simplex.GetSimplexEdge(0);
    EXPECT_EQ(sv.GetPointA(), sv_new.GetPointA());
    EXPECT_EQ(sv.GetIndexA(), sv_new.GetIndexA());
    EXPECT_EQ(sv.GetPointB(), sv_new.GetPointB());
    EXPECT_EQ(sv.GetIndexB(), sv_new.GetIndexB());
    
    const auto ce_new = simplex.GetCoefficient(0);
    EXPECT_EQ(ce_new, Real(1));
}

TEST(Simplex, Get2_of_same)
{
    const auto va = Length2{-4_m, 33_m};
    const auto vb = Length2{901.5_m, 0.06_m};
    const auto ia = VertexCounter{2};
    const auto ib = VertexCounter{7};
    const auto sv = SimplexEdge{va, ia, vb, ib};
    
    const auto simplex = Simplex::Get(sv, sv);
    EXPECT_EQ(size(simplex), decltype(size(simplex)){1});
    
    ASSERT_GT(size(simplex), decltype(size(simplex)){0});

    const auto sv_new = simplex.GetSimplexEdge(0);
    EXPECT_EQ(sv_new.GetPointA(), va);
    EXPECT_EQ(sv_new.GetIndexA(), ia);
    EXPECT_EQ(sv_new.GetPointB(), vb);
    EXPECT_EQ(sv_new.GetIndexB(), ib);
    
    const auto ce_new = simplex.GetCoefficient(0);
    EXPECT_EQ(ce_new, Real(1));
}

TEST(Simplex, Get2_fwd_perp)
{
    const auto va0 = Length2{-4_m, 33_m};
    const auto vb0 = Length2{901.5_m, 0.06_m};
    const auto ia0 = VertexCounter{2};
    const auto ib0 = VertexCounter{7};
    const auto sv0 = SimplexEdge{va0, ia0, vb0, ib0};

    const auto va1 = GetFwdPerpendicular(va0);
    const auto vb1 = GetFwdPerpendicular(vb0);
    const auto ia1 = VertexCounter{4};
    const auto ib1 = VertexCounter{1};
    const auto sv1 = SimplexEdge{va1, ia1, vb1, ib1};
    
    const auto simplex = Simplex::Get(sv0, sv1);
    EXPECT_EQ(size(simplex), decltype(size(simplex)){2});
    
    ASSERT_GT(size(simplex), decltype(size(simplex)){0});

    const auto sv_new_0 = simplex.GetSimplexEdge(0);
    EXPECT_EQ(sv_new_0.GetPointA(), va0);
    EXPECT_EQ(sv_new_0.GetIndexA(), ia0);
    EXPECT_EQ(sv_new_0.GetPointB(), vb0);
    EXPECT_EQ(sv_new_0.GetIndexB(), ib0);
    
    const auto ce_new_0 = simplex.GetCoefficient(0);
    EXPECT_TRUE(AlmostEqual(ce_new_0, Real(0.5)));
    
    ASSERT_GT(size(simplex), decltype(size(simplex)){1});
    
    const auto sv_new_1 = simplex.GetSimplexEdge(1);
    EXPECT_EQ(sv_new_1.GetPointA(), va1);
    EXPECT_EQ(sv_new_1.GetIndexA(), ia1);
    EXPECT_EQ(sv_new_1.GetPointB(), vb1);
    EXPECT_EQ(sv_new_1.GetIndexB(), ib1);
    
    const auto ce_new_1 = simplex.GetCoefficient(1);
    EXPECT_TRUE(AlmostEqual(ce_new_1, Real(0.5)));
}

TEST(Simplex, Get2_rev_perp)
{
    const auto va0 = Length2{-4_m, 33_m};
    const auto vb0 = Length2{901.5_m, 0.06_m};
    const auto ia0 = VertexCounter{2};
    const auto ib0 = VertexCounter{7};
    const auto sv0 = SimplexEdge{va0, ia0, vb0, ib0};
    
    const auto va1 = GetRevPerpendicular(va0);
    const auto vb1 = GetRevPerpendicular(vb0);
    const auto ia1 = VertexCounter{4};
    const auto ib1 = VertexCounter{1};
    const auto sv1 = SimplexEdge{va1, ia1, vb1, ib1};
    
    const auto simplex = Simplex::Get(sv0, sv1);
    EXPECT_EQ(size(simplex), decltype(size(simplex)){2});
    
    ASSERT_GT(size(simplex), decltype(size(simplex)){0});
    
    const auto sv_new_0 = simplex.GetSimplexEdge(0);
    EXPECT_EQ(sv_new_0.GetPointA(), va0);
    EXPECT_EQ(sv_new_0.GetIndexA(), ia0);
    EXPECT_EQ(sv_new_0.GetPointB(), vb0);
    EXPECT_EQ(sv_new_0.GetIndexB(), ib0);
    
    const auto ce_new_0 = simplex.GetCoefficient(0);
    EXPECT_TRUE(AlmostEqual(ce_new_0, Real(0.5)));
    
    ASSERT_GT(size(simplex), decltype(size(simplex)){1});
    
    const auto sv_new_1 = simplex.GetSimplexEdge(1);
    EXPECT_EQ(sv_new_1.GetPointA(), va1);
    EXPECT_EQ(sv_new_1.GetIndexA(), ia1);
    EXPECT_EQ(sv_new_1.GetPointB(), vb1);
    EXPECT_EQ(sv_new_1.GetIndexB(), ib1);
    
    const auto ce_new_1 = simplex.GetCoefficient(1);
    EXPECT_TRUE(AlmostEqual(ce_new_1, Real(0.5)));
}

TEST(Simplex, Get2_rot_plus_45)
{
    const auto va0 = Length2{-4_m, 33_m};
    const auto vb0 = Length2{901.5_m, 0.06_m};
    const auto ia0 = VertexCounter{2};
    const auto ib0 = VertexCounter{7};
    const auto sv0 = SimplexEdge{va0, ia0, vb0, ib0};
    
    const auto va1 = Rotate(va0, UnitVec::Get(45_deg));
    const auto vb1 = Rotate(vb0, UnitVec::Get(45_deg));
    const auto ia1 = VertexCounter{4};
    const auto ib1 = VertexCounter{1};
    const auto sv1 = SimplexEdge{va1, ia1, vb1, ib1};
    
    const auto simplex = Simplex::Get(sv0, sv1);
    EXPECT_EQ(size(simplex), decltype(size(simplex)){2});
    
    ASSERT_GT(size(simplex), decltype(size(simplex)){0});
    
    const auto sv_new_0 = simplex.GetSimplexEdge(0);
    EXPECT_EQ(sv_new_0.GetPointA(), va0);
    EXPECT_EQ(sv_new_0.GetIndexA(), ia0);
    EXPECT_EQ(sv_new_0.GetPointB(), vb0);
    EXPECT_EQ(sv_new_0.GetIndexB(), ib0);
    
    const auto ce_new_0 = simplex.GetCoefficient(0);
    EXPECT_TRUE(AlmostEqual(ce_new_0, Real(0.5)));
    
    ASSERT_GT(size(simplex), decltype(size(simplex)){1});
    
    const auto sv_new_1 = simplex.GetSimplexEdge(1);
    EXPECT_EQ(sv_new_1.GetPointA(), va1);
    EXPECT_EQ(sv_new_1.GetIndexA(), ia1);
    EXPECT_EQ(sv_new_1.GetPointB(), vb1);
    EXPECT_EQ(sv_new_1.GetIndexB(), ib1);
    
    const auto ce_new_1 = simplex.GetCoefficient(1);
    EXPECT_TRUE(AlmostEqual(ce_new_1, Real(0.5)));
}

TEST(Simplex, Get2_rot45_half)
{
    const auto va0 = Length2{-4_m, 33_m}; // upper left
    const auto vb0 = Length2{901_m, 6_m}; // lower right
    const auto ia0 = VertexCounter{2};
    const auto ib0 = VertexCounter{7};
    const auto sv0 = SimplexEdge{va0, ia0, vb0, ib0};
    
    const auto va1 = Rotate(va0, UnitVec::Get(45_deg)) / 2; // Vec2{-13.081475, 10.253049}
    const auto vb1 = Rotate(vb0, UnitVec::Get(45_deg)) / 2; // Vec2{316.4303, 320.67291}
    EXPECT_NEAR(double(Real{GetX(va1) / Meter}), -13.081475, 0.001);
    EXPECT_NEAR(double(Real{GetY(va1) / Meter}),  10.253049, 0.001);
    EXPECT_NEAR(double(Real{GetX(vb1) / Meter}), 316.4303,   0.001);
    EXPECT_NEAR(double(Real{GetY(vb1) / Meter}), 320.67291,  0.001);
    const auto ia1 = VertexCounter{4};
    const auto ib1 = VertexCounter{1};
    const auto sv1 = SimplexEdge{va1, ia1, vb1, ib1};

    const auto w1 = vb0 - va0; // Vec2{901, 6} - Vec2{-4, 33} = Vec2{905, -27}
    EXPECT_TRUE(AlmostEqual(Real{GetX(w1) / Meter}, Real(905)));
    EXPECT_TRUE(AlmostEqual(Real{GetY(w1) / Meter}, Real(-27)));
    const auto w2 = vb1 - va1; // Vec2{316.4303, 320.67291} - Vec2{-13.081475, 10.253049} = Vec2{329.51178, 310.41986}
    EXPECT_NEAR(double(Real{GetX(w2) / Meter}), 329.51178, 0.001);
    EXPECT_NEAR(double(Real{GetY(w2) / Meter}), 310.41986, 0.001);
    
    const auto e12 = w2 - w1; // Vec2{329.51178, 310.41986} - Vec2{905, -27} = Vec2{-575.48822, 337.41986}
    EXPECT_NEAR(double(Real{GetX(e12) / Meter}), -575.48822, 0.001);
    EXPECT_NEAR(double(Real{GetY(e12) / Meter}),  337.41986, 0.001);

    const auto d12_2 = Area{-Dot(w1, e12)}; // -Dot(Vec2{905, -27}, Vec2{-575.48822, 337.41986}) = 529927.19
    EXPECT_NEAR(double(Real{d12_2 / SquareMeter}), 529927.19, 1.0);

    const auto d12_1 = Area{Dot(w2, e12)}; // Dot(Vec2{329.51178, 310.41986}, Vec2{-575.48822, 337.41986}) = -84888.312
    EXPECT_NEAR(double(Real{d12_1 / SquareMeter}), -84888.312, 1.0);

    const auto simplex = Simplex::Get(sv0, sv1);
    
    EXPECT_EQ(size(simplex), decltype(size(simplex)){1});
    
    ASSERT_GT(size(simplex), decltype(size(simplex)){0});
    
    const auto sv_new_0 = simplex.GetSimplexEdge(0);
    EXPECT_EQ(sv_new_0.GetPointA(), va1);
    EXPECT_EQ(sv_new_0.GetIndexA(), ia1);
    EXPECT_EQ(sv_new_0.GetPointB(), vb1);
    EXPECT_EQ(sv_new_0.GetIndexB(), ib1);
    
    const auto ce_new_0 = simplex.GetCoefficient(0);
    EXPECT_TRUE(AlmostEqual(ce_new_0, Real(1)));
}

TEST(Simplex, GetOfSimplexVertices)
{
    Simplex foo;
    const auto roo = Simplex::Get(foo.GetEdges());
    EXPECT_EQ(size(foo), size(roo));
}

TEST(Simplex, CalcSearchDirectionOfEmpty)
{
    auto se = SimplexEdges{};
    EXPECT_EQ(CalcSearchDirection(se), Length2{});
}

TEST(Simplex, CalcMetricOfEmpty)
{
    auto se = SimplexEdges{};
    EXPECT_EQ(Simplex::CalcMetric(se), Real(0));
}
