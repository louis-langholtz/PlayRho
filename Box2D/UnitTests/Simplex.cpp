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
#include <Box2D/Collision/Simplex.hpp>

using namespace box2d;

TEST(SimplexCache, ByteSizeIs_12_16_or_32)
{
	switch (sizeof(RealNum))
	{
		case  4: EXPECT_EQ(sizeof(Simplex::Cache), size_t(12)); break;
		case  8: EXPECT_EQ(sizeof(Simplex::Cache), size_t(16)); break;
		case 16: EXPECT_EQ(sizeof(Simplex::Cache), size_t(32)); break;
		default: FAIL(); break;
	}
}

TEST(SimplexCache, IndexPairListByteSizeIs7)
{
	EXPECT_EQ(sizeof(Simplex::IndexPairs), size_t(7));
}

TEST(SimplexCache, DefaultInit)
{
	Simplex::Cache foo;
	EXPECT_EQ(decltype(foo.GetNumIndices()){0}, foo.GetNumIndices());
	EXPECT_FALSE(foo.IsMetricSet());
}

TEST(SimplexCache, InitializingConstructor)
{
	{
		const auto metric = RealNum(.3);
		const auto indices = Simplex::IndexPairs{};
		Simplex::Cache foo{metric, indices};
		
		EXPECT_EQ(foo.GetNumIndices(), decltype(foo.GetNumIndices()){0});
		EXPECT_TRUE(foo.IsMetricSet());
		EXPECT_EQ(foo.GetMetric(), metric);
	}
	{
		const auto ip0 = IndexPair{0, 0};
		const auto ip1 = IndexPair{1, 0};
		const auto ip2 = IndexPair{4, 3};
		const auto metric = RealNum(-1.4);
		Simplex::Cache foo{metric, Simplex::IndexPairs{ip0, ip1, ip2}};
		
		EXPECT_EQ(foo.GetNumIndices(), decltype(foo.GetNumIndices()){3});
		EXPECT_EQ(foo.GetIndexPair(0), ip0);
		EXPECT_EQ(foo.GetIndexPair(1), ip1);
		EXPECT_EQ(foo.GetIndexPair(2), ip2);
		EXPECT_TRUE(foo.IsMetricSet());
		EXPECT_EQ(foo.GetMetric(), metric);
	}
}

TEST(SimplexCache, Assignment)
{
	const auto metric = RealNum(.3);
	const auto indices = Simplex::IndexPairs{};
	Simplex::Cache foo{metric, indices};
	
	ASSERT_EQ(foo.GetNumIndices(), decltype(foo.GetNumIndices()){0});
	ASSERT_TRUE(foo.IsMetricSet());
	ASSERT_EQ(foo.GetMetric(), metric);
	
	const auto ip0 = IndexPair{0, 0};
	const auto ip1 = IndexPair{1, 0};
	const auto ip2 = IndexPair{4, 3};
	const auto roo_metric = RealNum(-1.4);
	Simplex::Cache roo{roo_metric, Simplex::IndexPairs{ip0, ip1, ip2}};
	
	foo = roo;
	
	EXPECT_EQ(foo.GetNumIndices(), decltype(foo.GetNumIndices()){3});
	EXPECT_EQ(foo.GetIndexPair(0), ip0);
	EXPECT_EQ(foo.GetIndexPair(1), ip1);
	EXPECT_EQ(foo.GetIndexPair(2), ip2);
	EXPECT_TRUE(foo.IsMetricSet());
	EXPECT_EQ(foo.GetMetric(), roo_metric);
}

TEST(SimplexEdgeList, ByteSizeIs_88_176_or_352)
{
	switch (sizeof(RealNum))
	{
		case  4: EXPECT_EQ(sizeof(Simplex::Edges), size_t(88)); break;
		case  8: EXPECT_EQ(sizeof(Simplex::Edges), size_t(176)); break;
		case 16: EXPECT_EQ(sizeof(Simplex::Edges), size_t(352)); break;
		default: FAIL();
	}
}

TEST(Simplex, ByteSizeIs_104_208_or_416)
{
	switch (sizeof(RealNum))
	{
		case  4: EXPECT_EQ(sizeof(Simplex), size_t(104)); break;
		case  8: EXPECT_EQ(sizeof(Simplex), size_t(208)); break;
		case 16: EXPECT_EQ(sizeof(Simplex), size_t(416)); break;
		default: FAIL();
	}
}

TEST(Simplex, DefaultConstruction)
{
	Simplex foo;
	
	EXPECT_EQ(foo.GetSize(), decltype(foo.GetSize()){0});
}

TEST(Simplex, Get1)
{
	const auto va = Vec2{-4, 33};
	const auto vb = Vec2{RealNum(901.5), RealNum(0.06)};
	const auto ia = SimplexEdge::index_type{2};
	const auto ib = SimplexEdge::index_type{7};
	const auto sv = SimplexEdge{va, ia, vb, ib};
	
	const auto simplex = Simplex::Get(sv);
	EXPECT_EQ(simplex.GetSize(), decltype(simplex.GetSize()){1});

	ASSERT_GT(simplex.GetSize(), decltype(simplex.GetSize()){0});
	const auto sv_new = simplex.GetSimplexEdge(0);
	EXPECT_EQ(sv.GetPointA(), sv_new.GetPointA());
	EXPECT_EQ(sv.GetIndexA(), sv_new.GetIndexA());
	EXPECT_EQ(sv.GetPointB(), sv_new.GetPointB());
	EXPECT_EQ(sv.GetIndexB(), sv_new.GetIndexB());
	
	const auto ce_new = simplex.GetCoefficient(0);
	EXPECT_EQ(ce_new, RealNum(1));
}

TEST(Simplex, Get2_of_same)
{
	const auto va = Vec2{-4, 33};
	const auto vb = Vec2{RealNum(901.5), RealNum(0.06)};
	const auto ia = SimplexEdge::index_type{2};
	const auto ib = SimplexEdge::index_type{7};
	const auto sv = SimplexEdge{va, ia, vb, ib};
	
	const auto simplex = Simplex::Get(sv, sv);
	EXPECT_EQ(simplex.GetSize(), decltype(simplex.GetSize()){1});
	
	ASSERT_GT(simplex.GetSize(), decltype(simplex.GetSize()){0});

	const auto sv_new = simplex.GetSimplexEdge(0);
	EXPECT_EQ(sv_new.GetPointA(), va);
	EXPECT_EQ(sv_new.GetIndexA(), ia);
	EXPECT_EQ(sv_new.GetPointB(), vb);
	EXPECT_EQ(sv_new.GetIndexB(), ib);
	
	const auto ce_new = simplex.GetCoefficient(0);
	EXPECT_EQ(ce_new, RealNum(1));
}

TEST(Simplex, Get2_fwd_perp)
{
	const auto va0 = Vec2{-4, 33};
	const auto vb0 = Vec2{RealNum(901.5), RealNum(0.06)};
	const auto ia0 = SimplexEdge::index_type{2};
	const auto ib0 = SimplexEdge::index_type{7};
	const auto sv0 = SimplexEdge{va0, ia0, vb0, ib0};

	const auto va1 = GetFwdPerpendicular(va0);
	const auto vb1 = GetFwdPerpendicular(vb0);
	const auto ia1 = SimplexEdge::index_type{4};
	const auto ib1 = SimplexEdge::index_type{1};
	const auto sv1 = SimplexEdge{va1, ia1, vb1, ib1};
	
	const auto simplex = Simplex::Get(sv0, sv1);
	EXPECT_EQ(simplex.GetSize(), decltype(simplex.GetSize()){2});
	
	ASSERT_GT(simplex.GetSize(), decltype(simplex.GetSize()){0});

	const auto sv_new_0 = simplex.GetSimplexEdge(0);
	EXPECT_EQ(sv_new_0.GetPointA(), va0);
	EXPECT_EQ(sv_new_0.GetIndexA(), ia0);
	EXPECT_EQ(sv_new_0.GetPointB(), vb0);
	EXPECT_EQ(sv_new_0.GetIndexB(), ib0);
	
	const auto ce_new_0 = simplex.GetCoefficient(0);
	EXPECT_TRUE(almost_equal(ce_new_0, RealNum(0.5)));
	
	ASSERT_GT(simplex.GetSize(), decltype(simplex.GetSize()){1});
	
	const auto sv_new_1 = simplex.GetSimplexEdge(1);
	EXPECT_EQ(sv_new_1.GetPointA(), va1);
	EXPECT_EQ(sv_new_1.GetIndexA(), ia1);
	EXPECT_EQ(sv_new_1.GetPointB(), vb1);
	EXPECT_EQ(sv_new_1.GetIndexB(), ib1);
	
	const auto ce_new_1 = simplex.GetCoefficient(1);
	EXPECT_TRUE(almost_equal(ce_new_1, RealNum(0.5)));
}

TEST(Simplex, Get2_rev_perp)
{
	const auto va0 = Vec2{-4, 33};
	const auto vb0 = Vec2{RealNum(901.5), RealNum(0.06)};
	const auto ia0 = SimplexEdge::index_type{2};
	const auto ib0 = SimplexEdge::index_type{7};
	const auto sv0 = SimplexEdge{va0, ia0, vb0, ib0};
	
	const auto va1 = GetRevPerpendicular(va0);
	const auto vb1 = GetRevPerpendicular(vb0);
	const auto ia1 = SimplexEdge::index_type{4};
	const auto ib1 = SimplexEdge::index_type{1};
	const auto sv1 = SimplexEdge{va1, ia1, vb1, ib1};
	
	const auto simplex = Simplex::Get(sv0, sv1);
	EXPECT_EQ(simplex.GetSize(), decltype(simplex.GetSize()){2});
	
	ASSERT_GT(simplex.GetSize(), decltype(simplex.GetSize()){0});
	
	const auto sv_new_0 = simplex.GetSimplexEdge(0);
	EXPECT_EQ(sv_new_0.GetPointA(), va0);
	EXPECT_EQ(sv_new_0.GetIndexA(), ia0);
	EXPECT_EQ(sv_new_0.GetPointB(), vb0);
	EXPECT_EQ(sv_new_0.GetIndexB(), ib0);
	
	const auto ce_new_0 = simplex.GetCoefficient(0);
	EXPECT_TRUE(almost_equal(ce_new_0, RealNum(0.5)));
	
	ASSERT_GT(simplex.GetSize(), decltype(simplex.GetSize()){1});
	
	const auto sv_new_1 = simplex.GetSimplexEdge(1);
	EXPECT_EQ(sv_new_1.GetPointA(), va1);
	EXPECT_EQ(sv_new_1.GetIndexA(), ia1);
	EXPECT_EQ(sv_new_1.GetPointB(), vb1);
	EXPECT_EQ(sv_new_1.GetIndexB(), ib1);
	
	const auto ce_new_1 = simplex.GetCoefficient(1);
	EXPECT_TRUE(almost_equal(ce_new_1, RealNum(0.5)));
}

TEST(Simplex, Get2_rot_plus_45)
{
	const auto va0 = Vec2{-4, 33};
	const auto vb0 = Vec2{RealNum(901.5), RealNum(0.06)};
	const auto ia0 = SimplexEdge::index_type{2};
	const auto ib0 = SimplexEdge::index_type{7};
	const auto sv0 = SimplexEdge{va0, ia0, vb0, ib0};
	
	const auto va1 = Rotate(va0, UnitVec2{45_deg});
	const auto vb1 = Rotate(vb0, UnitVec2{45_deg});
	const auto ia1 = SimplexEdge::index_type{4};
	const auto ib1 = SimplexEdge::index_type{1};
	const auto sv1 = SimplexEdge{va1, ia1, vb1, ib1};
	
	const auto simplex = Simplex::Get(sv0, sv1);
	EXPECT_EQ(simplex.GetSize(), decltype(simplex.GetSize()){2});
	
	ASSERT_GT(simplex.GetSize(), decltype(simplex.GetSize()){0});
	
	const auto sv_new_0 = simplex.GetSimplexEdge(0);
	EXPECT_EQ(sv_new_0.GetPointA(), va0);
	EXPECT_EQ(sv_new_0.GetIndexA(), ia0);
	EXPECT_EQ(sv_new_0.GetPointB(), vb0);
	EXPECT_EQ(sv_new_0.GetIndexB(), ib0);
	
	const auto ce_new_0 = simplex.GetCoefficient(0);
	EXPECT_TRUE(almost_equal(ce_new_0, RealNum(0.5)));
	
	ASSERT_GT(simplex.GetSize(), decltype(simplex.GetSize()){1});
	
	const auto sv_new_1 = simplex.GetSimplexEdge(1);
	EXPECT_EQ(sv_new_1.GetPointA(), va1);
	EXPECT_EQ(sv_new_1.GetIndexA(), ia1);
	EXPECT_EQ(sv_new_1.GetPointB(), vb1);
	EXPECT_EQ(sv_new_1.GetIndexB(), ib1);
	
	const auto ce_new_1 = simplex.GetCoefficient(1);
	EXPECT_TRUE(almost_equal(ce_new_1, RealNum(0.5)));
}

TEST(Simplex, Get2_rot45_half)
{
	const auto va0 = Vec2{-4, 33}; // upper left
	const auto vb0 = Vec2{901, 6}; // lower right
	const auto ia0 = SimplexEdge::index_type{2};
	const auto ib0 = SimplexEdge::index_type{7};
	const auto sv0 = SimplexEdge{va0, ia0, vb0, ib0};
	
	const auto va1 = Rotate(va0, UnitVec2{45_deg}) / 2; // Vec2{-13.081475, 10.253049}
	const auto vb1 = Rotate(vb0, UnitVec2{45_deg}) / 2; // Vec2{316.4303, 320.67291}
	EXPECT_NEAR(double(va1.x), -13.081475, 0.00001);
	EXPECT_NEAR(double(va1.y),  10.253049, 0.00001);
	EXPECT_NEAR(double(vb1.x), 316.4303,   0.0001);
	EXPECT_NEAR(double(vb1.y), 320.67291,  0.0001);
	const auto ia1 = SimplexEdge::index_type{4};
	const auto ib1 = SimplexEdge::index_type{1};
	const auto sv1 = SimplexEdge{va1, ia1, vb1, ib1};

	const auto w1 = vb0 - va0; // Vec2{901, 6} - Vec2{-4, 33} = Vec2{905, -27}
	EXPECT_TRUE(almost_equal(w1.x, RealNum(905)));
	EXPECT_TRUE(almost_equal(w1.y, RealNum(-27)));
	const auto w2 = vb1 - va1; // Vec2{316.4303, 320.67291} - Vec2{-13.081475, 10.253049} = Vec2{329.51178, 310.41986}
	EXPECT_NEAR(double(w2.x), 329.51178, 0.001);
	EXPECT_NEAR(double(w2.y), 310.41986, 0.001);
	
	const auto e12 = w2 - w1; // Vec2{329.51178, 310.41986} - Vec2{905, -27} = Vec2{-575.48822, 337.41986}
	EXPECT_NEAR(double(e12.x), -575.48822, 0.001);
	EXPECT_NEAR(double(e12.y),  337.41986, 0.001);

	const auto d12_2 = -Dot(w1, e12); // -Dot(Vec2{905, -27}, Vec2{-575.48822, 337.41986}) = 529927.19
	EXPECT_NEAR(double(d12_2), 529927.19, 0.1);

	const auto d12_1 = Dot(w2, e12); // Dot(Vec2{329.51178, 310.41986}, Vec2{-575.48822, 337.41986}) = -84888.312
	EXPECT_NEAR(double(d12_1), -84888.312, 0.1);

	const auto simplex = Simplex::Get(sv0, sv1);
	
	EXPECT_EQ(simplex.GetSize(), decltype(simplex.GetSize()){1});
	
	ASSERT_GT(simplex.GetSize(), decltype(simplex.GetSize()){0});
	
	const auto sv_new_0 = simplex.GetSimplexEdge(0);
	EXPECT_EQ(sv_new_0.GetPointA(), va1);
	EXPECT_EQ(sv_new_0.GetIndexA(), ia1);
	EXPECT_EQ(sv_new_0.GetPointB(), vb1);
	EXPECT_EQ(sv_new_0.GetIndexB(), ib1);
	
	const auto ce_new_0 = simplex.GetCoefficient(0);
	EXPECT_TRUE(almost_equal(ce_new_0, RealNum(1)));
}

TEST(Simplex, GetOfSimplexVertices)
{
	Simplex foo;

	const auto roo = Simplex::Get(foo.GetEdges());
	
	EXPECT_EQ(foo.GetSize(), roo.GetSize());
}
