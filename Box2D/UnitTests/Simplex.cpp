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
#include <Box2D/Collision/Simplex.hpp>

using namespace box2d;

TEST(SimplexEdgeList, ByteSizeIs88)
{
	EXPECT_EQ(sizeof(SimplexEdgeList), size_t(88));
}

TEST(Simplex, ByteSizeIs104)
{
	EXPECT_EQ(sizeof(Simplex), size_t(104));
}

TEST(Simplex, DefaultConstruction)
{
	Simplex foo;
	
	EXPECT_EQ(foo.GetSize(), decltype(foo.GetSize()){0});
}

TEST(Simplex, Get1)
{
	const auto va = Vec2{-4, 33};
	const auto vb = Vec2{float_t(901.5), float_t(0.06)};
	const auto ia = SimplexEdge::index_type{2};
	const auto ib = SimplexEdge::index_type{7};
	const auto sv = SimplexEdge{va, ia, vb, ib};
	
	const auto simplex = Simplex::Get(sv);
	EXPECT_EQ(simplex.GetSize(), decltype(simplex.GetSize()){1});

	ASSERT_GT(simplex.GetSize(), decltype(simplex.GetSize()){0});
	const auto sv_new = simplex.GetSimplexVertex(0);
	EXPECT_EQ(sv.GetPointA(), sv_new.GetPointA());
	EXPECT_EQ(sv.indexPair.a, sv_new.indexPair.a);
	EXPECT_EQ(sv.GetPointB(), sv_new.GetPointB());
	EXPECT_EQ(sv.indexPair.b, sv_new.indexPair.b);
	
	const auto ce_new = simplex.GetCoefficient(0);
	EXPECT_EQ(ce_new, float_t(1));
}

TEST(Simplex, Get2_of_same)
{
	const auto va = Vec2{-4, 33};
	const auto vb = Vec2{float_t(901.5), float_t(0.06)};
	const auto ia = SimplexEdge::index_type{2};
	const auto ib = SimplexEdge::index_type{7};
	const auto sv = SimplexEdge{va, ia, vb, ib};
	
	const auto simplex = Simplex::Get(sv, sv);
	EXPECT_EQ(simplex.GetSize(), decltype(simplex.GetSize()){1});
	
	ASSERT_GT(simplex.GetSize(), decltype(simplex.GetSize()){0});

	const auto sv_new = simplex.GetSimplexVertex(0);
	EXPECT_EQ(sv_new.GetPointA(), va);
	EXPECT_EQ(sv_new.indexPair.a, ia);
	EXPECT_EQ(sv_new.GetPointB(), vb);
	EXPECT_EQ(sv_new.indexPair.b, ib);
	
	const auto ce_new = simplex.GetCoefficient(0);
	EXPECT_EQ(ce_new, float_t(1));
}

TEST(Simplex, Get2_fwd_perp)
{
	const auto va0 = Vec2{-4, 33};
	const auto vb0 = Vec2{float_t(901.5), float_t(0.06)};
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

	const auto sv_new_0 = simplex.GetSimplexVertex(0);
	EXPECT_EQ(sv_new_0.GetPointA(), va0);
	EXPECT_EQ(sv_new_0.indexPair.a, ia0);
	EXPECT_EQ(sv_new_0.GetPointB(), vb0);
	EXPECT_EQ(sv_new_0.indexPair.b, ib0);
	
	const auto ce_new_0 = simplex.GetCoefficient(0);
	EXPECT_FLOAT_EQ(ce_new_0, float_t(0.5));
	
	ASSERT_GT(simplex.GetSize(), decltype(simplex.GetSize()){1});
	
	const auto sv_new_1 = simplex.GetSimplexVertex(1);
	EXPECT_EQ(sv_new_1.GetPointA(), va1);
	EXPECT_EQ(sv_new_1.indexPair.a, ia1);
	EXPECT_EQ(sv_new_1.GetPointB(), vb1);
	EXPECT_EQ(sv_new_1.indexPair.b, ib1);
	
	const auto ce_new_1 = simplex.GetCoefficient(1);
	EXPECT_FLOAT_EQ(ce_new_1, float_t(0.5));
}

TEST(Simplex, Get2_rev_perp)
{
	const auto va0 = Vec2{-4, 33};
	const auto vb0 = Vec2{float_t(901.5), float_t(0.06)};
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
	
	const auto sv_new_0 = simplex.GetSimplexVertex(0);
	EXPECT_EQ(sv_new_0.GetPointA(), va0);
	EXPECT_EQ(sv_new_0.indexPair.a, ia0);
	EXPECT_EQ(sv_new_0.GetPointB(), vb0);
	EXPECT_EQ(sv_new_0.indexPair.b, ib0);
	
	const auto ce_new_0 = simplex.GetCoefficient(0);
	EXPECT_FLOAT_EQ(ce_new_0, float_t(0.5));
	
	ASSERT_GT(simplex.GetSize(), decltype(simplex.GetSize()){1});
	
	const auto sv_new_1 = simplex.GetSimplexVertex(1);
	EXPECT_EQ(sv_new_1.GetPointA(), va1);
	EXPECT_EQ(sv_new_1.indexPair.a, ia1);
	EXPECT_EQ(sv_new_1.GetPointB(), vb1);
	EXPECT_EQ(sv_new_1.indexPair.b, ib1);
	
	const auto ce_new_1 = simplex.GetCoefficient(1);
	EXPECT_FLOAT_EQ(ce_new_1, float_t(0.5));
}

TEST(Simplex, Get2_rot_plus_45)
{
	const auto va0 = Vec2{-4, 33};
	const auto vb0 = Vec2{float_t(901.5), float_t(0.06)};
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
	
	const auto sv_new_0 = simplex.GetSimplexVertex(0);
	EXPECT_EQ(sv_new_0.GetPointA(), va0);
	EXPECT_EQ(sv_new_0.indexPair.a, ia0);
	EXPECT_EQ(sv_new_0.GetPointB(), vb0);
	EXPECT_EQ(sv_new_0.indexPair.b, ib0);
	
	const auto ce_new_0 = simplex.GetCoefficient(0);
	EXPECT_FLOAT_EQ(ce_new_0, float_t(0.5));
	
	ASSERT_GT(simplex.GetSize(), decltype(simplex.GetSize()){1});
	
	const auto sv_new_1 = simplex.GetSimplexVertex(1);
	EXPECT_EQ(sv_new_1.GetPointA(), va1);
	EXPECT_EQ(sv_new_1.indexPair.a, ia1);
	EXPECT_EQ(sv_new_1.GetPointB(), vb1);
	EXPECT_EQ(sv_new_1.indexPair.b, ib1);
	
	const auto ce_new_1 = simplex.GetCoefficient(1);
	EXPECT_FLOAT_EQ(ce_new_1, float_t(0.5));
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
	EXPECT_FLOAT_EQ(va1.x, float_t(-13.081475));
	EXPECT_FLOAT_EQ(va1.y, float_t(10.253049));
	EXPECT_FLOAT_EQ(vb1.x, float_t(316.4303));
	EXPECT_FLOAT_EQ(vb1.y, float_t(320.67291));
	const auto ia1 = SimplexEdge::index_type{4};
	const auto ib1 = SimplexEdge::index_type{1};
	const auto sv1 = SimplexEdge{va1, ia1, vb1, ib1};

	const auto w1 = vb0 - va0; // Vec2{901, 6} - Vec2{-4, 33} = Vec2{905, -27}
	EXPECT_FLOAT_EQ(w1.x, float_t(905));
	EXPECT_FLOAT_EQ(w1.y, float_t(-27));
	const auto w2 = vb1 - va1; // Vec2{316.4303, 320.67291} - Vec2{-13.081475, 10.253049} = Vec2{329.51178, 310.41986}
	EXPECT_FLOAT_EQ(w2.x, float_t(329.51178));
	EXPECT_FLOAT_EQ(w2.y, float_t(310.41986));
	
	const auto e12 = w2 - w1; // Vec2{329.51178, 310.41986} - Vec2{905, -27} = Vec2{-575.48822, 337.41986}
	EXPECT_FLOAT_EQ(e12.x, float_t(-575.48822));
	EXPECT_FLOAT_EQ(e12.y, float_t(337.41986));

	const auto d12_2 = -Dot(w1, e12); // -Dot(Vec2{905, -27}, Vec2{-575.48822, 337.41986}) = 529927.19
	EXPECT_FLOAT_EQ(d12_2, float_t(529927.19));

	const auto d12_1 = Dot(w2, e12); // Dot(Vec2{329.51178, 310.41986}, Vec2{-575.48822, 337.41986}) = -84888.312
	EXPECT_FLOAT_EQ(d12_1, float_t(-84888.312));

	const auto simplex = Simplex::Get(sv0, sv1);
	
	EXPECT_EQ(simplex.GetSize(), decltype(simplex.GetSize()){1});
	
	ASSERT_GT(simplex.GetSize(), decltype(simplex.GetSize()){0});
	
	const auto sv_new_0 = simplex.GetSimplexVertex(0);
	EXPECT_EQ(sv_new_0.GetPointA(), va1);
	EXPECT_EQ(sv_new_0.indexPair.a, ia1);
	EXPECT_EQ(sv_new_0.GetPointB(), vb1);
	EXPECT_EQ(sv_new_0.indexPair.b, ib1);
	
	const auto ce_new_0 = simplex.GetCoefficient(0);
	EXPECT_FLOAT_EQ(ce_new_0, float_t(1));
}

TEST(Simplex, GetOfSimplexVertices)
{
	Simplex foo;

	const auto roo = Simplex::Get(foo.GetSimplexVertices());
	
	EXPECT_EQ(foo.GetSize(), roo.GetSize());
}
