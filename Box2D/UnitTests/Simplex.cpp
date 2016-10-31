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

TEST(SimplexVertices, ByteSizeIs88)
{
	EXPECT_EQ(sizeof(SimplexVertices), size_t(88));
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
	const auto ia = SimplexVertex::size_type{2};
	const auto ib = SimplexVertex::size_type{7};
	const auto sv = SimplexVertex{va, ia, vb, ib};
	
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
	const auto ia = SimplexVertex::size_type{2};
	const auto ib = SimplexVertex::size_type{7};
	const auto sv = SimplexVertex{va, ia, vb, ib};
	
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

TEST(Simplex, Get2)
{
	const auto va0 = Vec2{-4, 33};
	const auto vb0 = Vec2{float_t(901.5), float_t(0.06)};
	const auto ia0 = SimplexVertex::size_type{2};
	const auto ib0 = SimplexVertex::size_type{7};
	const auto sv0 = SimplexVertex{va0, ia0, vb0, ib0};

	const auto va1 = Vec2{-4, 33};
	const auto vb1 = Vec2{float_t(901.5), float_t(0.06)};
	const auto ia1 = SimplexVertex::size_type{2};
	const auto ib1 = SimplexVertex::size_type{7};
	const auto sv1 = SimplexVertex{va1, ia1, vb1, ib1};
	
	const auto simplex = Simplex::Get(sv0, sv1);
	EXPECT_EQ(simplex.GetSize(), decltype(simplex.GetSize()){1});
	
	ASSERT_GT(simplex.GetSize(), decltype(simplex.GetSize()){0});
	const auto sv_new_0 = simplex.GetSimplexVertex(0);
	EXPECT_EQ(sv_new_0.GetPointA(), va0);
	EXPECT_EQ(sv_new_0.indexPair.a, ia0);
	EXPECT_EQ(sv_new_0.GetPointB(), vb0);
	EXPECT_EQ(sv_new_0.indexPair.b, ib0);
	
	const auto ce_new_0 = simplex.GetCoefficient(0);
	EXPECT_EQ(ce_new_0, float_t(1));
}

TEST(Simplex, GetOfSimplexVertices)
{
	Simplex foo;

	const auto roo = Simplex::Get(foo.GetSimplexVertices());
	
	EXPECT_EQ(foo.GetSize(), roo.GetSize());
}
