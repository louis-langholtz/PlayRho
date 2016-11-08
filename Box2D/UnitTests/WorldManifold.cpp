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
#include <Box2D/Collision/WorldManifold.hpp>
#include <Box2D/Collision/Manifold.hpp>

using namespace box2d;

TEST(WorldManifold, ByteSizeIs36)
{
	EXPECT_EQ(sizeof(WorldManifold), size_t(36));
}

TEST(WorldManifold, default_construction)
{
	const auto wm = WorldManifold{};
	
	EXPECT_EQ(wm.GetPointCount(), decltype(wm.GetPointCount()){0});
	EXPECT_FALSE(IsValid(wm.GetNormal()));
}

TEST(WorldManifold, GetWorldManifoldForCirclesTouchingManifold)
{
	const auto manifold = Manifold::GetForCircles(Vec2{0, 0}, Manifold::Point{Vec2{0, 0}});
	const auto xfA = Transformation{Vec2{4-1, 0}, UnitVec2{0_deg}};
	const auto xfB = Transformation{Vec2{4+1, 0}, UnitVec2{0_deg}};
	const auto rA = float_t(1);
	const auto rB = float_t(1);
	const auto wm = GetWorldManifold(manifold, xfA, rA, xfB, rB);
	
	EXPECT_EQ(wm.GetPointCount(), decltype(wm.GetPointCount()){1});
	EXPECT_TRUE(IsValid(wm.GetNormal()));
	EXPECT_EQ(wm.GetNormal(), Vec2(1, 0));
	EXPECT_EQ(wm.GetSeparation(0), decltype(wm.GetSeparation(0)){0});
	EXPECT_EQ(wm.GetPoint(0), Vec2(4, 0));
}

TEST(WorldManifold, GetWorldManifoldForCirclesHalfOverlappingManifold)
{
	const auto manifold = Manifold::GetForCircles(Vec2{0, 0}, Manifold::Point{Vec2{0, 0}});
	const auto xfA = Transformation{Vec2{7-0.5, 0}, UnitVec2{0_deg}};
	const auto xfB = Transformation{Vec2{7+0.5, 0}, UnitVec2{0_deg}};
	const auto rA = float_t(1);
	const auto rB = float_t(1);
	const auto wm = GetWorldManifold(manifold, xfA, rA, xfB, rB);
	
	EXPECT_EQ(wm.GetPointCount(), decltype(wm.GetPointCount()){1});
	EXPECT_TRUE(IsValid(wm.GetNormal()));
	EXPECT_EQ(wm.GetNormal(), Vec2(1, 0));
	EXPECT_EQ(wm.GetSeparation(0), decltype(wm.GetSeparation(0)){-1});
	EXPECT_EQ(wm.GetPoint(0), Vec2(7, 0));
}

TEST(WorldManifold, GetWorldManifoldForCirclesFullyOverlappingManifold)
{
	const auto manifold = Manifold::GetForCircles(Vec2{0, 0}, Manifold::Point{Vec2{0, 0}});
	const auto xfA = Transformation{Vec2{3-0, 0}, UnitVec2{0_deg}};
	const auto xfB = Transformation{Vec2{3+0, 0}, UnitVec2{0_deg}};
	const auto rA = float_t(1);
	const auto rB = float_t(1);
	const auto wm = GetWorldManifold(manifold, xfA, rA, xfB, rB);
	
	EXPECT_EQ(wm.GetPointCount(), decltype(wm.GetPointCount()){1});
	EXPECT_EQ(wm.GetSeparation(0), decltype(wm.GetSeparation(0)){-2});
	if (IsValid(wm.GetNormal()))
	{
		EXPECT_EQ(wm.GetPoint(0), Vec2(3, 0));
	}
	else
	{
		EXPECT_FALSE(IsValid(wm.GetPoint(0)));
	}
}
