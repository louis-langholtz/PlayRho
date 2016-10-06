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
#include <Box2D/Dynamics/Contacts/ContactSolver.h>
#include <Box2D/Dynamics/Contacts/PositionConstraint.hpp>
#include <Box2D/Collision/Shapes/PolygonShape.h>
#include <Box2D/Collision/Manifold.hpp>

using namespace box2d;

TEST(ContactSolver, ByteSizeIs40)
{
	EXPECT_EQ(sizeof(ContactSolver), size_t(40));
}

TEST(ContactSolver, ZeroCountInit)
{
	ContactSolver solver{nullptr, nullptr, 0, nullptr, nullptr};
	EXPECT_TRUE(solver.SolvePositionConstraints());
	solver.SolveVelocityConstraints();
	solver.UpdateVelocityConstraints();
	EXPECT_TRUE(solver.SolveTOIPositionConstraints(0, 0));
}

TEST(ContactSolver, SolveTouching)
{
	auto pA = Position{Vec2{-2, 0}, 0};
	auto pB = Position{Vec2{+2, 0}, 0};

	const auto dim = float_t(2);
	const auto shape = PolygonShape(dim, dim);
	const auto xfmA = Transformation(pA.c, Rot{pA.a});
	const auto xfmB = Transformation(pB.c, Rot{pB.a});
	const auto manifold = CollideShapes(shape, xfmA, shape, xfmB);
	ASSERT_EQ(manifold.GetType(), Manifold::e_faceA);
	ASSERT_EQ(manifold.GetPointCount(), 2);

	const auto indexA = PositionConstraint::BodyData::index_type{0};
	const auto indexB = PositionConstraint::BodyData::index_type{0};
	const auto lcA = Vec2{0, 0};
	const auto lcB = Vec2{0, 0};
	const auto bA = PositionConstraint::BodyData{indexA, float_t(1), float_t(1), lcA};
	const auto bB = PositionConstraint::BodyData{indexB, float_t(1), float_t(1), lcB};
	PositionConstraint pc{manifold, bA, 0, bB, 0};
	
	const auto old_pA = pA;
	const auto old_pB = pB;

	const auto min_sep = Solve(pc, pA, pB, Baumgarte, 0, MaxLinearCorrection);
	
	EXPECT_EQ(min_sep, 0);
	
	EXPECT_EQ(old_pA.c.x, pA.c.x);
	EXPECT_EQ(old_pA.c.y, pA.c.y);
	EXPECT_EQ(old_pA.a, pA.a);

	EXPECT_EQ(old_pB.c.x, pB.c.x);
	EXPECT_EQ(old_pB.c.y, pB.c.y);
	EXPECT_EQ(old_pB.a, pB.a);
}

TEST(ContactSolver, SolveOverlappingZeroRateDoesntMove)
{
	auto pA = Position{Vec2{0, 0}, 0};
	auto pB = Position{Vec2{0, 0}, 0};
	
	const auto dim = float_t(2);
	const auto shape = PolygonShape(dim, dim);
	const auto xfmA = Transformation(Vec2_zero, Rot{0});
	const auto xfmB = Transformation(Vec2_zero, Rot{0});
	const auto manifold = CollideShapes(shape, xfmA, shape, xfmB);
	ASSERT_EQ(manifold.GetType(), Manifold::e_faceA);
	ASSERT_EQ(manifold.GetPointCount(), 2);
	
	const auto indexA = PositionConstraint::BodyData::index_type{0};
	const auto indexB = PositionConstraint::BodyData::index_type{0};
	const auto lcA = Vec2{};
	const auto lcB = Vec2{};
	const auto bA = PositionConstraint::BodyData{indexA, float_t(1), float_t(1), lcA};
	const auto bB = PositionConstraint::BodyData{indexB, float_t(1), float_t(1), lcB};
	PositionConstraint pc{manifold, bA, 0, bB, 0};
	
	const auto old_pA = pA;
	const auto old_pB = pB;
	
	const auto min_sep = Solve(pc, pA, pB, 0, -LinearSlop, MaxLinearCorrection);
	
	EXPECT_EQ(min_sep, -2 * dim);
	
	EXPECT_EQ(old_pA.c.x, pA.c.x);
	EXPECT_EQ(old_pA.c.y, pA.c.y);
	EXPECT_EQ(old_pA.a, pA.a);
	
	EXPECT_EQ(old_pB.c.x, pB.c.x);
	EXPECT_EQ(old_pB.c.y, pB.c.y);
	EXPECT_EQ(old_pB.a, pB.a);
}

TEST(ContactSolver, SolvePerfectlyOverlappingSquares)
{
	auto pA = Position{Vec2{0, 0}, 0};
	auto pB = Position{Vec2{0, 0}, 0};
	
	const auto dim = float_t(2);
	const auto shape = PolygonShape(dim, dim);
	const auto xfmA = Transformation(Vec2_zero, Rot{0});
	const auto xfmB = Transformation(Vec2_zero, Rot{0});
	const auto manifold = CollideShapes(shape, xfmA, shape, xfmB);
	ASSERT_EQ(manifold.GetType(), Manifold::e_faceA);
	ASSERT_EQ(manifold.GetPointCount(), 2);

	const auto indexA = PositionConstraint::BodyData::index_type{0};
	const auto indexB = PositionConstraint::BodyData::index_type{0};
	const auto lcA = Vec2{};
	const auto lcB = Vec2{};
	const auto bA = PositionConstraint::BodyData{indexA, float_t(1), float_t(1), lcA};
	const auto bB = PositionConstraint::BodyData{indexB, float_t(1), float_t(1), lcB};
	PositionConstraint pc{manifold, bA, GetRadius(shape), bB, GetRadius(shape)};
	
	const auto old_pA = pA;
	const auto old_pB = pB;

	const auto max_sep = -LinearSlop;
	const auto min_sep = Solve(pc, pA, pB, Baumgarte, max_sep, MaxLinearCorrection);
	
	EXPECT_LT(min_sep, max_sep);
	
	EXPECT_EQ(old_pA.c.x, pA.c.x);
	EXPECT_EQ(old_pA.c.y, pA.c.y);
	EXPECT_EQ(old_pA.a, pA.a);
	
	EXPECT_EQ(old_pB.c.x, pB.c.x);
	EXPECT_EQ(old_pB.c.y, pB.c.y);
	EXPECT_EQ(old_pB.a, pB.a);
}