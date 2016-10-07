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

TEST(ContactSolver, SolveHorizontallyTouchingDoesntMove)
{
	const auto old_pA = Position{Vec2{-2, 0}, 0};
	const auto old_pB = Position{Vec2{+2, 0}, 0};

	const auto dim = float_t(2);
	const auto shape = PolygonShape(dim, dim);
	const auto xfmA = Transformation(old_pA.c, Rot{old_pA.a});
	const auto xfmB = Transformation(old_pB.c, Rot{old_pB.a});
	const auto manifold = CollideShapes(shape, xfmA, shape, xfmB);
	ASSERT_EQ(manifold.GetType(), Manifold::e_faceA);
	ASSERT_EQ(manifold.GetPointCount(), 2);

	const auto indexA = PositionConstraint::BodyData::index_type{0};
	const auto indexB = PositionConstraint::BodyData::index_type{0};
	const auto lcA = Vec2{0, 0};
	const auto lcB = Vec2{0, 0};
	const auto bA = PositionConstraint::BodyData{indexA, float_t(1), float_t(1), lcA};
	const auto bB = PositionConstraint::BodyData{indexB, float_t(1), float_t(1), lcB};
	const auto pc = PositionConstraint{manifold, bA, 0, bB, 0};
	
	const auto solution = Solve(pc, old_pA, old_pB, Baumgarte, 0, MaxLinearCorrection);
	
	EXPECT_EQ(solution.min_separation, 0);
	
	EXPECT_EQ(old_pA.c.x, solution.pos_a.c.x);
	EXPECT_EQ(old_pA.c.y, solution.pos_a.c.y);
	EXPECT_EQ(old_pA.a, solution.pos_a.a);

	EXPECT_EQ(old_pB.c.x, solution.pos_b.c.x);
	EXPECT_EQ(old_pB.c.y, solution.pos_b.c.y);
	EXPECT_EQ(old_pB.a, solution.pos_b.a);
}

TEST(ContactSolver, SolveVerticallyTouchingDoesntMove)
{
	const auto old_pA = Position{Vec2{0, -2}, 0};
	const auto old_pB = Position{Vec2{0, +2}, 0};
	
	const auto dim = float_t(2);
	const auto shape = PolygonShape(dim, dim);
	const auto xfmA = Transformation(old_pA.c, Rot{old_pA.a});
	const auto xfmB = Transformation(old_pB.c, Rot{old_pB.a});
	const auto manifold = CollideShapes(shape, xfmA, shape, xfmB);
	ASSERT_EQ(manifold.GetType(), Manifold::e_faceA);
	ASSERT_EQ(manifold.GetPointCount(), 2);
	
	const auto indexA = PositionConstraint::BodyData::index_type{0};
	const auto indexB = PositionConstraint::BodyData::index_type{0};
	const auto lcA = Vec2{0, 0};
	const auto lcB = Vec2{0, 0};
	const auto bA = PositionConstraint::BodyData{indexA, float_t(1), float_t(1), lcA};
	const auto bB = PositionConstraint::BodyData{indexB, float_t(1), float_t(1), lcB};
	const auto pc = PositionConstraint{manifold, bA, 0, bB, 0};
	
	const auto solution = Solve(pc, old_pA, old_pB, Baumgarte, 0, MaxLinearCorrection);
	
	EXPECT_EQ(solution.min_separation, 0);
	
	EXPECT_EQ(old_pA.c.x, solution.pos_a.c.x);
	EXPECT_EQ(old_pA.c.y, solution.pos_a.c.y);
	EXPECT_EQ(old_pA.a, solution.pos_a.a);
	
	EXPECT_EQ(old_pB.c.x, solution.pos_b.c.x);
	EXPECT_EQ(old_pB.c.y, solution.pos_b.c.y);
	EXPECT_EQ(old_pB.a, solution.pos_b.a);
}

TEST(ContactSolver, SolveOverlappingZeroRateDoesntMove)
{
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
	const auto pc = PositionConstraint{manifold, bA, 0, bB, 0};

	const auto old_pA = Position{Vec2{0, 0}, 0};
	const auto old_pB = Position{Vec2{0, 0}, 0};
	
	const auto solution = Solve(pc, old_pA, old_pB, 0, -LinearSlop, MaxLinearCorrection);

	EXPECT_EQ(solution.min_separation, -2 * dim);
	
	EXPECT_EQ(old_pA.c.x, solution.pos_a.c.x);
	EXPECT_EQ(old_pA.c.y, solution.pos_a.c.y);
	EXPECT_EQ(old_pA.a, solution.pos_a.a);
	
	EXPECT_EQ(old_pB.c.x, solution.pos_b.c.x);
	EXPECT_EQ(old_pB.c.y, solution.pos_b.c.y);
	EXPECT_EQ(old_pB.a, solution.pos_b.a);
}

TEST(ContactSolver, SolveHorizontallyOverlappingMovesHorizontallyOnly)
{
	const auto ctr_x = float_t(100);
	const auto old_pA = Position{{ctr_x - 1, 0}, 0};
	const auto old_pB = Position{{ctr_x + 1, 0}, 0};
	
	const auto dim = float_t(2);
	const auto shape = PolygonShape(dim, dim);
	const auto xfmA = Transformation(old_pA.c, Rot{old_pA.a});
	const auto xfmB = Transformation(old_pB.c, Rot{old_pB.a});
	const auto manifold = CollideShapes(shape, xfmA, shape, xfmB);
	ASSERT_EQ(manifold.GetType(), Manifold::e_faceA);
	ASSERT_EQ(manifold.GetLocalNormal().x, float_t(1));
	ASSERT_EQ(manifold.GetLocalNormal().y, float_t(0));
	ASSERT_EQ(manifold.GetLocalPoint().x, float_t(2));
	ASSERT_EQ(manifold.GetLocalPoint().y, float_t(0));
	ASSERT_EQ(manifold.GetPointCount(), 2);
	ASSERT_EQ(manifold.GetPoint(0).localPoint.x, float_t(-2));
	ASSERT_EQ(manifold.GetPoint(0).localPoint.y, float_t(+2));
	ASSERT_EQ(manifold.GetPoint(1).localPoint.x, float_t(-2));
	ASSERT_EQ(manifold.GetPoint(1).localPoint.y, float_t(-2));
	
	const auto indexA = PositionConstraint::BodyData::index_type{0};
	const auto indexB = PositionConstraint::BodyData::index_type{0};
	const auto lcA = Vec2{0, 0};
	const auto lcB = Vec2{0, 0};
	const auto bA = PositionConstraint::BodyData{indexA, float_t(1), float_t(1), lcA};
	const auto bB = PositionConstraint::BodyData{indexB, float_t(1), float_t(1), lcB};
	const auto pc = PositionConstraint{manifold, bA, 0, bB, 0};
	
	const auto solution = Solve(pc, old_pA, old_pB, Baumgarte, 0, MaxLinearCorrection);
	
	EXPECT_FLOAT_EQ(solution.min_separation, float_t(-2.002398));
		
	// object a just moves left
	EXPECT_LT(solution.pos_a.c.x, old_pA.c.x);
	EXPECT_EQ(solution.pos_a.c.y, old_pA.c.y);
	EXPECT_EQ(solution.pos_a.a, old_pA.a);

	{
		// confirm object a moves more in x direction than in y direction.
		const auto mov_a = solution.pos_a - old_pA;	
		EXPECT_GT(Abs(mov_a.c.x), Abs(mov_a.c.y));
	}
	
	// object b just moves right
	EXPECT_GT(solution.pos_b.c.x, old_pB.c.x);
	EXPECT_EQ(solution.pos_b.c.y, old_pB.c.y);
	EXPECT_EQ(solution.pos_b.a, old_pB.a);

	{
		// confirm object a moves more in x direction than in y direction.
		const auto mov_b = solution.pos_b - old_pB;
		EXPECT_GT(Abs(mov_b.c.x), Abs(mov_b.c.y));
	}
}

TEST(ContactSolver, SolvePerfectlyOverlappingSquares)
{
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
	const auto pc = PositionConstraint{manifold, bA, GetRadius(shape), bB, GetRadius(shape)};
	const auto max_sep = -LinearSlop;

	const auto old_pA = Position{Vec2{0, 0}, 0};
	const auto old_pB = Position{Vec2{0, 0}, 0};

	const auto solution = Solve(pc, old_pA, old_pB, Baumgarte, max_sep, MaxLinearCorrection);
	
	EXPECT_LT(solution.min_separation, max_sep);
	
	// object a moves down left and rotates counter-clockwise
	EXPECT_LT(solution.pos_a.c.x, old_pA.c.x);
	EXPECT_LT(solution.pos_a.c.y, old_pA.c.y);
	EXPECT_LT(solution.pos_a.a, old_pA.a);
	
	// object b moves up right and rotates clockwise.
	EXPECT_GT(solution.pos_b.c.x, old_pB.c.x);
	EXPECT_GT(solution.pos_b.c.y, old_pB.c.y);
	EXPECT_GT(solution.pos_b.a, old_pB.a);
}