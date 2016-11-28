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
#include <Box2D/Dynamics/Contacts/ContactSolver.hpp>
#include <Box2D/Dynamics/Contacts/PositionConstraint.hpp>
#include <Box2D/Collision/Shapes/PolygonShape.hpp>
#include <Box2D/Collision/CollideShapes.hpp>

using namespace box2d;

TEST(ContactSolver, SolvePosConstraintsForHorTouchingDoesntMove)
{
	const auto old_pA = Position{Vec2{-2, 0}, 0_deg};
	const auto old_pB = Position{Vec2{+2, 0}, 0_deg};

	const auto dim = float_t(2);
	const auto shape = PolygonShape(dim, dim);
	const auto xfmA = Transformation(old_pA.c, UnitVec2{old_pA.a});
	const auto xfmB = Transformation(old_pB.c, UnitVec2{old_pB.a});
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
	
	const auto conf = ConstraintSolverConf{};
	const auto solution = SolvePositionConstraint(pc, old_pA, true, old_pB, true, conf);
	
	EXPECT_EQ(solution.min_separation, 0);
	
	EXPECT_EQ(old_pA.c.x, solution.pos_a.c.x);
	EXPECT_EQ(old_pA.c.y, solution.pos_a.c.y);
	EXPECT_EQ(old_pA.a, solution.pos_a.a);

	EXPECT_EQ(old_pB.c.x, solution.pos_b.c.x);
	EXPECT_EQ(old_pB.c.y, solution.pos_b.c.y);
	EXPECT_EQ(old_pB.a, solution.pos_b.a);
}

TEST(ContactSolver, SolvePosConstraintsForVerTouchingDoesntMove)
{
	const auto old_pA = Position{Vec2{0, -2}, 0_deg};
	const auto old_pB = Position{Vec2{0, +2}, 0_deg};
	
	const auto dim = float_t(2);
	const auto shape = PolygonShape(dim, dim);
	const auto xfmA = Transformation(old_pA.c, UnitVec2{old_pA.a});
	const auto xfmB = Transformation(old_pB.c, UnitVec2{old_pB.a});
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
	
	const auto conf = ConstraintSolverConf{};
	const auto solution = SolvePositionConstraint(pc, old_pA, true, old_pB, true, conf);
	
	EXPECT_EQ(solution.min_separation, 0);
	
	EXPECT_EQ(old_pA.c.x, solution.pos_a.c.x);
	EXPECT_EQ(old_pA.c.y, solution.pos_a.c.y);
	EXPECT_EQ(old_pA.a, solution.pos_a.a);
	
	EXPECT_EQ(old_pB.c.x, solution.pos_b.c.x);
	EXPECT_EQ(old_pB.c.y, solution.pos_b.c.y);
	EXPECT_EQ(old_pB.a, solution.pos_b.a);
}

TEST(ContactSolver, SolvePosConstraintsForOverlappingZeroRateDoesntMove)
{
	const auto dim = float_t(2);
	const auto shape = PolygonShape(dim, dim);
	const auto xfmA = Transformation(Vec2_zero, UnitVec2{0_deg});
	const auto xfmB = Transformation(Vec2_zero, UnitVec2{0_deg});
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

	const auto old_pA = Position{Vec2{0, 0}, 0_deg};
	const auto old_pB = Position{Vec2{0, 0}, 0_deg};
	
	const auto maxLinearCorrection = std::numeric_limits<float_t>::infinity();
	const auto conf = ConstraintSolverConf{}.UseResolutionRate(0).UseMaxLinearCorrection(maxLinearCorrection);
	const auto solution = SolvePositionConstraint(pc, old_pA, true, old_pB, true, conf);

	EXPECT_EQ(solution.min_separation, -2 * dim);
	
	EXPECT_EQ(old_pA.c.x, solution.pos_a.c.x);
	EXPECT_EQ(old_pA.c.y, solution.pos_a.c.y);
	EXPECT_EQ(old_pA.a, solution.pos_a.a);
	
	EXPECT_EQ(old_pB.c.x, solution.pos_b.c.x);
	EXPECT_EQ(old_pB.c.y, solution.pos_b.c.y);
	EXPECT_EQ(old_pB.a, solution.pos_b.a);
}

TEST(ContactSolver, SolvePosConstraintsForHorOverlappingMovesHorOnly1)
{
	const auto ctr_x = float_t(100);
	
	// square A is left of square B
	const auto old_pA = Position{{ctr_x - 1, 0}, 0_deg};
	const auto old_pB = Position{{ctr_x + 1, 0}, 0_deg};
	
	const auto dim = float_t(2);
	const auto shape = PolygonShape(dim, dim);
	const auto xfmA = Transformation(old_pA.c, UnitVec2{old_pA.a});
	const auto xfmB = Transformation(old_pB.c, UnitVec2{old_pB.a});
	const auto manifold = CollideShapes(shape, xfmA, shape, xfmB);
	ASSERT_EQ(manifold.GetType(), Manifold::e_faceA);
	ASSERT_EQ(Vec2{manifold.GetLocalNormal()}, Vec2(+1, 0));
	ASSERT_EQ(manifold.GetLocalPoint(), Vec2(+2, 0));
	ASSERT_EQ(manifold.GetPointCount(), 2);
	ASSERT_EQ(manifold.GetPoint(0).localPoint, Vec2(-2, +2));
	ASSERT_EQ(manifold.GetPoint(1).localPoint, Vec2(-2, -2));
	
	const auto indexA = PositionConstraint::BodyData::index_type{0};
	const auto indexB = PositionConstraint::BodyData::index_type{0};
	const auto lcA = Vec2{0, 0};
	const auto lcB = Vec2{0, 0};
	const auto bA = PositionConstraint::BodyData{indexA, float_t(1), float_t(1), lcA};
	const auto bB = PositionConstraint::BodyData{indexB, float_t(1), float_t(1), lcB};
	const auto pc = PositionConstraint{manifold, bA, 0, bB, 0};
	
	const auto maxLinearCorrection = std::numeric_limits<float_t>::infinity();
	const auto conf = ConstraintSolverConf{}.UseResolutionRate(Baumgarte).UseMaxLinearCorrection(maxLinearCorrection);
	const auto solution = SolvePositionConstraint(pc, old_pA, true, old_pB, true, conf);
	
	EXPECT_FLOAT_EQ(solution.min_separation, float_t(-2)); // -2.002398
		
	// object a just moves left
	EXPECT_LT(solution.pos_a.c.x, old_pA.c.x);
	EXPECT_EQ(solution.pos_a.c.y, old_pA.c.y);
	EXPECT_EQ(solution.pos_a.a, old_pA.a);
	
	// object b just moves right
	EXPECT_GT(solution.pos_b.c.x, old_pB.c.x);
	EXPECT_EQ(solution.pos_b.c.y, old_pB.c.y);
	EXPECT_EQ(solution.pos_b.a, old_pB.a);
}

TEST(ContactSolver, SolvePosConstraintsForHorOverlappingMovesHorOnly2)
{
	const auto ctr_x = float_t(100);
	
	// square A is right of square B
	const auto old_pA =  Position{{ctr_x + 1, 0}, 0_deg};
	const auto old_pB = Position{{ctr_x - 1, 0}, 0_deg};
	
	const auto dim = float_t(2);
	const auto shape = PolygonShape(dim, dim);
	const auto xfmA = Transformation(old_pA.c, UnitVec2{old_pA.a});
	const auto xfmB = Transformation(old_pB.c, UnitVec2{old_pB.a});
	const auto manifold = CollideShapes(shape, xfmA, shape, xfmB);
	ASSERT_EQ(manifold.GetType(), Manifold::e_faceA);
	ASSERT_EQ(Vec2{manifold.GetLocalNormal()}, Vec2(-1, 0));
	ASSERT_EQ(manifold.GetLocalPoint(), Vec2(-2, 0));
	ASSERT_EQ(manifold.GetPointCount(), 2);
	ASSERT_EQ(manifold.GetPoint(0).localPoint, Vec2(+2, -2));
	ASSERT_EQ(manifold.GetPoint(1).localPoint, Vec2(+2, +2));
	
	const auto indexA = PositionConstraint::BodyData::index_type{0};
	const auto indexB = PositionConstraint::BodyData::index_type{0};
	const auto lcA = Vec2{0, 0};
	const auto lcB = Vec2{0, 0};
	const auto bA = PositionConstraint::BodyData{indexA, float_t(1), float_t(1), lcA};
	const auto bB = PositionConstraint::BodyData{indexB, float_t(1), float_t(1), lcB};
	const auto pc = PositionConstraint{manifold, bA, 0, bB, 0};
	
	const auto maxLinearCorrection = std::numeric_limits<float_t>::infinity();
	const auto conf = ConstraintSolverConf{}.UseResolutionRate(Baumgarte).UseMaxLinearCorrection(maxLinearCorrection);
	const auto solution = SolvePositionConstraint(pc, old_pA, true, old_pB, true, conf);
	
	EXPECT_FLOAT_EQ(solution.min_separation, float_t(-2)); // -2.002398
	
	// square A just moves right
	EXPECT_GT(solution.pos_a.c.x, old_pA.c.x);
	EXPECT_EQ(solution.pos_a.c.y, old_pA.c.y);
	EXPECT_EQ(solution.pos_a.a, old_pA.a);
	
	// square B just moves left
	EXPECT_LT(solution.pos_b.c.x, old_pB.c.x);
	EXPECT_EQ(solution.pos_b.c.y, old_pB.c.y);
	EXPECT_EQ(solution.pos_b.a, old_pB.a);
}

TEST(ContactSolver, SolvePosConstraintsForVerOverlappingMovesVerOnly1)
{
	const auto ctr_y = float_t(100);
	
	// square A is below square B
	const auto old_pA = Position{{0, ctr_y - 1}, 0_deg};
	const auto old_pB = Position{{0, ctr_y + 1}, 0_deg};
	
	const auto dim = float_t(2);
	const auto shape = PolygonShape(dim, dim);
	const auto xfmA = Transformation(old_pA.c, UnitVec2{old_pA.a});
	const auto xfmB = Transformation(old_pB.c, UnitVec2{old_pB.a});
	const auto manifold = CollideShapes(shape, xfmA, shape, xfmB);
	ASSERT_EQ(manifold.GetType(), Manifold::e_faceA);
	ASSERT_EQ(Vec2{manifold.GetLocalNormal()}, Vec2(0, 1));
	ASSERT_EQ(manifold.GetLocalPoint(), Vec2(0, 2));
	ASSERT_EQ(manifold.GetPointCount(), 2);
	ASSERT_EQ(manifold.GetPoint(0).localPoint, Vec2(-2, -2));
	ASSERT_EQ(manifold.GetPoint(1).localPoint, Vec2(+2, -2));
	
	const auto indexA = PositionConstraint::BodyData::index_type{0};
	const auto indexB = PositionConstraint::BodyData::index_type{0};
	const auto lcA = Vec2{0, 0};
	const auto lcB = Vec2{0, 0};
	const auto bA = PositionConstraint::BodyData{indexA, float_t(1), float_t(1), lcA};
	const auto bB = PositionConstraint::BodyData{indexB, float_t(1), float_t(1), lcB};
	const auto pc = PositionConstraint{manifold, bA, 0, bB, 0};
	
	const auto maxLinearCorrection = std::numeric_limits<float_t>::infinity();
	const auto conf = ConstraintSolverConf{}.UseResolutionRate(Baumgarte).UseMaxLinearCorrection(maxLinearCorrection);
	const auto solution = SolvePositionConstraint(pc, old_pA, true, old_pB, true, conf);
	
	EXPECT_FLOAT_EQ(solution.min_separation, float_t(-2)); // -2.002398
	
	// object a just moves down only
	EXPECT_EQ(solution.pos_a.c.x, old_pA.c.x);
	EXPECT_LT(solution.pos_a.c.y, old_pA.c.y);
	EXPECT_EQ(solution.pos_a.a, old_pA.a);
	
	{
		// confirm object a moves more in x direction than in y direction.
		const auto mov_a = solution.pos_a - old_pA;	
		EXPECT_LT(Abs(mov_a.c.x), Abs(mov_a.c.y));
	}
	
	// object b just moves up only
	EXPECT_EQ(solution.pos_b.c.x, old_pB.c.x);
	EXPECT_GT(solution.pos_b.c.y, old_pB.c.y);
	EXPECT_EQ(solution.pos_b.a, old_pB.a);
	
	{
		// confirm object a moves more in x direction than in y direction.
		const auto mov_b = solution.pos_b - old_pB;
		EXPECT_LT(Abs(mov_b.c.x), Abs(mov_b.c.y));
	}
}

TEST(ContactSolver, SolvePosConstraintsForVerOverlappingMovesVerOnly2)
{
	const auto ctr_y = float_t(100);

	// square A is above square B
	const auto old_pA = Position{{0, ctr_y + 1}, 0_deg};
	const auto old_pB = Position{{0, ctr_y - 1}, 0_deg};
	
	const auto dim = float_t(2);
	const auto shape = PolygonShape(dim, dim);
	const auto xfmA = Transformation(old_pA.c, UnitVec2{old_pA.a});
	const auto xfmB = Transformation(old_pB.c, UnitVec2{old_pB.a});
	const auto manifold = CollideShapes(shape, xfmA, shape, xfmB);
	
	ASSERT_EQ(manifold.GetType(), Manifold::e_faceA);
	ASSERT_EQ(Vec2{manifold.GetLocalNormal()}, Vec2(0, -1));
	ASSERT_EQ(manifold.GetLocalPoint(), Vec2(0, -2));
	ASSERT_EQ(manifold.GetPointCount(), 2);
	ASSERT_EQ(manifold.GetPoint(0).localPoint, Vec2(+2, +2));
	ASSERT_EQ(manifold.GetPoint(1).localPoint, Vec2(-2, +2));
	
	const auto indexA = PositionConstraint::BodyData::index_type{0};
	const auto indexB = PositionConstraint::BodyData::index_type{0};
	const auto lcA = Vec2{0, 0};
	const auto lcB = Vec2{0, 0};
	const auto bA = PositionConstraint::BodyData{indexA, float_t(1), float_t(1), lcA};
	const auto bB = PositionConstraint::BodyData{indexB, float_t(1), float_t(1), lcB};
	const auto pc = PositionConstraint{manifold, bA, 0, bB, 0};

	const auto maxLinearCorrection = std::numeric_limits<float_t>::infinity();
	const auto conf = ConstraintSolverConf{}.UseResolutionRate(Baumgarte).UseMaxLinearCorrection(maxLinearCorrection);
	const auto solution = SolvePositionConstraint(pc, old_pA, true, old_pB, true, conf);
	
	EXPECT_FLOAT_EQ(solution.min_separation, float_t(-2)); // -2.002398
	
	// square A just moves up only
	EXPECT_EQ(solution.pos_a.c.x, old_pA.c.x);
	EXPECT_GT(solution.pos_a.c.y, old_pA.c.y);
	EXPECT_EQ(solution.pos_a.a, old_pA.a);
	
	{
		// confirm object a moves more in x direction than in y direction.
		const auto mov_a = solution.pos_a - old_pA;	
		EXPECT_LT(Abs(mov_a.c.x), Abs(mov_a.c.y));
	}
	
	// square B just moves down only
	EXPECT_EQ(solution.pos_b.c.x, old_pB.c.x);
	EXPECT_LT(solution.pos_b.c.y, old_pB.c.y);
	EXPECT_EQ(solution.pos_b.a, old_pB.a);
	
	{
		// confirm object a moves more in x direction than in y direction.
		const auto mov_b = solution.pos_b - old_pB;
		EXPECT_LT(Abs(mov_b.c.x), Abs(mov_b.c.y));
	}
}

TEST(ContactSolver, SolvePosConstraintsForPerfectlyOverlappingSquares)
{
	const auto dim = float_t(2);
	const auto shape = PolygonShape(dim, dim);
	const auto xfmA = Transformation(Vec2_zero, UnitVec2{0_deg});
	const auto xfmB = Transformation(Vec2_zero, UnitVec2{0_deg});
	const auto manifold = CollideShapes(shape, xfmA, shape, xfmB);
	ASSERT_EQ(manifold.GetType(), Manifold::e_faceA);
	ASSERT_EQ(manifold.GetPointCount(), 2);

	const auto indexA = PositionConstraint::BodyData::index_type{0};
	const auto indexB = PositionConstraint::BodyData::index_type{0};
	const auto lcA = Vec2{};
	const auto lcB = Vec2{};
	const auto bA = PositionConstraint::BodyData{indexA, float_t(1), float_t(1), lcA};
	const auto bB = PositionConstraint::BodyData{indexB, float_t(1), float_t(1), lcB};
	const auto pc = PositionConstraint{manifold, bA, GetVertexRadius(shape), bB, GetVertexRadius(shape)};

	const auto old_pA = Position{Vec2{0, 0}, 0_deg};
	const auto old_pB = Position{Vec2{0, 0}, 0_deg};

	const auto conf = ConstraintSolverConf{};
	const auto solution = SolvePositionConstraint(pc, old_pA, true, old_pB, true, conf);
	
	EXPECT_LT(solution.min_separation, -conf.linearSlop);
	
	// object a moves left only
	EXPECT_LT(solution.pos_a.c.x, old_pA.c.x);
	EXPECT_EQ(solution.pos_a.c.y, old_pA.c.y);
	EXPECT_EQ(solution.pos_a.a, old_pA.a);
	
	// object b moves right only.
	EXPECT_GT(solution.pos_b.c.x, old_pB.c.x);
	EXPECT_EQ(solution.pos_b.c.y, old_pB.c.y);
	EXPECT_EQ(solution.pos_b.a, old_pB.a);
}
