//
//  ContactSolver.cpp
//  Box2D
//
//  Created by Louis D. Langholtz on 9/30/16.
//
//

#include "gtest/gtest.h"
#include <Box2D/Dynamics/Contacts/ContactSolver.h>
#include <Box2D/Dynamics/Contacts/ContactPositionConstraint.hpp>
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
	auto shape = PolygonShape{};
	shape.SetAsBox(dim, dim);
	const auto xfmA = Transformation(pA.c, Rot{pA.a});
	const auto xfmB = Transformation(pB.c, Rot{pB.a});
	const auto manifold = CollideShapes(shape, xfmA, shape, xfmB);
	ASSERT_EQ(manifold.GetType(), Manifold::e_faceA);
	ASSERT_EQ(manifold.GetPointCount(), 2);

	const auto indexA = ContactPositionConstraint::BodyData::index_type{0};
	const auto indexB = ContactPositionConstraint::BodyData::index_type{0};
	const auto lcA = Vec2{0, 0};
	const auto lcB = Vec2{0, 0};
	const auto bA = ContactPositionConstraint::BodyData{indexA, float_t(1), float_t(1), lcA};
	const auto bB = ContactPositionConstraint::BodyData{indexB, float_t(1), float_t(1), lcB};
	ContactPositionConstraint pc{manifold, bA, 0, bB, 0};
	
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
	auto shape = PolygonShape{};
	shape.SetAsBox(dim, dim);
	const auto xfmA = Transformation(Vec2_zero, Rot{0});
	const auto xfmB = Transformation(Vec2_zero, Rot{0});
	const auto manifold = CollideShapes(shape, xfmA, shape, xfmB);
	ASSERT_EQ(manifold.GetType(), Manifold::e_faceA);
	ASSERT_EQ(manifold.GetPointCount(), 2);
	
	const auto indexA = ContactPositionConstraint::BodyData::index_type{0};
	const auto indexB = ContactPositionConstraint::BodyData::index_type{0};
	const auto lcA = Vec2{};
	const auto lcB = Vec2{};
	const auto bA = ContactPositionConstraint::BodyData{indexA, float_t(1), float_t(1), lcA};
	const auto bB = ContactPositionConstraint::BodyData{indexB, float_t(1), float_t(1), lcB};
	ContactPositionConstraint pc{manifold, bA, 0, bB, 0};
	
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

TEST(ContactSolver, SolveOverlapping)
{
	auto pA = Position{Vec2{0, 0}, 0};
	auto pB = Position{Vec2{0, 0}, 0};
	
	const auto dim = float_t(2);
	auto shape = PolygonShape{};
	shape.SetAsBox(dim, dim);
	const auto xfmA = Transformation(Vec2_zero, Rot{0});
	const auto xfmB = Transformation(Vec2_zero, Rot{0});
	const auto manifold = CollideShapes(shape, xfmA, shape, xfmB);
	ASSERT_EQ(manifold.GetType(), Manifold::e_faceA);
	ASSERT_EQ(manifold.GetPointCount(), 2);

	const auto indexA = ContactPositionConstraint::BodyData::index_type{0};
	const auto indexB = ContactPositionConstraint::BodyData::index_type{0};
	const auto lcA = Vec2{};
	const auto lcB = Vec2{};
	const auto bA = ContactPositionConstraint::BodyData{indexA, float_t(1), float_t(1), lcA};
	const auto bB = ContactPositionConstraint::BodyData{indexB, float_t(1), float_t(1), lcB};
	ContactPositionConstraint pc{manifold, bA, 0, bB, 0};
	
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