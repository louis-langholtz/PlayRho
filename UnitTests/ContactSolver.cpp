/*
 * Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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
#include <PlayRho/Dynamics/Contacts/ContactSolver.hpp>
#include <PlayRho/Dynamics/Contacts/PositionConstraint.hpp>
#include <PlayRho/Dynamics/Contacts/VelocityConstraint.hpp>
#include <PlayRho/Dynamics/Contacts/BodyConstraint.hpp>
#include <PlayRho/Collision/Shapes/PolygonShape.hpp>
#include <PlayRho/Collision/Manifold.hpp>

using namespace playrho;

static constexpr auto Baumgarte = Real{2} / Real{10};

TEST(ContactSolver, SolvePosConstraintsForHorTouchingDoesntMove)
{
    const auto old_pA = Position{Vec2{-2, 0} * (Real(1) * Meter), Angle{0}};
    const auto old_pB = Position{Vec2{+2, 0} * (Real(1) * Meter), Angle{0}};
    const auto old_vA = Velocity{LinearVelocity2D{0, 0}, Angle{0} / Second};
    const auto old_vB = Velocity{LinearVelocity2D{0, 0}, Angle{0} / Second};

    const auto dim = Real(2) * Meter;
    const auto shape = PolygonShape(dim, dim);
    const auto xfmA = Transformation{old_pA.linear, UnitVec2{old_pA.angular}};
    const auto xfmB = Transformation{old_pB.linear, UnitVec2{old_pB.angular}};
    const auto manifold = CollideShapes(shape.GetChild(0), xfmA, shape.GetChild(0), xfmB);
    ASSERT_EQ(manifold.GetType(), Manifold::e_faceA);
    ASSERT_EQ(manifold.GetPointCount(), 2);

    const auto lcA = Length2D(0, 0);
    const auto lcB = Length2D(0, 0);
    auto bA = BodyConstraint{
        Real(1) / Kilogram,
        InvRotInertia{Real{1} * SquareRadian / (SquareMeter * Kilogram)},
        lcA, old_pA, old_vA}
    ;
    auto bB = BodyConstraint{
        Real(1) / Kilogram,
        InvRotInertia{Real{1} * SquareRadian / (SquareMeter * Kilogram)},
        lcB, old_pB, old_vB
    };
    const auto pc = PositionConstraint{manifold, bA, Real{0} * Meter, bB, Real{0} * Meter};
    
    const auto conf = ConstraintSolverConf{};
    const auto solution = GaussSeidel::SolvePositionConstraint(pc, true, true, conf);
    
    EXPECT_EQ(solution.min_separation, Length{0});
    
    EXPECT_EQ(old_pA.linear.x, solution.pos_a.linear.x);
    EXPECT_EQ(old_pA.linear.y, solution.pos_a.linear.y);
    EXPECT_EQ(old_pA.angular, solution.pos_a.angular);

    EXPECT_EQ(old_pB.linear.x, solution.pos_b.linear.x);
    EXPECT_EQ(old_pB.linear.y, solution.pos_b.linear.y);
    EXPECT_EQ(old_pB.angular, solution.pos_b.angular);
}

TEST(ContactSolver, SolvePosConstraintsForVerTouchingDoesntMove)
{
    const auto old_pA = Position{Vec2{0, -2} * (Real(1) * Meter), Angle{0}};
    const auto old_pB = Position{Vec2{0, +2} * (Real(1) * Meter), Angle{0}};
    const auto old_vA = Velocity{};
    const auto old_vB = Velocity{};
    
    const auto dim = Real(2) * Meter;
    const auto shape = PolygonShape(dim, dim);
    const auto xfmA = Transformation{old_pA.linear, UnitVec2{old_pA.angular}};
    const auto xfmB = Transformation{old_pB.linear, UnitVec2{old_pB.angular}};
    const auto manifold = CollideShapes(shape.GetChild(0), xfmA, shape.GetChild(0), xfmB);
    ASSERT_EQ(manifold.GetType(), Manifold::e_faceA);
    ASSERT_EQ(manifold.GetPointCount(), 2);
    
    const auto lcA = Length2D(0, 0);
    const auto lcB = Length2D(0, 0);
    auto bA = BodyConstraint{
        Real(1) / Kilogram,
        InvRotInertia{Real{1} * SquareRadian / (SquareMeter * Kilogram)},
        lcA, old_pA, old_vA
    };
    auto bB = BodyConstraint{
        Real(1) / Kilogram,
        InvRotInertia{Real{1} * SquareRadian / (SquareMeter * Kilogram)},
        lcB, old_pB, old_vB
    };
    const auto pc = PositionConstraint{manifold, bA, 0, bB, 0};
    
    const auto conf = ConstraintSolverConf{};
    const auto solution = GaussSeidel::SolvePositionConstraint(pc, true, true, conf);
    
    EXPECT_EQ(solution.min_separation, Length{0});
    
    EXPECT_EQ(old_pA.linear.x, solution.pos_a.linear.x);
    EXPECT_EQ(old_pA.linear.y, solution.pos_a.linear.y);
    EXPECT_EQ(old_pA.angular, solution.pos_a.angular);
    
    EXPECT_EQ(old_pB.linear.x, solution.pos_b.linear.x);
    EXPECT_EQ(old_pB.linear.y, solution.pos_b.linear.y);
    EXPECT_EQ(old_pB.angular, solution.pos_b.angular);
}

TEST(ContactSolver, SolvePosConstraintsForOverlappingZeroRateDoesntMove)
{
    const auto dim = Real(2) * Meter;
    const auto shape = PolygonShape(dim, dim);
    const auto xfmA = Transformation{Length2D(0, 0), UnitVec2{Angle{0}}};
    const auto xfmB = Transformation{Length2D(0, 0), UnitVec2{Angle{0}}};
    const auto manifold = CollideShapes(shape.GetChild(0), xfmA, shape.GetChild(0), xfmB);
    ASSERT_EQ(manifold.GetType(), Manifold::e_faceA);
    ASSERT_EQ(manifold.GetPointCount(), 2);
    
    const auto lcA = Vec2{} * (Real(1) * Meter);
    const auto lcB = Vec2{} * (Real(1) * Meter);
    const auto old_pA = Position{Length2D(0, 0), Angle{0}};
    const auto old_pB = Position{Length2D(0, 0), Angle{0}};
    const auto old_vA = Velocity{};
    const auto old_vB = Velocity{};
    auto bA = BodyConstraint{
        Real(1) / Kilogram,
        InvRotInertia{Real{1} * SquareRadian / (SquareMeter * Kilogram)},
        lcA, old_pA, old_vA
    };
    auto bB = BodyConstraint{
        Real(1) / Kilogram,
        InvRotInertia{Real{1} * SquareRadian / (SquareMeter * Kilogram)},
        lcB, old_pB, old_vB
    };
    const auto pc = PositionConstraint{manifold, bA, 0, bB, 0};

    const auto maxLinearCorrection = std::numeric_limits<Real>::infinity() * Meter;
    const auto conf = ConstraintSolverConf{}.UseResolutionRate(0).UseMaxLinearCorrection(maxLinearCorrection);
    const auto solution = GaussSeidel::SolvePositionConstraint(pc, true, true, conf);

    EXPECT_EQ(solution.min_separation, Real{-2} * dim);
    
    EXPECT_EQ(old_pA.linear.x, solution.pos_a.linear.x);
    EXPECT_EQ(old_pA.linear.y, solution.pos_a.linear.y);
    EXPECT_EQ(old_pA.angular, solution.pos_a.angular);
    
    EXPECT_EQ(old_pB.linear.x, solution.pos_b.linear.x);
    EXPECT_EQ(old_pB.linear.y, solution.pos_b.linear.y);
    EXPECT_EQ(old_pB.angular, solution.pos_b.angular);
}

TEST(ContactSolver, SolvePosConstraintsForHorOverlappingMovesHorOnly1)
{
    const auto ctr_x = Real(100);
    
    // square A is left of square B
    const auto old_pA = Position{Vec2{ctr_x - 1, 0} * (Real(1) * Meter), Angle{0}};
    const auto old_pB = Position{Vec2{ctr_x + 1, 0} * (Real(1) * Meter), Angle{0}};

    const auto old_vA = Velocity{};
    const auto old_vB = Velocity{};
    
    const auto dim = Real(2) * Meter;
    const auto shape = PolygonShape(dim, dim);
    const auto xfmA = Transformation{old_pA.linear, UnitVec2{old_pA.angular}};
    const auto xfmB = Transformation{old_pB.linear, UnitVec2{old_pB.angular}};
    const auto manifold = CollideShapes(shape.GetChild(0), xfmA, shape.GetChild(0), xfmB);
    ASSERT_EQ(manifold.GetType(), Manifold::e_faceA);
    ASSERT_EQ(GetVec2(manifold.GetLocalNormal()), Vec2(+1, 0));
    ASSERT_EQ(manifold.GetLocalPoint(), Vec2(+2, 0) * (Real(1) * Meter));
    ASSERT_EQ(manifold.GetPointCount(), 2);
    ASSERT_EQ(manifold.GetPoint(0).localPoint, Vec2(-2, +2) * (Real(1) * Meter));
    ASSERT_EQ(manifold.GetPoint(1).localPoint, Vec2(-2, -2) * (Real(1) * Meter));
    
    const auto lcA = Length2D(0, 0);
    const auto lcB = Length2D(0, 0);
    auto bA = BodyConstraint{
        Real(1) / Kilogram,
        InvRotInertia{Real{1} * SquareRadian / (SquareMeter * Kilogram)},
        lcA, old_pA, old_vA
    };
    auto bB = BodyConstraint{
        Real(1) / Kilogram,
        InvRotInertia{Real{1} * SquareRadian / (SquareMeter * Kilogram)},
        lcB, old_pB, old_vB
    };
    const auto pc = PositionConstraint{manifold, bA, 0, bB, 0};
    
    const auto maxLinearCorrection = std::numeric_limits<Real>::infinity() * Meter;
    const auto conf = ConstraintSolverConf{}.UseResolutionRate(Baumgarte).UseMaxLinearCorrection(maxLinearCorrection);
    const auto solution = GaussSeidel::SolvePositionConstraint(pc, true, true, conf);
    
    EXPECT_TRUE(almost_equal(solution.min_separation / Meter, Real(-2))); // -2.002398
        
    // object a just moves left
    EXPECT_LT(solution.pos_a.linear.x, old_pA.linear.x);
    EXPECT_EQ(solution.pos_a.linear.y, old_pA.linear.y);
    EXPECT_EQ(solution.pos_a.angular, old_pA.angular);
    
    // object b just moves right
    EXPECT_GT(solution.pos_b.linear.x, old_pB.linear.x);
    EXPECT_EQ(solution.pos_b.linear.y, old_pB.linear.y);
    EXPECT_EQ(solution.pos_b.angular, old_pB.angular);
}

TEST(ContactSolver, SolvePosConstraintsForHorOverlappingMovesHorOnly2)
{
    const auto ctr_x = Real(100);
    
    // square A is right of square B
    const auto old_pA = Position{Vec2{ctr_x + 1, 0} * (Real(1) * Meter), Angle{0}};
    const auto old_pB = Position{Vec2{ctr_x - 1, 0} * (Real(1) * Meter), Angle{0}};
    const auto old_vA = Velocity{};
    const auto old_vB = Velocity{};

    const auto dim = Real(2) * Meter;
    const auto shape = PolygonShape(dim, dim);
    const auto xfmA = Transformation{old_pA.linear, UnitVec2{old_pA.angular}};
    const auto xfmB = Transformation{old_pB.linear, UnitVec2{old_pB.angular}};
    const auto manifold = CollideShapes(shape.GetChild(0), xfmA, shape.GetChild(0), xfmB);
    ASSERT_EQ(manifold.GetType(), Manifold::e_faceA);
    ASSERT_EQ(GetVec2(manifold.GetLocalNormal()), Vec2(-1, 0));
    ASSERT_EQ(manifold.GetLocalPoint(), Vec2(-2, 0) * (Real(1) * Meter));
    ASSERT_EQ(manifold.GetPointCount(), 2);
    ASSERT_EQ(manifold.GetPoint(0).localPoint, Vec2(+2, -2) * (Real(1) * Meter));
    ASSERT_EQ(manifold.GetPoint(1).localPoint, Vec2(+2, +2) * (Real(1) * Meter));
    
    const auto lcA = Length2D(0, 0);
    const auto lcB = Length2D(0, 0);
    auto bA = BodyConstraint{
        Real(1) / Kilogram,
        InvRotInertia{Real{1} * SquareRadian / (SquareMeter * Kilogram)},
        lcA, old_pA, old_vA
    };
    auto bB = BodyConstraint{
        Real(1) / Kilogram,
        InvRotInertia{Real{1} * SquareRadian / (SquareMeter * Kilogram)},
        lcB, old_pB, old_vB
    };
    const auto pc = PositionConstraint{manifold, bA, 0, bB, 0};
    
    const auto maxLinearCorrection = std::numeric_limits<Real>::infinity() * Meter;
    const auto conf = ConstraintSolverConf{}.UseResolutionRate(Baumgarte).UseMaxLinearCorrection(maxLinearCorrection);
    const auto solution = GaussSeidel::SolvePositionConstraint(pc, true, true, conf);
    
    EXPECT_TRUE(almost_equal(solution.min_separation / Meter, Real(-2))); // -2.002398
    
    // square A just moves right
    EXPECT_GT(solution.pos_a.linear.x, old_pA.linear.x);
    EXPECT_EQ(solution.pos_a.linear.y, old_pA.linear.y);
    EXPECT_EQ(solution.pos_a.angular, old_pA.angular);
    
    // square B just moves left
    EXPECT_LT(solution.pos_b.linear.x, old_pB.linear.x);
    EXPECT_EQ(solution.pos_b.linear.y, old_pB.linear.y);
    EXPECT_EQ(solution.pos_b.angular, old_pB.angular);
}

TEST(ContactSolver, SolvePosConstraintsForVerOverlappingMovesVerOnly1)
{
    const auto ctr_y = Real(100);
    
    // square A is below square B
    const auto old_pA = Position{Vec2{0, ctr_y - 1} * (Real(1) * Meter), Angle{0}};
    const auto old_pB = Position{Vec2{0, ctr_y + 1} * (Real(1) * Meter), Angle{0}};
    const auto old_vA = Velocity{};
    const auto old_vB = Velocity{};

    const auto dim = Real(2) * Meter;
    const auto shape = PolygonShape(dim, dim);
    const auto xfmA = Transformation{old_pA.linear, UnitVec2{old_pA.angular}};
    const auto xfmB = Transformation{old_pB.linear, UnitVec2{old_pB.angular}};
    const auto manifold = CollideShapes(shape.GetChild(0), xfmA, shape.GetChild(0), xfmB);
    ASSERT_EQ(manifold.GetType(), Manifold::e_faceA);
    ASSERT_EQ(GetVec2(manifold.GetLocalNormal()), Vec2(0, 1));
    ASSERT_EQ(manifold.GetLocalPoint(), Vec2(0, 2) * (Real(1) * Meter));
    ASSERT_EQ(manifold.GetPointCount(), 2);
    ASSERT_EQ(manifold.GetPoint(0).localPoint, Vec2(-2, -2) * (Real(1) * Meter));
    ASSERT_EQ(manifold.GetPoint(1).localPoint, Vec2(+2, -2) * (Real(1) * Meter));
    
    const auto lcA = Length2D(0, 0);
    const auto lcB = Length2D(0, 0);
    auto bA = BodyConstraint{
        Real(1) / Kilogram,
        InvRotInertia{Real{1} * SquareRadian / (SquareMeter * Kilogram)},
        lcA, old_pA, old_vA
    };
    auto bB = BodyConstraint{
        Real(1) / Kilogram,
        InvRotInertia{Real{1} * SquareRadian / (SquareMeter * Kilogram)},
        lcB, old_pB, old_vB
    };
    const auto pc = PositionConstraint{manifold, bA, 0, bB, 0};
    
    const auto maxLinearCorrection = std::numeric_limits<Real>::infinity() * Meter;
    const auto conf = ConstraintSolverConf{}.UseResolutionRate(Baumgarte).UseMaxLinearCorrection(maxLinearCorrection);
    const auto solution = GaussSeidel::SolvePositionConstraint(pc, true, true, conf);
    
    EXPECT_TRUE(almost_equal(solution.min_separation / Meter, Real(-2))); // -2.002398
    
    // object a just moves down only
    EXPECT_EQ(solution.pos_a.linear.x, old_pA.linear.x);
    EXPECT_LT(solution.pos_a.linear.y, old_pA.linear.y);
    EXPECT_EQ(solution.pos_a.angular, old_pA.angular);
    
    {
        // confirm object a moves more in x direction than in y direction.
        const auto mov_a = solution.pos_a - old_pA;    
        EXPECT_LT(Abs(mov_a.linear.x), Abs(mov_a.linear.y));
    }
    
    // object b just moves up only
    EXPECT_EQ(solution.pos_b.linear.x, old_pB.linear.x);
    EXPECT_GT(solution.pos_b.linear.y, old_pB.linear.y);
    EXPECT_EQ(solution.pos_b.angular, old_pB.angular);
    
    {
        // confirm object a moves more in x direction than in y direction.
        const auto mov_b = solution.pos_b - old_pB;
        EXPECT_LT(Abs(mov_b.linear.x), Abs(mov_b.linear.y));
    }
}

TEST(ContactSolver, SolvePosConstraintsForVerOverlappingMovesVerOnly2)
{
    const auto ctr_y = Real(100);

    // square A is above square B
    const auto old_pA = Position{Vec2{0, ctr_y + 1} * (Real(1) * Meter), Angle{0}};
    const auto old_pB = Position{Vec2{0, ctr_y - 1} * (Real(1) * Meter), Angle{0}};
    const auto old_vA = Velocity{};
    const auto old_vB = Velocity{};

    const auto dim = Real(2) * Meter;
    const auto shape = PolygonShape(dim, dim);
    const auto xfmA = Transformation{old_pA.linear, UnitVec2{old_pA.angular}};
    const auto xfmB = Transformation{old_pB.linear, UnitVec2{old_pB.angular}};
    const auto manifold = CollideShapes(shape.GetChild(0), xfmA, shape.GetChild(0), xfmB);
    
    ASSERT_EQ(manifold.GetType(), Manifold::e_faceA);
    ASSERT_EQ(GetVec2(manifold.GetLocalNormal()), Vec2(0, -1));
    ASSERT_EQ(manifold.GetLocalPoint(), Vec2(0, -2) * (Real(1) * Meter));
    ASSERT_EQ(manifold.GetPointCount(), 2);
    ASSERT_EQ(manifold.GetPoint(0).localPoint, Vec2(+2, +2) * (Real(1) * Meter));
    ASSERT_EQ(manifold.GetPoint(1).localPoint, Vec2(-2, +2) * (Real(1) * Meter));
    
    const auto lcA = Length2D(0, 0);
    const auto lcB = Length2D(0, 0);
    auto bA = BodyConstraint{
        Real(1) / Kilogram,
        InvRotInertia{Real{1} * SquareRadian / (SquareMeter * Kilogram)},
        lcA, old_pA, old_vA
    };
    auto bB = BodyConstraint{
        Real(1) / Kilogram,
        InvRotInertia{Real{1} * SquareRadian / (SquareMeter * Kilogram)},
        lcB, old_pB, old_vB
    };
    const auto pc = PositionConstraint{manifold, bA, 0, bB, 0};

    const auto maxLinearCorrection = std::numeric_limits<Real>::infinity() * Meter;
    const auto conf = ConstraintSolverConf{}.UseResolutionRate(Baumgarte).UseMaxLinearCorrection(maxLinearCorrection);
    const auto solution = GaussSeidel::SolvePositionConstraint(pc, true, true, conf);
    
    EXPECT_TRUE(almost_equal(solution.min_separation / Meter, Real(-2))); // -2.002398
    
    // square A just moves up only
    EXPECT_EQ(solution.pos_a.linear.x, old_pA.linear.x);
    EXPECT_GT(solution.pos_a.linear.y, old_pA.linear.y);
    EXPECT_EQ(solution.pos_a.angular, old_pA.angular);
    
    {
        // confirm object a moves more in x direction than in y direction.
        const auto mov_a = solution.pos_a - old_pA;    
        EXPECT_LT(Abs(mov_a.linear.x), Abs(mov_a.linear.y));
    }
    
    // square B just moves down only
    EXPECT_EQ(solution.pos_b.linear.x, old_pB.linear.x);
    EXPECT_LT(solution.pos_b.linear.y, old_pB.linear.y);
    EXPECT_EQ(solution.pos_b.angular, old_pB.angular);
    
    {
        // confirm object a moves more in x direction than in y direction.
        const auto mov_b = solution.pos_b - old_pB;
        EXPECT_LT(Abs(mov_b.linear.x), Abs(mov_b.linear.y));
    }
}

TEST(ContactSolver, SolvePosConstraintsForPerfectlyOverlappingSquares)
{
    const auto dim = Real(2) * Meter;
    const auto shape = PolygonShape(dim, dim);
    const auto xfmA = Transformation{Length2D(0, 0), UnitVec2{Angle{0}}};
    const auto xfmB = Transformation{Length2D(0, 0), UnitVec2{Angle{0}}};
    const auto manifold = CollideShapes(shape.GetChild(0), xfmA, shape.GetChild(0), xfmB);
    ASSERT_EQ(manifold.GetType(), Manifold::e_faceA);
    ASSERT_EQ(manifold.GetPointCount(), 2);

    const auto old_pA = Position{Length2D(0, 0), Angle{0}};
    const auto old_pB = Position{Length2D(0, 0), Angle{0}};
    const auto old_vA = Velocity{};
    const auto old_vB = Velocity{};

    const auto lcA = Vec2{} * (Real(1) * Meter);
    const auto lcB = Vec2{} * (Real(1) * Meter);
    auto bA = BodyConstraint{
        Real(1) / Kilogram,
        InvRotInertia{Real{1} * SquareRadian / (SquareMeter * Kilogram)},
        lcA, old_pA, old_vA
    };
    auto bB = BodyConstraint{
        Real(1) / Kilogram,
        InvRotInertia{Real{1} * SquareRadian / (SquareMeter * Kilogram)},
        lcB, old_pB, old_vB
    };
    const auto pc = PositionConstraint{manifold, bA, GetVertexRadius(shape), bB, GetVertexRadius(shape)};
    
    const auto conf = ConstraintSolverConf{};
    const auto solution = GaussSeidel::SolvePositionConstraint(pc, true, true, conf);
    
    EXPECT_LT(solution.min_separation, -conf.linearSlop);
    
    // object a moves left only
    EXPECT_LT(solution.pos_a.linear.x, old_pA.linear.x);
    EXPECT_EQ(solution.pos_a.linear.y, old_pA.linear.y);
    EXPECT_EQ(solution.pos_a.angular, old_pA.angular);
    
    // object b moves right only.
    EXPECT_GT(solution.pos_b.linear.x, old_pB.linear.x);
    EXPECT_EQ(solution.pos_b.linear.y, old_pB.linear.y);
    EXPECT_EQ(solution.pos_b.angular, old_pB.angular);
}

#if 0
TEST(ContactSolver, SolveVelocityConstraint1)
{
    const auto inverse_mass_a = Real(0);
    const auto inverse_mass_b = Real(0);
    const auto inverse_mass = inverse_mass_a + inverse_mass_b;

    const auto linear_velocity = Vec2{1, 1};
    const auto angular_velocity = Angle{0};

    const auto old_pA = Position{Vec2{0, 0}, Angle{0}};
    const auto old_pB = Position{Vec2{0, 0}, Angle{0}};
    const auto vel_a = Velocity{linear_velocity, angular_velocity};
    const auto vel_b = Velocity{linear_velocity, angular_velocity};
    const auto lcA = Vec2{};
    const auto lcB = Vec2{};

    auto body_data_a = BodyConstraint{0, inverse_mass_a, 0, lcA, old_pA, vel_a};
    auto body_data_b = BodyConstraint{1, inverse_mass_b, 0, lcB, old_pB, vel_b};
    const auto normal = UnitVec2::GetTop();
    const auto friction = Real(1);
    const auto restitution = Real(0.5f);
    const auto tangent_speed = Real(0);
    const auto contact_index = VelocityConstraint::index_type{0};
    auto vc = VelocityConstraint{contact_index, friction, restitution, tangent_speed, body_data_a, body_data_b, normal};
    const auto r_a = Vec2{0, 0};
    const auto r_b = Vec2{0, 0};
    vc.AddPoint(0, 0, r_a, r_b, VelocityConstraint::Conf{});
    ASSERT_EQ(vc.GetPointCount(), VelocityConstraint::size_type{1});

    SolveVelocityConstraint(vc);
    
    EXPECT_EQ(vel_a.linear, linear_velocity);
    EXPECT_EQ(vel_a.angular, angular_velocity);
    EXPECT_EQ(vel_b.linear, linear_velocity);
    EXPECT_EQ(vel_b.angular, angular_velocity);
    
    EXPECT_FALSE(IsValid(vc.GetK()));
    EXPECT_FALSE(IsValid(vc.GetNormalMass()));

    EXPECT_EQ(vc.GetNormal(), normal);
    EXPECT_EQ(vc.GetFriction(), friction);
    EXPECT_EQ(vc.GetRestitution(), restitution);
    EXPECT_EQ(vc.GetTangentSpeed(), tangent_speed);
    EXPECT_EQ(vc.GetContactIndex(), contact_index);
    EXPECT_EQ(vc.GetInvMass(), inverse_mass);
    
    EXPECT_EQ(vc.GetPointCount(), VelocityConstraint::size_type{1});
    
    EXPECT_EQ(vc.GetNormalImpulseAtPoint(0), Real(0));
    EXPECT_EQ(vc.GetTangentImpulseAtPoint(0), Real(0));
    EXPECT_EQ(vc.GetNormalMassAtPoint(0), Real(0));
    EXPECT_EQ(vc.GetTangentMassAtPoint(0), Real(0));
    EXPECT_EQ(vc.GetVelocityBiasAtPoint(0), Real(0));
    EXPECT_EQ(vc.GetPointRelPosA(0), r_a);
    EXPECT_EQ(vc.GetPointRelPosB(0), r_b);
    
    EXPECT_FALSE(IsValid(vc.GetNormalImpulseAtPoint(1)));
    EXPECT_FALSE(IsValid(vc.GetTangentImpulseAtPoint(1)));
    EXPECT_FALSE(IsValid(vc.GetNormalMassAtPoint(1)));
    EXPECT_FALSE(IsValid(vc.GetTangentMassAtPoint(1)));
    EXPECT_FALSE(IsValid(vc.GetVelocityBiasAtPoint(1)));
    EXPECT_FALSE(IsValid(vc.GetPointRelPosA(1)));
    EXPECT_FALSE(IsValid(vc.GetPointRelPosB(1)));
}
#endif

#if 0
TEST(ContactSolver, SolveVelocityConstraint2)
{
    const auto linear_velocity = Vec2{1, 1};
    const auto angular_velocity = Angle{0};
    
    const auto old_pA = Position{Vec2{0, 0}, Angle{0}};
    const auto old_pB = Position{Vec2{0, 0}, Angle{0}};
    
    auto vel_a = Velocity{linear_velocity, angular_velocity};
    auto vel_b = Velocity{linear_velocity, angular_velocity};
    
    const auto lcA = Vec2{};
    const auto lcB = Vec2{};

    const auto inverse_mass_a = Real(1);
    const auto inverse_mass_b = Real(1);
    const auto inverse_mass = inverse_mass_a + inverse_mass_b;
    auto body_data_a = BodyConstraint{0, inverse_mass_a, 0, lcA, old_pA, vel_a};
    auto body_data_b = BodyConstraint{1, inverse_mass_b, 0, lcB, old_pB, vel_b};
    const auto normal = UnitVec2::GetTop();
    const auto friction = Real(1);
    const auto restitution = Real(0.5f);
    const auto tangent_speed = Real(0);
    const auto contact_index = VelocityConstraint::index_type{0};
    auto vc = VelocityConstraint{contact_index, friction, restitution, tangent_speed, body_data_a, body_data_b, normal};
    const auto r_a = Vec2{0, 0};
    const auto r_b = Vec2{0, 0};
    vc.AddPoint(0, 0, r_a, r_b, VelocityConstraint::Conf{});
    ASSERT_EQ(vc.GetPointCount(), VelocityConstraint::size_type{1});

    SolveVelocityConstraint(vc);
    
    EXPECT_EQ(vel_a.linear, linear_velocity);
    EXPECT_EQ(vel_a.angular, angular_velocity);
    EXPECT_EQ(vel_b.linear, linear_velocity);
    EXPECT_EQ(vel_b.angular, angular_velocity);
    
    EXPECT_FALSE(IsValid(vc.GetK()));
    EXPECT_FALSE(IsValid(vc.GetNormalMass()));
    
    EXPECT_EQ(vc.GetNormal(), normal);
    EXPECT_EQ(vc.GetFriction(), friction);
    EXPECT_EQ(vc.GetRestitution(), restitution);
    EXPECT_EQ(vc.GetTangentSpeed(), tangent_speed);
    EXPECT_EQ(vc.GetContactIndex(), contact_index);
    EXPECT_EQ(vc.GetInvMass(), inverse_mass);
    
    EXPECT_EQ(vc.GetPointCount(), VelocityConstraint::size_type{1});
    
    EXPECT_EQ(vc.GetNormalImpulseAtPoint(0), Real(0));
    EXPECT_EQ(vc.GetTangentImpulseAtPoint(0), Real(0));
    EXPECT_EQ(vc.GetNormalMassAtPoint(0), Real(0.5f));
    EXPECT_EQ(vc.GetTangentMassAtPoint(0), Real(0.5f));
    EXPECT_EQ(vc.GetVelocityBiasAtPoint(0), Real(0));
    EXPECT_EQ(vc.GetPointRelPosA(0), r_a);
    EXPECT_EQ(vc.GetPointRelPosB(0), r_b);
    
    EXPECT_FALSE(IsValid(vc.GetNormalImpulseAtPoint(1)));
    EXPECT_FALSE(IsValid(vc.GetTangentImpulseAtPoint(1)));
    EXPECT_FALSE(IsValid(vc.GetNormalMassAtPoint(1)));
    EXPECT_FALSE(IsValid(vc.GetTangentMassAtPoint(1)));
    EXPECT_FALSE(IsValid(vc.GetVelocityBiasAtPoint(1)));
    EXPECT_FALSE(IsValid(vc.GetPointRelPosA(1)));
    EXPECT_FALSE(IsValid(vc.GetPointRelPosB(1)));
}
#endif
