/*
 * Copyright (c) 2023 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#include <playrho/d2/ContactSolver.hpp>
#include <playrho/ConstraintSolverConf.hpp>
#include <playrho/d2/PositionConstraint.hpp>
#include <playrho/d2/VelocityConstraint.hpp>
#include <playrho/d2/BodyConstraint.hpp>
#include <playrho/d2/PolygonShapeConf.hpp>
#include <playrho/d2/Manifold.hpp>

using namespace playrho;
using namespace playrho::d2;

static constexpr auto Baumgarte = Real{2} / Real{10};

TEST(ContactSolver, SolvePosConstraintsForHorTouchingDoesntMove)
{
    const auto old_pA = Position{Vec2{-2, 0} * Meter, 0_deg};
    const auto old_pB = Position{Vec2{+2, 0} * Meter, 0_deg};
    const auto old_vA = Velocity{LinearVelocity2{}, 0_deg / 1_s};
    const auto old_vB = Velocity{LinearVelocity2{}, 0_deg / 1_s};

    const auto dim = 2_m;
    const auto shape = PolygonShapeConf(dim, dim);
    const auto xfmA = Transformation{old_pA.linear, UnitVec::Get(old_pA.angular)};
    const auto xfmB = Transformation{old_pB.linear, UnitVec::Get(old_pB.angular)};
    const auto manifold = CollideShapes(GetChild(shape, 0), xfmA, GetChild(shape, 0), xfmB);
    ASSERT_EQ(manifold.GetType(), Manifold::e_faceA);
    ASSERT_EQ(manifold.GetPointCount(), 2);

    const auto lcA = Length2{};
    const auto lcB = Length2{};
    auto bodies = std::vector<BodyConstraint>{
        BodyConstraint{
            Real(1) / 1_kg,
            InvRotInertia{Real{1} * SquareRadian / (SquareMeter * 1_kg)},
            lcA, old_pA, old_vA},
        BodyConstraint{
            Real(1) / 1_kg,
            InvRotInertia{Real{1} * SquareRadian / (SquareMeter * 1_kg)},
            lcB, old_pB, old_vB
        }
    };
    const auto pc = PositionConstraint{manifold, BodyID(0u), BodyID(1u), 0_m};
    
    const auto conf = ConstraintSolverConf{};
    const auto solution = GaussSeidel::SolvePositionConstraint(pc, true, true, bodies, conf);
    
    EXPECT_EQ(solution.min_separation, 0_m);
    
    EXPECT_EQ(GetX(old_pA.linear), GetX(solution.pos_a.linear));
    EXPECT_EQ(GetY(old_pA.linear), GetY(solution.pos_a.linear));
    EXPECT_EQ(old_pA.angular, solution.pos_a.angular);

    EXPECT_EQ(GetX(old_pB.linear), GetX(solution.pos_b.linear));
    EXPECT_EQ(GetY(old_pB.linear), GetY(solution.pos_b.linear));
    EXPECT_EQ(old_pB.angular, solution.pos_b.angular);
}

TEST(ContactSolver, SolvePosConstraintsForVerTouchingDoesntMove)
{
    const auto old_pA = Position{Vec2{0, -2} * Meter, 0_deg};
    const auto old_pB = Position{Vec2{0, +2} * Meter, 0_deg};
    const auto old_vA = Velocity{};
    const auto old_vB = Velocity{};
    
    const auto dim = 2_m;
    const auto shape = PolygonShapeConf(dim, dim);
    const auto xfmA = Transformation{old_pA.linear, UnitVec::Get(old_pA.angular)};
    const auto xfmB = Transformation{old_pB.linear, UnitVec::Get(old_pB.angular)};
    const auto manifold = CollideShapes(GetChild(shape, 0), xfmA, GetChild(shape, 0), xfmB);
    ASSERT_EQ(manifold.GetType(), Manifold::e_faceA);
    ASSERT_EQ(manifold.GetPointCount(), 2);
    
    const auto lcA = Length2{};
    const auto lcB = Length2{};
    auto bodies = std::vector<BodyConstraint>{
        BodyConstraint{
            Real(1) / 1_kg,
            InvRotInertia{Real{1} * SquareRadian / (SquareMeter * 1_kg)},
            lcA, old_pA, old_vA
        },
        BodyConstraint{
            Real(1) / 1_kg,
            InvRotInertia{Real{1} * SquareRadian / (SquareMeter * 1_kg)},
            lcB, old_pB, old_vB
        }
    };
    const auto pc = PositionConstraint{manifold, BodyID(0u), BodyID(1u), 0_m};
    
    const auto conf = ConstraintSolverConf{};
    const auto solution = GaussSeidel::SolvePositionConstraint(pc, true, true, bodies, conf);
    
    EXPECT_EQ(solution.min_separation, 0_m);
    
    EXPECT_EQ(GetX(old_pA.linear), GetX(solution.pos_a.linear));
    EXPECT_EQ(GetY(old_pA.linear), GetY(solution.pos_a.linear));
    EXPECT_EQ(old_pA.angular, solution.pos_a.angular);
    
    EXPECT_EQ(GetX(old_pB.linear), GetX(solution.pos_b.linear));
    EXPECT_EQ(GetY(old_pB.linear), GetY(solution.pos_b.linear));
    EXPECT_EQ(old_pB.angular, solution.pos_b.angular);
}

TEST(ContactSolver, SolvePosConstraintsForOverlappingZeroRateDoesntMove)
{
    const auto dim = 2_m;
    const auto shape = PolygonShapeConf(dim, dim);
    const auto xfmA = Transformation{Length2{}, UnitVec::GetRight()};
    const auto xfmB = Transformation{Length2{}, UnitVec::GetRight()};
    const auto manifold = CollideShapes(GetChild(shape, 0), xfmA, GetChild(shape, 0), xfmB);
    ASSERT_EQ(manifold.GetType(), Manifold::e_faceA);
    ASSERT_EQ(manifold.GetPointCount(), 2);
    
    const auto lcA = Length2{};
    const auto lcB = Length2{};
    const auto old_pA = Position{Length2{}, 0_deg};
    const auto old_pB = Position{Length2{}, 0_deg};
    const auto old_vA = Velocity{};
    const auto old_vB = Velocity{};
    auto bodies = std::vector<BodyConstraint>{
        BodyConstraint{
            Real(1) / 1_kg,
            InvRotInertia{Real{1} * SquareRadian / (SquareMeter * 1_kg)},
            lcA, old_pA, old_vA
        }, BodyConstraint{
            Real(1) / 1_kg,
            InvRotInertia{Real{1} * SquareRadian / (SquareMeter * 1_kg)},
            lcB, old_pB, old_vB
        }
    };
    const auto pc = PositionConstraint{manifold, BodyID(0u), BodyID(1u), 0_m};

    const auto maxLinearCorrection = std::numeric_limits<Real>::infinity() * Meter;
    const auto conf = ConstraintSolverConf{}.UseResolutionRate(0).UseMaxLinearCorrection(maxLinearCorrection);
    const auto solution = GaussSeidel::SolvePositionConstraint(pc, true, true, bodies, conf);

    EXPECT_NEAR(static_cast<double>(Real{solution.min_separation / Meter}),
                static_cast<double>(Real{-2 * dim / Meter}),
                0.0001);
    
    EXPECT_EQ(GetX(old_pA.linear), GetX(solution.pos_a.linear));
    EXPECT_EQ(GetY(old_pA.linear), GetY(solution.pos_a.linear));
    EXPECT_EQ(old_pA.angular, solution.pos_a.angular);
    
    EXPECT_EQ(GetX(old_pB.linear), GetX(solution.pos_b.linear));
    EXPECT_EQ(GetY(old_pB.linear), GetY(solution.pos_b.linear));
    EXPECT_EQ(old_pB.angular, solution.pos_b.angular);
}

TEST(ContactSolver, SolvePosConstraintsForHorOverlappingMovesHorOnly1)
{
    const auto ctr_x = Real(100);
    
    // square A is left of square B
    const auto old_pA = Position{Vec2{ctr_x - 1, 0} * Meter, 0_deg};
    const auto old_pB = Position{Vec2{ctr_x + 1, 0} * Meter, 0_deg};

    const auto old_vA = Velocity{};
    const auto old_vB = Velocity{};
    
    const auto dim = 2_m;
    const auto shape = PolygonShapeConf(dim, dim);
    const auto xfmA = Transformation{old_pA.linear, UnitVec::Get(old_pA.angular)};
    const auto xfmB = Transformation{old_pB.linear, UnitVec::Get(old_pB.angular)};
    const auto manifold = CollideShapes(GetChild(shape, 0), xfmA, GetChild(shape, 0), xfmB);
    ASSERT_EQ(manifold.GetType(), Manifold::e_faceA);
    ASSERT_NEAR(static_cast<double>(GetX(GetVec2(manifold.GetLocalNormal()))), +1.0, 0.00001);
    ASSERT_NEAR(static_cast<double>(GetY(GetVec2(manifold.GetLocalNormal()))), +0.0, 0.00001);
    ASSERT_EQ(manifold.GetLocalPoint(), Vec2(+2, 0) * Meter);
    ASSERT_EQ(manifold.GetPointCount(), 2);
    ASSERT_EQ(manifold.GetPoint(0).localPoint, Vec2(-2, +2) * Meter);
    ASSERT_EQ(manifold.GetPoint(1).localPoint, Vec2(-2, -2) * Meter);
    
    const auto lcA = Length2{};
    const auto lcB = Length2{};
    auto bodies = std::vector<BodyConstraint>{
        BodyConstraint{
            Real(1) / 1_kg,
            InvRotInertia{Real{1} * SquareRadian / (SquareMeter * 1_kg)},
            lcA, old_pA, old_vA
        },
        BodyConstraint{
            Real(1) / 1_kg,
            InvRotInertia{Real{1} * SquareRadian / (SquareMeter * 1_kg)},
            lcB, old_pB, old_vB
        }
    };
    const auto pc = PositionConstraint{manifold, BodyID(0u), BodyID(1u), 0_m};
    
    const auto maxLinearCorrection = std::numeric_limits<Real>::infinity() * Meter;
    const auto conf = ConstraintSolverConf{}.UseResolutionRate(Baumgarte).UseMaxLinearCorrection(maxLinearCorrection);
    const auto solution = GaussSeidel::SolvePositionConstraint(pc, true, true, bodies, conf);
    
    EXPECT_TRUE(AlmostEqual(Real{solution.min_separation / Meter}, Real(-2))); // -2.002398
        
    // object a just moves left
    EXPECT_LT(GetX(solution.pos_a.linear), GetX(old_pA.linear));
    EXPECT_EQ(GetY(solution.pos_a.linear), GetY(old_pA.linear));
    EXPECT_EQ(solution.pos_a.angular, old_pA.angular);
    
    // object b just moves right
    EXPECT_GT(GetX(solution.pos_b.linear), GetX(old_pB.linear));
    EXPECT_EQ(GetY(solution.pos_b.linear), GetY(old_pB.linear));
    EXPECT_EQ(solution.pos_b.angular, old_pB.angular);
}

TEST(ContactSolver, SolvePosConstraintsForHorOverlappingMovesHorOnly2)
{
    const auto ctr_x = Real(100);
    
    // square A is right of square B
    const auto old_pA = Position{Vec2{ctr_x + 1, 0} * Meter, 0_deg};
    const auto old_pB = Position{Vec2{ctr_x - 1, 0} * Meter, 0_deg};
    const auto old_vA = Velocity{};
    const auto old_vB = Velocity{};

    const auto dim = 2_m;
    const auto shape = PolygonShapeConf(dim, dim);
    const auto xfmA = Transformation{old_pA.linear, UnitVec::Get(old_pA.angular)};
    const auto xfmB = Transformation{old_pB.linear, UnitVec::Get(old_pB.angular)};
    const auto manifold = CollideShapes(GetChild(shape, 0), xfmA, GetChild(shape, 0), xfmB);
    ASSERT_EQ(manifold.GetType(), Manifold::e_faceA);
    ASSERT_NEAR(static_cast<double>(GetX(GetVec2(manifold.GetLocalNormal()))), -1.0, 0.00001);
    ASSERT_NEAR(static_cast<double>(GetY(GetVec2(manifold.GetLocalNormal()))), +0.0, 0.00001);
    ASSERT_EQ(manifold.GetLocalPoint(), Vec2(-2, 0) * Meter);
    ASSERT_EQ(manifold.GetPointCount(), 2);
    ASSERT_EQ(manifold.GetPoint(0).localPoint, Vec2(+2, -2) * Meter);
    ASSERT_EQ(manifold.GetPoint(1).localPoint, Vec2(+2, +2) * Meter);
    
    const auto lcA = Length2{};
    const auto lcB = Length2{};
    auto bodies = std::vector<BodyConstraint>{
        BodyConstraint{
            Real(1) / 1_kg,
            InvRotInertia{Real{1} * SquareRadian / (SquareMeter * 1_kg)},
            lcA, old_pA, old_vA
        },
        BodyConstraint{
            Real(1) / 1_kg,
            InvRotInertia{Real{1} * SquareRadian / (SquareMeter * 1_kg)},
            lcB, old_pB, old_vB
        }
    };
    const auto pc = PositionConstraint{manifold, BodyID(0u), BodyID(1u), 0_m};
    
    const auto maxLinearCorrection = std::numeric_limits<Real>::infinity() * Meter;
    const auto conf = ConstraintSolverConf{}.UseResolutionRate(Baumgarte).UseMaxLinearCorrection(maxLinearCorrection);
    const auto solution = GaussSeidel::SolvePositionConstraint(pc, true, true, bodies, conf);
    
    EXPECT_TRUE(AlmostEqual(Real{solution.min_separation / Meter}, Real(-2))); // -2.002398
    
    // square A just moves right
    EXPECT_GT(GetX(solution.pos_a.linear), GetX(old_pA.linear));
    EXPECT_EQ(GetY(solution.pos_a.linear), GetY(old_pA.linear));
    EXPECT_EQ(solution.pos_a.angular, old_pA.angular);
    
    // square B just moves left
    EXPECT_LT(GetX(solution.pos_b.linear), GetX(old_pB.linear));
    EXPECT_EQ(GetY(solution.pos_b.linear), GetY(old_pB.linear));
    EXPECT_EQ(solution.pos_b.angular, old_pB.angular);
}

TEST(ContactSolver, SolvePosConstraintsForVerOverlappingMovesVerOnly1)
{
    const auto ctr_y = Real(100);
    
    // square A is below square B
    const auto old_pA = Position{Vec2{0, ctr_y - 1} * Meter, 0_deg};
    const auto old_pB = Position{Vec2{0, ctr_y + 1} * Meter, 0_deg};
    const auto old_vA = Velocity{};
    const auto old_vB = Velocity{};

    const auto dim = 2_m;
    const auto shape = PolygonShapeConf(dim, dim);
    const auto xfmA = Transformation{old_pA.linear, UnitVec::Get(old_pA.angular)};
    const auto xfmB = Transformation{old_pB.linear, UnitVec::Get(old_pB.angular)};
    const auto manifold = CollideShapes(GetChild(shape, 0), xfmA, GetChild(shape, 0), xfmB);
    ASSERT_EQ(manifold.GetType(), Manifold::e_faceA);
    ASSERT_NEAR(static_cast<double>(GetX(GetVec2(manifold.GetLocalNormal()))), +0.0, 0.00001);
    ASSERT_NEAR(static_cast<double>(GetY(GetVec2(manifold.GetLocalNormal()))), +1.0, 0.00001);
    ASSERT_EQ(manifold.GetLocalPoint(), Vec2(0, 2) * Meter);
    ASSERT_EQ(manifold.GetPointCount(), 2);
    ASSERT_EQ(manifold.GetPoint(0).localPoint, Vec2(-2, -2) * Meter);
    ASSERT_EQ(manifold.GetPoint(1).localPoint, Vec2(+2, -2) * Meter);
    
    const auto lcA = Length2{};
    const auto lcB = Length2{};
    auto bodies = std::vector<BodyConstraint>{
        BodyConstraint{
            Real(1) / 1_kg,
            InvRotInertia{Real{1} * SquareRadian / (SquareMeter * 1_kg)},
            lcA, old_pA, old_vA
        },
        BodyConstraint{
            Real(1) / 1_kg,
            InvRotInertia{Real{1} * SquareRadian / (SquareMeter * 1_kg)},
            lcB, old_pB, old_vB
        }
    };
    const auto pc = PositionConstraint{manifold, BodyID(0u), BodyID(1u), 0_m};
    
    const auto maxLinearCorrection = std::numeric_limits<Real>::infinity() * Meter;
    const auto conf = ConstraintSolverConf{}.UseResolutionRate(Baumgarte).UseMaxLinearCorrection(maxLinearCorrection);
    const auto solution = GaussSeidel::SolvePositionConstraint(pc, true, true, bodies, conf);
    
    EXPECT_TRUE(AlmostEqual(Real{solution.min_separation / Meter}, Real(-2))); // -2.002398
    
    // object a just moves down only
    EXPECT_EQ(GetX(solution.pos_a.linear), GetX(old_pA.linear));
    EXPECT_LT(GetY(solution.pos_a.linear), GetY(old_pA.linear));
    EXPECT_EQ(solution.pos_a.angular, old_pA.angular);
    
    {
        // confirm object a moves more in x direction than in y direction.
        const auto mov_a = solution.pos_a - old_pA;    
        EXPECT_LT(abs(GetX(mov_a.linear)), abs(GetY(mov_a.linear)));
    }
    
    // object b just moves up only
    EXPECT_EQ(GetX(solution.pos_b.linear), GetX(old_pB.linear));
    EXPECT_GT(GetY(solution.pos_b.linear), GetY(old_pB.linear));
    EXPECT_EQ(solution.pos_b.angular, old_pB.angular);
    
    {
        // confirm object a moves more in x direction than in y direction.
        const auto mov_b = solution.pos_b - old_pB;
        EXPECT_LT(abs(GetX(mov_b.linear)), abs(GetY(mov_b.linear)));
    }
}

TEST(ContactSolver, SolvePosConstraintsForVerOverlappingMovesVerOnly2)
{
    const auto ctr_y = Real(100);

    // square A is above square B
    const auto old_pA = Position{Vec2{0, ctr_y + 1} * Meter, 0_deg};
    const auto old_pB = Position{Vec2{0, ctr_y - 1} * Meter, 0_deg};
    const auto old_vA = Velocity{};
    const auto old_vB = Velocity{};

    const auto dim = 2_m;
    const auto shape = PolygonShapeConf(dim, dim);
    const auto xfmA = Transformation{old_pA.linear, UnitVec::Get(old_pA.angular)};
    const auto xfmB = Transformation{old_pB.linear, UnitVec::Get(old_pB.angular)};
    const auto manifold = CollideShapes(GetChild(shape, 0), xfmA, GetChild(shape, 0), xfmB);
    
    ASSERT_EQ(manifold.GetType(), Manifold::e_faceA);
    ASSERT_NEAR(static_cast<double>(GetX(GetVec2(manifold.GetLocalNormal()))), +0.0, 0.00001);
    ASSERT_NEAR(static_cast<double>(GetY(GetVec2(manifold.GetLocalNormal()))), -1.0, 0.00001);
    ASSERT_EQ(manifold.GetLocalPoint(), Vec2(0, -2) * Meter);
    ASSERT_EQ(manifold.GetPointCount(), 2);
    ASSERT_EQ(manifold.GetPoint(0).localPoint, Vec2(+2, +2) * Meter);
    ASSERT_EQ(manifold.GetPoint(1).localPoint, Vec2(-2, +2) * Meter);
    
    const auto lcA = Length2{};
    const auto lcB = Length2{};
    auto bodies = std::vector<BodyConstraint>{
        BodyConstraint{
            Real(1) / 1_kg,
            InvRotInertia{Real{1} * SquareRadian / (SquareMeter * 1_kg)},
            lcA, old_pA, old_vA
        },
        BodyConstraint{
            Real(1) / 1_kg,
            InvRotInertia{Real{1} * SquareRadian / (SquareMeter * 1_kg)},
            lcB, old_pB, old_vB
        }
    };
    const auto pc = PositionConstraint{manifold, BodyID(0u), BodyID(1u), 0_m};

    const auto maxLinearCorrection = std::numeric_limits<Real>::infinity() * Meter;
    const auto conf = ConstraintSolverConf{}.UseResolutionRate(Baumgarte).UseMaxLinearCorrection(maxLinearCorrection);
    const auto solution = GaussSeidel::SolvePositionConstraint(pc, true, true, bodies, conf);
    
    EXPECT_TRUE(AlmostEqual(Real{solution.min_separation / Meter}, Real(-2))); // -2.002398
    
    // square A just moves up only
    EXPECT_EQ(GetX(solution.pos_a.linear), GetX(old_pA.linear));
    EXPECT_GT(GetY(solution.pos_a.linear), GetY(old_pA.linear));
    EXPECT_EQ(solution.pos_a.angular, old_pA.angular);
    
    {
        // confirm object a moves more in x direction than in y direction.
        const auto mov_a = solution.pos_a - old_pA;    
        EXPECT_LT(abs(GetX(mov_a.linear)), abs(GetY(mov_a.linear)));
    }
    
    // square B just moves down only
    EXPECT_EQ(GetX(solution.pos_b.linear), GetX(old_pB.linear));
    EXPECT_LT(GetY(solution.pos_b.linear), GetY(old_pB.linear));
    EXPECT_EQ(solution.pos_b.angular, old_pB.angular);
    
    {
        // confirm object a moves more in x direction than in y direction.
        const auto mov_b = solution.pos_b - old_pB;
        EXPECT_LT(abs(GetX(mov_b.linear)), abs(GetY(mov_b.linear)));
    }
}

TEST(ContactSolver, SolvePosConstraintsForPerfectlyOverlappingSquares)
{
    const auto dim = 2_m;
    const auto shape = PolygonShapeConf(dim, dim);
    const auto xfmA = Transformation{Length2{}, UnitVec::GetRight()};
    const auto xfmB = Transformation{Length2{}, UnitVec::GetRight()};
    const auto manifold = CollideShapes(GetChild(shape, 0), xfmA, GetChild(shape, 0), xfmB);
    ASSERT_EQ(manifold.GetType(), Manifold::e_faceA);
    ASSERT_EQ(manifold.GetPointCount(), 2);

    const auto old_pA = Position{Length2{}, 0_deg};
    const auto old_pB = Position{Length2{}, 0_deg};
    const auto old_vA = Velocity{};
    const auto old_vB = Velocity{};

    const auto lcA = Length2{};
    const auto lcB = Length2{};
    auto bodies = std::vector<BodyConstraint>{
        BodyConstraint{
            Real(1) / 1_kg,
            InvRotInertia{Real{1} * SquareRadian / (SquareMeter * 1_kg)},
            lcA, old_pA, old_vA
        },
        BodyConstraint{
            Real(1) / 1_kg,
            InvRotInertia{Real{1} * SquareRadian / (SquareMeter * 1_kg)},
            lcB, old_pB, old_vB
        }
    };
    const auto pc = PositionConstraint{manifold, BodyID(0u), BodyID(1u),
        GetVertexRadius(shape) + GetVertexRadius(shape)};
    
    const auto conf = ConstraintSolverConf{};
    const auto solution = GaussSeidel::SolvePositionConstraint(pc, true, true, bodies, conf);
    
    EXPECT_LT(solution.min_separation, -conf.linearSlop);
    
    // object a moves left only
    EXPECT_LT(GetX(solution.pos_a.linear), GetX(old_pA.linear));
    EXPECT_EQ(GetY(solution.pos_a.linear), GetY(old_pA.linear));
    EXPECT_EQ(solution.pos_a.angular, old_pA.angular);
    
    // object b moves right only.
    EXPECT_GT(GetX(solution.pos_b.linear), GetX(old_pB.linear));
    EXPECT_EQ(GetY(solution.pos_b.linear), GetY(old_pB.linear));
    EXPECT_EQ(solution.pos_b.angular, old_pB.angular);
}

#if 0
TEST(ContactSolver, SolveVelocityConstraint1)
{
    const auto inverse_mass_a = Real(0);
    const auto inverse_mass_b = Real(0);
    const auto inverse_mass = inverse_mass_a + inverse_mass_b;

    const auto linear_velocity = Vec2{1, 1};
    const auto angular_velocity = 0_deg;

    const auto old_pA = Position{Vec2{0, 0}, 0_deg};
    const auto old_pB = Position{Vec2{0, 0}, 0_deg};
    const auto vel_a = Velocity{linear_velocity, angular_velocity};
    const auto vel_b = Velocity{linear_velocity, angular_velocity};
    const auto lcA = Vec2{};
    const auto lcB = Vec2{};

    auto body_data_a = BodyConstraint{0, inverse_mass_a, 0, lcA, old_pA, vel_a};
    auto body_data_b = BodyConstraint{1, inverse_mass_b, 0, lcB, old_pB, vel_b};
    const auto normal = UnitVec::GetTop();
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
    const auto angular_velocity = 0_deg;
    
    const auto old_pA = Position{Vec2{0, 0}, 0_deg};
    const auto old_pB = Position{Vec2{0, 0}, 0_deg};
    
    auto vel_a = Velocity{linear_velocity, angular_velocity};
    auto vel_b = Velocity{linear_velocity, angular_velocity};
    
    const auto lcA = Vec2{};
    const auto lcB = Vec2{};

    const auto inverse_mass_a = Real(1);
    const auto inverse_mass_b = Real(1);
    const auto inverse_mass = inverse_mass_a + inverse_mass_b;
    auto body_data_a = BodyConstraint{0, inverse_mass_a, 0, lcA, old_pA, vel_a};
    auto body_data_b = BodyConstraint{1, inverse_mass_b, 0, lcB, old_pB, vel_b};
    const auto normal = UnitVec::GetTop();
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
