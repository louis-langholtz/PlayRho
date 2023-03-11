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

#include <PlayRho/Dynamics/Joints/RopeJointConf.hpp>

#include <PlayRho/Dynamics/Joints/Joint.hpp>
#include <PlayRho/Dynamics/BodyConf.hpp>
#include <PlayRho/Dynamics/World.hpp>
#include <PlayRho/Dynamics/WorldJoint.hpp>
#include <PlayRho/Dynamics/WorldBody.hpp>
#include <PlayRho/Dynamics/WorldMisc.hpp>
#include <PlayRho/Dynamics/WorldShape.hpp>
#include <PlayRho/Dynamics/StepConf.hpp>
#include <PlayRho/Dynamics/Contacts/BodyConstraint.hpp>
#include <PlayRho/Dynamics/Contacts/ConstraintSolverConf.hpp>
#include <PlayRho/Collision/Shapes/DiskShapeConf.hpp>

#include <cstring> // for std::memcmp

using namespace playrho;
using namespace playrho::d2;

TEST(RopeJointConf, ByteSize)
{
    // Check size at test runtime instead of compile-time via static_assert to avoid stopping
    // builds and to report actual size rather than just reporting that expected size is wrong.
    switch (sizeof(Real)) {
    case 4:
        EXPECT_EQ(sizeof(RopeJointConf), std::size_t(68));
        break;
    case 8:
        EXPECT_EQ(sizeof(RopeJointConf), std::size_t(128));
        break;
    case 16:
        EXPECT_EQ(sizeof(RopeJointConf), std::size_t(256));
        break;
    default:
        FAIL();
        break;
    }
}

TEST(RopeJointConf, DefaultConstruction)
{
    RopeJointConf def{};

    EXPECT_EQ(def.bodyA, InvalidBodyID);
    EXPECT_EQ(def.bodyB, InvalidBodyID);
    EXPECT_EQ(def.collideConnected, false);

    EXPECT_EQ(def.localAnchorA, Length2(-1_m, 0_m));
    EXPECT_EQ(def.localAnchorB, Length2(+1_m, 0_m));
    EXPECT_EQ(def.maxLength, 0_m);
}

TEST(RopeJointConf, Construction)
{
    World world;
    const auto b0 = CreateBody(world);
    const auto b1 = CreateBody(world);

    auto def = RopeJointConf{b0, b1};
    Joint joint{def};

    EXPECT_EQ(GetType(joint), GetTypeID<RopeJointConf>());
    EXPECT_EQ(GetBodyA(joint), def.bodyA);
    EXPECT_EQ(GetBodyB(joint), def.bodyB);
    EXPECT_EQ(GetCollideConnected(joint), def.collideConnected);
    EXPECT_EQ(GetLinearReaction(joint), Momentum2{});
    EXPECT_EQ(GetAngularReaction(joint), AngularMomentum{0});

    const auto id = CreateJoint(world, joint);
    EXPECT_EQ(GetLocalAnchorA(joint), def.localAnchorA);
    EXPECT_EQ(GetLocalAnchorB(joint), def.localAnchorB);
    EXPECT_EQ(GetAnchorA(world, id), Length2(-1_m, 0_m));
    EXPECT_EQ(GetAnchorB(world, id), Length2(+1_m, 0_m));
    EXPECT_EQ(GetLimitState(joint), LimitState::e_inactiveLimit);
    const auto conf = TypeCast<RopeJointConf>(GetJoint(world, id));
    EXPECT_EQ(GetMaxLength(conf), def.maxLength);
}

TEST(RopeJointConf, GetRopeJointConfThrows)
{
    EXPECT_THROW(GetRopeJointConf(Joint{}), std::bad_cast);
}

TEST(RopeJointConf, GetRopeJointConf)
{
    auto world = World{};
    const auto bodyA = CreateBody(world);
    const auto bodyB = CreateBody(world);
    auto def = RopeJointConf{bodyA, bodyB};
    const auto localAnchorA = Length2{-2_m, 0_m};
    const auto localAnchorB = Length2{+2_m, 0_m};
    def.localAnchorA = localAnchorA;
    def.localAnchorB = localAnchorB;
    const auto joint = Joint{def};

    ASSERT_EQ(GetType(joint), GetTypeID<RopeJointConf>());
    ASSERT_EQ(GetBodyA(joint), def.bodyA);
    ASSERT_EQ(GetBodyB(joint), def.bodyB);
    ASSERT_EQ(GetCollideConnected(joint), def.collideConnected);

    ASSERT_EQ(GetLocalAnchorA(joint), def.localAnchorA);
    ASSERT_EQ(GetLocalAnchorB(joint), def.localAnchorB);
    const auto conf = TypeCast<RopeJointConf>(joint);
    ASSERT_EQ(GetMaxLength(conf), def.maxLength);

    const auto cdef = GetRopeJointConf(joint);
    EXPECT_EQ(cdef.bodyA, bodyA);
    EXPECT_EQ(cdef.bodyB, bodyB);
    EXPECT_EQ(cdef.collideConnected, false);

    EXPECT_EQ(cdef.localAnchorA, localAnchorA);
    EXPECT_EQ(cdef.localAnchorB, localAnchorB);
    EXPECT_EQ(cdef.maxLength, 0_m);
}

TEST(RopeJointConf, WithDynamicCircles)
{
    auto world = World{};
    const auto p1 = Length2{-1_m, 0_m};
    const auto p2 = Length2{+1_m, 0_m};
    const auto b1 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p1));
    const auto b2 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p2));
    const auto shapeId = CreateShape(world, DiskShapeConf{}.UseRadius(0.2_m));
    Attach(world, b1, shapeId);
    Attach(world, b2, shapeId);
    const auto jd = RopeJointConf{b1, b2};
    ASSERT_NE(CreateJoint(world, jd), InvalidJointID);

    auto stepConf = StepConf{};

    stepConf.doWarmStart = true;
    Step(world, stepConf);
    EXPECT_GT(GetX(GetLocation(world, b1)), -1_m);
    EXPECT_EQ(GetY(GetLocation(world, b1)), 0_m);
    EXPECT_LT(GetX(GetLocation(world, b2)), +1_m);
    EXPECT_EQ(GetY(GetLocation(world, b2)), 0_m);
    EXPECT_EQ(GetAngle(world, b1), 0_deg);
    EXPECT_EQ(GetAngle(world, b2), 0_deg);

    stepConf.doWarmStart = false;
    Step(world, stepConf);
    EXPECT_GT(GetX(GetLocation(world, b1)), -1_m);
    EXPECT_EQ(GetY(GetLocation(world, b1)), 0_m);
    EXPECT_LT(GetX(GetLocation(world, b2)), +1_m);
    EXPECT_EQ(GetY(GetLocation(world, b2)), 0_m);
    EXPECT_EQ(GetAngle(world, b1), 0_deg);
    EXPECT_EQ(GetAngle(world, b2), 0_deg);

    stepConf.doWarmStart = true;
    stepConf.linearSlop = 10_m;
    Step(world, stepConf);
    EXPECT_GT(GetX(GetLocation(world, b1)), -1_m);
    EXPECT_EQ(GetY(GetLocation(world, b1)), 0_m);
    EXPECT_LT(GetX(GetLocation(world, b2)), +1_m);
    EXPECT_EQ(GetY(GetLocation(world, b2)), 0_m);
    EXPECT_EQ(GetAngle(world, b1), 0_deg);
    EXPECT_EQ(GetAngle(world, b2), 0_deg);
}

TEST(RopeJointConf, ShiftOrigin)
{
    auto jd = RopeJointConf{BodyID(0u), BodyID(1u)};
    auto copy = RopeJointConf{};

    // Do copy = jd without missing padding so memcmp works
    std::memcpy(&copy, &jd, sizeof(RopeJointConf));

    EXPECT_FALSE(ShiftOrigin(jd, Length2{0_m, 0_m}));

    // Use memcmp since easier than writing full == suport pre C++20.
    EXPECT_TRUE(std::memcmp(&jd, &copy, sizeof(RopeJointConf)) == 0);
}

TEST(RopeJointConf, EqualsOperator)
{
    EXPECT_TRUE(RopeJointConf() == RopeJointConf());
    {
        auto conf = RopeJointConf{};
        conf.localAnchorA = Length2{1.2_m, -3_m};
        EXPECT_TRUE(conf == conf);
        EXPECT_FALSE(RopeJointConf() == conf);
    }
    {
        auto conf = RopeJointConf{};
        conf.localAnchorB = Length2{1.2_m, -3_m};
        EXPECT_TRUE(conf == conf);
        EXPECT_FALSE(RopeJointConf() == conf);
    }
    {
        auto conf = RopeJointConf{};
        conf.maxLength = 12_m;
        EXPECT_TRUE(conf == conf);
        EXPECT_FALSE(RopeJointConf() == conf);
    }
    // TODO: test remaining fields.
}

TEST(RopeJointConf, NotEqualsOperator)
{
    EXPECT_FALSE(RopeJointConf() != RopeJointConf());
    {
        auto conf = RopeJointConf{};
        conf.mass = 13_kg;
        EXPECT_FALSE(conf != conf);
        EXPECT_TRUE(RopeJointConf() != conf);
    }
    // TODO: test remaining fields.
}

TEST(RopeJointConf, GetName)
{
    EXPECT_STREQ(GetName(GetTypeID<RopeJointConf>()), "d2::RopeJointConf");
}

TEST(RopeJointConf, InitVelocity)
{
    auto conf = RopeJointConf{};
    std::vector<BodyConstraint> bodies;
    EXPECT_NO_THROW(InitVelocity(conf, bodies, StepConf{}, ConstraintSolverConf{}));
    conf.bodyA = BodyID(0);
    conf.bodyB = BodyID(0);
    EXPECT_THROW(InitVelocity(conf, bodies, StepConf{}, ConstraintSolverConf{}), std::out_of_range);
    const auto posA = Position{Length2{-5_m, 0_m}, 0_deg};
    bodies.push_back(BodyConstraint{Real(1) / 4_kg, InvRotInertia{}, Length2{}, posA, Velocity{}});
    EXPECT_NO_THROW(InitVelocity(conf, bodies, StepConf{}, ConstraintSolverConf{}));
}

TEST(RopeJointConf, SolveVelocity)
{
    auto conf = RopeJointConf{};
    std::vector<BodyConstraint> bodies;
    auto result = false;
    EXPECT_NO_THROW(result = SolveVelocity(conf, bodies, StepConf{}));
    EXPECT_TRUE(result);
    conf.bodyA = BodyID(0);
    conf.bodyB = BodyID(0);
    EXPECT_THROW(SolveVelocity(conf, bodies, StepConf{}), std::out_of_range);
    const auto posA = Position{Length2{-5_m, 0_m}, 0_deg};
    bodies.push_back(BodyConstraint{Real(1) / 4_kg, InvRotInertia{}, Length2{}, posA, Velocity{}});
    EXPECT_NO_THROW(result = SolveVelocity(conf, bodies, StepConf{}));
}

TEST(RopeJointConf, SolvePosition)
{
    auto conf = RopeJointConf{};
    std::vector<BodyConstraint> bodies;
    auto result = false;
    EXPECT_NO_THROW(result = SolvePosition(conf, bodies, ConstraintSolverConf{}));
    EXPECT_TRUE(result);
    conf.bodyA = BodyID(0);
    conf.bodyB = BodyID(0);
    EXPECT_THROW(SolvePosition(conf, bodies, ConstraintSolverConf{}), std::out_of_range);
    const auto posA = Position{Length2{-5_m, 0_m}, 0_deg};
    bodies.push_back(BodyConstraint{Real(1) / 4_kg, InvRotInertia{}, Length2{}, posA, Velocity{}});
    EXPECT_NO_THROW(result = SolvePosition(conf, bodies, ConstraintSolverConf{}));
}
