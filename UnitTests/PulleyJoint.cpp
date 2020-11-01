/*
 * Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#include <PlayRho/Dynamics/Joints/PulleyJointConf.hpp>

#include <PlayRho/Dynamics/StepConf.hpp>
#include <PlayRho/Dynamics/World.hpp>
#include <PlayRho/Dynamics/WorldJoint.hpp>
#include <PlayRho/Dynamics/WorldBody.hpp>
#include <PlayRho/Dynamics/Joints/Joint.hpp>

#include <PlayRho/Dynamics/Contacts/ContactSolver.hpp>
#include <PlayRho/Dynamics/Contacts/BodyConstraint.hpp>

#include <stdexcept>

using namespace playrho;
using namespace playrho::d2;

TEST(PulleyJointConf, DefaultConstruction)
{
    PulleyJointConf def;
    
    EXPECT_EQ(def.bodyA, InvalidBodyID);
    EXPECT_EQ(def.bodyB, InvalidBodyID);
    EXPECT_EQ(def.collideConnected, true);
    
    EXPECT_EQ(def.groundAnchorA, PulleyJointConf::DefaultGroundAnchorA);
    EXPECT_EQ(def.groundAnchorB, PulleyJointConf::DefaultGroundAnchorB);
    EXPECT_EQ(def.localAnchorA, PulleyJointConf::DefaultLocalAnchorA);
    EXPECT_EQ(def.localAnchorB, PulleyJointConf::DefaultLocalAnchorB);
    EXPECT_EQ(def.lengthA, 0_m);
    EXPECT_EQ(def.lengthB, 0_m);
    EXPECT_EQ(def.ratio, Real(1));
    EXPECT_EQ(def.constant, 0_m);

    EXPECT_EQ(def.impulse, 0_Ns);
    ASSERT_EQ(UnitVec(), UnitVec::GetZero());
    EXPECT_EQ(def.uA, UnitVec());
    EXPECT_EQ(def.uB, UnitVec());
    EXPECT_EQ(def.rA, Length2());
    EXPECT_EQ(def.rB, Length2());
    EXPECT_EQ(def.mass, 0_kg);
}

TEST(PulleyJointConf, InitializingConstructor)
{
    const auto bA = BodyID(2u);
    const auto bB = BodyID(4u);
    const auto gndA = Length2{-5_m, -4.2_m};
    const auto gndB = Length2{+2.3_m, +3.1_m};
    const auto locA = Length2{-1.1_m, +0.2_m};
    const auto locB = Length2{-1.4_m, +2.9_m};
    const auto lenA = 2.2_m;
    const auto lenB = 0.24_m;
    EXPECT_EQ(PulleyJointConf(bA, bB).bodyA, bA);
    EXPECT_EQ(PulleyJointConf(bA, bB).bodyB, bB);
    EXPECT_EQ(PulleyJointConf(bA, bB, gndA, gndB).groundAnchorA, gndA);
    EXPECT_EQ(PulleyJointConf(bA, bB, gndA, gndB).groundAnchorB, gndB);
    EXPECT_EQ(PulleyJointConf(bA, bB, gndA, gndB, locA, locB).localAnchorA, locA);
    EXPECT_EQ(PulleyJointConf(bA, bB, gndA, gndB, locA, locB).localAnchorB, locB);
    EXPECT_EQ(PulleyJointConf(bA, bB, gndA, gndB, locA, locB, lenA, lenB).lengthA, lenA);
    EXPECT_EQ(PulleyJointConf(bA, bB, gndA, gndB, locA, locB, lenA, lenB).lengthB, lenB);
}

TEST(PulleyJointConf, GetPulleyJointConfForWorld)
{
    auto world = World{};
    const auto posA = Length2{+1_m, +1_m};
    const auto posB = Length2{-1_m, -1_m};
    const auto bA = CreateBody(world, BodyConf{}.UseLocation(posA));
    const auto bB = CreateBody(world, BodyConf{}.UseLocation(posB));
    const auto gA = Length2{2.2_m, 3.0_m};
    const auto gB = Length2{-1.0_m, 1_m};
    const auto aA = Length2{+10_m, 10_m};
    const auto aB = Length2{-10_m, 10_m};
    const auto conf = GetPulleyJointConf(world, bA, bB, gA, gB, aA, aB);
    EXPECT_EQ(conf.bodyA, bA);
    EXPECT_EQ(conf.bodyB, bB);
    EXPECT_EQ(conf.groundAnchorA, gA);
    EXPECT_EQ(conf.groundAnchorB, gB);
    EXPECT_EQ(conf.localAnchorA, aA - posA);
    EXPECT_EQ(conf.localAnchorB, aB - posB);
    EXPECT_NEAR(static_cast<double>(Real(conf.lengthA/1_m)), 10.4805, 0.0001);
    EXPECT_NEAR(static_cast<double>(Real(conf.lengthB/1_m)), 12.7279, 0.0001);
}

TEST(PulleyJointConf, GetPulleyJointConfForJoint)
{
    const auto bA = BodyID(2u);
    const auto bB = BodyID(4u);
    const auto gndA = Length2{-5_m, -4.2_m};
    const auto gndB = Length2{+2.3_m, +3.1_m};
    const auto locA = Length2{-1.1_m, +0.2_m};
    const auto locB = Length2{-1.4_m, +2.9_m};
    const auto lenA = 2.2_m;
    const auto lenB = 0.24_m;
    const auto joint = PulleyJointConf(bA, bB, gndA, gndB, locA, locB, lenA, lenB);
    const auto conf = GetPulleyJointConf(joint);
    EXPECT_EQ(conf.bodyA, bA);
    EXPECT_EQ(conf.bodyB, bB);
    EXPECT_EQ(conf.groundAnchorA, gndA);
    EXPECT_EQ(conf.groundAnchorB, gndB);
    EXPECT_EQ(conf.localAnchorA, locA);
    EXPECT_EQ(conf.localAnchorB, locB);
    EXPECT_EQ(conf.lengthA, lenA);
    EXPECT_EQ(conf.lengthB, lenB);
}

TEST(PulleyJointConf, UseRatio)
{
    const auto value = Real(31);
    EXPECT_NE(PulleyJointConf{}.ratio, value);
    EXPECT_EQ(PulleyJointConf{}.UseRatio(value).ratio, value);
}

TEST(PulleyJointConf, ByteSize)
{
    switch (sizeof(Real))
    {
        case  4:
#if defined(_WIN32) && !defined(_WIN64)
            EXPECT_EQ(sizeof(PulleyJointConf), std::size_t(96));
#else
            EXPECT_EQ(sizeof(PulleyJointConf), std::size_t(96));
#endif
            break;
        case  8: EXPECT_EQ(sizeof(PulleyJointConf), std::size_t(184)); break;
        case 16: EXPECT_EQ(sizeof(PulleyJointConf), std::size_t(368)); break;
        default: FAIL(); break;
    }
}

TEST(PulleyJoint, Construction)
{
    PulleyJointConf def;
    Joint joint{def};

    EXPECT_EQ(GetType(joint), GetTypeID<PulleyJointConf>());
    EXPECT_EQ(GetBodyA(joint), def.bodyA);
    EXPECT_EQ(GetBodyB(joint), def.bodyB);
    EXPECT_EQ(GetCollideConnected(joint), def.collideConnected);
    EXPECT_EQ(GetLinearReaction(joint), Momentum2{});
    EXPECT_EQ(GetAngularReaction(joint), AngularMomentum{0});

    EXPECT_EQ(GetGroundAnchorA(joint), def.groundAnchorA);
    EXPECT_EQ(GetGroundAnchorB(joint), def.groundAnchorB);
    EXPECT_EQ(GetLocalAnchorA(joint), def.localAnchorA);
    EXPECT_EQ(GetLocalAnchorB(joint), def.localAnchorB);
#if 0
    EXPECT_EQ(GetLengthA(joint), def.lengthA);
    EXPECT_EQ(GetLengthB(joint), def.lengthB);
#endif
    EXPECT_EQ(GetRatio(joint), def.ratio);
}

TEST(PulleyJoint, GetAnchorAandB)
{
    auto world = World{};
    
    const auto loc0 = Length2{+1_m, -3_m};
    const auto loc1 = Length2{-2_m, Real(+1.2f) * Meter};
    
    const auto b0 = world.CreateBody(BodyConf{}.UseLocation(loc0));
    const auto b1 = world.CreateBody(BodyConf{}.UseLocation(loc1));
    
    auto jd = PulleyJointConf{};
    jd.bodyA = b0;
    jd.bodyB = b1;
    jd.localAnchorA = Length2(4_m, 5_m);
    jd.localAnchorB = Length2(6_m, 7_m);
    
    auto joint = Joint{jd};
    ASSERT_EQ(GetLocalAnchorA(joint), jd.localAnchorA);
    ASSERT_EQ(GetLocalAnchorB(joint), jd.localAnchorB);
#if 0
    EXPECT_EQ(joint.GetAnchorA(world), loc0 + jd.localAnchorA);
    EXPECT_EQ(joint.GetAnchorB(world), loc1 + jd.localAnchorB);
#endif
}

TEST(PulleyJoint, ShiftOrigin)
{
    PulleyJointConf def;
    Joint joint{def};
    
    ASSERT_EQ(GetGroundAnchorA(joint), def.groundAnchorA);
    ASSERT_EQ(GetGroundAnchorB(joint), def.groundAnchorB);
    
    const auto newOrigin = Length2{1_m, 1_m};

    EXPECT_TRUE(ShiftOrigin(joint, newOrigin));
    EXPECT_EQ(GetGroundAnchorA(joint), def.groundAnchorA - newOrigin);
    EXPECT_EQ(GetGroundAnchorB(joint), def.groundAnchorB - newOrigin);
}

TEST(PulleyJoint, GetCurrentLength)
{
    auto world = World{};

    const auto loc0 = Length2{+1_m, -3_m};
    const auto loc1 = Length2{-2_m, Real(+1.2f) * Meter};

    const auto b0 = world.CreateBody(BodyConf{}.UseLocation(loc0));
    const auto b1 = world.CreateBody(BodyConf{}.UseLocation(loc1));

    auto jd = PulleyJointConf{};
    jd.bodyA = b0;
    jd.bodyB = b1;
    jd.localAnchorA = Length2(4_m, 5_m);
    jd.localAnchorB = Length2(6_m, 7_m);

    auto joint = Joint{jd};
    ASSERT_EQ(GetLocalAnchorA(joint), jd.localAnchorA);
    ASSERT_EQ(GetLocalAnchorB(joint), jd.localAnchorB);
    ASSERT_EQ(GetGroundAnchorA(joint), jd.groundAnchorA);
    ASSERT_EQ(GetGroundAnchorB(joint), jd.groundAnchorB);

    const auto lenA = GetMagnitude(GetWorldPoint(world, GetBodyA(joint),
                                                 jd.localAnchorA - jd.groundAnchorA));
    const auto lenB = GetMagnitude(GetWorldPoint(world, GetBodyB(joint),
                                                 jd.localAnchorB - jd.groundAnchorB));
    const auto id = CreateJoint(world, joint);
    EXPECT_EQ(GetCurrentLengthA(world, id), lenA);
    EXPECT_EQ(GetCurrentLengthB(world, id), lenB);
}

TEST(PulleyJointConf, InitVelocityThrowsOutOfRange)
{
    auto jd = PulleyJointConf{};
    jd.bodyA = BodyID(0u);
    jd.bodyB = BodyID(0u);
    std::vector<BodyConstraint> bodies;
    EXPECT_THROW(InitVelocity(jd, bodies, StepConf{}, ConstraintSolverConf{}),
                 std::out_of_range);
    bodies.push_back(BodyConstraint{});
    EXPECT_NO_THROW(InitVelocity(jd, bodies, StepConf{}, ConstraintSolverConf{}));
}

TEST(PulleyJointConf, InitVelocityWithDefaultConstructed)
{
    std::vector<BodyConstraint> bodies;
    bodies.push_back(BodyConstraint{});
    bodies.push_back(BodyConstraint{});
    ASSERT_EQ(bodies.size(), 2u);
    ASSERT_EQ(bodies[0].GetPosition(), Position());
    ASSERT_EQ(bodies[0].GetVelocity(), Velocity());
    ASSERT_EQ(bodies[1].GetPosition(), Position());
    ASSERT_EQ(bodies[1].GetVelocity(), Velocity());

    auto jd = PulleyJointConf{};
    jd.bodyA = BodyID(0u);
    jd.bodyB = BodyID(1u);
    const auto copy = jd;
    ASSERT_EQ(jd.bodyA, BodyID(0u));
    ASSERT_EQ(jd.bodyB, BodyID(1u));
    ASSERT_EQ(jd.mass, 0_kg);
    ASSERT_EQ(jd.impulse, 0_Ns);
    ASSERT_EQ(jd.uA, UnitVec());
    ASSERT_EQ(jd.uB, UnitVec());
    ASSERT_EQ(jd.rA, Length2());
    ASSERT_EQ(jd.rB, Length2());

    EXPECT_NO_THROW(InitVelocity(jd, bodies, StepConf{}, ConstraintSolverConf{}));
    EXPECT_EQ(jd.bodyA, copy.bodyA);
    EXPECT_EQ(jd.bodyB, copy.bodyB);
    EXPECT_EQ(jd.collideConnected, copy.collideConnected);
    EXPECT_EQ(jd.mass, copy.mass);
    EXPECT_EQ(jd.impulse, copy.impulse);
    EXPECT_NE(jd.uA, copy.uA);
    EXPECT_NE(jd.uB, copy.uB);
    EXPECT_EQ(jd.uA, UnitVec::GetBottom());
    EXPECT_EQ(jd.uB, UnitVec::GetBottom());
    EXPECT_NE(jd.rA, copy.rA);
    EXPECT_NE(jd.rB, copy.rB);
    EXPECT_EQ(jd.rA, Length2(-1_m, 0_m));
    EXPECT_EQ(jd.rB, Length2(+1_m, 0_m));
    EXPECT_EQ(bodies[0].GetPosition(), Position());
    EXPECT_EQ(bodies[0].GetVelocity(), Velocity());
    EXPECT_EQ(bodies[1].GetPosition(), Position());
    EXPECT_EQ(bodies[1].GetVelocity(), Velocity());
}

TEST(PulleyJointConf, InitVelocityWarmStartUpdatesImpulse)
{
    auto stepConf = StepConf{};
    auto jd = PulleyJointConf{};
    jd.bodyA = BodyID(0u);
    jd.bodyB = BodyID(1u);
    std::vector<BodyConstraint> bodies;
    bodies.push_back(BodyConstraint{});
    bodies.push_back(BodyConstraint{});
    stepConf.dtRatio = Real(3);
    stepConf.doWarmStart = true;
    const auto originalImpulse = 2_Ns;
    jd.impulse = originalImpulse;

    EXPECT_NO_THROW(InitVelocity(jd, bodies, stepConf, ConstraintSolverConf{}));
    EXPECT_EQ(jd.impulse, originalImpulse * stepConf.dtRatio);
}

TEST(PulleyJointConf, InitVelocityColdStartResetsImpulse)
{
    auto stepConf = StepConf{};
    auto jd = PulleyJointConf{};
    jd.bodyA = BodyID(0u);
    jd.bodyB = BodyID(1u);
    std::vector<BodyConstraint> bodies;
    bodies.push_back(BodyConstraint{});
    bodies.push_back(BodyConstraint{});
    stepConf.dtRatio = Real(3);
    stepConf.doWarmStart = false;
    const auto originalImpulse = 2_Ns;
    jd.impulse = originalImpulse;

    EXPECT_NO_THROW(InitVelocity(jd, bodies, stepConf, ConstraintSolverConf{}));
    EXPECT_EQ(jd.impulse, 0_Ns);
}

TEST(PulleyJointConf, InitVelocitySetsMass)
{
    auto stepConf = StepConf{};
    auto jd = PulleyJointConf{};
    jd.bodyA = BodyID(0u);
    jd.bodyB = BodyID(1u);
    std::vector<BodyConstraint> bodies;
    bodies.push_back(BodyConstraint{
        Real(1)/4_kg, InvRotInertia{}, Length2{}, Position{}, Velocity{}
    });
    bodies.push_back(BodyConstraint{
        Real(1)/4_kg, InvRotInertia{}, Length2{}, Position{}, Velocity{}
    });
    stepConf.dtRatio = Real(1);
    stepConf.doWarmStart = false;
    ASSERT_EQ(jd.mass, 0_kg);
    EXPECT_NO_THROW(InitVelocity(jd, bodies, stepConf, ConstraintSolverConf{}));
    EXPECT_EQ(jd.mass, 2_kg);
}

TEST(PulleyJointConf, SolveVelocity)
{
    auto jd = PulleyJointConf{};
    std::vector<BodyConstraint> bodies;
    EXPECT_THROW(SolveVelocity(jd, bodies, StepConf{}), std::out_of_range);

    jd.bodyA = BodyID(0u);
    jd.bodyB = BodyID(0u);
    bodies.push_back(BodyConstraint{});
    EXPECT_NO_THROW(SolveVelocity(jd, bodies, StepConf{}));
    EXPECT_EQ(bodies[0].GetPosition(), Position());
    EXPECT_EQ(bodies[0].GetVelocity(), Velocity());

    jd.bodyB = BodyID(1u);
    bodies.push_back(BodyConstraint{});
    EXPECT_NO_THROW(SolveVelocity(jd, bodies, StepConf{}));
    EXPECT_EQ(bodies[0].GetPosition(), Position());
    EXPECT_EQ(bodies[1].GetPosition(), Position());
    EXPECT_EQ(bodies[0].GetVelocity(), Velocity());
    EXPECT_EQ(bodies[1].GetVelocity(), Velocity());
}

TEST(PulleyJointConf, SolvePosition)
{
    auto jd = PulleyJointConf{};
    std::vector<BodyConstraint> bodies;
    EXPECT_THROW(SolvePosition(jd, bodies, ConstraintSolverConf{}), std::out_of_range);

    jd.bodyA = BodyID(0u);
    jd.bodyB = BodyID(0u);
    bodies.push_back(BodyConstraint{});
    EXPECT_NO_THROW(SolvePosition(jd, bodies, ConstraintSolverConf{}));
    EXPECT_EQ(bodies[0].GetPosition(), Position());

    jd.bodyB = BodyID(1u);
    bodies.push_back(BodyConstraint{});
    EXPECT_NO_THROW(SolvePosition(jd, bodies, ConstraintSolverConf{}));
    EXPECT_EQ(bodies[0].GetPosition(), Position());
    EXPECT_EQ(bodies[1].GetPosition(), Position());
    EXPECT_EQ(bodies[0].GetVelocity(), Velocity());
    EXPECT_EQ(bodies[1].GetVelocity(), Velocity());
}
