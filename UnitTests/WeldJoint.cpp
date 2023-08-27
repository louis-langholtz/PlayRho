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

#include <PlayRho/d2/WeldJointConf.hpp>
#include <PlayRho/d2/Joint.hpp>

#include <PlayRho/Dynamics/BodyConf.hpp>
#include <PlayRho/Dynamics/World.hpp>
#include <PlayRho/Dynamics/WorldShape.hpp>
#include <PlayRho/Dynamics/WorldMisc.hpp>
#include <PlayRho/Dynamics/WorldBody.hpp>
#include <PlayRho/Dynamics/WorldJoint.hpp>
#include <PlayRho/StepConf.hpp>
#include <PlayRho/d2/BodyConstraint.hpp>
#include <PlayRho/ConstraintSolverConf.hpp>
#include <PlayRho/Collision/Shapes/DiskShapeConf.hpp>

using namespace playrho;
using namespace playrho::d2;

TEST(WeldJoint, Traits)
{
    EXPECT_FALSE((IsIterable<WeldJointConf>::value));
    EXPECT_FALSE((IsAddable<WeldJointConf>::value));
    EXPECT_FALSE((IsAddable<WeldJointConf, WeldJointConf>::value));
}

TEST(WeldJointConf, DefaultConstruction)
{
    WeldJointConf def{};

    EXPECT_EQ(def.bodyA, InvalidBodyID);
    EXPECT_EQ(def.bodyB, InvalidBodyID);
    EXPECT_EQ(def.collideConnected, false);

    EXPECT_EQ(def.localAnchorA, (Length2{}));
    EXPECT_EQ(def.localAnchorB, (Length2{}));
    EXPECT_EQ(def.referenceAngle, 0_deg);
    EXPECT_EQ(def.frequency, 0_Hz);
    EXPECT_EQ(def.dampingRatio, Real(0));
}

TEST(WeldJointConf, InitalizingConstruction)
{
    const auto bA = BodyID(1);
    const auto bB = BodyID(2);
    const auto laA = Length2(-4.2_m, 3.8_m);
    const auto laB = Length2(5.1_m, 4_m);
    const auto ra = 90_deg;
    const auto def = WeldJointConf{bA, bB, laA, laB, ra};

    EXPECT_EQ(def.bodyA, bA);
    EXPECT_EQ(def.bodyB, bB);
    EXPECT_EQ(def.collideConnected, false);

    EXPECT_EQ(def.localAnchorA, laA);
    EXPECT_EQ(def.localAnchorB, laB);
    EXPECT_EQ(def.referenceAngle, ra);
    EXPECT_EQ(def.frequency, 0_Hz);
    EXPECT_EQ(def.dampingRatio, Real(0));
}

TEST(WeldJoint, Construction)
{
    WeldJointConf def;
    Joint joint{def};

    EXPECT_EQ(GetType(joint), GetTypeID<WeldJointConf>());
    EXPECT_EQ(GetBodyA(joint), def.bodyA);
    EXPECT_EQ(GetBodyB(joint), def.bodyB);
    EXPECT_EQ(GetCollideConnected(joint), def.collideConnected);
    EXPECT_EQ(GetLinearReaction(joint), Momentum2{});
    EXPECT_EQ(GetAngularReaction(joint), AngularMomentum{0});

    EXPECT_EQ(GetLocalAnchorA(joint), def.localAnchorA);
    EXPECT_EQ(GetLocalAnchorB(joint), def.localAnchorB);
    EXPECT_EQ(GetReferenceAngle(joint), def.referenceAngle);
    EXPECT_EQ(GetFrequency(joint), def.frequency);
    EXPECT_EQ(GetDampingRatio(joint), def.dampingRatio);
}

TEST(WeldJoint, GetWeldJointConfThrows)
{
    EXPECT_THROW(GetWeldJointConf(Joint{}), std::bad_cast);
}

TEST(WeldJoint, GetWeldJointConf)
{
    auto world = World{};
    const auto bodyA = CreateBody(world);
    const auto bodyB = CreateBody(world);
    const auto anchor = Length2(2_m, 1_m);
    const auto def = GetWeldJointConf(world, bodyA, bodyB, anchor);
    Joint joint{def};

    ASSERT_EQ(GetType(joint), GetTypeID<WeldJointConf>());
    ASSERT_EQ(GetBodyA(joint), def.bodyA);
    ASSERT_EQ(GetBodyB(joint), def.bodyB);
    ASSERT_EQ(GetCollideConnected(joint), def.collideConnected);

    ASSERT_EQ(GetLocalAnchorA(joint), def.localAnchorA);
    ASSERT_EQ(GetLocalAnchorB(joint), def.localAnchorB);
    ASSERT_EQ(GetReferenceAngle(joint), def.referenceAngle);
    ASSERT_EQ(GetFrequency(joint), def.frequency);
    ASSERT_EQ(GetDampingRatio(joint), def.dampingRatio);

    const auto cdef = GetWeldJointConf(joint);
    EXPECT_EQ(cdef.bodyA, bodyA);
    EXPECT_EQ(cdef.bodyB, bodyB);
    EXPECT_EQ(cdef.collideConnected, false);

    EXPECT_EQ(cdef.localAnchorA, anchor);
    EXPECT_EQ(cdef.localAnchorB, anchor);
    EXPECT_EQ(def.referenceAngle, 0_deg);
    EXPECT_EQ(def.frequency, 0_Hz);
    EXPECT_EQ(def.dampingRatio, Real(0));
}

TEST(WeldJoint, WithDynamicCircles)
{
    auto world = World{};
    const auto p1 = Length2{-1_m, 0_m};
    const auto p2 = Length2{+1_m, 0_m};
    const auto shapeId = CreateShape(world, DiskShapeConf{}.UseRadius(0.2_m));
    const auto b1 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p1));
    const auto b2 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p2));
    Attach(world, b1, shapeId);
    Attach(world, b2, shapeId);
    const auto anchor = Length2(2_m, 1_m);
    const auto jd = GetWeldJointConf(world, b1, b2, anchor);
    CreateJoint(world, Joint{jd});
    Step(world, 1_s);
    EXPECT_NEAR(double(Real{GetX(GetLocation(world, b1)) / Meter}), -1.0, 0.001);
    EXPECT_NEAR(double(Real{GetY(GetLocation(world, b1)) / Meter}), 0.0, 0.001);
    EXPECT_NEAR(double(Real{GetX(GetLocation(world, b2)) / Meter}), +1.0, 0.01);
    EXPECT_NEAR(double(Real{GetY(GetLocation(world, b2)) / Meter}), 0.0, 0.01);
    EXPECT_EQ(GetAngle(world, b1), 0_deg);
    EXPECT_EQ(GetAngle(world, b2), 0_deg);
}

TEST(WeldJoint, WithDynamicCircles2)
{
    auto world = World{};
    const auto p1 = Length2{-1_m, 0_m};
    const auto p2 = Length2{+1_m, 0_m};
    const auto shapeId = CreateShape(world, DiskShapeConf{}.UseRadius(0.2_m));
    const auto b1 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p1));
    const auto b2 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p2));
    Attach(world, b1, shapeId);
    Attach(world, b2, shapeId);
    const auto anchor = Length2(2_m, 1_m);
    const auto jd = GetWeldJointConf(world, b1, b2, anchor).UseFrequency(10_Hz);
    const auto joint = CreateJoint(world, Joint{jd});
    ASSERT_NE(joint, InvalidJointID);
    ASSERT_EQ(GetFrequency(world, joint), 10_Hz);
    auto stepConf = StepConf{};

    stepConf.doWarmStart = true;
    Step(world, stepConf);
    EXPECT_NEAR(double(Real{GetX(GetLocation(world, b1)) / Meter}), -1.0, 0.001);
    EXPECT_NEAR(double(Real{GetY(GetLocation(world, b1)) / Meter}), 0.0, 0.001);
    EXPECT_NEAR(double(Real{GetX(GetLocation(world, b2)) / Meter}), +1.0, 0.01);
    EXPECT_NEAR(double(Real{GetY(GetLocation(world, b2)) / Meter}), 0.0, 0.01);
    EXPECT_EQ(GetAngle(world, b1), 0_deg);
    EXPECT_EQ(GetAngle(world, b2), 0_deg);

    stepConf.doWarmStart = false;
    Step(world, stepConf);
    EXPECT_NEAR(double(Real{GetX(GetLocation(world, b1)) / Meter}), -1.0, 0.001);
    EXPECT_NEAR(double(Real{GetY(GetLocation(world, b1)) / Meter}), 0.0, 0.001);
    EXPECT_NEAR(double(Real{GetX(GetLocation(world, b2)) / Meter}), +1.0, 0.01);
    EXPECT_NEAR(double(Real{GetY(GetLocation(world, b2)) / Meter}), 0.0, 0.01);
    EXPECT_EQ(GetAngle(world, b1), 0_deg);
    EXPECT_EQ(GetAngle(world, b2), 0_deg);
}

TEST(WeldJoint, GetAnchorAandB)
{
    auto world = World{};

    const auto loc1 = Length2{+1_m, -3_m};
    const auto loc2 = Length2{-2_m, Real(+1.2f) * Meter};
    const auto anchor = Length2(2_m, 1_m);

    const auto b1 = CreateBody(world, BodyConf{}.UseLocation(loc1));
    const auto b2 = CreateBody(world, BodyConf{}.UseLocation(loc2));

    auto jd = GetWeldJointConf(world, b1, b2, anchor);
    jd.localAnchorA = Length2(4_m, 5_m);
    jd.localAnchorB = Length2(6_m, 7_m);
    const auto joint = CreateJoint(world, Joint{jd});
    ASSERT_NE(joint, InvalidJointID);

    ASSERT_EQ(GetLocalAnchorA(world, joint), jd.localAnchorA);
    ASSERT_EQ(GetLocalAnchorB(world, joint), jd.localAnchorB);
    EXPECT_EQ(GetAnchorA(world, joint), loc1 + jd.localAnchorA);
    EXPECT_EQ(GetAnchorB(world, joint), loc2 + jd.localAnchorB);
}

TEST(WeldJointConf, ShiftOrigin)
{
    auto def = WeldJointConf{};
    def.bodyA = BodyID(1u);
    def.bodyB = BodyID(2u);
    def.localAnchorA = Length2{-2_m, +3_m};
    def.localAnchorB = Length2{+2_m, -3_m};
    def.referenceAngle = 23_deg;
    def.frequency = 44_Hz;
    def.dampingRatio = Real(99);
    def.impulse = Vec3{Real(1), Real(2), Real(3)};
    const auto rotInertia = RotInertia{1_kg * 1_m2 / SquareRadian};
    def.gamma = Real(2) / rotInertia;
    def.bias = 2_rpm;
    def.rA = Length2{3_m, 22_m};
    def.rB = Length2{2_m, 22_m};
    def.mass = Mat33{Vec3{Real(1), Real(2), Real(3)}, Vec3{Real(4), Real(5), Real(6)},
                     Vec3{Real(7), Real(8), Real(9)}};
    const auto amount = Length2{1_m, 2_m};
    const auto copy = def;
    EXPECT_FALSE(ShiftOrigin(def, amount));
    EXPECT_EQ(def.bodyA, copy.bodyA);
    EXPECT_EQ(def.bodyB, copy.bodyB);
    EXPECT_EQ(def.collideConnected, copy.collideConnected);
    EXPECT_EQ(def.localAnchorA, copy.localAnchorA);
    EXPECT_EQ(def.localAnchorB, copy.localAnchorB);
    EXPECT_EQ(def.referenceAngle, copy.referenceAngle);
    EXPECT_EQ(def.frequency, copy.frequency);
    EXPECT_EQ(def.dampingRatio, copy.dampingRatio);
    EXPECT_EQ(def.impulse, copy.impulse);
    EXPECT_EQ(def.gamma, copy.gamma);
    EXPECT_EQ(def.bias, copy.bias);
    EXPECT_EQ(def.rA, copy.rA);
    EXPECT_EQ(def.rB, copy.rB);
    EXPECT_EQ(def.mass, copy.mass);
}

TEST(WeldJointConf, EqualsOperator)
{
    EXPECT_TRUE(WeldJointConf() == WeldJointConf());
    {
        auto conf = WeldJointConf{};
        conf.localAnchorA = Length2{1.2_m, -3_m};
        EXPECT_TRUE(conf == conf);
        EXPECT_FALSE(WeldJointConf() == conf);
    }
    {
        auto conf = WeldJointConf{};
        conf.localAnchorB = Length2{1.2_m, -3_m};
        EXPECT_TRUE(conf == conf);
        EXPECT_FALSE(WeldJointConf() == conf);
    }
    {
        auto conf = WeldJointConf{};
        conf.referenceAngle = 12.4_deg;
        EXPECT_TRUE(conf == conf);
        EXPECT_FALSE(WeldJointConf() == conf);
    }
    // TODO: test remaining fields.
}

TEST(WeldJointConf, NotEqualsOperator)
{
    EXPECT_FALSE(WeldJointConf() != WeldJointConf());
    {
        auto conf = WeldJointConf{};
        conf.frequency = 13_Hz;
        EXPECT_FALSE(conf != conf);
        EXPECT_TRUE(WeldJointConf() != conf);
    }
    // TODO: test remaining fields.
}

TEST(WeldJointConf, GetName)
{
    EXPECT_STREQ(GetName(GetTypeID<WeldJointConf>()), "d2::WeldJointConf");
}

TEST(WeldJointConf, SetFrequencyFreeFunction)
{
    auto def = WeldJointConf{};
    const auto frequencyA = 67_Hz;
    const auto frequencyB = 2_Hz;
    def.frequency = frequencyA;
    auto joint = Joint(def);
    EXPECT_EQ(GetFrequency(joint), frequencyA);
    EXPECT_NO_THROW(SetFrequency(joint, frequencyB));
    EXPECT_EQ(GetFrequency(joint), frequencyB);
}

TEST(WeldJointConf, InitVelocity)
{
    auto conf = WeldJointConf{};
    std::vector<BodyConstraint> bodies;
    EXPECT_NO_THROW(InitVelocity(conf, bodies, StepConf{}, ConstraintSolverConf{}));
    conf.bodyA = BodyID(0);
    conf.bodyB = BodyID(0);
    EXPECT_THROW(InitVelocity(conf, bodies, StepConf{}, ConstraintSolverConf{}), std::out_of_range);
    const auto posA = Position{Length2{-5_m, 0_m}, 0_deg};
    bodies.push_back(BodyConstraint{Real(1) / 4_kg, InvRotInertia{}, Length2{}, posA, Velocity{}});
    EXPECT_NO_THROW(InitVelocity(conf, bodies, StepConf{}, ConstraintSolverConf{}));
}

TEST(WeldJointConf, SolveVelocity)
{
    auto conf = WeldJointConf{};
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

TEST(WeldJointConf, SolvePosition)
{
    auto conf = WeldJointConf{};
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
