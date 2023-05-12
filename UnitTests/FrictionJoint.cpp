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

#include <PlayRho/Dynamics/Joints/FrictionJointConf.hpp>
#include <PlayRho/Dynamics/Joints/Joint.hpp>

#include <PlayRho/Collision/Shapes/DiskShapeConf.hpp>
#include <PlayRho/Dynamics/World.hpp>
#include <PlayRho/Dynamics/WorldBody.hpp>
#include <PlayRho/Dynamics/WorldShape.hpp>
#include <PlayRho/Dynamics/WorldJoint.hpp>
#include <PlayRho/Dynamics/WorldMisc.hpp>
#include <PlayRho/Dynamics/BodyConf.hpp>
#include <PlayRho/Dynamics/StepConf.hpp>
#include <PlayRho/Dynamics/Contacts/BodyConstraint.hpp>
#include <PlayRho/Dynamics/Contacts/ConstraintSolverConf.hpp>

using namespace playrho;
using namespace playrho::d2;

TEST(FrictionJointConf, DefaultConstruction)
{
    FrictionJointConf def{};

    EXPECT_EQ(def.bodyA, InvalidBodyID);
    EXPECT_EQ(def.bodyB, InvalidBodyID);
    EXPECT_EQ(def.collideConnected, false);

    EXPECT_EQ(def.localAnchorA, (Length2{}));
    EXPECT_EQ(def.localAnchorB, (Length2{}));
    EXPECT_EQ(def.maxForce, 0_N);
    EXPECT_EQ(def.maxTorque, 0_Nm);
}

TEST(FrictionJointConf, InitializingConstructor)
{
    const auto laA = Length2{-1_m, 0_m};
    const auto laB = Length2{+1_m, 0_m};
    const auto bA = BodyID(0);
    const auto bB = BodyID(1);
    const auto def = FrictionJointConf(bA, bB, laA, laB);
    EXPECT_EQ(def.bodyA, bA);
    EXPECT_EQ(def.bodyB, bB);
    EXPECT_EQ(def.localAnchorA, laA);
    EXPECT_EQ(def.localAnchorB, laB);
    EXPECT_EQ(def.maxForce, NonNegative<Force>());
    EXPECT_EQ(def.maxTorque, NonNegative<Torque>());
    EXPECT_EQ(def.linearImpulse, Momentum2());
    EXPECT_EQ(def.angularImpulse, AngularMomentum());
    EXPECT_EQ(def.rA, Length2());
    EXPECT_EQ(def.rB, Length2());
    EXPECT_EQ(def.linearMass, Mass22());
    EXPECT_EQ(def.angularMass, RotInertia());
}

TEST(FrictionJointConf, GetFrictionJointConf)
{
    World world{};
    const auto p1 = Length2{-1_m, 0_m};
    const auto p2 = Length2{+1_m, 0_m};
    const auto b1 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p1));
    const auto b2 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p2));
    const auto anchor = Length2{0_m, 0_m};
    const auto def = GetFrictionJointConf(world, b1, b2, anchor);
    EXPECT_EQ(def.bodyA, b1);
    EXPECT_EQ(def.bodyB, b2);
    EXPECT_EQ(def.localAnchorA, GetLocalPoint(world, b1, anchor));
    EXPECT_EQ(def.localAnchorB, GetLocalPoint(world, b2, anchor));
}

TEST(FrictionJoint, Construction)
{
    auto world = World{};
    const auto b0 = CreateBody(world);
    const auto b1 = CreateBody(world);

    auto def = GetFrictionJointConf(world, b0, b1, Length2{});
    const auto joint = Joint{def};

    EXPECT_EQ(GetType(joint), GetTypeID<FrictionJointConf>());
    EXPECT_EQ(GetBodyA(joint), def.bodyA);
    EXPECT_EQ(GetBodyB(joint), def.bodyB);
    EXPECT_EQ(GetCollideConnected(joint), def.collideConnected);
    EXPECT_EQ(GetLinearReaction(joint), Momentum2{});
    EXPECT_EQ(GetAngularReaction(joint), AngularMomentum{0});

    EXPECT_EQ(GetLocalAnchorA(joint), def.localAnchorA);
    EXPECT_EQ(GetLocalAnchorB(joint), def.localAnchorB);
    EXPECT_EQ(GetMaxForce(joint), def.maxForce);
    EXPECT_EQ(GetMaxTorque(joint), def.maxTorque);
}

TEST(FrictionJoint, GetFrictionJointConfThrows)
{
    EXPECT_THROW(GetFrictionJointConf(Joint{}), std::bad_cast);
}

TEST(FrictionJoint, GetFrictionJointConf)
{
    auto world = World{};
    const auto b0 = CreateBody(world);
    const auto b1 = CreateBody(world);

    auto def = GetFrictionJointConf(world, b0, b1, Length2{});
    const auto joint = Joint{def};

    ASSERT_EQ(GetType(joint), GetTypeID<FrictionJointConf>());
    ASSERT_EQ(GetBodyA(joint), def.bodyA);
    ASSERT_EQ(GetBodyB(joint), def.bodyB);
    ASSERT_EQ(GetCollideConnected(joint), def.collideConnected);

    ASSERT_EQ(GetLocalAnchorA(joint), def.localAnchorA);
    ASSERT_EQ(GetLocalAnchorB(joint), def.localAnchorB);
    ASSERT_EQ(GetMaxForce(joint), def.maxForce);
    ASSERT_EQ(GetMaxTorque(joint), def.maxTorque);

    const auto cdef = GetFrictionJointConf(joint);
    EXPECT_EQ(cdef.bodyA, b0);
    EXPECT_EQ(cdef.bodyB, b1);
    EXPECT_EQ(cdef.collideConnected, false);

    EXPECT_EQ(cdef.localAnchorA, (Length2{}));
    EXPECT_EQ(cdef.localAnchorB, (Length2{}));
    EXPECT_EQ(cdef.maxForce, 0_N);
    EXPECT_EQ(cdef.maxTorque, 0_Nm);
}

TEST(FrictionJoint, WithDynamicCircles)
{
    World world{};
    const auto p1 = Length2{-1_m, 0_m};
    const auto p2 = Length2{+1_m, 0_m};
    const auto s0 = CreateShape(world, DiskShapeConf{}.UseRadius(0.2_m));
    const auto b1 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p1));
    const auto b2 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p2));
    Attach(world, b1, s0);
    Attach(world, b2, s0);
    auto jd = FrictionJointConf{};
    jd.bodyA = b1;
    jd.bodyB = b2;
    ASSERT_NE(CreateJoint(world, Joint{jd}), InvalidJointID);
    auto stepConf = StepConf{};

    stepConf.doWarmStart = true;
    Step(world, stepConf);
    EXPECT_NEAR(double(Real{GetX(GetLocation(world, b1)) / 1_m}), -1.0, 0.001);
    EXPECT_NEAR(double(Real{GetY(GetLocation(world, b1)) / 1_m}), 0.0, 0.001);
    EXPECT_NEAR(double(Real{GetX(GetLocation(world, b2)) / 1_m}), +1.0, 0.01);
    EXPECT_NEAR(double(Real{GetY(GetLocation(world, b2)) / 1_m}), 0.0, 0.01);
    EXPECT_EQ(GetAngle(world, b1), 0_deg);
    EXPECT_EQ(GetAngle(world, b2), 0_deg);

    stepConf.doWarmStart = false;
    Step(world, stepConf);
    EXPECT_NEAR(double(Real{GetX(GetLocation(world, b1)) / 1_m}), -1.0, 0.001);
    EXPECT_NEAR(double(Real{GetY(GetLocation(world, b1)) / 1_m}), 0.0, 0.001);
    EXPECT_NEAR(double(Real{GetX(GetLocation(world, b2)) / 1_m}), +1.0, 0.01);
    EXPECT_NEAR(double(Real{GetY(GetLocation(world, b2)) / 1_m}), 0.0, 0.01);
    EXPECT_EQ(GetAngle(world, b1), 0_deg);
    EXPECT_EQ(GetAngle(world, b2), 0_deg);
}

TEST(FrictionJointConf, ShiftOrigin)
{
    auto def = FrictionJointConf{};
    def.bodyA = BodyID(1u);
    def.bodyB = BodyID(2u);
    def.localAnchorA = Length2{-2_m, +3_m};
    def.localAnchorB = Length2{+2_m, -3_m};
    def.maxForce = 2_N;
    def.maxTorque = 3_Nm;
    def.linearImpulse = Momentum2{1_Ns, 2_Ns};
    def.angularImpulse = AngularMomentum{};
    def.rA = Length2{3_m, 22_m};
    def.rB = Length2{2_m, 22_m};
    def.linearMass = Mass22{Vector2<Mass>{1_kg, 2_kg}, Vector2<Mass>{3_kg, 4_kg}};
    def.angularMass = RotInertia{};

    const auto copy = def;
    const auto amount = Length2{1_m, 2_m};
    EXPECT_FALSE(ShiftOrigin(def, amount));
    EXPECT_EQ(def.bodyA, copy.bodyA);
    EXPECT_EQ(def.bodyB, copy.bodyB);
    EXPECT_EQ(def.collideConnected, copy.collideConnected);
    EXPECT_EQ(def.localAnchorA, copy.localAnchorA);
    EXPECT_EQ(def.localAnchorB, copy.localAnchorB);
    EXPECT_EQ(def.maxForce, copy.maxForce);
    EXPECT_EQ(def.maxTorque, copy.maxTorque);
    EXPECT_EQ(def.linearImpulse, copy.linearImpulse);
    EXPECT_EQ(def.angularImpulse, copy.angularImpulse);
    EXPECT_EQ(def.rA, copy.rA);
    EXPECT_EQ(def.rB, copy.rB);
    EXPECT_EQ(def.linearMass, copy.linearMass);
    EXPECT_EQ(def.angularMass, copy.angularMass);
}

TEST(FrictionJointConf, GetMotorSpeedThrows)
{
    const auto joint = Joint{FrictionJointConf{}};
    EXPECT_THROW(GetMotorSpeed(joint), std::invalid_argument);
}

TEST(FrictionJointConf, SetMotorSpeedThrows)
{
    auto joint = Joint{FrictionJointConf{}};
    EXPECT_THROW(SetMotorSpeed(joint, 1_rpm), std::invalid_argument);
}

TEST(FrictionJointConf, GetAngularMass)
{
    auto conf = FrictionJointConf{};
    conf.angularMass = RotInertia{2_m2 * 3_kg / SquareRadian}; // L^2 M QP^-2
    auto rotInertia = RotInertia{};
    EXPECT_NO_THROW(rotInertia = GetAngularMass(Joint{conf}));
    EXPECT_EQ(conf.angularMass, rotInertia);
}

TEST(FrictionJointConf, EqualsOperator)
{
    EXPECT_TRUE(FrictionJointConf() == FrictionJointConf());
    {
        auto conf = FrictionJointConf{};
        conf.localAnchorA = Length2{1.2_m, -3_m};
        EXPECT_TRUE(conf == conf);
        EXPECT_FALSE(FrictionJointConf() == conf);
    }
    {
        auto conf = FrictionJointConf{};
        conf.localAnchorB = Length2{1.2_m, -3_m};
        EXPECT_TRUE(conf == conf);
        EXPECT_FALSE(FrictionJointConf() == conf);
    }
    {
        auto conf = FrictionJointConf{};
        conf.maxForce = 2.4_N;
        EXPECT_TRUE(conf == conf);
        EXPECT_FALSE(FrictionJointConf() == conf);
    }
    {
        auto conf = FrictionJointConf{};
        conf.maxTorque = 1.5_Nm;
        EXPECT_TRUE(conf == conf);
        EXPECT_FALSE(FrictionJointConf() == conf);
    }
    // TODO: test remaining fields.
}

TEST(FrictionJointConf, NotEqualsOperator)
{
    EXPECT_FALSE(FrictionJointConf() != FrictionJointConf());
    {
        auto conf = FrictionJointConf{};
        conf.rB = Length2{-1_m, 0.4_m};
        EXPECT_FALSE(conf != conf);
        EXPECT_TRUE(FrictionJointConf() != conf);
    }
    // TODO: test remaining fields.
}

TEST(FrictionJointConf, GetName)
{
    EXPECT_STREQ(GetName(GetTypeID<FrictionJointConf>()), "d2::FrictionJointConf");
}

TEST(FrictionJointConf, InitVelocity)
{
    auto conf = FrictionJointConf{};
    std::vector<BodyConstraint> bodies;
    EXPECT_NO_THROW(InitVelocity(conf, bodies, StepConf{}, ConstraintSolverConf{}));
    conf.bodyA = BodyID(0);
    conf.bodyB = BodyID(0);
    EXPECT_THROW(InitVelocity(conf, bodies, StepConf{}, ConstraintSolverConf{}), std::out_of_range);
    const auto posA = Position{Length2{-5_m, 0_m}, 0_deg};
    bodies.push_back(BodyConstraint{Real(1) / 4_kg, InvRotInertia{}, Length2{}, posA, Velocity{}});
    EXPECT_NO_THROW(InitVelocity(conf, bodies, StepConf{}, ConstraintSolverConf{}));
}

TEST(FrictionJointConf, SolveVelocity)
{
    auto conf = FrictionJointConf{};
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

TEST(FrictionJointConf, SolvePosition)
{
    auto conf = FrictionJointConf{};
    std::vector<BodyConstraint> bodies;
    auto result = false;
    EXPECT_NO_THROW(result = SolvePosition(conf, bodies, ConstraintSolverConf{}));
    EXPECT_TRUE(result);
}
