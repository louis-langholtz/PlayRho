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

#include <PlayRho/d2/MotorJointConf.hpp>
#include <PlayRho/d2/Joint.hpp>

#include <PlayRho/Dynamics/BodyConf.hpp>
#include <PlayRho/Dynamics/World.hpp>
#include <PlayRho/Dynamics/WorldBody.hpp>
#include <PlayRho/Dynamics/WorldJoint.hpp>
#include <PlayRho/Dynamics/WorldMisc.hpp>
#include <PlayRho/Dynamics/WorldShape.hpp>
#include <PlayRho/Dynamics/StepConf.hpp>
#include <PlayRho/Collision/Shapes/DiskShapeConf.hpp>
#include <PlayRho/d2/BodyConstraint.hpp>
#include <PlayRho/ConstraintSolverConf.hpp>

using namespace playrho;
using namespace playrho::d2;

TEST(MotorJointConf, ByteSize)
{
    // Check size at test runtime instead of compile-time via static_assert to avoid stopping
    // builds and to report actual size rather than just reporting that expected size is wrong.
    switch (sizeof(Real)) {
    case 4:
        EXPECT_EQ(sizeof(MotorJointConf), std::size_t(96));
        break;
    case 8:
        EXPECT_EQ(sizeof(MotorJointConf), std::size_t(176));
        break;
    case 16:
        EXPECT_EQ(sizeof(MotorJointConf), std::size_t(352));
        break;
    default:
        FAIL();
        break;
    }
}

TEST(MotorJointConf, DefaultConstruction)
{
    MotorJointConf def{};

    EXPECT_EQ(def.bodyA, InvalidBodyID);
    EXPECT_EQ(def.bodyB, InvalidBodyID);
    EXPECT_EQ(def.collideConnected, false);

    EXPECT_EQ(def.linearOffset, (Length2{}));
    EXPECT_EQ(def.angularOffset, 0_deg);
    EXPECT_EQ(def.maxForce, MotorJointConf::DefaultMaxForce);
    EXPECT_EQ(def.maxTorque, MotorJointConf::DefaultMaxTorque);
    EXPECT_EQ(def.maxForce, 1_N);
    EXPECT_EQ(def.maxTorque, 1_Nm);
    EXPECT_EQ(def.correctionFactor, MotorJointConf::DefaultCorrectionFactor);
}

TEST(MotorJointConf, BuilderConstruction)
{
    const auto bodyA = static_cast<BodyID>(0x1);
    const auto bodyB = static_cast<BodyID>(0x2);
    const auto collideConnected = true;
    const auto linearOffset = Length2{2_m, 3_m};
    const auto angularOffset = 33_rad;
    const auto maxForce = 22_N;
    const auto maxTorque = 31_Nm;
    const auto correctionFactor = Real(0.44f);
    const auto def = MotorJointConf{}
                         .UseBodyA(bodyA)
                         .UseBodyB(bodyB)
                         .UseCollideConnected(collideConnected)
                         .UseLinearOffset(linearOffset)
                         .UseAngularOffset(angularOffset)
                         .UseMaxForce(maxForce)
                         .UseMaxTorque(maxTorque)
                         .UseCorrectionFactor(correctionFactor);

    EXPECT_EQ(def.bodyA, bodyA);
    EXPECT_EQ(def.bodyB, bodyB);
    EXPECT_EQ(def.collideConnected, collideConnected);

    EXPECT_EQ(def.linearOffset, linearOffset);
    EXPECT_EQ(def.angularOffset, angularOffset);
    EXPECT_EQ(def.maxForce, maxForce);
    EXPECT_EQ(def.maxTorque, maxTorque);
    EXPECT_EQ(def.correctionFactor, correctionFactor);
}

TEST(MotorJoint, Construction)
{
    auto world = World{};
    const auto b0 = CreateBody(world);
    const auto b1 = CreateBody(world);

    auto def = GetMotorJointConf(world, b0, b1);
    const auto jointID = CreateJoint(world, def);

    EXPECT_EQ(GetType(world, jointID), GetTypeID<MotorJointConf>());
    EXPECT_EQ(GetBodyA(world, jointID), def.bodyA);
    EXPECT_EQ(GetBodyB(world, jointID), def.bodyB);
    EXPECT_EQ(GetCollideConnected(world, jointID), def.collideConnected);
    EXPECT_EQ(GetLinearReaction(world, jointID), Momentum2{});
    EXPECT_EQ(GetAngularReaction(world, jointID), AngularMomentum{0});

    EXPECT_EQ(GetLinearOffset(world, jointID), def.linearOffset);
    EXPECT_EQ(GetAngularOffset(world, jointID), def.angularOffset);

    const auto conf = TypeCast<MotorJointConf>(GetJoint(world, jointID));
    EXPECT_EQ(GetMaxForce(conf), def.maxForce);
    EXPECT_EQ(GetMaxTorque(conf), def.maxTorque);
    EXPECT_EQ(GetCorrectionFactor(conf), def.correctionFactor);
    EXPECT_EQ(GetMaxForce(Joint{conf}), def.maxForce);
    EXPECT_EQ(GetMaxTorque(Joint{conf}), def.maxTorque);
}

TEST(MotorJoint, ShiftOrigin)
{
    auto world = World{};
    const auto b0 = CreateBody(world);
    const auto b1 = CreateBody(world);

    auto def = GetMotorJointConf(world, b0, b1);
    const auto joint = CreateJoint(world, def);
    const auto newOrigin = Length2{1_m, 1_m};
    EXPECT_FALSE(ShiftOrigin(world, joint, newOrigin));
}

TEST(MotorJoint, SetCorrectionFactor)
{
    auto world = World{};
    const auto b0 = CreateBody(world);
    const auto b1 = CreateBody(world);

    auto def = GetMotorJointConf(world, b0, b1);
    const auto jointID = CreateJoint(world, def);
    auto conf = TypeCast<MotorJointConf>(GetJoint(world, jointID));

    ASSERT_EQ(GetCorrectionFactor(conf), def.correctionFactor);
    ASSERT_EQ(Real(0.3), def.correctionFactor);

    EXPECT_NO_THROW(SetCorrectionFactor(conf, Real(0.9)));
    EXPECT_EQ(GetCorrectionFactor(conf), Real(0.9));

    EXPECT_NO_THROW(SetJoint(world, jointID, conf));
    auto conf2 = TypeCast<MotorJointConf>(GetJoint(world, jointID));
    EXPECT_EQ(GetCorrectionFactor(conf2), Real(0.9));
}

TEST(MotorJoint, GetMotorJointConfThrows)
{
    EXPECT_THROW(GetMotorJointConf(Joint{}), std::bad_cast);
}

TEST(MotorJoint, GetMotorJointConf)
{
    auto world = World{};
    const auto b0 = CreateBody(world);
    const auto b1 = CreateBody(world);

    auto def = GetMotorJointConf(world, b0, b1);
    const auto jointID = CreateJoint(world, def);

    ASSERT_EQ(GetType(world, jointID), GetTypeID<MotorJointConf>());
    ASSERT_EQ(GetBodyA(world, jointID), def.bodyA);
    ASSERT_EQ(GetBodyB(world, jointID), def.bodyB);
    ASSERT_EQ(GetCollideConnected(world, jointID), def.collideConnected);

    ASSERT_EQ(GetLinearOffset(world, jointID), def.linearOffset);
    ASSERT_EQ(GetAngularOffset(world, jointID), def.angularOffset);
    const auto conf = TypeCast<MotorJointConf>(GetJoint(world, jointID));
    ASSERT_EQ(GetMaxForce(conf), def.maxForce);
    ASSERT_EQ(GetMaxTorque(conf), def.maxTorque);
    ASSERT_EQ(GetCorrectionFactor(conf), def.correctionFactor);

    const auto cdef = GetMotorJointConf(GetJoint(world, jointID));
    EXPECT_EQ(cdef.bodyA, b0);
    EXPECT_EQ(cdef.bodyB, b1);
    EXPECT_EQ(cdef.collideConnected, false);

    EXPECT_EQ(cdef.linearOffset, (Length2{}));
    EXPECT_EQ(cdef.angularOffset, 0_deg);
    EXPECT_EQ(cdef.maxForce, 1_N);
    EXPECT_EQ(cdef.maxTorque, 1_Nm);
    EXPECT_EQ(cdef.correctionFactor, Real(0.3));
}

TEST(MotorJoint, WithDynamicCircles)
{
    auto world = World{};
    const auto p1 = Length2{-1_m, 0_m};
    const auto p2 = Length2{+1_m, 0_m};
    const auto b1 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p1));
    const auto b2 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p2));
    const auto circle = CreateShape(world, DiskShapeConf{}.UseRadius(0.2_m));
    ASSERT_NO_THROW(Attach(world, b1, circle));
    ASSERT_NO_THROW(Attach(world, b2, circle));
    // const auto anchor = Length2(2_m, 1_m);
    const auto jd = GetMotorJointConf(world, b1, b2);
    const auto joint = CreateJoint(world, jd);
    ASSERT_NE(joint, InvalidJointID);
    EXPECT_EQ(GetAnchorA(world, joint), p1);
    EXPECT_EQ(GetAnchorB(world, joint), p2);

    auto stepConf = StepConf{};

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

TEST(MotorJoint, SetLinearOffset)
{
    auto world = World{};
    const auto p1 = Length2{-1_m, 0_m};
    const auto p2 = Length2{+1_m, 0_m};
    const auto b1 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p1));
    const auto b2 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p2));
    const auto circle = CreateShape(world, DiskShapeConf{}.UseRadius(0.2_m));
    Attach(world, b1, circle);
    Attach(world, b2, circle);
    // const auto anchor = Length2(2_m, 1_m);
    const auto jd = GetMotorJointConf(world, b1, b2);
    const auto joint = CreateJoint(world, jd);
    ASSERT_NE(joint, InvalidJointID);
    EXPECT_EQ(GetAnchorA(world, joint), p1);
    EXPECT_EQ(GetAnchorB(world, joint), p2);

    const auto linearOffset = Length2{2_m, 1_m};
    ASSERT_EQ(GetLinearOffset(world, joint), jd.linearOffset);
    ASSERT_NE(jd.linearOffset, linearOffset);
    SetLinearOffset(world, joint, linearOffset);
    EXPECT_EQ(GetLinearOffset(world, joint), linearOffset);
}

TEST(MotorJoint, SetAngularOffset)
{
    auto world = World{};
    const auto p1 = Length2{-1_m, 0_m};
    const auto p2 = Length2{+1_m, 0_m};
    const auto b1 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p1));
    const auto b2 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p2));
    const auto circle = CreateShape(world, DiskShapeConf{}.UseRadius(0.2_m));
    Attach(world, b1, circle);
    Attach(world, b2, circle);
    // const auto anchor = Length2(2_m, 1_m);
    const auto jd = GetMotorJointConf(world, b1, b2);
    const auto joint = CreateJoint(world, jd);
    ASSERT_NE(joint, InvalidJointID);
    EXPECT_EQ(GetAnchorA(world, joint), p1);
    EXPECT_EQ(GetAnchorB(world, joint), p2);

    ASSERT_EQ(GetAngularOffset(world, joint), 0_deg);
    SetAngularOffset(world, joint, 45_deg);
    EXPECT_EQ(GetAngularOffset(world, joint), 45_deg);
}

TEST(MotorJointConf, GetAngularMass)
{
    auto conf = MotorJointConf{};
    conf.angularMass = RotInertia{2_m2 * 3_kg / SquareRadian}; // L^2 M QP^-2
    auto rotInertia = RotInertia{};
    EXPECT_NO_THROW(rotInertia = GetAngularMass(Joint{conf}));
    EXPECT_EQ(conf.angularMass, rotInertia);
}

TEST(MotorJointConf, EqualsOperator)
{
    EXPECT_TRUE(MotorJointConf() == MotorJointConf());
    {
        auto conf = MotorJointConf{};
        conf.linearOffset = Length2{1.2_m, -3_m};
        EXPECT_TRUE(conf == conf);
        EXPECT_FALSE(MotorJointConf() == conf);
    }
    {
        auto conf = MotorJointConf{};
        conf.angularOffset = 33_deg;
        EXPECT_TRUE(conf == conf);
        EXPECT_FALSE(MotorJointConf() == conf);
    }
    {
        auto conf = MotorJointConf{};
        conf.correctionFactor = Real(3.4);
        EXPECT_TRUE(conf == conf);
        EXPECT_FALSE(MotorJointConf() == conf);
    }
    {
        auto conf = MotorJointConf{};
        conf.angularError = 19_deg;
        EXPECT_TRUE(conf == conf);
        EXPECT_FALSE(MotorJointConf() == conf);
    }
    // TODO: test remaining fields.
}

TEST(MotorJointConf, NotEqualsOperator)
{
    EXPECT_FALSE(MotorJointConf() != MotorJointConf());
    {
        auto conf = MotorJointConf{};
        conf.maxForce = 2.5_N;
        EXPECT_FALSE(conf != conf);
        EXPECT_TRUE(MotorJointConf() != conf);
    }
    // TODO: test remaining fields.
}

TEST(MotorJointConf, GetName)
{
    EXPECT_STREQ(GetName(GetTypeID<MotorJointConf>()), "d2::MotorJointConf");
}

TEST(MotorJointConf, InitVelocity)
{
    auto conf = MotorJointConf{};
    std::vector<BodyConstraint> bodies;
    EXPECT_NO_THROW(InitVelocity(conf, bodies, StepConf{}, ConstraintSolverConf{}));
    conf.bodyA = BodyID(0);
    conf.bodyB = BodyID(0);
    EXPECT_THROW(InitVelocity(conf, bodies, StepConf{}, ConstraintSolverConf{}), std::out_of_range);
    const auto posA = Position{Length2{-5_m, 0_m}, 0_deg};
    bodies.push_back(BodyConstraint{Real(1) / 4_kg, InvRotInertia{}, Length2{}, posA, Velocity{}});
    EXPECT_NO_THROW(InitVelocity(conf, bodies, StepConf{}, ConstraintSolverConf{}));
}

TEST(MotorJointConf, SolveVelocity)
{
    auto conf = MotorJointConf{};
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

TEST(MotorJointConf, SolvePosition)
{
    auto conf = MotorJointConf{};
    std::vector<BodyConstraint> bodies;
    auto result = false;
    EXPECT_NO_THROW(result = SolvePosition(conf, bodies, ConstraintSolverConf{}));
    EXPECT_TRUE(result);
}
