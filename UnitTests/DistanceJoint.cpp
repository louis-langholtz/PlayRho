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

#include <playrho/d2/DistanceJointConf.hpp>
#include <playrho/d2/Joint.hpp>
#include <playrho/d2/World.hpp>
#include <playrho/d2/WorldJoint.hpp>
#include <playrho/d2/WorldBody.hpp>
#include <playrho/d2/WorldMisc.hpp>
#include <playrho/d2/WorldShape.hpp>
#include <playrho/StepConf.hpp>
#include <playrho/d2/BodyConf.hpp>
#include <playrho/d2/DiskShapeConf.hpp>
#include <playrho/d2/BodyConstraint.hpp>
#include <playrho/ConstraintSolverConf.hpp>

#include <stdexcept> // for std::invalid_argument

using namespace playrho;
using namespace playrho::d2;

TEST(DistanceJointConf, ByteSize)
{
    // Check size at test runtime instead of compile-time via static_assert to avoid stopping
    // builds and to report actual size rather than just reporting that expected size is wrong.
    switch (sizeof(Real)) {
    case 4:
        EXPECT_EQ(sizeof(DistanceJointConf), std::size_t(80));
        break;
    case 8:
        EXPECT_EQ(sizeof(DistanceJointConf), std::size_t(144));
        break;
    case 16:
        EXPECT_EQ(sizeof(DistanceJointConf), std::size_t(288));
        break;
    default:
        FAIL();
        break;
    }
}

TEST(DistanceJointConf, Traits)
{
    EXPECT_TRUE(std::is_default_constructible_v<DistanceJointConf>);
    EXPECT_TRUE(std::is_copy_constructible_v<DistanceJointConf>);

    EXPECT_TRUE(std::is_nothrow_default_constructible_v<DistanceJointConf>);
#ifndef PLAYRHO_USE_BOOST_UNITS
    EXPECT_TRUE(std::is_nothrow_copy_constructible_v<DistanceJointConf>);
#endif
}

TEST(DistanceJointConf, DefaultConstruction)
{
    auto def = DistanceJointConf{};

    EXPECT_EQ(def.bodyA, InvalidBodyID);
    EXPECT_EQ(def.bodyB, InvalidBodyID);
    EXPECT_EQ(def.collideConnected, false);

    EXPECT_EQ(def.localAnchorA, (Length2{}));
    EXPECT_EQ(def.localAnchorB, (Length2{}));
    EXPECT_EQ(def.length, 1_m);
    EXPECT_EQ(def.frequency, 0_Hz);
    EXPECT_EQ(def.dampingRatio, Real(0));
}

TEST(DistanceJointConf, UseLength)
{
    const auto value = Length(31_m);
    EXPECT_NE(DistanceJointConf{}.length, value);
    EXPECT_EQ(DistanceJointConf{}.UseLength(value).length, value);
}

TEST(DistanceJointConf, UseFrequency)
{
    const auto value = 19_Hz;
    EXPECT_NE(DistanceJointConf{}.frequency, value);
    EXPECT_EQ(DistanceJointConf{}.UseFrequency(value).frequency, value);
}

TEST(DistanceJointConf, UseDampingRatio)
{
    const auto value = Real(0.4);
    EXPECT_NE(DistanceJointConf{}.dampingRatio, value);
    EXPECT_EQ(DistanceJointConf{}.UseDampingRatio(value).dampingRatio, value);
}

TEST(DistanceJoint, TypeCast)
{
    const auto joint = Joint{DistanceJointConf{}};
    EXPECT_THROW(TypeCast<int>(joint), std::bad_cast);
    EXPECT_NO_THROW(TypeCast<DistanceJointConf>(joint));
}

TEST(DistanceJoint, Construction)
{
    auto world = World{};
    const auto body0 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic));
    const auto body1 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic));
    auto def = DistanceJointConf{body0, body1};
    const auto joint = CreateJoint(world, Joint{def});

    EXPECT_EQ(GetType(world, joint), GetTypeID<DistanceJointConf>());
    EXPECT_EQ(GetBodyA(world, joint), def.bodyA);
    EXPECT_EQ(GetBodyB(world, joint), def.bodyB);
    EXPECT_EQ(GetCollideConnected(world, joint), def.collideConnected);
    EXPECT_EQ(GetLinearReaction(world, joint), Momentum2{});
    EXPECT_EQ(GetAngularReaction(world, joint), AngularMomentum{0});

    EXPECT_EQ(GetLocalAnchorA(world, joint), def.localAnchorA);
    EXPECT_EQ(GetLocalAnchorB(world, joint), def.localAnchorB);

    EXPECT_EQ(GetFrequency(world, joint), def.frequency);
    EXPECT_EQ(GetLength(world, joint), def.length);
    EXPECT_EQ(GetDampingRatio(world, joint), def.dampingRatio);
}

TEST(DistanceJoint, ShiftOrigin)
{
    auto world = World{};
    const auto body0 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic));
    const auto body1 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic));
    auto def = DistanceJointConf{body0, body1};
    const auto joint = CreateJoint(world, def);
    const auto newOrigin = Length2{1_m, 1_m};
    EXPECT_FALSE(ShiftOrigin(world, joint, newOrigin));
}

TEST(DistanceJoint, InZeroGravBodiesMoveOutToLength)
{
    auto world = World{};

    const auto shape = CreateShape(world, DiskShapeConf{}.UseRadius(0.2_m));

    const auto location1 = Length2{-1_m, 0_m};
    const auto body1 =
        CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic).UseLocation(location1));
    ASSERT_EQ(GetLocation(world, body1), location1);
    ASSERT_NO_THROW(Attach(world, body1, shape));

    const auto location2 = Length2{+1_m, 0_m};
    const auto body2 =
        CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic).UseLocation(location2));
    ASSERT_EQ(GetLocation(world, body2), location2);
    ASSERT_NO_THROW(Attach(world, body2, shape));

    auto jointdef = DistanceJointConf{};
    jointdef.bodyA = body1;
    jointdef.bodyB = body2;
    jointdef.collideConnected = false;
    jointdef.localAnchorA = Length2{};
    jointdef.localAnchorB = Length2{};
    jointdef.length = 5_m;
    jointdef.frequency = 0_Hz;
    jointdef.dampingRatio = 0;
    EXPECT_NE(CreateJoint(world, Joint{jointdef}), InvalidJointID);

    auto oldDistance = GetMagnitude(GetLocation(world, body1) - GetLocation(world, body2));

    auto distanceMet = 0u;
    auto stepConf = StepConf{};
    for (auto i = 0u; !distanceMet || i < distanceMet + 100; ++i) {
        Step(world, stepConf);

        const auto newDistance =
            GetMagnitude(GetLocation(world, body1) - GetLocation(world, body2));
        if (distanceMet) {
            EXPECT_NEAR(double(Real{newDistance / Meter}), double(Real{oldDistance / Meter}), 0.01);
        }
        else {
            EXPECT_GE(newDistance, oldDistance);
        }

        if (!distanceMet && (abs(newDistance - jointdef.length) < 0.01_m)) {
            distanceMet = i;
        }
        oldDistance = newDistance;
    }
}

TEST(DistanceJoint, InZeroGravBodiesMoveInToLength)
{
    auto world = World{};

    const auto shape = CreateShape(world, DiskShapeConf{}.UseRadius(0.2_m).UseDensity(1_kgpm2));
    const auto location1 = Length2{-10_m, 10_m};
    const auto body1 =
        CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic).UseLocation(location1));
    ASSERT_EQ(GetLocation(world, body1), location1);
    ASSERT_NO_THROW(Attach(world, body1, shape));

    const auto location2 = Length2{+10_m, -10_m};
    const auto body2 =
        CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic).UseLocation(location2));
    ASSERT_EQ(GetLocation(world, body2), location2);
    ASSERT_NO_THROW(Attach(world, body2, shape));

    auto jointdef = DistanceJointConf{};
    jointdef.bodyA = body1;
    jointdef.bodyB = body2;
    jointdef.collideConnected = false;
    jointdef.localAnchorA = Length2{};
    jointdef.localAnchorB = Length2{};
    jointdef.length = 5_m;
    jointdef.frequency = 60_Hz;
    jointdef.dampingRatio = 0;
    EXPECT_NE(CreateJoint(world, Joint{jointdef}), InvalidJointID);

    auto oldDistance = GetMagnitude(GetLocation(world, body1) - GetLocation(world, body2));

    auto distanceMet = 0u;
    auto stepConf = StepConf{};
    SetAccelerations(world, Acceleration{LinearAcceleration2{0, 10 * MeterPerSquareSecond},
                                         0 * RadianPerSquareSecond});
    for (auto i = 0u; !distanceMet || i < distanceMet + 1000; ++i) {
        Step(world, stepConf);

        const auto newDistance =
            GetMagnitude(GetLocation(world, body1) - GetLocation(world, body2));
        if (!distanceMet && (newDistance - oldDistance) >= 0_m) {
            distanceMet = i;
        }
        if (distanceMet) {
            EXPECT_NEAR(double(Real{newDistance / Meter}), double(Real{oldDistance / Meter}), 2.5);
        }
        else {
            EXPECT_LE(newDistance, oldDistance);
        }

        oldDistance = newDistance;
    }

    EXPECT_NEAR(double(Real{oldDistance / Meter}), double(Real{jointdef.length / Meter}), 0.1);
}

TEST(DistanceJointConf, GetDistanceJointConfThrows)
{
    EXPECT_THROW(GetDistanceJointConf(Joint{}), std::bad_cast);
}

TEST(DistanceJointConf, GetDistanceJointDefFreeFunction)
{
    auto world = World{};
    const auto bA = CreateBody(world);
    ASSERT_NE(bA, InvalidBodyID);
    const auto bB = CreateBody(world);
    ASSERT_NE(bB, InvalidBodyID);

    auto def = DistanceJointConf{};
    def.bodyA = bA;
    def.bodyB = bB;
    def.collideConnected = false;
    def.localAnchorA = Length2(21_m, -2_m);
    def.localAnchorB = Length2(13_m, 12_m);
    def.length = 5_m;
    def.frequency = 67_Hz;
    def.dampingRatio = Real(0.8);

    const auto joint = CreateJoint(world, def);
    const auto got = GetDistanceJointConf(GetJoint(world, joint));

    EXPECT_EQ(def.bodyA, got.bodyA);
    EXPECT_EQ(def.bodyB, got.bodyB);
    EXPECT_EQ(def.localAnchorA, got.localAnchorA);
    EXPECT_EQ(def.localAnchorB, got.localAnchorB);
    EXPECT_EQ(def.length, got.length);
    EXPECT_EQ(def.frequency, got.frequency);
    EXPECT_EQ(def.dampingRatio, got.dampingRatio);
}

TEST(DistanceJointConf, GetReferenceAngleThrows)
{
    auto world = World{};
    const auto bA = CreateBody(world);
    ASSERT_NE(bA, InvalidBodyID);
    const auto bB = CreateBody(world);
    ASSERT_NE(bB, InvalidBodyID);
    auto def = DistanceJointConf{};
    def.bodyA = bA;
    def.bodyB = bB;
    def.collideConnected = false;
    def.localAnchorA = Length2(21_m, -2_m);
    def.localAnchorB = Length2(13_m, 12_m);
    def.length = 5_m;
    def.frequency = 67_Hz;
    def.dampingRatio = Real(0.8);
    const auto joint = CreateJoint(world, def);
    EXPECT_THROW(GetReferenceAngle(GetJoint(world, joint)), std::invalid_argument);
}

TEST(DistanceJointConf, GetMotorSpeedThrows)
{
    const auto joint = Joint{DistanceJointConf{}};
    EXPECT_THROW(GetMotorSpeed(joint), std::invalid_argument);
}

TEST(DistanceJointConf, SetMotorSpeedThrows)
{
    auto joint = Joint{DistanceJointConf{}};
    EXPECT_THROW(SetMotorSpeed(joint, 1_rpm), std::invalid_argument);
}

TEST(DistanceJointConf, SetFrequencyFreeFunction)
{
    auto def = DistanceJointConf{};
    def.collideConnected = false;
    def.localAnchorA = Length2(21_m, -2_m);
    def.localAnchorB = Length2(13_m, 12_m);
    def.length = 5_m;
    def.frequency = 67_Hz;
    def.dampingRatio = Real(0.8);
    auto joint = Joint(def);
    EXPECT_EQ(GetFrequency(joint), 67_Hz);
    EXPECT_NO_THROW(SetFrequency(joint, 2_Hz));
    EXPECT_EQ(GetFrequency(joint), 2_Hz);
}

TEST(DistanceJointConf, EqualsOperator)
{
    EXPECT_TRUE(DistanceJointConf() == DistanceJointConf());
    {
        auto conf = DistanceJointConf{};
        conf.localAnchorA = Length2{1.2_m, -3_m};
        EXPECT_TRUE(conf == conf);
        EXPECT_FALSE(DistanceJointConf() == conf);
    }
    {
        auto conf = DistanceJointConf{};
        conf.localAnchorB = Length2{1.2_m, -3_m};
        EXPECT_TRUE(conf == conf);
        EXPECT_FALSE(DistanceJointConf() == conf);
    }
    {
        auto conf = DistanceJointConf{};
        conf.length = 2.4_m;
        EXPECT_TRUE(conf == conf);
        EXPECT_FALSE(DistanceJointConf() == conf);
    }
    {
        auto conf = DistanceJointConf{};
        conf.bias = 1.5_mps;
        EXPECT_TRUE(conf == conf);
        EXPECT_FALSE(DistanceJointConf() == conf);
    }
    // TODO: test remaining fields.
}

TEST(DistanceJointConf, NotEqualsOperator)
{
    EXPECT_FALSE(DistanceJointConf() != DistanceJointConf());
    {
        auto conf = DistanceJointConf{};
        conf.dampingRatio = Real(2.3);
        EXPECT_FALSE(conf != conf);
        EXPECT_TRUE(DistanceJointConf() != conf);
    }
    // TODO: test remaining fields.
}

TEST(DistanceJointConf, GetName)
{
    EXPECT_STREQ(GetName(GetTypeID<DistanceJointConf>()), "d2::DistanceJointConf");
}

TEST(DistanceJointConf, InitVelocity)
{
    auto conf = DistanceJointConf{};
    std::vector<BodyConstraint> bodies;
    EXPECT_NO_THROW(InitVelocity(conf, bodies, StepConf{}, ConstraintSolverConf{}));
    conf.bodyA = BodyID(0);
    conf.bodyB = BodyID(0);
    EXPECT_THROW(InitVelocity(conf, bodies, StepConf{}, ConstraintSolverConf{}), std::out_of_range);
    const auto posA = Position{Length2{-5_m, 0_m}, 0_deg};
    bodies.push_back(BodyConstraint{Real(1) / 4_kg, InvRotInertia{}, Length2{}, posA, Velocity{}});
    EXPECT_NO_THROW(InitVelocity(conf, bodies, StepConf{}, ConstraintSolverConf{}));
}

TEST(DistanceJointConf, SolveVelocity)
{
    auto conf = DistanceJointConf{};
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

TEST(DistanceJointConf, SolvePosition)
{
    std::vector<BodyConstraint> bodies;
    auto conf = DistanceJointConf{};
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
