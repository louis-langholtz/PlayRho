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

#include <PlayRho/Dynamics/Joints/WheelJoint.hpp>

#include <PlayRho/Dynamics/World.hpp>
#include <PlayRho/Dynamics/Joints/TypeJointVisitor.hpp>
#include <PlayRho/Dynamics/BodyConf.hpp>
#include <PlayRho/Dynamics/WorldBody.hpp>
#include <PlayRho/Dynamics/WorldJoint.hpp>
#include <PlayRho/Dynamics/StepConf.hpp>
#include <PlayRho/Collision/Shapes/DiskShapeConf.hpp>

using namespace playrho;
using namespace playrho::d2;

TEST(WheelJointConf, ByteSize)
{
    switch (sizeof(Real))
    {
        case  4:
#if defined(_WIN32) && !defined(_WIN64)
            EXPECT_EQ(sizeof(WheelJointConf), std::size_t(64));
#else
            EXPECT_EQ(sizeof(WheelJointConf), std::size_t(64));
#endif
            break;
        case  8: EXPECT_EQ(sizeof(WheelJointConf), std::size_t(128)); break;
        case 16: EXPECT_EQ(sizeof(WheelJointConf), std::size_t(224)); break;
        default: FAIL(); break;
    }
}

TEST(WheelJointConf, DefaultConstruction)
{
    WheelJointConf def{};
    
    EXPECT_EQ(def.type, JointType::Wheel);
    EXPECT_EQ(def.bodyA, InvalidBodyID);
    EXPECT_EQ(def.bodyB, InvalidBodyID);
    EXPECT_EQ(def.collideConnected, false);
    EXPECT_EQ(def.userData, nullptr);
    
    EXPECT_EQ(def.localAnchorA, (Length2{}));
    EXPECT_EQ(def.localAnchorB, (Length2{}));
    EXPECT_EQ(def.localAxisA, UnitVec::GetRight());
    EXPECT_FALSE(def.enableMotor);
    EXPECT_EQ(def.maxMotorTorque, Torque(0));
    EXPECT_EQ(def.motorSpeed, 0_rpm);
    EXPECT_EQ(def.frequency, 2_Hz);
    EXPECT_EQ(def.dampingRatio, Real(0.7f));
}

TEST(WheelJoint, Traits)
{
    EXPECT_FALSE((IsIterable<WheelJoint>::value));
    EXPECT_FALSE((IsAddable<WheelJoint>::value));
    EXPECT_FALSE((IsAddable<WheelJoint,WheelJoint>::value));
}

TEST(WheelJoint, ByteSize)
{
    switch (sizeof(Real))
    {
        case  4:
#if defined(_WIN64)
            EXPECT_EQ(sizeof(WheelJoint), std::size_t(160));
#elif defined(_WIN32)
            EXPECT_EQ(sizeof(WheelJoint), std::size_t(136));
#else
            EXPECT_EQ(sizeof(WheelJoint), std::size_t(144));
#endif
            break;
        case  8: EXPECT_EQ(sizeof(WheelJoint), std::size_t(272)); break;
        case 16: EXPECT_EQ(sizeof(WheelJoint), std::size_t(512)); break;
        default: FAIL(); break;
    }
}

TEST(WheelJoint, Construction)
{
    WheelJointConf def;
    WheelJoint joint{def};
    
    EXPECT_EQ(GetType(joint), def.type);
    EXPECT_EQ(joint.GetBodyA(), def.bodyA);
    EXPECT_EQ(joint.GetBodyB(), def.bodyB);
    EXPECT_EQ(joint.GetCollideConnected(), def.collideConnected);
    EXPECT_EQ(joint.GetUserData(), def.userData);
    EXPECT_EQ(joint.GetLinearReaction(), Momentum2{});
    EXPECT_EQ(joint.GetAngularReaction(), AngularMomentum{0});

    EXPECT_EQ(joint.GetLocalAnchorA(), def.localAnchorA);
    EXPECT_EQ(joint.GetLocalAnchorB(), def.localAnchorB);
    EXPECT_EQ(joint.GetLocalAxisA(), def.localAxisA);
    EXPECT_EQ(joint.IsMotorEnabled(), def.enableMotor);
    EXPECT_EQ(joint.GetMaxMotorTorque(), def.maxMotorTorque);
    EXPECT_EQ(joint.GetMotorSpeed(), def.motorSpeed);
    EXPECT_EQ(joint.GetFrequency(), def.frequency);
    EXPECT_EQ(joint.GetSpringDampingRatio(), def.dampingRatio);
    
    TypeJointVisitor visitor;
    joint.Accept(visitor);
    EXPECT_EQ(visitor.GetType().value(), JointType::Wheel);
    
    EXPECT_EQ(GetMotorTorque(joint, 1_Hz), 0 * NewtonMeter);
}

TEST(WheelJoint, EnableMotor)
{
    World world;
    const auto b0 = world.CreateBody();
    const auto b1 = world.CreateBody();
    
    auto jd = WheelJointConf{};
    jd.bodyA = b0;
    jd.bodyB = b1;
    jd.localAnchorA = Length2(4_m, 5_m);
    jd.localAnchorB = Length2(6_m, 7_m);
    
    auto joint = WheelJoint{jd};
    EXPECT_FALSE(joint.IsMotorEnabled());
    joint.EnableMotor(false);
    EXPECT_FALSE(joint.IsMotorEnabled());
    joint.EnableMotor(true);
    EXPECT_TRUE(joint.IsMotorEnabled());
}

TEST(WheelJoint, MotorSpeed)
{
    World world;
    const auto b0 = world.CreateBody();
    const auto b1 = world.CreateBody();
    
    auto jd = WheelJointConf{};
    jd.bodyA = b0;
    jd.bodyB = b1;
    jd.localAnchorA = Length2(4_m, 5_m);
    jd.localAnchorB = Length2(6_m, 7_m);
    
    const auto newValue = 5_rad / 1_s;
    auto joint = WheelJoint{jd};
    ASSERT_NE(joint.GetMotorSpeed(), newValue);
    EXPECT_EQ(joint.GetMotorSpeed(), jd.motorSpeed);
    joint.SetMotorSpeed(newValue);
    EXPECT_EQ(joint.GetMotorSpeed(), newValue);
}

TEST(WheelJoint, MaxMotorTorque)
{
    World world;
    const auto b0 = world.CreateBody();
    const auto b1 = world.CreateBody();
    
    auto jd = WheelJointConf{};
    jd.bodyA = b0;
    jd.bodyB = b1;
    jd.localAnchorA = Length2(4_m, 5_m);
    jd.localAnchorB = Length2(6_m, 7_m);
    
    const auto newValue = 5_Nm;
    auto joint = WheelJoint{jd};
    ASSERT_NE(joint.GetMaxMotorTorque(), newValue);
    EXPECT_EQ(joint.GetMaxMotorTorque(), jd.maxMotorTorque);
    joint.SetMaxMotorTorque(newValue);
    EXPECT_EQ(joint.GetMaxMotorTorque(), newValue);
}

TEST(WheelJoint, GetAnchorAandB)
{
    World world;
    
    const auto loc0 = Length2{+1_m, -3_m};
    const auto loc1 = Length2{-2_m, Real(+1.2f) * Meter};
    
    const auto b0 = world.CreateBody(BodyConf{}.UseLocation(loc0));
    const auto b1 = world.CreateBody(BodyConf{}.UseLocation(loc1));
    
    auto jd = WheelJointConf{};
    jd.bodyA = b0;
    jd.bodyB = b1;
    jd.localAnchorA = Length2(4_m, 5_m);
    jd.localAnchorB = Length2(6_m, 7_m);
    
    auto joint = world.CreateJoint(jd);
    ASSERT_EQ(GetLocalAnchorA(world, joint), jd.localAnchorA);
    ASSERT_EQ(GetLocalAnchorB(world, joint), jd.localAnchorB);
    EXPECT_EQ(GetAnchorA(world, joint), loc0 + jd.localAnchorA);
    EXPECT_EQ(GetAnchorB(world, joint), loc1 + jd.localAnchorB);
}

TEST(WheelJoint, GetJointTranslation)
{
    World world;
    
    const auto loc0 = Length2{+1_m, -3_m};
    const auto loc1 = Length2{+1_m, +3_m};
    
    const auto b0 = world.CreateBody(BodyConf{}.UseLocation(loc0));
    const auto b1 = world.CreateBody(BodyConf{}.UseLocation(loc1));
    
    auto jd = WheelJointConf{};
    jd.bodyA = b0;
    jd.bodyB = b1;
    jd.localAnchorA = Length2(-1_m, 5_m);
    jd.localAnchorB = Length2(+1_m, 5_m);
    
    auto joint = world.CreateJoint(jd);
    EXPECT_EQ(GetJointTranslation(world, joint), Length(2_m));
}

TEST(WheelJoint, GetWheelJointConf)
{
    WheelJointConf def;
    WheelJoint joint{def};
    
    ASSERT_EQ(GetType(joint), def.type);
    ASSERT_EQ(joint.GetBodyA(), def.bodyA);
    ASSERT_EQ(joint.GetBodyB(), def.bodyB);
    ASSERT_EQ(joint.GetCollideConnected(), def.collideConnected);
    ASSERT_EQ(joint.GetUserData(), def.userData);
    
    ASSERT_EQ(joint.GetLocalAnchorA(), def.localAnchorA);
    ASSERT_EQ(joint.GetLocalAnchorB(), def.localAnchorB);
    ASSERT_EQ(joint.GetLocalAxisA(), def.localAxisA);
    ASSERT_EQ(joint.IsMotorEnabled(), def.enableMotor);
    ASSERT_EQ(joint.GetMaxMotorTorque(), def.maxMotorTorque);
    ASSERT_EQ(joint.GetMotorSpeed(), def.motorSpeed);
    ASSERT_EQ(joint.GetFrequency(), def.frequency);
    ASSERT_EQ(joint.GetSpringDampingRatio(), def.dampingRatio);
    
    const auto cdef = GetWheelJointConf(joint);
    EXPECT_EQ(cdef.type, JointType::Wheel);
    EXPECT_EQ(cdef.bodyA, InvalidBodyID);
    EXPECT_EQ(cdef.bodyB, InvalidBodyID);
    EXPECT_EQ(cdef.collideConnected, false);
    EXPECT_EQ(cdef.userData, nullptr);
    
    EXPECT_EQ(cdef.localAnchorA, (Length2{}));
    EXPECT_EQ(cdef.localAnchorB, (Length2{}));
    EXPECT_EQ(cdef.localAxisA, UnitVec::GetRight());
    EXPECT_FALSE(cdef.enableMotor);
    EXPECT_EQ(cdef.maxMotorTorque, Torque(0));
    EXPECT_EQ(cdef.motorSpeed, 0_rpm);
    EXPECT_EQ(cdef.frequency, 2_Hz);
    EXPECT_EQ(cdef.dampingRatio, Real(0.7f));
}

TEST(WheelJoint, WithDynamicCircles)
{
    const auto circle = DiskShapeConf{}.UseRadius(2_m).UseDensity(10_kgpm2);
    auto world = World{};
    const auto p1 = Length2{-1_m, 0_m};
    const auto p2 = Length2{+1_m, 0_m};
    const auto b1 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p1));
    const auto b2 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p2));
    world.CreateFixture(b1, Shape{circle});
    world.CreateFixture(b2, Shape{circle});
    const auto anchor = Length2(2_m, 1_m);
    const auto jd = GetWheelJointConf(world, b1, b2, anchor);
    const auto joint = world.CreateJoint(jd);
    ASSERT_NE(joint, InvalidJointID);
    auto stepConf = StepConf{};
    
    stepConf.doWarmStart = true;
    world.Step(stepConf);
    EXPECT_NEAR(double(Real{GetX(GetLocation(world, b1)) / Meter}), -1.0, 0.001);
    EXPECT_NEAR(double(Real{GetY(GetLocation(world, b1)) / Meter}), 0.0, 0.001);
    EXPECT_NEAR(double(Real{GetX(GetLocation(world, b2)) / Meter}), +1.0, 0.01);
    EXPECT_NEAR(double(Real{GetY(GetLocation(world, b2)) / Meter}), 0.0, 0.01);
    EXPECT_EQ(GetAngle(world, b1), 0_deg);
    EXPECT_EQ(GetAngle(world, b2), 0_deg);
    EXPECT_EQ(GetAngularVelocity(world, joint), 0 * RadianPerSecond);
    EXPECT_EQ(GetAngularMass(world, joint), RotInertia(0));
    
    SetFrequency(world, joint, 0_Hz);
    world.Step(stepConf);
    EXPECT_FALSE(IsMotorEnabled(world, joint));
    EXPECT_EQ(GetFrequency(world, joint), 0_Hz);
    EXPECT_EQ(GetLinearReaction(world, joint), Momentum2{});
    EXPECT_EQ(GetAngularMass(world, joint), RotInertia(0));

    EnableMotor(world, joint, true);
    EXPECT_TRUE(IsMotorEnabled(world, joint));
    world.Step(stepConf);
    EXPECT_NEAR(static_cast<double>(StripUnit(GetAngularMass(world, joint))),
                125.66370391845703, 0.1);
    
    stepConf.doWarmStart = false;
    world.Step(stepConf);
    EXPECT_NEAR(double(Real{GetX(GetLocation(world, b1)) / Meter}), -1.0, 0.001);
    EXPECT_NEAR(double(Real{GetY(GetLocation(world, b1)) / Meter}), 0.0, 0.001);
    EXPECT_NEAR(double(Real{GetX(GetLocation(world, b2)) / Meter}), +1.0, 0.01);
    EXPECT_NEAR(double(Real{GetY(GetLocation(world, b2)) / Meter}), 0.0, 0.01);
    EXPECT_EQ(GetAngle(world, b1), 0_deg);
    EXPECT_EQ(GetAngle(world, b2), 0_deg);
    EXPECT_EQ(GetAngularVelocity(world, joint), 0 * RadianPerSecond);
    EXPECT_NEAR(static_cast<double>(StripUnit(GetAngularMass(world, joint))),
                125.66370391845703, 0.1);
}
