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

#include "gtest/gtest.h"

#include <PlayRho/Dynamics/Joints/MotorJoint.hpp>
#include <PlayRho/Dynamics/Joints/TypeJointVisitor.hpp>
#include <PlayRho/Dynamics/Body.hpp>
#include <PlayRho/Dynamics/BodyDef.hpp>
#include <PlayRho/Dynamics/World.hpp>
#include <PlayRho/Collision/Shapes/DiskShapeConf.hpp>

using namespace playrho;

TEST(MotorJointDef, ByteSize)
{
    switch (sizeof(Real))
    {
        case  4:
#if defined(_WIN32) && !defined(_WIN64)
            EXPECT_EQ(sizeof(MotorJointDef), std::size_t(44));
#else
            EXPECT_EQ(sizeof(MotorJointDef), std::size_t(64));
#endif
            break;
        case  8: EXPECT_EQ(sizeof(MotorJointDef), std::size_t(88)); break;
        case 16: EXPECT_EQ(sizeof(MotorJointDef), std::size_t(144)); break;
        default: FAIL(); break;
    }
}

TEST(MotorJointDef, DefaultConstruction)
{
    MotorJointDef def{};
    
    EXPECT_EQ(def.type, JointType::Motor);
    EXPECT_EQ(def.bodyA, nullptr);
    EXPECT_EQ(def.bodyB, nullptr);
    EXPECT_EQ(def.collideConnected, false);
    EXPECT_EQ(def.userData, nullptr);
    
    EXPECT_EQ(def.linearOffset, (Length2{}));
    EXPECT_EQ(def.angularOffset, 0_deg);
    EXPECT_EQ(def.maxForce, 1_N);
    EXPECT_EQ(def.maxTorque, 1_Nm);
    EXPECT_EQ(def.correctionFactor, Real(0.3));
}

TEST(MotorJointDef, BuilderConstruction)
{
    const auto bodyA = reinterpret_cast<Body*>(0x1);
    const auto bodyB = reinterpret_cast<Body*>(0x2);
    const auto collideConnected = true;
    int tmp;
    const auto userData = &tmp;
    const auto linearOffset = Length2{2_m, 3_m};
    const auto angularOffset = 33_rad;
    const auto maxForce = 22_N;
    const auto maxTorque = 31_Nm;
    const auto correctionFactor = Real(0.44f);
    const auto def = MotorJointDef{}
        .UseBodyA(bodyA).UseBodyB(bodyB).UseCollideConnected(collideConnected).UseUserData(userData)
        .UseLinearOffset(linearOffset).UseAngularOffset(angularOffset)
        .UseMaxForce(maxForce).UseMaxTorque(maxTorque).UseCorrectionFactor(correctionFactor);
    
    EXPECT_EQ(def.type, JointType::Motor);
    EXPECT_EQ(def.bodyA, bodyA);
    EXPECT_EQ(def.bodyB, bodyB);
    EXPECT_EQ(def.collideConnected, collideConnected);
    EXPECT_EQ(def.userData, userData);
    
    EXPECT_EQ(def.linearOffset, linearOffset);
    EXPECT_EQ(def.angularOffset, angularOffset);
    EXPECT_EQ(def.maxForce, maxForce);
    EXPECT_EQ(def.maxTorque, maxTorque);
    EXPECT_EQ(def.correctionFactor, correctionFactor);
}

TEST(MotorJoint, ByteSize)
{
    switch (sizeof(Real))
    {
        case  4:
#if defined(_WIN64)
            EXPECT_EQ(sizeof(MotorJoint), std::size_t(128));
#elif defined(_WIN32)
            EXPECT_EQ(sizeof(MotorJoint), std::size_t(104));
#else
            EXPECT_EQ(sizeof(MotorJoint), std::size_t(120));
#endif
            break;
        case  8: EXPECT_EQ(sizeof(MotorJoint), std::size_t(208)); break;
        case 16: EXPECT_EQ(sizeof(MotorJoint), std::size_t(384)); break;
        default: FAIL(); break;
    }
}

TEST(MotorJoint, Construction)
{
    MotorJointDef def;
    MotorJoint joint{def};
    
    EXPECT_EQ(GetType(joint), def.type);
    EXPECT_EQ(joint.GetBodyA(), def.bodyA);
    EXPECT_EQ(joint.GetBodyB(), def.bodyB);
    EXPECT_EQ(joint.GetCollideConnected(), def.collideConnected);
    EXPECT_EQ(joint.GetUserData(), def.userData);
    EXPECT_EQ(joint.GetLinearReaction(), Momentum2{});
    EXPECT_EQ(joint.GetAngularReaction(), AngularMomentum{0});

    EXPECT_EQ(joint.GetLinearOffset(), def.linearOffset);
    EXPECT_EQ(joint.GetAngularOffset(), def.angularOffset);
    EXPECT_EQ(joint.GetMaxForce(), def.maxForce);
    EXPECT_EQ(joint.GetMaxTorque(), def.maxTorque);
    EXPECT_EQ(joint.GetCorrectionFactor(), def.correctionFactor);
    
    TypeJointVisitor visitor;
    joint.Accept(visitor);
    EXPECT_EQ(visitor.GetType().value(), JointType::Motor);
}

TEST(MotorJoint, SetCorrectionFactor)
{
    MotorJointDef def;
    MotorJoint joint{def};
    
    ASSERT_EQ(joint.GetCorrectionFactor(), def.correctionFactor);
    ASSERT_EQ(Real(0.3), def.correctionFactor);
    
    joint.SetCorrectionFactor(Real(0.9));
    EXPECT_EQ(joint.GetCorrectionFactor(), Real(0.9));
}

TEST(MotorJoint, GetMotorJointDef)
{
    MotorJointDef def;
    MotorJoint joint{def};
    
    ASSERT_EQ(GetType(joint), def.type);
    ASSERT_EQ(joint.GetBodyA(), def.bodyA);
    ASSERT_EQ(joint.GetBodyB(), def.bodyB);
    ASSERT_EQ(joint.GetCollideConnected(), def.collideConnected);
    ASSERT_EQ(joint.GetUserData(), def.userData);
    
    ASSERT_EQ(joint.GetLinearOffset(), def.linearOffset);
    ASSERT_EQ(joint.GetAngularOffset(), def.angularOffset);
    ASSERT_EQ(joint.GetMaxForce(), def.maxForce);
    ASSERT_EQ(joint.GetMaxTorque(), def.maxTorque);
    ASSERT_EQ(joint.GetCorrectionFactor(), def.correctionFactor);
    
    const auto cdef = GetMotorJointDef(joint);
    EXPECT_EQ(cdef.type, JointType::Motor);
    EXPECT_EQ(cdef.bodyA, nullptr);
    EXPECT_EQ(cdef.bodyB, nullptr);
    EXPECT_EQ(cdef.collideConnected, false);
    EXPECT_EQ(cdef.userData, nullptr);
    
    EXPECT_EQ(cdef.linearOffset, (Length2{}));
    EXPECT_EQ(cdef.angularOffset, 0_deg);
    EXPECT_EQ(cdef.maxForce, 1_N);
    EXPECT_EQ(cdef.maxTorque, 1_Nm);
    EXPECT_EQ(cdef.correctionFactor, Real(0.3));
}

TEST(MotorJoint, WithDynamicCircles)
{
    const auto circle = DiskShapeConf{}.UseRadius(0.2_m);
    auto world = World{WorldDef{}.UseGravity(LinearAcceleration2{})};
    const auto p1 = Length2{-1_m, 0_m};
    const auto p2 = Length2{+1_m, 0_m};
    const auto b1 = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(p1));
    const auto b2 = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(p2));
    b1->CreateFixture(circle);
    b2->CreateFixture(circle);
    //const auto anchor = Length2(2_m, 1_m);
    const auto jd = MotorJointDef{b1, b2};
    const auto joint = static_cast<MotorJoint*>(world.CreateJoint(jd));
    ASSERT_NE(joint, nullptr);
    EXPECT_EQ(joint->GetAnchorA(), p1);
    EXPECT_EQ(joint->GetAnchorB(), p2);

    Step(world, 1_s);
    EXPECT_NEAR(double(Real{GetX(b1->GetLocation()) / Meter}), -1.0, 0.001);
    EXPECT_NEAR(double(Real{GetY(b1->GetLocation()) / Meter}), 0.0, 0.001);
    EXPECT_NEAR(double(Real{GetX(b2->GetLocation()) / Meter}), +1.0, 0.01);
    EXPECT_NEAR(double(Real{GetY(b2->GetLocation()) / Meter}), 0.0, 0.01);
    EXPECT_EQ(b1->GetAngle(), 0_deg);
    EXPECT_EQ(b2->GetAngle(), 0_deg);
}

TEST(MotorJoint, SetLinearOffset)
{
    const auto circle = DiskShapeConf{}.UseRadius(0.2_m);
    auto world = World{WorldDef{}.UseGravity(LinearAcceleration2{})};
    const auto p1 = Length2{-1_m, 0_m};
    const auto p2 = Length2{+1_m, 0_m};
    const auto b1 = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(p1));
    const auto b2 = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(p2));
    b1->CreateFixture(circle);
    b2->CreateFixture(circle);
    //const auto anchor = Length2(2_m, 1_m);
    const auto jd = MotorJointDef{b1, b2};
    const auto joint = static_cast<MotorJoint*>(world.CreateJoint(jd));
    ASSERT_NE(joint, nullptr);
    EXPECT_EQ(joint->GetAnchorA(), p1);
    EXPECT_EQ(joint->GetAnchorB(), p2);
    
    const auto linearOffset = Length2{2_m, 1_m};
    ASSERT_EQ(joint->GetLinearOffset(), jd.linearOffset);
    ASSERT_NE(jd.linearOffset, linearOffset);
    joint->SetLinearOffset(linearOffset);
    EXPECT_EQ(joint->GetLinearOffset(), linearOffset);
}

TEST(MotorJoint, SetAngularOffset)
{
    const auto circle = DiskShapeConf{}.UseRadius(0.2_m);
    auto world = World{WorldDef{}.UseGravity(LinearAcceleration2{})};
    const auto p1 = Length2{-1_m, 0_m};
    const auto p2 = Length2{+1_m, 0_m};
    const auto b1 = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(p1));
    const auto b2 = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(p2));
    b1->CreateFixture(circle);
    b2->CreateFixture(circle);
    //const auto anchor = Length2(2_m, 1_m);
    const auto jd = MotorJointDef{b1, b2};
    const auto joint = static_cast<MotorJoint*>(world.CreateJoint(jd));
    ASSERT_NE(joint, nullptr);
    EXPECT_EQ(joint->GetAnchorA(), p1);
    EXPECT_EQ(joint->GetAnchorB(), p2);

    ASSERT_EQ(joint->GetAngularOffset(), 0_deg);
    joint->SetAngularOffset(45_deg);
    EXPECT_EQ(joint->GetAngularOffset(), 45_deg);
}
