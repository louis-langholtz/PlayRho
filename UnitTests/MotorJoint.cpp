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
#include <PlayRho/Dynamics/Body.hpp>
#include <PlayRho/Dynamics/BodyDef.hpp>
#include <PlayRho/Dynamics/World.hpp>
#include <PlayRho/Collision/Shapes/DiskShape.hpp>

using namespace playrho;

TEST(MotorJointDef, ByteSize)
{
    switch (sizeof(Real))
    {
        case  4: EXPECT_EQ(sizeof(MotorJointDef), std::size_t(64)); break;
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
    
    EXPECT_EQ(def.linearOffset, (Length2D{}));
    EXPECT_EQ(def.angularOffset, Angle(0));
    EXPECT_EQ(def.maxForce, Real(1) * Newton);
    EXPECT_EQ(def.maxTorque, Real{1} * NewtonMeter);
    EXPECT_EQ(def.correctionFactor, Real(0.3));
}

TEST(MotorJoint, ByteSize)
{
    switch (sizeof(Real))
    {
        case  4: EXPECT_EQ(sizeof(MotorJoint), std::size_t(120)); break;
        case  8: EXPECT_EQ(sizeof(MotorJoint), std::size_t(208)); break;
        case 16: EXPECT_EQ(sizeof(MotorJoint), std::size_t(384)); break;
        default: FAIL(); break;
    }
}

TEST(MotorJoint, Construction)
{
    MotorJointDef def;
    MotorJoint joint{def};
    
    EXPECT_EQ(joint.GetType(), def.type);
    EXPECT_EQ(joint.GetBodyA(), def.bodyA);
    EXPECT_EQ(joint.GetBodyB(), def.bodyB);
    EXPECT_EQ(joint.GetCollideConnected(), def.collideConnected);
    EXPECT_EQ(joint.GetUserData(), def.userData);
    
    EXPECT_EQ(joint.GetLinearOffset(), def.linearOffset);
    EXPECT_EQ(joint.GetAngularOffset(), def.angularOffset);
    EXPECT_EQ(joint.GetMaxForce(), def.maxForce);
    EXPECT_EQ(joint.GetMaxTorque(), def.maxTorque);
    EXPECT_EQ(joint.GetCorrectionFactor(), def.correctionFactor);
}

TEST(MotorJoint, GetMotorJointDef)
{
    MotorJointDef def;
    MotorJoint joint{def};
    
    ASSERT_EQ(joint.GetType(), def.type);
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
    
    EXPECT_EQ(cdef.linearOffset, (Length2D{}));
    EXPECT_EQ(cdef.angularOffset, Angle(0));
    EXPECT_EQ(cdef.maxForce, Real(1) * Newton);
    EXPECT_EQ(cdef.maxTorque, Real{1} * NewtonMeter);
    EXPECT_EQ(cdef.correctionFactor, Real(0.3));
}

TEST(MotorJoint, WithDynamicCircles)
{
    const auto circle = std::make_shared<DiskShape>(Real{0.2f} * Meter);
    auto world = World{WorldDef{}.UseGravity(LinearAcceleration2D{})};
    const auto p1 = Length2D{-Real(1) * Meter, Real(0) * Meter};
    const auto p2 = Length2D{+Real(1) * Meter, Real(0) * Meter};
    const auto b1 = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(p1));
    const auto b2 = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(p2));
    b1->CreateFixture(circle);
    b2->CreateFixture(circle);
    //const auto anchor = Length2D(Real(2) * Meter, Real(1) * Meter);
    const auto jd = MotorJointDef{b1, b2};
    world.CreateJoint(jd);
    Step(world, Time{Second * Real{1}});
    EXPECT_NEAR(double(Real{GetX(b1->GetLocation()) / Meter}), -1.0, 0.001);
    EXPECT_NEAR(double(Real{GetY(b1->GetLocation()) / Meter}), 0.0, 0.001);
    EXPECT_NEAR(double(Real{GetX(b2->GetLocation()) / Meter}), +1.0, 0.01);
    EXPECT_NEAR(double(Real{GetY(b2->GetLocation()) / Meter}), 0.0, 0.01);
    EXPECT_EQ(b1->GetAngle(), Angle{0});
    EXPECT_EQ(b2->GetAngle(), Angle{0});
}
