/*
 * Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

#include <Box2D/Dynamics/Joints/WheelJoint.hpp>

using namespace box2d;

TEST(WheelJointDef, ByteSize)
{
    switch (sizeof(RealNum))
    {
        case  4: EXPECT_EQ(sizeof(WheelJointDef), size_t(88)); break;
        case  8: EXPECT_EQ(sizeof(WheelJointDef), size_t(128)); break;
        case 16: EXPECT_EQ(sizeof(WheelJointDef), size_t(144)); break;
        default: FAIL(); break;
    }
}

TEST(WheelJointDef, DefaultConstruction)
{
    WheelJointDef def{};
    
    EXPECT_EQ(def.type, JointType::Wheel);
    EXPECT_EQ(def.bodyA, nullptr);
    EXPECT_EQ(def.bodyB, nullptr);
    EXPECT_EQ(def.collideConnected, false);
    EXPECT_EQ(def.userData, nullptr);
    
    EXPECT_EQ(def.localAnchorA, Vec2_zero * Meter);
    EXPECT_EQ(def.localAnchorB, Vec2_zero * Meter);
    EXPECT_EQ(def.localAxisA, UnitVec2::GetRight());
    EXPECT_FALSE(def.enableMotor);
    EXPECT_EQ(def.maxMotorTorque, Torque(0));
    EXPECT_EQ(def.motorSpeed, AngularVelocity(0));
    EXPECT_EQ(def.frequencyHz, RealNum{2} * Hertz);
    EXPECT_EQ(def.dampingRatio, RealNum(0.7f));
}

TEST(WheelJoint, ByteSize)
{
    switch (sizeof(RealNum))
    {
        case  4: EXPECT_EQ(sizeof(WheelJoint), size_t(160)); break;
        case  8: EXPECT_EQ(sizeof(WheelJoint), size_t(272)); break;
        case 16: EXPECT_EQ(sizeof(WheelJoint), size_t(384)); break;
        default: FAIL(); break;
    }
}

TEST(WheelJoint, Construction)
{
    WheelJointDef def;
    WheelJoint joint{def};
    
    EXPECT_EQ(joint.GetType(), def.type);
    EXPECT_EQ(joint.GetBodyA(), def.bodyA);
    EXPECT_EQ(joint.GetBodyB(), def.bodyB);
    EXPECT_EQ(joint.GetCollideConnected(), def.collideConnected);
    EXPECT_EQ(joint.GetUserData(), def.userData);
    
    EXPECT_EQ(joint.GetLocalAnchorA(), def.localAnchorA);
    EXPECT_EQ(joint.GetLocalAnchorB(), def.localAnchorB);
    EXPECT_EQ(joint.GetLocalAxisA(), def.localAxisA);
    EXPECT_EQ(joint.IsMotorEnabled(), def.enableMotor);
    EXPECT_EQ(joint.GetMaxMotorTorque(), def.maxMotorTorque);
    EXPECT_EQ(joint.GetMotorSpeed(), def.motorSpeed);
    EXPECT_EQ(joint.GetSpringFrequencyHz(), def.frequencyHz);
    EXPECT_EQ(joint.GetSpringDampingRatio(), def.dampingRatio);
}

TEST(WheelJoint, GetWheelJointDef)
{
    WheelJointDef def;
    WheelJoint joint{def};
    
    ASSERT_EQ(joint.GetType(), def.type);
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
    ASSERT_EQ(joint.GetSpringFrequencyHz(), def.frequencyHz);
    ASSERT_EQ(joint.GetSpringDampingRatio(), def.dampingRatio);
    
    const auto cdef = GetWheelJointDef(joint);
    EXPECT_EQ(cdef.type, JointType::Wheel);
    EXPECT_EQ(cdef.bodyA, nullptr);
    EXPECT_EQ(cdef.bodyB, nullptr);
    EXPECT_EQ(cdef.collideConnected, false);
    EXPECT_EQ(cdef.userData, nullptr);
    
    EXPECT_EQ(cdef.localAnchorA, Vec2_zero * Meter);
    EXPECT_EQ(cdef.localAnchorB, Vec2_zero * Meter);
    EXPECT_EQ(cdef.localAxisA, UnitVec2::GetRight());
    EXPECT_FALSE(cdef.enableMotor);
    EXPECT_EQ(cdef.maxMotorTorque, Torque(0));
    EXPECT_EQ(cdef.motorSpeed, AngularVelocity(0));
    EXPECT_EQ(cdef.frequencyHz, RealNum{2} * Hertz);
    EXPECT_EQ(cdef.dampingRatio, RealNum(0.7f));
}
