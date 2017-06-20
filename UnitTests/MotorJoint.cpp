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

#include <Box2D/Dynamics/Joints/MotorJoint.hpp>

#include <type_traits>

using namespace box2d;

TEST(MotorJointDef, ByteSize)
{
    switch (sizeof(RealNum))
    {
        case  4: EXPECT_EQ(sizeof(MotorJointDef), size_t(64)); break;
        case  8: EXPECT_EQ(sizeof(MotorJointDef), size_t(88)); break;
        case 16: EXPECT_EQ(sizeof(MotorJointDef), size_t(144)); break;
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
    
    EXPECT_EQ(def.linearOffset, Vec2_zero * Meter);
    EXPECT_EQ(def.angularOffset, Angle(0));
    EXPECT_EQ(def.maxForce, RealNum(1) * Newton);
    EXPECT_EQ(def.maxTorque, RealNum{1} * NewtonMeter);
    EXPECT_EQ(def.correctionFactor, RealNum(0.3));
}

TEST(MotorJoint, ByteSize)
{
    switch (sizeof(RealNum))
    {
        case  4: EXPECT_EQ(sizeof(MotorJoint), size_t(128)); break;
        case  8: EXPECT_EQ(sizeof(MotorJoint), size_t(208)); break;
        case 16: EXPECT_EQ(sizeof(MotorJoint), size_t(384)); break;
        default: FAIL(); break;
    }
}

TEST(MotorJoint, Traits)
{
    EXPECT_FALSE(std::is_default_constructible<MotorJoint>::value);
    EXPECT_FALSE(std::is_nothrow_default_constructible<MotorJoint>::value);
    EXPECT_FALSE(std::is_trivially_default_constructible<MotorJoint>::value);
    
    EXPECT_FALSE(std::is_constructible<MotorJoint>::value);
    EXPECT_FALSE(std::is_nothrow_constructible<MotorJoint>::value);
    EXPECT_FALSE(std::is_trivially_constructible<MotorJoint>::value);
    
    EXPECT_TRUE(std::is_copy_constructible<MotorJoint>::value);
    EXPECT_FALSE(std::is_nothrow_copy_constructible<MotorJoint>::value);
    EXPECT_FALSE(std::is_trivially_copy_constructible<MotorJoint>::value);
    
    EXPECT_FALSE(std::is_copy_assignable<MotorJoint>::value);
    EXPECT_FALSE(std::is_nothrow_copy_assignable<MotorJoint>::value);
    EXPECT_FALSE(std::is_trivially_copy_assignable<MotorJoint>::value);
    
    EXPECT_TRUE(std::is_destructible<MotorJoint>::value);
    EXPECT_TRUE(std::is_nothrow_destructible<MotorJoint>::value);
    EXPECT_FALSE(std::is_trivially_destructible<MotorJoint>::value);
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
