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

#include <Box2D/Dynamics/Joints/WeldJoint.hpp>
#include <Box2D/Dynamics/Body.hpp>
#include <Box2D/Dynamics/BodyDef.hpp>

using namespace box2d;

TEST(WeldJointDef, ByteSize)
{
    switch (sizeof(RealNum))
    {
        case  4: EXPECT_EQ(sizeof(WeldJointDef), size_t(72)); break;
        case  8: EXPECT_EQ(sizeof(WeldJointDef), size_t(96)); break;
        case 16: EXPECT_EQ(sizeof(WeldJointDef), size_t(160)); break;
        default: FAIL(); break;
    }
}

TEST(WeldJointDef, DefaultConstruction)
{
    WeldJointDef def{};
    
    EXPECT_EQ(def.type, JointType::Weld);
    EXPECT_EQ(def.bodyA, nullptr);
    EXPECT_EQ(def.bodyB, nullptr);
    EXPECT_EQ(def.collideConnected, false);
    EXPECT_EQ(def.userData, nullptr);
    
    EXPECT_EQ(def.localAnchorA, Vec2_zero * Meter);
    EXPECT_EQ(def.localAnchorB, Vec2_zero * Meter);
    EXPECT_EQ(def.referenceAngle, Angle(0));
    EXPECT_EQ(def.frequencyHz, RealNum{0} * Hertz);
    EXPECT_EQ(def.dampingRatio, RealNum(0));
}

TEST(WeldJoint, ByteSize)
{
    switch (sizeof(RealNum))
    {
        case  4: EXPECT_EQ(sizeof(WeldJoint), size_t(144)); break;
        case  8: EXPECT_EQ(sizeof(WeldJoint), size_t(240)); break;
        case 16: EXPECT_EQ(sizeof(WeldJoint), size_t(448)); break;
        default: FAIL(); break;
    }
}

TEST(WeldJoint, Construction)
{
    WeldJointDef def;
    WeldJoint joint{def};
    
    EXPECT_EQ(joint.GetType(), def.type);
    EXPECT_EQ(joint.GetBodyA(), def.bodyA);
    EXPECT_EQ(joint.GetBodyB(), def.bodyB);
    EXPECT_EQ(joint.GetCollideConnected(), def.collideConnected);
    EXPECT_EQ(joint.GetUserData(), def.userData);
    
    EXPECT_EQ(joint.GetLocalAnchorA(), def.localAnchorA);
    EXPECT_EQ(joint.GetLocalAnchorB(), def.localAnchorB);
    EXPECT_EQ(joint.GetReferenceAngle(), def.referenceAngle);
    EXPECT_EQ(joint.GetFrequency(), def.frequencyHz);
    EXPECT_EQ(joint.GetDampingRatio(), def.dampingRatio);
}

TEST(WeldJoint, GetWeldJointDef)
{
    auto bodyA = Body{BodyDef{}};
    auto bodyB = Body{BodyDef{}};
    const auto anchor = Vec2(2, 1) * Meter;
    WeldJointDef def{&bodyA, &bodyB, anchor};
    WeldJoint joint{def};
    
    ASSERT_EQ(joint.GetType(), def.type);
    ASSERT_EQ(joint.GetBodyA(), def.bodyA);
    ASSERT_EQ(joint.GetBodyB(), def.bodyB);
    ASSERT_EQ(joint.GetCollideConnected(), def.collideConnected);
    ASSERT_EQ(joint.GetUserData(), def.userData);
    
    ASSERT_EQ(joint.GetLocalAnchorA(), def.localAnchorA);
    ASSERT_EQ(joint.GetLocalAnchorB(), def.localAnchorB);
    ASSERT_EQ(joint.GetReferenceAngle(), def.referenceAngle);
    ASSERT_EQ(joint.GetFrequency(), def.frequencyHz);
    ASSERT_EQ(joint.GetDampingRatio(), def.dampingRatio);
    
    const auto cdef = GetWeldJointDef(joint);
    EXPECT_EQ(cdef.type, JointType::Weld);
    EXPECT_EQ(cdef.bodyA, &bodyA);
    EXPECT_EQ(cdef.bodyB, &bodyB);
    EXPECT_EQ(cdef.collideConnected, false);
    EXPECT_EQ(cdef.userData, nullptr);
    
    EXPECT_EQ(cdef.localAnchorA, anchor);
    EXPECT_EQ(cdef.localAnchorB, anchor);
    EXPECT_EQ(def.referenceAngle, Angle(0));
    EXPECT_EQ(def.frequencyHz, RealNum(0) * Hertz);
    EXPECT_EQ(def.dampingRatio, RealNum(0));
}
