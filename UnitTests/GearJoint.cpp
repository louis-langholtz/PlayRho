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

#include <Box2D/Dynamics/Joints/GearJoint.hpp>
#include <Box2D/Dynamics/Joints/RevoluteJoint.hpp>
#include <Box2D/Dynamics/Body.hpp>
#include <Box2D/Dynamics/BodyDef.hpp>

using namespace box2d;

TEST(GearJointDef, ByteSize)
{
    switch (sizeof(RealNum))
    {
        case  4: EXPECT_EQ(sizeof(GearJointDef), size_t(64)); break;
        case  8: EXPECT_EQ(sizeof(GearJointDef), size_t(64)); break;
        case 16: EXPECT_EQ(sizeof(GearJointDef), size_t(80)); break;
        default: FAIL(); break;
    }
}

TEST(GearJointDef, DefaultConstruction)
{
    GearJointDef def{};
    
    EXPECT_EQ(def.type, JointType::Gear);
    EXPECT_EQ(def.bodyA, nullptr);
    EXPECT_EQ(def.bodyB, nullptr);
    EXPECT_EQ(def.collideConnected, false);
    EXPECT_EQ(def.userData, nullptr);
    
    EXPECT_EQ(def.joint1, nullptr);
    EXPECT_EQ(def.joint2, nullptr);
    EXPECT_EQ(def.ratio, RealNum(1));
}

TEST(GearJoint, ByteSize)
{
    switch (sizeof(RealNum))
    {
        case  4: EXPECT_EQ(sizeof(GearJoint), size_t(184)); break;
        case  8: EXPECT_EQ(sizeof(GearJoint), size_t(288)); break;
        case 16: EXPECT_EQ(sizeof(GearJoint), size_t(496)); break;
        default: FAIL(); break;
    }
}

TEST(GearJoint, Construction)
{
    Body body{BodyDef{}};
    RevoluteJointDef rdef{&body, &body, Vec2_zero * Meter};
    RevoluteJoint revJoint1{rdef};
    RevoluteJoint revJoint2{rdef};
    GearJointDef def;
    def.joint1 = &revJoint1;
    def.joint2 = &revJoint2;
    GearJoint joint{def};
    
    EXPECT_EQ(joint.GetType(), def.type);
    EXPECT_EQ(joint.GetBodyA(), def.joint1->GetBodyB());
    EXPECT_EQ(joint.GetBodyB(), def.joint2->GetBodyB());
    EXPECT_EQ(joint.GetCollideConnected(), def.collideConnected);
    EXPECT_EQ(joint.GetUserData(), def.userData);
    
    EXPECT_EQ(joint.GetLocalAnchorA(), revJoint1.GetLocalAnchorB());
    EXPECT_EQ(joint.GetLocalAnchorB(), revJoint2.GetLocalAnchorB());
    EXPECT_EQ(joint.GetJoint1(), def.joint1);
    EXPECT_EQ(joint.GetJoint2(), def.joint2);
    EXPECT_EQ(joint.GetRatio(), def.ratio);
}

TEST(GearJoint, GetGearJointDef)
{
    Body body{BodyDef{}};
    RevoluteJointDef rdef{&body, &body, Vec2_zero * Meter};
    RevoluteJoint revJoint1{rdef};
    RevoluteJoint revJoint2{rdef};
    GearJointDef def;
    def.joint1 = &revJoint1;
    def.joint2 = &revJoint2;
    GearJoint joint{def};
    
    ASSERT_EQ(joint.GetType(), def.type);
    ASSERT_EQ(joint.GetBodyA(), def.joint1->GetBodyB());
    ASSERT_EQ(joint.GetBodyB(), def.joint2->GetBodyB());
    ASSERT_EQ(joint.GetCollideConnected(), def.collideConnected);
    ASSERT_EQ(joint.GetUserData(), def.userData);
    
    ASSERT_EQ(joint.GetLocalAnchorA(), revJoint1.GetLocalAnchorB());
    ASSERT_EQ(joint.GetLocalAnchorB(), revJoint2.GetLocalAnchorB());
    ASSERT_EQ(joint.GetJoint1(), def.joint1);
    ASSERT_EQ(joint.GetJoint2(), def.joint2);
    ASSERT_EQ(joint.GetRatio(), def.ratio);
    
    const auto cdef = GetGearJointDef(joint);
    EXPECT_EQ(cdef.type, JointType::Gear);
    EXPECT_EQ(cdef.bodyA, def.joint1->GetBodyB());
    EXPECT_EQ(cdef.bodyB, def.joint2->GetBodyB());
    EXPECT_EQ(cdef.collideConnected, false);
    EXPECT_EQ(cdef.userData, nullptr);
    
    EXPECT_EQ(cdef.joint1, &revJoint1);
    EXPECT_EQ(cdef.joint2, &revJoint2);
    EXPECT_EQ(cdef.ratio, RealNum(1));
}
