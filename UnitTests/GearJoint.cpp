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
#include <Box2D/Dynamics/World.hpp>
#include <Box2D/Collision/Shapes/DiskShape.hpp>

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


TEST(GearJoint, WithDynamicCircles)
{
    const auto circle = std::make_shared<DiskShape>(RealNum{0.2f} * Meter);
    auto world = World{WorldDef{}.UseGravity(Vec2_zero * MeterPerSquareSecond)};
    const auto p1 = Vec2{-1, 0} * Meter;
    const auto p2 = Vec2{+1, 0} * Meter;
    const auto p3 = Vec2{+2, 0} * Meter;
    const auto p4 = Vec2{+3, 0} * Meter;
    const auto b1 = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(p1));
    const auto b2 = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(p2));
    const auto b3 = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(p3));
    const auto b4 = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(p4));
    b1->CreateFixture(circle);
    b2->CreateFixture(circle);
    GearJointDef def;
    def.joint1 = world.CreateJoint(RevoluteJointDef{b1, b2, Vec2_zero * Meter});
    def.joint2 = world.CreateJoint(RevoluteJointDef{b4, b3, Vec2_zero * Meter});
    ASSERT_NE(def.joint1, nullptr);
    ASSERT_NE(def.joint2, nullptr);
    const auto gearJoint = world.CreateJoint(def);
    ASSERT_NE(gearJoint, nullptr);
    Step(world, Time{Second * RealNum{1}});
    EXPECT_NEAR(double(RealNum{b1->GetLocation().x / Meter}), -1.0, 0.001);
    EXPECT_NEAR(double(RealNum{b1->GetLocation().y / Meter}), 0.0, 0.001);
    EXPECT_NEAR(double(RealNum{b2->GetLocation().x / Meter}), +1.0, 0.01);
    EXPECT_NEAR(double(RealNum{b2->GetLocation().y / Meter}), 0.0, 0.01);
    EXPECT_EQ(b1->GetAngle(), RealNum{0} * Degree);
    EXPECT_EQ(b2->GetAngle(), RealNum{0} * Degree);
}
