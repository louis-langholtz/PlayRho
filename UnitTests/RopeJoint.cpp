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

#include <Box2D/Dynamics/Joints/RopeJoint.hpp>
#include <Box2D/Dynamics/Body.hpp>
#include <Box2D/Dynamics/BodyDef.hpp>
#include <Box2D/Dynamics/World.hpp>
#include <Box2D/Collision/Shapes/DiskShape.hpp>

using namespace box2d;

TEST(RopeJointDef, ByteSize)
{
    switch (sizeof(RealNum))
    {
        case  4: EXPECT_EQ(sizeof(RopeJointDef), std::size_t(64)); break;
        case  8: EXPECT_EQ(sizeof(RopeJointDef), std::size_t(80)); break;
        case 16: EXPECT_EQ(sizeof(RopeJointDef), std::size_t(128)); break;
        default: FAIL(); break;
    }
}

TEST(RopeJointDef, DefaultConstruction)
{
    RopeJointDef def{};
    
    EXPECT_EQ(def.type, JointType::Rope);
    EXPECT_EQ(def.bodyA, nullptr);
    EXPECT_EQ(def.bodyB, nullptr);
    EXPECT_EQ(def.collideConnected, false);
    EXPECT_EQ(def.userData, nullptr);
    
    EXPECT_EQ(def.localAnchorA, Vec2(-1, 0) * Meter);
    EXPECT_EQ(def.localAnchorB, Vec2(+1, 0) * Meter);
    EXPECT_EQ(def.maxLength, Length{0});
}

TEST(RopeJoint, ByteSize)
{
    switch (sizeof(RealNum))
    {
        case  4: EXPECT_EQ(sizeof(RopeJoint), std::size_t(104)); break;
        case  8: EXPECT_EQ(sizeof(RopeJoint), std::size_t(160)); break;
        case 16: EXPECT_EQ(sizeof(RopeJoint), std::size_t(288)); break;
        default: FAIL(); break;
    }
}

TEST(RopeJoint, Construction)
{
    RopeJointDef def;
    RopeJoint joint{def};
    
    EXPECT_EQ(joint.GetType(), def.type);
    EXPECT_EQ(joint.GetBodyA(), def.bodyA);
    EXPECT_EQ(joint.GetBodyB(), def.bodyB);
    EXPECT_EQ(joint.GetCollideConnected(), def.collideConnected);
    EXPECT_EQ(joint.GetUserData(), def.userData);
    
    EXPECT_EQ(joint.GetLocalAnchorA(), def.localAnchorA);
    EXPECT_EQ(joint.GetLocalAnchorB(), def.localAnchorB);
    EXPECT_EQ(joint.GetMaxLength(), def.maxLength);
}

TEST(RopeJoint, GetRopeJointDef)
{
    auto bodyA = Body{BodyDef{}};
    auto bodyB = Body{BodyDef{}};
    RopeJointDef def{&bodyA, &bodyB};
    const auto localAnchorA = Vec2{-2, 0} * Meter;
    const auto localAnchorB = Vec2{+2, 0} * Meter;
    def.localAnchorA = localAnchorA;
    def.localAnchorB = localAnchorB;
    RopeJoint joint{def};
    
    ASSERT_EQ(joint.GetType(), def.type);
    ASSERT_EQ(joint.GetBodyA(), def.bodyA);
    ASSERT_EQ(joint.GetBodyB(), def.bodyB);
    ASSERT_EQ(joint.GetCollideConnected(), def.collideConnected);
    ASSERT_EQ(joint.GetUserData(), def.userData);
    
    ASSERT_EQ(joint.GetLocalAnchorA(), def.localAnchorA);
    ASSERT_EQ(joint.GetLocalAnchorB(), def.localAnchorB);
    ASSERT_EQ(joint.GetMaxLength(), def.maxLength);
    
    const auto cdef = GetRopeJointDef(joint);
    EXPECT_EQ(cdef.type, JointType::Rope);
    EXPECT_EQ(cdef.bodyA, &bodyA);
    EXPECT_EQ(cdef.bodyB, &bodyB);
    EXPECT_EQ(cdef.collideConnected, false);
    EXPECT_EQ(cdef.userData, nullptr);
    
    EXPECT_EQ(cdef.localAnchorA, localAnchorA);
    EXPECT_EQ(cdef.localAnchorB, localAnchorB);
    EXPECT_EQ(cdef.maxLength, Length{0});
}

TEST(RopeJoint, WithDynamicCircles)
{
    const auto circle = std::make_shared<DiskShape>(RealNum{0.2f} * Meter);
    auto world = World{WorldDef{}.UseGravity(Vec2_zero * MeterPerSquareSecond)};
    const auto p1 = Vec2{-1, 0} * Meter;
    const auto p2 = Vec2{+1, 0} * Meter;
    const auto b1 = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(p1));
    const auto b2 = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(p2));
    b1->CreateFixture(circle);
    b2->CreateFixture(circle);
    const auto jd = RopeJointDef{b1, b2};
    world.CreateJoint(jd);
    Step(world, Time{Second * RealNum{1}});
    EXPECT_GT(b1->GetLocation().x, RealNum(-1) * Meter);
    EXPECT_EQ(b1->GetLocation().y, RealNum(0) * Meter);
    EXPECT_LT(b2->GetLocation().x, RealNum(+1) * Meter);
    EXPECT_EQ(b2->GetLocation().y, RealNum(0) * Meter);
    EXPECT_EQ(b1->GetAngle(), RealNum{0} * Degree);
    EXPECT_EQ(b2->GetAngle(), RealNum{0} * Degree);
}

