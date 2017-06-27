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
#include <Box2D/Dynamics/Joints/FrictionJoint.hpp>
#include <Box2D/Collision/Shapes/DiskShape.hpp>
#include <Box2D/Dynamics/World.hpp>
#include <Box2D/Dynamics/Body.hpp>
#include <Box2D/Dynamics/BodyDef.hpp>
#include <Box2D/Dynamics/Fixture.hpp>

using namespace box2d;

TEST(FrictionJoint, ByteSize)
{
    switch (sizeof(RealNum))
    {
        case  4: EXPECT_EQ(sizeof(FrictionJoint), std::size_t(112)); break;
        case  8: EXPECT_EQ(sizeof(FrictionJoint), std::size_t(184)); break;
        case 16: EXPECT_EQ(sizeof(FrictionJoint), std::size_t(336)); break;
        default: FAIL(); break;
    }
}

TEST(FrictionJointDef, DefaultConstruction)
{
    FrictionJointDef def{};

    EXPECT_EQ(def.type, JointType::Friction);
    EXPECT_EQ(def.bodyA, nullptr);
    EXPECT_EQ(def.bodyB, nullptr);
    EXPECT_EQ(def.collideConnected, false);
    EXPECT_EQ(def.userData, nullptr);
    
    EXPECT_EQ(def.localAnchorA, Vec2_zero * Meter);
    EXPECT_EQ(def.localAnchorB, Vec2_zero * Meter);
    EXPECT_EQ(def.maxForce, RealNum(0) * Newton);
    EXPECT_EQ(def.maxTorque, RealNum{0} * NewtonMeter);
}

TEST(FrictionJoint, Construction)
{
    FrictionJointDef def;
    FrictionJoint joint{def};
    
    EXPECT_EQ(joint.GetType(), def.type);
    EXPECT_EQ(joint.GetBodyA(), def.bodyA);
    EXPECT_EQ(joint.GetBodyB(), def.bodyB);
    EXPECT_EQ(joint.GetCollideConnected(), def.collideConnected);
    EXPECT_EQ(joint.GetUserData(), def.userData);
    
    EXPECT_EQ(joint.GetLocalAnchorA(), def.localAnchorA);
    EXPECT_EQ(joint.GetLocalAnchorB(), def.localAnchorB);
    EXPECT_EQ(joint.GetMaxForce(), def.maxForce);
    EXPECT_EQ(joint.GetMaxTorque(), def.maxTorque);
}

TEST(FrictionJoint, GetFrictionJointDef)
{
    FrictionJointDef def;
    FrictionJoint joint{def};
    
    ASSERT_EQ(joint.GetType(), def.type);
    ASSERT_EQ(joint.GetBodyA(), def.bodyA);
    ASSERT_EQ(joint.GetBodyB(), def.bodyB);
    ASSERT_EQ(joint.GetCollideConnected(), def.collideConnected);
    ASSERT_EQ(joint.GetUserData(), def.userData);
    
    ASSERT_EQ(joint.GetLocalAnchorA(), def.localAnchorA);
    ASSERT_EQ(joint.GetLocalAnchorB(), def.localAnchorB);
    ASSERT_EQ(joint.GetMaxForce(), def.maxForce);
    ASSERT_EQ(joint.GetMaxTorque(), def.maxTorque);
    
    const auto cdef = GetFrictionJointDef(joint);
    EXPECT_EQ(cdef.type, JointType::Friction);
    EXPECT_EQ(cdef.bodyA, nullptr);
    EXPECT_EQ(cdef.bodyB, nullptr);
    EXPECT_EQ(cdef.collideConnected, false);
    EXPECT_EQ(cdef.userData, nullptr);
    
    EXPECT_EQ(cdef.localAnchorA, Vec2_zero * Meter);
    EXPECT_EQ(cdef.localAnchorB, Vec2_zero * Meter);
    EXPECT_EQ(cdef.maxForce, RealNum(0) * Newton);
    EXPECT_EQ(cdef.maxTorque, RealNum{0} * NewtonMeter);
}

TEST(FrictionJoint, WithDynamicCircles)
{
    const auto circle = std::make_shared<DiskShape>(RealNum{0.2f} * Meter);
    World world{WorldDef{}.UseGravity(Vec2_zero * MeterPerSquareSecond)};
    const auto p1 = Vec2{-1, 0} * Meter;
    const auto p2 = Vec2{+1, 0} * Meter;
    const auto b1 = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(p1));
    const auto b2 = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(p2));
    b1->CreateFixture(circle);
    b2->CreateFixture(circle);
    auto jd = FrictionJointDef{};
    jd.bodyA = b1;
    jd.bodyB = b2;
    world.CreateJoint(jd);
    Step(world, Time{Second * RealNum{1}});
    EXPECT_NEAR(double(RealNum{b1->GetLocation().x / Meter}), -1.0, 0.001);
    EXPECT_NEAR(double(RealNum{b1->GetLocation().y / Meter}), 0.0, 0.001);
    EXPECT_NEAR(double(RealNum{b2->GetLocation().x / Meter}), +1.0, 0.01);
    EXPECT_NEAR(double(RealNum{b2->GetLocation().y / Meter}), 0.0, 0.01);
    EXPECT_EQ(b1->GetAngle(), RealNum{0} * Degree);
    EXPECT_EQ(b2->GetAngle(), RealNum{0} * Degree);
}
