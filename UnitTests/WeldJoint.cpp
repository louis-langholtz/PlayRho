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
#include <Box2D/Dynamics/World.hpp>
#include <Box2D/Collision/Shapes/DiskShape.hpp>

using namespace box2d;

TEST(WeldJointDef, ByteSize)
{
    switch (sizeof(Real))
    {
        case  4: EXPECT_EQ(sizeof(WeldJointDef), std::size_t(72)); break;
        case  8: EXPECT_EQ(sizeof(WeldJointDef), std::size_t(96)); break;
        case 16: EXPECT_EQ(sizeof(WeldJointDef), std::size_t(160)); break;
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
    
    EXPECT_EQ(def.localAnchorA, Length2D(0, 0));
    EXPECT_EQ(def.localAnchorB, Length2D(0, 0));
    EXPECT_EQ(def.referenceAngle, Angle(0));
    EXPECT_EQ(def.frequency, Real{0} * Hertz);
    EXPECT_EQ(def.dampingRatio, Real(0));
}

TEST(WeldJoint, ByteSize)
{
    switch (sizeof(Real))
    {
        case  4: EXPECT_EQ(sizeof(WeldJoint), std::size_t(136)); break;
        case  8: EXPECT_EQ(sizeof(WeldJoint), std::size_t(240)); break;
        case 16: EXPECT_EQ(sizeof(WeldJoint), std::size_t(448)); break;
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
    EXPECT_EQ(joint.GetFrequency(), def.frequency);
    EXPECT_EQ(joint.GetDampingRatio(), def.dampingRatio);
}

TEST(WeldJoint, GetWeldJointDef)
{
    auto bodyA = Body{BodyDef{}};
    auto bodyB = Body{BodyDef{}};
    const auto anchor = Length2D(Real(2) * Meter, Real(1) * Meter);
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
    ASSERT_EQ(joint.GetFrequency(), def.frequency);
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
    EXPECT_EQ(def.frequency, Real(0) * Hertz);
    EXPECT_EQ(def.dampingRatio, Real(0));
}

TEST(WeldJoint, WithDynamicCircles)
{
    const auto circle = std::make_shared<DiskShape>(Real{0.2f} * Meter);
    auto world = World{WorldDef{}.UseGravity(LinearAcceleration2D{0, 0})};
    const auto p1 = Length2D{-Real(1) * Meter, Real(0) * Meter};
    const auto p2 = Length2D{+Real(1) * Meter, Real(0) * Meter};
    const auto b1 = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(p1));
    const auto b2 = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(p2));
    b1->CreateFixture(circle);
    b2->CreateFixture(circle);
    const auto anchor = Length2D(Real(2) * Meter, Real(1) * Meter);
    const auto jd = WeldJointDef{b1, b2, anchor};
    world.CreateJoint(jd);
    Step(world, Time{Second * Real{1}});
    EXPECT_NEAR(double(Real{b1->GetLocation().x / Meter}), -1.0, 0.001);
    EXPECT_NEAR(double(Real{b1->GetLocation().y / Meter}), 0.0, 0.001);
    EXPECT_NEAR(double(Real{b2->GetLocation().x / Meter}), +1.0, 0.01);
    EXPECT_NEAR(double(Real{b2->GetLocation().y / Meter}), 0.0, 0.01);
    EXPECT_EQ(b1->GetAngle(), Angle{0});
    EXPECT_EQ(b2->GetAngle(), Angle{0});
}
