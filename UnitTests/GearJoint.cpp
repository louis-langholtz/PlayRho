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

#include <PlayRho/Dynamics/Joints/GearJoint.hpp>
#include <PlayRho/Dynamics/Joints/RevoluteJoint.hpp>
#include <PlayRho/Dynamics/Joints/PrismaticJoint.hpp>
#include <PlayRho/Dynamics/Body.hpp>
#include <PlayRho/Dynamics/BodyDef.hpp>
#include <PlayRho/Dynamics/World.hpp>
#include <PlayRho/Collision/Shapes/DiskShape.hpp>
#include <type_traits>

using namespace playrho;

TEST(GearJointDef, ByteSize)
{
    switch (sizeof(Real))
    {
        case  4: EXPECT_EQ(sizeof(GearJointDef), std::size_t(64)); break;
        case  8: EXPECT_EQ(sizeof(GearJointDef), std::size_t(64)); break;
        case 16: EXPECT_EQ(sizeof(GearJointDef), std::size_t(80)); break;
        default: FAIL(); break;
    }
}

TEST(GearJointDef, Traits)
{
    EXPECT_FALSE(std::is_default_constructible<GearJointDef>::value);
    EXPECT_FALSE(std::is_nothrow_default_constructible<GearJointDef>::value);
    EXPECT_FALSE(std::is_trivially_default_constructible<GearJointDef>::value);
    
    EXPECT_FALSE(std::is_constructible<GearJointDef>::value);
    EXPECT_FALSE(std::is_nothrow_constructible<GearJointDef>::value);
    EXPECT_FALSE(std::is_trivially_constructible<GearJointDef>::value);
    
    EXPECT_TRUE(std::is_copy_constructible<GearJointDef>::value);
    EXPECT_TRUE(std::is_nothrow_copy_constructible<GearJointDef>::value);
    EXPECT_TRUE(std::is_trivially_copy_constructible<GearJointDef>::value);
    
    EXPECT_FALSE(std::is_copy_assignable<GearJointDef>::value);
    EXPECT_FALSE(std::is_nothrow_copy_assignable<GearJointDef>::value);
    EXPECT_FALSE(std::is_trivially_copy_assignable<GearJointDef>::value);
    
    EXPECT_TRUE(std::is_destructible<GearJointDef>::value);
    EXPECT_TRUE(std::is_nothrow_destructible<GearJointDef>::value);
    EXPECT_TRUE(std::is_trivially_destructible<GearJointDef>::value);
}

TEST(GearJointDef, ConstructionRequiresNonNullJoints)
{
    EXPECT_THROW(GearJointDef(nullptr, nullptr), InvalidArgument);
}

TEST(GearJoint, ByteSize)
{
    switch (sizeof(Real))
    {
        case  4: EXPECT_EQ(sizeof(GearJoint), std::size_t(184)); break;
        case  8: EXPECT_EQ(sizeof(GearJoint), std::size_t(288)); break;
        case 16: EXPECT_EQ(sizeof(GearJoint), std::size_t(496)); break;
        default: FAIL(); break;
    }
}

TEST(GearJoint, Construction)
{
    Body body{BodyDef{}};
    RevoluteJointDef rdef{&body, &body, Length2D(0, 0)};
    RevoluteJoint revJoint1{rdef};
    RevoluteJoint revJoint2{rdef};
    GearJointDef def{&revJoint1, &revJoint2};
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

TEST(GearJoint, SetRatio)
{
    Body body{BodyDef{}};
    RevoluteJointDef rdef{&body, &body, Length2D(0, 0)};
    RevoluteJoint revJoint1{rdef};
    RevoluteJoint revJoint2{rdef};
    auto def = GearJointDef{&revJoint1, &revJoint2};
    auto joint = GearJoint{def};
    ASSERT_EQ(joint.GetRatio(), Real(1));
    joint.SetRatio(Real(2));
    EXPECT_EQ(joint.GetRatio(), Real(2));
}

TEST(GearJoint, GetGearJointDef)
{
    Body body{BodyDef{}};
    RevoluteJointDef rdef{&body, &body, Length2D(0, 0)};
    RevoluteJoint revJoint1{rdef};
    RevoluteJoint revJoint2{rdef};
    GearJointDef def{&revJoint1, &revJoint2};
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
    EXPECT_EQ(cdef.ratio, Real(1));
}

TEST(GearJoint, WithDynamicCirclesAndRevoluteJoints)
{
    const auto circle = std::make_shared<DiskShape>(Real{0.2f} * Meter);
    auto world = World{WorldDef{}.UseGravity(LinearAcceleration2D{0, 0})};
    const auto p1 = Length2D{-Real(1) * Meter, Real(0) * Meter};
    const auto p2 = Length2D{+Real(1) * Meter, Real(0) * Meter};
    const auto p3 = Length2D{+Real(2) * Meter, Real(0) * Meter};
    const auto p4 = Length2D{+Real(3) * Meter, Real(0) * Meter};
    const auto b1 = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(p1));
    const auto b2 = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(p2));
    const auto b3 = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(p3));
    const auto b4 = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(p4));
    b1->CreateFixture(circle);
    b2->CreateFixture(circle);
    GearJointDef def{
        world.CreateJoint(RevoluteJointDef{b1, b2, Length2D(0, 0)}),
        world.CreateJoint(RevoluteJointDef{b4, b3, Length2D(0, 0)})
    };
    ASSERT_NE(def.joint1, nullptr);
    ASSERT_NE(def.joint2, nullptr);
    const auto joint = world.CreateJoint(def);
    ASSERT_NE(joint, nullptr);
    Step(world, Time{Second * Real{1}});
    EXPECT_NEAR(double(Real{b1->GetLocation().x / Meter}), -1.0, 0.001);
    EXPECT_NEAR(double(Real{b1->GetLocation().y / Meter}), 0.0, 0.001);
    EXPECT_NEAR(double(Real{b2->GetLocation().x / Meter}), +1.0, 0.01);
    EXPECT_NEAR(double(Real{b2->GetLocation().y / Meter}), 0.0, 0.01);
    EXPECT_EQ(b1->GetAngle(), Angle{0});
    EXPECT_EQ(b2->GetAngle(), Angle{0});
}

TEST(GearJoint, WithDynamicCirclesAndPrismaticJoints)
{
    const auto circle = std::make_shared<DiskShape>(Real{0.2f} * Meter);
    auto world = World{WorldDef{}.UseGravity(LinearAcceleration2D{0, 0})};
    const auto p1 = Length2D{-Real(1) * Meter, Real(0) * Meter};
    const auto p2 = Length2D{+Real(1) * Meter, Real(0) * Meter};
    const auto p3 = Length2D{+Real(2) * Meter, Real(0) * Meter};
    const auto p4 = Length2D{+Real(3) * Meter, Real(0) * Meter};
    const auto b1 = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(p1));
    const auto b2 = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(p2));
    const auto b3 = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(p3));
    const auto b4 = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(p4));
    b1->CreateFixture(circle);
    b2->CreateFixture(circle);
    GearJointDef def{
        world.CreateJoint(PrismaticJointDef{b1, b2, Length2D(0, 0), UnitVec2::GetTop()}),
        world.CreateJoint(PrismaticJointDef{b4, b3, Length2D(0, 0), UnitVec2::GetTop()})
    };
    ASSERT_NE(def.joint1, nullptr);
    ASSERT_NE(def.joint2, nullptr);
    const auto joint = world.CreateJoint(def);
    ASSERT_NE(joint, nullptr);
    Step(world, Time{Second * Real{1}});
    EXPECT_NEAR(double(Real{b1->GetLocation().x / Meter}), -1.0, 0.001);
    EXPECT_NEAR(double(Real{b1->GetLocation().y / Meter}), 0.0, 0.001);
    EXPECT_NEAR(double(Real{b2->GetLocation().x / Meter}), +1.0, 0.01);
    EXPECT_NEAR(double(Real{b2->GetLocation().y / Meter}), 0.0, 0.01);
    EXPECT_EQ(b1->GetAngle(), Angle{0});
    EXPECT_EQ(b2->GetAngle(), Angle{0});
}

TEST(GearJoint, GetAnchorAandB)
{
    const auto circle = std::make_shared<DiskShape>(Real{0.2f} * Meter);
    auto world = World{WorldDef{}.UseGravity(LinearAcceleration2D{0, 0})};
    const auto p1 = Length2D{-Real(1) * Meter, Real(0) * Meter};
    const auto p2 = Length2D{+Real(1) * Meter, Real(0) * Meter};
    const auto p3 = Length2D{+Real(2) * Meter, Real(0) * Meter};
    const auto p4 = Length2D{+Real(3) * Meter, Real(0) * Meter};
    const auto b1 = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(p1));
    const auto b2 = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(p2));
    const auto b3 = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(p3));
    const auto b4 = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(p4));
    b1->CreateFixture(circle);
    b2->CreateFixture(circle);
    GearJointDef def{
        world.CreateJoint(RevoluteJointDef{b1, b2, Length2D(0, 0)}),
        world.CreateJoint(RevoluteJointDef{b4, b3, Length2D(0, 0)})
    };
    ASSERT_NE(def.joint1, nullptr);
    ASSERT_NE(def.joint2, nullptr);
    const auto joint = static_cast<GearJoint*>(world.CreateJoint(def));
    ASSERT_NE(joint, nullptr);
    
    const auto anchorA = GetWorldPoint(*(joint->GetBodyA()), joint->GetLocalAnchorA());
    const auto anchorB = GetWorldPoint(*(joint->GetBodyB()), joint->GetLocalAnchorB());

    EXPECT_EQ(joint->GetAnchorA(), anchorA);
    EXPECT_EQ(joint->GetAnchorB(), anchorB);
}
