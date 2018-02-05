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

#include "UnitTests.hpp"

#include <PlayRho/Dynamics/Joints/RopeJoint.hpp>
#include <PlayRho/Dynamics/Joints/TypeJointVisitor.hpp>
#include <PlayRho/Dynamics/Body.hpp>
#include <PlayRho/Dynamics/BodyConf.hpp>
#include <PlayRho/Dynamics/World.hpp>
#include <PlayRho/Collision/Shapes/DiskShapeConf.hpp>

using namespace playrho;
using namespace playrho::d2;

TEST(RopeJointConf, ByteSize)
{
    switch (sizeof(Real))
    {
        case  4:
#if defined(_WIN32) && !defined(_WIN64)
            EXPECT_EQ(sizeof(RopeJointConf), std::size_t(40));
#else
            EXPECT_EQ(sizeof(RopeJointConf), std::size_t(64));
#endif
            break;
        case  8: EXPECT_EQ(sizeof(RopeJointConf), std::size_t(80)); break;
        case 16: EXPECT_EQ(sizeof(RopeJointConf), std::size_t(128)); break;
        default: FAIL(); break;
    }
}

TEST(RopeJointConf, DefaultConstruction)
{
    RopeJointConf def{};
    
    EXPECT_EQ(def.type, JointType::Rope);
    EXPECT_EQ(def.bodyA, nullptr);
    EXPECT_EQ(def.bodyB, nullptr);
    EXPECT_EQ(def.collideConnected, false);
    EXPECT_EQ(def.userData, nullptr);
    
    EXPECT_EQ(def.localAnchorA, Length2(-1_m, 0_m));
    EXPECT_EQ(def.localAnchorB, Length2(+1_m, 0_m));
    EXPECT_EQ(def.maxLength, 0_m);
}

TEST(RopeJoint, ByteSize)
{
    switch (sizeof(Real))
    {
        case  4:
#if defined(_WIN64)
            EXPECT_EQ(sizeof(RopeJoint), std::size_t(104));
#elif defined(_WIN32)
            EXPECT_EQ(sizeof(RopeJoint), std::size_t(80));
#else
            EXPECT_EQ(sizeof(RopeJoint), std::size_t(96));
#endif
            break;
        case  8: EXPECT_EQ(sizeof(RopeJoint), std::size_t(160)); break;
        case 16: EXPECT_EQ(sizeof(RopeJoint), std::size_t(288)); break;
        default: FAIL(); break;
    }
}

TEST(RopeJoint, Construction)
{
    World world;
    const auto b0 = world.CreateBody();
    const auto b1 = world.CreateBody();

    auto def = RopeJointConf{b0, b1};
    RopeJoint joint{def};
    
    EXPECT_EQ(GetType(joint), def.type);
    EXPECT_EQ(joint.GetBodyA(), def.bodyA);
    EXPECT_EQ(joint.GetBodyB(), def.bodyB);
    EXPECT_EQ(joint.GetCollideConnected(), def.collideConnected);
    EXPECT_EQ(joint.GetUserData(), def.userData);
    EXPECT_EQ(joint.GetLinearReaction(), Momentum2{});
    EXPECT_EQ(joint.GetAngularReaction(), AngularMomentum{0});

    EXPECT_EQ(joint.GetLocalAnchorA(), def.localAnchorA);
    EXPECT_EQ(joint.GetLocalAnchorB(), def.localAnchorB);
    EXPECT_EQ(joint.GetAnchorA(), Length2(-1_m, 0_m));
    EXPECT_EQ(joint.GetAnchorB(), Length2(+1_m, 0_m));
    EXPECT_EQ(joint.GetMaxLength(), def.maxLength);
    EXPECT_EQ(joint.GetLimitState(), Joint::e_inactiveLimit);
    
    TypeJointVisitor visitor;
    joint.Accept(visitor);
    EXPECT_EQ(visitor.GetType().value(), JointType::Rope);
}

TEST(RopeJoint, GetRopeJointConf)
{
    auto world = World{};
    auto& bodyA = *world.CreateBody();
    auto& bodyB = *world.CreateBody();
    RopeJointConf def{&bodyA, &bodyB};
    const auto localAnchorA = Length2{-2_m, 0_m};
    const auto localAnchorB = Length2{+2_m, 0_m};
    def.localAnchorA = localAnchorA;
    def.localAnchorB = localAnchorB;
    RopeJoint joint{def};
    
    ASSERT_EQ(GetType(joint), def.type);
    ASSERT_EQ(joint.GetBodyA(), def.bodyA);
    ASSERT_EQ(joint.GetBodyB(), def.bodyB);
    ASSERT_EQ(joint.GetCollideConnected(), def.collideConnected);
    ASSERT_EQ(joint.GetUserData(), def.userData);
    
    ASSERT_EQ(joint.GetLocalAnchorA(), def.localAnchorA);
    ASSERT_EQ(joint.GetLocalAnchorB(), def.localAnchorB);
    ASSERT_EQ(joint.GetMaxLength(), def.maxLength);
    
    const auto cdef = GetRopeJointConf(joint);
    EXPECT_EQ(cdef.type, JointType::Rope);
    EXPECT_EQ(cdef.bodyA, &bodyA);
    EXPECT_EQ(cdef.bodyB, &bodyB);
    EXPECT_EQ(cdef.collideConnected, false);
    EXPECT_EQ(cdef.userData, nullptr);
    
    EXPECT_EQ(cdef.localAnchorA, localAnchorA);
    EXPECT_EQ(cdef.localAnchorB, localAnchorB);
    EXPECT_EQ(cdef.maxLength, 0_m);
}

TEST(RopeJoint, WithDynamicCircles)
{
    const auto circle = Shape{DiskShapeConf{}.UseRadius(0.2_m)};
    auto world = World{};
    const auto p1 = Length2{-1_m, 0_m};
    const auto p2 = Length2{+1_m, 0_m};
    const auto b1 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p1));
    const auto b2 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p2));
    b1->CreateFixture(circle);
    b2->CreateFixture(circle);
    const auto jd = RopeJointConf{b1, b2};
    world.CreateJoint(jd);
    Step(world, 1_s);
    EXPECT_GT(GetX(b1->GetLocation()), -1_m);
    EXPECT_EQ(GetY(b1->GetLocation()), 0_m);
    EXPECT_LT(GetX(b2->GetLocation()), +1_m);
    EXPECT_EQ(GetY(b2->GetLocation()), 0_m);
    EXPECT_EQ(b1->GetAngle(), 0_deg);
    EXPECT_EQ(b2->GetAngle(), 0_deg);
}

