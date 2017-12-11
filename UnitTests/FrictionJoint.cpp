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
#include <PlayRho/Dynamics/Joints/FrictionJoint.hpp>
#include <PlayRho/Dynamics/Joints/TypeJointVisitor.hpp>
#include <PlayRho/Collision/Shapes/DiskShapeConf.hpp>
#include <PlayRho/Dynamics/World.hpp>
#include <PlayRho/Dynamics/Body.hpp>
#include <PlayRho/Dynamics/BodyDef.hpp>
#include <PlayRho/Dynamics/Fixture.hpp>

using namespace playrho;

TEST(FrictionJoint, ByteSize)
{
    switch (sizeof(Real))
    {
        case  4:
#if defined(_WIN32) && !defined(_WIN64)
            EXPECT_EQ(sizeof(FrictionJoint), std::size_t(92));
#else
            EXPECT_EQ(sizeof(FrictionJoint), std::size_t(112));
#endif
            break;
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
    
    EXPECT_EQ(def.localAnchorA, (Length2{}));
    EXPECT_EQ(def.localAnchorB, (Length2{}));
    EXPECT_EQ(def.maxForce, 0_N);
    EXPECT_EQ(def.maxTorque, 0_Nm);
}

TEST(FrictionJointDef, InitializingConstructor)
{
    World world{WorldDef{}.UseGravity(LinearAcceleration2{})};
    const auto p1 = Length2{-1_m, 0_m};
    const auto p2 = Length2{+1_m, 0_m};
    const auto b1 = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(p1));
    const auto b2 = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(p2));
    const auto anchor = Length2{0_m, 0_m};
    const auto def = FrictionJointDef{b1, b2, anchor};
    EXPECT_EQ(def.bodyA, b1);
    EXPECT_EQ(def.bodyB, b2);
    EXPECT_EQ(def.localAnchorA, GetLocalPoint(*b1, anchor));
    EXPECT_EQ(def.localAnchorB, GetLocalPoint(*b2, anchor));
}

TEST(FrictionJoint, Construction)
{
    FrictionJointDef def;
    FrictionJoint joint{def};
    
    EXPECT_EQ(GetType(joint), def.type);
    EXPECT_EQ(joint.GetBodyA(), def.bodyA);
    EXPECT_EQ(joint.GetBodyB(), def.bodyB);
    EXPECT_EQ(joint.GetCollideConnected(), def.collideConnected);
    EXPECT_EQ(joint.GetUserData(), def.userData);
    EXPECT_EQ(joint.GetLinearReaction(), Momentum2{});
    EXPECT_EQ(joint.GetAngularReaction(), AngularMomentum{0});
    
    EXPECT_EQ(joint.GetLocalAnchorA(), def.localAnchorA);
    EXPECT_EQ(joint.GetLocalAnchorB(), def.localAnchorB);
    EXPECT_EQ(joint.GetMaxForce(), def.maxForce);
    EXPECT_EQ(joint.GetMaxTorque(), def.maxTorque);
    
    TypeJointVisitor visitor;
    joint.Accept(visitor);
    EXPECT_EQ(visitor.GetType().value(), JointType::Friction);
}

TEST(FrictionJoint, GetFrictionJointDef)
{
    FrictionJointDef def;
    FrictionJoint joint{def};
    
    ASSERT_EQ(GetType(joint), def.type);
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
    
    EXPECT_EQ(cdef.localAnchorA, (Length2{}));
    EXPECT_EQ(cdef.localAnchorB, (Length2{}));
    EXPECT_EQ(cdef.maxForce, 0_N);
    EXPECT_EQ(cdef.maxTorque, 0_Nm);
}

TEST(FrictionJoint, WithDynamicCircles)
{
    const auto circle = DiskShapeConf{}.UseRadius(0.2_m);
    World world{WorldDef{}.UseGravity(LinearAcceleration2{})};
    const auto p1 = Length2{-1_m, 0_m};
    const auto p2 = Length2{+1_m, 0_m};
    const auto b1 = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(p1));
    const auto b2 = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(p2));
    b1->CreateFixture(circle);
    b2->CreateFixture(circle);
    auto jd = FrictionJointDef{};
    jd.bodyA = b1;
    jd.bodyB = b2;
    world.CreateJoint(jd);
    Step(world, 1_s);
    EXPECT_NEAR(double(Real{GetX(b1->GetLocation()) / 1_m}), -1.0, 0.001);
    EXPECT_NEAR(double(Real{GetY(b1->GetLocation()) / 1_m}), 0.0, 0.001);
    EXPECT_NEAR(double(Real{GetX(b2->GetLocation()) / 1_m}), +1.0, 0.01);
    EXPECT_NEAR(double(Real{GetY(b2->GetLocation()) / 1_m}), 0.0, 0.01);
    EXPECT_EQ(b1->GetAngle(), 0_deg);
    EXPECT_EQ(b2->GetAngle(), 0_deg);
}
