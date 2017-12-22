/*
 * Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

#include "gtest/gtest.h"
#include <PlayRho/Dynamics/Joints/MouseJoint.hpp>
#include <PlayRho/Dynamics/Joints/TypeJointVisitor.hpp>
#include <PlayRho/Dynamics/World.hpp>

using namespace playrho;

TEST(MouseJointDef, UseTarget)
{
    const auto value = Length2(19_m, -9_m);
    EXPECT_NE(MouseJointDef{}.target, value);
    EXPECT_EQ(MouseJointDef{}.UseTarget(value).target, value);
}

TEST(MouseJointDef, UseMaxForce)
{
    const auto value = Force(19_N);
    EXPECT_NE(MouseJointDef{}.maxForce, value);
    EXPECT_EQ(MouseJointDef{}.UseMaxForce(value).maxForce, value);
}

TEST(MouseJointDef, UseFrequency)
{
    const auto value = 19_Hz;
    EXPECT_NE(MouseJointDef{}.frequency, value);
    EXPECT_EQ(MouseJointDef{}.UseFrequency(value).frequency, value);
}

TEST(MouseJointDef, UseDampingRatio)
{
    const auto value = Real(0.4);
    EXPECT_NE(MouseJointDef{}.dampingRatio, value);
    EXPECT_EQ(MouseJointDef{}.UseDampingRatio(value).dampingRatio, value);
}

TEST(MouseJoint, ByteSize)
{
    switch (sizeof(Real))
    {
        case  4:
#if defined(_WIN32) && !defined(_WIN64)
            EXPECT_EQ(sizeof(MouseJoint), std::size_t(92));
#else
            EXPECT_EQ(sizeof(MouseJoint), std::size_t(112));
#endif
            break;
        case  8: EXPECT_EQ(sizeof(MouseJoint), std::size_t(184)); break;
        case 16: EXPECT_EQ(sizeof(MouseJoint), std::size_t(336)); break;
        default: FAIL(); break;
    }
}

TEST(MouseJoint, DefaultInitialized)
{
    const auto def = MouseJointDef{};
    auto joint = MouseJoint{def};
    
    EXPECT_EQ(GetType(joint), JointType::Mouse);
    EXPECT_EQ(joint.GetBodyA(), def.bodyA);
    EXPECT_EQ(joint.GetBodyB(), def.bodyB);
    EXPECT_EQ(joint.GetAnchorA(), def.target);
    //EXPECT_FALSE(IsValid(joint.GetAnchorB()));
    EXPECT_EQ(joint.GetLinearReaction(), Momentum2{});
    EXPECT_EQ(joint.GetAngularReaction(), AngularMomentum{0});
    EXPECT_EQ(joint.GetUserData(), nullptr);
    EXPECT_FALSE(joint.GetCollideConnected());
    //EXPECT_FALSE(IsValid(joint.GetLocalAnchorB()));
    EXPECT_EQ(joint.GetTarget(), def.target);
    EXPECT_EQ(joint.GetMaxForce(), def.maxForce);
    EXPECT_EQ(joint.GetFrequency(), def.frequency);
    EXPECT_EQ(joint.GetDampingRatio(), def.dampingRatio);
    
    TypeJointVisitor visitor;
    joint.Accept(visitor);
    EXPECT_EQ(visitor.GetType().value(), JointType::Mouse);
    EXPECT_TRUE(visitor.GetWritable());
}

TEST(MouseJoint, GetLocalAnchorB)
{
    auto world = World{};
    const auto bA = world.CreateBody();
    const auto bB = world.CreateBody();
    ASSERT_NE(bA, nullptr);
    ASSERT_NE(bB, nullptr);
    
    auto def = MouseJointDef{};
    def.bodyA = bA;
    def.bodyB = bB;
    def.userData = reinterpret_cast<void*>(71);
    def.target = Length2(-1.4_m, -2_m);
    def.maxForce = 3_N;
    def.frequency = 67_Hz;
    def.dampingRatio = Real(0.8);
    
    const auto joint = MouseJoint{def};
    EXPECT_EQ(joint.GetLocalAnchorB(), def.target);
}

TEST(MouseJoint, ShiftOrigin)
{
    auto world = World{};
    const auto bA = world.CreateBody();
    const auto bB = world.CreateBody();
    ASSERT_NE(bA, nullptr);
    ASSERT_NE(bB, nullptr);
    auto def = MouseJointDef{};
    def.bodyA = bA;
    def.bodyB = bB;
    def.target = Length2(-1.4_m, -2_m);
    auto joint = MouseJoint{def};
    ASSERT_EQ(joint.GetTarget(), def.target);
    
    const auto newOrigin = Length2{1_m, 1_m};
    EXPECT_TRUE(joint.ShiftOrigin(newOrigin));
    EXPECT_EQ(joint.GetTarget(), def.target - newOrigin);
}

TEST(MouseJointDef, GetMouseJointDefFreeFunction)
{
    World world;
    
    const auto bA = world.CreateBody();
    ASSERT_NE(bA, nullptr);
    const auto bB = world.CreateBody();
    ASSERT_NE(bB, nullptr);

    auto def = MouseJointDef{};
    def.bodyA = bA;
    def.bodyB = bB;
    def.userData = reinterpret_cast<void*>(71);
    def.target = Length2(-1.4_m, -2_m);
    def.maxForce = 3_N;
    def.frequency = 67_Hz;
    def.dampingRatio = Real(0.8);

    const auto joint = MouseJoint{def};
    const auto got = GetMouseJointDef(joint);
    
    EXPECT_EQ(def.bodyA, got.bodyA);
    EXPECT_EQ(def.bodyB, got.bodyB);
    EXPECT_EQ(def.userData, got.userData);
    EXPECT_EQ(def.target, got.target);
    EXPECT_EQ(def.maxForce, got.maxForce);
    EXPECT_EQ(def.frequency, got.frequency);
    EXPECT_EQ(def.dampingRatio, got.dampingRatio);
}
