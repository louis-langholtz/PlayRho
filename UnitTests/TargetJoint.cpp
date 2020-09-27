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

#include "UnitTests.hpp"
#include <PlayRho/Dynamics/Joints/TargetJoint.hpp>
#include <PlayRho/Dynamics/Joints/TypeJointVisitor.hpp>
#include <PlayRho/Dynamics/World.hpp>

using namespace playrho;
using namespace playrho::d2;

TEST(TargetJointConf, UseMaxForce)
{
    const auto value = Force(19_N);
    EXPECT_NE(TargetJointConf{}.maxForce, value);
    EXPECT_EQ(TargetJointConf{}.UseMaxForce(value).maxForce, value);
}

TEST(TargetJointConf, UseFrequency)
{
    const auto value = 19_Hz;
    EXPECT_NE(TargetJointConf{}.frequency, value);
    EXPECT_EQ(TargetJointConf{}.UseFrequency(value).frequency, value);
}

TEST(TargetJointConf, UseDampingRatio)
{
    const auto value = Real(0.4);
    EXPECT_NE(TargetJointConf{}.dampingRatio, value);
    EXPECT_EQ(TargetJointConf{}.UseDampingRatio(value).dampingRatio, value);
}

TEST(TargetJoint, ByteSize)
{
    switch (sizeof(Real))
    {
        case  4:
#if defined(_WIN32) && !defined(_WIN64)
            EXPECT_EQ(sizeof(TargetJoint), std::size_t(92));
#else
            EXPECT_EQ(sizeof(TargetJoint), std::size_t(88));
#endif
            break;
        case  8: EXPECT_EQ(sizeof(TargetJoint), std::size_t(184)); break;
        case 16: EXPECT_EQ(sizeof(TargetJoint), std::size_t(336)); break;
        default: FAIL(); break;
    }
}

TEST(TargetJoint, IsOkay)
{
    auto def = TargetJointConf{};

    ASSERT_FALSE(Joint::IsOkay(def));
    def.bodyA = static_cast<BodyID>(1u);
    def.bodyB = static_cast<BodyID>(2u);
    ASSERT_TRUE(Joint::IsOkay(def));

    EXPECT_TRUE(TargetJoint::IsOkay(def));
    def.bodyA = InvalidBodyID;
    EXPECT_FALSE(TargetJoint::IsOkay(def));
}

TEST(TargetJoint, DefaultInitialized)
{
    const auto def = TargetJointConf{};
    auto joint = TargetJoint{def};

    EXPECT_EQ(GetType(joint), JointType::Target);
    EXPECT_EQ(joint.GetBodyA(), def.bodyA);
    EXPECT_EQ(joint.GetBodyB(), def.bodyB);
    EXPECT_EQ(joint.GetLocalAnchorB(), def.anchor);
    //EXPECT_FALSE(IsValid(joint.GetAnchorB()));
    EXPECT_EQ(joint.GetLinearReaction(), Momentum2{});
    EXPECT_EQ(joint.GetAngularReaction(), AngularMomentum{0});
    EXPECT_EQ(joint.GetUserData(), nullptr);
    EXPECT_FALSE(joint.GetCollideConnected());
    //EXPECT_FALSE(IsValid(joint.GetLocalAnchorB()));
    EXPECT_EQ(joint.GetMaxForce(), def.maxForce);
    EXPECT_EQ(joint.GetFrequency(), def.frequency);
    EXPECT_EQ(joint.GetDampingRatio(), def.dampingRatio);

    TypeJointVisitor visitor;
    joint.Accept(visitor);
    EXPECT_EQ(visitor.GetType().value(), JointType::Target);
    EXPECT_TRUE(visitor.GetWritable());
}

TEST(TargetJoint, GetLocalAnchorB)
{
    auto world = World{};
    const auto bA = world.CreateBody();
    const auto bB = world.CreateBody();
    ASSERT_NE(bA, InvalidBodyID);
    ASSERT_NE(bB, InvalidBodyID);
    
    auto def = TargetJointConf{};
    def.bodyA = bA;
    def.bodyB = bB;
    def.userData = reinterpret_cast<void*>(71);
    def.anchor = Length2(-1.4_m, -2_m);
    def.maxForce = 3_N;
    def.frequency = 67_Hz;
    def.dampingRatio = Real(0.8);
    
    const auto joint = TargetJoint{def};
    EXPECT_EQ(joint.GetLocalAnchorB(), def.anchor);
}

TEST(TargetJoint, GetAnchorB)
{
    auto world = World{};
    const auto bA = world.CreateBody();
    const auto bB = world.CreateBody();
    ASSERT_NE(bA, InvalidBodyID);
    ASSERT_NE(bB, InvalidBodyID);
    
    auto def = TargetJointConf{};
    def.bodyA = bA;
    def.bodyB = bB;
    def.userData = reinterpret_cast<void*>(71);
    def.anchor = Length2(-1.4_m, -2_m);
    def.maxForce = 3_N;
    def.frequency = 67_Hz;
    def.dampingRatio = Real(0.8);
    
    const auto joint = TargetJoint{def};
    ASSERT_EQ(joint.GetLocalAnchorB(), def.anchor);
}

TEST(TargetJoint, ShiftOrigin)
{
    auto world = World{};
    const auto bA = world.CreateBody(BodyConf{}.UseLocation(Length2(-1.4_m, -2_m)));
    const auto bB = world.CreateBody();
    ASSERT_NE(bA, InvalidBodyID);
    ASSERT_NE(bB, InvalidBodyID);
    auto def = TargetJointConf{};
    def.bodyA = bA;
    def.bodyB = bB;
    auto joint = TargetJoint{def};
    const auto newOrigin = Length2{1_m, 1_m};
    EXPECT_TRUE(joint.ShiftOrigin(newOrigin));
}

TEST(TargetJointConf, GetTargetJointDefFreeFunction)
{
    World world;
    
    const auto bA = world.CreateBody();
    ASSERT_NE(bA, InvalidBodyID);
    const auto bB = world.CreateBody();
    ASSERT_NE(bB, InvalidBodyID);

    auto def = TargetJointConf{};
    def.bodyA = bA;
    def.bodyB = bB;
    def.userData = reinterpret_cast<void*>(71);
    def.anchor = Length2(-1.4_m, -2_m);
    def.maxForce = 3_N;
    def.frequency = 67_Hz;
    def.dampingRatio = Real(0.8);

    const auto joint = TargetJoint{def};
    const auto got = GetTargetJointConf(joint);
    
    EXPECT_EQ(def.bodyA, got.bodyA);
    EXPECT_EQ(def.bodyB, got.bodyB);
    EXPECT_EQ(def.userData, got.userData);
    EXPECT_EQ(def.anchor, got.anchor);
    EXPECT_EQ(def.maxForce, got.maxForce);
    EXPECT_EQ(def.frequency, got.frequency);
    EXPECT_EQ(def.dampingRatio, got.dampingRatio);
}
