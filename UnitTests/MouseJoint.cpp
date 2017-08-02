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
#include <PlayRho/Dynamics/World.hpp>

using namespace playrho;

TEST(MouseJointDef, UseTarget)
{
    const auto value = Length2D(Real(19) * Meter, Real(-9) * Meter);
    EXPECT_NE(MouseJointDef{}.target, value);
    EXPECT_EQ(MouseJointDef{}.UseTarget(value).target, value);
}

TEST(MouseJointDef, UseMaxForce)
{
    const auto value = Force(Real(19) * Newton);
    EXPECT_NE(MouseJointDef{}.maxForce, value);
    EXPECT_EQ(MouseJointDef{}.UseMaxForce(value).maxForce, value);
}

TEST(MouseJointDef, UseFrequency)
{
    const auto value = Frequency(Real(19) * Hertz);
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
        case  4: EXPECT_EQ(sizeof(MouseJoint), std::size_t(112)); break;
        case  8: EXPECT_EQ(sizeof(MouseJoint), std::size_t(184)); break;
        case 16: EXPECT_EQ(sizeof(MouseJoint), std::size_t(336)); break;
        default: FAIL(); break;
    }
}

TEST(MouseJoint, DefaultInitialized)
{
    const auto def = MouseJointDef{};
    const auto joint = MouseJoint{def};
    
    EXPECT_EQ(joint.GetType(), JointType::Mouse);
    EXPECT_EQ(joint.GetBodyA(), def.bodyA);
    EXPECT_EQ(joint.GetBodyB(), def.bodyB);
    EXPECT_EQ(joint.GetAnchorA(), def.target);
    EXPECT_FALSE(IsValid(joint.GetAnchorB()));
    EXPECT_EQ(joint.GetReactionForce(Real{1} * Hertz), Force2D{});
    EXPECT_EQ(joint.GetReactionTorque(Real{1} * Hertz), Torque{0});
    EXPECT_EQ(joint.GetUserData(), nullptr);
    EXPECT_FALSE(joint.GetCollideConnected());
    EXPECT_FALSE(IsValid(joint.GetLocalAnchorB()));
    EXPECT_EQ(joint.GetTarget(), def.target);
    EXPECT_EQ(joint.GetMaxForce(), def.maxForce);
    EXPECT_EQ(joint.GetFrequency(), def.frequency);
    EXPECT_EQ(joint.GetDampingRatio(), def.dampingRatio);
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
    def.target = Length2D(Real(-1.4) * Meter, Real(-2) * Meter);
    def.maxForce = Real(3) * Newton;
    def.frequency = Real(67) * Hertz;
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
