/*
 * Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/Box2D
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
#include <Box2D/Dynamics/Joints/MouseJoint.hpp>

using namespace box2d;

TEST(MouseJoint, ByteSize)
{
    switch (sizeof(RealNum))
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
    EXPECT_EQ(joint.GetReactionForce(RealNum{1} * Hertz), Force2D{Vec2_zero * Kilogram * MeterPerSquareSecond});
    EXPECT_EQ(joint.GetReactionTorque(RealNum{1} * Hertz), Torque{0});
    EXPECT_EQ(joint.GetUserData(), nullptr);
    EXPECT_FALSE(joint.GetCollideConnected());
    EXPECT_FALSE(IsValid(joint.GetLocalAnchorB()));
    EXPECT_EQ(joint.GetTarget(), def.target);
    EXPECT_EQ(joint.GetMaxForce(), def.maxForce);
    EXPECT_EQ(joint.GetFrequency(), def.frequency);
    EXPECT_EQ(joint.GetDampingRatio(), def.dampingRatio);
}
