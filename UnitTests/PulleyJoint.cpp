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
#include <PlayRho/Dynamics/Joints/PulleyJoint.hpp>
#include <PlayRho/Dynamics/World.hpp>

using namespace playrho;

TEST(PulleyJointDef, DefaultConstruction)
{
    PulleyJointDef def;
    
    EXPECT_EQ(def.type, JointType::Pulley);
    EXPECT_EQ(def.bodyA, nullptr);
    EXPECT_EQ(def.bodyB, nullptr);
    EXPECT_EQ(def.collideConnected, true);
    EXPECT_EQ(def.userData, nullptr);
    
    EXPECT_EQ(def.localAnchorA, (Length2{-1_m, 0_m}));
    EXPECT_EQ(def.localAnchorB, (Length2{+1_m, 0_m}));
    EXPECT_EQ(def.lengthA, 0_m);
    EXPECT_EQ(def.lengthB, 0_m);
    EXPECT_EQ(def.ratio, Real(1));
}

TEST(PulleyJointDef, UseRatio)
{
    const auto value = Real(31);
    EXPECT_NE(PulleyJointDef{}.ratio, value);
    EXPECT_EQ(PulleyJointDef{}.UseRatio(value).ratio, value);
}

TEST(PulleyJoint, ByteSize)
{
    switch (sizeof(Real))
    {
        case  4: EXPECT_EQ(sizeof(PulleyJoint), std::size_t(128)); break;
        case  8: EXPECT_EQ(sizeof(PulleyJoint), std::size_t(216)); break;
        case 16: EXPECT_EQ(sizeof(PulleyJoint), std::size_t(400)); break;
        default: FAIL(); break;
    }
}

TEST(PulleyJoint, Construction)
{
    PulleyJointDef def;
    PulleyJoint joint{def};
    
    EXPECT_EQ(GetType(joint), def.type);
    EXPECT_EQ(joint.GetBodyA(), def.bodyA);
    EXPECT_EQ(joint.GetBodyB(), def.bodyB);
    EXPECT_EQ(joint.GetCollideConnected(), def.collideConnected);
    EXPECT_EQ(joint.GetUserData(), def.userData);
    EXPECT_EQ(joint.GetLinearReaction(), Momentum2{});
    EXPECT_EQ(joint.GetAngularReaction(), AngularMomentum{0});

    EXPECT_EQ(joint.GetGroundAnchorA(), def.groundAnchorA);
    EXPECT_EQ(joint.GetGroundAnchorB(), def.groundAnchorB);
    EXPECT_EQ(joint.GetLocalAnchorA(), def.localAnchorA);
    EXPECT_EQ(joint.GetLocalAnchorB(), def.localAnchorB);
    EXPECT_EQ(joint.GetLengthA(), def.lengthA);
    EXPECT_EQ(joint.GetLengthB(), def.lengthB);
    EXPECT_EQ(joint.GetRatio(), def.ratio);
}

TEST(PulleyJoint, GetAnchorAandB)
{
    World world;
    
    const auto loc0 = Length2{+1_m, -3_m};
    const auto loc1 = Length2{-2_m, Real(+1.2f) * Meter};
    
    const auto b0 = world.CreateBody(BodyDef{}.UseLocation(loc0));
    const auto b1 = world.CreateBody(BodyDef{}.UseLocation(loc1));
    
    auto jd = PulleyJointDef{};
    jd.bodyA = b0;
    jd.bodyB = b1;
    jd.localAnchorA = Length2(4_m, 5_m);
    jd.localAnchorB = Length2(6_m, 7_m);
    
    auto joint = PulleyJoint{jd};
    ASSERT_EQ(joint.GetLocalAnchorA(), jd.localAnchorA);
    ASSERT_EQ(joint.GetLocalAnchorB(), jd.localAnchorB);
    EXPECT_EQ(joint.GetAnchorA(), loc0 + jd.localAnchorA);
    EXPECT_EQ(joint.GetAnchorB(), loc1 + jd.localAnchorB);
}

TEST(PulleyJoint, ShiftOrigin)
{
    PulleyJointDef def;
    PulleyJoint joint{def};
    
    ASSERT_EQ(joint.GetGroundAnchorA(), def.groundAnchorA);
    ASSERT_EQ(joint.GetGroundAnchorB(), def.groundAnchorB);
    
    const auto newOrigin = Length2{1_m, 1_m};

    joint.ShiftOrigin(newOrigin);

    EXPECT_EQ(joint.GetGroundAnchorA(), def.groundAnchorA - newOrigin);
    EXPECT_EQ(joint.GetGroundAnchorB(), def.groundAnchorB - newOrigin);
}
