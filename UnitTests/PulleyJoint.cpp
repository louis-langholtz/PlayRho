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
#include <PlayRho/Dynamics/Joints/PulleyJointConf.hpp>
#include <PlayRho/Dynamics/World.hpp>

using namespace playrho;
using namespace playrho::d2;

TEST(PulleyJointConf, DefaultConstruction)
{
    PulleyJointConf def;
    
    EXPECT_EQ(def.bodyA, InvalidBodyID);
    EXPECT_EQ(def.bodyB, InvalidBodyID);
    EXPECT_EQ(def.collideConnected, true);
    EXPECT_EQ(def.userData, nullptr);
    
    EXPECT_EQ(def.localAnchorA, (Length2{-1_m, 0_m}));
    EXPECT_EQ(def.localAnchorB, (Length2{+1_m, 0_m}));
    EXPECT_EQ(def.lengthA, 0_m);
    EXPECT_EQ(def.lengthB, 0_m);
    EXPECT_EQ(def.ratio, Real(1));
}

TEST(PulleyJointConf, UseRatio)
{
    const auto value = Real(31);
    EXPECT_NE(PulleyJointConf{}.ratio, value);
    EXPECT_EQ(PulleyJointConf{}.UseRatio(value).ratio, value);
}

TEST(PulleyJointConf, ByteSize)
{
    switch (sizeof(Real))
    {
        case  4:
#if defined(_WIN32) && !defined(_WIN64)
            EXPECT_EQ(sizeof(PulleyJointConf), std::size_t(108));
#else
            EXPECT_EQ(sizeof(PulleyJointConf), std::size_t(104));
#endif
            break;
        case  8: EXPECT_EQ(sizeof(PulleyJointConf), std::size_t(192)); break;
        case 16: EXPECT_EQ(sizeof(PulleyJointConf), std::size_t(368)); break;
        default: FAIL(); break;
    }
}

TEST(PulleyJoint, Construction)
{
    PulleyJointConf def;
    Joint joint{def};

    EXPECT_EQ(GetType(joint), GetTypeID<PulleyJointConf>());
    EXPECT_EQ(GetBodyA(joint), def.bodyA);
    EXPECT_EQ(GetBodyB(joint), def.bodyB);
    EXPECT_EQ(GetCollideConnected(joint), def.collideConnected);
    EXPECT_EQ(GetUserData(joint), def.userData);
    EXPECT_EQ(GetLinearReaction(joint), Momentum2{});
    EXPECT_EQ(GetAngularReaction(joint), AngularMomentum{0});

    EXPECT_EQ(GetGroundAnchorA(joint), def.groundAnchorA);
    EXPECT_EQ(GetGroundAnchorB(joint), def.groundAnchorB);
    EXPECT_EQ(GetLocalAnchorA(joint), def.localAnchorA);
    EXPECT_EQ(GetLocalAnchorB(joint), def.localAnchorB);
#if 0
    EXPECT_EQ(GetLengthA(joint), def.lengthA);
    EXPECT_EQ(GetLengthB(joint), def.lengthB);
#endif
    EXPECT_EQ(GetRatio(joint), def.ratio);
}

TEST(PulleyJoint, GetAnchorAandB)
{
    auto world = World{};
    
    const auto loc0 = Length2{+1_m, -3_m};
    const auto loc1 = Length2{-2_m, Real(+1.2f) * Meter};
    
    const auto b0 = world.CreateBody(BodyConf{}.UseLocation(loc0));
    const auto b1 = world.CreateBody(BodyConf{}.UseLocation(loc1));
    
    auto jd = PulleyJointConf{};
    jd.bodyA = b0;
    jd.bodyB = b1;
    jd.localAnchorA = Length2(4_m, 5_m);
    jd.localAnchorB = Length2(6_m, 7_m);
    
    auto joint = Joint{jd};
    ASSERT_EQ(GetLocalAnchorA(joint), jd.localAnchorA);
    ASSERT_EQ(GetLocalAnchorB(joint), jd.localAnchorB);
#if 0
    EXPECT_EQ(joint.GetAnchorA(world), loc0 + jd.localAnchorA);
    EXPECT_EQ(joint.GetAnchorB(world), loc1 + jd.localAnchorB);
#endif
}

TEST(PulleyJoint, ShiftOrigin)
{
    PulleyJointConf def;
    Joint joint{def};
    
    ASSERT_EQ(GetGroundAnchorA(joint), def.groundAnchorA);
    ASSERT_EQ(GetGroundAnchorB(joint), def.groundAnchorB);
    
    const auto newOrigin = Length2{1_m, 1_m};

    EXPECT_TRUE(ShiftOrigin(joint, newOrigin));
    EXPECT_EQ(GetGroundAnchorA(joint), def.groundAnchorA - newOrigin);
    EXPECT_EQ(GetGroundAnchorB(joint), def.groundAnchorB - newOrigin);
}

TEST(PulleyJoint, GetCurrentLength)
{
    auto world = World{};

    const auto loc0 = Length2{+1_m, -3_m};
    const auto loc1 = Length2{-2_m, Real(+1.2f) * Meter};

    const auto b0 = world.CreateBody(BodyConf{}.UseLocation(loc0));
    const auto b1 = world.CreateBody(BodyConf{}.UseLocation(loc1));

    auto jd = PulleyJointConf{};
    jd.bodyA = b0;
    jd.bodyB = b1;
    jd.localAnchorA = Length2(4_m, 5_m);
    jd.localAnchorB = Length2(6_m, 7_m);

    auto joint = Joint{jd};
    ASSERT_EQ(GetLocalAnchorA(joint), jd.localAnchorA);
    ASSERT_EQ(GetLocalAnchorB(joint), jd.localAnchorB);
    ASSERT_EQ(GetGroundAnchorA(joint), jd.groundAnchorA);
    ASSERT_EQ(GetGroundAnchorB(joint), jd.groundAnchorB);

#if 0
    const auto lenA = GetMagnitude(GetWorldPoint(*GetBodyA(joint), jd.localAnchorA - jd.groundAnchorA));
    const auto lenB = GetMagnitude(GetWorldPoint(*GetBodyB(joint), jd.localAnchorB - jd.groundAnchorB));
    EXPECT_EQ(GetCurrentLengthA(joint), lenA);
    EXPECT_EQ(GetCurrentLengthB(joint), lenB);
#endif
}
