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

#include <PlayRho/Dynamics/Joints/RopeJointConf.hpp>
#include <PlayRho/Dynamics/Joints/Joint.hpp>

#include <PlayRho/Dynamics/BodyConf.hpp>
#include <PlayRho/Dynamics/World.hpp>
#include <PlayRho/Dynamics/WorldJoint.hpp>
#include <PlayRho/Dynamics/WorldBody.hpp>
#include <PlayRho/Dynamics/StepConf.hpp>
#include <PlayRho/Collision/Shapes/DiskShapeConf.hpp>

using namespace playrho;
using namespace playrho::d2;

TEST(RopeJointConf, ByteSize)
{
    switch (sizeof(Real))
    {
        case  4:
#if defined(_WIN32) && !defined(_WIN64)
            EXPECT_EQ(sizeof(RopeJointConf), std::size_t(72));
#else
            EXPECT_EQ(sizeof(RopeJointConf), std::size_t(80));
#endif
            break;
        case  8: EXPECT_EQ(sizeof(RopeJointConf), std::size_t(136)); break;
        case 16: EXPECT_EQ(sizeof(RopeJointConf), std::size_t(256)); break;
        default: FAIL(); break;
    }
}

TEST(RopeJointConf, DefaultConstruction)
{
    RopeJointConf def{};

    EXPECT_EQ(def.bodyA, InvalidBodyID);
    EXPECT_EQ(def.bodyB, InvalidBodyID);
    EXPECT_EQ(def.collideConnected, false);
    EXPECT_EQ(def.userData, nullptr);

    EXPECT_EQ(def.localAnchorA, Length2(-1_m, 0_m));
    EXPECT_EQ(def.localAnchorB, Length2(+1_m, 0_m));
    EXPECT_EQ(def.maxLength, 0_m);
}

TEST(RopeJoint, Construction)
{
    World world;
    const auto b0 = world.CreateBody();
    const auto b1 = world.CreateBody();

    auto def = RopeJointConf{b0, b1};
    Joint joint{def};

    EXPECT_EQ(GetType(joint), GetTypeID<RopeJointConf>());
    EXPECT_EQ(GetBodyA(joint), def.bodyA);
    EXPECT_EQ(GetBodyB(joint), def.bodyB);
    EXPECT_EQ(GetCollideConnected(joint), def.collideConnected);
    EXPECT_EQ(GetUserData(joint), def.userData);
    EXPECT_EQ(GetLinearReaction(joint), Momentum2{});
    EXPECT_EQ(GetAngularReaction(joint), AngularMomentum{0});

    const auto id = world.CreateJoint(joint);
    EXPECT_EQ(GetLocalAnchorA(joint), def.localAnchorA);
    EXPECT_EQ(GetLocalAnchorB(joint), def.localAnchorB);
    EXPECT_EQ(GetAnchorA(world, id), Length2(-1_m, 0_m));
    EXPECT_EQ(GetAnchorB(world, id), Length2(+1_m, 0_m));
    EXPECT_EQ(GetLimitState(joint), LimitState::e_inactiveLimit);
    const auto conf = TypeCast<RopeJointConf>(GetJoint(world, id));
    EXPECT_EQ(GetMaxLength(conf), def.maxLength);
}

TEST(RopeJoint, GetRopeJointConf)
{
    auto world = World{};
    const auto bodyA = world.CreateBody();
    const auto bodyB = world.CreateBody();
    auto def = RopeJointConf{bodyA, bodyB};
    const auto localAnchorA = Length2{-2_m, 0_m};
    const auto localAnchorB = Length2{+2_m, 0_m};
    def.localAnchorA = localAnchorA;
    def.localAnchorB = localAnchorB;
    const auto joint = Joint{def};
    
    ASSERT_EQ(GetType(joint), GetTypeID<RopeJointConf>());
    ASSERT_EQ(GetBodyA(joint), def.bodyA);
    ASSERT_EQ(GetBodyB(joint), def.bodyB);
    ASSERT_EQ(GetCollideConnected(joint), def.collideConnected);
    ASSERT_EQ(GetUserData(joint), def.userData);
    
    ASSERT_EQ(GetLocalAnchorA(joint), def.localAnchorA);
    ASSERT_EQ(GetLocalAnchorB(joint), def.localAnchorB);
    const auto conf = TypeCast<RopeJointConf>(joint);
    ASSERT_EQ(GetMaxLength(conf), def.maxLength);
    
    const auto cdef = GetRopeJointConf(joint);
    EXPECT_EQ(cdef.bodyA, bodyA);
    EXPECT_EQ(cdef.bodyB, bodyB);
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
    world.CreateFixture(b1, circle);
    world.CreateFixture(b2, circle);
    const auto jd = RopeJointConf{b1, b2};
    ASSERT_NE(world.CreateJoint(jd), InvalidJointID);

    auto stepConf = StepConf{};

    stepConf.doWarmStart = true;
    world.Step(stepConf);
    EXPECT_GT(GetX(GetLocation(world, b1)), -1_m);
    EXPECT_EQ(GetY(GetLocation(world, b1)), 0_m);
    EXPECT_LT(GetX(GetLocation(world, b2)), +1_m);
    EXPECT_EQ(GetY(GetLocation(world, b2)), 0_m);
    EXPECT_EQ(GetAngle(world, b1), 0_deg);
    EXPECT_EQ(GetAngle(world, b2), 0_deg);
    
    stepConf.doWarmStart = false;
    world.Step(stepConf);
    EXPECT_GT(GetX(GetLocation(world, b1)), -1_m);
    EXPECT_EQ(GetY(GetLocation(world, b1)), 0_m);
    EXPECT_LT(GetX(GetLocation(world, b2)), +1_m);
    EXPECT_EQ(GetY(GetLocation(world, b2)), 0_m);
    EXPECT_EQ(GetAngle(world, b1), 0_deg);
    EXPECT_EQ(GetAngle(world, b2), 0_deg);
    
    stepConf.doWarmStart = true;
    stepConf.linearSlop = 10_m;
    world.Step(stepConf);
    EXPECT_GT(GetX(GetLocation(world, b1)), -1_m);
    EXPECT_EQ(GetY(GetLocation(world, b1)), 0_m);
    EXPECT_LT(GetX(GetLocation(world, b2)), +1_m);
    EXPECT_EQ(GetY(GetLocation(world, b2)), 0_m);
    EXPECT_EQ(GetAngle(world, b1), 0_deg);
    EXPECT_EQ(GetAngle(world, b2), 0_deg);
}
