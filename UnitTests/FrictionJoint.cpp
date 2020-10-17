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

#include <PlayRho/Dynamics/Joints/FrictionJointConf.hpp>
#include <PlayRho/Dynamics/Joints/Joint.hpp>

#include <PlayRho/Collision/Shapes/DiskShapeConf.hpp>
#include <PlayRho/Dynamics/World.hpp>
#include <PlayRho/Dynamics/WorldBody.hpp>
#include <PlayRho/Dynamics/BodyConf.hpp>
#include <PlayRho/Dynamics/StepConf.hpp>

using namespace playrho;
using namespace playrho::d2;

TEST(FrictionJointConf, ByteSize)
{
    switch (sizeof(Real))
    {
        case  4:
#if defined(_WIN32) && !defined(_WIN64)
            EXPECT_EQ(sizeof(FrictionJointConf), std::size_t(84));
#elif defined(_WIN64)
            EXPECT_EQ(sizeof(FrictionJointConf), std::size_t(104));
#else
            EXPECT_EQ(sizeof(FrictionJointConf), std::size_t(88));
#endif
            break;
        case  8: EXPECT_EQ(sizeof(FrictionJointConf), std::size_t(160)); break;
        case 16: EXPECT_EQ(sizeof(FrictionJointConf), std::size_t(304)); break;
        default: FAIL(); break;
    }
}

TEST(FrictionJointConf, DefaultConstruction)
{
    FrictionJointConf def{};

    EXPECT_EQ(def.bodyA, InvalidBodyID);
    EXPECT_EQ(def.bodyB, InvalidBodyID);
    EXPECT_EQ(def.collideConnected, false);
    EXPECT_EQ(def.userData, nullptr);
    
    EXPECT_EQ(def.localAnchorA, (Length2{}));
    EXPECT_EQ(def.localAnchorB, (Length2{}));
    EXPECT_EQ(def.maxForce, 0_N);
    EXPECT_EQ(def.maxTorque, 0_Nm);
}

TEST(FrictionJointConf, InitializingConstructor)
{
    World world{};
    const auto p1 = Length2{-1_m, 0_m};
    const auto p2 = Length2{+1_m, 0_m};
    const auto b1 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p1));
    const auto b2 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p2));
    const auto anchor = Length2{0_m, 0_m};
    const auto def = GetFrictionJointConf(world, b1, b2, anchor);
    EXPECT_EQ(def.bodyA, b1);
    EXPECT_EQ(def.bodyB, b2);
    EXPECT_EQ(def.localAnchorA, GetLocalPoint(world, b1, anchor));
    EXPECT_EQ(def.localAnchorB, GetLocalPoint(world, b2, anchor));
}

TEST(FrictionJoint, Construction)
{
    auto world = World{};
    const auto b0 = world.CreateBody();
    const auto b1 = world.CreateBody();

    auto def = GetFrictionJointConf(world, b0, b1, Length2{});
    const auto joint = Joint{def};

    EXPECT_EQ(GetType(joint), GetTypeID<FrictionJointConf>());
    EXPECT_EQ(GetBodyA(joint), def.bodyA);
    EXPECT_EQ(GetBodyB(joint), def.bodyB);
    EXPECT_EQ(GetCollideConnected(joint), def.collideConnected);
    EXPECT_EQ(GetUserData(joint), def.userData);
    EXPECT_EQ(GetLinearReaction(joint), Momentum2{});
    EXPECT_EQ(GetAngularReaction(joint), AngularMomentum{0});
    
    EXPECT_EQ(GetLocalAnchorA(joint), def.localAnchorA);
    EXPECT_EQ(GetLocalAnchorB(joint), def.localAnchorB);
    EXPECT_EQ(GetMaxForce(joint), def.maxForce);
    EXPECT_EQ(GetMaxTorque(joint), def.maxTorque);
}

TEST(FrictionJoint, GetFrictionJointConf)
{
    auto world = World{};
    const auto b0 = world.CreateBody();
    const auto b1 = world.CreateBody();

    auto def = GetFrictionJointConf(world, b0, b1, Length2{});
    const auto joint = Joint{def};

    ASSERT_EQ(GetType(joint), GetTypeID<FrictionJointConf>());
    ASSERT_EQ(GetBodyA(joint), def.bodyA);
    ASSERT_EQ(GetBodyB(joint), def.bodyB);
    ASSERT_EQ(GetCollideConnected(joint), def.collideConnected);
    ASSERT_EQ(GetUserData(joint), def.userData);

    ASSERT_EQ(GetLocalAnchorA(joint), def.localAnchorA);
    ASSERT_EQ(GetLocalAnchorB(joint), def.localAnchorB);
    ASSERT_EQ(GetMaxForce(joint), def.maxForce);
    ASSERT_EQ(GetMaxTorque(joint), def.maxTorque);

    const auto cdef = GetFrictionJointConf(joint);
    EXPECT_EQ(cdef.bodyA, b0);
    EXPECT_EQ(cdef.bodyB, b1);
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
    World world{};
    const auto p1 = Length2{-1_m, 0_m};
    const auto p2 = Length2{+1_m, 0_m};
    const auto b1 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p1));
    const auto b2 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p2));
    world.CreateFixture(b1, Shape{circle});
    world.CreateFixture(b2, Shape{circle});
    auto jd = FrictionJointConf{};
    jd.bodyA = b1;
    jd.bodyB = b2;
    ASSERT_NE(world.CreateJoint(Joint{jd}), InvalidJointID);
    auto stepConf = StepConf{};
 
    stepConf.doWarmStart = true;
    world.Step(stepConf);
    EXPECT_NEAR(double(Real{GetX(GetLocation(world, b1)) / 1_m}), -1.0, 0.001);
    EXPECT_NEAR(double(Real{GetY(GetLocation(world, b1)) / 1_m}), 0.0, 0.001);
    EXPECT_NEAR(double(Real{GetX(GetLocation(world, b2)) / 1_m}), +1.0, 0.01);
    EXPECT_NEAR(double(Real{GetY(GetLocation(world, b2)) / 1_m}), 0.0, 0.01);
    EXPECT_EQ(GetAngle(world, b1), 0_deg);
    EXPECT_EQ(GetAngle(world, b2), 0_deg);

    stepConf.doWarmStart = false;
    world.Step(stepConf);
    EXPECT_NEAR(double(Real{GetX(GetLocation(world, b1)) / 1_m}), -1.0, 0.001);
    EXPECT_NEAR(double(Real{GetY(GetLocation(world, b1)) / 1_m}), 0.0, 0.001);
    EXPECT_NEAR(double(Real{GetX(GetLocation(world, b2)) / 1_m}), +1.0, 0.01);
    EXPECT_NEAR(double(Real{GetY(GetLocation(world, b2)) / 1_m}), 0.0, 0.01);
    EXPECT_EQ(GetAngle(world, b1), 0_deg);
    EXPECT_EQ(GetAngle(world, b2), 0_deg);
}
