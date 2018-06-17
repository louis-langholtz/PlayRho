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
#include <PlayRho/Dynamics/Joints/DistanceJoint.hpp>
#include <PlayRho/Dynamics/World.hpp>
#include <PlayRho/Dynamics/StepConf.hpp>
#include <PlayRho/Dynamics/Body.hpp>
#include <PlayRho/Dynamics/BodyConf.hpp>
#include <PlayRho/Dynamics/Fixture.hpp>
#include <PlayRho/Collision/Shapes/DiskShapeConf.hpp>
#include <PlayRho/Dynamics/Joints/TypeJointVisitor.hpp>

using namespace playrho;
using namespace playrho::d2;

TEST(DistanceJoint, ByteSize)
{
    switch (sizeof(Real))
    {
        case  4:
#if defined(_WIN32)
#if defined(_WIN64)
            EXPECT_EQ(sizeof(DistanceJoint), std::size_t(112));
#else
            EXPECT_EQ(sizeof(DistanceJoint), std::size_t(88));
#endif
#else
            EXPECT_EQ(sizeof(DistanceJoint), std::size_t(104));
#endif
            break;
        case  8: EXPECT_EQ(sizeof(DistanceJoint), std::size_t(176)); break;
        case 16: EXPECT_EQ(sizeof(DistanceJoint), std::size_t(320)); break;
        default: FAIL(); break;
    }
}

TEST(DistanceJointConf, DefaultConstruction)
{
    auto def = DistanceJointConf{};

    EXPECT_EQ(def.type, JointType::Distance);
    EXPECT_EQ(def.bodyA, nullptr);
    EXPECT_EQ(def.bodyB, nullptr);
    EXPECT_EQ(def.collideConnected, false);
    EXPECT_EQ(def.userData, nullptr);
    
    EXPECT_EQ(def.localAnchorA, (Length2{}));
    EXPECT_EQ(def.localAnchorB, (Length2{}));
    EXPECT_EQ(def.length, 1_m);
    EXPECT_EQ(def.frequency, 0_Hz);
    EXPECT_EQ(def.dampingRatio, Real(0));
}

TEST(DistanceJointConf, UseLength)
{
    const auto value = Length(31_m);
    EXPECT_NE(DistanceJointConf{}.length, value);
    EXPECT_EQ(DistanceJointConf{}.UseLength(value).length, value);
}

TEST(DistanceJointConf, UseFrequency)
{
    const auto value = 19_Hz;
    EXPECT_NE(DistanceJointConf{}.frequency, value);
    EXPECT_EQ(DistanceJointConf{}.UseFrequency(value).frequency, value);
}

TEST(DistanceJointConf, UseDampingRatio)
{
    const auto value = Real(0.4);
    EXPECT_NE(DistanceJointConf{}.dampingRatio, value);
    EXPECT_EQ(DistanceJointConf{}.UseDampingRatio(value).dampingRatio, value);
}

TEST(DistanceJoint, Construction)
{
    auto world = World{};
    const auto body0 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic));
    const auto body1 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic));

    auto def = DistanceJointConf{body0, body1};
    auto& joint = *static_cast<DistanceJoint*>(world.CreateJoint(def));
    
    EXPECT_EQ(GetType(joint), def.type);
    EXPECT_EQ(joint.GetBodyA(), def.bodyA);
    EXPECT_EQ(joint.GetBodyB(), def.bodyB);
    EXPECT_EQ(joint.GetCollideConnected(), def.collideConnected);
    EXPECT_EQ(joint.GetUserData(), def.userData);
    EXPECT_EQ(joint.GetLinearReaction(), Momentum2{});
    EXPECT_EQ(joint.GetAngularReaction(), AngularMomentum{0});

    EXPECT_EQ(joint.GetLocalAnchorA(), def.localAnchorA);
    EXPECT_EQ(joint.GetLocalAnchorB(), def.localAnchorB);
    EXPECT_EQ(joint.GetLength(), def.length);
    EXPECT_EQ(joint.GetFrequency(), def.frequency);
    EXPECT_EQ(joint.GetDampingRatio(), def.dampingRatio);
    
    TypeJointVisitor visitor;
    joint.Accept(visitor);
    EXPECT_EQ(visitor.GetType().value(), JointType::Distance);
}

TEST(DistanceJoint, ShiftOrigin)
{
    auto world = World{};
    const auto body0 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic));
    const auto body1 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic));
    auto def = DistanceJointConf{body0, body1};
    auto& joint = *static_cast<DistanceJoint*>(world.CreateJoint(def));
    const auto newOrigin = Length2{1_m, 1_m};
    EXPECT_FALSE(joint.ShiftOrigin(newOrigin));
}

TEST(DistanceJoint, InZeroGravBodiesMoveOutToLength)
{
    auto world = World{};
    
    const auto shape = Shape{DiskShapeConf{}.UseRadius(0.2_m)};
    
    const auto location1 = Length2{-1_m, 0_m};
    const auto body1 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic).UseLocation(location1));
    ASSERT_EQ(body1->GetLocation(), location1);
    ASSERT_NE(body1->CreateFixture(shape), nullptr);
    
    const auto location2 = Length2{+1_m, 0_m};
    const auto body2 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic).UseLocation(location2));
    ASSERT_EQ(body2->GetLocation(), location2);
    ASSERT_NE(body2->CreateFixture(shape), nullptr);
    
    auto jointdef = DistanceJointConf{};
    jointdef.bodyA = body1;
    jointdef.bodyB = body2;
    jointdef.collideConnected = false;
    jointdef.localAnchorA = Length2{};
    jointdef.localAnchorB = Length2{};
    jointdef.length = 5_m;
    jointdef.frequency = 0_Hz;
    jointdef.dampingRatio = 0;
    EXPECT_NE(world.CreateJoint(jointdef), nullptr);
    
    auto oldDistance = GetMagnitude(body1->GetLocation() - body2->GetLocation());
    
    auto distanceMet = 0u;
    auto stepConf = StepConf{};
    for (auto i = 0u; !distanceMet || i < distanceMet + 100; ++i)
    {
        world.Step(stepConf);

        const auto newDistance = GetMagnitude(body1->GetLocation() - body2->GetLocation());
        if (distanceMet)
        {
            EXPECT_NEAR(double(Real{newDistance / Meter}),
                        double(Real{oldDistance / Meter}), 0.01);
        }
        else
        {
            EXPECT_GE(newDistance, oldDistance);
        }
        
        if (!distanceMet && (abs(newDistance - jointdef.length) < 0.01_m))
        {
            distanceMet = i;
        }
        oldDistance = newDistance;
    }
}

TEST(DistanceJoint, InZeroGravBodiesMoveInToLength)
{
    auto world = World{};
    
    const auto shape = Shape{DiskShapeConf{}.UseRadius(0.2_m).UseDensity(1_kgpm2)};
    const auto location1 = Length2{-10_m, 10_m};
    const auto body1 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic).UseLocation(location1));
    ASSERT_EQ(body1->GetLocation(), location1);
    ASSERT_NE(body1->CreateFixture(shape), nullptr);
    
    const auto location2 = Length2{+10_m, -10_m};
    const auto body2 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic).UseLocation(location2));
    ASSERT_EQ(body2->GetLocation(), location2);
    ASSERT_NE(body2->CreateFixture(shape), nullptr);
    
    auto jointdef = DistanceJointConf{};
    jointdef.bodyA = body1;
    jointdef.bodyB = body2;
    jointdef.collideConnected = false;
    jointdef.localAnchorA = Length2{};
    jointdef.localAnchorB = Length2{};
    jointdef.length = 5_m;
    jointdef.frequency = 60_Hz;
    jointdef.dampingRatio = 0;
    EXPECT_NE(world.CreateJoint(jointdef), nullptr);
    
    auto oldDistance = GetMagnitude(body1->GetLocation() - body2->GetLocation());
    
    auto distanceMet = 0u;
    auto stepConf = StepConf{};
    SetAccelerations(world, Acceleration{
        LinearAcceleration2{0, 10 * MeterPerSquareSecond}, 0 * RadianPerSquareSecond
    });
    for (auto i = 0u; !distanceMet || i < distanceMet + 1000; ++i)
    {
        world.Step(stepConf);
        
        const auto newDistance = GetMagnitude(body1->GetLocation() - body2->GetLocation());
        if (!distanceMet && (newDistance - oldDistance) >= 0_m)
        {
            distanceMet = i;
        }
        if (distanceMet)
        {
            EXPECT_NEAR(double(Real{newDistance / Meter}),
                        double(Real{oldDistance / Meter}), 2.5);
        }
        else
        {
            EXPECT_LE(newDistance, oldDistance);
        }
        
        oldDistance = newDistance;
    }
    
    EXPECT_NEAR(double(Real{oldDistance / Meter}),
                double(Real{jointdef.length / Meter}), 0.1);
}

TEST(DistanceJointConf, GetDistanceJointDefFreeFunction)
{
    auto world = World{};
    
    const auto bA = world.CreateBody();
    ASSERT_NE(bA, nullptr);
    const auto bB = world.CreateBody();
    ASSERT_NE(bB, nullptr);
    
    auto def = DistanceJointConf{};
    def.bodyA = bA;
    def.bodyB = bB;
    def.collideConnected = false;
    def.userData = reinterpret_cast<void*>(71);
    def.localAnchorA = Length2(21_m, -2_m);
    def.localAnchorB = Length2(13_m, 12_m);
    def.length = 5_m;
    def.frequency = 67_Hz;
    def.dampingRatio = Real(0.8);
    
    auto& joint = *static_cast<DistanceJoint*>(world.CreateJoint(def));
    const auto got = GetDistanceJointConf(joint);
    
    EXPECT_EQ(def.bodyA, got.bodyA);
    EXPECT_EQ(def.bodyB, got.bodyB);
    EXPECT_EQ(def.userData, got.userData);
    EXPECT_EQ(def.localAnchorA, got.localAnchorA);
    EXPECT_EQ(def.localAnchorB, got.localAnchorB);
    EXPECT_EQ(def.length, got.length);
    EXPECT_EQ(def.frequency, got.frequency);
    EXPECT_EQ(def.dampingRatio, got.dampingRatio);
}
