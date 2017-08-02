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
#include <PlayRho/Dynamics/Joints/DistanceJoint.hpp>
#include <PlayRho/Dynamics/World.hpp>
#include <PlayRho/Dynamics/StepConf.hpp>
#include <PlayRho/Dynamics/Body.hpp>
#include <PlayRho/Dynamics/BodyDef.hpp>
#include <PlayRho/Dynamics/Fixture.hpp>
#include <PlayRho/Collision/Shapes/DiskShape.hpp>

using namespace playrho;

TEST(DistanceJoint, ByteSize)
{
    switch (sizeof(Real))
    {
        case  4: EXPECT_EQ(sizeof(DistanceJoint), std::size_t(104)); break;
        case  8: EXPECT_EQ(sizeof(DistanceJoint), std::size_t(176)); break;
        case 16: EXPECT_EQ(sizeof(DistanceJoint), std::size_t(320)); break;
        default: FAIL(); break;
    }
}

TEST(DistanceJointDef, DefaultConstruction)
{
    DistanceJointDef def;

    EXPECT_EQ(def.type, JointType::Distance);
    EXPECT_EQ(def.bodyA, nullptr);
    EXPECT_EQ(def.bodyB, nullptr);
    EXPECT_EQ(def.collideConnected, false);
    EXPECT_EQ(def.userData, nullptr);
    
    EXPECT_EQ(def.localAnchorA, (Length2D{}));
    EXPECT_EQ(def.localAnchorB, (Length2D{}));
    EXPECT_EQ(def.length, Real(1) * Meter);
    EXPECT_EQ(def.frequency, Frequency(0));
    EXPECT_EQ(def.dampingRatio, Real(0));
}

TEST(DistanceJointDef, UseLength)
{
    const auto value = Length(Real(31) * Meter);
    EXPECT_NE(DistanceJointDef{}.length, value);
    EXPECT_EQ(DistanceJointDef{}.UseLength(value).length, value);
}

TEST(DistanceJointDef, UseFrequency)
{
    const auto value = Frequency(Real(19) * Hertz);
    EXPECT_NE(DistanceJointDef{}.frequency, value);
    EXPECT_EQ(DistanceJointDef{}.UseFrequency(value).frequency, value);
}

TEST(DistanceJointDef, UseDampingRatio)
{
    const auto value = Real(0.4);
    EXPECT_NE(DistanceJointDef{}.dampingRatio, value);
    EXPECT_EQ(DistanceJointDef{}.UseDampingRatio(value).dampingRatio, value);
}

TEST(DistanceJoint, Construction)
{
    DistanceJointDef def;
    DistanceJoint joint{def};
    
    EXPECT_EQ(joint.GetType(), def.type);
    EXPECT_EQ(joint.GetBodyA(), def.bodyA);
    EXPECT_EQ(joint.GetBodyB(), def.bodyB);
    EXPECT_EQ(joint.GetCollideConnected(), def.collideConnected);
    EXPECT_EQ(joint.GetUserData(), def.userData);
    
    EXPECT_EQ(joint.GetLocalAnchorA(), def.localAnchorA);
    EXPECT_EQ(joint.GetLocalAnchorB(), def.localAnchorB);
    EXPECT_EQ(joint.GetLength(), def.length);
    EXPECT_EQ(joint.GetFrequency(), def.frequency);
    EXPECT_EQ(joint.GetDampingRatio(), def.dampingRatio);
}

TEST(DistanceJoint, InZeroGravBodiesMoveOutToLength)
{
    World world{WorldDef{}.UseGravity(LinearAcceleration2D{})};

    const auto shape = std::make_shared<DiskShape>(Real{0.2f} * Meter);
    
    const auto location1 = Length2D{-Real(1) * Meter, Real(0) * Meter};
    const auto body1 = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(location1));
    ASSERT_EQ(body1->GetLocation(), location1);
    ASSERT_NE(body1->CreateFixture(shape), nullptr);
    
    const auto location2 = Length2D{+Real(1) * Meter, Real(0) * Meter};
    const auto body2 = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(location2));
    ASSERT_EQ(body2->GetLocation(), location2);
    ASSERT_NE(body2->CreateFixture(shape), nullptr);
    
    DistanceJointDef jointdef;
    jointdef.bodyA = body1;
    jointdef.bodyB = body2;
    jointdef.collideConnected = false;
    jointdef.localAnchorA = Length2D{};
    jointdef.localAnchorB = Length2D{};
    jointdef.length = Real{5} * Meter;
    jointdef.frequency = 0;
    jointdef.dampingRatio = 0;
    EXPECT_NE(world.CreateJoint(jointdef), nullptr);
    
    auto oldDistance = GetLength(body1->GetLocation() - body2->GetLocation());
    
    auto distanceMet = 0u;
    StepConf stepConf;
    for (auto i = 0u; !distanceMet || i < distanceMet + 100; ++i)
    {
        world.Step(stepConf);

        const auto newDistance = GetLength(body1->GetLocation() - body2->GetLocation());
        if (distanceMet)
        {
            EXPECT_NEAR(double(Real{newDistance / Meter}),
                        double(Real{oldDistance / Meter}), 0.01);
        }
        else
        {
            EXPECT_GE(newDistance, oldDistance);
        }
        
        if (!distanceMet && (Abs(newDistance - jointdef.length) < Real{0.01f} * Meter))
        {
            distanceMet = i;
        }
        oldDistance = newDistance;
    }
}

TEST(DistanceJoint, InZeroGravBodiesMoveInToLength)
{
    World world{WorldDef{}.UseGravity(LinearAcceleration2D{0, Real(10) * MeterPerSquareSecond})};
    
    const auto shape = std::make_shared<DiskShape>(Real{0.2f} * Meter);
    shape->SetDensity(Real{1} * KilogramPerSquareMeter);
    
    const auto location1 = Length2D{-Real(10) * Meter, Real(10) * Meter};
    const auto body1 = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(location1));
    ASSERT_EQ(body1->GetLocation(), location1);
    ASSERT_NE(body1->CreateFixture(shape), nullptr);
    
    const auto location2 = Length2D{+Real(10) * Meter, -Real(10) * Meter};
    const auto body2 = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(location2));
    ASSERT_EQ(body2->GetLocation(), location2);
    ASSERT_NE(body2->CreateFixture(shape), nullptr);
    
    DistanceJointDef jointdef;
    jointdef.bodyA = body1;
    jointdef.bodyB = body2;
    jointdef.collideConnected = false;
    jointdef.localAnchorA = Length2D{};
    jointdef.localAnchorB = Length2D{};
    jointdef.length = Real{5} * Meter;
    jointdef.frequency = Real{60} * Hertz;
    jointdef.dampingRatio = 0;
    EXPECT_NE(world.CreateJoint(jointdef), nullptr);
    
    auto oldDistance = GetLength(body1->GetLocation() - body2->GetLocation());
    
    auto distanceMet = 0u;
    StepConf stepConf;
    for (auto i = 0u; !distanceMet || i < distanceMet + 1000; ++i)
    {
        world.Step(stepConf);
        
        const auto newDistance = GetLength(body1->GetLocation() - body2->GetLocation());
        if (!distanceMet && (newDistance - oldDistance) >= Length{0})
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

TEST(DistanceJointDef, GetDistanceJointDefFreeFunction)
{
    World world;
    
    const auto bA = world.CreateBody();
    ASSERT_NE(bA, nullptr);
    const auto bB = world.CreateBody();
    ASSERT_NE(bB, nullptr);
    
    auto def = DistanceJointDef{};
    def.bodyA = bA;
    def.bodyB = bB;
    def.collideConnected = false;
    def.userData = reinterpret_cast<void*>(71);
    def.localAnchorA = Length2D(Real(21) * Meter, Real(-2) * Meter);
    def.localAnchorB = Length2D(Real(13) * Meter, Real(12) * Meter);
    def.length = Real{5} * Meter;
    def.frequency = Real(67) * Hertz;
    def.dampingRatio = Real(0.8);
    
    const auto joint = DistanceJoint{def};
    const auto got = GetDistanceJointDef(joint);
    
    EXPECT_EQ(def.bodyA, got.bodyA);
    EXPECT_EQ(def.bodyB, got.bodyB);
    EXPECT_EQ(def.userData, got.userData);
    EXPECT_EQ(def.localAnchorA, got.localAnchorA);
    EXPECT_EQ(def.localAnchorB, got.localAnchorB);
    EXPECT_EQ(def.length, got.length);
    EXPECT_EQ(def.frequency, got.frequency);
    EXPECT_EQ(def.dampingRatio, got.dampingRatio);
}
