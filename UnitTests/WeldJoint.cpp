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

#include <PlayRho/Dynamics/Joints/WeldJoint.hpp>
#include <PlayRho/Dynamics/Joints/TypeJointVisitor.hpp>
#include <PlayRho/Dynamics/BodyConf.hpp>
#include <PlayRho/Dynamics/World.hpp>
#include <PlayRho/Dynamics/WorldMisc.hpp>
#include <PlayRho/Dynamics/WorldBody.hpp>
#include <PlayRho/Dynamics/WorldJoint.hpp>
#include <PlayRho/Dynamics/StepConf.hpp>
#include <PlayRho/Collision/Shapes/DiskShapeConf.hpp>

using namespace playrho;
using namespace playrho::d2;

TEST(WeldJoint, Traits)
{
    EXPECT_FALSE((IsIterable<WeldJoint>::value));
    EXPECT_FALSE((IsAddable<WeldJoint>::value));
    EXPECT_FALSE((IsAddable<WeldJoint,WeldJoint>::value));
}

TEST(WeldJointConf, ByteSize)
{
    switch (sizeof(Real))
    {
        case  4:
#if defined(_WIN32) && !defined(_WIN64)
            EXPECT_EQ(sizeof(WeldJointConf), std::size_t(48));
#else
            EXPECT_EQ(sizeof(WeldJointConf), std::size_t(48));
#endif
            break;
        case  8: EXPECT_EQ(sizeof(WeldJointConf), std::size_t(96)); break;
        case 16: EXPECT_EQ(sizeof(WeldJointConf), std::size_t(160)); break;
        default: FAIL(); break;
    }
}

TEST(WeldJointConf, DefaultConstruction)
{
    WeldJointConf def{};
    
    EXPECT_EQ(def.type, JointType::Weld);
    EXPECT_EQ(def.bodyA, InvalidBodyID);
    EXPECT_EQ(def.bodyB, InvalidBodyID);
    EXPECT_EQ(def.collideConnected, false);
    EXPECT_EQ(def.userData, nullptr);
    
    EXPECT_EQ(def.localAnchorA, (Length2{}));
    EXPECT_EQ(def.localAnchorB, (Length2{}));
    EXPECT_EQ(def.referenceAngle, 0_deg);
    EXPECT_EQ(def.frequency, 0_Hz);
    EXPECT_EQ(def.dampingRatio, Real(0));
}

TEST(WeldJoint, ByteSize)
{
    switch (sizeof(Real))
    {
        case  4:
#if defined(_WIN64)
            EXPECT_EQ(sizeof(WeldJoint), std::size_t(144));
#elif defined(_WIN32)
            EXPECT_EQ(sizeof(WeldJoint), std::size_t(120));
#else
            EXPECT_EQ(sizeof(WeldJoint), std::size_t(128));
#endif
            break;
        case  8: EXPECT_EQ(sizeof(WeldJoint), std::size_t(240)); break;
        case 16: EXPECT_EQ(sizeof(WeldJoint), std::size_t(448)); break;
        default: FAIL(); break;
    }
}

TEST(WeldJoint, Construction)
{
    WeldJointConf def;
    WeldJoint joint{def};
    
    EXPECT_EQ(GetType(joint), def.type);
    EXPECT_EQ(joint.GetBodyA(), def.bodyA);
    EXPECT_EQ(joint.GetBodyB(), def.bodyB);
    EXPECT_EQ(joint.GetCollideConnected(), def.collideConnected);
    EXPECT_EQ(joint.GetUserData(), def.userData);
    EXPECT_EQ(joint.GetLinearReaction(), Momentum2{});
    EXPECT_EQ(joint.GetAngularReaction(), AngularMomentum{0});

    EXPECT_EQ(joint.GetLocalAnchorA(), def.localAnchorA);
    EXPECT_EQ(joint.GetLocalAnchorB(), def.localAnchorB);
    EXPECT_EQ(joint.GetReferenceAngle(), def.referenceAngle);
    EXPECT_EQ(joint.GetFrequency(), def.frequency);
    EXPECT_EQ(joint.GetDampingRatio(), def.dampingRatio);
    
    TypeJointVisitor visitor;
    joint.Accept(visitor);
    EXPECT_EQ(visitor.GetType().value(), JointType::Weld);
}

TEST(WeldJoint, GetWeldJointConf)
{
    auto world = World{};
    const auto bodyA = world.CreateBody();
    const auto bodyB = world.CreateBody();
    const auto anchor = Length2(2_m, 1_m);
    const auto def = GetWeldJointConf(world, bodyA, bodyB, anchor);
    WeldJoint joint{def};

    ASSERT_EQ(GetType(joint), def.type);
    ASSERT_EQ(joint.GetBodyA(), def.bodyA);
    ASSERT_EQ(joint.GetBodyB(), def.bodyB);
    ASSERT_EQ(joint.GetCollideConnected(), def.collideConnected);
    ASSERT_EQ(joint.GetUserData(), def.userData);
    
    ASSERT_EQ(joint.GetLocalAnchorA(), def.localAnchorA);
    ASSERT_EQ(joint.GetLocalAnchorB(), def.localAnchorB);
    ASSERT_EQ(joint.GetReferenceAngle(), def.referenceAngle);
    ASSERT_EQ(joint.GetFrequency(), def.frequency);
    ASSERT_EQ(joint.GetDampingRatio(), def.dampingRatio);
    
    const auto cdef = GetWeldJointConf(joint);
    EXPECT_EQ(cdef.type, JointType::Weld);
    EXPECT_EQ(cdef.bodyA, bodyA);
    EXPECT_EQ(cdef.bodyB, bodyB);
    EXPECT_EQ(cdef.collideConnected, false);
    EXPECT_EQ(cdef.userData, nullptr);
    
    EXPECT_EQ(cdef.localAnchorA, anchor);
    EXPECT_EQ(cdef.localAnchorB, anchor);
    EXPECT_EQ(def.referenceAngle, 0_deg);
    EXPECT_EQ(def.frequency, 0_Hz);
    EXPECT_EQ(def.dampingRatio, Real(0));
}

TEST(WeldJoint, WithDynamicCircles)
{
    const auto circle = Shape{DiskShapeConf{}.UseRadius(0.2_m)};
    auto world = World{};
    const auto p1 = Length2{-1_m, 0_m};
    const auto p2 = Length2{+1_m, 0_m};
    const auto b1 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p1));
    const auto b2 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p2));
    world.CreateFixture(b1, circle);
    world.CreateFixture(b2, circle);
    const auto anchor = Length2(2_m, 1_m);
    const auto jd = GetWeldJointConf(world, b1, b2, anchor);
    world.CreateJoint(jd);
    Step(world, 1_s);
    EXPECT_NEAR(double(Real{GetX(GetLocation(world, b1)) / Meter}), -1.0, 0.001);
    EXPECT_NEAR(double(Real{GetY(GetLocation(world, b1)) / Meter}), 0.0, 0.001);
    EXPECT_NEAR(double(Real{GetX(GetLocation(world, b2)) / Meter}), +1.0, 0.01);
    EXPECT_NEAR(double(Real{GetY(GetLocation(world, b2)) / Meter}), 0.0, 0.01);
    EXPECT_EQ(GetAngle(world, b1), 0_deg);
    EXPECT_EQ(GetAngle(world, b2), 0_deg);
}

TEST(WeldJoint, WithDynamicCircles2)
{
    const auto circle = Shape{DiskShapeConf{}.UseRadius(0.2_m)};
    auto world = World{};
    const auto p1 = Length2{-1_m, 0_m};
    const auto p2 = Length2{+1_m, 0_m};
    const auto b1 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p1));
    const auto b2 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p2));
    world.CreateFixture(b1, circle);
    world.CreateFixture(b2, circle);
    const auto anchor = Length2(2_m, 1_m);
    const auto jd = GetWeldJointConf(world, b1, b2, anchor).UseFrequency(10_Hz);
    const auto joint = world.CreateJoint(jd);
    ASSERT_NE(joint, InvalidJointID);
    ASSERT_EQ(GetFrequency(world, joint), 10_Hz);
    auto stepConf = StepConf{};

    stepConf.doWarmStart = true;
    world.Step(stepConf);
    EXPECT_NEAR(double(Real{GetX(GetLocation(world, b1)) / Meter}), -1.0, 0.001);
    EXPECT_NEAR(double(Real{GetY(GetLocation(world, b1)) / Meter}), 0.0, 0.001);
    EXPECT_NEAR(double(Real{GetX(GetLocation(world, b2)) / Meter}), +1.0, 0.01);
    EXPECT_NEAR(double(Real{GetY(GetLocation(world, b2)) / Meter}), 0.0, 0.01);
    EXPECT_EQ(GetAngle(world, b1), 0_deg);
    EXPECT_EQ(GetAngle(world, b2), 0_deg);

    stepConf.doWarmStart = false;
    world.Step(stepConf);
    EXPECT_NEAR(double(Real{GetX(GetLocation(world, b1)) / Meter}), -1.0, 0.001);
    EXPECT_NEAR(double(Real{GetY(GetLocation(world, b1)) / Meter}), 0.0, 0.001);
    EXPECT_NEAR(double(Real{GetX(GetLocation(world, b2)) / Meter}), +1.0, 0.01);
    EXPECT_NEAR(double(Real{GetY(GetLocation(world, b2)) / Meter}), 0.0, 0.01);
    EXPECT_EQ(GetAngle(world, b1), 0_deg);
    EXPECT_EQ(GetAngle(world, b2), 0_deg);
}

TEST(WeldJoint, GetAnchorAandB)
{
    auto world = World{};
    
    const auto loc1 = Length2{+1_m, -3_m};
    const auto loc2 = Length2{-2_m, Real(+1.2f) * Meter};
    const auto anchor = Length2(2_m, 1_m);

    const auto b1 = world.CreateBody(BodyConf{}.UseLocation(loc1));
    const auto b2 = world.CreateBody(BodyConf{}.UseLocation(loc2));

    auto jd = GetWeldJointConf(world, b1, b2, anchor);
    jd.localAnchorA = Length2(4_m, 5_m);
    jd.localAnchorB = Length2(6_m, 7_m);
    const auto joint = world.CreateJoint(jd);
    ASSERT_NE(joint, InvalidJointID);

    ASSERT_EQ(GetLocalAnchorA(world, joint), jd.localAnchorA);
    ASSERT_EQ(GetLocalAnchorB(world, joint), jd.localAnchorB);
    EXPECT_EQ(GetAnchorA(world, joint), loc1 + jd.localAnchorA);
    EXPECT_EQ(GetAnchorB(world, joint), loc2 + jd.localAnchorB);
}
