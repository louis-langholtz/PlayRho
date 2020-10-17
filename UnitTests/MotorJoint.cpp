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

#include <PlayRho/Dynamics/Joints/MotorJointConf.hpp>
#include <PlayRho/Dynamics/Joints/Joint.hpp>

#include <PlayRho/Dynamics/BodyConf.hpp>
#include <PlayRho/Dynamics/World.hpp>
#include <PlayRho/Dynamics/WorldBody.hpp>
#include <PlayRho/Dynamics/WorldJoint.hpp>
#include <PlayRho/Dynamics/StepConf.hpp>
#include <PlayRho/Collision/Shapes/DiskShapeConf.hpp>

using namespace playrho;
using namespace playrho::d2;

TEST(MotorJointConf, ByteSize)
{
    switch (sizeof(Real))
    {
        case  4:
#if defined(_WIN64)
            EXPECT_EQ(sizeof(MotorJointConf), std::size_t(120));
#elif defined(_WIN32)
            EXPECT_EQ(sizeof(MotorJointConf), std::size_t(96));
#else
            EXPECT_EQ(sizeof(MotorJointConf), std::size_t(104));
#endif
            break;
        case  8: EXPECT_EQ(sizeof(MotorJointConf), std::size_t(184)); break;
        case 16: EXPECT_EQ(sizeof(MotorJointConf), std::size_t(352)); break;
        default: FAIL(); break;
    }
}

TEST(MotorJointConf, DefaultConstruction)
{
    MotorJointConf def{};
    
    EXPECT_EQ(def.bodyA, InvalidBodyID);
    EXPECT_EQ(def.bodyB, InvalidBodyID);
    EXPECT_EQ(def.collideConnected, false);
    EXPECT_EQ(def.userData, nullptr);
    
    EXPECT_EQ(def.linearOffset, (Length2{}));
    EXPECT_EQ(def.angularOffset, 0_deg);
    EXPECT_EQ(def.maxForce, 1_N);
    EXPECT_EQ(def.maxTorque, 1_Nm);
    EXPECT_EQ(def.correctionFactor, Real(0.3));
}

TEST(MotorJointConf, BuilderConstruction)
{
    const auto bodyA = static_cast<BodyID>(0x1);
    const auto bodyB = static_cast<BodyID>(0x2);
    const auto collideConnected = true;
    int tmp;
    const auto userData = &tmp;
    const auto linearOffset = Length2{2_m, 3_m};
    const auto angularOffset = 33_rad;
    const auto maxForce = 22_N;
    const auto maxTorque = 31_Nm;
    const auto correctionFactor = Real(0.44f);
    const auto def = MotorJointConf{}
        .UseBodyA(bodyA).UseBodyB(bodyB).UseCollideConnected(collideConnected).UseUserData(userData)
        .UseLinearOffset(linearOffset).UseAngularOffset(angularOffset)
        .UseMaxForce(maxForce).UseMaxTorque(maxTorque).UseCorrectionFactor(correctionFactor);
    
    EXPECT_EQ(def.bodyA, bodyA);
    EXPECT_EQ(def.bodyB, bodyB);
    EXPECT_EQ(def.collideConnected, collideConnected);
    EXPECT_EQ(def.userData, userData);
    
    EXPECT_EQ(def.linearOffset, linearOffset);
    EXPECT_EQ(def.angularOffset, angularOffset);
    EXPECT_EQ(def.maxForce, maxForce);
    EXPECT_EQ(def.maxTorque, maxTorque);
    EXPECT_EQ(def.correctionFactor, correctionFactor);
}

TEST(MotorJoint, Construction)
{
    auto world = World{};
    const auto b0 = CreateBody(world);
    const auto b1 = CreateBody(world);

    auto def = GetMotorJointConf(world, b0, b1);
    const auto jointID = CreateJoint(world, def);

    EXPECT_EQ(GetType(world, jointID), GetTypeID<MotorJointConf>());
    EXPECT_EQ(GetBodyA(world, jointID), def.bodyA);
    EXPECT_EQ(GetBodyB(world, jointID), def.bodyB);
    EXPECT_EQ(GetCollideConnected(world, jointID), def.collideConnected);
    EXPECT_EQ(GetUserData(world, jointID), def.userData);
    EXPECT_EQ(GetLinearReaction(world, jointID), Momentum2{});
    EXPECT_EQ(GetAngularReaction(world, jointID), AngularMomentum{0});

    EXPECT_EQ(GetLinearOffset(world, jointID), def.linearOffset);
    EXPECT_EQ(GetAngularOffset(world, jointID), def.angularOffset);

    const auto conf = TypeCast<MotorJointConf>(GetJoint(world, jointID));
    EXPECT_EQ(GetMaxForce(conf), def.maxForce);
    EXPECT_EQ(GetMaxTorque(conf), def.maxTorque);
    EXPECT_EQ(GetCorrectionFactor(conf), def.correctionFactor);
}

TEST(MotorJoint, ShiftOrigin)
{
    auto world = World{};
    const auto b0 = world.CreateBody();
    const auto b1 = world.CreateBody();

    auto def = GetMotorJointConf(world, b0, b1);
    const auto joint = CreateJoint(world, def);
    const auto newOrigin = Length2{1_m, 1_m};
    EXPECT_FALSE(ShiftOrigin(world, joint, newOrigin));
}

TEST(MotorJoint, SetCorrectionFactor)
{
    auto world = World{};
    const auto b0 = world.CreateBody();
    const auto b1 = world.CreateBody();
    
    auto def = GetMotorJointConf(world, b0, b1);
    const auto jointID = CreateJoint(world, def);
    auto conf = TypeCast<MotorJointConf>(GetJoint(world, jointID));

    ASSERT_EQ(GetCorrectionFactor(conf), def.correctionFactor);
    ASSERT_EQ(Real(0.3), def.correctionFactor);
    
    EXPECT_NO_THROW(SetCorrectionFactor(conf, Real(0.9)));
    EXPECT_EQ(GetCorrectionFactor(conf), Real(0.9));

    EXPECT_NO_THROW(SetJoint(world, jointID, conf));
    auto conf2 = TypeCast<MotorJointConf>(GetJoint(world, jointID));
    EXPECT_EQ(GetCorrectionFactor(conf2), Real(0.9));
}

TEST(MotorJoint, GetMotorJointConf)
{
    auto world = World{};
    const auto b0 = world.CreateBody();
    const auto b1 = world.CreateBody();
    
    auto def = GetMotorJointConf(world, b0, b1);
    const auto jointID = CreateJoint(world, def);
    
    ASSERT_EQ(GetType(world, jointID), GetTypeID<MotorJointConf>());
    ASSERT_EQ(GetBodyA(world, jointID), def.bodyA);
    ASSERT_EQ(GetBodyB(world, jointID), def.bodyB);
    ASSERT_EQ(GetCollideConnected(world, jointID), def.collideConnected);
    ASSERT_EQ(GetUserData(world, jointID), def.userData);
    
    ASSERT_EQ(GetLinearOffset(world, jointID), def.linearOffset);
    ASSERT_EQ(GetAngularOffset(world, jointID), def.angularOffset);
    const auto conf = TypeCast<MotorJointConf>(GetJoint(world, jointID));
    ASSERT_EQ(GetMaxForce(conf), def.maxForce);
    ASSERT_EQ(GetMaxTorque(conf), def.maxTorque);
    ASSERT_EQ(GetCorrectionFactor(conf), def.correctionFactor);
    
    const auto cdef = GetMotorJointConf(GetJoint(world, jointID));
    EXPECT_EQ(cdef.bodyA, b0);
    EXPECT_EQ(cdef.bodyB, b1);
    EXPECT_EQ(cdef.collideConnected, false);
    EXPECT_EQ(cdef.userData, nullptr);
    
    EXPECT_EQ(cdef.linearOffset, (Length2{}));
    EXPECT_EQ(cdef.angularOffset, 0_deg);
    EXPECT_EQ(cdef.maxForce, 1_N);
    EXPECT_EQ(cdef.maxTorque, 1_Nm);
    EXPECT_EQ(cdef.correctionFactor, Real(0.3));
}

TEST(MotorJoint, WithDynamicCircles)
{
    const auto circle = Shape{DiskShapeConf{}.UseRadius(0.2_m)};
    auto world = World{};
    const auto p1 = Length2{-1_m, 0_m};
    const auto p2 = Length2{+1_m, 0_m};
    const auto b1 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p1));
    const auto b2 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p2));
    world.CreateFixture(b1, circle);
    world.CreateFixture(b2, circle);
    //const auto anchor = Length2(2_m, 1_m);
    const auto jd = GetMotorJointConf(world, b1, b2);
    const auto joint = CreateJoint(world, jd);
    ASSERT_NE(joint, InvalidJointID);
    EXPECT_EQ(GetAnchorA(world, joint), p1);
    EXPECT_EQ(GetAnchorB(world, joint), p2);

    auto stepConf = StepConf{};
    
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

TEST(MotorJoint, SetLinearOffset)
{
    const auto circle = Shape{DiskShapeConf{}.UseRadius(0.2_m)};
    auto world = World{};
    const auto p1 = Length2{-1_m, 0_m};
    const auto p2 = Length2{+1_m, 0_m};
    const auto b1 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p1));
    const auto b2 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p2));
    world.CreateFixture(b1, circle);
    world.CreateFixture(b2, circle);
    //const auto anchor = Length2(2_m, 1_m);
    const auto jd = GetMotorJointConf(world, b1, b2);
    const auto joint = CreateJoint(world, jd);
    ASSERT_NE(joint, InvalidJointID);
    EXPECT_EQ(GetAnchorA(world, joint), p1);
    EXPECT_EQ(GetAnchorB(world, joint), p2);
    
    const auto linearOffset = Length2{2_m, 1_m};
    ASSERT_EQ(GetLinearOffset(world, joint), jd.linearOffset);
    ASSERT_NE(jd.linearOffset, linearOffset);
    SetLinearOffset(world, joint, linearOffset);
    EXPECT_EQ(GetLinearOffset(world, joint), linearOffset);
}

TEST(MotorJoint, SetAngularOffset)
{
    const auto circle = Shape{DiskShapeConf{}.UseRadius(0.2_m)};
    auto world = World{};
    const auto p1 = Length2{-1_m, 0_m};
    const auto p2 = Length2{+1_m, 0_m};
    const auto b1 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p1));
    const auto b2 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p2));
    world.CreateFixture(b1, circle);
    world.CreateFixture(b2, circle);
    //const auto anchor = Length2(2_m, 1_m);
    const auto jd = GetMotorJointConf(world, b1, b2);
    const auto joint = CreateJoint(world, jd);
    ASSERT_NE(joint, InvalidJointID);
    EXPECT_EQ(GetAnchorA(world, joint), p1);
    EXPECT_EQ(GetAnchorB(world, joint), p2);

    ASSERT_EQ(GetAngularOffset(world, joint), 0_deg);
    SetAngularOffset(world, joint, 45_deg);
    EXPECT_EQ(GetAngularOffset(world, joint), 45_deg);
}
