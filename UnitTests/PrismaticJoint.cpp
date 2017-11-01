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
#include <PlayRho/Dynamics/Joints/PrismaticJoint.hpp>
#include <PlayRho/Dynamics/World.hpp>
#include <PlayRho/Dynamics/Body.hpp>
#include <PlayRho/Collision/Shapes/DiskShape.hpp>

using namespace playrho;

TEST(PrismaticJoint, ByteSize)
{
    switch (sizeof(Real))
    {
        case  4: EXPECT_EQ(sizeof(PrismaticJoint), std::size_t(184)); break;
        case  8: EXPECT_EQ(sizeof(PrismaticJoint), std::size_t(328)); break;
        case 16: EXPECT_EQ(sizeof(PrismaticJoint), std::size_t(624)); break;
        default: FAIL(); break;
    }
}

TEST(PrismaticJoint, EnableLimit)
{
    World world;
    const auto b0 = world.CreateBody();
    const auto b1 = world.CreateBody();
    
    auto jd = PrismaticJointDef{};
    jd.bodyA = b0;
    jd.bodyB = b1;
    jd.localAnchorA = Length2(Real(4) * Meter, Real(5) * Meter);
    jd.localAnchorB = Length2(Real(6) * Meter, Real(7) * Meter);
    
    auto joint = PrismaticJoint{jd};
    EXPECT_FALSE(joint.IsLimitEnabled());
    joint.EnableLimit(false);
    EXPECT_FALSE(joint.IsLimitEnabled());
    joint.EnableLimit(true);
    EXPECT_TRUE(joint.IsLimitEnabled());
}

TEST(PrismaticJoint, EnableMotor)
{
    World world;
    const auto b0 = world.CreateBody();
    const auto b1 = world.CreateBody();
    
    auto jd = PrismaticJointDef{};
    jd.bodyA = b0;
    jd.bodyB = b1;
    jd.localAnchorA = Length2(Real(4) * Meter, Real(5) * Meter);
    jd.localAnchorB = Length2(Real(6) * Meter, Real(7) * Meter);
    
    auto joint = PrismaticJoint{jd};
    EXPECT_FALSE(joint.IsMotorEnabled());
    joint.EnableMotor(false);
    EXPECT_FALSE(joint.IsMotorEnabled());
    joint.EnableMotor(true);
    EXPECT_TRUE(joint.IsMotorEnabled());
}

TEST(PrismaticJoint, SetMaxMotorForce)
{
    World world;
    const auto b0 = world.CreateBody();
    const auto b1 = world.CreateBody();
    
    auto jd = PrismaticJointDef{};
    jd.bodyA = b0;
    jd.bodyB = b1;
    jd.localAnchorA = Length2(Real(4) * Meter, Real(5) * Meter);
    jd.localAnchorB = Length2(Real(6) * Meter, Real(7) * Meter);
    
    auto joint = PrismaticJoint{jd};
    ASSERT_EQ(joint.GetMaxMotorForce(), 0_N);
    joint.SetMaxMotorForce(2_N);
    EXPECT_EQ(joint.GetMaxMotorForce(), 2_N);
}

TEST(PrismaticJoint, MotorSpeed)
{
    World world;
    const auto b0 = world.CreateBody();
    const auto b1 = world.CreateBody();
    
    auto jd = PrismaticJointDef{};
    jd.bodyA = b0;
    jd.bodyB = b1;
    jd.localAnchorA = Length2(Real(4) * Meter, Real(5) * Meter);
    jd.localAnchorB = Length2(Real(6) * Meter, Real(7) * Meter);
    
    const auto newValue = Real(5) * RadianPerSecond;
    auto joint = PrismaticJoint{jd};
    ASSERT_NE(joint.GetMotorSpeed(), newValue);
    EXPECT_EQ(joint.GetMotorSpeed(), jd.motorSpeed);
    joint.SetMotorSpeed(newValue);
    EXPECT_EQ(joint.GetMotorSpeed(), newValue);
}

TEST(PrismaticJoint, SetLimits)
{
    World world;
    const auto b0 = world.CreateBody();
    const auto b1 = world.CreateBody();
    
    auto jd = PrismaticJointDef{};
    jd.bodyA = b0;
    jd.bodyB = b1;
    jd.localAnchorA = Length2(Real(4) * Meter, Real(5) * Meter);
    jd.localAnchorB = Length2(Real(6) * Meter, Real(7) * Meter);
    
    const auto upperValue = Real(+5) * Meter;
    const auto lowerValue = Real(-8) * Meter;
    auto joint = PrismaticJoint{jd};
    ASSERT_NE(joint.GetUpperLimit(), upperValue);
    ASSERT_NE(joint.GetLowerLimit(), lowerValue);
    joint.SetLimits(lowerValue, upperValue);
    EXPECT_EQ(joint.GetUpperLimit(), upperValue);
    EXPECT_EQ(joint.GetLowerLimit(), lowerValue);
}

TEST(PrismaticJoint, GetAnchorAandB)
{
    World world;
    
    const auto loc0 = Length2{Real(+1) * Meter, Real(-3) * Meter};
    const auto loc1 = Length2{Real(-2) * Meter, Real(+1.2f) * Meter};

    const auto b0 = world.CreateBody(BodyDef{}.UseLocation(loc0));
    const auto b1 = world.CreateBody(BodyDef{}.UseLocation(loc1));
    
    auto jd = PrismaticJointDef{};
    jd.bodyA = b0;
    jd.bodyB = b1;
    jd.localAnchorA = Length2(Real(4) * Meter, Real(5) * Meter);
    jd.localAnchorB = Length2(Real(6) * Meter, Real(7) * Meter);
    
    auto joint = PrismaticJoint{jd};
    ASSERT_EQ(joint.GetLocalAnchorA(), jd.localAnchorA);
    ASSERT_EQ(joint.GetLocalAnchorB(), jd.localAnchorB);
    EXPECT_EQ(joint.GetAnchorA(), loc0 + jd.localAnchorA);
    EXPECT_EQ(joint.GetAnchorB(), loc1 + jd.localAnchorB);
}

TEST(PrismaticJoint, GetJointTranslation)
{
    World world;
    
    const auto loc0 = Length2{Real(+1) * Meter, Real(-3) * Meter};
    const auto loc1 = Length2{Real(+1) * Meter, Real(+3) * Meter};
    
    const auto b0 = world.CreateBody(BodyDef{}.UseLocation(loc0));
    const auto b1 = world.CreateBody(BodyDef{}.UseLocation(loc1));
    
    auto jd = PrismaticJointDef{};
    jd.bodyA = b0;
    jd.bodyB = b1;
    jd.localAnchorA = Length2(Real(-1) * Meter, Real(5) * Meter);
    jd.localAnchorB = Length2(Real(+1) * Meter, Real(5) * Meter);
    
    auto joint = PrismaticJoint{jd};
    EXPECT_EQ(GetJointTranslation(joint), Length(Real(2) * Meter));
}

TEST(PrismaticJoint, GetLinearVelocity)
{
    World world;
    
    const auto loc0 = Length2{Real(+1) * Meter, Real(-3) * Meter};
    const auto loc1 = Length2{Real(+1) * Meter, Real(+3) * Meter};
    
    const auto b0 = world.CreateBody(BodyDef{}.UseLocation(loc0));
    const auto b1 = world.CreateBody(BodyDef{}.UseLocation(loc1));
    
    auto jd = PrismaticJointDef{};
    jd.bodyA = b0;
    jd.bodyB = b1;
    jd.localAnchorA = Length2(Real(-1) * Meter, Real(5) * Meter);
    jd.localAnchorB = Length2(Real(+1) * Meter, Real(5) * Meter);
    
    auto joint = PrismaticJoint{jd};
    EXPECT_EQ(GetLinearVelocity(joint), LinearVelocity(0));
}

TEST(PrismaticJoint, WithDynamicCirclesAndLimitEnabled)
{
    const auto circle = std::make_shared<DiskShape>(Real{0.2f} * Meter);
    auto world = World{WorldDef{}.UseGravity(LinearAcceleration2{})};
    const auto p1 = Length2{-Real(1) * Meter, Real(0) * Meter};
    const auto p2 = Length2{+Real(1) * Meter, Real(0) * Meter};
    const auto b1 = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(p1));
    const auto b2 = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(p2));
    b1->CreateFixture(circle);
    b2->CreateFixture(circle);
    const auto anchor = Length2(Real(2) * Meter, Real(1) * Meter);
    const auto jd = PrismaticJointDef{b1, b2, anchor, UnitVec2::GetRight()}.UseEnableLimit(true);
    const auto joint = static_cast<PrismaticJoint*>(world.CreateJoint(jd));
    ASSERT_NE(joint, nullptr);
    ASSERT_EQ(joint->GetLimitState(), Joint::e_inactiveLimit);
    ASSERT_EQ(joint->GetLowerLimit(), Length(0));
    ASSERT_EQ(joint->GetUpperLimit(), Length(0));

    Step(world, 1_s);
    EXPECT_NEAR(double(Real{GetX(b1->GetLocation()) / Meter}), -1.0, 0.001);
    EXPECT_NEAR(double(Real{GetY(b1->GetLocation()) / Meter}), 0.0, 0.001);
    EXPECT_NEAR(double(Real{GetX(b2->GetLocation()) / Meter}), +1.0, 0.01);
    EXPECT_NEAR(double(Real{GetY(b2->GetLocation()) / Meter}), 0.0, 0.01);
    EXPECT_EQ(b1->GetAngle(), Angle{0});
    EXPECT_EQ(b2->GetAngle(), Angle{0});
    EXPECT_EQ(joint->GetLowerLimit(), Length(0));
    EXPECT_EQ(joint->GetUpperLimit(), Length(0));
    EXPECT_EQ(joint->GetLimitState(), Joint::e_equalLimits);
    
    joint->SetLimits(Real(0) * Meter, Real(2) * Meter);
    Step(world, 1_s);
    EXPECT_EQ(joint->GetLowerLimit(), Length(0));
    EXPECT_EQ(joint->GetUpperLimit(), Real(2) * Meter);
    EXPECT_EQ(joint->GetLimitState(), Joint::e_atLowerLimit);
    
    joint->SetLimits(Real(-2) * Meter, Real(0) * Meter);
    Step(world, 1_s);
    EXPECT_EQ(joint->GetLowerLimit(), Real(-2) * Meter);
    EXPECT_EQ(joint->GetUpperLimit(), Real(0) * Meter);
    EXPECT_EQ(joint->GetLimitState(), Joint::e_atUpperLimit);
}
