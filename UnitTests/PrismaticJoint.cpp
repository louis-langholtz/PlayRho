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
    jd.localAnchorA = Length2D(Real(4) * Meter, Real(5) * Meter);
    jd.localAnchorB = Length2D(Real(6) * Meter, Real(7) * Meter);
    
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
    jd.localAnchorA = Length2D(Real(4) * Meter, Real(5) * Meter);
    jd.localAnchorB = Length2D(Real(6) * Meter, Real(7) * Meter);
    
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
    jd.localAnchorA = Length2D(Real(4) * Meter, Real(5) * Meter);
    jd.localAnchorB = Length2D(Real(6) * Meter, Real(7) * Meter);
    
    auto joint = PrismaticJoint{jd};
    ASSERT_EQ(joint.GetMaxMotorForce(), Real(0) * Newton);
    joint.SetMaxMotorForce(Real(2) * Newton);
    EXPECT_EQ(joint.GetMaxMotorForce(), Real(2) * Newton);
}

TEST(PrismaticJoint, MotorSpeed)
{
    World world;
    const auto b0 = world.CreateBody();
    const auto b1 = world.CreateBody();
    
    auto jd = PrismaticJointDef{};
    jd.bodyA = b0;
    jd.bodyB = b1;
    jd.localAnchorA = Length2D(Real(4) * Meter, Real(5) * Meter);
    jd.localAnchorB = Length2D(Real(6) * Meter, Real(7) * Meter);
    
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
    jd.localAnchorA = Length2D(Real(4) * Meter, Real(5) * Meter);
    jd.localAnchorB = Length2D(Real(6) * Meter, Real(7) * Meter);
    
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
    
    const auto loc0 = Length2D{Real(+1) * Meter, Real(-3) * Meter};
    const auto loc1 = Length2D{Real(-2) * Meter, Real(+1.2f) * Meter};

    const auto b0 = world.CreateBody(BodyDef{}.UseLocation(loc0));
    const auto b1 = world.CreateBody(BodyDef{}.UseLocation(loc1));
    
    auto jd = PrismaticJointDef{};
    jd.bodyA = b0;
    jd.bodyB = b1;
    jd.localAnchorA = Length2D(Real(4) * Meter, Real(5) * Meter);
    jd.localAnchorB = Length2D(Real(6) * Meter, Real(7) * Meter);
    
    auto joint = PrismaticJoint{jd};
    ASSERT_EQ(joint.GetLocalAnchorA(), jd.localAnchorA);
    ASSERT_EQ(joint.GetLocalAnchorB(), jd.localAnchorB);
    EXPECT_EQ(joint.GetAnchorA(), loc0 + jd.localAnchorA);
    EXPECT_EQ(joint.GetAnchorB(), loc1 + jd.localAnchorB);
}

TEST(PrismaticJoint, GetJointTranslation)
{
    World world;
    
    const auto loc0 = Length2D{Real(+1) * Meter, Real(-3) * Meter};
    const auto loc1 = Length2D{Real(+1) * Meter, Real(+3) * Meter};
    
    const auto b0 = world.CreateBody(BodyDef{}.UseLocation(loc0));
    const auto b1 = world.CreateBody(BodyDef{}.UseLocation(loc1));
    
    auto jd = PrismaticJointDef{};
    jd.bodyA = b0;
    jd.bodyB = b1;
    jd.localAnchorA = Length2D(Real(-1) * Meter, Real(5) * Meter);
    jd.localAnchorB = Length2D(Real(+1) * Meter, Real(5) * Meter);
    
    auto joint = PrismaticJoint{jd};
    EXPECT_EQ(GetJointTranslation(joint), Length(Real(2) * Meter));
}

TEST(PrismaticJoint, GetLinearVelocity)
{
    World world;
    
    const auto loc0 = Length2D{Real(+1) * Meter, Real(-3) * Meter};
    const auto loc1 = Length2D{Real(+1) * Meter, Real(+3) * Meter};
    
    const auto b0 = world.CreateBody(BodyDef{}.UseLocation(loc0));
    const auto b1 = world.CreateBody(BodyDef{}.UseLocation(loc1));
    
    auto jd = PrismaticJointDef{};
    jd.bodyA = b0;
    jd.bodyB = b1;
    jd.localAnchorA = Length2D(Real(-1) * Meter, Real(5) * Meter);
    jd.localAnchorB = Length2D(Real(+1) * Meter, Real(5) * Meter);
    
    auto joint = PrismaticJoint{jd};
    EXPECT_EQ(GetLinearVelocity(joint), LinearVelocity(0));
}
