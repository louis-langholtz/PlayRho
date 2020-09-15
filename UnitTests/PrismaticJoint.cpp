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
#include <PlayRho/Dynamics/Joints/PrismaticJoint.hpp>
#include <PlayRho/Dynamics/Joints/TypeJointVisitor.hpp>
#include <PlayRho/Dynamics/World.hpp>
#include <PlayRho/Dynamics/Body.hpp>
#include <PlayRho/Collision/Shapes/DiskShapeConf.hpp>

using namespace playrho;
using namespace playrho::d2;

TEST(PrismaticJoint, ByteSize)
{
    switch (sizeof(Real))
    {
        case  4:
#if defined(_WIN64)
            EXPECT_EQ(sizeof(PrismaticJoint), std::size_t(192));
#elif defined(_WIN32)
            EXPECT_EQ(sizeof(PrismaticJoint), std::size_t(168));
#else
            EXPECT_EQ(sizeof(PrismaticJoint), std::size_t(184));
#endif
            break;
        case  8: EXPECT_EQ(sizeof(PrismaticJoint), std::size_t(328)); break;
        case 16: EXPECT_EQ(sizeof(PrismaticJoint), std::size_t(624)); break;
        default: FAIL(); break;
    }
}

TEST(PrismaticJoint, Construction)
{
    auto world = World{};
    const auto b0 = world.CreateBody();
    const auto b1 = world.CreateBody();
    
    auto jd = PrismaticJointConf{};
    jd.bodyA = b0;
    jd.bodyB = b1;
    jd.localAnchorA = Length2(4_m, 5_m);
    jd.localAnchorB = Length2(6_m, 7_m);
    
    auto joint = PrismaticJoint{jd};
    EXPECT_EQ(joint.GetBodyA(), b0);
    EXPECT_EQ(joint.GetBodyB(), b1);
    EXPECT_EQ(joint.GetLocalAnchorA(), jd.localAnchorA);
    EXPECT_EQ(joint.GetLocalAnchorB(), jd.localAnchorB);
    EXPECT_EQ(joint.GetLinearReaction(), Momentum2{});
    EXPECT_EQ(joint.GetAngularReaction(), AngularMomentum{0});
}

TEST(PrismaticJoint, EnableLimit)
{
    auto world = World{};
    const auto b0 = world.CreateBody();
    const auto b1 = world.CreateBody();
    
    auto jd = PrismaticJointConf{};
    jd.bodyA = b0;
    jd.bodyB = b1;
    jd.localAnchorA = Length2(4_m, 5_m);
    jd.localAnchorB = Length2(6_m, 7_m);
    
    auto joint = PrismaticJoint{jd};
    EXPECT_FALSE(joint.IsLimitEnabled());
    joint.EnableLimit(false);
    EXPECT_FALSE(joint.IsLimitEnabled());
    joint.EnableLimit(true);
    EXPECT_TRUE(joint.IsLimitEnabled());
    EXPECT_EQ(joint.GetMotorImpulse(), 0_Ns);

    EXPECT_EQ(GetMotorForce(joint, 1_Hz), 0 * Newton);

    TypeJointVisitor visitor;
    joint.Accept(visitor);
    EXPECT_EQ(visitor.GetType().value(), JointType::Prismatic);
}

TEST(PrismaticJoint, ShiftOrigin)
{
    auto world = World{};
    const auto b0 = world.CreateBody();
    const auto b1 = world.CreateBody();
    
    auto jd = PrismaticJointConf{};
    jd.bodyA = b0;
    jd.bodyB = b1;
    jd.localAnchorA = Length2(4_m, 5_m);
    jd.localAnchorB = Length2(6_m, 7_m);
    
    auto joint = PrismaticJoint{jd};

    const auto newOrigin = Length2{1_m, 1_m};
    EXPECT_FALSE(joint.ShiftOrigin(newOrigin));
}

TEST(PrismaticJoint, EnableMotor)
{
    World world;
    const auto b0 = world.CreateBody();
    const auto b1 = world.CreateBody();
    
    auto jd = PrismaticJointConf{};
    jd.bodyA = b0;
    jd.bodyB = b1;
    jd.localAnchorA = Length2(4_m, 5_m);
    jd.localAnchorB = Length2(6_m, 7_m);
    
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
    
    auto jd = PrismaticJointConf{};
    jd.bodyA = b0;
    jd.bodyB = b1;
    jd.localAnchorA = Length2(4_m, 5_m);
    jd.localAnchorB = Length2(6_m, 7_m);
    
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
    
    auto jd = PrismaticJointConf{};
    jd.bodyA = b0;
    jd.bodyB = b1;
    jd.localAnchorA = Length2(4_m, 5_m);
    jd.localAnchorB = Length2(6_m, 7_m);
    
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
    
    auto jd = PrismaticJointConf{};
    jd.bodyA = b0;
    jd.bodyB = b1;
    jd.localAnchorA = Length2(4_m, 5_m);
    jd.localAnchorB = Length2(6_m, 7_m);
    
    const auto upperValue = +5_m;
    const auto lowerValue = -8_m;
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
    
    const auto loc0 = Length2{+1_m, -3_m};
    const auto loc1 = Length2{-2_m, Real(+1.2f) * Meter};

    const auto b0 = world.CreateBody(BodyConf{}.UseLocation(loc0));
    const auto b1 = world.CreateBody(BodyConf{}.UseLocation(loc1));
    
    auto jd = PrismaticJointConf{};
    jd.bodyA = b0;
    jd.bodyB = b1;
    jd.localAnchorA = Length2(4_m, 5_m);
    jd.localAnchorB = Length2(6_m, 7_m);
    
    auto joint = world.CreateJoint(jd);
    ASSERT_EQ(GetLocalAnchorA(world, joint), jd.localAnchorA);
    ASSERT_EQ(GetLocalAnchorB(world, joint), jd.localAnchorB);
    EXPECT_EQ(GetAnchorA(world, joint), loc0 + jd.localAnchorA);
    EXPECT_EQ(GetAnchorB(world, joint), loc1 + jd.localAnchorB);
}

TEST(PrismaticJoint, GetJointTranslation)
{
    World world;
    
    const auto loc0 = Length2{+1_m, -3_m};
    const auto loc1 = Length2{+1_m, +3_m};
    
    const auto b0 = world.CreateBody(BodyConf{}.UseLocation(loc0));
    const auto b1 = world.CreateBody(BodyConf{}.UseLocation(loc1));
    
    auto jd = PrismaticJointConf{};
    jd.bodyA = b0;
    jd.bodyB = b1;
    jd.localAnchorA = Length2(-1_m, 5_m);
    jd.localAnchorB = Length2(+1_m, 5_m);
    
    auto joint = PrismaticJoint{jd};
#if 0
    EXPECT_EQ(GetJointTranslation(joint), Length(2_m));
#endif
}

TEST(PrismaticJoint, GetLinearVelocity)
{
    World world;
    
    const auto loc0 = Length2{+1_m, -3_m};
    const auto loc1 = Length2{+1_m, +3_m};
    
    const auto b0 = world.CreateBody(BodyConf{}.UseLocation(loc0));
    const auto b1 = world.CreateBody(BodyConf{}.UseLocation(loc1));
    
    auto jd = PrismaticJointConf{};
    jd.bodyA = b0;
    jd.bodyB = b1;
    jd.localAnchorA = Length2(-1_m, 5_m);
    jd.localAnchorB = Length2(+1_m, 5_m);
    
    auto joint = PrismaticJoint{jd};
#if 0
    EXPECT_EQ(GetLinearVelocity(joint), LinearVelocity(0));
#endif
}

TEST(PrismaticJoint, WithDynamicCirclesAndLimitEnabled)
{
    const auto circle = DiskShapeConf{}.UseRadius(0.2_m);
    auto world = World{};
    const auto p1 = Length2{-1_m, 0_m};
    const auto p2 = Length2{+1_m, 0_m};
    const auto b1 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p1));
    const auto b2 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p2));
    world.CreateFixture(b1, Shape{circle});
    world.CreateFixture(b2, Shape{circle});
    const auto anchor = Length2(2_m, 1_m);
#if 0
    const auto jd = PrismaticJointConf{b1, b2, anchor, UnitVec::GetRight()}.UseEnableLimit(true);
    const auto joint = static_cast<PrismaticJoint*>(world.CreateJoint(jd));
    ASSERT_NE(joint, nullptr);
    ASSERT_EQ(joint->GetLimitState(), Joint::e_inactiveLimit);
    ASSERT_EQ(joint->GetLowerLimit(), 0_m);
    ASSERT_EQ(joint->GetUpperLimit(), 0_m);

    Step(world, 1_s);
    EXPECT_NEAR(double(Real{GetX(b1->GetLocation()) / Meter}), -1.0, 0.001);
    EXPECT_NEAR(double(Real{GetY(b1->GetLocation()) / Meter}), 0.0, 0.001);
    EXPECT_NEAR(double(Real{GetX(b2->GetLocation()) / Meter}), +1.0, 0.01);
    EXPECT_NEAR(double(Real{GetY(b2->GetLocation()) / Meter}), 0.0, 0.01);
    EXPECT_EQ(b1->GetAngle(), 0_deg);
    EXPECT_EQ(b2->GetAngle(), 0_deg);
    EXPECT_EQ(joint->GetLowerLimit(), 0_m);
    EXPECT_EQ(joint->GetUpperLimit(), 0_m);
    EXPECT_EQ(joint->GetLimitState(), Joint::e_equalLimits);
    
    joint->SetLimits(0_m, 2_m);
    Step(world, 1_s);
    EXPECT_EQ(joint->GetLowerLimit(), 0_m);
    EXPECT_EQ(joint->GetUpperLimit(), 2_m);
    EXPECT_EQ(joint->GetLimitState(), Joint::e_atLowerLimit);
    
    joint->SetLimits(-2_m, 0_m);
    Step(world, 1_s);
    EXPECT_EQ(joint->GetLowerLimit(), -2_m);
    EXPECT_EQ(joint->GetUpperLimit(), 0_m);
    EXPECT_EQ(joint->GetLimitState(), Joint::e_atUpperLimit);
    
    joint->EnableMotor(true);
    Step(world, 1_s);
    EXPECT_EQ(joint->GetMotorImpulse(), Momentum(0));
#endif
}
