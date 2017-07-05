/*
 * Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

#include "gtest/gtest.h"
#include <Box2D/Dynamics/Joints/RevoluteJoint.hpp>
#include <Box2D/Dynamics/World.hpp>
#include <Box2D/Dynamics/Body.hpp>
#include <Box2D/Dynamics/BodyDef.hpp>
#include <Box2D/Dynamics/Fixture.hpp>
#include <Box2D/Collision/Shapes/DiskShape.hpp>
#include <Box2D/Collision/Shapes/PolygonShape.hpp>

using namespace box2d;

TEST(RevoluteJoint, ByteSize)
{
    switch (sizeof(RealNum))
    {
        case  4: EXPECT_EQ(sizeof(RevoluteJoint), std::size_t(160)); break;
        case  8: EXPECT_EQ(sizeof(RevoluteJoint), std::size_t(280)); break;
        case 16: EXPECT_EQ(sizeof(RevoluteJoint), std::size_t(528)); break;
        default: FAIL(); break;
    }
}

TEST(RevoluteJoint, Construction)
{
    auto jd = RevoluteJointDef{};

    jd.bodyA = reinterpret_cast<Body*>(0x04);
    jd.bodyB = reinterpret_cast<Body*>(0x08);
    jd.collideConnected = true;
    jd.userData = reinterpret_cast<void*>(0x011);

    jd.localAnchorA = Length2D(RealNum(4) * Meter, RealNum(5) * Meter);
    jd.localAnchorB = Length2D(RealNum(6) * Meter, RealNum(7) * Meter);
    jd.enableLimit = true;
    jd.enableMotor = true;
    jd.motorSpeed = RealNum{4.4f} * RadianPerSecond;
    jd.maxMotorTorque = RealNum{1.0f} * NewtonMeter;
    jd.lowerAngle = Angle{RealNum{33.0f} * Degree};
    jd.upperAngle = Angle{RealNum{40.0f} * Degree};
    jd.referenceAngle = Angle{RealNum{45.0f} * Degree};
    
    const auto joint = RevoluteJoint{jd};

    EXPECT_EQ(joint.GetType(), jd.type);
    EXPECT_EQ(joint.GetBodyA(), jd.bodyA);
    EXPECT_EQ(joint.GetBodyB(), jd.bodyB);
    EXPECT_EQ(joint.GetCollideConnected(), jd.collideConnected);
    EXPECT_EQ(joint.GetUserData(), jd.userData);
    
    EXPECT_EQ(joint.GetLocalAnchorA(), jd.localAnchorA);
    EXPECT_EQ(joint.GetLocalAnchorB(), jd.localAnchorB);
    EXPECT_EQ(joint.GetLowerLimit(), jd.lowerAngle);
    EXPECT_EQ(joint.GetUpperLimit(), jd.upperAngle);
    EXPECT_EQ(joint.GetMotorSpeed(), jd.motorSpeed);
    EXPECT_EQ(joint.GetReferenceAngle(), jd.referenceAngle);
    EXPECT_EQ(joint.IsMotorEnabled(), jd.enableMotor);
    EXPECT_EQ(joint.GetMaxMotorTorque(), jd.maxMotorTorque);
    EXPECT_EQ(joint.IsLimitEnabled(), jd.enableLimit);
}

TEST(RevoluteJoint, MovesDynamicCircles)
{
    const auto circle = std::make_shared<DiskShape>(RealNum{0.2f} * Meter);
    World world;
    const auto p1 = Length2D{-RealNum(1) * Meter, RealNum(0) * Meter};
    const auto p2 = Length2D{+RealNum(1) * Meter, RealNum(0) * Meter};
    const auto b1 = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(p1));
    const auto b2 = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(p2));
    b1->CreateFixture(circle);
    b2->CreateFixture(circle);
    auto jd = RevoluteJointDef{};
    jd.bodyA = b1;
    jd.bodyB = b2;
    world.CreateJoint(jd);
    Step(world, Time{Second * RealNum{1}});
    EXPECT_NEAR(double(RealNum{b1->GetLocation().x / Meter}), 0, 0.001);
    EXPECT_NEAR(double(RealNum{b1->GetLocation().y / Meter}), -4, 0.001);
    EXPECT_NEAR(double(RealNum{b2->GetLocation().x / Meter}), 0, 0.01);
    EXPECT_NEAR(double(RealNum{b2->GetLocation().y / Meter}), -4, 0.01);
    EXPECT_EQ(b1->GetAngle(), Angle{0});
    EXPECT_EQ(b2->GetAngle(), Angle{0});
}

TEST(RevoluteJoint, LimitEnabledDynamicCircles)
{
    const auto circle = std::make_shared<DiskShape>(DiskShape::Conf{}
                                                    .UseVertexRadius(RealNum{0.2f} * Meter)
                                                    .UseDensity(RealNum(1) * KilogramPerSquareMeter));
    World world;
    const auto p1 = Length2D{-RealNum(1) * Meter, RealNum(0) * Meter};
    const auto p2 = Length2D{+RealNum(1) * Meter, RealNum(0) * Meter};
    const auto b1 = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(p1));
    const auto b2 = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(p2));
    b1->CreateFixture(circle);
    b2->CreateFixture(circle);
    auto jd = RevoluteJointDef{b1, b2, Length2D(0, 0)};
    jd.enableLimit = true;
    const auto joint = world.CreateJoint(jd);
    EXPECT_NE(joint, nullptr);
    Step(world, Time{Second * RealNum{1}});
    EXPECT_NEAR(double(RealNum{b1->GetLocation().x / Meter}), -1.0, 0.001);
    EXPECT_NEAR(double(RealNum{b1->GetLocation().y / Meter}), -4, 0.001);
    EXPECT_NEAR(double(RealNum{b2->GetLocation().x / Meter}), +1.0, 0.01);
    EXPECT_NEAR(double(RealNum{b2->GetLocation().y / Meter}), -4, 0.01);
    EXPECT_EQ(b1->GetAngle(), Angle{0});
    EXPECT_EQ(b2->GetAngle(), Angle{0});
    EXPECT_TRUE(IsEnabled(*joint));
    b1->UnsetAwake();
    b2->UnsetAwake();
    ASSERT_FALSE(b1->IsAwake());
    ASSERT_FALSE(b2->IsAwake());
    SetAwake(*joint);
    EXPECT_TRUE(b1->IsAwake());
    EXPECT_TRUE(b2->IsAwake());
    
    EXPECT_EQ(GetWorldIndex(joint), std::size_t(0));
}

TEST(RevoluteJoint, DynamicJoinedToStaticStaysPut)
{
    World world{WorldDef{}.UseGravity(LinearAcceleration2D{
        RealNum(0) * MeterPerSquareSecond,
        -RealNum(10) * MeterPerSquareSecond
    })};
    
    const auto p1 = Length2D{RealNum(0) * Meter, RealNum(4) * Meter}; // Vec2{-1, 0};
    const auto p2 = Length2D{RealNum(0) * Meter, -RealNum(2) * Meter}; // Vec2{+1, 0};
    const auto b1 = world.CreateBody(BodyDef{}.UseType(BodyType::Static).UseLocation(p1));
    const auto b2 = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(p2));

    const auto shape1 = std::make_shared<PolygonShape>();
    shape1->SetAsBox(RealNum{1} * Meter, RealNum{1} * Meter);
    b1->CreateFixture(shape1);
    
    const auto shape2 = std::make_shared<PolygonShape>();
    shape2->SetAsBox(RealNum{0.5f} * Meter, RealNum{0.5f} * Meter);
    shape2->SetDensity(RealNum{1} * KilogramPerSquareMeter);
    b2->CreateFixture(shape2);
    
    auto jd = RevoluteJointDef{b1, b2, Length2D(0, 0)};
    const auto joint = world.CreateJoint(jd);
    
    for (auto i = 0; i < 1000; ++i)
    {
        Step(world, Second * RealNum{0.1f});
        EXPECT_EQ(b1->GetLocation(), p1);
        EXPECT_NEAR(double(RealNum{b2->GetLocation().x / Meter}),
                    double(RealNum{p2.x / Meter}), 0.0001);
        EXPECT_NEAR(double(RealNum{b2->GetLocation().y / Meter}),
                    double(RealNum{p2.y / Meter}), 0.0001);
        EXPECT_EQ(b2->GetAngle(), Angle{0});
    }
    
    world.Destroy(joint);
    
    for (auto i = 0; i < 10; ++i)
    {
        Step(world, Time{Second * RealNum{0.1f}});
        EXPECT_EQ(b1->GetLocation(), p1);
        EXPECT_NE(b2->GetLocation(), p2);
        EXPECT_EQ(b2->GetAngle(), Angle{0});
    }

}
