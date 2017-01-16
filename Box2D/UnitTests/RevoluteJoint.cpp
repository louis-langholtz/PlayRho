/*
 * Copyright (c) 2016 Louis Langholtz https://github.com/louis-langholtz/Box2D
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
#include <Box2D/Dynamics/Joints/RevoluteJoint.hpp>
#include <Box2D/Dynamics/World.hpp>
#include <Box2D/Dynamics/Body.hpp>
#include <Box2D/Collision/Shapes/CircleShape.hpp>
#include <Box2D/Collision/Shapes/PolygonShape.hpp>

using namespace box2d;

TEST(RevoluteJoint, ByteSizeIs312)
{
	EXPECT_EQ(sizeof(RevoluteJoint), size_t(312));
}

TEST(RevoluteJoint, Construction)
{
	auto jd = RevoluteJointDef{};

	jd.bodyA = reinterpret_cast<Body*>(0x04);
	jd.bodyB = reinterpret_cast<Body*>(0x08);
	jd.collideConnected = true;
	jd.userData = reinterpret_cast<void*>(0x011);

	jd.localAnchorA = Vec2(4, 5);
	jd.localAnchorB = Vec2(6, 7);
	jd.enableLimit = true;
	jd.enableMotor = true;
	jd.motorSpeed = 4.4f;
	jd.maxMotorTorque = 1.0f;
	jd.lowerAngle = 33_deg;
	jd.upperAngle = 40_deg;
	jd.referenceAngle = 45_deg;
	
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
	const auto circle = std::make_shared<CircleShape>(0.2f);
	World world;
	const auto p1 = Vec2{-1, 0};
	const auto p2 = Vec2{+1, 0};
	const auto b1 = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(p1));
	const auto b2 = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(p2));
	b1->CreateFixture(circle);
	b2->CreateFixture(circle);
	auto jd = RevoluteJointDef{};
	jd.bodyA = b1;
	jd.bodyB = b2;
	world.CreateJoint(jd);
	Step(world, 1);
	EXPECT_EQ(b1->GetLocation(), Vec2(0, -4));
	EXPECT_EQ(b2->GetLocation(), Vec2(0, -4));
	EXPECT_EQ(b1->GetAngle(), 0_deg);
	EXPECT_EQ(b2->GetAngle(), 0_deg);
}

TEST(RevoluteJoint, MovesDynamicPolygons)
{
	const auto shape1 = std::make_shared<PolygonShape>();
	shape1->SetAsBox(1, 1);
	const auto shape2 = std::make_shared<PolygonShape>();
	shape2->SetAsBox(0.5, 0.5);
	World world;
	const auto p1 = Vec2{0, 0}; // Vec2{-1, 0};
	const auto p2 = Vec2{0, 0}; // Vec2{+1, 0};
	const auto b1 = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(p1));
	const auto b2 = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(p2));
	b1->CreateFixture(shape1);
	b2->CreateFixture(shape2);
	auto jd = RevoluteJointDef{};
	jd.bodyA = b1;
	jd.bodyB = b2;
	jd.localAnchorA = Vec2{1, 1};
	jd.localAnchorB = Vec2{0, 0};
	world.CreateJoint(jd);
	Step(world, 1);
	EXPECT_EQ(b1->GetLocation(), Vec2(-0.5f, -4.5f));
	EXPECT_EQ(b2->GetLocation(), Vec2(0.5f, -3.5f));
}
