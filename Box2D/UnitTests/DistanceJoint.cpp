/*
 * Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/Box2D
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
#include <Box2D/Dynamics/Joints/DistanceJoint.hpp>
#include <Box2D/Dynamics/World.hpp>
#include <Box2D/Dynamics/StepConf.hpp>
#include <Box2D/Dynamics/Body.hpp>
#include <Box2D/Dynamics/Fixture.hpp>
#include <Box2D/Collision/Shapes/CircleShape.hpp>

using namespace box2d;

TEST(DistanceJoint, ByteSize)
{
	switch (sizeof(RealNum))
	{
		case  4: EXPECT_EQ(sizeof(DistanceJoint), size_t(120)); break;
		case  8: EXPECT_EQ(sizeof(DistanceJoint), size_t(240)); break;
		case 16: EXPECT_EQ(sizeof(DistanceJoint), size_t(480)); break;
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
	
	EXPECT_EQ(def.localAnchorA, Vec2_zero);
	EXPECT_EQ(def.localAnchorB, Vec2_zero);
	EXPECT_EQ(def.length, RealNum(1));
	EXPECT_EQ(def.frequencyHz, RealNum(0));
	EXPECT_EQ(def.dampingRatio, RealNum(0));
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
	EXPECT_EQ(joint.GetFrequency(), def.frequencyHz);
	EXPECT_EQ(joint.GetDampingRatio(), def.dampingRatio);
}

TEST(DistanceJoint, InZeroGravBodiesMoveOutToLength)
{
	World world{World::Def{}.UseGravity(Vec2_zero * MeterPerSquareSecond)};

	const auto shape = std::make_shared<CircleShape>(0.2f);
	
	const auto location1 = Vec2{-1, 0};
	const auto body1 = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(location1));
	ASSERT_EQ(body1->GetLocation(), location1);
	ASSERT_NE(body1->CreateFixture(shape), nullptr);
	
	const auto location2 = Vec2{+1, 0};
	const auto body2 = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(location2));
	ASSERT_EQ(body2->GetLocation(), location2);
	ASSERT_NE(body2->CreateFixture(shape), nullptr);
	
	DistanceJointDef jointdef;
	jointdef.bodyA = body1;
	jointdef.bodyB = body2;
	jointdef.collideConnected = false;
	jointdef.localAnchorA = Vec2_zero;
	jointdef.localAnchorB = Vec2_zero;
	jointdef.length = 5;
	jointdef.frequencyHz = 0;
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
			EXPECT_NEAR(double(newDistance), double(oldDistance), 0.01);
		}
		else
		{
			EXPECT_GE(newDistance, oldDistance);
		}
		
		if (!distanceMet && (std::abs(newDistance - jointdef.length) < 0.01))
		{
			distanceMet = i;
		}
		oldDistance = newDistance;
	}
}

TEST(DistanceJoint, InZeroGravBodiesMoveInToLength)
{
	World world{World::Def{}.UseGravity(Vec2{0, 10} * MeterPerSquareSecond)};
	
	const auto shape = std::make_shared<CircleShape>(0.2f);
	shape->SetDensity(RealNum{1} * KilogramPerSquareMeter);
	
	const auto location1 = Vec2{-10, 10};
	const auto body1 = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(location1));
	ASSERT_EQ(body1->GetLocation(), location1);
	ASSERT_NE(body1->CreateFixture(shape), nullptr);
	
	const auto location2 = Vec2{+10, -10};
	const auto body2 = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(location2));
	ASSERT_EQ(body2->GetLocation(), location2);
	ASSERT_NE(body2->CreateFixture(shape), nullptr);
	
	DistanceJointDef jointdef;
	jointdef.bodyA = body1;
	jointdef.bodyB = body2;
	jointdef.collideConnected = false;
	jointdef.localAnchorA = Vec2_zero;
	jointdef.localAnchorB = Vec2_zero;
	jointdef.length = 5;
	jointdef.frequencyHz = 60;
	jointdef.dampingRatio = 0;
	EXPECT_NE(world.CreateJoint(jointdef), nullptr);
	
	auto oldDistance = GetLength(body1->GetLocation() - body2->GetLocation());
	
	auto distanceMet = 0u;
	StepConf stepConf;
	for (auto i = 0u; !distanceMet || i < distanceMet + 1000; ++i)
	{
		world.Step(stepConf);
		
		const auto newDistance = GetLength(body1->GetLocation() - body2->GetLocation());
		if (!distanceMet && (newDistance - oldDistance) >= 0)
		{
			distanceMet = i;
		}
		if (distanceMet)
		{
			EXPECT_NEAR(double(newDistance), double(oldDistance), 2.5);
		}
		else
		{
			EXPECT_LE(newDistance, oldDistance);
		}
		
		oldDistance = newDistance;
	}
	
	EXPECT_NEAR(double(oldDistance), double(jointdef.length), 0.1);
}
