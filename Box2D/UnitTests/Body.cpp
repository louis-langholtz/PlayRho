//
//  Body.cpp
//  Box2D
//
//  Created by Louis D. Langholtz on 8/10/16.
//
//

#include "gtest/gtest.h"
#include <Box2D/Dynamics/Body.h>
#include <Box2D/Dynamics/World.h>
#include <Box2D/Dynamics/Fixture.h>
#include <Box2D/Dynamics/Joints/Joint.h>
#include <Box2D/Collision/Shapes/CircleShape.h>

using namespace box2d;

TEST(Body, DefaultCreation)
{
	World world;
	
	auto body = world.CreateBody(BodyDef{});
	ASSERT_NE(body, nullptr);

	EXPECT_EQ(body->GetWorld(), &world);
	EXPECT_EQ(body->GetUserData(), nullptr);
	EXPECT_TRUE(body->IsAwake());
	EXPECT_TRUE(body->IsActive());
	EXPECT_FALSE(body->IsSpeedable());
	EXPECT_FALSE(body->IsAccelerable());
	
	EXPECT_TRUE(body->GetFixtures().empty());
	{
		int i = 0;
		for (auto&& fixture: body->GetFixtures())
		{
			EXPECT_EQ(fixture.GetBody(), body);
			++i;
		}
		EXPECT_EQ(i, 0);
	}

	EXPECT_TRUE(body->GetJoints().empty());
	{
		int i = 0;
		for (auto&& joint: body->GetJoints())
		{
			BOX2D_NOT_USED(joint);
			++i;
		}
		EXPECT_EQ(i, 0);		
	}
	
	EXPECT_TRUE(body->GetContactEdges().empty());
	{
		int i = 0;
		for (auto&& ce: body->GetContactEdges())
		{
			BOX2D_NOT_USED(ce);
			++i;
		}
		EXPECT_EQ(i, 0);		
	}
}

TEST(Body, CreateAndDestroyFixture)
{
	World world;

	auto body = world.CreateBody(BodyDef{});
	ASSERT_NE(body, nullptr);
	EXPECT_TRUE(body->GetFixtures().empty());

	CircleShape shape{float_t(2.871), Vec2{float_t(1.912), float_t(-77.31)}};
	
	auto fixture = body->CreateFixture(FixtureDef{&shape, 0});
	ASSERT_NE(fixture, nullptr);
	ASSERT_NE(fixture->GetShape(), nullptr);
	EXPECT_EQ(fixture->GetShape()->GetType(), shape.GetType());
	EXPECT_EQ(fixture->GetShape()->GetRadius(), shape.GetRadius());
	EXPECT_EQ(static_cast<CircleShape*>(fixture->GetShape())->GetPosition().x, shape.GetPosition().x);
	EXPECT_EQ(static_cast<CircleShape*>(fixture->GetShape())->GetPosition().y, shape.GetPosition().y);
	EXPECT_FALSE(body->GetFixtures().empty());
	{
		int i = 0;
		for (auto&& f: body->GetFixtures())
		{
			EXPECT_EQ(&f, fixture);
			++i;
		}
		EXPECT_EQ(i, 1);
	}

	body->DestroyFixture(fixture);
	EXPECT_TRUE(body->GetFixtures().empty());
}

TEST(Body, CreateFixtures)
{
	World world;
	
	auto body = world.CreateBody(BodyDef{});
	ASSERT_NE(body, nullptr);
	EXPECT_TRUE(body->GetFixtures().empty());
	
	CircleShape shape{float_t(2.871), Vec2{float_t(1.912), float_t(-77.31)}};
	
	const auto num = 10000;
	
	for (auto i = decltype(num){0}; i < num; ++i)
	{
		auto fixture = body->CreateFixture(FixtureDef{&shape, 0});
		ASSERT_NE(fixture, nullptr);
	}
	
	EXPECT_FALSE(body->GetFixtures().empty());
	{
		int i = decltype(num){0};
		for (auto&& f: body->GetFixtures())
		{
			BOX2D_NOT_USED(f);
			++i;
		}
		EXPECT_EQ(i, num);
	}
}