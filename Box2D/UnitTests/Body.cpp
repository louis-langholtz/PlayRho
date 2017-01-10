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
#include <Box2D/Dynamics/Body.hpp>
#include <Box2D/Dynamics/World.hpp>
#include <Box2D/Dynamics/Fixture.hpp>
#include <Box2D/Dynamics/Joints/Joint.hpp>
#include <Box2D/Collision/Shapes/CircleShape.hpp>

#include <chrono>

using namespace box2d;

TEST(Body, ByteSizeIs160)
{
	// architecture dependent...
	EXPECT_EQ(sizeof(Body), size_t(160));
}

TEST(Body, WorldCreated)
{
	World world;
	
	auto body = world.CreateBody();
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

	auto body = world.CreateBody();
	ASSERT_NE(body, nullptr);
	EXPECT_TRUE(body->GetFixtures().empty());
	EXPECT_FALSE(body->IsMassDataDirty());

	const auto shape = std::make_shared<CircleShape>(2.871f, Vec2{1.912f, -77.31f});
	
	auto fixture = body->CreateFixture(shape, FixtureDef{}.UseDensity(1), false);
	ASSERT_NE(fixture, nullptr);
	ASSERT_NE(fixture->GetShape(), nullptr);
	EXPECT_EQ(fixture->GetShape()->GetType(), shape->GetType());
	EXPECT_EQ(GetVertexRadius(*fixture->GetShape()), GetVertexRadius(*shape));
	EXPECT_EQ(static_cast<const CircleShape*>(fixture->GetShape())->GetLocation().x, shape->GetLocation().x);
	EXPECT_EQ(static_cast<const CircleShape*>(fixture->GetShape())->GetLocation().y, shape->GetLocation().y);
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
	EXPECT_TRUE(body->IsMassDataDirty());
	body->ResetMassData();
	EXPECT_FALSE(body->IsMassDataDirty());

	body->DestroyFixture(fixture, false);
	EXPECT_TRUE(body->GetFixtures().empty());
	EXPECT_TRUE(body->IsMassDataDirty());
	
	body->ResetMassData();
	EXPECT_FALSE(body->IsMassDataDirty());
}

TEST(Body, CreateLotsOfFixtures)
{
	BodyDef bd;
	bd.type = BodyType::Dynamic;
	const auto shape = std::make_shared<CircleShape>(2.871f, Vec2{1.912f, -77.31f});
	const auto num = 5000;
	std::chrono::time_point<std::chrono::system_clock> start, end;
	
	start = std::chrono::system_clock::now();
	{
		World world;

		auto body = world.CreateBody(bd);
		ASSERT_NE(body, nullptr);
		EXPECT_TRUE(body->GetFixtures().empty());
		
		for (auto i = decltype(num){0}; i < num; ++i)
		{
			auto fixture = body->CreateFixture(shape, FixtureDef{}.UseDensity(1.3f), false);
			ASSERT_NE(fixture, nullptr);
		}
		body->ResetMassData();
		
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
	end = std::chrono::system_clock::now();
	const std::chrono::duration<double> elapsed_secs_resetting_at_end = end - start;

	start = std::chrono::system_clock::now();
	{
		World world;
		
		auto body = world.CreateBody(bd);
		ASSERT_NE(body, nullptr);
		EXPECT_TRUE(body->GetFixtures().empty());
		
		for (auto i = decltype(num){0}; i < num; ++i)
		{
			auto fixture = body->CreateFixture(shape, FixtureDef{}.UseDensity(1.3f), true);
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
	end = std::chrono::system_clock::now();
	const std::chrono::duration<double> elapsed_secs_resetting_in_create = end - start;

	EXPECT_LT(elapsed_secs_resetting_at_end.count(), elapsed_secs_resetting_in_create.count());
}
