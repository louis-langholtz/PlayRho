//
//  World.cpp
//  Box2D
//
//  Created by Louis D. Langholtz on 7/18/16.
//
//

#include "gtest/gtest.h"
#include <Box2D/Dynamics/World.h>
#include <Box2D/Dynamics/Body.h>

using namespace box2d;

TEST(World, DefaultInit)
{
	World world;

	EXPECT_EQ(GetBodyCount(world), body_count_t(0));
	EXPECT_EQ(world.GetProxyCount(), World::size_type(0));
	EXPECT_EQ(world.GetJointCount(), World::size_type(0));
	EXPECT_EQ(world.GetContactCount(), contact_count_t(0));
	EXPECT_EQ(world.GetTreeHeight(), World::size_type(0));
	EXPECT_EQ(world.GetTreeQuality(), float_t(0));

	EXPECT_EQ(world.GetGravity(), EarthlyGravity);
	
	EXPECT_TRUE(world.GetContinuousPhysics());
	EXPECT_TRUE(world.GetWarmStarting());
	EXPECT_TRUE(world.GetAllowSleeping());
	EXPECT_TRUE(world.GetAutoClearForces());
	
	EXPECT_TRUE(world.GetBodies().empty());
	EXPECT_EQ(world.GetBodies().size(), body_count_t(0));
	EXPECT_EQ(world.GetBodies().begin(), world.GetBodies().end());

	EXPECT_TRUE(world.GetContacts().empty());
	EXPECT_EQ(world.GetContacts().size(), contact_count_t(0));
	EXPECT_EQ(world.GetContacts().begin(), world.GetContacts().end());
	
	EXPECT_TRUE(world.GetJoints().empty());
	EXPECT_EQ(world.GetJoints().size(), World::size_type(0));
	EXPECT_EQ(world.GetJoints().begin(), world.GetJoints().end());
	
	EXPECT_FALSE(world.GetSubStepping());
}

TEST(World, Init)
{
	const auto gravity = Vec2{float_t(-4.2), float_t(3.4)};
	World world{gravity};
	EXPECT_EQ(world.GetGravity(), gravity);
}

TEST(World, SetGravity)
{
	const auto gravity = Vec2{float_t(-4.2), float_t(3.4)};
	World world;
	EXPECT_NE(world.GetGravity(), gravity);
	world.SetGravity(gravity);
	EXPECT_EQ(world.GetGravity(), gravity);	
	world.SetGravity(-gravity);
	EXPECT_NE(world.GetGravity(), gravity);
}

TEST(World, SetContinuousPhysics)
{
	World world;
	EXPECT_TRUE(world.GetContinuousPhysics());
	world.SetContinuousPhysics(false);
	EXPECT_FALSE(world.GetContinuousPhysics());
	world.SetContinuousPhysics(true);
	EXPECT_TRUE(world.GetContinuousPhysics());
}

TEST(World, CreateAndDestoryBody)
{
	BodyDef bodyDef;
	World world;
	EXPECT_EQ(GetBodyCount(world), body_count_t(0));

	const auto body = world.CreateBody(bodyDef);
	EXPECT_NE(body, nullptr);
	EXPECT_EQ(GetBodyCount(world), body_count_t(1));
	EXPECT_FALSE(world.GetBodies().empty());
	EXPECT_EQ(world.GetBodies().size(), body_count_t(1));
	EXPECT_NE(world.GetBodies().begin(), world.GetBodies().end());

	world.DestroyBody(body);
	EXPECT_EQ(GetBodyCount(world), body_count_t(0));
	EXPECT_TRUE(world.GetBodies().empty());
	EXPECT_EQ(world.GetBodies().size(), body_count_t(0));
	EXPECT_EQ(world.GetBodies().begin(), world.GetBodies().end());
}
