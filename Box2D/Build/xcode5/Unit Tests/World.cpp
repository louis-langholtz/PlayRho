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
#include <Box2D/Dynamics/Fixture.h>
#include <Box2D/Dynamics/Contacts/Contact.h>
#include <Box2D/Dynamics/Joints/DistanceJoint.h>
#include <Box2D/Collision/Shapes/CircleShape.h>

using namespace box2d;

TEST(World, DefaultInit)
{
	World world;

	EXPECT_EQ(GetBodyCount(world), body_count_t(0));
	EXPECT_EQ(world.GetProxyCount(), World::size_type(0));
	EXPECT_EQ(GetJointCount(world), World::size_type(0));
	EXPECT_EQ(GetContactCount(world), contact_count_t(0));
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
	EXPECT_FALSE(world.IsLocked());
}

TEST(World, Init)
{
	const auto gravity = Vec2{float_t(-4.2), float_t(3.4)};
	World world{gravity};
	EXPECT_EQ(world.GetGravity(), gravity);
	EXPECT_FALSE(world.IsLocked());
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

TEST(World, CreateAndDestroyBody)
{
	World world;
	EXPECT_EQ(GetBodyCount(world), body_count_t(0));

	const auto body = world.CreateBody(BodyDef{});
	EXPECT_NE(body, nullptr);
	EXPECT_EQ(GetBodyCount(world), body_count_t(1));
	EXPECT_FALSE(world.GetBodies().empty());
	EXPECT_EQ(world.GetBodies().size(), body_count_t(1));
	EXPECT_NE(world.GetBodies().begin(), world.GetBodies().end());
	const auto& first = *(world.GetBodies().begin());
	EXPECT_EQ(body, &first);

	world.DestroyBody(body);
	EXPECT_EQ(GetBodyCount(world), body_count_t(0));
	EXPECT_TRUE(world.GetBodies().empty());
	EXPECT_EQ(world.GetBodies().size(), body_count_t(0));
	EXPECT_EQ(world.GetBodies().begin(), world.GetBodies().end());
}

TEST(World, CreateAndDestroyJoint)
{
	World world;

	const auto body1 = world.CreateBody(BodyDef{});
	const auto body2 = world.CreateBody(BodyDef{});
	EXPECT_NE(body1, nullptr);
	EXPECT_NE(body2, nullptr);
	EXPECT_EQ(GetBodyCount(world), body_count_t(2));
	EXPECT_EQ(GetJointCount(world), joint_count_t(0));
	EXPECT_TRUE(world.GetJoints().empty());
	EXPECT_EQ(world.GetJoints().begin(), world.GetJoints().end());
	
	const auto anchorA = Vec2{float_t(+0.4), float_t(-1.2)};
	const auto anchorB = Vec2{float_t(-2.3), float_t(+0.7)};
	const auto joint = world.CreateJoint(DistanceJointDef{body1, body2, anchorA, anchorB});
	EXPECT_EQ(GetJointCount(world), joint_count_t(1));
	EXPECT_FALSE(world.GetJoints().empty());
	EXPECT_NE(world.GetJoints().begin(), world.GetJoints().end());
	const auto& first = *world.GetJoints().begin();
	EXPECT_EQ(joint, &first);
	EXPECT_EQ(joint->GetType(), JointType::Distance);
	EXPECT_EQ(joint->GetBodyA(), body1);
	EXPECT_EQ(joint->GetBodyB(), body2);
	EXPECT_EQ(joint->GetAnchorA(), anchorA);
	EXPECT_EQ(joint->GetAnchorB(), anchorB);
	EXPECT_FALSE(joint->GetCollideConnected());

	world.DestroyJoint(joint);
	EXPECT_EQ(GetJointCount(world), joint_count_t(0));
	EXPECT_TRUE(world.GetJoints().empty());
	EXPECT_EQ(world.GetJoints().begin(), world.GetJoints().end());
}

TEST(World, GravitationalBodyMovement)
{
	auto p0 = Vec2{0, 1};

	auto body_def = BodyDef{};
	body_def.type = BodyType::Dynamic;
	body_def.position = p0;

	const auto a = float_t(-10);
	const auto gravity = Vec2{0, a};
	const auto t = float_t(.01);
	
	World world{gravity};

	const auto body = world.CreateBody(body_def);
	EXPECT_EQ(body->GetLinearVelocity().x, 0);
	EXPECT_EQ(body->GetLinearVelocity().y, 0);
	EXPECT_EQ(body->GetPosition().x, p0.x);
	EXPECT_EQ(body->GetPosition().y, p0.y);

	world.Step(t);
	EXPECT_EQ(body->GetLinearVelocity().x, 0);
	EXPECT_EQ(body->GetLinearVelocity().y, a * (t * 1));
	EXPECT_EQ(body->GetPosition().x, p0.x);
	EXPECT_EQ(body->GetPosition().y, p0.y + (body->GetLinearVelocity().y * t));

	p0 = body->GetPosition();
	world.Step(t);
	EXPECT_EQ(body->GetLinearVelocity().x, 0);
	EXPECT_EQ(body->GetLinearVelocity().y, a * (t * 2));
	EXPECT_EQ(body->GetPosition().x, p0.x);
	EXPECT_EQ(body->GetPosition().y, p0.y + (body->GetLinearVelocity().y * t));
	
	p0 = body->GetPosition();
	world.Step(t);
	EXPECT_EQ(body->GetLinearVelocity().x, 0);
	EXPECT_EQ(body->GetLinearVelocity().y, a * (t * 3));
	EXPECT_EQ(body->GetPosition().x, p0.x);
	EXPECT_EQ(body->GetPosition().y, p0.y + (body->GetLinearVelocity().y * t));
}

class MyContactListener: public ContactListener
{
public:
	void BeginContact(Contact* contact) override
	{
		contacting = true;
		touching = contact->IsTouching();
	}
	
	void EndContact(Contact* contact) override
	{
		contacting = false;
		touching = contact->IsTouching();
	}
	
	void PreSolve(Contact* contact, const Manifold* oldManifold) override
	{
		touching = contact->IsTouching();		
	}
	
	bool contacting = false;
	bool touching = false;
};

TEST(World, CollidingBodies)
{
	const auto x = float_t(10);

	auto body_def = BodyDef{};
	body_def.type = BodyType::Dynamic;
	
	MyContactListener listener;

	const auto gravity = Vec2_zero;
	World world{gravity};
	EXPECT_EQ(world.GetGravity(), gravity);
	world.SetContactListener(&listener);
	
	CircleShape shape{1};
	FixtureDef fixtureDef;
	fixtureDef.shape = &shape;
	fixtureDef.density = 1;
	fixtureDef.restitution = 0;

	body_def.position = Vec2{-(x + 1), 0};
	body_def.linearVelocity = Vec2{+x, 0};
	const auto body1 = world.CreateBody(body_def);
	ASSERT_NE(body1, nullptr);
	const auto fixture1 = body1->CreateFixture(fixtureDef);
	ASSERT_NE(fixture1, nullptr);

	body_def.position = Vec2{+(x + 1), 0};
	body_def.linearVelocity = Vec2{-x, 0};
	const auto body2 = world.CreateBody(body_def);
	ASSERT_NE(body2, nullptr);
	const auto fixture2 = body2->CreateFixture(fixtureDef);
	ASSERT_NE(fixture2, nullptr);
	
	const auto t = float_t(.01);
	auto elapsed_time = float_t(0);
	for (;;)
	{
		world.Step(t);
		elapsed_time += t;
		if (listener.contacting)
		{
			break;
		}
	}
	EXPECT_TRUE(listener.touching);

	EXPECT_FLOAT_EQ(elapsed_time, float_t(1.0099994));

	const auto expected_x = float_t(0.9999944);
	
	EXPECT_EQ(body1->GetPosition().y, 0);
	EXPECT_GT(body1->GetPosition().x, -1);
	EXPECT_LT(body1->GetPosition().x, 0);
	EXPECT_FLOAT_EQ(body1->GetPosition().x, -expected_x);

	EXPECT_EQ(body2->GetPosition().y, 0);
	EXPECT_LT(body2->GetPosition().x, +1);
	EXPECT_GT(body2->GetPosition().x, 0);
	EXPECT_FLOAT_EQ(body2->GetPosition().x, +expected_x);
#if 0
	for (;;)
	{
		world.Step(t);
		elapsed_time += t;
		if (!listener.contacting)
		{
			break;
		}
	}
	EXPECT_FALSE(listener.touching);
	
	EXPECT_FLOAT_EQ(elapsed_time, float_t(1.0099994));
#endif
}
