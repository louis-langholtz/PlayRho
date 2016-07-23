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
#include <Box2D/Collision/Shapes/EdgeShape.h>

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
	EXPECT_EQ(body->GetType(), BodyType::Static);
	EXPECT_FALSE(body->IsSpeedable());
	EXPECT_FALSE(body->IsAccelerable());
	EXPECT_TRUE(body->IsImpenetrable());

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
	ASSERT_NE(body, nullptr);
	EXPECT_FALSE(body->IsImpenetrable());
	EXPECT_EQ(body->GetType(), BodyType::Dynamic);
	EXPECT_EQ(GetLinearVelocity(*body).x, 0);
	EXPECT_EQ(GetLinearVelocity(*body).y, 0);
	EXPECT_EQ(body->GetPosition().x, p0.x);
	EXPECT_EQ(body->GetPosition().y, p0.y);

	world.Step(t);
	EXPECT_EQ(GetLinearVelocity(*body).x, 0);
	EXPECT_EQ(GetLinearVelocity(*body).y, a * (t * 1));
	EXPECT_EQ(body->GetPosition().x, p0.x);
	EXPECT_EQ(body->GetPosition().y, p0.y + (GetLinearVelocity(*body).y * t));

	p0 = body->GetPosition();
	world.Step(t);
	EXPECT_EQ(GetLinearVelocity(*body).x, 0);
	EXPECT_EQ(GetLinearVelocity(*body).y, a * (t * 2));
	EXPECT_EQ(body->GetPosition().x, p0.x);
	EXPECT_EQ(body->GetPosition().y, p0.y + (GetLinearVelocity(*body).y * t));
	
	p0 = body->GetPosition();
	world.Step(t);
	EXPECT_EQ(GetLinearVelocity(*body).x, 0);
	EXPECT_EQ(GetLinearVelocity(*body).y, a * (t * 3));
	EXPECT_EQ(body->GetPosition().x, p0.x);
	EXPECT_EQ(body->GetPosition().y, p0.y + (GetLinearVelocity(*body).y * t));
}

class MyContactListener: public ContactListener
{
public:
	void BeginContact(Contact* contact) override
	{
		contacting = true;
		touching = contact->IsTouching();
		
		body_a[0] = contact->GetFixtureA()->GetBody()->GetPosition();
		body_b[0] = contact->GetFixtureB()->GetBody()->GetPosition();
	}
	
	void EndContact(Contact* contact) override
	{
		contacting = false;
		touching = contact->IsTouching();

		body_a[1] = contact->GetFixtureA()->GetBody()->GetPosition();
		body_b[1] = contact->GetFixtureB()->GetBody()->GetPosition();
	}
	
	void PreSolve(Contact* contact, const Manifold* oldManifold) override
	{
		touching = contact->IsTouching();		
	}
	
	bool contacting = false;
	bool touching = false;
	Vec2 body_a[2] = {Vec2_zero, Vec2_zero};
	Vec2 body_b[2] = {Vec2_zero, Vec2_zero};
};

TEST(World, CollidingDynamicBodies)
{
	const auto x = float_t(10); // other test parameters tuned to this value being 10

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
	fixtureDef.density = float_t(1);
	fixtureDef.restitution = float_t(1); // changes where bodies will be after collision

	body_def.position = Vec2{-(x + 1), 0};
	body_def.linearVelocity = Vec2{+x, 0};
	const auto body_a = world.CreateBody(body_def);
	ASSERT_NE(body_a, nullptr);
	EXPECT_EQ(body_a->GetType(), BodyType::Dynamic);
	EXPECT_TRUE(body_a->IsSpeedable());
	EXPECT_TRUE(body_a->IsAccelerable());

	const auto fixture1 = body_a->CreateFixture(fixtureDef);
	ASSERT_NE(fixture1, nullptr);

	body_def.position = Vec2{+(x + 1), 0};
	body_def.linearVelocity = Vec2{-x, 0};
	const auto body_b = world.CreateBody(body_def);
	ASSERT_NE(body_b, nullptr);
	const auto fixture2 = body_b->CreateFixture(fixtureDef);
	ASSERT_NE(fixture2, nullptr);
	EXPECT_EQ(body_b->GetType(), BodyType::Dynamic);
	EXPECT_TRUE(body_b->IsSpeedable());
	EXPECT_TRUE(body_b->IsAccelerable());

	EXPECT_EQ(GetLinearVelocity(*body_a).x, +x);
	EXPECT_EQ(GetLinearVelocity(*body_a).y, 0);
	EXPECT_EQ(GetLinearVelocity(*body_b).x, -x);
	EXPECT_EQ(GetLinearVelocity(*body_b).y, 0);
	
	const auto time_collision = float_t(1.0099994); // only valid for x >= around 4.214
	const auto time_inc = float_t(.01);
	
	auto elapsed_time = float_t(0);
	for (;;)
	{
		world.Step(time_inc);
		elapsed_time += time_inc;
		if (listener.contacting)
		{
			break;
		}
	}
	
	const auto time_contacting = elapsed_time;

	EXPECT_TRUE(listener.touching);
	EXPECT_FLOAT_EQ(time_contacting, time_collision);
	EXPECT_EQ(body_a->GetPosition().y, 0);
	EXPECT_EQ(body_b->GetPosition().y, 0);

	const auto tolerance = x / 100;
	
	// x position for body1 depends on restitution but it should be around -1
	EXPECT_GE(body_a->GetPosition().x, float_t(-1) - tolerance);
	EXPECT_LT(body_a->GetPosition().x, float_t(-1) + tolerance);

	// x position for body2 depends on restitution but it should be around +1
	EXPECT_LE(body_b->GetPosition().x, float_t(+1) + tolerance);
	EXPECT_GT(body_b->GetPosition().x, float_t(+1) - tolerance);
	
	// and their deltas from -1 and +1 should be about equal.
	EXPECT_FLOAT_EQ(body_a->GetPosition().x + 1, 1 - body_b->GetPosition().x);

	EXPECT_GE(listener.body_a[0].x, -1);
	EXPECT_LE(listener.body_b[0].x, +1);

	for (;;)
	{
		world.Step(time_inc);
		elapsed_time += time_inc;
		if (!listener.contacting && !listener.touching)
		{
			break;
		}
	}
	EXPECT_FALSE(listener.touching);
	
	EXPECT_FLOAT_EQ(elapsed_time, time_contacting + time_inc);
	
	// collision should be fully resolved now...
	EXPECT_LT(body_a->GetPosition().x, float_t(-1));
	EXPECT_GT(body_b->GetPosition().x, float_t(+1));
	
	// and their deltas from -1 and +1 should be about equal.
	EXPECT_FLOAT_EQ(body_a->GetPosition().x + 1, 1 - body_b->GetPosition().x);

	EXPECT_LT(listener.body_a[1].x, -1);
	EXPECT_GT(listener.body_b[1].x, +1);
	
	// confirm conservation of momentum:
	// velocities should now be same magnitude but in opposite directions
	EXPECT_EQ(GetLinearVelocity(*body_a).x, -x);
	EXPECT_EQ(GetLinearVelocity(*body_a).y, 0);
	EXPECT_EQ(GetLinearVelocity(*body_b).x, +x);
	EXPECT_EQ(GetLinearVelocity(*body_b).y, 0);
}

TEST(World, PreventsTunnelling)
{
	World world{Vec2_zero};
	
	BodyDef body_def;
	FixtureDef fixtureDef;
	EdgeShape edge_shape;
	CircleShape circle_shape;

	body_def.type = BodyType::Static;
	body_def.position = Vec2{0, 0};
	const auto wall_body = world.CreateBody(body_def);
	ASSERT_NE(wall_body, nullptr);

	edge_shape.Set(Vec2{0, +10}, Vec2{0, -10});
	fixtureDef.shape = &edge_shape;
	fixtureDef.restitution = float_t(1); // changes where bodies will be after collision
	const auto wall_fixture = wall_body->CreateFixture(fixtureDef);
	ASSERT_NE(wall_fixture, nullptr);
	
	body_def.type = BodyType::Dynamic;
	body_def.position = Vec2{-10, 0};
	const auto ball_body = world.CreateBody(body_def);
	ASSERT_NE(ball_body, nullptr);
	
	circle_shape.SetRadius(1);
	fixtureDef.shape = &circle_shape;
	fixtureDef.density = float_t(1);
	fixtureDef.restitution = float_t(1); // changes where bodies will be after collision
	const auto ball_fixture = ball_body->CreateFixture(fixtureDef);
	ASSERT_NE(ball_fixture, nullptr);

	const auto velocity = Vec2{+10, 0};
	ball_body->SetVelocity(Velocity{velocity, 0});
	
	const auto time_inc = float_t(.01);
	world.Step(time_inc);
	
	EXPECT_EQ(GetLinearVelocity(*ball_body).x, velocity.x);
	EXPECT_EQ(GetLinearVelocity(*ball_body).y, velocity.y);
}