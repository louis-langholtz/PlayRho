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
#include <Box2D/Dynamics/World.h>
#include <Box2D/Dynamics/Body.h>
#include <Box2D/Dynamics/Fixture.h>
#include <Box2D/Dynamics/Contacts/Contact.h>
#include <Box2D/Dynamics/Joints/DistanceJoint.h>
#include <Box2D/Collision/Shapes/CircleShape.h>
#include <Box2D/Collision/Shapes/PolygonShape.h>
#include <Box2D/Collision/Shapes/EdgeShape.h>
#include <Box2D/Dynamics/Joints/MouseJoint.h>
#include <Box2D/Dynamics/Joints/RopeJoint.h>
#include <Box2D/Common/Angle.hpp>

#include <unistd.h>
#include <setjmp.h>
#include <signal.h>

using namespace box2d;

TEST(World, ByteSizeIs432)
{
	EXPECT_EQ(sizeof(World), size_t(432));
}

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
	World world{World::Def{}.UseGravity(gravity)};
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

	world.Destroy(body);
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

	world.Destroy(joint);
	EXPECT_EQ(GetJointCount(world), joint_count_t(0));
	EXPECT_TRUE(world.GetJoints().empty());
	EXPECT_EQ(world.GetJoints().begin(), world.GetJoints().end());
}

TEST(World, MaxBodies)
{
	World world;
	for (auto i = decltype(MaxBodies){0}; i < MaxBodies; ++i)
	{
		const auto body = world.CreateBody(BodyDef{});
		ASSERT_NE(body, nullptr);
	}
	{
		const auto body = world.CreateBody(BodyDef{});
		EXPECT_EQ(body, nullptr);		
	}
}

TEST(World, MaxJoints)
{
	World world;
	
	const auto body1 = world.CreateBody(BodyDef{});
	ASSERT_NE(body1, nullptr);
	const auto body2 = world.CreateBody(BodyDef{});
	ASSERT_NE(body2, nullptr);
	
	for (auto i = decltype(MaxJoints){0}; i < MaxJoints; ++i)
	{
		const auto joint = world.CreateJoint(RopeJointDef{body1, body2});
		ASSERT_NE(joint, nullptr);
	}
	{
		const auto joint = world.CreateJoint(RopeJointDef{body1, body2});
		EXPECT_EQ(joint, nullptr);
	}
}

TEST(World, StepZeroTimeDoesNothing)
{
	const auto gravity = Vec2{0, float_t(-9.8)};
	
	World world{World::Def{}.UseGravity(gravity)};
	
	BodyDef def;
	def.position = Vec2{float_t(31.9), float_t(-19.24)};
	def.type = BodyType::Dynamic;
	
	const auto body = world.CreateBody(def);
	ASSERT_NE(body, nullptr);
	EXPECT_EQ(body->GetPosition().x, def.position.x);
	EXPECT_EQ(body->GetPosition().y, def.position.y);
	EXPECT_EQ(GetLinearVelocity(*body).x, float_t(0));
	EXPECT_EQ(GetLinearVelocity(*body).y, float_t(0));
	EXPECT_EQ(body->GetLinearAcceleration().x, 0);
	EXPECT_EQ(body->GetLinearAcceleration().y, gravity.y);
	
	const auto time_inc = float_t(0);
	
	auto pos = body->GetPosition();
	auto vel = GetLinearVelocity(*body);
	for (auto i = 0; i < 100; ++i)
	{
		world.Step(time_inc);
		
		EXPECT_EQ(body->GetLinearAcceleration().y, gravity.y);
		
		EXPECT_EQ(body->GetPosition().x, def.position.x);
		EXPECT_EQ(body->GetPosition().y, pos.y);
		pos = body->GetPosition();
		
		EXPECT_EQ(GetLinearVelocity(*body).x, float_t(0));
		EXPECT_FLOAT_EQ(GetLinearVelocity(*body).y, vel.y);
		vel = GetLinearVelocity(*body);
	}
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
	
	World world{World::Def{}.UseGravity(gravity)};

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

TEST(World, BodyAccelPerSpecWithNoVelOrPosIterations)
{
	const auto gravity = Vec2{0, float_t(-9.8)};
	
	World world{World::Def{}.UseGravity(gravity)};
	
	BodyDef def;
	def.position = Vec2{float_t(31.9), float_t(-19.24)};
	def.type = BodyType::Dynamic;
	
	const auto body = world.CreateBody(def);
	ASSERT_NE(body, nullptr);
	EXPECT_EQ(body->GetPosition().x, def.position.x);
	EXPECT_EQ(body->GetPosition().y, def.position.y);
	EXPECT_EQ(GetLinearVelocity(*body).x, float_t(0));
	EXPECT_EQ(GetLinearVelocity(*body).y, float_t(0));
	EXPECT_EQ(body->GetLinearAcceleration().x, 0);
	EXPECT_EQ(body->GetLinearAcceleration().y, gravity.y);
	
	const auto time_inc = float_t(0.01);
	
	auto pos = body->GetPosition();
	auto vel = GetLinearVelocity(*body);
	for (auto i = 0; i < 100; ++i)
	{
		world.Step(time_inc, 0, 0);
		
		EXPECT_EQ(body->GetLinearAcceleration().y, gravity.y);
		
		EXPECT_EQ(body->GetPosition().x, def.position.x);
		EXPECT_LT(body->GetPosition().y, pos.y);
		EXPECT_EQ(body->GetPosition().y, pos.y + (vel.y + gravity.y * time_inc) * time_inc);
		pos = body->GetPosition();
		
		EXPECT_EQ(GetLinearVelocity(*body).x, float_t(0));
		EXPECT_LT(GetLinearVelocity(*body).y, vel.y);
		EXPECT_FLOAT_EQ(GetLinearVelocity(*body).y, vel.y + gravity.y * time_inc);
		vel = GetLinearVelocity(*body);
	}
}

class MyContactListener: public ContactListener
{
public:
	using PreSolver = std::function<void(Contact&, const Manifold&)>;
	using PostSolver = std::function<void(Contact&, const ContactImpulse&, ContactListener::iteration_type)>;
	using Ender = std::function<void(Contact&)>;

	MyContactListener(PreSolver&& pre, PostSolver&& post, Ender&& end): presolver(pre), postsolver(post), ender(end) {}

	virtual ~MyContactListener() {}

	void BeginContact(Contact& contact) override
	{
		++begin_contacts;
		contacting = true;
		touching = contact.IsTouching();
		
		body_a[0] = contact.GetFixtureA()->GetBody()->GetPosition();
		body_b[0] = contact.GetFixtureB()->GetBody()->GetPosition();
	}
	
	void EndContact(Contact& contact) override
	{
		++end_contacts;
		contacting = false;
		touching = contact.IsTouching();

		body_a[1] = contact.GetFixtureA()->GetBody()->GetPosition();
		body_b[1] = contact.GetFixtureB()->GetBody()->GetPosition();
		
		ender(contact);
	}
	
	void PreSolve(Contact& contact, const Manifold& oldManifold) override
	{
		++pre_solves;
		presolver(contact, oldManifold);
	}
	
	void PostSolve(Contact& contact, const ContactImpulse& impulse, ContactListener::iteration_type solved) override
	{
		++post_solves;
		postsolver(contact, impulse, solved);
	}

	unsigned begin_contacts = 0;
	unsigned end_contacts = 0;
	unsigned pre_solves = 0;
	unsigned post_solves = 0;
	bool contacting = false;
	bool touching = false;
	Vec2 body_a[2] = {Vec2_zero, Vec2_zero};
	Vec2 body_b[2] = {Vec2_zero, Vec2_zero};
	PreSolver presolver;
	PostSolver postsolver;
	Ender ender;
};

TEST(World, NoCorrectionsWithNoVelOrPosIterations)
{
	const auto x = float_t(10); // other test parameters tuned to this value being 10

	auto presolved = unsigned{0};
	auto postsolved = unsigned{0};
	MyContactListener listener{
		[&](Contact& contact, const Manifold& oldManifold) { ++presolved; },
		[&](Contact& contact, const ContactImpulse& impulse, ContactListener::iteration_type solved) { ++postsolved; },
		[&](Contact& contact) {},
	};

	const Vec2 gravity{0, 0};
	World world{World::Def{}.UseGravity(gravity)};
	world.SetContactListener(&listener);
	
	ASSERT_EQ(listener.begin_contacts, unsigned(0));
	ASSERT_EQ(listener.end_contacts, unsigned(0));
	
	auto body_def = BodyDef{};
	body_def.type = BodyType::Dynamic;
	body_def.bullet = true;
	
	CircleShape shape{1};
	FixtureDef fixtureDef;
	fixtureDef.shape = &shape;
	fixtureDef.density = float_t(1);
	fixtureDef.restitution = float_t(1);
	
	body_def.position = Vec2{-x, 0};
	body_def.linearVelocity = Vec2{+x, 0};
	const auto body_a = world.CreateBody(body_def);
	ASSERT_NE(body_a, nullptr);
	EXPECT_EQ(body_a->GetType(), BodyType::Dynamic);
	EXPECT_TRUE(body_a->IsSpeedable());
	EXPECT_TRUE(body_a->IsAccelerable());
	const auto fixture1 = body_a->CreateFixture(fixtureDef);
	ASSERT_NE(fixture1, nullptr);
	
	body_def.position = Vec2{+x, 0};
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

	const auto time_inc = float_t(.01);

	auto pos_a = body_a->GetPosition();
	auto pos_b = body_b->GetPosition();
	ASSERT_LT(pos_a.x, pos_b.x);

	auto steps = unsigned{0};
	while (pos_a.x < x && pos_b.x > -x)
	{
		world.Step(time_inc, 0, 0);
		++steps;
		
		EXPECT_EQ(body_a->GetPosition().x, pos_a.x + x * time_inc);
		EXPECT_EQ(body_a->GetPosition().y, 0);
		EXPECT_EQ(body_b->GetPosition().x, pos_b.x - x * time_inc);
		EXPECT_EQ(body_b->GetPosition().y, 0);

		EXPECT_EQ(GetLinearVelocity(*body_a).x, +x);
		EXPECT_EQ(GetLinearVelocity(*body_a).y, 0);
		EXPECT_EQ(GetLinearVelocity(*body_b).x, -x);
		EXPECT_EQ(GetLinearVelocity(*body_b).y, 0);

		pos_a = body_a->GetPosition();
		pos_b = body_b->GetPosition();
	}
	
	// d = v * t
	// d = 20, v = 10:
	// 20 = 10 * t, t = d/v = 20 / 10 = 2
	// steps = t / time_inc = 200
	EXPECT_EQ(steps, ((x * 2) / x) / time_inc);
}

TEST(World, PerfectlyOverlappedIdenticalCirclesStayPut)
{
	const auto radius = float_t(1);
	const CircleShape shape{radius};
	const Vec2 gravity{0, 0};

	World world{World::Def{}.UseGravity(gravity)};
	
	auto body_def = BodyDef{};
	body_def.type = BodyType::Dynamic;
	body_def.bullet = false;
	body_def.position = Vec2{float_t(0), float_t(0)};

	FixtureDef fixtureDef;
	fixtureDef.shape = &shape;
	fixtureDef.density = float_t(1);
	fixtureDef.restitution = float_t(1); // changes where bodies will be after collision

	const auto body1 = world.CreateBody(body_def);
	{
		const auto fixture = body1->CreateFixture(fixtureDef);
		ASSERT_NE(fixture, nullptr);
	}
	ASSERT_EQ(body1->GetPosition().x, body_def.position.x);
	ASSERT_EQ(body1->GetPosition().y, body_def.position.y);
	
	const auto body2 = world.CreateBody(body_def);
	{
		const auto fixture = body2->CreateFixture(fixtureDef);
		ASSERT_NE(fixture, nullptr);
	}
	ASSERT_EQ(body2->GetPosition().x, body_def.position.x);
	ASSERT_EQ(body2->GetPosition().y, body_def.position.y);
	
	const auto time_inc = float_t(.01);
	for (auto i = 0; i < 100; ++i)
	{
		world.Step(time_inc);
		EXPECT_EQ(body1->GetPosition().x, body_def.position.x);
		EXPECT_EQ(body1->GetPosition().y, body_def.position.y);
		EXPECT_EQ(body2->GetPosition().x, body_def.position.x);
		EXPECT_EQ(body2->GetPosition().y, body_def.position.y);
	}
}

TEST(World, PerfectlyOverlappedConcentricCirclesStayPut)
{
	const auto radius1 = float_t(1);
	const auto radius2 = float_t(0.6);
	const CircleShape shape1{radius1};
	const CircleShape shape2{radius2};
	const Vec2 gravity{0, 0};
	
	World world{World::Def{}.UseGravity(gravity)};
	
	auto body_def = BodyDef{};
	body_def.type = BodyType::Dynamic;
	body_def.bullet = false;
	body_def.position = Vec2{float_t(0), float_t(0)};
	
	FixtureDef fixtureDef1;
	fixtureDef1.shape = &shape1;
	fixtureDef1.density = float_t(1);
	fixtureDef1.restitution = float_t(1); // changes where bodies will be after collision
	
	FixtureDef fixtureDef2;
	fixtureDef2.shape = &shape2;
	fixtureDef2.density = float_t(1);
	fixtureDef2.restitution = float_t(1); // changes where bodies will be after collision

	const auto body1 = world.CreateBody(body_def);
	{
		const auto fixture = body1->CreateFixture(fixtureDef1);
		ASSERT_NE(fixture, nullptr);
	}
	ASSERT_EQ(body1->GetPosition().x, body_def.position.x);
	ASSERT_EQ(body1->GetPosition().y, body_def.position.y);
	
	const auto body2 = world.CreateBody(body_def);
	{
		const auto fixture = body2->CreateFixture(fixtureDef2);
		ASSERT_NE(fixture, nullptr);
	}
	ASSERT_EQ(body2->GetPosition().x, body_def.position.x);
	ASSERT_EQ(body2->GetPosition().y, body_def.position.y);
	
	const auto time_inc = float_t(.01);
	for (auto i = 0; i < 100; ++i)
	{
		world.Step(time_inc);
		EXPECT_EQ(body1->GetPosition().x, body_def.position.x);
		EXPECT_EQ(body1->GetPosition().y, body_def.position.y);
		EXPECT_EQ(body2->GetPosition().x, body_def.position.x);
		EXPECT_EQ(body2->GetPosition().y, body_def.position.y);
	}
}

TEST(World, PartiallyOverlappedCirclesSeparate)
{
	const auto radius = float_t(1);
	
	const Vec2 gravity{0, 0};
	World world{World::Def{}.UseGravity(gravity)};
	
	auto body_def = BodyDef{};
	body_def.type = BodyType::Dynamic;
	body_def.bullet = false; // separation is faster if true.
	
	CircleShape shape{radius};
	FixtureDef fixtureDef;
	fixtureDef.shape = &shape;
	fixtureDef.density = float_t(1);
	fixtureDef.restitution = float_t(1); // changes where bodies will be after collision
	
	const auto body1pos = Vec2{float_t(-radius/4), float_t(0)};
	body_def.position = body1pos;
	const auto body1 = world.CreateBody(body_def);
	{
		const auto fixture = body1->CreateFixture(fixtureDef);
		ASSERT_NE(fixture, nullptr);
	}
	ASSERT_EQ(body1->GetPosition().x, body_def.position.x);
	ASSERT_EQ(body1->GetPosition().y, body_def.position.y);
	
	const auto body2pos = Vec2{float_t(radius/4), float_t(0)};
	body_def.position = body2pos;
	const auto body2 = world.CreateBody(body_def);
	{
		const auto fixture = body2->CreateFixture(fixtureDef);
		ASSERT_NE(fixture, nullptr);
	}
	ASSERT_EQ(body2->GetPosition().x, body_def.position.x);
	ASSERT_EQ(body2->GetPosition().y, body_def.position.y);
	
	auto position_diff = body2pos - body1pos;
	auto distance = GetLength(position_diff);

	const auto angle = GetAngle(position_diff);

	auto lastpos1 = body1->GetPosition();
	auto lastpos2 = body2->GetPosition();

	const auto time_inc = float_t(.01);
	// Solver won't separate more than -world.GetLinearSlop().
	const auto full_separation = radius * 2 - world.GetLinearSlop();
	for (auto i = 0; i < 100; ++i)
	{
		world.Step(time_inc);

		const auto new_pos_diff = body2->GetPosition() - body1->GetPosition();
		const auto new_distance = GetLength(new_pos_diff);
		
		if (almost_equal(new_distance, full_separation) || new_distance > full_separation)
		{
			break;
		}
		
		if (new_distance == distance)
		{
			if (cos(angle.ToRadians()) != 0)
			{
				EXPECT_NE(body1->GetPosition().x, lastpos1.x);
				EXPECT_NE(body2->GetPosition().x, lastpos2.x);
			}
			if (sin(angle.ToRadians()) != 0)
			{
				EXPECT_NE(body1->GetPosition().y, lastpos1.y);
				EXPECT_NE(body2->GetPosition().y, lastpos2.y);
			}
			ASSERT_GE(new_distance, float_t(2));
			break;
		}

		ASSERT_NE(body1->GetPosition(), lastpos1);
		ASSERT_NE(body2->GetPosition(), lastpos2);
		
		lastpos1 = body1->GetPosition();
		lastpos2 = body2->GetPosition();

		ASSERT_NE(new_pos_diff, position_diff);
		position_diff = new_pos_diff;

		ASSERT_NE(new_distance, distance);
		distance = new_distance;

		// angle of the delta of their positions should stay the same as they move away
		const auto new_angle = GetAngle(new_pos_diff);
		EXPECT_EQ(angle, new_angle);
	}
}

TEST(World, PerfectlyOverlappedIdenticalSquaresSeparate)
{
	const auto shape = PolygonShape(1, 1);
	const Vec2 gravity{0, 0};
	
	World world{World::Def{}.UseGravity(gravity)};
	
	auto body_def = BodyDef{};
	body_def.type = BodyType::Dynamic;
	body_def.bullet = false;
	body_def.position = Vec2{float_t(0), float_t(0)};
	
	FixtureDef fixtureDef;
	fixtureDef.shape = &shape;
	fixtureDef.density = float_t(1);
	fixtureDef.restitution = float_t(1); // changes where bodies will be after collision
	
	const auto body1 = world.CreateBody(body_def);
	{
		const auto fixture = body1->CreateFixture(fixtureDef);
		ASSERT_NE(fixture, nullptr);
	}
	ASSERT_EQ(body1->GetPosition().x, body_def.position.x);
	ASSERT_EQ(body1->GetPosition().y, body_def.position.y);
	
	const auto body2 = world.CreateBody(body_def);
	{
		const auto fixture = body2->CreateFixture(fixtureDef);
		ASSERT_NE(fixture, nullptr);
	}
	ASSERT_EQ(body2->GetPosition().x, body_def.position.x);
	ASSERT_EQ(body2->GetPosition().y, body_def.position.y);
	
	auto lastpos1 = body1->GetPosition();
	auto lastpos2 = body2->GetPosition();

	const auto time_inc = float_t(.01);
	for (auto i = 0; i < 100; ++i)
	{
		world.Step(time_inc);
		
		// body1 moves left only
		EXPECT_LT(body1->GetPosition().x, lastpos1.x);
		EXPECT_EQ(body1->GetPosition().y, lastpos1.y);

		// body2 moves right only
		EXPECT_GT(body2->GetPosition().x, lastpos2.x);
		EXPECT_EQ(body2->GetPosition().y, lastpos2.y);
		
		// body1 and body2 move away from each other equally.
		EXPECT_EQ(body1->GetPosition().x, -body2->GetPosition().x);
		EXPECT_EQ(body1->GetPosition().y, -body2->GetPosition().y);
		
		lastpos1 = body1->GetPosition();
		lastpos2 = body2->GetPosition();
	}
}

TEST(World, PartiallyOverlappedSquaresSeparateProperly)
{
	/*
	 * Sets up 2 equally sized squares - body A and body B - where body A is to the right of body B
	 * but they partially overlap. Position solver code should move body A to the right more and
	 * move body B to the left more till they're almost separated.
	 *
	 * This tests at a high level what the position solver code does with overlapping shapes.
	 */

	const Vec2 gravity{0, 0};
	World world{World::Def{}.UseGravity(gravity)};
	
	auto body_def = BodyDef{};
	body_def.type = BodyType::Dynamic;
	body_def.bullet = false; // separation is faster if true.
	
	const auto half_dim = float_t(64); // 1 causes additional y-axis separation
	const auto shape = PolygonShape{half_dim, half_dim};
	FixtureDef fixtureDef;
	fixtureDef.shape = &shape;
	fixtureDef.density = float_t(1);
	fixtureDef.restitution = float_t(1); // changes where bodies will be after collision
	
	const auto body1pos = Vec2{float_t(half_dim/2), float_t(0)}; // 0 causes additional y-axis separation
	body_def.position = body1pos;
	const auto body1 = world.CreateBody(body_def);
	{
		const auto fixture = body1->CreateFixture(fixtureDef);
		ASSERT_NE(fixture, nullptr);
	}
	ASSERT_EQ(body1->GetPosition().x, body1pos.x);
	ASSERT_EQ(body1->GetPosition().y, body1pos.y);
	
	const auto body2pos = Vec2{-float_t(half_dim/2), float_t(0)}; // 0 causes additional y-axis separation
	body_def.position = body2pos;
	const auto body2 = world.CreateBody(body_def);
	{
		const auto fixture = body2->CreateFixture(fixtureDef);
		ASSERT_NE(fixture, nullptr);
	}
	ASSERT_EQ(body2->GetPosition().x, body2pos.x);
	ASSERT_EQ(body2->GetPosition().y, body2pos.y);

	ASSERT_EQ(body1->GetAngle(), 0_deg);
	ASSERT_EQ(body2->GetAngle(), 0_deg);
	auto last_angle_1 = body1->GetAngle();
	auto last_angle_2 = body2->GetAngle();

	ASSERT_EQ(world.GetBodies().size(), BodyList::size_type(2));
	ASSERT_EQ(world.GetContacts().size(), ContactList::size_type(0));

	auto position_diff = body1pos - body2pos;
	auto distance = GetLength(position_diff);
	
	auto angle = GetAngle(position_diff);
	ASSERT_FLOAT_EQ(angle.ToRadians(), (0_deg).ToRadians());
	
	auto lastpos1 = body1->GetPosition();
	auto lastpos2 = body2->GetPosition();
	
	const auto velocity_iters = 10u;
	const auto position_iters = 10u;
	
	const auto time_inc = float_t(.01);
	// Solver won't separate more than -world.GetLinearSlop().
	const auto full_separation = half_dim * 2 - world.GetLinearSlop();
	for (auto i = 0; i < 100; ++i)
	{
		world.Step(time_inc, velocity_iters, position_iters);
		
		ASSERT_EQ(world.GetContacts().size(), decltype(world.GetContacts().size())(1));

		auto count = decltype(world.GetContacts().size())(0);
		const auto& contacts = world.GetContacts();
		for (auto&& c: contacts)
		{
			++count;

			const auto fa = c.GetFixtureA();
			const auto fb = c.GetFixtureB();
			const auto body_a = fa->GetBody();
			const auto body_b = fb->GetBody();
			EXPECT_EQ(body_a, body1);
			EXPECT_EQ(body_b, body2);
			
			const auto& manifold = c.GetManifold();
			EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
			EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(2));
		}
		ASSERT_EQ(count, decltype(world.GetContacts().size())(1));

		const auto v1 = body1->GetVelocity();
		EXPECT_EQ(v1.w, 0_deg);
		EXPECT_EQ(v1.v.x, float_t(0));
		EXPECT_EQ(v1.v.y, float_t(0));

		const auto v2 = body2->GetVelocity();
		EXPECT_EQ(v2.w, 0_deg);
		EXPECT_EQ(v2.v.x, float_t(0));
		EXPECT_EQ(v2.v.y, float_t(0));

		EXPECT_FLOAT_EQ(body1->GetAngle().ToRadians(), last_angle_1.ToRadians());
		EXPECT_FLOAT_EQ(body2->GetAngle().ToRadians(), last_angle_2.ToRadians());
		last_angle_1 = body1->GetAngle();
		last_angle_2 = body2->GetAngle();

		const auto new_pos_diff = body1->GetPosition() - body2->GetPosition();
		const auto new_distance = GetLength(new_pos_diff);
		
		if (almost_equal(new_distance, full_separation) || new_distance > full_separation)
		{
			break;
		}
		
		if (new_distance == distance)
		{
			if (cos(angle.ToRadians()) != 0)
			{
				EXPECT_NE(body1->GetPosition().x, lastpos1.x);
				EXPECT_NE(body2->GetPosition().x, lastpos2.x);
			}
			if (sin(angle.ToRadians()) != 0)
			{
				EXPECT_NE(body1->GetPosition().y, lastpos1.y);
				EXPECT_NE(body2->GetPosition().y, lastpos2.y);
			}
			ASSERT_GE(new_distance, float_t(2));
			break;
		}
		
		ASSERT_NE(body1->GetPosition(), lastpos1);
		ASSERT_NE(body2->GetPosition(), lastpos2);
		
		// Body 1 moves right only.
		EXPECT_GT(body1->GetPosition().x, lastpos1.x);
		EXPECT_FLOAT_EQ(body1->GetPosition().y, lastpos1.y);

		// Body 2 moves left only.
		EXPECT_LT(body2->GetPosition().x, lastpos2.x);
		EXPECT_FLOAT_EQ(body2->GetPosition().y, lastpos2.y);

		lastpos1 = body1->GetPosition();
		lastpos2 = body2->GetPosition();
		
		ASSERT_NE(new_pos_diff, position_diff);
		position_diff = new_pos_diff;
		
		ASSERT_NE(new_distance, distance);
		distance = new_distance;
		
		const auto new_angle = GetAngle(new_pos_diff);
		EXPECT_FLOAT_EQ(angle.ToRadians(), new_angle.ToRadians());
		
		angle = new_angle;
	}
}

TEST(World, CollidingDynamicBodies)
{
	const auto radius = float_t(1);
	const auto x = float_t(10); // other test parameters tuned to this value being 10

	auto body_def = BodyDef{};
	body_def.type = BodyType::Dynamic;
	
	MyContactListener listener{
		[](Contact& contact, const Manifold& oldManifold) {},
		[](Contact& contact, const ContactImpulse& impulse, ContactListener::iteration_type solved) {},
		[&](Contact& contact) {},
	};

	const auto gravity = Vec2_zero;
	World world{World::Def{}.UseGravity(gravity)};
	EXPECT_EQ(world.GetGravity(), gravity);
	world.SetContactListener(&listener);
	
	CircleShape shape{radius};
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

static jmp_buf jmp_env;

static void catch_alarm(int sig)
{
	longjmp(jmp_env, 1);
}

#define ASSERT_USECS(fn, usecs) { \
	const auto val = setjmp(jmp_env); \
	if (val == 0) { \
		signal(SIGALRM, catch_alarm); \
		ualarm((usecs), 0); \
		{ fn; }; \
		ualarm(0, 0); \
	} else { \
		GTEST_FATAL_FAILURE_(#usecs " usecs timer tripped for " #fn); \
	} }

TEST(World, SpeedingBulletBallWontTunnel)
{
	World world{World::Def{}.UseGravity(Vec2_zero)};

	MyContactListener listener{
		[](Contact& contact, const Manifold& oldManifold) {},
		[](Contact& contact, const ContactImpulse& impulse, ContactListener::iteration_type solved) {},
		[&](Contact& contact) {},
	};
	world.SetContactListener(&listener);

	ASSERT_EQ(listener.begin_contacts, unsigned{0});

	const auto left_edge_x = float_t(-0.1);
	const auto right_edge_x = float_t(+0.1);

	BodyDef body_def;
	FixtureDef fixtureDef;
	EdgeShape edge_shape;
	CircleShape circle_shape;

	edge_shape.Set(Vec2{0, +10}, Vec2{0, -10});
	fixtureDef.shape = &edge_shape;
	fixtureDef.restitution = float_t(1); // changes where bodies will be after collision
	body_def.type = BodyType::Static;

	body_def.position = Vec2{left_edge_x, 0};
	const auto left_wall_body = world.CreateBody(body_def);
	ASSERT_NE(left_wall_body, nullptr);
	{
		const auto wall_fixture = left_wall_body->CreateFixture(fixtureDef);
		ASSERT_NE(wall_fixture, nullptr);
	}

	body_def.position = Vec2{right_edge_x, 0};
	const auto right_wall_body = world.CreateBody(body_def);
	ASSERT_NE(right_wall_body, nullptr);
	{
		const auto wall_fixture = right_wall_body->CreateFixture(fixtureDef);
		ASSERT_NE(wall_fixture, nullptr);
	}
	
	const auto begin_x = float_t(0);

	body_def.type = BodyType::Dynamic;
	body_def.position = Vec2{begin_x, 0};
	body_def.bullet = false;
	const auto ball_body = world.CreateBody(body_def);
	ASSERT_NE(ball_body, nullptr);
	
	const auto ball_radius = float_t(.01);
	circle_shape.SetRadius(ball_radius);
	fixtureDef.shape = &circle_shape;
	fixtureDef.density = float_t(1);
	fixtureDef.restitution = float_t(1); // changes where bodies will be after collision
	const auto ball_fixture = ball_body->CreateFixture(fixtureDef);
	ASSERT_NE(ball_fixture, nullptr);

	const auto velocity = Vec2{+1, 0};
	ball_body->SetVelocity(Velocity{velocity, 0_deg});

	const auto time_inc = float_t(.01);
	const auto max_velocity = world.GetMaxTranslation() / time_inc;
	world.Step(time_inc);

	ASSERT_EQ(listener.begin_contacts, unsigned{0});

	EXPECT_GT(ball_body->GetPosition().x, begin_x);

	EXPECT_EQ(GetLinearVelocity(*ball_body).x, velocity.x);
	EXPECT_EQ(GetLinearVelocity(*ball_body).y, velocity.y);
	
	const auto max_travel = unsigned{10000};

	auto increments = int{1};
	for (auto laps = int{1}; laps < 100; ++laps)
	{
		// traveling to the right
		listener.begin_contacts = 0;
		for (auto travel_r = unsigned{0}; ; ++travel_r)
		{
			if (travel_r == max_travel)
			{
				std::cout << "begin_contacts=" << listener.begin_contacts << std::endl;
				ASSERT_LT(travel_r, max_travel);
			}

			const auto last_contact_count = listener.begin_contacts;
			ASSERT_USECS(world.Step(time_inc), 5000);

			EXPECT_LT(ball_body->GetPosition().x, right_edge_x - (ball_radius/2));
			EXPECT_GT(ball_body->GetPosition().x, left_edge_x + (ball_radius/2));

			if (ball_body->GetVelocity().v.x >= max_velocity)
			{
				return;
			}

			if (listener.begin_contacts % 2 != 0) // direction switched
			{
				EXPECT_LT(ball_body->GetVelocity().v.x, 0);
				break; // going left now
			}
			else if (listener.begin_contacts > last_contact_count)
			{
				++increments;
				ball_body->SetVelocity(Velocity{Vec2{+increments * velocity.x, ball_body->GetVelocity().v.y}, ball_body->GetVelocity().w});
			}
			else
			{
				EXPECT_FLOAT_EQ(ball_body->GetVelocity().v.x, +increments * velocity.x);				
			}
		}
		
		// traveling to the left
		listener.begin_contacts = 0;
		for (auto travel_l = unsigned{0}; ; ++travel_l)
		{
			if (travel_l == max_travel)
			{
				std::cout << "begin_contacts=" << listener.begin_contacts << std::endl;
				ASSERT_LT(travel_l, max_travel);
			}
			
			const auto last_contact_count = listener.begin_contacts;
			ASSERT_USECS(world.Step(time_inc), 5000);
			
			EXPECT_LT(ball_body->GetPosition().x, right_edge_x - (ball_radius/2));
			EXPECT_GT(ball_body->GetPosition().x, left_edge_x + (ball_radius/2));

			if (ball_body->GetVelocity().v.x <= -max_velocity)
			{
				return;
			}

			if (listener.begin_contacts % 2 != 0) // direction switched
			{
				EXPECT_GT(ball_body->GetVelocity().v.x, 0);
				break; // going right now
			}
			else if (listener.begin_contacts > last_contact_count)
			{
				++increments;
				ball_body->SetVelocity(Velocity{Vec2{-increments * velocity.x, ball_body->GetVelocity().v.y}, ball_body->GetVelocity().w});
			}
			else
			{
				EXPECT_FLOAT_EQ(ball_body->GetVelocity().v.x, -increments * velocity.x);				
			}
		}
		
		++increments;
		ball_body->SetVelocity(Velocity{Vec2{+increments * velocity.x, ball_body->GetVelocity().v.y}, ball_body->GetVelocity().w});
	}
}

TEST(World, MouseJointWontCauseTunnelling)
{
	World world{World::Def{}.UseGravity(Vec2_zero)};
	world.SetContinuousPhysics(true);
	
	const auto half_box_width = float_t(0.2);
	const auto left_edge_x = -half_box_width;
	const auto right_edge_x = +half_box_width;

	const auto half_box_height = float_t(0.2);
	const auto btm_edge_y = -half_box_height;
	const auto top_edge_y = +half_box_height;
	
	BodyDef body_def;
	EdgeShape edge_shape;
	FixtureDef fixtureDef;

	fixtureDef.shape = &edge_shape;
	fixtureDef.friction = float_t(0.4);
	fixtureDef.restitution = float_t(0.94); // changes where bodies will be after collision
	body_def.type = BodyType::Static;
	
	edge_shape.Set(Vec2{0, +half_box_height * 2}, Vec2{0, -half_box_height * 2});

	body_def.position = Vec2{left_edge_x, 0};
	{
		const auto left_wall_body = world.CreateBody(body_def);
		ASSERT_NE(left_wall_body, nullptr);
		{
			const auto wall_fixture = left_wall_body->CreateFixture(fixtureDef);
			ASSERT_NE(wall_fixture, nullptr);
		}
	}
	
	body_def.position = Vec2{right_edge_x, 0};
	{
		const auto right_wall_body = world.CreateBody(body_def);
		ASSERT_NE(right_wall_body, nullptr);
		{
			const auto wall_fixture = right_wall_body->CreateFixture(fixtureDef);
			ASSERT_NE(wall_fixture, nullptr);
		}
	}

	edge_shape.Set(Vec2{-half_box_width * 2, 0}, Vec2{+half_box_width * 2, 0});
	
	body_def.position = Vec2{0, btm_edge_y};
	{
		const auto btm_wall_body = world.CreateBody(body_def);
		ASSERT_NE(btm_wall_body, nullptr);
		{
			const auto wall_fixture = btm_wall_body->CreateFixture(fixtureDef);
			ASSERT_NE(wall_fixture, nullptr);
		}
	}
	
	body_def.position = Vec2{0, top_edge_y};
	{
		const auto top_wall_body = world.CreateBody(body_def);
		ASSERT_NE(top_wall_body, nullptr);
		{
			const auto wall_fixture = top_wall_body->CreateFixture(fixtureDef);
			ASSERT_NE(wall_fixture, nullptr);
		}
	}

	body_def.type = BodyType::Dynamic;
	body_def.position = Vec2_zero;
	body_def.bullet = true;
	
	const auto ball_body = world.CreateBody(body_def);
	ASSERT_NE(ball_body, nullptr);
	ASSERT_EQ(ball_body->GetPosition().x, 0);
	ASSERT_EQ(ball_body->GetPosition().y, 0);
	
	const auto ball_radius = float_t(half_box_width / 4);
	const auto object_shape = PolygonShape{ball_radius, ball_radius};
	fixtureDef.shape = &object_shape;
	fixtureDef.density = float_t(10);
	{
		const auto ball_fixture = ball_body->CreateFixture(fixtureDef);
		ASSERT_NE(ball_fixture, nullptr);
	}

	constexpr unsigned numBodies = 1;
	Vec2 last_opos[numBodies];
	Body *bodies[numBodies];
	for (auto i = decltype(numBodies){0}; i < numBodies; ++i)
	{
		const auto angle = i * 2 * Pi / numBodies;
		const auto x = ball_radius * float_t(2.1) * std::cos(angle);
		const auto y = ball_radius * float_t(2.1) * std::sin(angle);
		body_def.position = Vec2{x, y};
		bodies[i] = world.CreateBody(body_def);
		ASSERT_NE(bodies[i], nullptr);
		ASSERT_EQ(bodies[i]->GetPosition().x, x);
		ASSERT_EQ(bodies[i]->GetPosition().y, y);
		last_opos[i] = bodies[i]->GetPosition();
		{
			const auto fixture = bodies[i]->CreateFixture(fixtureDef);
			ASSERT_NE(fixture, nullptr);
		}
	}

	BodyDef bodyDef;
	const auto spare_body = world.CreateBody(bodyDef);

	const auto mouse_joint = [&]() {
		MouseJointDef mjd;
		mjd.bodyA = spare_body;
		mjd.bodyB = ball_body;
		const auto ball_body_pos = ball_body->GetPosition();
		mjd.target = Vec2{ball_body_pos.x - ball_radius / 2, ball_body_pos.y + ball_radius / 2};
		mjd.maxForce = float_t(1000) * GetMass(*ball_body);
		return static_cast<MouseJoint*>(world.CreateJoint(mjd));
	}();
	ASSERT_NE(mouse_joint, nullptr);

	ball_body->SetAwake();

	auto max_x = float_t(0);
	auto min_x = float_t(0);
	auto max_y = float_t(0);
	auto min_y = float_t(0);

	auto max_velocity = float_t(0);

	//const auto time_inc = float_t(.0043268126901); // numBodies = 6, somewhat dependent on fixture density (10 or less?).
	//const auto time_inc = float_t(.0039224); // numBodies = 4, maybe dependent on fixture density
	//const auto time_inc = float_t(.003746); // numBodies = 2, maybe dependent on fixture density
	//const auto time_inc = float_t(.0036728129); // numBodies = 1, maybe dependent on fixture density
	const auto time_inc = float_t(.00367281295); // numBodies = 1, maybe dependent on fixture density

	auto angle = float_t(0);
	auto anglular_speed = float_t(0.01); // radians / timestep
	const auto anglular_accel = float_t(1.002);
	auto distance = half_box_width / 2;
	auto distance_speed = float_t(0.003); // meters / timestep
	const auto distance_accel = float_t(1.001);

	MyContactListener listener{
		[&](Contact& contact, const Manifold& old_manifold)
		{
			// PreSolve...
			const auto new_manifold = contact.GetManifold();
			ASSERT_NE(old_manifold.GetType(), Manifold::e_circles);
			ASSERT_NE(new_manifold.GetType(), Manifold::e_circles);
#if 0
			if (old_manifold.GetType() != Manifold::e_unset && new_manifold.GetType() != Manifold::e_unset)
			{
				if (old_manifold.GetType() != new_manifold.GetType())
				{
					const auto oln = old_manifold.GetLocalNormal();
					const auto nln = new_manifold.GetLocalNormal();
					if (Dot(oln, nln) <= 0)
					{
						std::cout << "PreSolve normal changed";
						std::cout << std::endl;
					}
				}
			}
#endif
		},
		[&](Contact& contact, const ContactImpulse& impulse, ContactListener::iteration_type solved)
		{
			const auto fA = contact.GetFixtureA();
			const auto fB = contact.GetFixtureB();

			ASSERT_NE(fA, nullptr);
			ASSERT_NE(fB, nullptr);

			const auto body_a = fA->GetBody();
			const auto body_b = fB->GetBody();

			ASSERT_NE(body_a, nullptr);
			ASSERT_NE(body_b, nullptr);

			auto fail_count = unsigned{0};
			for (auto&& body: {body_a, body_b})
			{
				if (!body->IsSpeedable())
				{
					continue;
				}
				const auto bpos = body->GetPosition();
				const auto lt = Vec2{right_edge_x, top_edge_y} - bpos;
				const auto gt = bpos - Vec2{left_edge_x, btm_edge_y};
				
				EXPECT_LT(body->GetPosition().x, right_edge_x);
				EXPECT_LT(body->GetPosition().y, top_edge_y);

				EXPECT_GT(body->GetPosition().x, left_edge_x);
				EXPECT_GT(body->GetPosition().y, btm_edge_y);
				
				if (lt.x <= 0 || lt.y <= 0 || gt.x <= 0 || gt.y <= 0)
				{
					++fail_count;
				}
			}
			if (fail_count > 0)
			{
				std::cout << " angl=" << angle;
				std::cout << " ctoi=" << contact.GetToiCount();
				std::cout << " solv=" << solved;
				std::cout << " targ=(" << distance * std::cos(angle) << "," << distance * std::sin(angle) << ")";
				std::cout << " maxv=" << max_velocity;
				std::cout << " rang=(" << min_x << "," << min_y << ")-(" << max_x << "," << max_y << ")";
				std::cout << " bpos=(" << ball_body->GetPosition().x << "," << ball_body->GetPosition().y << ")";
				std::cout << std::endl;
				for (auto i = decltype(impulse.GetCount()){0}; i < impulse.GetCount(); ++i)
				{
					std::cout << " i#" << (0 + i) << "={n" << impulse.GetEntryNormal(i) << ",t" << impulse.GetEntryTanget(i) << "}";
				}
				std::cout << std::endl;

				std::cout << " bodyA=(" << body_a->GetPosition().x << "," << body_a->GetPosition().y << ")";
				if (body_a == ball_body) std::cout << " ball";
				if (!body_a->IsSpeedable()) std::cout << " wall";
				std::cout << " " << body_a;
				std::cout << std::endl;
				std::cout << " bodyB=(" << body_b->GetPosition().x << "," << body_b->GetPosition().y << ")";
				if (body_b == ball_body) std::cout << " ball";
				if (!body_b->IsSpeedable()) std::cout << " wall";
				std::cout << " " << body_b;
				std::cout << std::endl;

				//GTEST_FATAL_FAILURE_("");				
			}
		},
		[=](Contact& contact) {
			const auto fA = contact.GetFixtureA();
			const auto fB = contact.GetFixtureB();
			const auto body_a = fA->GetBody();
			const auto body_b = fB->GetBody();

			auto escaped = false;
			for (auto&& body: {body_a, body_b})
			{
				if (!body->IsSpeedable())
				{
					continue;
				}

				if (body->GetPosition().x >= right_edge_x)
				{
					escaped = true;
				}
				if (body->GetPosition().y >= top_edge_y)
				{
					escaped = true;
				}
				if (body->GetPosition().x <= left_edge_x)
				{
					escaped = true;
				}
				if (body->GetPosition().y <= btm_edge_y)
				{
					escaped = true;					
				}
			}
			if (escaped && !contact.IsTouching())
			{
				std::cout << "Escaped at EndContact[" << &contact << "]:";
				std::cout << " toiSteps=" << static_cast<unsigned>(contact.GetToiCount());
				std::cout << " toiCalls=" << static_cast<unsigned>(contact.GetToiCalls());
				std::cout << " itersTot=" << static_cast<unsigned>(contact.GetToiItersTotal());
				std::cout << " itersMax=" << static_cast<unsigned>(contact.GetToiItersMax());
				std::cout << " distSum=" << static_cast<unsigned>(contact.GetDistItersTotal());
				std::cout << " distMax=" << static_cast<unsigned>(contact.GetDistItersMax());
				std::cout << " rootSum=" << static_cast<unsigned>(contact.GetRootItersTotal());
				std::cout << " rootMax=" << static_cast<unsigned>(contact.GetRootItersMax());
				std::cout << " toiValid=" << contact.HasValidToi();
				std::cout << " a[" << body_a << "]@(" << body_a->GetPosition().x << "," << body_a->GetPosition().y << ")";
				std::cout << " b[" << body_b << "]@(" << body_b->GetPosition().x << "," << body_b->GetPosition().y << ")";
				std::cout << std::endl;
				//exit(1);
			}
		},
	};
	world.SetContactListener(&listener);
	ASSERT_EQ(listener.begin_contacts, unsigned{0});

	for (auto outer = unsigned{0}; outer < 1000; ++outer)
	{
		auto last_pos = ball_body->GetPosition();
		for (auto loops = unsigned{0};; ++loops)
		{
			mouse_joint->SetTarget(Vec2{distance * std::cos(angle), distance * std::sin(angle)});
			angle += anglular_speed;
			distance += distance_speed;

			ASSERT_USECS(world.Step(time_inc, 8, 3), 100000);
			
			ASSERT_LT(ball_body->GetPosition().x, right_edge_x);
			ASSERT_LT(ball_body->GetPosition().y, top_edge_y);
			ASSERT_GT(ball_body->GetPosition().x, left_edge_x);
			ASSERT_GT(ball_body->GetPosition().y, btm_edge_y);
			for (auto i = decltype(numBodies){0}; i < numBodies; ++i)
			{
				ASSERT_LT(bodies[i]->GetPosition().x, right_edge_x);
				ASSERT_LT(bodies[i]->GetPosition().y, top_edge_y);			
				ASSERT_GT(bodies[i]->GetPosition().x, left_edge_x);
				ASSERT_GT(bodies[i]->GetPosition().y, btm_edge_y);
			}

			max_x = Max(ball_body->GetPosition().x, max_x);
			min_x = Min(ball_body->GetPosition().x, min_x);

			max_y = Max(ball_body->GetPosition().y, max_y);
			min_y = Min(ball_body->GetPosition().y, min_y);

			max_velocity = Max(GetLength(ball_body->GetVelocity().v), max_velocity);

			if (loops > 50)
			{
				if (mouse_joint->GetTarget().x < 0)
				{
					if (ball_body->GetPosition().x >= last_pos.x)
						break;					
				}
				else
				{
					if (ball_body->GetPosition().x <= last_pos.x)
						break;
				}
				if (mouse_joint->GetTarget().y < 0)
				{
					if (ball_body->GetPosition().y >= last_pos.y)
						break;
				}
				else
				{
					if (ball_body->GetPosition().y <= last_pos.y)
						break;
				}
			}
			last_pos = ball_body->GetPosition();
		}
		anglular_speed *= anglular_accel;
		distance_speed *= distance_accel;

		ASSERT_NE(ball_body->GetPosition(), Vec2_zero);
#if 0
		if (outer > 100)
		{
			for (auto i = decltype(numBodies){0}; i < numBodies; ++i)
			{
				// a sanity check to ensure the other bodies are getting moved
				EXPECT_NE(last_opos[i], bodies[i]->GetPosition());
				last_opos[i] = bodies[i]->GetPosition();
			}
		}
#endif
	}
	std::cout << "angle=" << angle;
	std::cout << " target=(" << distance * std::cos(angle) << "," << distance * std::sin(angle) << ")";
	std::cout << " maxvel=" << max_velocity;
	std::cout << " range=(" << min_x << "," << min_y << ")-(" << max_x << "," << max_y << ")";
	std::cout << std::endl;
}

static void smaller_still_conserves_momentum(bool bullet, float_t multiplier, float_t time_inc)
{
	const auto radius = float_t(1);
	const auto start_distance = float_t(10);
	
	auto scale = float_t(1);
	for (;;)
	{
		const auto gravity = Vec2_zero;
		World world{World::Def{}.UseGravity(gravity)};
		ASSERT_EQ(world.GetGravity().x, 0);
		ASSERT_EQ(world.GetGravity().y, 0);

		auto maxNormalImpulse = float_t(0);
		auto maxTangentImpulse = float_t(0);
		auto maxPoints = 0u;
		auto numSteps = 0u;
		auto failed = false;
		auto preB1 = Vec2_zero;
		auto preB2 = Vec2_zero;
		
		MyContactListener listener{
			[&](Contact& contact, const Manifold& old_manifold)
			{
				const auto fA = contact.GetFixtureA();
				const auto fB = contact.GetFixtureB();
				const auto bA = fA->GetBody();
				const auto bB = fB->GetBody();
				preB1 = bA->GetPosition();
				preB2 = bB->GetPosition();
			},
			[&](Contact& contact, const ContactImpulse& impulse, ContactListener::iteration_type solved)
			{
				{
					const auto count = impulse.GetCount();
					maxPoints = Max(maxPoints, decltype(maxPoints){count});
					for (auto i = decltype(count){0}; i < count; ++i)
					{
						maxNormalImpulse = Max(maxNormalImpulse, impulse.GetEntryNormal(i));
						maxTangentImpulse = Max(maxTangentImpulse, impulse.GetEntryTanget(i));
					}
				}
				if (maxNormalImpulse == 0 && maxTangentImpulse == 0)
				{
					failed = true;
					const auto& manifold = contact.GetManifold();
					std::cout << " solved=" << unsigned(solved);
					std::cout << " numstp=" << numSteps;
					std::cout << " type=" << unsigned(manifold.GetType());
					std::cout << " lp.x=" << manifold.GetLocalPoint().x;
					std::cout << " lp.y=" << manifold.GetLocalPoint().y;
					const auto count = manifold.GetPointCount();
					std::cout << " points=" << unsigned(count);
					for (auto i = decltype(count){0}; i < count; ++i)
					{
						std::cout << " ni[" << unsigned(i) << "]=" << manifold.GetPoint(i).normalImpulse;
						std::cout << " ti[" << unsigned(i) << "]=" << manifold.GetPoint(i).tangentImpulse;
						std::cout << " lp[" << unsigned(i) << "].x=" << manifold.GetPoint(i).localPoint.x;
						std::cout << " lp[" << unsigned(i) << "].y=" << manifold.GetPoint(i).localPoint.y;
					}
					std::cout << std::endl;
				}
			},
			[=](Contact& contact) {
			}
		};
		world.SetContactListener(&listener);

		const auto shape = CircleShape{scale * radius};
		ASSERT_EQ(shape.GetRadius(), scale * radius);
		
		auto fixture_def = FixtureDef{&shape, 1};
		fixture_def.friction = 0;
		fixture_def.restitution = 1;
		
		auto body_def = BodyDef{};
		body_def.type = BodyType::Dynamic;
		body_def.bullet = bullet;
		
		body_def.position = Vec2{+(scale * start_distance), 0};
		body_def.linearVelocity = Vec2{-start_distance, 0};
		const auto body_1 = world.CreateBody(body_def);
		ASSERT_EQ(body_1->GetPosition().x, body_def.position.x);
		ASSERT_EQ(body_1->GetPosition().y, body_def.position.y);
		ASSERT_EQ(GetLinearVelocity(*body_1).x, body_def.linearVelocity.x);
		ASSERT_EQ(GetLinearVelocity(*body_1).y, body_def.linearVelocity.y);
		body_1->CreateFixture(fixture_def);
		
		body_def.position = Vec2{-(scale * start_distance), 0};
		body_def.linearVelocity = Vec2{+start_distance, 0};
		const auto body_2 = world.CreateBody(body_def);
		ASSERT_EQ(body_2->GetPosition().x, body_def.position.x);
		ASSERT_EQ(body_2->GetPosition().y, body_def.position.y);
		ASSERT_EQ(GetLinearVelocity(*body_2).x, body_def.linearVelocity.x);
		ASSERT_EQ(GetLinearVelocity(*body_2).y, body_def.linearVelocity.y);
		body_2->CreateFixture(fixture_def);
		
		for (;;)
		{
			const auto relative_velocity = GetLinearVelocity(*body_1) - GetLinearVelocity(*body_2);
			if (relative_velocity.x >= 0)
			{
				EXPECT_FLOAT_EQ(relative_velocity.x, Abs(body_def.linearVelocity.x) * +2);
				break;
			}
			if (failed)
			{
				std::cout << " scale=" << scale;
				std::cout << " dist0=" << (scale * start_distance * 2);
				std::cout << " bcont=" << listener.begin_contacts;
				std::cout << " econt=" << listener.end_contacts;
				std::cout << " pre-#=" << listener.pre_solves;
				std::cout << " post#=" << listener.post_solves;
				std::cout << " normi=" << maxNormalImpulse;
				std::cout << " tangi=" << maxTangentImpulse;
				std::cout << " n-pts=" << maxPoints;
				std::cout << std::endl;
				std::cout << " pre1.x=" << preB1.x;
				std::cout << " pre2.x=" << preB2.x;
				std::cout << " pos1.x=" << body_1->GetPosition().x;
				std::cout << " pos2.x=" << body_2->GetPosition().x;
				std::cout << " preDel=" << (preB1.x - preB2.x);
				std::cout << " posDel=" << (body_1->GetPosition().x - body_2->GetPosition().x);
				std::cout << " travel=" << (body_1->GetPosition().x - preB1.x);
				std::cout << std::endl;
				ASSERT_FALSE(failed);
			}
			
			EXPECT_FLOAT_EQ(relative_velocity.x, Abs(body_def.linearVelocity.x) * -2);
			world.Step(time_inc);
			++numSteps;
		}
		
		scale *= multiplier;
	}
}

TEST(World, SmallerStillConservesMomemtum)
{
	// smaller_still_conserves_momentum(false, float_t(0.999), float_t(0.01));
	// fails around scale=0.0899796 dist0=1.79959
	// goin to smaller time increment fails nearly same point.
	smaller_still_conserves_momentum(false, float_t(0.999), float_t(0.01));
}

TEST(World, SmallerBulletStillConservesMomemtum)
{
	// smaller_still_conserves_momentum(true, float_t(0.999), float_t(0.01))
	// fails around scale=4.99832e-05 dist0=0.000999664
	// goin to smaller time increment fails nearly same point.
// smaller_still_conserves_momentum(true, float_t(0.999), float_t(0.01));
}
