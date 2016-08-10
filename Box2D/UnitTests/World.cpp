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
#include <Box2D/Collision/Shapes/PolygonShape.h>
#include <Box2D/Collision/Shapes/EdgeShape.h>
#include <Box2D/Dynamics/Joints/MouseJoint.h>

#include <unistd.h>
#include <setjmp.h>
#include <signal.h>

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

	const auto body = world.Create(BodyDef{});
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

	const auto body1 = world.Create(BodyDef{});
	const auto body2 = world.Create(BodyDef{});
	EXPECT_NE(body1, nullptr);
	EXPECT_NE(body2, nullptr);
	EXPECT_EQ(GetBodyCount(world), body_count_t(2));
	EXPECT_EQ(GetJointCount(world), joint_count_t(0));
	EXPECT_TRUE(world.GetJoints().empty());
	EXPECT_EQ(world.GetJoints().begin(), world.GetJoints().end());
	
	const auto anchorA = Vec2{float_t(+0.4), float_t(-1.2)};
	const auto anchorB = Vec2{float_t(-2.3), float_t(+0.7)};
	const auto joint = world.Create(DistanceJointDef{body1, body2, anchorA, anchorB});
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

	const auto body = world.Create(body_def);
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

TEST(World, MaxBodies)
{
	World world;
	for (auto i = 0; i < MaxBodies; ++i)
	{
		const auto body = world.Create(BodyDef{});
		ASSERT_NE(body, nullptr);
	}
	{
		const auto body = world.Create(BodyDef{});
		EXPECT_EQ(body, nullptr);		
	}
}

class MyContactListener: public ContactListener
{
public:
	using PreSolver = std::function<void(Contact&, const Manifold&)>;
	using PostSolver = std::function<void(Contact&, const ContactImpulse&, ContactListener::iteration_type)>;

	MyContactListener(PreSolver&& pre, PostSolver&& post): presolver(pre), postsolver(post) {}

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
		contacting = false;
		touching = contact.IsTouching();

		body_a[1] = contact.GetFixtureA()->GetBody()->GetPosition();
		body_b[1] = contact.GetFixtureB()->GetBody()->GetPosition();
	}
	
	void PreSolve(Contact& contact, const Manifold& oldManifold) override
	{
		presolver(contact, oldManifold);
	}
	
	void PostSolve(Contact& contact, const ContactImpulse& impulse, ContactListener::iteration_type solved) override
	{
		postsolver(contact, impulse, solved);
	}

	unsigned begin_contacts = 0;
	bool contacting = false;
	bool touching = false;
	Vec2 body_a[2] = {Vec2_zero, Vec2_zero};
	Vec2 body_b[2] = {Vec2_zero, Vec2_zero};
	PreSolver presolver;
	PostSolver postsolver;
};

TEST(World, CollidingDynamicBodies)
{
	const auto x = float_t(10); // other test parameters tuned to this value being 10

	auto body_def = BodyDef{};
	body_def.type = BodyType::Dynamic;
	
	MyContactListener listener{
		[](Contact& contact, const Manifold& oldManifold) {},
		[](Contact& contact, const ContactImpulse& impulse, ContactListener::iteration_type solved) {}
	};

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
	const auto body_a = world.Create(body_def);
	ASSERT_NE(body_a, nullptr);
	EXPECT_EQ(body_a->GetType(), BodyType::Dynamic);
	EXPECT_TRUE(body_a->IsSpeedable());
	EXPECT_TRUE(body_a->IsAccelerable());

	const auto fixture1 = body_a->CreateFixture(fixtureDef);
	ASSERT_NE(fixture1, nullptr);

	body_def.position = Vec2{+(x + 1), 0};
	body_def.linearVelocity = Vec2{-x, 0};
	const auto body_b = world.Create(body_def);
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
	World world{Vec2_zero};

	MyContactListener listener{
		[](Contact& contact, const Manifold& oldManifold) {},
		[](Contact& contact, const ContactImpulse& impulse, ContactListener::iteration_type solved) {}
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
	const auto left_wall_body = world.Create(body_def);
	ASSERT_NE(left_wall_body, nullptr);
	{
		const auto wall_fixture = left_wall_body->CreateFixture(fixtureDef);
		ASSERT_NE(wall_fixture, nullptr);
	}

	body_def.position = Vec2{right_edge_x, 0};
	const auto right_wall_body = world.Create(body_def);
	ASSERT_NE(right_wall_body, nullptr);
	{
		const auto wall_fixture = right_wall_body->CreateFixture(fixtureDef);
		ASSERT_NE(wall_fixture, nullptr);
	}
	
	const auto begin_x = float_t(0);

	body_def.type = BodyType::Dynamic;
	body_def.position = Vec2{begin_x, 0};
	body_def.bullet = true;
	const auto ball_body = world.Create(body_def);
	ASSERT_NE(ball_body, nullptr);
	
	const auto ball_radius = float_t(.01);
	circle_shape.SetRadius(ball_radius);
	fixtureDef.shape = &circle_shape;
	fixtureDef.density = float_t(1);
	fixtureDef.restitution = float_t(1); // changes where bodies will be after collision
	const auto ball_fixture = ball_body->CreateFixture(fixtureDef);
	ASSERT_NE(ball_fixture, nullptr);

	const auto velocity = Vec2{+1, 0};
	ball_body->SetVelocity(Velocity{velocity, 0});

	const auto time_inc = float_t(.01);
	const auto max_velocity = MaxTranslation / time_inc;
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
			ASSERT_USECS(world.Step(time_inc), 1000);

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
			ASSERT_USECS(world.Step(time_inc), 1000);
			
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

TEST(World, MouseJointWontTunnelBulletBall)
{
	World world{Vec2_zero};
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
		const auto left_wall_body = world.Create(body_def);
		ASSERT_NE(left_wall_body, nullptr);
		{
			const auto wall_fixture = left_wall_body->CreateFixture(fixtureDef);
			ASSERT_NE(wall_fixture, nullptr);
		}
	}
	
	body_def.position = Vec2{right_edge_x, 0};
	{
		const auto right_wall_body = world.Create(body_def);
		ASSERT_NE(right_wall_body, nullptr);
		{
			const auto wall_fixture = right_wall_body->CreateFixture(fixtureDef);
			ASSERT_NE(wall_fixture, nullptr);
		}
	}

	edge_shape.Set(Vec2{-half_box_width * 2, 0}, Vec2{+half_box_width * 2, 0});
	
	body_def.position = Vec2{0, btm_edge_y};
	{
		const auto btm_wall_body = world.Create(body_def);
		ASSERT_NE(btm_wall_body, nullptr);
		{
			const auto wall_fixture = btm_wall_body->CreateFixture(fixtureDef);
			ASSERT_NE(wall_fixture, nullptr);
		}
	}
	
	body_def.position = Vec2{0, top_edge_y};
	{
		const auto top_wall_body = world.Create(body_def);
		ASSERT_NE(top_wall_body, nullptr);
		{
			const auto wall_fixture = top_wall_body->CreateFixture(fixtureDef);
			ASSERT_NE(wall_fixture, nullptr);
		}
	}

	body_def.type = BodyType::Dynamic;
	body_def.position = Vec2_zero;
	body_def.bullet = true;
	
	const auto ball_body = world.Create(body_def);
	ASSERT_NE(ball_body, nullptr);
	ASSERT_EQ(ball_body->GetPosition().x, 0);
	ASSERT_EQ(ball_body->GetPosition().y, 0);
	
	const auto ball_radius = float_t(half_box_width / 4);
	PolygonShape object_shape;
	object_shape.SetAsBox(ball_radius, ball_radius);
	fixtureDef.shape = &object_shape;
	fixtureDef.density = float_t(10);
	{
		const auto ball_fixture = ball_body->CreateFixture(fixtureDef);
		ASSERT_NE(ball_fixture, nullptr);
	}

	constexpr unsigned numBodies = 6;
	Vec2 last_opos[numBodies];
	Body *bodies[numBodies];
	for (auto i = decltype(numBodies){0}; i < numBodies; ++i)
	{
		const auto angle = i * 2 * Pi / numBodies;
		const auto x = ball_radius * float_t(2.1) * std::cos(angle);
		const auto y = ball_radius * float_t(2.1) * std::sin(angle);
		body_def.position = Vec2{x, y};
		bodies[i] = world.Create(body_def);
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
	const auto spare_body = world.Create(bodyDef);

	const auto mouse_joint = [&]() {
		MouseJointDef mjd;
		mjd.bodyA = spare_body;
		mjd.bodyB = ball_body;
		const auto ball_body_pos = ball_body->GetPosition();
		mjd.target = Vec2{ball_body_pos.x - ball_radius / 2, ball_body_pos.y + ball_radius / 2};
		mjd.maxForce = float_t(1000) * GetMass(*ball_body);
		return static_cast<MouseJoint*>(world.Create(mjd));
	}();
	ASSERT_NE(mouse_joint, nullptr);

	ball_body->SetAwake();

	auto max_x = float_t(0);
	auto min_x = float_t(0);
	auto max_y = float_t(0);
	auto min_y = float_t(0);

	auto max_velocity = float_t(0);

	const auto time_inc = float_t(.01);

	auto angle = float_t(0);
	auto anglular_speed = float_t(0.01); // radians / timestep
	const auto anglular_accel = float_t(1.002);
	auto distance = half_box_width / 2;
	auto distance_speed = float_t(0.003); // meters / timestep
	const auto distance_accel = float_t(1.001);

	MyContactListener listener{
		[&](Contact& contact, const Manifold& old_manifold)
		{
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
				std::cout << std::endl;
				std::cout << " bodyB=(" << body_b->GetPosition().x << "," << body_b->GetPosition().y << ")";
				if (body_b == ball_body) std::cout << " ball";
				if (!body_b->IsSpeedable()) std::cout << " wall";
				std::cout << std::endl;

				//GTEST_FATAL_FAILURE_("");				
			}
		}
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

			max_velocity = Max(Length(ball_body->GetVelocity().v), max_velocity);

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
		
		if (outer > 100)
		{
			for (auto i = decltype(numBodies){0}; i < numBodies; ++i)
			{
				// a sanity check to ensure the other bodies are getting moved
				EXPECT_NE(last_opos[i], bodies[i]->GetPosition());
				last_opos[i] = bodies[i]->GetPosition();
			}
		}
	}
	std::cout << "angle=" << angle;
	std::cout << " target=(" << distance * std::cos(angle) << "," << distance * std::sin(angle) << ")";
	std::cout << " maxvel=" << max_velocity;
	std::cout << " range=(" << min_x << "," << min_y << ")-(" << max_x << "," << max_y << ")";
	std::cout << std::endl;
}