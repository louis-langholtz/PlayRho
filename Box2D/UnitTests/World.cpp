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
#include <Box2D/Dynamics/World.hpp>
#include <Box2D/Dynamics/StepConf.hpp>
#include <Box2D/Dynamics/Body.hpp>
#include <Box2D/Dynamics/Fixture.hpp>
#include <Box2D/Dynamics/Contacts/Contact.hpp>
#include <Box2D/Dynamics/Joints/DistanceJoint.hpp>
#include <Box2D/Collision/Shapes/CircleShape.hpp>
#include <Box2D/Collision/Shapes/PolygonShape.hpp>
#include <Box2D/Collision/Shapes/EdgeShape.hpp>
#include <Box2D/Dynamics/Joints/MouseJoint.hpp>
#include <Box2D/Dynamics/Joints/RopeJoint.hpp>
#include <Box2D/Common/Angle.hpp>
#include <chrono>

using namespace box2d;

TEST(World, ByteSize)
{
	switch (sizeof(RealNum))
	{
		case  4:
		{
			// Size is OS dependent.
			// Seems linux containers are bigger in size...
#ifdef __APPLE__
			EXPECT_EQ(sizeof(World), size_t(352));
#endif
#ifdef __linux__
			EXPECT_EQ(sizeof(World), size_t(376));
#endif
			break;
		}
		case  8: EXPECT_EQ(sizeof(World), size_t(352)); break;
		case 16: EXPECT_EQ(sizeof(World), size_t(400)); break;
		default: FAIL(); break;
	}
}

TEST(World, Def)
{
	const auto worldDef = World::Def{};
	const auto defaultDef = World::GetDefaultDef();
	
	EXPECT_EQ(defaultDef.gravity, worldDef.gravity);
	EXPECT_EQ(defaultDef.maxVertexRadius, worldDef.maxVertexRadius);
	EXPECT_EQ(defaultDef.minVertexRadius, worldDef.minVertexRadius);
	const auto stepConf = StepConf{};

	const auto v = RealNum(1);
	const auto n = std::nextafter(v, RealNum(0));
	const auto time_inc = v - n;
	ASSERT_GT(time_inc, RealNum(0));
	ASSERT_LT(time_inc, RealNum(1));
	const auto max_inc = time_inc * stepConf.maxTranslation;
	EXPECT_GT(max_inc, RealNum(0));
}

TEST(World, DefaultInit)
{
	World world;

	EXPECT_EQ(GetBodyCount(world), body_count_t(0));
	EXPECT_EQ(world.GetProxyCount(), World::proxy_size_type(0));
	EXPECT_EQ(GetJointCount(world), joint_count_t(0));
	EXPECT_EQ(GetContactCount(world), contact_count_t(0));
	EXPECT_EQ(world.GetTreeHeight(), World::proxy_size_type(0));
	EXPECT_EQ(world.GetTreeQuality(), RealNum(0));

	EXPECT_EQ(world.GetGravity(), EarthlyGravity);
		
	EXPECT_TRUE(world.GetBodies().empty());
	EXPECT_EQ(world.GetBodies().size(), body_count_t(0));
	EXPECT_EQ(world.GetBodies().begin(), world.GetBodies().end());

	EXPECT_TRUE(world.GetContacts().empty());
	EXPECT_EQ(world.GetContacts().size(), contact_count_t(0));
	EXPECT_EQ(world.GetContacts().begin(), world.GetContacts().end());
	
	EXPECT_TRUE(world.GetJoints().empty());
	EXPECT_EQ(world.GetJoints().size(), joint_count_t(0));
	EXPECT_EQ(world.GetJoints().begin(), world.GetJoints().end());
	
	EXPECT_FALSE(world.GetSubStepping());
	EXPECT_FALSE(world.IsLocked());
}

TEST(World, Init)
{
	const auto gravity = Vec2{RealNum(-4.2), RealNum(3.4)};
	World world{World::Def{}.UseGravity(gravity)};
	EXPECT_EQ(world.GetGravity(), gravity);
	EXPECT_FALSE(world.IsLocked());
}

TEST(World, SetGravity)
{
	const auto gravity = Vec2{RealNum(-4.2), RealNum(3.4)};
	World world;
	EXPECT_NE(world.GetGravity(), gravity);
	world.SetGravity(gravity);
	EXPECT_EQ(world.GetGravity(), gravity);	
	world.SetGravity(-gravity);
	EXPECT_NE(world.GetGravity(), gravity);
}

TEST(World, CreateAndDestroyBody)
{
	World world;
	EXPECT_EQ(GetBodyCount(world), body_count_t(0));

	const auto body = world.CreateBody();
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
	EXPECT_EQ(body, first);

	world.Destroy(body);
	EXPECT_EQ(GetBodyCount(world), body_count_t(0));
	EXPECT_TRUE(world.GetBodies().empty());
	EXPECT_EQ(world.GetBodies().size(), body_count_t(0));
	EXPECT_EQ(world.GetBodies().begin(), world.GetBodies().end());
}

TEST(World, DynamicEdgeBodyHasCorrectMass)
{
	World world;
	
	auto bodyDef = BodyDef{};
	bodyDef.type = BodyType::Dynamic;
	const auto body = world.CreateBody(bodyDef);
	ASSERT_EQ(body->GetType(), BodyType::Dynamic);
	
	const auto v1 = Vec2{-1, 0};
	const auto v2 = Vec2{+1, 0};
	auto conf = EdgeShape::Conf{};
	conf.v0 = GetInvalid<Vec2>();
	conf.v3 = GetInvalid<Vec2>();
	conf.vertexRadius = 1;
	const auto shape = std::make_shared<EdgeShape>(v1, v2, conf);
	shape->SetDensity(RealNum{1} * KilogramPerSquareMeter);
	ASSERT_EQ(shape->GetVertexRadius(), RealNum(1));
	ASSERT_EQ(shape->GetType(), Shape::e_edge);

	const auto fixture = body->CreateFixture(shape);
	ASSERT_NE(fixture, nullptr);
	ASSERT_EQ(fixture->GetDensity(), RealNum{1} * KilogramPerSquareMeter);

	const auto circleMass = RealNum{fixture->GetDensity() / KilogramPerSquareMeter} * Pi * Square(shape->GetVertexRadius());
	const auto rectMass = RealNum{fixture->GetDensity() / KilogramPerSquareMeter} * shape->GetVertexRadius() * 2 * GetLength(v2 - v1);
	const auto totalMass = circleMass + rectMass;
	
	EXPECT_EQ(body->GetType(), BodyType::Dynamic);
	EXPECT_EQ(body->GetInvMass(), RealNum(1) / totalMass);

	ASSERT_NE(fixture->GetShape(), nullptr);
	EXPECT_EQ(fixture->GetShape()->GetType(), shape->GetType());
}

TEST(World, CreateAndDestroyJoint)
{
	World world;

	const auto body1 = world.CreateBody();
	const auto body2 = world.CreateBody();
	EXPECT_NE(body1, nullptr);
	EXPECT_NE(body2, nullptr);
	EXPECT_EQ(GetBodyCount(world), body_count_t(2));
	EXPECT_EQ(GetJointCount(world), joint_count_t(0));
	EXPECT_TRUE(world.GetJoints().empty());
	EXPECT_EQ(world.GetJoints().begin(), world.GetJoints().end());
	
	const auto anchorA = Vec2{RealNum(+0.4), RealNum(-1.2)};
	const auto anchorB = Vec2{RealNum(-2.3), RealNum(+0.7)};
	const auto joint = world.CreateJoint(DistanceJointDef{body1, body2, anchorA, anchorB});
	EXPECT_EQ(GetJointCount(world), joint_count_t(1));
	EXPECT_FALSE(world.GetJoints().empty());
	EXPECT_NE(world.GetJoints().begin(), world.GetJoints().end());
	const auto first = *world.GetJoints().begin();
	EXPECT_EQ(joint, first);
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
		const auto body = world.CreateBody();
		ASSERT_NE(body, nullptr);
	}
	{
		const auto body = world.CreateBody();
		EXPECT_EQ(body, nullptr);		
	}
}

TEST(World, MaxJoints)
{
	World world;
	
	const auto body1 = world.CreateBody();
	ASSERT_NE(body1, nullptr);
	const auto body2 = world.CreateBody();
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
	const auto gravity = Vec2{0, RealNum(-9.8)};
	
	World world{World::Def{}.UseGravity(gravity)};
	
	BodyDef def;
	def.position = Vec2{RealNum(31.9), RealNum(-19.24)};
	def.type = BodyType::Dynamic;
	
	const auto body = world.CreateBody(def);
	ASSERT_NE(body, nullptr);
	EXPECT_EQ(body->GetLocation().x, def.position.x);
	EXPECT_EQ(body->GetLocation().y, def.position.y);
	EXPECT_EQ(GetLinearVelocity(*body).x, RealNum(0));
	EXPECT_EQ(GetLinearVelocity(*body).y, RealNum(0));
	EXPECT_EQ(body->GetLinearAcceleration().x, 0);
	EXPECT_EQ(body->GetLinearAcceleration().y, gravity.y);
	
	const auto time_inc = Time{Second * RealNum{0}};
	
	auto pos = body->GetLocation();
	auto vel = GetLinearVelocity(*body);
	for (auto i = 0; i < 100; ++i)
	{
		Step(world, time_inc);
		
		EXPECT_EQ(body->GetLinearAcceleration().y, gravity.y);
		
		EXPECT_EQ(body->GetLocation().x, def.position.x);
		EXPECT_EQ(body->GetLocation().y, pos.y);
		pos = body->GetLocation();
		
		EXPECT_EQ(GetLinearVelocity(*body).x, RealNum(0));
		EXPECT_TRUE(almost_equal(GetLinearVelocity(*body).y, vel.y));
		vel = GetLinearVelocity(*body);
	}
}

TEST(World, GravitationalBodyMovement)
{
	auto p0 = Vec2{0, 1};

	auto body_def = BodyDef{};
	body_def.type = BodyType::Dynamic;
	body_def.position = p0;

	const auto a = RealNum(-10);
	const auto gravity = Vec2{0, a};
	const auto t = RealNum(.01);
	
	World world{World::Def{}.UseGravity(gravity)};

	const auto body = world.CreateBody(body_def);
	ASSERT_NE(body, nullptr);
	EXPECT_FALSE(body->IsImpenetrable());
	EXPECT_EQ(body->GetType(), BodyType::Dynamic);
	EXPECT_EQ(GetLinearVelocity(*body).x, 0);
	EXPECT_EQ(GetLinearVelocity(*body).y, 0);
	EXPECT_EQ(body->GetLocation().x, p0.x);
	EXPECT_EQ(body->GetLocation().y, p0.y);

	Step(world, Time{Second * t});
	EXPECT_EQ(GetLinearVelocity(*body).x, 0);
	EXPECT_EQ(GetLinearVelocity(*body).y, a * (t * 1));
	EXPECT_EQ(body->GetLocation().x, p0.x);
	EXPECT_EQ(body->GetLocation().y, p0.y + (GetLinearVelocity(*body).y * t));

	p0 = body->GetLocation();
	Step(world, Time{Second * t});
	EXPECT_EQ(GetLinearVelocity(*body).x, 0);
	EXPECT_EQ(GetLinearVelocity(*body).y, a * (t * 2));
	EXPECT_EQ(body->GetLocation().x, p0.x);
	EXPECT_EQ(body->GetLocation().y, p0.y + (GetLinearVelocity(*body).y * t));
	
	p0 = body->GetLocation();
	Step(world, Time{Second * t});
	EXPECT_EQ(GetLinearVelocity(*body).x, 0);
	EXPECT_NEAR(double(GetLinearVelocity(*body).y), double(a * (t * 3)), 0.00001);
	EXPECT_EQ(body->GetLocation().x, p0.x);
	EXPECT_EQ(body->GetLocation().y, p0.y + (GetLinearVelocity(*body).y * t));
}

TEST(World, BodyAccelPerSpecWithNoVelOrPosIterations)
{
	const auto gravity = Vec2{0, RealNum(-9.8)};
	
	World world{World::Def{}.UseGravity(gravity)};
	
	BodyDef def;
	def.position = Vec2{RealNum(31.9), RealNum(-19.24)};
	def.type = BodyType::Dynamic;
	
	const auto body = world.CreateBody(def);
	ASSERT_NE(body, nullptr);
	EXPECT_EQ(body->GetLocation().x, def.position.x);
	EXPECT_EQ(body->GetLocation().y, def.position.y);
	EXPECT_EQ(GetLinearVelocity(*body).x, RealNum(0));
	EXPECT_EQ(GetLinearVelocity(*body).y, RealNum(0));
	EXPECT_EQ(body->GetLinearAcceleration().x, 0);
	EXPECT_EQ(body->GetLinearAcceleration().y, gravity.y);
	
	const auto time_inc = RealNum(0.01);
	
	auto pos = body->GetLocation();
	auto vel = GetLinearVelocity(*body);
	for (auto i = 0; i < 100; ++i)
	{
		Step(world, Time{Second * time_inc}, 0, 0);
		
		EXPECT_EQ(body->GetLinearAcceleration().y, gravity.y);
		
		EXPECT_EQ(body->GetLocation().x, def.position.x);
		EXPECT_LT(body->GetLocation().y, pos.y);
		EXPECT_EQ(body->GetLocation().y, pos.y + (vel.y + gravity.y * time_inc) * time_inc);
		pos = body->GetLocation();
		
		EXPECT_EQ(GetLinearVelocity(*body).x, RealNum(0));
		EXPECT_LT(GetLinearVelocity(*body).y, vel.y);
		EXPECT_TRUE(almost_equal(GetLinearVelocity(*body).y, vel.y + gravity.y * time_inc));
		vel = GetLinearVelocity(*body);
	}
}


TEST(World, BodyAccelRevPerSpecWithNegativeTimeAndNoVelOrPosIterations)
{
	const auto gravity = Vec2{0, RealNum(-9.8)};
	
	World world{World::Def{}.UseGravity(gravity)};
	
	BodyDef def;
	def.position = Vec2{RealNum(31.9), RealNum(-19.24)};
	def.linearVelocity = Vec2{0, RealNum(-9.8)};
	def.type = BodyType::Dynamic;
	
	const auto body = world.CreateBody(def);
	ASSERT_NE(body, nullptr);
	EXPECT_EQ(body->GetLocation().x, def.position.x);
	EXPECT_EQ(body->GetLocation().y, def.position.y);
	EXPECT_EQ(GetLinearVelocity(*body).x, RealNum(0));
	EXPECT_EQ(GetLinearVelocity(*body).y, RealNum(-9.8));
	EXPECT_EQ(body->GetLinearAcceleration().x, 0);
	EXPECT_EQ(body->GetLinearAcceleration().y, gravity.y);
	
	const auto time_inc = RealNum{-0.01f};
	auto stepConf = StepConf{};
	stepConf.set_dt(Time{Second * time_inc});
	stepConf.dtRatio = -1;
	stepConf.regPositionIterations = 0;
	stepConf.regVelocityIterations = 0;
	stepConf.toiPositionIterations = 0;
	stepConf.toiVelocityIterations = 0;
	
	auto pos = body->GetLocation();
	auto vel = GetLinearVelocity(*body);
	for (auto i = 0; i < 99; ++i)
	{
		world.Step(stepConf);
		
		EXPECT_EQ(body->GetLinearAcceleration().y, gravity.y);
		
		EXPECT_EQ(body->GetLocation().x, def.position.x);
		EXPECT_GT(body->GetLocation().y, pos.y);
		EXPECT_EQ(body->GetLocation().y, pos.y + (vel.y + gravity.y * time_inc) * time_inc);
		pos = body->GetLocation();
		
		EXPECT_EQ(GetLinearVelocity(*body).x, RealNum(0));
		EXPECT_GT(GetLinearVelocity(*body).y, vel.y);
		EXPECT_TRUE(almost_equal(GetLinearVelocity(*body).y, vel.y + gravity.y * time_inc));
		vel = GetLinearVelocity(*body);
	}
}

class MyContactListener: public ContactListener
{
public:
	using PreSolver = std::function<void(Contact&, const Manifold&)>;
	using PostSolver = std::function<void(Contact&, const ContactImpulsesList&, ContactListener::iteration_type)>;
	using Ender = std::function<void(Contact&)>;

	MyContactListener(PreSolver&& pre, PostSolver&& post, Ender&& end): presolver(pre), postsolver(post), ender(end) {}

	virtual ~MyContactListener() {}

	void BeginContact(Contact& contact) override
	{
		++begin_contacts;
		contacting = true;
		touching = contact.IsTouching();
		
		body_a[0] = contact.GetFixtureA()->GetBody()->GetLocation();
		body_b[0] = contact.GetFixtureB()->GetBody()->GetLocation();
	}
	
	void EndContact(Contact& contact) override
	{
		++end_contacts;
		contacting = false;
		touching = contact.IsTouching();

		body_a[1] = contact.GetFixtureA()->GetBody()->GetLocation();
		body_b[1] = contact.GetFixtureB()->GetBody()->GetLocation();
		
		if (ender)
		{
			ender(contact);
		}
	}
	
	void PreSolve(Contact& contact, const Manifold& oldManifold) override
	{
		++pre_solves;
		presolver(contact, oldManifold);
	}
	
	void PostSolve(Contact& contact, const ContactImpulsesList& impulse, ContactListener::iteration_type solved) override
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
	const auto x = RealNum(10); // other test parameters tuned to this value being 10

	auto presolved = unsigned{0};
	auto postsolved = unsigned{0};
	MyContactListener listener{
		[&](Contact&, const Manifold&) { ++presolved; },
		[&](Contact&, const ContactImpulsesList&, ContactListener::iteration_type) { ++postsolved; },
		[&](Contact&) {},
	};

	const Vec2 gravity{0, 0};
	World world{World::Def{}.UseGravity(gravity)};
	world.SetContactListener(&listener);
	
	ASSERT_EQ(listener.begin_contacts, unsigned(0));
	ASSERT_EQ(listener.end_contacts, unsigned(0));
	
	auto body_def = BodyDef{};
	body_def.type = BodyType::Dynamic;
	body_def.bullet = true;
	
	const auto shape = std::make_shared<CircleShape>(1);
	shape->SetDensity(RealNum{1} * KilogramPerSquareMeter);
	shape->SetRestitution(1);
	
	body_def.position = Vec2{-x, 0};
	body_def.linearVelocity = Vec2{+x, 0};
	const auto body_a = world.CreateBody(body_def);
	ASSERT_NE(body_a, nullptr);
	EXPECT_EQ(body_a->GetType(), BodyType::Dynamic);
	EXPECT_TRUE(body_a->IsSpeedable());
	EXPECT_TRUE(body_a->IsAccelerable());
	const auto fixture1 = body_a->CreateFixture(shape);
	ASSERT_NE(fixture1, nullptr);
	
	body_def.position = Vec2{+x, 0};
	body_def.linearVelocity = Vec2{-x, 0};
	const auto body_b = world.CreateBody(body_def);
	ASSERT_NE(body_b, nullptr);
	const auto fixture2 = body_b->CreateFixture(shape);
	ASSERT_NE(fixture2, nullptr);
	EXPECT_EQ(body_b->GetType(), BodyType::Dynamic);
	EXPECT_TRUE(body_b->IsSpeedable());
	EXPECT_TRUE(body_b->IsAccelerable());

	EXPECT_EQ(GetLinearVelocity(*body_a).x, +x);
	EXPECT_EQ(GetLinearVelocity(*body_a).y, 0);
	EXPECT_EQ(GetLinearVelocity(*body_b).x, -x);
	EXPECT_EQ(GetLinearVelocity(*body_b).y, 0);

	const auto time_inc = RealNum(.01);

	auto pos_a = body_a->GetLocation();
	auto pos_b = body_b->GetLocation();
	ASSERT_LT(pos_a.x, pos_b.x);

	auto steps = unsigned{0};
	while (pos_a.x < x && pos_b.x > -x)
	{
		Step(world, Time{Second * time_inc}, 0, 0);
		++steps;
		
		EXPECT_TRUE(almost_equal(body_a->GetLocation().x, pos_a.x + x * time_inc));
		EXPECT_EQ(body_a->GetLocation().y, 0);
		EXPECT_TRUE(almost_equal(body_b->GetLocation().x, pos_b.x - x * time_inc));
		EXPECT_EQ(body_b->GetLocation().y, 0);

		EXPECT_EQ(GetLinearVelocity(*body_a).x, +x);
		EXPECT_EQ(GetLinearVelocity(*body_a).y, 0);
		EXPECT_EQ(GetLinearVelocity(*body_b).x, -x);
		EXPECT_EQ(GetLinearVelocity(*body_b).y, 0);

		pos_a = body_a->GetLocation();
		pos_b = body_b->GetLocation();
	}
	
	// d = v * t
	// d = 20, v = 10:
	// 20 = 10 * t, t = d/v = 20 / 10 = 2
	// steps = t / time_inc = 200
	EXPECT_GE(steps, 199u);
	EXPECT_LE(steps, 201u);
	//EXPECT_EQ(int64_t(steps), static_cast<int64_t>(std::round(((x * 2) / x) / time_inc)));
}

TEST(World, PerfectlyOverlappedSameCirclesStayPut)
{
	const auto radius = RealNum(1);
	const auto shape = std::make_shared<CircleShape>(radius);
	shape->SetDensity(RealNum{1} * KilogramPerSquareMeter);
	shape->SetRestitution(1); // changes where bodies will be after collision
	const Vec2 gravity{0, 0};

	World world{World::Def{}.UseGravity(gravity)};
	
	auto body_def = BodyDef{};
	body_def.type = BodyType::Dynamic;
	body_def.bullet = false;
	body_def.position = Vec2{RealNum(0), RealNum(0)};

	const auto body1 = world.CreateBody(body_def);
	{
		const auto fixture = body1->CreateFixture(shape);
		ASSERT_NE(fixture, nullptr);
	}
	ASSERT_EQ(body1->GetLocation().x, body_def.position.x);
	ASSERT_EQ(body1->GetLocation().y, body_def.position.y);
	
	const auto body2 = world.CreateBody(body_def);
	{
		const auto fixture = body2->CreateFixture(shape);
		ASSERT_NE(fixture, nullptr);
	}
	ASSERT_EQ(body2->GetLocation().x, body_def.position.x);
	ASSERT_EQ(body2->GetLocation().y, body_def.position.y);
	
	const auto time_inc = RealNum(.01);
	for (auto i = 0; i < 100; ++i)
	{
		Step(world, Time{Second * time_inc});
		EXPECT_EQ(body1->GetLocation().x, body_def.position.x);
		EXPECT_EQ(body1->GetLocation().y, body_def.position.y);
		EXPECT_EQ(body2->GetLocation().x, body_def.position.x);
		EXPECT_EQ(body2->GetLocation().y, body_def.position.y);
	}
}

TEST(World, PerfectlyOverlappedConcentricCirclesStayPut)
{
	const auto radius1 = RealNum(1);
	const auto radius2 = RealNum(0.6);
	
	const auto shape1 = std::make_shared<CircleShape>(radius1);
	shape1->SetDensity(RealNum{1} * KilogramPerSquareMeter);
	shape1->SetRestitution(1); // changes where bodies will be after collision
	
	const auto shape2 = std::make_shared<CircleShape>(radius2);
	shape2->SetDensity(RealNum{1} * KilogramPerSquareMeter);
	shape2->SetRestitution(1); // changes where bodies will be after collision

	const Vec2 gravity{0, 0};
	
	World world{World::Def{}.UseGravity(gravity)};
	
	auto body_def = BodyDef{};
	body_def.type = BodyType::Dynamic;
	body_def.bullet = false;
	body_def.position = Vec2{RealNum(0), RealNum(0)};
	
	const auto body1 = world.CreateBody(body_def);
	{
		const auto fixture = body1->CreateFixture(shape1);
		ASSERT_NE(fixture, nullptr);
	}
	ASSERT_EQ(body1->GetLocation().x, body_def.position.x);
	ASSERT_EQ(body1->GetLocation().y, body_def.position.y);
	
	const auto body2 = world.CreateBody(body_def);
	{
		const auto fixture = body2->CreateFixture(shape2);
		ASSERT_NE(fixture, nullptr);
	}
	ASSERT_EQ(body2->GetLocation().x, body_def.position.x);
	ASSERT_EQ(body2->GetLocation().y, body_def.position.y);
	
	const auto time_inc = RealNum(.01);
	for (auto i = 0; i < 100; ++i)
	{
		Step(world, Time{Second * time_inc});
		EXPECT_EQ(body1->GetLocation().x, body_def.position.x);
		EXPECT_EQ(body1->GetLocation().y, body_def.position.y);
		EXPECT_EQ(body2->GetLocation().x, body_def.position.x);
		EXPECT_EQ(body2->GetLocation().y, body_def.position.y);
	}
}

TEST(World, ListenerCalledForCircleBodyWithinCircleBody)
{
	World world{World::Def{}.UseGravity(Vec2(0, 0))};
	MyContactListener listener{
		[&](Contact&, const Manifold&) {},
		[&](Contact&, const ContactImpulsesList&, ContactListener::iteration_type) {},
		[&](Contact&) {},
	};
	world.SetContactListener(&listener);

	auto body_def = BodyDef{};
	body_def.type = BodyType::Dynamic;
	body_def.position = Vec2{RealNum(0), RealNum(0)};
	const auto shape = std::make_shared<CircleShape>(1);
	shape->SetDensity(RealNum{1} * KilogramPerSquareMeter);
	shape->SetRestitution(1);
	for (auto i = 0; i < 2; ++i)
	{
		const auto body = world.CreateBody(body_def);
		ASSERT_NE(body, nullptr);
		ASSERT_NE(body->CreateFixture(shape), nullptr);
	}

	ASSERT_EQ(listener.begin_contacts, 0u);
	ASSERT_EQ(listener.end_contacts, 0u);
	ASSERT_EQ(listener.pre_solves, 0u);
	ASSERT_EQ(listener.post_solves, 0u);

	Step(world, Second);

	EXPECT_NE(listener.begin_contacts, 0u);
	EXPECT_EQ(listener.end_contacts, 0u);
	EXPECT_NE(listener.pre_solves, 0u);
	EXPECT_NE(listener.post_solves, 0u);
}

TEST(World, ListenerCalledForSquareBodyWithinSquareBody)
{
	World world{World::Def{}.UseGravity(Vec2(0, 0))};
	MyContactListener listener{
		[&](Contact&, const Manifold&) {},
		[&](Contact&, const ContactImpulsesList&, ContactListener::iteration_type) {},
		[&](Contact&) {},
	};
	world.SetContactListener(&listener);
	
	auto body_def = BodyDef{};
	body_def.type = BodyType::Dynamic;
	body_def.position = Vec2{RealNum(0), RealNum(0)};
	auto shape = std::make_shared<PolygonShape>();
	shape->SetVertexRadius(1);
	shape->SetAsBox(2, 2);
	shape->SetDensity(RealNum{1} * KilogramPerSquareMeter);
	shape->SetRestitution(1);
	for (auto i = 0; i < 2; ++i)
	{
		const auto body = world.CreateBody(body_def);
		ASSERT_NE(body, nullptr);
		ASSERT_NE(body->CreateFixture(shape), nullptr);
	}
	
	ASSERT_EQ(listener.begin_contacts, 0u);
	ASSERT_EQ(listener.end_contacts, 0u);
	ASSERT_EQ(listener.pre_solves, 0u);
	ASSERT_EQ(listener.post_solves, 0u);
	
	Step(world, Second);
	
	EXPECT_NE(listener.begin_contacts, 0u);
	EXPECT_EQ(listener.end_contacts, 0u);
	EXPECT_NE(listener.pre_solves, 0u);
	EXPECT_NE(listener.post_solves, 0u);
}

TEST(World, PartiallyOverlappedSameCirclesSeparate)
{
	const auto radius = RealNum(1);
	
	const Vec2 gravity{0, 0};
	World world{World::Def{}.UseGravity(gravity)};
	
	auto body_def = BodyDef{};
	body_def.type = BodyType::Dynamic;
	body_def.bullet = false; // separation is faster if true.
	
	const auto shape = std::make_shared<CircleShape>(radius);
	shape->SetDensity(RealNum{1} * KilogramPerSquareMeter);
	shape->SetRestitution(1); // changes where bodies will be after collision
	
	const auto body1pos = Vec2{-radius/4, 0};
	body_def.position = body1pos;
	const auto body1 = world.CreateBody(body_def);
	{
		const auto fixture = body1->CreateFixture(shape);
		ASSERT_NE(fixture, nullptr);
	}
	ASSERT_EQ(body1->GetLocation().x, body_def.position.x);
	ASSERT_EQ(body1->GetLocation().y, body_def.position.y);
	
	const auto body2pos = Vec2{+radius/4, 0};
	body_def.position = body2pos;
	const auto body2 = world.CreateBody(body_def);
	{
		const auto fixture = body2->CreateFixture(shape);
		ASSERT_NE(fixture, nullptr);
	}
	ASSERT_EQ(body2->GetLocation().x, body_def.position.x);
	ASSERT_EQ(body2->GetLocation().y, body_def.position.y);
	
	auto position_diff = body2pos - body1pos;
	auto distance = GetLength(position_diff);

	const auto angle = GetAngle(position_diff);
	ASSERT_EQ(angle, 0_deg);

	auto lastpos1 = body1->GetLocation();
	auto lastpos2 = body2->GetLocation();

	const auto time_inc = RealNum(.01f);
	StepConf step;
	step.set_dt(Time{Second * time_inc});

	// Solver won't separate more than -step.linearSlop.
	const auto full_separation = radius * 2 - step.linearSlop;
	for (auto i = 0; i < 100; ++i)
	{
		world.Step(step);

		const auto new_pos_diff = body2->GetLocation() - body1->GetLocation();
		const auto new_distance = GetLength(new_pos_diff);
		
		if (almost_equal(new_distance, full_separation) || new_distance > full_separation)
		{
			break;
		}
		
		ASSERT_GE(new_distance, distance);

		if (new_distance == distance)
		{
			// position resolution has come to tolerance
			ASSERT_GE(new_distance, radius * 2 - step.linearSlop * 4);
			break;
		}
		else // new_distance > distance
		{
			if (Cos(angle) != 0)
			{
				EXPECT_LT(body1->GetLocation().x, lastpos1.x);
				EXPECT_GT(body2->GetLocation().x, lastpos2.x);
			}
			if (Sin(angle) != 0)
			{
				EXPECT_LT(body1->GetLocation().y, lastpos1.y);
				EXPECT_GT(body2->GetLocation().y, lastpos2.y);
			}
		}

		ASSERT_NE(body1->GetLocation(), lastpos1);
		ASSERT_NE(body2->GetLocation(), lastpos2);
		
		lastpos1 = body1->GetLocation();
		lastpos2 = body2->GetLocation();

		ASSERT_NE(new_pos_diff, position_diff);
		position_diff = new_pos_diff;

		ASSERT_NE(new_distance, distance);
		distance = new_distance;

		// angle of the delta of their positions should stay the same as they move away
		const auto new_angle = GetAngle(new_pos_diff);
		EXPECT_EQ(angle, new_angle);
	}
}

TEST(World, PerfectlyOverlappedSameSquaresSeparateHorizontally)
{
	const auto shape = std::make_shared<PolygonShape>(1, 1);
	shape->SetDensity(RealNum{1} * KilogramPerSquareMeter);
	shape->SetRestitution(1); // changes where bodies will be after collision

	const Vec2 gravity{0, 0};
	
	World world{World::Def{}.UseGravity(gravity)};
	
	auto body_def = BodyDef{};
	body_def.type = BodyType::Dynamic;
	body_def.bullet = false;
	body_def.position = Vec2{RealNum(0), RealNum(0)};
	
	const auto body1 = world.CreateBody(body_def);
	{
		const auto fixture = body1->CreateFixture(shape);
		ASSERT_NE(fixture, nullptr);
	}
	ASSERT_EQ(body1->GetLocation().x, body_def.position.x);
	ASSERT_EQ(body1->GetLocation().y, body_def.position.y);
	
	const auto body2 = world.CreateBody(body_def);
	{
		const auto fixture = body2->CreateFixture(shape);
		ASSERT_NE(fixture, nullptr);
	}
	ASSERT_EQ(body2->GetLocation().x, body_def.position.x);
	ASSERT_EQ(body2->GetLocation().y, body_def.position.y);
	
	auto lastpos1 = body1->GetLocation();
	auto lastpos2 = body2->GetLocation();

	auto stepConf = StepConf{};
	const auto time_inc = RealNum(.01);
	stepConf.set_dt(Time{Second * time_inc});
	stepConf.maxLinearCorrection = 0.0001f * 40;
	for (auto i = 0; i < 100; ++i)
	{
		world.Step(stepConf);
		
		// body1 moves left only
		EXPECT_LT(body1->GetLocation().x, lastpos1.x);
		EXPECT_EQ(body1->GetLocation().y, lastpos1.y);

		// body2 moves right only
		EXPECT_GT(body2->GetLocation().x, lastpos2.x);
		EXPECT_EQ(body2->GetLocation().y, lastpos2.y);
		
		// body1 and body2 move away from each other equally.
		EXPECT_EQ(body1->GetLocation().x, -body2->GetLocation().x);
		EXPECT_EQ(body1->GetLocation().y, -body2->GetLocation().y);
		
		lastpos1 = body1->GetLocation();
		lastpos2 = body2->GetLocation();
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
	
	const auto half_dim = RealNum(64); // 1 causes additional y-axis separation
	const auto shape = std::make_shared<PolygonShape>(half_dim, half_dim);
	shape->SetDensity(RealNum{1} * KilogramPerSquareMeter);
	shape->SetRestitution(1); // changes where bodies will be after collision
	
	const auto body1pos = Vec2{RealNum(half_dim/2), RealNum(0)}; // 0 causes additional y-axis separation
	body_def.position = body1pos;
	const auto body1 = world.CreateBody(body_def);
	{
		const auto fixture = body1->CreateFixture(shape);
		ASSERT_NE(fixture, nullptr);
	}
	ASSERT_EQ(body1->GetLocation().x, body1pos.x);
	ASSERT_EQ(body1->GetLocation().y, body1pos.y);
	
	const auto body2pos = Vec2{-RealNum(half_dim/2), RealNum(0)}; // 0 causes additional y-axis separation
	body_def.position = body2pos;
	const auto body2 = world.CreateBody(body_def);
	{
		const auto fixture = body2->CreateFixture(shape);
		ASSERT_NE(fixture, nullptr);
	}
	ASSERT_EQ(body2->GetLocation().x, body2pos.x);
	ASSERT_EQ(body2->GetLocation().y, body2pos.y);

	ASSERT_EQ(body1->GetAngle(), 0_deg);
	ASSERT_EQ(body2->GetAngle(), 0_deg);
	auto last_angle_1 = body1->GetAngle();
	auto last_angle_2 = body2->GetAngle();

	ASSERT_EQ(world.GetBodies().size(), World::Bodies::size_type(2));
	ASSERT_EQ(world.GetContacts().size(), World::Contacts::size_type(0));

	auto position_diff = body1pos - body2pos;
	auto distance = GetLength(position_diff);
	
	auto angle = GetAngle(position_diff);
	EXPECT_TRUE(almost_equal(angle.ToRadians(), (0_deg).ToRadians()));
	
	auto lastpos1 = body1->GetLocation();
	auto lastpos2 = body2->GetLocation();
	
	const auto velocity_iters = 10u;
	const auto position_iters = 10u;
	
	const auto time_inc = RealNum(.01);
	StepConf step;
	step.set_dt(Time{Second * time_inc});
	// Solver won't separate more than -step.linearSlop.
	const auto full_separation = half_dim * 2 - step.linearSlop;
	for (auto i = 0; i < 100; ++i)
	{
		Step(world, Time{Second * time_inc}, velocity_iters, position_iters);
		
		ASSERT_EQ(world.GetContacts().size(), decltype(world.GetContacts().size())(1));

		auto count = decltype(world.GetContacts().size())(0);
		const auto& contacts = world.GetContacts();
		for (auto&& c: contacts)
		{
			++count;

			const auto fa = c->GetFixtureA();
			const auto fb = c->GetFixtureB();
			const auto body_a = fa->GetBody();
			const auto body_b = fb->GetBody();
			EXPECT_EQ(body_a, body1);
			EXPECT_EQ(body_b, body2);
			
			const auto& manifold = c->GetManifold();
			EXPECT_EQ(manifold.GetType(), Manifold::e_faceA);
			EXPECT_EQ(manifold.GetPointCount(), Manifold::size_type(2));
		}
		ASSERT_EQ(count, decltype(world.GetContacts().size())(1));

		const auto v1 = body1->GetVelocity();
		EXPECT_EQ(v1.angular, 0_deg);
		EXPECT_EQ(v1.linear.x, RealNum(0));
		EXPECT_EQ(v1.linear.y, RealNum(0));

		const auto v2 = body2->GetVelocity();
		EXPECT_EQ(v2.angular, 0_deg);
		EXPECT_EQ(v2.linear.x, RealNum(0));
		EXPECT_EQ(v2.linear.y, RealNum(0));

		EXPECT_TRUE(almost_equal(body1->GetAngle().ToRadians(), last_angle_1.ToRadians()));
		EXPECT_TRUE(almost_equal(body2->GetAngle().ToRadians(), last_angle_2.ToRadians()));
		last_angle_1 = body1->GetAngle();
		last_angle_2 = body2->GetAngle();

		const auto new_pos_diff = body1->GetLocation() - body2->GetLocation();
		const auto new_distance = GetLength(new_pos_diff);
		
		if (almost_equal(new_distance, full_separation) || new_distance > full_separation)
		{
			break;
		}
		
		if (new_distance == distance)
		{
			if (std::cos(angle.ToRadians()) != 0)
			{
				EXPECT_NE(body1->GetLocation().x, lastpos1.x);
				EXPECT_NE(body2->GetLocation().x, lastpos2.x);
			}
			if (std::sin(angle.ToRadians()) != 0)
			{
				EXPECT_NE(body1->GetLocation().y, lastpos1.y);
				EXPECT_NE(body2->GetLocation().y, lastpos2.y);
			}
			ASSERT_GE(new_distance, RealNum(2));
			break;
		}
		
		ASSERT_NE(body1->GetLocation(), lastpos1);
		ASSERT_NE(body2->GetLocation(), lastpos2);
		
		// Body 1 moves right only.
		EXPECT_GT(body1->GetLocation().x, lastpos1.x);
		EXPECT_TRUE(almost_equal(body1->GetLocation().y, lastpos1.y));

		// Body 2 moves left only.
		EXPECT_LT(body2->GetLocation().x, lastpos2.x);
		EXPECT_TRUE(almost_equal(body2->GetLocation().y, lastpos2.y));

		lastpos1 = body1->GetLocation();
		lastpos2 = body2->GetLocation();
		
		ASSERT_NE(new_pos_diff, position_diff);
		position_diff = new_pos_diff;
		
		ASSERT_NE(new_distance, distance);
		distance = new_distance;
		
		const auto new_angle = GetAngle(new_pos_diff);
		EXPECT_TRUE(almost_equal(angle.ToRadians(), new_angle.ToRadians()));
		
		angle = new_angle;
	}
}

TEST(World, CollidingDynamicBodies)
{
	const auto radius = RealNum(1);
	const auto x = RealNum(10); // other test parameters tuned to this value being 10

	auto body_def = BodyDef{};
	body_def.type = BodyType::Dynamic;
	
	MyContactListener listener{
		[](Contact&, const Manifold&) {},
		[](Contact&, const ContactImpulsesList&, ContactListener::iteration_type) {},
		[&](Contact&) {},
	};

	const auto gravity = Vec2_zero;
	World world{World::Def{}.UseGravity(gravity)};
	EXPECT_EQ(world.GetGravity(), gravity);
	world.SetContactListener(&listener);
	
	const auto shape = std::make_shared<CircleShape>(radius);
	shape->SetDensity(RealNum{1} * KilogramPerSquareMeter);
	shape->SetRestitution(1); // changes where bodies will be after collision

	body_def.position = Vec2{-(x + 1), 0};
	body_def.linearVelocity = Vec2{+x, 0};
	const auto body_a = world.CreateBody(body_def);
	ASSERT_NE(body_a, nullptr);
	EXPECT_EQ(body_a->GetType(), BodyType::Dynamic);
	EXPECT_TRUE(body_a->IsSpeedable());
	EXPECT_TRUE(body_a->IsAccelerable());
	const auto fixture1 = body_a->CreateFixture(shape);
	ASSERT_NE(fixture1, nullptr);

	body_def.position = Vec2{+(x + 1), 0};
	body_def.linearVelocity = Vec2{-x, 0};
	const auto body_b = world.CreateBody(body_def);
	ASSERT_NE(body_b, nullptr);
	const auto fixture2 = body_b->CreateFixture(shape);
	ASSERT_NE(fixture2, nullptr);
	EXPECT_EQ(body_b->GetType(), BodyType::Dynamic);
	EXPECT_TRUE(body_b->IsSpeedable());
	EXPECT_TRUE(body_b->IsAccelerable());

	EXPECT_EQ(GetLinearVelocity(*body_a).x, +x);
	EXPECT_EQ(GetLinearVelocity(*body_a).y, 0);
	EXPECT_EQ(GetLinearVelocity(*body_b).x, -x);
	EXPECT_EQ(GetLinearVelocity(*body_b).y, 0);
	
	const auto time_collision = RealNum(1.0099994); // only valid for x >= around 4.214
	const auto time_inc = RealNum(.01);
	
	auto elapsed_time = RealNum(0);
	for (;;)
	{
		Step(world, Time{Second * time_inc});
		elapsed_time += time_inc;
		if (listener.contacting)
		{
			break;
		}
	}
	
	const auto time_contacting = elapsed_time;

	EXPECT_TRUE(listener.touching);
	EXPECT_NEAR(double(time_contacting), double(time_collision), 0.02);
	EXPECT_EQ(body_a->GetLocation().y, 0);
	EXPECT_EQ(body_b->GetLocation().y, 0);

	const auto tolerance = x / 100;
	
	// x position for body1 depends on restitution but it should be around -1
	EXPECT_GE(body_a->GetLocation().x, RealNum(-1) - tolerance);
	EXPECT_LT(body_a->GetLocation().x, RealNum(-1) + tolerance);

	// x position for body2 depends on restitution but it should be around +1
	EXPECT_LE(body_b->GetLocation().x, RealNum(+1) + tolerance);
	EXPECT_GT(body_b->GetLocation().x, RealNum(+1) - tolerance);
	
	// and their deltas from -1 and +1 should be about equal.
	EXPECT_TRUE(almost_equal(body_a->GetLocation().x + RealNum{1}, RealNum{1} - body_b->GetLocation().x));

	EXPECT_GE(listener.body_a[0].x, -1);
	EXPECT_LE(listener.body_b[0].x, +1);

	for (;;)
	{
		Step(world, Time{Second * time_inc});
		elapsed_time += time_inc;
		if (!listener.contacting && !listener.touching)
		{
			break;
		}
	}
	EXPECT_FALSE(listener.touching);
	
	EXPECT_TRUE(almost_equal(elapsed_time, time_contacting + time_inc));
	
	// collision should be fully resolved now...
	EXPECT_LT(body_a->GetLocation().x, RealNum(-1));
	EXPECT_GT(body_b->GetLocation().x, RealNum(+1));
	
	// and their deltas from -1 and +1 should be about equal.
	EXPECT_TRUE(almost_equal(body_a->GetLocation().x + RealNum{1}, RealNum{1} - body_b->GetLocation().x));

	EXPECT_LT(listener.body_a[1].x, -1);
	EXPECT_GT(listener.body_b[1].x, +1);
	
	// confirm conservation of momentum:
	// velocities should now be same magnitude but in opposite directions
	EXPECT_NEAR(double(GetLinearVelocity(*body_a).x), double(-x), 0.0001);
	EXPECT_EQ(GetLinearVelocity(*body_a).y, 0);
	EXPECT_NEAR(double(GetLinearVelocity(*body_b).x), double(+x), 0.0001);
	EXPECT_EQ(GetLinearVelocity(*body_b).y, 0);
}

#include <unistd.h>
#include <setjmp.h>
#include <signal.h>

static jmp_buf jmp_env;

static void catch_alarm(int)
{
	longjmp(jmp_env, 1);
}

/// Assert if the given function takes more than the given amount of microseconds.
/// @warning The use of <code>setjmp</code> and <code>longjmp</code> will result in undefined
///   behavior if non trivial destructors would be called if replaced with try and catch.
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


TEST(World, TilesComesToRestInUnder7secs)
{
	const auto m_world = std::make_unique<World>();
	
	constexpr auto e_count = 36;
	
	{
		const auto a = 0.5f;
		const auto ground = m_world->CreateBody(BodyDef{}.UseLocation(Vec2{0, -a}));
		
		const auto N = 200;
		const auto M = 10;
		Vec2 position;
		position.y = 0.0f;
		for (auto j = 0; j < M; ++j)
		{
			position.x = -N * a;
			for (auto i = 0; i < N; ++i)
			{
				PolygonShape shape;
				SetAsBox(shape, a, a, position, 0_rad);
				ground->CreateFixture(std::make_shared<PolygonShape>(shape));
				position.x += 2.0f * a;
			}
			position.y -= 2.0f * a;
		}
	}
	
	{
		const auto a = 0.5f;
		const auto shape = std::make_shared<PolygonShape>(a, a);
		shape->SetDensity(RealNum{5} * KilogramPerSquareMeter);
		
		Vec2 x(-7.0f, 0.75f);
		Vec2 y;
		const auto deltaX = Vec2(0.5625f, 1.25f);
		const auto deltaY = Vec2(1.125f, 0.0f);
		
		for (auto i = 0; i < e_count; ++i)
		{
			y = x;
			
			for (auto j = i; j < e_count; ++j)
			{
				const auto body = m_world->CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(y));
				body->CreateFixture(shape);
				y += deltaY;
			}
			
			x += deltaX;
		}
	}
	
	StepConf step;
	step.set_dt(Time{Second / RealNum{60}});

	const auto start_time = std::chrono::high_resolution_clock::now();
	while (GetAwakeCount(*m_world) > 0)
	{
		m_world->Step(step);
	}
	const auto end_time = std::chrono::high_resolution_clock::now();
	
	const std::chrono::duration<double> elapsed_time = end_time - start_time;
	
	// seeing e_count=20 times around:
	//   0.447077s with RealNum=float and NDEBUG defined.
	//   6.45222s with RealNum=float and NDEBUG not defined.
	//   0.456306s with RealNum=double and NDEBUG defined.
	//   6.74324s with RealNum=double and NDEBUG not defined.
	
	// seeing e_count=24 times around:
	//   0.956078s with RealNum=float and NDEBUG defined.
	//   0.989387s with RealNum=double and NDEBUG defined.
	
	// seeing e_count=30 times around:
	//   2.35464s with RealNum=float and NDEBUG defined.
	//   2.51661s with RealNum=double and NDEBUG defined.
	
	// seeing e_count=36 times around:
	//   4.85618s with RealNum=float and NDEBUG defined.
	//   5.32973s with RealNum=double and NDEBUG defined.
	
	//std::cout << "Time: " << elapsed_time.count() << "s" << std::endl;
	EXPECT_LT(elapsed_time.count(), 7.0);
}

TEST(World, SpeedingBulletBallWontTunnel)
{
	World world{World::Def{}.UseGravity(Vec2_zero)};

	MyContactListener listener{
		[](Contact&, const Manifold&) {},
		[](Contact&, const ContactImpulsesList&, ContactListener::iteration_type) {},
		[&](Contact&) {},
	};
	world.SetContactListener(&listener);

	ASSERT_EQ(listener.begin_contacts, unsigned{0});

	const auto left_edge_x = RealNum(-0.1);
	const auto right_edge_x = RealNum(+0.1);

	BodyDef body_def;
	const auto edge_shape = std::make_shared<EdgeShape>(Vec2{0, +10}, Vec2{0, -10});
	edge_shape->SetRestitution(1);

	body_def.type = BodyType::Static;

	body_def.position = Vec2{left_edge_x, 0};
	const auto left_wall_body = world.CreateBody(body_def);
	ASSERT_NE(left_wall_body, nullptr);
	{
		const auto wall_fixture = left_wall_body->CreateFixture(edge_shape);
		ASSERT_NE(wall_fixture, nullptr);
	}

	body_def.position = Vec2{right_edge_x, 0};
	const auto right_wall_body = world.CreateBody(body_def);
	ASSERT_NE(right_wall_body, nullptr);
	{
		const auto wall_fixture = right_wall_body->CreateFixture(edge_shape);
		ASSERT_NE(wall_fixture, nullptr);
	}
	
	const auto begin_x = RealNum(0);

	body_def.type = BodyType::Dynamic;
	body_def.position = Vec2{begin_x, 0};
	body_def.bullet = false;
	const auto ball_body = world.CreateBody(body_def);
	ASSERT_NE(ball_body, nullptr);
	
	const auto ball_radius = RealNum(.01);
	const auto circle_shape = std::make_shared<CircleShape>(ball_radius);
	circle_shape->SetDensity(RealNum{1} * KilogramPerSquareMeter);
	circle_shape->SetRestitution(1); // changes where bodies will be after collision
	const auto ball_fixture = ball_body->CreateFixture(circle_shape);
	ASSERT_NE(ball_fixture, nullptr);

	const auto velocity = Vec2{+1, 0};
	ball_body->SetVelocity(Velocity{velocity, 0_deg});

	const auto time_inc = RealNum(.01);
	auto stepConf = StepConf{};
	stepConf.set_dt(Time{Second * time_inc});
	const auto max_velocity = stepConf.maxTranslation / time_inc;
	world.Step(stepConf);

	ASSERT_EQ(listener.begin_contacts, unsigned{0});

	EXPECT_GT(ball_body->GetLocation().x, begin_x);

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
			ASSERT_USECS(world.Step(stepConf), 5000);

			EXPECT_LT(ball_body->GetLocation().x, right_edge_x - (ball_radius/2));
			EXPECT_GT(ball_body->GetLocation().x, left_edge_x + (ball_radius/2));

			if (ball_body->GetVelocity().linear.x >= max_velocity)
			{
				return;
			}

			if (listener.begin_contacts % 2 != 0) // direction switched
			{
				EXPECT_LT(ball_body->GetVelocity().linear.x, 0);
				break; // going left now
			}
			else if (listener.begin_contacts > last_contact_count)
			{
				++increments;
				ball_body->SetVelocity(Velocity{Vec2{+increments * velocity.x, ball_body->GetVelocity().linear.y}, ball_body->GetVelocity().angular});
			}
			else
			{
				EXPECT_TRUE(almost_equal(ball_body->GetVelocity().linear.x, +increments * velocity.x));
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
			ASSERT_USECS(world.Step(stepConf), 5000);
			
			EXPECT_LT(ball_body->GetLocation().x, right_edge_x - (ball_radius/2));
			EXPECT_GT(ball_body->GetLocation().x, left_edge_x + (ball_radius/2));

			if (ball_body->GetVelocity().linear.x <= -max_velocity)
			{
				return;
			}

			if (listener.begin_contacts % 2 != 0) // direction switched
			{
				EXPECT_GT(ball_body->GetVelocity().linear.x, 0);
				break; // going right now
			}
			else if (listener.begin_contacts > last_contact_count)
			{
				++increments;
				ball_body->SetVelocity(Velocity{Vec2{-increments * velocity.x, ball_body->GetVelocity().linear.y}, ball_body->GetVelocity().angular});
			}
			else
			{
				EXPECT_TRUE(almost_equal(ball_body->GetVelocity().linear.x, -increments * velocity.x));
			}
		}
		
		++increments;
		ball_body->SetVelocity(Velocity{Vec2{+increments * velocity.x, ball_body->GetVelocity().linear.y}, ball_body->GetVelocity().angular});
	}
}

TEST(World, MouseJointWontCauseTunnelling)
{
	World world{World::Def{}.UseGravity(Vec2_zero)};
	
	const auto half_box_width = RealNum(0.2);
	const auto left_edge_x = -half_box_width;
	const auto right_edge_x = +half_box_width;

	const auto half_box_height = RealNum(0.2);
	const auto btm_edge_y = -half_box_height;
	const auto top_edge_y = +half_box_height;

	AABB container_aabb;

	BodyDef body_def;
	EdgeShape edge_shape;
	edge_shape.SetFriction(0.4f);
	edge_shape.SetRestitution(0.94f); // changes where bodies will be after collision
	body_def.type = BodyType::Static;
	
	// Setup vertical bounderies
	edge_shape.Set(Vec2{0, +half_box_height * 2}, Vec2{0, -half_box_height * 2});

	body_def.position = Vec2{left_edge_x, 0};
	{
		const auto left_wall_body = world.CreateBody(body_def);
		ASSERT_NE(left_wall_body, nullptr);
		{
			const auto wall_fixture = left_wall_body->CreateFixture(std::make_shared<EdgeShape>(edge_shape));
			ASSERT_NE(wall_fixture, nullptr);
		}
		container_aabb += ComputeAABB(*left_wall_body);
	}
	
	body_def.position = Vec2{right_edge_x, 0};
	{
		const auto right_wall_body = world.CreateBody(body_def);
		ASSERT_NE(right_wall_body, nullptr);
		{
			const auto wall_fixture = right_wall_body->CreateFixture(std::make_shared<EdgeShape>(edge_shape));
			ASSERT_NE(wall_fixture, nullptr);
		}
		container_aabb += ComputeAABB(*right_wall_body);
	}

	// Setup horizontal bounderies
	edge_shape.Set(Vec2{-half_box_width * 2, 0}, Vec2{+half_box_width * 2, 0});
	
	body_def.position = Vec2{0, btm_edge_y};
	{
		const auto btm_wall_body = world.CreateBody(body_def);
		ASSERT_NE(btm_wall_body, nullptr);
		{
			const auto wall_fixture = btm_wall_body->CreateFixture(std::make_shared<EdgeShape>(edge_shape));
			ASSERT_NE(wall_fixture, nullptr);
		}
		container_aabb += ComputeAABB(*btm_wall_body);
	}
	
	body_def.position = Vec2{0, top_edge_y};
	{
		const auto top_wall_body = world.CreateBody(body_def);
		ASSERT_NE(top_wall_body, nullptr);
		{
			const auto wall_fixture = top_wall_body->CreateFixture(std::make_shared<EdgeShape>(edge_shape));
			ASSERT_NE(wall_fixture, nullptr);
		}
		container_aabb += ComputeAABB(*top_wall_body);
	}

	body_def.type = BodyType::Dynamic;
	body_def.position = Vec2_zero;
	body_def.bullet = true;
	
	const auto ball_body = world.CreateBody(body_def);
	ASSERT_NE(ball_body, nullptr);
	ASSERT_EQ(ball_body->GetLocation().x, 0);
	ASSERT_EQ(ball_body->GetLocation().y, 0);
	
	const auto ball_radius = RealNum(half_box_width / 4);
	const auto object_shape = std::make_shared<PolygonShape>(ball_radius, ball_radius);
	object_shape->SetDensity(RealNum{10} * KilogramPerSquareMeter);
	{
		const auto ball_fixture = ball_body->CreateFixture(object_shape);
		ASSERT_NE(ball_fixture, nullptr);
	}

	constexpr unsigned numBodies = 1;
	Vec2 last_opos[numBodies];
	Body *bodies[numBodies];
	for (auto i = decltype(numBodies){0}; i < numBodies; ++i)
	{
		const auto angle = i * 2 * Pi / numBodies;
		const auto x = ball_radius * RealNum(2.1) * std::cos(angle);
		const auto y = ball_radius * RealNum(2.1) * std::sin(angle);
		body_def.position = Vec2{x, y};
		bodies[i] = world.CreateBody(body_def);
		ASSERT_NE(bodies[i], nullptr);
		ASSERT_EQ(bodies[i]->GetLocation().x, x);
		ASSERT_EQ(bodies[i]->GetLocation().y, y);
		last_opos[i] = bodies[i]->GetLocation();
		{
			const auto fixture = bodies[i]->CreateFixture(object_shape);
			ASSERT_NE(fixture, nullptr);
		}
	}

	BodyDef bodyDef;
	const auto spare_body = world.CreateBody(bodyDef);

	const auto mouse_joint = [&]() {
		MouseJointDef mjd;
		mjd.bodyA = spare_body;
		mjd.bodyB = ball_body;
		const auto ball_body_pos = ball_body->GetLocation();
		mjd.target = Vec2{ball_body_pos.x - ball_radius / 2, ball_body_pos.y + ball_radius / 2};
		mjd.maxForce = RealNum(1000) * RealNum{GetMass(*ball_body) / Kilogram};
		return static_cast<MouseJoint*>(world.CreateJoint(mjd));
	}();
	ASSERT_NE(mouse_joint, nullptr);

	ball_body->SetAwake();

	auto max_x = RealNum(0);
	auto min_x = RealNum(0);
	auto max_y = RealNum(0);
	auto min_y = RealNum(0);

	auto max_velocity = RealNum(0);

	//const auto time_inc = RealNum(.0043268126901); // numBodies = 6, somewhat dependent on fixture density (10 or less?).
	//const auto time_inc = RealNum(.0039224); // numBodies = 4, maybe dependent on fixture density
	//const auto time_inc = RealNum(.003746); // numBodies = 2, maybe dependent on fixture density
	//const auto time_inc = RealNum(.0036728129); // numBodies = 1, maybe dependent on fixture density
	const auto time_inc = RealNum(.00367281295); // numBodies = 1, maybe dependent on fixture density

	auto angle = RealNum(0);
	auto anglular_speed = RealNum(0.01); // radians / timestep
	const auto anglular_accel = RealNum(1.002);
	auto distance = half_box_width / 2;
	auto distance_speed = RealNum(0.003); // meters / timestep
	const auto distance_accel = RealNum(1.001);

	MyContactListener listener{
		[&](Contact&, const Manifold& /* old_manifold */)
		{
			// PreSolve...
#if 0
			const auto new_manifold = contact.GetManifold();
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
		[&](Contact& contact, const ContactImpulsesList& impulse, ContactListener::iteration_type solved)
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
				const auto bpos = body->GetLocation();
				const auto lt = Vec2{right_edge_x, top_edge_y} - bpos;
				const auto gt = bpos - Vec2{left_edge_x, btm_edge_y};
				
				if (lt.x <= 0 || lt.y <= 0 || gt.x <= 0 || gt.y <= 0)
				{
					if (!TestOverlap(container_aabb, ComputeAABB(*body)))
					{
						// Body out of bounds and no longer even overlapping container!
						EXPECT_LT(body->GetLocation().x, right_edge_x);
						EXPECT_LT(body->GetLocation().y, top_edge_y);
						EXPECT_GT(body->GetLocation().x, left_edge_x);
						EXPECT_GT(body->GetLocation().y, btm_edge_y);
						++fail_count;
					}
				}
			}
			if (fail_count > 0)
			{
				std::cout << " angl=" << angle;
				std::cout << " ctoi=" << 0 + contact.GetToiCount();
				std::cout << " solv=" << 0 + solved;
				std::cout << " targ=(" << distance * std::cos(angle) << "," << distance * std::sin(angle) << ")";
				std::cout << " maxv=" << max_velocity;
				std::cout << " rang=(" << min_x << "," << min_y << ")-(" << max_x << "," << max_y << ")";
				std::cout << " bpos=(" << ball_body->GetLocation().x << "," << ball_body->GetLocation().y << ")";
				std::cout << std::endl;
				for (auto i = decltype(impulse.GetCount()){0}; i < impulse.GetCount(); ++i)
				{
					std::cout << " i#" << (0 + i) << "={n" << impulse.GetEntryNormal(i) << ",t" << impulse.GetEntryTanget(i) << "}";
				}
				std::cout << std::endl;

				std::cout << " bodyA=(" << body_a->GetLocation().x << "," << body_a->GetLocation().y << ")";
				if (body_a == ball_body) std::cout << " ball";
				if (!body_a->IsSpeedable()) std::cout << " wall";
				std::cout << " " << body_a;
				std::cout << std::endl;
				std::cout << " bodyB=(" << body_b->GetLocation().x << "," << body_b->GetLocation().y << ")";
				if (body_b == ball_body) std::cout << " ball";
				if (!body_b->IsSpeedable()) std::cout << " wall";
				std::cout << " " << body_b;
				std::cout << std::endl;

				//GTEST_FATAL_FAILURE_("");				
			}
		},
		[&](Contact& contact) {
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

				if (body->GetLocation().x >= right_edge_x)
				{
					escaped = true;
				}
				if (body->GetLocation().y >= top_edge_y)
				{
					escaped = true;
				}
				if (body->GetLocation().x <= left_edge_x)
				{
					escaped = true;
				}
				if (body->GetLocation().y <= btm_edge_y)
				{
					escaped = true;					
				}
			}
			if (escaped && !contact.IsTouching())
			{
				std::cout << "Escaped at EndContact[" << &contact << "]:";
				std::cout << " toiSteps=" << static_cast<unsigned>(contact.GetToiCount());
				std::cout << " toiValid=" << contact.HasValidToi();
				std::cout << " a[" << body_a << "]@(" << body_a->GetLocation().x << "," << body_a->GetLocation().y << ")";
				std::cout << " b[" << body_b << "]@(" << body_b->GetLocation().x << "," << body_b->GetLocation().y << ")";
				std::cout << std::endl;
				//exit(1);
			}
		},
	};
	ASSERT_EQ(listener.begin_contacts, unsigned{0});

	world.SetContactListener(&listener);
	
	for (auto outer = unsigned{0}; outer < 2000; ++outer)
	{
		auto last_pos = ball_body->GetLocation();
		for (auto loops = unsigned{0};; ++loops)
		{
			mouse_joint->SetTarget(Vec2{distance * std::cos(angle), distance * std::sin(angle)});
			angle += anglular_speed;
			distance += distance_speed;

			ASSERT_USECS(Step(world, Time{Second * time_inc}, 8, 3), 100000);
			
			ASSERT_LT(ball_body->GetLocation().x, right_edge_x);
			ASSERT_LT(ball_body->GetLocation().y, top_edge_y);
			ASSERT_GT(ball_body->GetLocation().x, left_edge_x);
			ASSERT_GT(ball_body->GetLocation().y, btm_edge_y);
			for (auto i = decltype(numBodies){0}; i < numBodies; ++i)
			{
				ASSERT_LT(bodies[i]->GetLocation().x, right_edge_x);
				ASSERT_LT(bodies[i]->GetLocation().y, top_edge_y);			
				ASSERT_GT(bodies[i]->GetLocation().x, left_edge_x);
				ASSERT_GT(bodies[i]->GetLocation().y, btm_edge_y);
			}

			max_x = Max(ball_body->GetLocation().x, max_x);
			min_x = Min(ball_body->GetLocation().x, min_x);

			max_y = Max(ball_body->GetLocation().y, max_y);
			min_y = Min(ball_body->GetLocation().y, min_y);

			max_velocity = Max(GetLength(ball_body->GetVelocity().linear), max_velocity);

			if (loops > 50)
			{
				if (mouse_joint->GetTarget().x < 0)
				{
					if (ball_body->GetLocation().x >= last_pos.x)
						break;					
				}
				else
				{
					if (ball_body->GetLocation().x <= last_pos.x)
						break;
				}
				if (mouse_joint->GetTarget().y < 0)
				{
					if (ball_body->GetLocation().y >= last_pos.y)
						break;
				}
				else
				{
					if (ball_body->GetLocation().y <= last_pos.y)
						break;
				}
			}
			last_pos = ball_body->GetLocation();
		}
		anglular_speed *= anglular_accel;
		distance_speed *= distance_accel;

		ASSERT_NE(ball_body->GetLocation(), Vec2_zero);
#if 0
		if (outer > 100)
		{
			for (auto i = decltype(numBodies){0}; i < numBodies; ++i)
			{
				// a sanity check to ensure the other bodies are getting moved
				EXPECT_NE(last_opos[i], bodies[i]->GetLocation());
				last_opos[i] = bodies[i]->GetLocation();
			}
		}
#endif
	}
#if 0
	std::cout << "angle=" << angle;
	std::cout << " target=(" << distance * std::cos(angle) << "," << distance * std::sin(angle) << ")";
	std::cout << " maxvel=" << max_velocity;
	std::cout << " range=(" << min_x << "," << min_y << ")-(" << max_x << "," << max_y << ")";
	std::cout << std::endl;
#endif
}

#if 0
static void smaller_still_conserves_momentum(bool bullet, RealNum multiplier, RealNum time_inc)
{
	const auto radius = RealNum(1);
	const auto start_distance = RealNum(10);
	
	auto scale = RealNum(1);
	for (;;)
	{
		const auto gravity = Vec2_zero;
		World world{World::Def{}.UseGravity(gravity)};
		ASSERT_EQ(world.GetGravity().x, 0);
		ASSERT_EQ(world.GetGravity().y, 0);

		auto maxNormalImpulse = RealNum(0);
		auto maxTangentImpulse = RealNum(0);
		auto maxPoints = 0u;
		auto numSteps = 0u;
		auto failed = false;
		auto preB1 = Vec2_zero;
		auto preB2 = Vec2_zero;
		
		MyContactListener listener{
			[&](Contact& contact, const Manifold&)
			{
				const auto fA = contact.GetFixtureA();
				const auto fB = contact.GetFixtureB();
				const auto bA = fA->GetBody();
				const auto bB = fB->GetBody();
				preB1 = bA->GetLocation();
				preB2 = bB->GetLocation();
			},
			[&](Contact& /* contact */, const ContactImpulsesList& impulse, ContactListener::iteration_type /* solved */)
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
#if 0
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
#endif
				}
			},
			[=](Contact&) {
			}
		};
		world.SetContactListener(&listener);

		const auto shape = std::make_shared<CircleShape>(scale * radius);
		ASSERT_EQ(shape->GetRadius(), scale * radius);
		
		auto fixture_def = FixtureDef{}.UseDensity(1);
		fixture_def.friction = 0;
		fixture_def.restitution = 1;
		
		auto body_def = BodyDef{};
		body_def.type = BodyType::Dynamic;
		body_def.bullet = bullet;
		
		body_def.position = Vec2{+(scale * start_distance), 0};
		body_def.linearVelocity = Vec2{-start_distance, 0};
		const auto body_1 = world.CreateBody(body_def);
		ASSERT_EQ(body_1->GetLocation().x, body_def.position.x);
		ASSERT_EQ(body_1->GetLocation().y, body_def.position.y);
		ASSERT_EQ(GetLinearVelocity(*body_1).x, body_def.linearVelocity.x);
		ASSERT_EQ(GetLinearVelocity(*body_1).y, body_def.linearVelocity.y);
		body_1->CreateFixture(shape, fixture_def);
		
		body_def.position = Vec2{-(scale * start_distance), 0};
		body_def.linearVelocity = Vec2{+start_distance, 0};
		const auto body_2 = world.CreateBody(body_def);
		ASSERT_EQ(body_2->GetLocation().x, body_def.position.x);
		ASSERT_EQ(body_2->GetLocation().y, body_def.position.y);
		ASSERT_EQ(GetLinearVelocity(*body_2).x, body_def.linearVelocity.x);
		ASSERT_EQ(GetLinearVelocity(*body_2).y, body_def.linearVelocity.y);
		body_2->CreateFixture(shape, fixture_def);
		
		for (;;)
		{
			const auto relative_velocity = GetLinearVelocity(*body_1) - GetLinearVelocity(*body_2);
			if (relative_velocity.x >= 0)
			{
				EXPECT_NEAR(double(relative_velocity.x), double(Abs(body_def.linearVelocity.x) * +2), 0.0001);
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
				std::cout << " pos1.x=" << body_1->GetLocation().x;
				std::cout << " pos2.x=" << body_2->GetLocation().x;
				std::cout << " preDel=" << (preB1.x - preB2.x);
				std::cout << " posDel=" << (body_1->GetLocation().x - body_2->GetLocation().x);
				std::cout << " travel=" << (body_1->GetLocation().x - preB1.x);
				std::cout << std::endl;
				ASSERT_FALSE(failed);
			}
			
			EXPECT_TRUE(almost_equal(relative_velocity.x, Abs(body_def.linearVelocity.x) * -2));
			Step(world, time_inc);
			++numSteps;
		}
		
		scale *= multiplier;
	}
}

TEST(World, SmallerStillConservesMomemtum)
{
	// smaller_still_conserves_momentum(false, RealNum(0.999), RealNum(0.01));
	// fails around scale=0.0899796 dist0=1.79959
	// goin to smaller time increment fails nearly same point.
	smaller_still_conserves_momentum(false, RealNum(0.999), RealNum(0.01));
}

TEST(World, SmallerBulletStillConservesMomemtum)
{
	// smaller_still_conserves_momentum(true, RealNum(0.999), RealNum(0.01))
	// fails around scale=4.99832e-05 dist0=0.000999664
	// goin to smaller time increment fails nearly same point.
// smaller_still_conserves_momentum(true, RealNum(0.999), RealNum(0.01));
}
#endif

class VerticalStackTest: public ::testing::TestWithParam<RealNum>
{
public:
	virtual void SetUp()
	{
		const auto hw_ground = 40.0f;
		const auto ground = world.CreateBody();
		ground->CreateFixture(std::make_shared<EdgeShape>(Vec2{-hw_ground, 0}, Vec2{hw_ground, 0}));
		
		const auto numboxes = boxes.size();
		
		original_x = GetParam();
		
		const auto boxShape = std::make_shared<PolygonShape>(hdim, hdim);
		boxShape->SetDensity(RealNum{1} * KilogramPerSquareMeter);
		boxShape->SetFriction(0.3f);
		for (auto i = decltype(numboxes){0}; i < numboxes; ++i)
		{
			// (hdim + 0.05f) + (hdim * 2 + 0.1f) * i
			const auto location = Vec2{original_x, (i + 1) * hdim * 4};
			const auto box = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(location));
			box->CreateFixture(boxShape);
			boxes[i] = box;
		}
		
		const auto stepConf = StepConf{}.set_dt(Time{Second / RealNum{60}});
		while (loopsTillSleeping < maxLoops)
		{
			world.Step(stepConf);
			if (GetAwakeCount(world) == 0)
			{
				break;
			}
			++loopsTillSleeping;
		}
	}

protected:
	World world{World::Def{}.UseGravity(Vec2(0, -10))};
	size_t loopsTillSleeping = 0;
	const size_t maxLoops = 10000;
	std::vector<Body*> boxes{10};
	RealNum original_x = 0;
	const RealNum hdim = 0.1f;
};

TEST_P(VerticalStackTest, EndsBeforeMaxLoops)
{
	EXPECT_LT(loopsTillSleeping, maxLoops);
}

TEST_P(VerticalStackTest, BoxesAtOriginalX)
{
	for (auto&& box: boxes)
	{
		EXPECT_EQ(box->GetLocation().x, original_x);
	}
}

TEST_P(VerticalStackTest, EachBoxAboveLast)
{
	auto lasty = RealNum{0};
	for (auto&& box: boxes)
	{
		EXPECT_GT(box->GetLocation().y, lasty + hdim);
		lasty = box->GetLocation().y;
	}
}

TEST_P(VerticalStackTest, EachBodyLevel)
{
	for (auto&& box: boxes)
	{
		EXPECT_EQ(box->GetAngle(), 0_deg);
	}
}

static std::string test_suffix_generator(::testing::TestParamInfo<RealNum> param_info)
{
	std::stringstream strbuf;
	strbuf << param_info.index;
	return strbuf.str();
}

extern ::testing::internal::ParamGenerator<VerticalStackTest::ParamType> gtest_WorldVerticalStackTest_EvalGenerator_();
extern ::std::string gtest_WorldVerticalStackTest_EvalGenerateName_(const ::testing::TestParamInfo<VerticalStackTest::ParamType>& info);

INSTANTIATE_TEST_CASE_P(World, VerticalStackTest, ::testing::Values(RealNum(0), RealNum(5)), test_suffix_generator);
