/*
* Original work Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
* Modified work Copyright (c) 2016 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

#include <Box2D/Dynamics/Body.h>
#include <Box2D/Dynamics/Fixture.h>
#include <Box2D/Dynamics/World.h>
#include <Box2D/Dynamics/Contacts/Contact.h>
#include <Box2D/Dynamics/Joints/Joint.h>
#include <Box2D/Collision/Shapes/CircleShape.h>
#include <Box2D/Collision/Shapes/EdgeShape.h>
#include <Box2D/Collision/Shapes/PolygonShape.h>
#include <Box2D/Collision/Shapes/ChainShape.h>

using namespace box2d;

template <>
inline void box2d::Delete(Shape* shape, BlockAllocator& allocator)
{
	switch (shape->GetType())
	{
		case Shape::e_circle:
			Delete(static_cast<CircleShape*>(shape), allocator);
			break;
		case Shape::e_edge:
			Delete(static_cast<EdgeShape*>(shape), allocator);
			break;
		case Shape::e_polygon:
			Delete(static_cast<PolygonShape*>(shape), allocator);
			break;
		case Shape::e_chain:
			Delete(static_cast<ChainShape*>(shape), allocator);
			break;
		default:
			assert(false);
			break;
	}
}

uint16 Body::GetFlags(const BodyDef& bd) noexcept
{
	uint16 flags = 0;
	if (bd.bullet)
	{
		flags |= e_impenetrableFlag;
	}
	if (bd.fixedRotation)
	{
		flags |= e_fixedRotationFlag;
	}
	if (bd.allowSleep)
	{
		flags |= e_autoSleepFlag;
	}
	if (bd.awake)
	{
		flags |= e_awakeFlag;
	}
	if (bd.active)
	{
		flags |= e_activeFlag;
	}
	switch (bd.type)
	{
		case BodyType::Dynamic:   flags |= (e_velocityFlag|e_accelerationFlag); break;
		case BodyType::Kinematic: flags |= (e_impenetrableFlag|e_velocityFlag); break;
		case BodyType::Static:    flags |= (e_impenetrableFlag); break;
	}
	return flags;
}

Body::Body(const BodyDef& bd, World* world):
	m_flags{GetFlags(bd)},
	m_xf{bd.position, Rot{bd.angle}},
	m_world{world},
	m_sweep{Position{bd.position, bd.angle}},
	m_velocity{Velocity{bd.linearVelocity, bd.angularVelocity}},
	m_invMass{(bd.type == BodyType::Dynamic)? float_t{1}: float_t{0}},
	m_linearDamping{bd.linearDamping},
	m_angularDamping{bd.angularDamping},
	m_userData{bd.userData}
{
	assert(IsValid(bd.position));
	assert(IsValid(bd.linearVelocity));
	assert(IsValid(bd.angle));
	assert(IsValid(bd.angularVelocity));
	assert(IsValid(bd.angularDamping) && (bd.angularDamping >= float_t{0}));
	assert(IsValid(bd.linearDamping) && (bd.linearDamping >= float_t{0}));
}

Body::~Body()
{
	DestroyJoints();
	DestroyContacts();
	DestroyFixtures();
}

void Body::DestroyContacts()
{
	// Destroy the attached contacts.
	while (!m_contacts.empty())
	{
		auto& ce = m_contacts.front();
		m_contacts.pop_front();
		m_world->m_contactMgr.Destroy(ce.contact);
	}
}

void Body::DestroyJoints()
{
	// Delete the attached joints.
	while (!m_joints.empty())
	{
		auto& je = m_joints.front();
		m_joints.pop_front();
		if (m_world->m_destructionListener)
		{
			m_world->m_destructionListener->SayGoodbye(*(je.joint));
		}
		
		m_world->Destroy(je.joint);
	}
}

void Body::DestroyFixtures()
{
	// Delete the attached fixtures. This destroys broad-phase proxies.
	while (!m_fixtures.empty())
	{
		auto& fixture = m_fixtures.front();
		m_fixtures.pop_front();
		
		if (m_world->m_destructionListener)
		{
			m_world->m_destructionListener->SayGoodbye(fixture);
		}
		
		fixture.DestroyProxies(m_world->m_blockAllocator, m_world->m_contactMgr.m_broadPhase);
		const auto shape = fixture.GetShape();
		Delete(&fixture, m_world->m_blockAllocator);
		Delete(shape, m_world->m_blockAllocator);
	}
}

void Body::SetType(BodyType type)
{
	assert(!m_world->IsLocked());
	if (m_world->IsLocked())
	{
		return;
	}

	if (GetType() == type)
	{
		return;
	}

	m_flags &= ~(e_impenetrableFlag|e_velocityFlag|e_accelerationFlag);
	switch (type)
	{
		case BodyType::Dynamic:   m_flags |= (e_velocityFlag|e_accelerationFlag); break;
		case BodyType::Kinematic: m_flags |= (e_impenetrableFlag|e_velocityFlag); break;
		case BodyType::Static:    m_flags |= (e_impenetrableFlag); break;
	}

	ResetMassData();

	if (type == BodyType::Static)
	{
		m_velocity = Velocity{Vec2_zero, 0};
		m_sweep.pos0 = m_sweep.pos1;
		SynchronizeFixtures();
	}

	SetAwake();

	m_linearAcceleration = Vec2_zero;
	m_angularAcceleration = float_t{0};
	if (IsAccelerable())
	{
		m_linearAcceleration += m_world->GetGravity();
	}

	DestroyContacts();

	auto& broadPhase = m_world->m_contactMgr.m_broadPhase;
	for (auto&& fixture: GetFixtures())
	{
		fixture.TouchProxies(broadPhase);
	}
}

Fixture* Body::CreateFixture(const FixtureDef& def, bool resetMassData)
{
	assert(!m_world->IsLocked());
	if (m_world->IsLocked())
	{
		return nullptr;
	}

	auto& allocator = m_world->m_blockAllocator;

	const auto shape = def.shape->Clone(&allocator);
	const auto memory = allocator.Allocate(sizeof(Fixture));
	const auto fixture = new (memory) Fixture{this, def, shape};
	
	if (IsActive())
	{
		fixture->CreateProxies(allocator, m_world->m_contactMgr.m_broadPhase, GetTransformation());
	}

	m_fixtures.push_front(fixture);

	// Adjust mass properties if needed.
	if (fixture->GetDensity() > float_t{0})
	{
		SetMassDataDirty();
		if (resetMassData)
		{
			ResetMassData();
		}
	}

	// Let the world know we have a new fixture. This will cause new contacts
	// to be created at the beginning of the next time step.
	m_world->SetNewFixtures();

	return fixture;
}

void Body::DestroyFixture(Fixture* fixture, bool resetMassData)
{
	assert(!m_world->IsLocked());
	if (m_world->IsLocked())
	{
		return;
	}

	assert(fixture->m_body == this);

	// Remove the fixture from this body's singly linked list.
	auto found = false;
	{
		for (auto it = m_fixtures.begin(); it != m_fixtures.end(); ++it)
		{
			if (&(*it) == fixture)
			{
				m_fixtures.erase(it);
				found = true;
				break;
			}
		}
	}

	// You tried to remove a shape that is not attached to this body.
	assert(found);

	// Destroy any contacts associated with the fixture.
	auto edge = m_contacts.p;
	while (edge)
	{
		auto c = edge->contact;
		edge = edge->next;

		const auto fixtureA = c->GetFixtureA();
		const auto fixtureB = c->GetFixtureB();

		if ((fixture == fixtureA) || (fixture == fixtureB))
		{
			// This destroys the contact and removes it from
			// this body's contact list.
			m_world->m_contactMgr.Destroy(c);
		}
	}

	fixture->DestroyProxies(m_world->m_blockAllocator, m_world->m_contactMgr.m_broadPhase);

	fixture->m_next = nullptr;
	
	const auto shape = fixture->GetShape();
	Delete(fixture, m_world->m_blockAllocator);
	Delete(shape, m_world->m_blockAllocator);
	
	SetMassDataDirty();		
	if (resetMassData)
	{
		ResetMassData();
	}
}

MassData Body::ComputeMassData() const noexcept
{
	auto mass = float_t{0};
	auto I = float_t{0};
	auto center = Vec2_zero;
	for (auto&& fixture: GetFixtures())
	{
		if (fixture.GetDensity() != float_t{0})
		{
			const auto massData = fixture.ComputeMassData();
			mass += massData.mass;
			center += massData.mass * massData.center;
			I += massData.I;
		}
	}
	return MassData{mass, center, I};
}

void Body::ResetMassData()
{
	// Compute mass data from shapes. Each shape has its own density.

	// Non-dynamic bodies (Static and kinematic ones) have zero mass.
	if (!IsAccelerable())
	{
		m_invMass = float_t{0};
		m_invI = float_t{0};
		m_sweep = Sweep{Position{GetPosition(), GetAngle()}};
		UnsetMassDataDirty();
		return;
	}

	const auto massData = ComputeMassData();

	// Force all dynamic bodies to have a positive mass.
	const auto mass = (massData.mass > float_t{0})? massData.mass: float_t{1};
	m_invMass = float_t{1} / mass;
	
	// Compute center of mass.
	const auto localCenter = massData.center * m_invMass;
	
	const auto I = massData.I;
	if ((I > float_t{0}) && (!IsFixedRotation()))
	{
		// Center the inertia about the center of mass.
		assert((I - mass * LengthSquared(localCenter)) > float_t{0});
		m_invI = float_t{1} / (I - mass * LengthSquared(localCenter));
	}
	else
	{
		m_invI = float_t{0};
	}

	// Move center of mass.
	const auto oldCenter = GetWorldCenter();
	m_sweep = Sweep{Position{Transform(localCenter, GetTransformation()), GetAngle()}, localCenter};

	// Update center of mass velocity.
	m_velocity.v += GetReversePerpendicular(GetWorldCenter() - oldCenter) * m_velocity.w;
	
	UnsetMassDataDirty();
}

void Body::SetMassData(const MassData& massData)
{
	assert(!m_world->IsLocked());
	if (m_world->IsLocked())
	{
		return;
	}

	if (!IsAccelerable())
	{
		return;
	}

	const auto mass = (massData.mass > float_t(0))? massData.mass: float_t{1};
	m_invMass = float_t{1} / mass;

	if ((massData.I > float_t{0}) && (!IsFixedRotation()))
	{
		const auto I = massData.I - mass * LengthSquared(massData.center);
		assert(I > float_t{0});
		m_invI = float_t{1} / I;
	}
	else
	{
		m_invI = float_t{0};
	}

	// Move center of mass.
	const auto oldCenter = GetWorldCenter();

	m_sweep = Sweep{Position{Transform(massData.center, GetTransformation()), GetAngle()}, massData.center};

	// Update center of mass velocity.
	m_velocity.v += GetReversePerpendicular(GetWorldCenter() - oldCenter) * m_velocity.w;
	
	UnsetMassDataDirty();
}

void Body::SetVelocity(const Velocity& velocity) noexcept
{
	if ((velocity.v != Vec2_zero) || (velocity.w != 0))
	{
		if (!IsSpeedable())
		{
			return;
		}
		SetAwake();
	}
	m_velocity = velocity;
}

void Body::SetAcceleration(const Vec2& linear, const float_t angular) noexcept
{
	assert(IsValid(linear));
	assert(IsValid(angular));

	if ((linear != Vec2_zero) || (angular != 0))
	{
		if (!IsAccelerable())
		{
			return;
		}
	}
	m_linearAcceleration = linear;
	m_angularAcceleration = angular;
}

bool Body::ShouldCollide(const Body* other) const
{
	// At least one body should be accelerable/dynamic.
	if (!IsAccelerable() && !other->IsAccelerable())
	{
		return false;
	}

	// Does a joint prevent collision?
	for (auto&& jn: m_joints)
	{
		if (jn.other == other)
		{
			if (!jn.joint->m_collideConnected)
			{
				return false;
			}
		}
	}

	return true;
}

void Body::SynchronizeFixtures(const Transformation& t1, const Transformation& t2)
{
	auto& broadPhase = m_world->m_contactMgr.m_broadPhase;
	for (auto&& fixture: GetFixtures())
	{
		fixture.Synchronize(broadPhase, t1, t2);
	}
}

void Body::SetTransform(const Vec2& position, float_t angle)
{
	assert(IsValid(position));
	assert(IsValid(angle));

	assert(!m_world->IsLocked());
	if (m_world->IsLocked())
	{
		return;
	}

	const auto xf = Transformation{position, Rot(angle)};
	m_xf = xf;
	m_sweep = Sweep{Position{Transform(GetLocalCenter(), xf), angle}, GetLocalCenter()};
	SynchronizeFixtures(xf, xf);
}

void Body::SynchronizeFixtures()
{
	SynchronizeFixtures(GetTransform0(m_sweep), GetTransformation());
}

void Body::SetActive(bool flag)
{
	assert(!m_world->IsLocked());

	if (flag == IsActive())
	{
		return;
	}

	if (flag)
	{
		m_flags |= e_activeFlag;

		// Create all proxies.
		auto& broadPhase = m_world->m_contactMgr.m_broadPhase;
		auto& allocator = m_world->m_blockAllocator;
		const auto xf = GetTransformation();
		for (auto&& fixture: GetFixtures())
		{
			fixture.CreateProxies(allocator, broadPhase, xf);
		}

		// Contacts are created the next time step.
	}
	else
	{
		m_flags &= ~e_activeFlag;

		// Destroy all proxies.
		auto& broadPhase = m_world->m_contactMgr.m_broadPhase;
		auto& allocator = m_world->m_blockAllocator;
		for (auto&& fixture: GetFixtures())
		{
			fixture.DestroyProxies(allocator, broadPhase);
		}

		DestroyContacts();
	}
}

void Body::SetFixedRotation(bool flag)
{
	const auto status = IsFixedRotation();
	if (status == flag)
	{
		return;
	}

	if (flag)
	{
		m_flags |= e_fixedRotationFlag;
	}
	else
	{
		m_flags &= ~e_fixedRotationFlag;
	}

	m_velocity.w = float_t{0};

	ResetMassData();
}

void Body::Dump()
{
	const auto bodyIndex = m_islandIndex;

	log("{\n");
	log("  BodyDef bd;\n");
	log("  bd.type = BodyType(%d);\n", GetType());
	log("  bd.position = Vec2(%.15lef, %.15lef);\n", GetPosition().x, GetPosition().y);
	log("  bd.angle = %.15lef;\n", GetAngle());
	log("  bd.linearVelocity = Vec2(%.15lef, %.15lef);\n", GetVelocity().v.x, GetVelocity().v.y);
	log("  bd.angularVelocity = %.15lef;\n", GetVelocity().w);
	log("  bd.linearDamping = %.15lef;\n", GetLinearDamping());
	log("  bd.angularDamping = %.15lef;\n", GetAngularDamping());
	log("  bd.allowSleep = bool(%d);\n", IsSleepingAllowed());
	log("  bd.awake = bool(%d);\n", IsAwake());
	log("  bd.fixedRotation = bool(%d);\n", IsFixedRotation());
	log("  bd.bullet = bool(%d);\n", IsImpenetrable());
	log("  bd.active = bool(%d);\n", IsActive());
	log("  bodies[%d] = m_world->CreateBody(bd);\n", m_islandIndex);
	log("\n");
	for (auto&& fixture: GetFixtures())
	{
		log("  {\n");
		fixture.Dump(bodyIndex);
		log("  }\n");
	}
	log("}\n");
}

Velocity box2d::GetVelocity(const Body& body, float_t h) noexcept
{
	assert(IsValid(h));

	// Integrate velocity and apply damping.
	auto velocity = body.GetVelocity();
	if (body.IsAccelerable())
	{
		// Integrate velocities.
		velocity.v += h * body.GetLinearAcceleration();
		velocity.w += h * body.GetAngularAcceleration();
		
		// Apply damping.
		// ODE: dv/dt + c * v = 0
		// Solution: v(t) = v0 * exp(-c * t)
		// Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v * exp(-c * dt)
		// v2 = exp(-c * dt) * v1
		// Pade approximation:
		// v2 = v1 * 1 / (1 + c * dt)
		velocity.v *= float_t{1} / (float_t{1} + h * body.GetLinearDamping());
		velocity.w *= float_t{1} / (float_t{1} + h * body.GetAngularDamping());
	}
	return velocity;
}
