/*
* Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
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

#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Dynamics/b2Fixture.h>
#include <Box2D/Dynamics/b2World.h>
#include <Box2D/Dynamics/Contacts/b2Contact.h>
#include <Box2D/Dynamics/Joints/b2Joint.h>

using namespace box2d;

uint16 Body::GetFlags(const BodyDef& bd) noexcept
{
	uint16 flags = 0;
	if (bd.bullet)
	{
		flags |= e_bulletFlag;
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
	return flags;
}

Body::Body(const BodyDef* bd, b2World* world):
	m_type(bd->type), m_flags(GetFlags(*bd)), m_xf(bd->position, Rot(bd->angle)), m_world(world),
	m_linearVelocity(bd->linearVelocity), m_angularVelocity(bd->angularVelocity)
{
	assert(bd->position.IsValid());
	assert(bd->linearVelocity.IsValid());
	assert(IsValid(bd->angle));
	assert(IsValid(bd->angularVelocity));
	assert(IsValid(bd->angularDamping) && (bd->angularDamping >= float_t{0}));
	assert(IsValid(bd->linearDamping) && (bd->linearDamping >= float_t{0}));

	m_sweep.localCenter = Vec2_zero;
	m_sweep.c0 = m_xf.p;
	m_sweep.c = m_xf.p;
	m_sweep.a0 = bd->angle;
	m_sweep.a = bd->angle;
	m_sweep.alpha0 = float_t{0};

	m_linearDamping = bd->linearDamping;
	m_angularDamping = bd->angularDamping;
	m_gravityScale = bd->gravityScale;

	if (m_type == DynamicBody)
	{
		m_mass = float_t{1};
		m_invMass = float_t{1};
	}
	else
	{
		m_mass = float_t{0};
		m_invMass = float_t{0};
	}

	m_userData = bd->userData;
}

Body::~Body()
{
	// shapes and joints are destroyed in b2World::Destroy
}

void Body::DestroyContacts()
{
	// Destroy the attached contacts.
	auto ce = m_contactList;
	while (ce)
	{
		auto ce0 = ce;
		ce = ce->next;
		m_world->m_contactManager.Destroy(ce0->contact);
	}
	m_contactList = nullptr;
}

void Body::SetType(BodyType type)
{
	assert(!m_world->IsLocked());
	if (m_world->IsLocked())
	{
		return;
	}

	if (m_type == type)
	{
		return;
	}

	m_type = type;

	ResetMassData();

	if (m_type == StaticBody)
	{
		m_linearVelocity = Vec2_zero;
		m_angularVelocity = float_t{0};
		m_sweep.a0 = m_sweep.a;
		m_sweep.c0 = m_sweep.c;
		SynchronizeFixtures();
	}

	SetAwake();

	m_force = Vec2_zero;
	m_torque = float_t{0};

	DestroyContacts();

	// Touch the proxies so that new contacts will be created (when appropriate)
	auto broadPhase = &m_world->m_contactManager.m_broadPhase;
	for (auto f = m_fixtureList; f; f = f->m_next)
	{
		const auto proxyCount = f->m_proxyCount;
		for (auto i = decltype(proxyCount){0}; i < proxyCount; ++i)
		{
			broadPhase->TouchProxy(f->m_proxies[i].proxyId);
		}
	}
}

Fixture* Body::CreateFixture(const FixtureDef* def)
{
	assert(!m_world->IsLocked());
	if (m_world->IsLocked())
	{
		return nullptr;
	}

	auto allocator = &m_world->m_blockAllocator;

	auto memory = allocator->Allocate(sizeof(Fixture));
	auto fixture = new (memory) Fixture(this);
	fixture->Create(allocator, def);

	if (m_flags & e_activeFlag)
	{
		fixture->CreateProxies(m_world->m_contactManager.m_broadPhase, m_xf);
	}

	fixture->m_next = m_fixtureList;
	m_fixtureList = fixture;
	++m_fixtureCount;

	// Adjust mass properties if needed.
	if (fixture->m_density > float_t{0})
	{
		ResetMassData();
	}

	// Let the world know we have a new fixture. This will cause new contacts
	// to be created at the beginning of the next time step.
	m_world->SetNewFixtures();

	return fixture;
}

Fixture* Body::CreateFixture(const b2Shape* shape, float_t density)
{
	FixtureDef def;
	def.shape = shape;
	def.density = density;

	return CreateFixture(&def);
}

void Body::DestroyFixture(Fixture* fixture)
{
	assert(!m_world->IsLocked());
	if (m_world->IsLocked())
	{
		return;
	}

	assert(fixture->m_body == this);

	// Remove the fixture from this body's singly linked list.
	assert(m_fixtureCount > 0);
	auto node = &m_fixtureList;
	auto found = false;
	while (*node != nullptr)
	{
		if (*node == fixture)
		{
			*node = fixture->m_next;
			found = true;
			break;
		}

		node = &(*node)->m_next;
	}

	// You tried to remove a shape that is not attached to this body.
	assert(found);

	// Destroy any contacts associated with the fixture.
	auto edge = m_contactList;
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
			m_world->m_contactManager.Destroy(c);
		}
	}

	auto allocator = &m_world->m_blockAllocator;

	if (m_flags & e_activeFlag)
	{
		fixture->DestroyProxies(m_world->m_contactManager.m_broadPhase);
	}

	fixture->Destroy(allocator);
	fixture->m_next = nullptr;
	fixture->~Fixture();
	allocator->Free(fixture, sizeof(Fixture));

	--m_fixtureCount;

	// Reset the mass data.
	ResetMassData();
}

b2MassData Body::CalculateMassData() const noexcept
{
	auto mass = float_t(0);
	auto center = Vec2_zero;
	auto I = float_t(0);
	for (auto f = m_fixtureList; f; f = f->m_next)
	{
		if (f->m_density == float_t{0})
		{
			continue;
		}
		
		const auto massData = f->GetMassData();
		mass += massData.mass;
		center += massData.mass * massData.center;
		I += massData.I;
	}
	return b2MassData{mass, (mass != float_t(0))? center / mass: Vec2_zero, I};
}

void Body::ResetMassData()
{
	// Compute mass data from shapes. Each shape has its own density.

	// Static and kinematic bodies have zero mass.
	if ((m_type == StaticBody) || (m_type == KinematicBody))
	{
		m_mass = float_t{0};
		m_invMass = float_t{0};
		m_I = float_t{0};
		m_invI = float_t{0};
		m_sweep.localCenter = Vec2_zero;
		m_sweep.c0 = m_xf.p;
		m_sweep.c = m_xf.p;
		m_sweep.a0 = m_sweep.a;
		return;
	}

	assert(m_type == DynamicBody);

	// Accumulate mass over all fixtures.
	m_mass = float_t{0};
	m_I = float_t{0};
	auto localCenter = Vec2_zero;
	for (auto f = m_fixtureList; f; f = f->m_next)
	{
		if (f->m_density == float_t{0})
		{
			continue;
		}

		const auto massData = f->GetMassData();
		m_mass += massData.mass;
		localCenter += massData.mass * massData.center;
		m_I += massData.I;
	}

	// Compute center of mass.
	if (m_mass > float_t{0})
	{
		m_invMass = float_t(1) / m_mass;
		localCenter *= m_invMass;
	}
	else
	{
		// Force all dynamic bodies to have a positive mass.
		m_mass = float_t(1);
		m_invMass = float_t(1);
	}

	if ((m_I > float_t{0}) && (!IsFixedRotation()))
	{
		// Center the inertia about the center of mass.
		m_I -= m_mass * localCenter.LengthSquared();
		assert(m_I > float_t{0});
		m_invI = float_t(1) / m_I;
	}
	else
	{
		m_I = float_t{0};
		m_invI = float_t{0};
	}

	// Move center of mass.
	const auto oldCenter = m_sweep.c;
	m_sweep.localCenter = localCenter;
	m_sweep.c0 = m_sweep.c = Mul(m_xf, m_sweep.localCenter);

	// Update center of mass velocity.
	m_linearVelocity += Cross(m_angularVelocity, m_sweep.c - oldCenter);
}

void Body::SetMassData(const b2MassData* massData)
{
	assert(!m_world->IsLocked());
	if (m_world->IsLocked())
	{
		return;
	}

	if (m_type != DynamicBody)
	{
		return;
	}

	m_mass = (massData->mass > float_t(0))? massData->mass: float_t(1);
	m_invMass = float_t(1) / m_mass;

	if ((massData->I > float_t{0}) && (!IsFixedRotation()))
	{
		m_I = massData->I - m_mass * massData->center.LengthSquared();
		assert(m_I > float_t{0});
		m_invI = float_t(1) / m_I;
	}
	else
	{
		m_I = float_t{0};
		m_invI = float_t{0};
	}

	// Move center of mass.
	const auto oldCenter = m_sweep.c;
	m_sweep.localCenter =  massData->center;
	m_sweep.c0 = m_sweep.c = Mul(m_xf, m_sweep.localCenter);

	// Update center of mass velocity.
	m_linearVelocity += Cross(m_angularVelocity, m_sweep.c - oldCenter);
}

bool Body::ShouldCollide(const Body* other) const
{
	// At least one body should be dynamic.
	if ((m_type != DynamicBody) && (other->m_type != DynamicBody))
	{
		return false;
	}

	// Does a joint prevent collision?
	for (auto jn = m_jointList; jn; jn = jn->next)
	{
		if (jn->other == other)
		{
			if (!jn->joint->m_collideConnected)
			{
				return false;
			}
		}
	}

	return true;
}

void Body::SetTransform(const Vec2& position, float_t angle)
{
	assert(!m_world->IsLocked());
	if (m_world->IsLocked())
	{
		return;
	}

	m_xf = Transform{position, Rot(angle)};

	m_sweep.c = Mul(m_xf, m_sweep.localCenter);
	m_sweep.a = angle;

	m_sweep.c0 = m_sweep.c;
	m_sweep.a0 = angle;

	auto& broadPhase = m_world->m_contactManager.m_broadPhase;
	for (auto f = m_fixtureList; f; f = f->m_next)
	{
		f->Synchronize(broadPhase, m_xf, m_xf);
	}
}

void Body::SynchronizeFixtures()
{
	const auto xf1 = GetTransformZero(m_sweep);
	auto& broadPhase = m_world->m_contactManager.m_broadPhase;
	for (auto f = m_fixtureList; f; f = f->m_next)
	{
		f->Synchronize(broadPhase, xf1, m_xf);
	}
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
		auto& broadPhase = m_world->m_contactManager.m_broadPhase;
		for (auto f = m_fixtureList; f; f = f->m_next)
		{
			f->CreateProxies(broadPhase, m_xf);
		}

		// Contacts are created the next time step.
	}
	else
	{
		m_flags &= ~e_activeFlag;

		// Destroy all proxies.
		auto& broadPhase = m_world->m_contactManager.m_broadPhase;
		for (auto f = m_fixtureList; f; f = f->m_next)
		{
			f->DestroyProxies(broadPhase);
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

	m_angularVelocity = float_t{0};

	ResetMassData();
}

void Body::Dump()
{
	const auto bodyIndex = m_islandIndex;

	log("{\n");
	log("  BodyDef bd;\n");
	log("  bd.type = BodyType(%d);\n", m_type);
	log("  bd.position = Vec2(%.15lef, %.15lef);\n", m_xf.p.x, m_xf.p.y);
	log("  bd.angle = %.15lef;\n", m_sweep.a);
	log("  bd.linearVelocity = Vec2(%.15lef, %.15lef);\n", m_linearVelocity.x, m_linearVelocity.y);
	log("  bd.angularVelocity = %.15lef;\n", m_angularVelocity);
	log("  bd.linearDamping = %.15lef;\n", m_linearDamping);
	log("  bd.angularDamping = %.15lef;\n", m_angularDamping);
	log("  bd.allowSleep = bool(%d);\n", m_flags & e_autoSleepFlag);
	log("  bd.awake = bool(%d);\n", m_flags & e_awakeFlag);
	log("  bd.fixedRotation = bool(%d);\n", m_flags & e_fixedRotationFlag);
	log("  bd.bullet = bool(%d);\n", m_flags & e_bulletFlag);
	log("  bd.active = bool(%d);\n", m_flags & e_activeFlag);
	log("  bd.gravityScale = %.15lef;\n", m_gravityScale);
	log("  bodies[%d] = m_world->CreateBody(&bd);\n", m_islandIndex);
	log("\n");
	for (auto f = m_fixtureList; f; f = f->m_next)
	{
		log("  {\n");
		f->Dump(bodyIndex);
		log("  }\n");
	}
	log("}\n");
}
