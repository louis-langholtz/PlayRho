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

#include <Box2D/Dynamics/Body.h>
#include <Box2D/Dynamics/Fixture.h>
#include <Box2D/Dynamics/World.h>
#include <Box2D/Dynamics/Contacts/Contact.h>
#include <Box2D/Dynamics/Joints/Joint.h>

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
	switch (bd.type)
	{
		case BodyType::Dynamic: flags |= (e_velocityFlag|e_accelerationFlag); break;
		case BodyType::Kinematic: flags |= e_velocityFlag; break;
		case BodyType::Static: break;
	}
	return flags;
}

Body::Body(const BodyDef* bd, World* world):
	m_flags{GetFlags(*bd)}, m_xf{bd->position, Rot{bd->angle}}, m_world{world},
	m_sweep{Sweep{Position{m_xf.p, bd->angle}, Position{m_xf.p, bd->angle}, Vec2_zero}},
	m_velocity{Velocity{bd->linearVelocity, bd->angularVelocity}},
	m_mass{(bd->type == BodyType::Dynamic)? float_t{1}: float_t{0}},
	m_invMass{(bd->type == BodyType::Dynamic)? float_t{1}: float_t{0}},
	m_linearDamping{bd->linearDamping}, m_angularDamping{bd->angularDamping},
	m_userData{bd->userData}
{
	assert(bd->position.IsValid());
	assert(bd->linearVelocity.IsValid());
	assert(IsValid(bd->angle));
	assert(IsValid(bd->angularVelocity));
	assert(IsValid(bd->angularDamping) && (bd->angularDamping >= float_t{0}));
	assert(IsValid(bd->linearDamping) && (bd->linearDamping >= float_t{0}));
}

Body::~Body()
{
	// shapes and joints are destroyed in World::Destroy
}

void Body::DestroyContacts()
{
	// Destroy the attached contacts.
	auto ce = m_contactList;
	while (ce)
	{
		const auto ce0 = ce;
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

	if (GetType() == type)
	{
		return;
	}

	switch (type)
	{
		case BodyType::Dynamic: m_flags |= (e_velocityFlag|e_accelerationFlag); break;
		case BodyType::Kinematic: m_flags |= e_velocityFlag; break;
		case BodyType::Static: break;
	}

	ResetMassData();

	if (GetType() == BodyType::Static)
	{
		m_velocity = Velocity{Vec2_zero, 0};
		m_sweep.pos0 = m_sweep.pos1;
		SynchronizeFixtures();
	}

	SetAwake();

	m_force = Vec2_zero;
	m_torque = float_t{0};

	DestroyContacts();

	// Touch the proxies so that new contacts will be created (when appropriate)
	auto& broadPhase = m_world->m_contactManager.m_broadPhase;
	for (auto f = m_fixtureList; f; f = f->m_next)
	{
		const auto proxyCount = f->m_proxyCount;
		for (auto i = decltype(proxyCount){0}; i < proxyCount; ++i)
		{
			broadPhase.TouchProxy(f->m_proxies[i].proxyId);
		}
	}
}

Fixture* Body::CreateFixture(const FixtureDef& def)
{
	assert(!m_world->IsLocked());
	if (m_world->IsLocked())
	{
		return nullptr;
	}

	const auto allocator = &m_world->m_blockAllocator;

	const auto memory = allocator->Allocate(sizeof(Fixture));
	const auto fixture = new (memory) Fixture(this);
	fixture->Create(allocator, def);

	if (IsActive())
	{
		fixture->CreateProxies(m_world->m_contactManager.m_broadPhase, m_xf);
	}

	fixture->m_next = m_fixtureList;
	m_fixtureList = fixture;

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

void Body::DestroyFixture(Fixture* fixture)
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
		auto node = &m_fixtureList;
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

	if (IsActive())
	{
		fixture->DestroyProxies(m_world->m_contactManager.m_broadPhase);
	}

	fixture->Destroy(allocator);
	fixture->m_next = nullptr;
	fixture->~Fixture();
	allocator->Free(fixture, sizeof(Fixture));

	// Reset the mass data.
	ResetMassData();
}

MassData Body::CalculateMassData() const noexcept
{
	auto mass = float_t{0};
	auto I = float_t{0};
	auto center = Vec2_zero;
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
	return MassData{mass, center, I};
}

Velocity Body::GetVelocity(float_t h, Vec2 gravity) const noexcept
{
	// Integrate velocity and apply damping.
	auto velocity = m_velocity;
	if (GetType() == BodyType::Dynamic)
	{
		// Integrate velocities.
		velocity.v += h * (gravity + (m_force * m_invMass));
		velocity.w += h * (m_torque * m_invI);
		
		// Apply damping.
		// ODE: dv/dt + c * v = 0
		// Solution: v(t) = v0 * exp(-c * t)
		// Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v * exp(-c * dt)
		// v2 = exp(-c * dt) * v1
		// Pade approximation:
		// v2 = v1 * 1 / (1 + c * dt)
		velocity.v *= float_t{1} / (float_t{1} + h * m_linearDamping);
		velocity.w *= float_t{1} / (float_t{1} + h * m_angularDamping);
	}
	return velocity;
}

void Body::ResetMassData()
{
	// Compute mass data from shapes. Each shape has its own density.

	// Static and kinematic bodies have zero mass.
	if (GetType() != BodyType::Dynamic)
	{
		m_mass = float_t{0};
		m_invMass = float_t{0};
		m_I = float_t{0};
		m_invI = float_t{0};

		m_sweep.localCenter = Vec2_zero;
		const auto position = Position{m_xf.p, m_sweep.pos1.a};
		m_sweep.pos0 = position;
		m_sweep.pos1 = position;
		return;
	}

	const auto massData = CalculateMassData();

	// Force all dynamic bodies to have a positive mass.
	m_mass = (massData.mass > float_t{0})? massData.mass: float_t{1};
	m_invMass = float_t{1} / m_mass;
	m_I = massData.I;
	
	// Compute center of mass.
	const auto localCenter = massData.center * m_invMass;
	
	if ((m_I > float_t{0}) && (!IsFixedRotation()))
	{
		// Center the inertia about the center of mass.
		m_I -= m_mass * localCenter.LengthSquared();
		assert(m_I > float_t{0});
		m_invI = float_t{1} / m_I;
	}
	else
	{
		m_I = float_t{0};
		m_invI = float_t{0};
	}

	// Move center of mass.
	const auto oldCenter = m_sweep.pos1.c;
	m_sweep.localCenter = localCenter;
	m_sweep.pos0.c = m_sweep.pos1.c = Mul(m_xf, m_sweep.localCenter);

	// Update center of mass velocity.
	m_velocity.v += Cross(m_velocity.w, m_sweep.pos1.c - oldCenter);
}

void Body::SetMassData(const MassData* massData)
{
	assert(!m_world->IsLocked());
	if (m_world->IsLocked())
	{
		return;
	}

	if (GetType() != BodyType::Dynamic)
	{
		return;
	}

	m_mass = (massData->mass > float_t(0))? massData->mass: float_t{1};
	m_invMass = float_t{1} / m_mass;

	if ((massData->I > float_t{0}) && (!IsFixedRotation()))
	{
		m_I = massData->I - m_mass * massData->center.LengthSquared();
		assert(m_I > float_t{0});
		m_invI = float_t{1} / m_I;
	}
	else
	{
		m_I = float_t{0};
		m_invI = float_t{0};
	}

	// Move center of mass.
	const auto oldCenter = m_sweep.pos1.c;
	m_sweep.localCenter =  massData->center;
	m_sweep.pos0.c = m_sweep.pos1.c = Mul(m_xf, m_sweep.localCenter);

	// Update center of mass velocity.
	m_velocity.v += Cross(m_velocity.w, m_sweep.pos1.c - oldCenter);
}

bool Body::ShouldCollide(const Body* other) const
{
	// At least one body should be dynamic.
	if ((GetType() != BodyType::Dynamic) && (other->GetType() != BodyType::Dynamic))
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

	{
		const auto center = Mul(m_xf, m_sweep.localCenter);
		const auto newPosition = Position{center, angle};
		m_sweep.pos1 = newPosition;
		m_sweep.pos0 = newPosition;
	}

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

	m_velocity.w = float_t{0};

	ResetMassData();
}

void Body::Dump()
{
	const auto bodyIndex = m_islandIndex;

	log("{\n");
	log("  BodyDef bd;\n");
	log("  bd.type = BodyType(%d);\n", GetType());
	log("  bd.position = Vec2(%.15lef, %.15lef);\n", m_xf.p.x, m_xf.p.y);
	log("  bd.angle = %.15lef;\n", m_sweep.pos1.a);
	log("  bd.linearVelocity = Vec2(%.15lef, %.15lef);\n", m_velocity.v.x, m_velocity.v.y);
	log("  bd.angularVelocity = %.15lef;\n", m_velocity.w);
	log("  bd.linearDamping = %.15lef;\n", m_linearDamping);
	log("  bd.angularDamping = %.15lef;\n", m_angularDamping);
	log("  bd.allowSleep = bool(%d);\n", m_flags & e_autoSleepFlag);
	log("  bd.awake = bool(%d);\n", m_flags & e_awakeFlag);
	log("  bd.fixedRotation = bool(%d);\n", m_flags & e_fixedRotationFlag);
	log("  bd.bullet = bool(%d);\n", m_flags & e_bulletFlag);
	log("  bd.active = bool(%d);\n", m_flags & e_activeFlag);
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
