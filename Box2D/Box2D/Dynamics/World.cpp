/*
* Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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

#include <Box2D/Dynamics/World.h>
#include <Box2D/Dynamics/Body.h>
#include <Box2D/Dynamics/Fixture.h>
#include <Box2D/Dynamics/Island.h>
#include <Box2D/Dynamics/Joints/PulleyJoint.h>
#include <Box2D/Dynamics/Contacts/Contact.h>
#include <Box2D/Collision/Collision.h>
#include <Box2D/Collision/BroadPhase.h>
#include <Box2D/Collision/Shapes/CircleShape.h>
#include <Box2D/Collision/Shapes/EdgeShape.h>
#include <Box2D/Collision/Shapes/ChainShape.h>
#include <Box2D/Collision/Shapes/PolygonShape.h>
#include <Box2D/Common/Draw.h>
#include <Box2D/Common/Timer.h>

#include <new>
#include <functional>
#include <type_traits>
#include <memory>

namespace box2d
{

template <typename T>
class FlagGuard
{
public:
	FlagGuard(T& flag, T value) : m_flag(flag), m_value(value)
	{
		static_assert(std::is_unsigned<T>::value, "Unsigned integer required");
		m_flag |= m_value;
	}

	~FlagGuard()
	{
		m_flag &= ~m_value;
	}

	FlagGuard() = delete;

private:
	T& m_flag;
	T m_value;
};

template <class T>
class RaiiWrapper
{
public:
	RaiiWrapper() = delete;
	RaiiWrapper(std::function<void(T&)> on_destruction): m_on_destruction(on_destruction) {}
	~RaiiWrapper() { m_on_destruction(m_wrapped); }
	T m_wrapped;

private:
	std::function<void(T&)> m_on_destruction;
};

World::World(const Vec2& gravity): m_gravity(gravity)
{
	memset(&m_profile, 0, sizeof(Profile));
}

World::~World()
{
	// Some shapes allocate using alloc.
	for (auto&& b: m_bodies)
	{
		auto& fixtures = b.m_fixtures;
		while (fixtures)
		{
			auto& f = fixtures.front();
			fixtures.pop_front();
			f.m_proxyCount = 0;
			f.Destroy(&m_blockAllocator);
		}
	}
}

void World::SetDestructionListener(DestructionListener* listener) noexcept
{
	m_destructionListener = listener;
}

void World::SetContactFilter(ContactFilter* filter) noexcept
{
	m_contactManager.m_contactFilter = filter;
}

void World::SetContactListener(ContactListener* listener) noexcept
{
	m_contactManager.m_contactListener = listener;
}

void World::SetDebugDraw(Draw* debugDraw) noexcept
{
	g_debugDraw = debugDraw;
}

Body* World::CreateBody(const BodyDef* def)
{
	assert(!IsLocked());
	if (IsLocked())
	{
		return nullptr;
	}

	void* mem = m_blockAllocator.Allocate(sizeof(Body));
	auto b = new (mem) Body(def, this);
	if (b)
	{
		if (!Add(*b))
		{
			b->~Body();
			m_blockAllocator.Free(b, sizeof(Body));
			return nullptr;
		}		
	}

	return b;
}

bool World::Add(Body& b)
{
	assert(!b.m_prev);
	assert(!b.m_next);

	assert(m_bodies.size() < m_bodies.max_size());
	if (m_bodies.size() >= m_bodies.max_size())
	{
		return false;
	}
	
	// Add to world doubly linked list.
	m_bodies.push_front(&b);
	return true;
}

bool World::Remove(Body& b)
{
	assert(!m_bodies.empty());
	if (m_bodies.empty())
	{
		return false;
	}

	m_bodies.erase(BodyIterator{&b});
	return true;
}

void World::DestroyBody(Body* b)
{
	assert(b->m_world == this);
	
	assert(!IsLocked());
	if (IsLocked())
	{
		return;
	}
	
	Remove(*b);
	
	b->~Body();
	m_blockAllocator.Free(b, sizeof(Body));
}

Joint* World::CreateJoint(const JointDef* def)
{
	assert(def != nullptr);

	assert(!IsLocked());
	if (IsLocked())
	{
		return nullptr;
	}

	auto j = Joint::Create(*def, &m_blockAllocator);

	// Connect to the world list.
	j->m_prev = nullptr;
	j->m_next = m_joints;
	if (m_joints)
	{
		m_joints->m_prev = j;
	}
	m_joints = j;
	++m_jointCount;

	// Connect to the bodies' doubly linked lists.
	j->m_edgeA.joint = j;
	j->m_edgeA.other = j->m_bodyB;
	j->m_edgeA.prev = nullptr;
	j->m_edgeA.next = j->m_bodyA->m_joints;
	if (j->m_bodyA->m_joints) j->m_bodyA->m_joints->prev = &j->m_edgeA;
	j->m_bodyA->m_joints = &j->m_edgeA;

	j->m_edgeB.joint = j;
	j->m_edgeB.other = j->m_bodyA;
	j->m_edgeB.prev = nullptr;
	j->m_edgeB.next = j->m_bodyB->m_joints;
	if (j->m_bodyB->m_joints) j->m_bodyB->m_joints->prev = &j->m_edgeB;
	j->m_bodyB->m_joints = &j->m_edgeB;

	auto bodyA = def->bodyA;
	auto bodyB = def->bodyB;

	// If the joint prevents collisions, then flag any contacts for filtering.
	if (!def->collideConnected)
	{
		auto edge = bodyB->GetContactList();
		while (edge)
		{
			if (edge->other == bodyA)
			{
				// Flag the contact for filtering at the next time step (where either
				// body is awake).
				edge->contact->FlagForFiltering();
			}

			edge = edge->next;
		}
	}

	// Note: creating a joint doesn't wake the bodies.

	return j;
}

void World::DestroyJoint(Joint* j)
{
	assert(!IsLocked());
	if (IsLocked())
	{
		return;
	}

	const auto collideConnected = j->m_collideConnected;

	// Remove from the doubly linked list.
	if (j->m_prev)
	{
		j->m_prev->m_next = j->m_next;
	}

	if (j->m_next)
	{
		j->m_next->m_prev = j->m_prev;
	}

	if (j == m_joints)
	{
		m_joints = j->m_next;
	}

	// Disconnect from island graph.
	auto bodyA = j->m_bodyA;
	auto bodyB = j->m_bodyB;

	// Wake up connected bodies.
	bodyA->SetAwake();
	bodyB->SetAwake();

	// Remove from body 1.
	if (j->m_edgeA.prev)
	{
		j->m_edgeA.prev->next = j->m_edgeA.next;
	}

	if (j->m_edgeA.next)
	{
		j->m_edgeA.next->prev = j->m_edgeA.prev;
	}

	if (&j->m_edgeA == bodyA->m_joints)
	{
		bodyA->m_joints = j->m_edgeA.next;
	}

	j->m_edgeA.prev = nullptr;
	j->m_edgeA.next = nullptr;

	// Remove from body 2
	if (j->m_edgeB.prev)
	{
		j->m_edgeB.prev->next = j->m_edgeB.next;
	}

	if (j->m_edgeB.next)
	{
		j->m_edgeB.next->prev = j->m_edgeB.prev;
	}

	if (&j->m_edgeB == bodyB->m_joints)
	{
		bodyB->m_joints = j->m_edgeB.next;
	}

	j->m_edgeB.prev = nullptr;
	j->m_edgeB.next = nullptr;

	Joint::Destroy(j, &m_blockAllocator);

	assert(m_jointCount > 0);
	--m_jointCount;

	// If the joint prevents collisions, then flag any contacts for filtering.
	if (!collideConnected)
	{
		auto edge = bodyB->GetContactList();
		while (edge)
		{
			if (edge->other == bodyA)
			{
				// Flag the contact for filtering at the next time step (where either
				// body is awake).
				edge->contact->FlagForFiltering();
			}

			edge = edge->next;
		}
	}
}

//
void World::SetAllowSleeping(bool flag) noexcept
{
	if (flag == m_allowSleep)
	{
		return;
	}

	m_allowSleep = flag;
	if (!m_allowSleep)
	{
		for (auto&& b: m_bodies)
		{
			b.SetAwake();
		}
	}
}

void World::Solve(const TimeStep& step)
{
	m_profile.solveInit = float_t{0};
	m_profile.solveVelocity = float_t{0};
	m_profile.solvePosition = float_t{0};

	// Clear all the island flags.
	for (auto&& b: m_bodies)
	{
		assert(b.m_islandIndex == Body::InvalidIslandIndex);
		b.UnsetInIsland();
	}
	for (auto c = m_contactManager.GetContactList(); c; c = c->GetNext())
	{
		c->UnsetInIsland();
	}
	for (auto j = m_joints; j; j = j->GetNext())
	{
		j->SetInIsland(false);
	}

	// Size the island for the worst case.
	Island island(GetBodyCount(), m_contactManager.GetContactCount(), m_jointCount,
				  m_stackAllocator, m_contactManager.m_contactListener);
	
	// Build and simulate all awake islands.
	const auto stackSize = GetBodyCount();
	auto stack = std::unique_ptr<Body*[], StackAllocator&>(m_stackAllocator.Allocate<Body*>(stackSize), m_stackAllocator);
	for (auto seed = m_bodies; seed; seed = seed->GetNext())
	{
		// Skip seed (body) if static, already in island, not-awake, or not-active.
		if ((seed->GetType() == BodyType::Static) || seed->IsInIsland() || !seed->IsAwake() || !seed->IsActive())
		{
			// ((seed->m_flags & (Body::e_accelerationFlag|Body::e_velocityFlag)) == 0) ||
			// (seed->m_flags & Body::e_islandFlag) ||
			// (~(seed->m_flags) & (Body::e_awakeFlag|Body::e_activeFlag));
			continue;
		}

		// Seed (body): is not in island, is awake, is active, and, is dynamic or kinematic.

		// Reset island and stack.
		island.Clear();
		auto stackCount = size_type{0};
		stack[stackCount++] = seed.get();
		seed->SetInIsland();

		// Perform a depth first search (DFS) on the constraint graph.
		while (stackCount > 0)
		{
			// Grab the next body off the stack and add it to the island.
			const auto b = stack[--stackCount];
			assert(b->IsActive());
			island.Add(b);

			// Make sure the body is awake.
			b->SetAwake();

			// To keep islands smaller, don't propagate islands across static bodies.
			if (b->GetType() == BodyType::Static)
			{
				continue;
			}

			// Add to island: appropriate contacts of current body and appropriate 'other' bodies of those contacts.
			for (auto ce = b->m_contacts; ce; ce = ce->next)
			{
				const auto contact = ce->contact;

				// Skip contacts that are already in island, disabled, not-touching, or that have sensors.
				if ((contact->IsInIsland()) || (!contact->IsEnabled()) || (!contact->IsTouching()) || (contact->HasSensor()))
				{
					continue;
				}

				island.Add(contact);
				contact->SetInIsland();

				const auto other = ce->other;
				if (other->IsInIsland())
				{
					continue; // Other already in island, skip it.
				}

				assert(stackCount < stackSize);
				stack[stackCount++] = other;
				other->SetInIsland();
			}

			// Add to island: appropriate joints of current body and appropriate 'other' bodies of those joint.
			for (auto je = b->m_joints; je; je = je->next)
			{
				const auto joint = je->joint;
				const auto other = je->other;
				
				// Skip joints already in island or that are connected to inactive bodies.
				if (joint->IsInIsland() || !other->IsActive())
				{
					continue;
				}

				island.Add(joint);
				joint->SetInIsland(true);

				if (other->IsInIsland())
				{
					continue;
				}

				assert(stackCount < stackSize);
				stack[stackCount++] = other;
				other->SetInIsland();
			}
		}

		island.Solve(step, m_gravity, m_allowSleep);

		// Post solve cleanup.
		for (auto i = decltype(island.GetBodyCount()){0}; i < island.GetBodyCount(); ++i)
		{
			// Allow static bodies to participate in other islands.
			auto b = island.GetBody(i);
			if (b->GetType() == BodyType::Static)
			{
				b->UnsetInIsland();
			}
		}
	}

	{
		Timer timer;
		// Synchronize fixtures, check for out of range bodies.
		for (auto&& b: m_bodies)
		{
			// A non-static body that was in an island may have moved.
			if ((b.GetType() != BodyType::Static) && b.IsInIsland())
			{
				// Update fixtures (for broad-phase).
				b.SynchronizeFixtures();
			}
		}

		// Look for new contacts.
		m_contactManager.FindNewContacts();
		m_profile.broadphase = timer.GetMilliseconds();
	}
}

void World::ResetBodiesForSolveTOI()
{
	for (auto&& b: m_bodies)
	{
		b.UnsetInIsland();
		b.m_sweep.ResetAlpha0();
	}
}

void World::ResetContactsForSolveTOI()
{
	for (auto c = m_contactManager.GetContactList(); c; c = c->m_next)
	{
		// Invalidate TOI
		c->UnsetInIsland();
		c->UnsetToi();
		c->m_toiCount = 0;
	}	
}

World::ContactToiPair World::UpdateContactTOIs()
{
	auto minContact = static_cast<Contact*>(nullptr);
	auto minToi = float_t{1};
	
	for (auto c = m_contactManager.GetContactList(); c; c = c->m_next)
	{
		// Skip the disabled and excessive sub-stepped contacts
		if (!c->IsEnabled() || (c->m_toiCount >= MaxSubSteps))
		{
			continue;
		}
		
		if (!c->HasValidToi() && !c->UpdateTOI())
		{
			continue;
		}
		
		const auto toi = c->GetToi();
		if (minToi > toi)
		{
			// This is the minimum TOI found so far.
			minContact = c;
			minToi = toi;
		}
	}

	return ContactToiPair{minContact, minToi};
}

// Find TOI contacts and solve them.
void World::SolveTOI(const TimeStep& step)
{
	Island island(2 * MaxTOIContacts, MaxTOIContacts, 0, m_stackAllocator, m_contactManager.m_contactListener);

	if (m_stepComplete)
	{
		ResetBodiesForSolveTOI();
		ResetContactsForSolveTOI();
	}

	// Find TOI events and solve them.
	for (;;)
	{
		// Find the first TOI - the soonest one.
		const auto minContactToi = UpdateContactTOIs();

		//if ((!minContactToi.contact) || (minContactToi.toi >= float_t{1}))
		if ((!minContactToi.contact) || (minContactToi.toi > (float_t{1} - (float_t{10} * Epsilon))))
		{
			// No more TOI events. Done!
			m_stepComplete = true;
			break;
		}

		auto bA = minContactToi.contact->GetFixtureA()->GetBody();
		auto bB = minContactToi.contact->GetFixtureB()->GetBody();

		const auto backupA = bA->m_sweep;
		const auto backupB = bB->m_sweep;

		// Advance the bodies to the TOI.
		bA->Advance(minContactToi.toi);
		bB->Advance(minContactToi.toi);

		// The TOI contact likely has some new contact points.
		minContactToi.contact->Update(m_contactManager.m_contactListener);
		minContactToi.contact->UnsetToi();
		++(minContactToi.contact->m_toiCount);

		// Is contact disabled or separated?
		if (!minContactToi.contact->IsEnabled() || !minContactToi.contact->IsTouching())
		{
			// Restore the sweeps by undoing the Advance calls (and anything else done movement-wise)
			minContactToi.contact->UnsetEnabled();
			bA->m_sweep = backupA;
			bA->m_xf = GetTransformOne(bA->m_sweep);
			bB->m_sweep = backupB;
			bB->m_xf = GetTransformOne(bB->m_sweep);
			continue;
		}

		bA->SetAwake();
		bB->SetAwake();

		// Build the island
		island.Clear();
		island.Add(bA);
		bA->SetInIsland();
		island.Add(bB);
		bB->SetInIsland();
		island.Add(minContactToi.contact);
		minContactToi.contact->SetInIsland();

		// Get contacts on bodyA and bodyB.
		for (auto body: {bA, bB})
		{
			if (body->GetType() == BodyType::Dynamic)
			{
				ProcessContactsForTOI(island, *body, minContactToi.toi);
			}
		}

		TimeStep subStep;
		subStep.set_dt((float_t{1} - minContactToi.toi) * step.get_dt());
		subStep.dtRatio = float_t{1};
		subStep.positionIterations = MaxSubStepPositionIterations;
		subStep.velocityIterations = step.velocityIterations;
		subStep.warmStarting = false;
		island.SolveTOI(subStep, bA->m_islandIndex, bB->m_islandIndex);

		// Reset island flags and synchronize broad-phase proxies.
		for (auto i = decltype(island.GetBodyCount()){0}; i < island.GetBodyCount(); ++i)
		{
			auto body = island.GetBody(i);
			body->UnsetInIsland();

			if (body->GetType() == BodyType::Dynamic)
			{
				body->SynchronizeFixtures();
				
				// Invalidate all contact TOIs on this displaced body.
				for (auto ce = body->m_contacts; ce; ce = ce->next)
				{
					ce->contact->UnsetInIsland();
					ce->contact->UnsetToi();
				}
			}
		}

		// Commit fixture proxy movements to the broad-phase so that new contacts are created.
		// Also, some contacts can be destroyed.
		m_contactManager.FindNewContacts();

		if (m_subStepping)
		{
			m_stepComplete = false;
			break;
		}
	}
}

void World::ProcessContactsForTOI(Island& island, Body& body, float_t toi)
{
	assert(body.GetType() == BodyType::Dynamic);

	for (auto ce = body.m_contacts; ce; ce = ce->next)
	{
		if (IsFullOfBodies(island) || IsFullOfContacts(island))
		{
			break; // processed all bodies or all contacts, done.
		}
		
		auto contact = ce->contact;
		
		// Skip already added or sensor contacts
		if (contact->IsInIsland() || contact->HasSensor())
		{
			continue;
		}
		
		// Only static, kinematic, or bullet bodies are appropriate for CCD.
		auto other = ce->other;
		
		// Skip if neither bodies are appropriate for CCD
		if ((other->GetType() == BodyType::Dynamic) && !other->IsBullet() && !body.IsBullet())
		{
			continue;
		}
		
		// Tentatively advance the body to the TOI.
		const auto backup = other->m_sweep;
		if (!other->IsInIsland())
		{
			other->Advance(toi);
		}
		
		// Update the contact points
		contact->Update(m_contactManager.m_contactListener);
		
		// Revert and skip if contact disabled by user or if there are there no contact points anymore.
		if (!contact->IsEnabled() || !contact->IsTouching())
		{
			other->m_sweep = backup;
			other->m_xf = GetTransformOne(other->m_sweep);
			continue;
		}
		
		// Add the contact to the island
		island.Add(contact);
		contact->SetInIsland();
		
		// Has the other body already been added to the island?
		if (other->IsInIsland())
		{
			continue;
		}
		
		// Add the other body to the island.
		other->SetInIsland();
		
		if (other->GetType() != BodyType::Static)
		{
			other->SetAwake();
		}
		
		island.Add(other);
	}
}

void World::Step(float_t dt, unsigned velocityIterations, unsigned positionIterations)
{
	Timer stepTimer;

	// If new fixtures were added, we need to find the new contacts.
	if (HasNewFixtures())
	{
		m_contactManager.FindNewContacts();
		UnsetNewFixtures();
	}

	assert(!IsLocked());
	FlagGuard<decltype(m_flags)> flagGaurd(m_flags, e_locked);

	TimeStep step;
	step.set_dt(dt);
	step.velocityIterations	= velocityIterations;
	step.positionIterations = positionIterations;
	step.dtRatio = dt * m_inv_dt0;
	step.warmStarting = m_warmStarting;
	
	// Update contacts. This is where some contacts are destroyed.
	{
		Timer timer;
		m_contactManager.Collide();
		m_profile.collide = timer.GetMilliseconds();
	}

	// Integrate velocities, solve velocity constraints, and integrate positions.
	if (m_stepComplete && (step.get_dt() > float_t{0}))
	{
		Timer timer;
		Solve(step);
		m_profile.solve = timer.GetMilliseconds();
	}

	// Handle TOI events.
	if (m_continuousPhysics && (step.get_dt() > float_t{0}))
	{
		Timer timer;
		SolveTOI(step);
		m_profile.solveTOI = timer.GetMilliseconds();
	}

	if (step.get_dt() > float_t{0})
	{
		m_inv_dt0 = step.get_inv_dt();
	}

	if (GetAutoClearForces())
	{
		ClearForces();
	}

	m_profile.step = stepTimer.GetMilliseconds();
}

void World::ClearForces() noexcept
{
	for (auto&& body: m_bodies)
	{
		body.m_force = Vec2_zero;
		body.m_torque = float_t{0};
	}
}

struct WorldQueryWrapper
{
	using size_type = BroadPhase::size_type;

	bool QueryCallback(size_type proxyId)
	{
		const auto proxy = static_cast<FixtureProxy*>(broadPhase->GetUserData(proxyId));
		return callback->ReportFixture(proxy->fixture);
	}

	const BroadPhase* broadPhase;
	QueryFixtureReporter* callback;
};

void World::QueryAABB(QueryFixtureReporter* callback, const AABB& aabb) const
{
	WorldQueryWrapper wrapper;
	wrapper.broadPhase = &m_contactManager.m_broadPhase;
	wrapper.callback = callback;
	m_contactManager.m_broadPhase.Query(&wrapper, aabb);
}

struct WorldRayCastWrapper
{
	using size_type = BroadPhase::size_type;

	float_t RayCastCallback(const RayCastInput& input, size_type proxyId)
	{
		auto userData = broadPhase->GetUserData(proxyId);
		const auto proxy = static_cast<FixtureProxy*>(userData);
		auto fixture = proxy->fixture;
		const auto index = proxy->childIndex;
		RayCastOutput output;
		const auto hit = fixture->RayCast(&output, input, index);

		if (hit)
		{
			const auto fraction = output.fraction;
			assert(fraction >= 0 && fraction <= 1);
			const auto point = (float_t{1} - fraction) * input.p1 + fraction * input.p2;
			return callback->ReportFixture(fixture, point, output.normal, fraction);
		}

		return input.maxFraction;
	}

	WorldRayCastWrapper() = delete;

	constexpr WorldRayCastWrapper(const BroadPhase* bp, RayCastFixtureReporter* cb): broadPhase(bp), callback(cb) {}

	const BroadPhase* const broadPhase;
	RayCastFixtureReporter* const callback;
};

void World::RayCast(RayCastFixtureReporter* callback, const Vec2& point1, const Vec2& point2) const
{
	WorldRayCastWrapper wrapper(&m_contactManager.m_broadPhase, callback);
	const auto input = RayCastInput{point1, point2, float_t{1}};
	m_contactManager.m_broadPhase.RayCast(&wrapper, input);
}

void World::DrawShape(const Fixture* fixture, const Transform& xf, const Color& color)
{
	switch (fixture->GetType())
	{
	case Shape::e_circle:
		{
			const auto circle = static_cast<const CircleShape*>(fixture->GetShape());
			const auto center = Mul(xf, circle->GetPosition());
			const auto radius = circle->GetRadius();
			const auto axis = Mul(xf.q, Vec2{float_t{1}, float_t{0}});
			g_debugDraw->DrawSolidCircle(center, radius, axis, color);
		}
		break;

	case Shape::e_edge:
		{
			const auto edge = static_cast<const EdgeShape*>(fixture->GetShape());
			const auto v1 = Mul(xf, edge->GetVertex1());
			const auto v2 = Mul(xf, edge->GetVertex2());
			g_debugDraw->DrawSegment(v1, v2, color);
		}
		break;

	case Shape::e_chain:
		{
			const auto chain = static_cast<const ChainShape*>(fixture->GetShape());
			const auto count = chain->GetVertexCount();
			auto v1 = Mul(xf, chain->GetVertex(0));
			for (auto i = decltype(count){1}; i < count; ++i)
			{
				const auto v2 = Mul(xf, chain->GetVertex(i));
				g_debugDraw->DrawSegment(v1, v2, color);
				g_debugDraw->DrawCircle(v1, float_t(0.05), color);
				v1 = v2;
			}
		}
		break;

	case Shape::e_polygon:
		{
			const auto poly = static_cast<const PolygonShape*>(fixture->GetShape());
			const auto vertexCount = poly->GetVertexCount();
			assert(vertexCount <= MaxPolygonVertices);
			Vec2 vertices[MaxPolygonVertices];
			for (auto i = decltype(vertexCount){0}; i < vertexCount; ++i)
			{
				vertices[i] = Mul(xf, poly->GetVertex(i));
			}
			g_debugDraw->DrawSolidPolygon(vertices, vertexCount, color);
		}
		break;
            
    default:
        break;
	}
}

void World::DrawJoint(Joint* joint)
{
	const auto bodyA = joint->GetBodyA();
	const auto bodyB = joint->GetBodyB();
	const auto xf1 = bodyA->GetTransform();
	const auto xf2 = bodyB->GetTransform();
	const auto x1 = xf1.p;
	const auto x2 = xf2.p;
	const auto p1 = joint->GetAnchorA();
	const auto p2 = joint->GetAnchorB();

	const Color color(float_t(0.5), float_t(0.8), float_t(0.8));

	switch (joint->GetType())
	{
	case e_distanceJoint:
		g_debugDraw->DrawSegment(p1, p2, color);
		break;

	case e_pulleyJoint:
		{
			const auto pulley = static_cast<PulleyJoint*>(joint);
			const auto s1 = pulley->GetGroundAnchorA();
			const auto s2 = pulley->GetGroundAnchorB();
			g_debugDraw->DrawSegment(s1, p1, color);
			g_debugDraw->DrawSegment(s2, p2, color);
			g_debugDraw->DrawSegment(s1, s2, color);
		}
		break;

	case e_mouseJoint:
		// don't draw this
		break;

	default:
		g_debugDraw->DrawSegment(x1, p1, color);
		g_debugDraw->DrawSegment(p1, p2, color);
		g_debugDraw->DrawSegment(x2, p2, color);
	}
}

void World::DrawDebugData()
{
	if (g_debugDraw == nullptr)
	{
		return;
	}

	const auto flags = g_debugDraw->GetFlags();

	if (flags & Draw::e_shapeBit)
	{
		for (auto&& b: m_bodies)
		{
			const auto xf = b.GetTransform();
			for (auto&& f: b.GetFixtures())
			{
				if (!b.IsActive())
				{
					DrawShape(&f, xf, Color(0.5f, 0.5f, 0.3f));
				}
				else if (b.GetType() == BodyType::Static)
				{
					DrawShape(&f, xf, Color(0.5f, 0.9f, 0.5f));
				}
				else if (b.GetType() == BodyType::Kinematic)
				{
					DrawShape(&f, xf, Color(0.5f, 0.5f, 0.9f));
				}
				else if (!b.IsAwake())
				{
					DrawShape(&f, xf, Color(0.6f, 0.6f, 0.6f));
				}
				else
				{
					DrawShape(&f, xf, Color(0.9f, 0.7f, 0.7f));
				}
			}
		}
	}

	if (flags & Draw::e_jointBit)
	{
		for (auto j = m_joints; j; j = j->GetNext())
		{
			DrawJoint(j);
		}
	}

	if (flags & Draw::e_pairBit)
	{
		const Color color(0.3f, 0.9f, 0.9f);
		for (auto c = m_contactManager.GetContactList(); c; c = c->GetNext())
		{
			//Fixture* fixtureA = c->GetFixtureA();
			//Fixture* fixtureB = c->GetFixtureB();

			//Vec2 cA = fixtureA->GetAABB().GetCenter();
			//Vec2 cB = fixtureB->GetAABB().GetCenter();

			//g_debugDraw->DrawSegment(cA, cB, color);
		}
	}

	if (flags & Draw::e_aabbBit)
	{
		const Color color(0.9f, 0.3f, 0.9f);
		const auto bp = &m_contactManager.m_broadPhase;

		for (auto&& b: m_bodies)
		{
			if (!b.IsActive())
			{
				continue;
			}

			for (auto&& f: b.GetFixtures())
			{
				for (auto i = decltype(f.m_proxyCount){0}; i < f.m_proxyCount; ++i)
				{
					const auto proxy = f.m_proxies + i;
					const auto aabb = bp->GetFatAABB(proxy->proxyId);
					Vec2 vs[4];
					vs[0] = Vec2{aabb.GetLowerBound().x, aabb.GetLowerBound().y};
					vs[1] = Vec2{aabb.GetUpperBound().x, aabb.GetLowerBound().y};
					vs[2] = Vec2{aabb.GetUpperBound().x, aabb.GetUpperBound().y};
					vs[3] = Vec2{aabb.GetLowerBound().x, aabb.GetUpperBound().y};

					g_debugDraw->DrawPolygon(vs, 4, color);
				}
			}
		}
	}

	if (flags & Draw::e_centerOfMassBit)
	{
		for (auto&& b: m_bodies)
		{
			auto xf = b.GetTransform();
			xf.p = b.GetWorldCenter();
			g_debugDraw->DrawTransform(xf);
		}
	}
}

World::size_type World::GetProxyCount() const noexcept
{
	return m_contactManager.m_broadPhase.GetProxyCount();
}

World::size_type World::GetTreeHeight() const noexcept
{
	return m_contactManager.m_broadPhase.GetTreeHeight();
}

World::size_type World::GetTreeBalance() const
{
	return m_contactManager.m_broadPhase.GetTreeBalance();
}

float_t World::GetTreeQuality() const
{
	return m_contactManager.m_broadPhase.GetTreeQuality();
}

void World::ShiftOrigin(const Vec2& newOrigin)
{
	assert(!IsLocked());
	if (IsLocked())
	{
		return;
	}

	for (auto&& b: m_bodies)
	{
		b.m_xf.p -= newOrigin;
		b.m_sweep.pos0.c -= newOrigin;
		b.m_sweep.pos1.c -= newOrigin;
	}

	for (auto j = m_joints; j; j = j->m_next)
	{
		j->ShiftOrigin(newOrigin);
	}

	m_contactManager.m_broadPhase.ShiftOrigin(newOrigin);
}

void World::Dump()
{
	if (IsLocked())
	{
		return;
	}

	log("Vec2 g(%.15lef, %.15lef);\n", m_gravity.x, m_gravity.y);
	log("m_world->SetGravity(g);\n");

	log("Body** bodies = (Body**)alloc(%d * sizeof(Body*));\n", GetBodyCount());
	log("Joint** joints = (Joint**)alloc(%d * sizeof(Joint*));\n", m_jointCount);
	auto i = body_count_t{0};
	for (auto&& b: m_bodies)
	{
		b.m_islandIndex = i;
		b.Dump();
		++i;
	}

	i = 0;
	for (auto j = m_joints; j; j = j->m_next)
	{
		j->m_index = i;
		++i;
	}

	// First pass on joints, skip gear joints.
	for (auto j = m_joints; j; j = j->m_next)
	{
		if (j->m_type == e_gearJoint)
		{
			continue;
		}

		log("{\n");
		j->Dump();
		log("}\n");
	}

	// Second pass on joints, only gear joints.
	for (auto j = m_joints; j; j = j->m_next)
	{
		if (j->m_type != e_gearJoint)
		{
			continue;
		}

		log("{\n");
		j->Dump();
		log("}\n");
	}

	log("free(joints);\n");
	log("free(bodies);\n");
	log("joints = nullptr;\n");
	log("bodies = nullptr;\n");
}

} // namespace box2d