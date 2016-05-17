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

#include <Box2D/Dynamics/b2World.h>
#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Dynamics/b2Fixture.h>
#include <Box2D/Dynamics/b2Island.h>
#include <Box2D/Dynamics/Joints/b2PulleyJoint.h>
#include <Box2D/Dynamics/Contacts/b2Contact.h>
#include <Box2D/Collision/b2Collision.h>
#include <Box2D/Collision/b2BroadPhase.h>
#include <Box2D/Collision/Shapes/b2CircleShape.h>
#include <Box2D/Collision/Shapes/b2EdgeShape.h>
#include <Box2D/Collision/Shapes/b2ChainShape.h>
#include <Box2D/Collision/Shapes/b2PolygonShape.h>
#include <Box2D/Collision/b2TimeOfImpact.h>
#include <Box2D/Common/b2Draw.h>
#include <Box2D/Common/b2Timer.h>
#include <new>
#include <functional>
#include <type_traits>

template <typename T>
class b2FlagGuard
{
public:
	b2FlagGuard(T& flag, T value) : m_flag(flag), m_value(value)
	{
		static_assert(std::is_unsigned<T>::value, "Unsigned interger required");
		m_flag |= m_value;
	}

	~b2FlagGuard()
	{
		m_flag &= ~m_value;
	}

	b2FlagGuard() = delete;

private:
	T& m_flag;
	T m_value;
};

template <class T>
class b2RaiiWrapper
{
public:
	b2RaiiWrapper() = delete;
	b2RaiiWrapper(std::function<void(T&)> on_destruction): m_on_destruction(on_destruction) {}
	~b2RaiiWrapper() { m_on_destruction(m_wrapped); }
	T m_wrapped;

private:
	std::function<void(T&)> m_on_destruction;
};

b2World::b2World(const b2Vec2& gravity): m_gravity(gravity)
{
	memset(&m_profile, 0, sizeof(b2Profile));
}

b2World::~b2World()
{
	// Some shapes allocate using b2Alloc.
	auto b = m_bodyList;
	while (b)
	{
		auto bNext = b->m_next;
		auto f = b->m_fixtureList;
		while (f)
		{
			auto fNext = f->m_next;
			f->m_proxyCount = 0;
			f->Destroy(&m_blockAllocator);
			f = fNext;
		}
		b = bNext;
	}
}

void b2World::SetDestructionListener(b2DestructionListener* listener) noexcept
{
	m_destructionListener = listener;
}

void b2World::SetContactFilter(b2ContactFilter* filter) noexcept
{
	m_contactManager.m_contactFilter = filter;
}

void b2World::SetContactListener(b2ContactListener* listener) noexcept
{
	m_contactManager.m_contactListener = listener;
}

void b2World::SetDebugDraw(b2Draw* debugDraw) noexcept
{
	g_debugDraw = debugDraw;
}

b2Body* b2World::CreateBody(const b2BodyDef* def)
{
	b2Assert(!IsLocked());
	if (IsLocked())
	{
		return nullptr;
	}

	void* mem = m_blockAllocator.Allocate(sizeof(b2Body));
	auto b = new (mem) b2Body(def, this);

	// Add to world doubly linked list.
	b->m_prev = nullptr;
	b->m_next = m_bodyList;
	if (m_bodyList)
	{
		m_bodyList->m_prev = b;
	}
	m_bodyList = b;
	++m_bodyCount;

	return b;
}

void b2World::DestroyBody(b2Body* b)
{
	b2Assert(m_bodyCount > 0);
	b2Assert(!IsLocked());
	if (IsLocked())
	{
		return;
	}

	// Delete the attached joints.
	auto je = b->m_jointList;
	while (je)
	{
		auto je0 = je;
		je = je->next;

		if (m_destructionListener)
		{
			m_destructionListener->SayGoodbye(je0->joint);
		}

		DestroyJoint(je0->joint);

		b->m_jointList = je;
	}
	b->m_jointList = nullptr;

	// Delete the attached contacts.
	auto ce = b->m_contactList;
	while (ce)
	{
		auto ce0 = ce;
		ce = ce->next;
		m_contactManager.Destroy(ce0->contact);
	}
	b->m_contactList = nullptr;

	// Delete the attached fixtures. This destroys broad-phase proxies.
	auto f = b->m_fixtureList;
	while (f)
	{
		auto f0 = f;
		f = f->m_next;

		if (m_destructionListener)
		{
			m_destructionListener->SayGoodbye(f0);
		}

		f0->DestroyProxies(m_contactManager.m_broadPhase);
		f0->Destroy(&m_blockAllocator);
		f0->~b2Fixture();
		m_blockAllocator.Free(f0, sizeof(b2Fixture));

		b->m_fixtureList = f;
		b->m_fixtureCount -= 1;
	}
	b->m_fixtureList = nullptr;
	b->m_fixtureCount = 0;

	// Remove world body list.
	if (b->m_prev)
	{
		b->m_prev->m_next = b->m_next;
	}

	if (b->m_next)
	{
		b->m_next->m_prev = b->m_prev;
	}

	if (b == m_bodyList)
	{
		m_bodyList = b->m_next;
	}

	--m_bodyCount;
	b->~b2Body();
	m_blockAllocator.Free(b, sizeof(b2Body));
}

b2Joint* b2World::CreateJoint(const b2JointDef* def)
{
	b2Assert(!IsLocked());
	if (IsLocked())
	{
		return nullptr;
	}

	auto j = b2Joint::Create(def, &m_blockAllocator);

	// Connect to the world list.
	j->m_prev = nullptr;
	j->m_next = m_jointList;
	if (m_jointList)
	{
		m_jointList->m_prev = j;
	}
	m_jointList = j;
	++m_jointCount;

	// Connect to the bodies' doubly linked lists.
	j->m_edgeA.joint = j;
	j->m_edgeA.other = j->m_bodyB;
	j->m_edgeA.prev = nullptr;
	j->m_edgeA.next = j->m_bodyA->m_jointList;
	if (j->m_bodyA->m_jointList) j->m_bodyA->m_jointList->prev = &j->m_edgeA;
	j->m_bodyA->m_jointList = &j->m_edgeA;

	j->m_edgeB.joint = j;
	j->m_edgeB.other = j->m_bodyA;
	j->m_edgeB.prev = nullptr;
	j->m_edgeB.next = j->m_bodyB->m_jointList;
	if (j->m_bodyB->m_jointList) j->m_bodyB->m_jointList->prev = &j->m_edgeB;
	j->m_bodyB->m_jointList = &j->m_edgeB;

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

void b2World::DestroyJoint(b2Joint* j)
{
	b2Assert(!IsLocked());
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

	if (j == m_jointList)
	{
		m_jointList = j->m_next;
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

	if (&j->m_edgeA == bodyA->m_jointList)
	{
		bodyA->m_jointList = j->m_edgeA.next;
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

	if (&j->m_edgeB == bodyB->m_jointList)
	{
		bodyB->m_jointList = j->m_edgeB.next;
	}

	j->m_edgeB.prev = nullptr;
	j->m_edgeB.next = nullptr;

	b2Joint::Destroy(j, &m_blockAllocator);

	b2Assert(m_jointCount > 0);
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
void b2World::SetAllowSleeping(bool flag) noexcept
{
	if (flag == m_allowSleep)
	{
		return;
	}

	m_allowSleep = flag;
	if (!m_allowSleep)
	{
		for (auto b = m_bodyList; b; b = b->m_next)
		{
			b->SetAwake();
		}
	}
}

// Find islands, integrate and solve constraints, solve position constraints
void b2World::Solve(const b2TimeStep& step)
{
	m_profile.solveInit = b2Float{0};
	m_profile.solveVelocity = b2Float{0};
	m_profile.solvePosition = b2Float{0};

	// Size the island for the worst case.
	b2Island island(m_bodyCount,
					m_contactManager.GetContactCount(),
					m_jointCount,
					&m_stackAllocator,
					m_contactManager.m_contactListener);

	// Clear all the island flags.
	for (auto b = m_bodyList; b; b = b->GetNext())
	{
		b2Assert(b->m_islandIndex == b2Body::InvalidIslandIndex);
		b->UnsetInIsland();
	}
	for (auto c = m_contactManager.GetContactList(); c; c = c->GetNext())
	{
		c->UnsetInIsland();
	}
	for (auto j = m_jointList; j; j = j->GetNext())
	{
		j->SetInIsland(false);
	}

	// Build and simulate all awake islands.
	const auto stackSize = m_bodyCount;
	auto stack = static_cast<b2Body**>(m_stackAllocator.Allocate(stackSize * sizeof(b2Body*)));
	for (auto seed = m_bodyList; seed; seed = seed->GetNext())
	{
		if (seed->IsInIsland())
		{
			continue;
		}

		if ((!seed->IsAwake()) || (!seed->IsActive()))
		{
			continue;
		}

		// The seed can be dynamic or kinematic.
		if (seed->GetType() == b2_staticBody)
		{
			continue;
		}

		// Reset island and stack.
		island.Clear();
		auto stackCount = size_type{0};
		stack[stackCount++] = seed;
		seed->SetInIsland();

		// Perform a depth first search (DFS) on the constraint graph.
		while (stackCount > 0)
		{
			// Grab the next body off the stack and add it to the island.
			auto b = stack[--stackCount];
			b2Assert(b->IsActive());
			island.Add(b);

			// Make sure the body is awake.
			b->SetAwake();

			// To keep islands as small as possible, we don't
			// propagate islands across static bodies.
			if (b->GetType() == b2_staticBody)
			{
				continue;
			}

			// Search all contacts connected to this body.
			for (auto ce = b->m_contactList; ce; ce = ce->next)
			{
				auto contact = ce->contact;

				// Has this contact already been added to an island?
				if (contact->IsInIsland())
				{
					continue;
				}

				// Is this contact solid and touching?
				if ((!contact->IsEnabled()) || (!contact->IsTouching()))
				{
					continue;
				}

				// Skip sensors.
				if (contact->GetFixtureA()->IsSensor() || contact->GetFixtureB()->IsSensor())
				{
					continue;
				}

				island.Add(contact);
				contact->SetInIsland();

				auto other = ce->other;

				// Was the other body already added to this island?
				if (other->IsInIsland())
				{
					continue;
				}

				b2Assert(stackCount < stackSize);
				stack[stackCount++] = other;
				other->SetInIsland();
			}

			// Search all joints connect to this body.
			for (auto je = b->m_jointList; je; je = je->next)
			{
				if (je->joint->IsInIsland())
				{
					continue;
				}

				auto other = je->other;

				// Don't simulate joints connected to inactive bodies.
				if (!other->IsActive())
				{
					continue;
				}

				island.Add(je->joint);
				je->joint->SetInIsland(true);

				if (other->IsInIsland())
				{
					continue;
				}

				b2Assert(stackCount < stackSize);
				stack[stackCount++] = other;
				other->SetInIsland();
			}
		}

		b2Profile profile;
		island.Solve(&profile, step, m_gravity, m_allowSleep);
		m_profile.solveInit += profile.solveInit;
		m_profile.solveVelocity += profile.solveVelocity;
		m_profile.solvePosition += profile.solvePosition;

		// Post solve cleanup.
		for (auto i = decltype(island.GetBodyCount()){0}; i < island.GetBodyCount(); ++i)
		{
			// Allow static bodies to participate in other islands.
			auto b = island.GetBody(i);
			if (b->GetType() == b2_staticBody)
			{
				b->UnsetInIsland();
			}
		}
	}

	m_stackAllocator.Free(stack);

	{
		b2Timer timer;
		// Synchronize fixtures, check for out of range bodies.
		for (auto b = m_bodyList; b; b = b->GetNext())
		{
			// If a body was not in an island then it did not move.
			if (!(b->IsInIsland()))
			{
				continue;
			}

			if (b->GetType() == b2_staticBody)
			{
				continue;
			}

			// Update fixtures (for broad-phase).
			b->SynchronizeFixtures();
		}

		// Look for new contacts.
		m_contactManager.FindNewContacts();
		m_profile.broadphase = timer.GetMilliseconds();
	}
}

// Find TOI contacts and solve them.
void b2World::SolveTOI(const b2TimeStep& step)
{
	b2Island island(2 * b2_maxTOIContacts, b2_maxTOIContacts, 0, &m_stackAllocator, m_contactManager.m_contactListener);

	if (m_stepComplete)
	{
		for (auto b = m_bodyList; b; b = b->m_next)
		{
			b->UnsetInIsland();
			b->m_sweep.alpha0 = b2Float{0};
		}

		for (auto c = m_contactManager.GetContactList(); c; c = c->m_next)
		{
			// Invalidate TOI
			c->UnsetInIsland();
			c->m_toiCount = 0;
			c->UnsetToi();
		}
	}

	// Find TOI events and solve them.
	for (;;)
	{
		// Find the first TOI.
		auto minContact = static_cast<b2Contact*>(nullptr);
		auto minAlpha = b2Float{1};

		for (auto c = m_contactManager.GetContactList(); c; c = c->m_next)
		{
			// Is this contact disabled?
			if (!c->IsEnabled())
			{
				continue;
			}

			// Prevent excessive sub-stepping.
			if (c->m_toiCount >= b2_maxSubSteps)
			{
				continue;
			}

			if (!c->HasValidToi() && !c->UpdateTOI())
			{
				continue;
			}

			const auto alpha = c->GetToi();
			if (minAlpha > alpha)
			{
				// This is the minimum TOI found so far.
				minContact = c;
				minAlpha = alpha;
			}
		}

		// if ((!minContact) || (minAlpha >= b2Float(1)))
		if ((!minContact) || (minAlpha > (b2Float(1) - (b2Float(10) * b2_epsilon))))
		{
			// No more TOI events. Done!
			m_stepComplete = true;
			break;
		}

		// Advance the bodies to the TOI.
		auto fA = minContact->GetFixtureA();
		auto fB = minContact->GetFixtureB();
		auto bA = fA->GetBody();
		auto bB = fB->GetBody();

		const auto backupA = bA->m_sweep;
		const auto backupB = bB->m_sweep;

		bA->Advance(minAlpha);
		bB->Advance(minAlpha);

		// The TOI contact likely has some new contact points.
		minContact->Update(m_contactManager.m_contactListener);
		minContact->UnsetToi();
		++(minContact->m_toiCount);

		// Is the contact solid?
		if ((!minContact->IsEnabled()) || (!minContact->IsTouching()))
		{
			// Restore the sweeps.
			minContact->UnsetEnabled();
			bA->m_sweep = backupA;
			bB->m_sweep = backupB;
			bA->m_xf = b2GetTransformOne(bA->m_sweep);
			bB->m_xf = b2GetTransformOne(bB->m_sweep);
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

		island.Add(minContact);
		minContact->SetInIsland();

		// Get contacts on bodyA and bodyB.
		b2Body* bodies[2] = {bA, bB};
		for (auto body: bodies)
		{
			if (body->m_type == b2_dynamicBody)
			{
				for (auto ce = body->m_contactList; ce; ce = ce->next)
				{
					if (island.GetBodyCount() == island.GetBodyCapacity())
					{
						break;
					}

					if (island.GetContactCount() == island.GetContactCapacity())
					{
						break;
					}

					auto contact = ce->contact;

					// Has this contact already been added to the island?
					if (contact->IsInIsland())
					{
						continue;
					}

					// Only add static, kinematic, or bullet bodies.
					auto other = ce->other;
					if ((other->m_type == b2_dynamicBody) &&
						(!body->IsBullet()) && (!other->IsBullet()))
					{
						continue;
					}

					// Skip sensors.
					if (contact->m_fixtureA->m_isSensor || contact->m_fixtureB->m_isSensor)
					{
						continue;
					}

					// Tentatively advance the body to the TOI.
					const auto backup = other->m_sweep;
					if (!other->IsInIsland())
					{
						other->Advance(minAlpha);
					}

					// Update the contact points
					contact->Update(m_contactManager.m_contactListener);

					// Was the contact disabled by the user?
					if (!contact->IsEnabled())
					{
						other->m_sweep = backup;
						other->m_xf = b2GetTransformOne(other->m_sweep);
						continue;
					}

					// Are there contact points?
					if (!contact->IsTouching())
					{
						other->m_sweep = backup;
						other->m_xf = b2GetTransformOne(other->m_sweep);
						continue;
					}

					// Add the contact to the island
					contact->SetInIsland();
					island.Add(contact);

					// Has the other body already been added to the island?
					if (other->IsInIsland())
					{
						continue;
					}
					
					// Add the other body to the island.
					other->SetInIsland();

					if (other->m_type != b2_staticBody)
					{
						other->SetAwake();
					}

					island.Add(other);
				}
			}
		}

		b2TimeStep subStep;
		subStep.set_dt((b2Float(1) - minAlpha) * step.get_dt());
		subStep.dtRatio = b2Float(1);
		subStep.positionIterations = b2_maxSubStepPositionIterations;
		subStep.velocityIterations = step.velocityIterations;
		subStep.warmStarting = false;
		island.SolveTOI(subStep, bA->m_islandIndex, bB->m_islandIndex);

		// Reset island flags and synchronize broad-phase proxies.
		for (auto i = decltype(island.GetBodyCount()){0}; i < island.GetBodyCount(); ++i)
		{
			auto body = island.GetBody(i);
			body->UnsetInIsland();

			if (body->m_type != b2_dynamicBody)
			{
				continue;
			}

			body->SynchronizeFixtures();

			// Invalidate all contact TOIs on this displaced body.
			for (auto ce = body->m_contactList; ce; ce = ce->next)
			{
				ce->contact->UnsetInIsland();
				ce->contact->UnsetToi();
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

void b2World::Step(b2Float dt, int32 velocityIterations, int32 positionIterations)
{
	b2Timer stepTimer;

	// If new fixtures were added, we need to find the new contacts.
	if (HasNewFixtures())
	{
		m_contactManager.FindNewContacts();
		UnsetNewFixtures();
	}

	b2Assert(!IsLocked());
	b2FlagGuard<decltype(m_flags)> flagGaurd(m_flags, e_locked);

	b2TimeStep step;
	step.set_dt(dt);
	step.velocityIterations	= velocityIterations;
	step.positionIterations = positionIterations;
	step.dtRatio = m_inv_dt0 * dt;
	step.warmStarting = m_warmStarting;
	
	// Update contacts. This is where some contacts are destroyed.
	{
		b2Timer timer;
		m_contactManager.Collide();
		m_profile.collide = timer.GetMilliseconds();
	}

	// Integrate velocities, solve velocity constraints, and integrate positions.
	if (m_stepComplete && (step.get_dt() > b2Float{0}))
	{
		b2Timer timer;
		Solve(step);
		m_profile.solve = timer.GetMilliseconds();
	}

	// Handle TOI events.
	if (m_continuousPhysics && (step.get_dt() > b2Float{0}))
	{
		b2Timer timer;
		SolveTOI(step);
		m_profile.solveTOI = timer.GetMilliseconds();
	}

	if (step.get_dt() > b2Float{0})
	{
		m_inv_dt0 = step.get_inv_dt();
	}

	if (GetAutoClearForces())
	{
		ClearForces();
	}

	m_profile.step = stepTimer.GetMilliseconds();
}

void b2World::ClearForces() noexcept
{
	for (auto body = m_bodyList; body; body = body->GetNext())
	{
		body->m_force = b2Vec2_zero;
		body->m_torque = b2Float{0};
	}
}

struct b2WorldQueryWrapper
{
	using size_type = b2BroadPhase::size_type;

	bool QueryCallback(size_type proxyId)
	{
		const auto proxy = static_cast<b2FixtureProxy*>(broadPhase->GetUserData(proxyId));
		return callback->ReportFixture(proxy->fixture);
	}

	const b2BroadPhase* broadPhase;
	b2QueryCallback* callback;
};

void b2World::QueryAABB(b2QueryCallback* callback, const b2AABB& aabb) const
{
	b2WorldQueryWrapper wrapper;
	wrapper.broadPhase = &m_contactManager.m_broadPhase;
	wrapper.callback = callback;
	m_contactManager.m_broadPhase.Query(&wrapper, aabb);
}

struct b2WorldRayCastWrapper
{
	using size_type = b2BroadPhase::size_type;

	b2Float RayCastCallback(const b2RayCastInput& input, size_type proxyId)
	{
		auto userData = broadPhase->GetUserData(proxyId);
		const auto proxy = static_cast<b2FixtureProxy*>(userData);
		auto fixture = proxy->fixture;
		const auto index = proxy->childIndex;
		b2RayCastOutput output;
		const auto hit = fixture->RayCast(&output, input, index);

		if (hit)
		{
			const auto fraction = output.fraction;
			const auto point = (b2Float(1) - fraction) * input.p1 + fraction * input.p2;
			return callback->ReportFixture(fixture, point, output.normal, fraction);
		}

		return input.maxFraction;
	}

	b2WorldRayCastWrapper() = delete;

	constexpr b2WorldRayCastWrapper(const b2BroadPhase* bp, b2RayCastCallback* cb): broadPhase(bp), callback(cb) {}

	const b2BroadPhase* const broadPhase;
	b2RayCastCallback* const callback;
};

void b2World::RayCast(b2RayCastCallback* callback, const b2Vec2& point1, const b2Vec2& point2) const
{
	b2WorldRayCastWrapper wrapper(&m_contactManager.m_broadPhase, callback);
	const auto input = b2RayCastInput{point1, point2, b2Float(1)};
	m_contactManager.m_broadPhase.RayCast(&wrapper, input);
}

void b2World::DrawShape(const b2Fixture* fixture, const b2Transform& xf, const b2Color& color)
{
	switch (fixture->GetType())
	{
	case b2Shape::e_circle:
		{
			const auto circle = static_cast<const b2CircleShape*>(fixture->GetShape());
			const auto center = b2Mul(xf, circle->GetPosition());
			const auto radius = circle->GetRadius();
			const auto axis = b2Mul(xf.q, b2Vec2(b2Float(1), b2Float{0}));
			g_debugDraw->DrawSolidCircle(center, radius, axis, color);
		}
		break;

	case b2Shape::e_edge:
		{
			const auto edge = static_cast<const b2EdgeShape*>(fixture->GetShape());
			const auto v1 = b2Mul(xf, edge->GetVertex1());
			const auto v2 = b2Mul(xf, edge->GetVertex2());
			g_debugDraw->DrawSegment(v1, v2, color);
		}
		break;

	case b2Shape::e_chain:
		{
			const auto chain = static_cast<const b2ChainShape*>(fixture->GetShape());
			const auto count = chain->GetVertexCount();
			auto v1 = b2Mul(xf, chain->GetVertex(0));
			for (auto i = decltype(count){1}; i < count; ++i)
			{
				const auto v2 = b2Mul(xf, chain->GetVertex(i));
				g_debugDraw->DrawSegment(v1, v2, color);
				g_debugDraw->DrawCircle(v1, b2Float(0.05), color);
				v1 = v2;
			}
		}
		break;

	case b2Shape::e_polygon:
		{
			const auto poly = static_cast<const b2PolygonShape*>(fixture->GetShape());
			const auto vertexCount = poly->GetVertexCount();
			b2Assert(vertexCount <= b2_maxPolygonVertices);
			b2Vec2 vertices[b2_maxPolygonVertices];
			for (auto i = decltype(vertexCount){0}; i < vertexCount; ++i)
			{
				vertices[i] = b2Mul(xf, poly->GetVertex(i));
			}
			g_debugDraw->DrawSolidPolygon(vertices, vertexCount, color);
		}
		break;
            
    default:
        break;
	}
}

void b2World::DrawJoint(b2Joint* joint)
{
	const auto bodyA = joint->GetBodyA();
	const auto bodyB = joint->GetBodyB();
	const auto xf1 = bodyA->GetTransform();
	const auto xf2 = bodyB->GetTransform();
	const auto x1 = xf1.p;
	const auto x2 = xf2.p;
	const auto p1 = joint->GetAnchorA();
	const auto p2 = joint->GetAnchorB();

	const b2Color color(b2Float(0.5), b2Float(0.8), b2Float(0.8));

	switch (joint->GetType())
	{
	case e_distanceJoint:
		g_debugDraw->DrawSegment(p1, p2, color);
		break;

	case e_pulleyJoint:
		{
			const auto pulley = static_cast<b2PulleyJoint*>(joint);
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

void b2World::DrawDebugData()
{
	if (g_debugDraw == nullptr)
	{
		return;
	}

	const auto flags = g_debugDraw->GetFlags();

	if (flags & b2Draw::e_shapeBit)
	{
		for (auto b = m_bodyList; b; b = b->GetNext())
		{
			const auto xf = b->GetTransform();
			for (auto f = b->GetFixtureList(); f; f = f->GetNext())
			{
				if (!b->IsActive())
				{
					DrawShape(f, xf, b2Color(0.5f, 0.5f, 0.3f));
				}
				else if (b->GetType() == b2_staticBody)
				{
					DrawShape(f, xf, b2Color(0.5f, 0.9f, 0.5f));
				}
				else if (b->GetType() == b2_kinematicBody)
				{
					DrawShape(f, xf, b2Color(0.5f, 0.5f, 0.9f));
				}
				else if (!b->IsAwake())
				{
					DrawShape(f, xf, b2Color(0.6f, 0.6f, 0.6f));
				}
				else
				{
					DrawShape(f, xf, b2Color(0.9f, 0.7f, 0.7f));
				}
			}
		}
	}

	if (flags & b2Draw::e_jointBit)
	{
		for (auto j = m_jointList; j; j = j->GetNext())
		{
			DrawJoint(j);
		}
	}

	if (flags & b2Draw::e_pairBit)
	{
		const b2Color color(0.3f, 0.9f, 0.9f);
		for (auto c = m_contactManager.GetContactList(); c; c = c->GetNext())
		{
			//b2Fixture* fixtureA = c->GetFixtureA();
			//b2Fixture* fixtureB = c->GetFixtureB();

			//b2Vec2 cA = fixtureA->GetAABB().GetCenter();
			//b2Vec2 cB = fixtureB->GetAABB().GetCenter();

			//g_debugDraw->DrawSegment(cA, cB, color);
		}
	}

	if (flags & b2Draw::e_aabbBit)
	{
		const b2Color color(0.9f, 0.3f, 0.9f);
		const auto bp = &m_contactManager.m_broadPhase;

		for (auto b = m_bodyList; b; b = b->GetNext())
		{
			if (!b->IsActive())
			{
				continue;
			}

			for (auto f = b->GetFixtureList(); f; f = f->GetNext())
			{
				for (auto i = decltype(f->m_proxyCount){0}; i < f->m_proxyCount; ++i)
				{
					const auto proxy = f->m_proxies + i;
					const auto aabb = bp->GetFatAABB(proxy->proxyId);
					b2Vec2 vs[4];
					vs[0] = b2Vec2(aabb.GetLowerBound().x, aabb.GetLowerBound().y);
					vs[1] = b2Vec2(aabb.GetUpperBound().x, aabb.GetLowerBound().y);
					vs[2] = b2Vec2(aabb.GetUpperBound().x, aabb.GetUpperBound().y);
					vs[3] = b2Vec2(aabb.GetLowerBound().x, aabb.GetUpperBound().y);

					g_debugDraw->DrawPolygon(vs, 4, color);
				}
			}
		}
	}

	if (flags & b2Draw::e_centerOfMassBit)
	{
		for (auto b = m_bodyList; b; b = b->GetNext())
		{
			auto xf = b->GetTransform();
			xf.p = b->GetWorldCenter();
			g_debugDraw->DrawTransform(xf);
		}
	}
}

b2World::size_type b2World::GetProxyCount() const noexcept
{
	return m_contactManager.m_broadPhase.GetProxyCount();
}

b2World::size_type b2World::GetTreeHeight() const noexcept
{
	return m_contactManager.m_broadPhase.GetTreeHeight();
}

b2World::size_type b2World::GetTreeBalance() const
{
	return m_contactManager.m_broadPhase.GetTreeBalance();
}

b2Float b2World::GetTreeQuality() const
{
	return m_contactManager.m_broadPhase.GetTreeQuality();
}

void b2World::ShiftOrigin(const b2Vec2& newOrigin)
{
	b2Assert(!IsLocked());
	if (IsLocked())
	{
		return;
	}

	for (auto b = m_bodyList; b; b = b->m_next)
	{
		b->m_xf.p -= newOrigin;
		b->m_sweep.c0 -= newOrigin;
		b->m_sweep.c -= newOrigin;
	}

	for (auto j = m_jointList; j; j = j->m_next)
	{
		j->ShiftOrigin(newOrigin);
	}

	m_contactManager.m_broadPhase.ShiftOrigin(newOrigin);
}

void b2World::Dump()
{
	if (IsLocked())
	{
		return;
	}

	b2Log("b2Vec2 g(%.15lef, %.15lef);\n", m_gravity.x, m_gravity.y);
	b2Log("m_world->SetGravity(g);\n");

	b2Log("b2Body** bodies = (b2Body**)b2Alloc(%d * sizeof(b2Body*));\n", m_bodyCount);
	b2Log("b2Joint** joints = (b2Joint**)b2Alloc(%d * sizeof(b2Joint*));\n", m_jointCount);
	auto i = island_count_t{0};
	for (auto b = m_bodyList; b; b = b->m_next)
	{
		b->m_islandIndex = i;
		b->Dump();
		++i;
	}

	i = 0;
	for (auto j = m_jointList; j; j = j->m_next)
	{
		j->m_index = i;
		++i;
	}

	// First pass on joints, skip gear joints.
	for (auto j = m_jointList; j; j = j->m_next)
	{
		if (j->m_type == e_gearJoint)
		{
			continue;
		}

		b2Log("{\n");
		j->Dump();
		b2Log("}\n");
	}

	// Second pass on joints, only gear joints.
	for (auto j = m_jointList; j; j = j->m_next)
	{
		if (j->m_type != e_gearJoint)
		{
			continue;
		}

		b2Log("{\n");
		j->Dump();
		b2Log("}\n");
	}

	b2Log("b2Free(joints);\n");
	b2Log("b2Free(bodies);\n");
	b2Log("joints = nullptr;\n");
	b2Log("bodies = nullptr;\n");
}
