/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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

#include <Box2D/Dynamics/b2ContactManager.h>
#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Dynamics/b2Fixture.h>
#include <Box2D/Dynamics/b2WorldCallbacks.h>
#include <Box2D/Dynamics/Contacts/b2Contact.h>

b2ContactFilter b2_defaultFilter;
b2ContactListener b2_defaultListener;

b2ContactManager::b2ContactManager(b2BlockAllocator* allocator):
	m_allocator(allocator),
	m_contactFilter(&b2_defaultFilter), m_contactListener(&b2_defaultListener)
{}

void b2ContactManager::Destroy(b2Contact* c)
{
	auto fixtureA = c->GetFixtureA();
	auto fixtureB = c->GetFixtureB();
	auto bodyA = fixtureA->GetBody();
	auto bodyB = fixtureB->GetBody();

	if (m_contactListener && c->IsTouching())
	{
		m_contactListener->EndContact(c);
	}

	// Remove from the world.
	if (c->m_prev)
	{
		c->m_prev->m_next = c->m_next;
	}

	if (c->m_next)
	{
		c->m_next->m_prev = c->m_prev;
	}

	if (c == m_contactList)
	{
		m_contactList = c->m_next;
	}

	// Remove from body 1
	if (c->m_nodeA.prev)
	{
		c->m_nodeA.prev->next = c->m_nodeA.next;
	}

	if (c->m_nodeA.next)
	{
		c->m_nodeA.next->prev = c->m_nodeA.prev;
	}

	if (&c->m_nodeA == bodyA->m_contactList)
	{
		bodyA->m_contactList = c->m_nodeA.next;
	}

	// Remove from body 2
	if (c->m_nodeB.prev)
	{
		c->m_nodeB.prev->next = c->m_nodeB.next;
	}

	if (c->m_nodeB.next)
	{
		c->m_nodeB.next->prev = c->m_nodeB.prev;
	}

	if (&c->m_nodeB == bodyB->m_contactList)
	{
		bodyB->m_contactList = c->m_nodeB.next;
	}

	// Call the factory.
	b2Contact::Destroy(c, m_allocator);
	--m_contactCount;
}

// This is the top level collision call for the time step. Here
// all the narrow phase collision is processed for the world
// contact list.
void b2ContactManager::Collide()
{
	// Update awake contacts.
	auto next = m_contactList;
	for (auto c = m_contactList; c; c = next)
	{
		next = c->GetNext();

		const auto fixtureA = c->GetFixtureA();
		const auto fixtureB = c->GetFixtureB();
		const auto bodyA = fixtureA->GetBody();
		const auto bodyB = fixtureB->GetBody();
		 
		// Is this contact flagged for filtering?
		if (c->NeedsFiltering())
		{
			// Should these bodies collide?
			if (!(bodyB->ShouldCollide(bodyA)))
			{
				Destroy(c);
				continue;
			}

			// Check user filtering.
			if (m_contactFilter && !(m_contactFilter->ShouldCollide(fixtureA, fixtureB)))
			{
				Destroy(c);
				continue;
			}

			// Clear the filtering flag.
			c->UnflagForFiltering();
		}

		const bool activeA = bodyA->IsAwake() && (bodyA->m_type != b2_staticBody);
		const bool activeB = bodyB->IsAwake() && (bodyB->m_type != b2_staticBody);

		// At least one body must be awake and it must be dynamic or kinematic.
		if (!activeA && !activeB)
		{
			continue;
		}

		const auto indexA = c->GetChildIndexA();
		const auto indexB = c->GetChildIndexB();
		const auto proxyIdA = fixtureA->m_proxies[indexA].proxyId;
		const auto proxyIdB = fixtureB->m_proxies[indexB].proxyId;
		const auto overlap = m_broadPhase.TestOverlap(proxyIdA, proxyIdB);

		// Here we destroy contacts that cease to overlap in the broad-phase.
		if (!overlap)
		{
			Destroy(c);
			continue;
		}

		// The contact persists.
		c->Update(m_contactListener);
	}
}

void b2ContactManager::FindNewContacts()
{
	m_broadPhase.UpdatePairs(this);
}

static bool IsFor(const b2Contact* contact,
				  const b2Fixture* fixtureA, int32 indexA, const b2Fixture* fixtureB, int32 indexB)
{
	const auto fA = contact->GetFixtureA();
	const auto fB = contact->GetFixtureB();
	const auto iA = contact->GetChildIndexA();
	const auto iB = contact->GetChildIndexB();
	
	if ((fA == fixtureA) && (fB == fixtureB) && (iA == indexA) && (iB == indexB))
	{
		// A contact already exists.
		return true;
	}
	
	if ((fA == fixtureB) && (fB == fixtureA) && (iA == indexB) && (iB == indexA))
	{
		// A contact already exists.
		return true;
	}
	
	return false;
}

void b2ContactManager::AddPair(void* proxyUserDataA, void* proxyUserDataB)
{
	const auto proxyA = static_cast<b2FixtureProxy*>(proxyUserDataA);
	const auto proxyB = static_cast<b2FixtureProxy*>(proxyUserDataB);

	auto fixtureA = proxyA->fixture;
	auto fixtureB = proxyB->fixture;

	const auto indexA = proxyA->childIndex;
	const auto indexB = proxyB->childIndex;

	auto bodyA = fixtureA->GetBody();
	auto bodyB = fixtureB->GetBody();

	// Are the fixtures on the same body?
	if (bodyA == bodyB)
	{
		return;
	}

	// TODO_ERIN use a hash table to remove a potential bottleneck when both
	// bodies have a lot of contacts.
	// Does a contact already exist?
	auto edge = bodyB->GetContactList();
	while (edge)
	{
		if (edge->other == bodyA)
		{
			if (IsFor(edge->contact, fixtureA, indexA, fixtureB, indexB))
				return;
		}

		edge = edge->next;
	}

	// Does a joint override collision? Is at least one body dynamic?
	if (!bodyB->ShouldCollide(bodyA))
	{
		return;
	}

	// Check user filtering.
	if (m_contactFilter && !m_contactFilter->ShouldCollide(fixtureA, fixtureB))
	{
		return;
	}

	// Call the factory.
	auto c = b2Contact::Create(fixtureA, indexA, fixtureB, indexB, m_allocator);
	if (c == nullptr)
	{
		return;
	}

	// Contact creation may swap fixtures.
	fixtureA = c->GetFixtureA();
	fixtureB = c->GetFixtureB();
	bodyA = fixtureA->GetBody();
	bodyB = fixtureB->GetBody();

	// Insert into the world.
	c->m_prev = nullptr;
	c->m_next = m_contactList;
	if (m_contactList != nullptr)
	{
		m_contactList->m_prev = c;
	}
	m_contactList = c;

	// Connect to island graph.

	// Connect to body A
	c->m_nodeA.contact = c;
	c->m_nodeA.other = bodyB;

	c->m_nodeA.prev = nullptr;
	c->m_nodeA.next = bodyA->m_contactList;
	if (bodyA->m_contactList != nullptr)
	{
		bodyA->m_contactList->prev = &c->m_nodeA;
	}
	bodyA->m_contactList = &c->m_nodeA;

	// Connect to body B
	c->m_nodeB.contact = c;
	c->m_nodeB.other = bodyA;

	c->m_nodeB.prev = nullptr;
	c->m_nodeB.next = bodyB->m_contactList;
	if (bodyB->m_contactList != nullptr)
	{
		bodyB->m_contactList->prev = &c->m_nodeB;
	}
	bodyB->m_contactList = &c->m_nodeB;

	// Wake up the bodies
	if (!fixtureA->IsSensor() && !fixtureB->IsSensor())
	{
		bodyA->SetAwake(true);
		bodyB->SetAwake(true);
	}

	++m_contactCount;
}
