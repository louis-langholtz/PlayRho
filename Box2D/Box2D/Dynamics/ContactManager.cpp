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

#include <Box2D/Dynamics/ContactManager.h>
#include <Box2D/Dynamics/Body.h>
#include <Box2D/Dynamics/Fixture.h>
#include <Box2D/Dynamics/WorldCallbacks.h>
#include <Box2D/Dynamics/Contacts/Contact.h>

using namespace box2d;

ContactManager::ContactManager(BlockAllocator* allocator, ContactFilter* filter, ContactListener* listener):
	m_allocator(allocator),
	m_contactFilter(filter), m_contactListener(listener)
{}

void ContactManager::Destroy(Contact* c)
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

	// Call the factory destroy method.
	Contact::Destroy(c, m_allocator);
	
	assert(m_contactCount > 0);
	--m_contactCount;
}

void ContactManager::Collide()
{
	// Update awake contacts.
	auto next = static_cast<Contact*>(nullptr);
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

		const bool activeA = bodyA->IsAwake() && (bodyA->m_type != StaticBody);
		const bool activeB = bodyB->IsAwake() && (bodyB->m_type != StaticBody);

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

void ContactManager::FindNewContacts()
{
	// Here m_broadPhase will call this object's ContactManager::AddPair method.
	m_broadPhase.UpdatePairs(this);
}

static bool IsFor(const Contact& contact,
				  const Fixture* fixtureA, ContactManager::size_type indexA,
				  const Fixture* fixtureB, ContactManager::size_type indexB)
{
	const auto fA = contact.GetFixtureA();
	const auto fB = contact.GetFixtureB();
	const auto iA = contact.GetChildIndexA();
	const auto iB = contact.GetChildIndexB();
	
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

void ContactManager::AddPair(void* proxyUserDataA, void* proxyUserDataB)
{
	const auto proxyA = static_cast<FixtureProxy*>(proxyUserDataA);
	const auto proxyB = static_cast<FixtureProxy*>(proxyUserDataB);

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
	auto contactEdge = bodyB->GetContactList();
	while (contactEdge)
	{
		if (contactEdge->other == bodyA)
		{
			if (IsFor(*(contactEdge->contact), fixtureA, indexA, fixtureB, indexB))
				return;
		}
		contactEdge = contactEdge->next;
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

	// Call the contact factory create method.
	auto c = Contact::Create(fixtureA, indexA, fixtureB, indexB, m_allocator);
	assert(c);
	if (!c)
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
	if (m_contactList)
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
	if (bodyA->m_contactList)
	{
		bodyA->m_contactList->prev = &c->m_nodeA;
	}
	bodyA->m_contactList = &c->m_nodeA;

	// Connect to body B
	c->m_nodeB.contact = c;
	c->m_nodeB.other = bodyA;

	c->m_nodeB.prev = nullptr;
	c->m_nodeB.next = bodyB->m_contactList;
	if (bodyB->m_contactList)
	{
		bodyB->m_contactList->prev = &c->m_nodeB;
	}
	bodyB->m_contactList = &c->m_nodeB;

	// Wake up the bodies
	if (!fixtureA->IsSensor() && !fixtureB->IsSensor())
	{
		bodyA->SetAwake();
		bodyB->SetAwake();
	}

	++m_contactCount;
}
