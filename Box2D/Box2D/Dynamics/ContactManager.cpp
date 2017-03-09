/*
* Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
* Modified work Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

#include <Box2D/Dynamics/ContactManager.hpp>
#include <Box2D/Dynamics/Body.hpp>
#include <Box2D/Dynamics/Fixture.hpp>
#include <Box2D/Dynamics/FixtureProxy.hpp>
#include <Box2D/Dynamics/WorldCallbacks.hpp>
#include <Box2D/Dynamics/Contacts/Contact.hpp>

using namespace box2d;

void ContactManager::Remove(Contact* c)
{
	assert(c);

	// Remove from the world.
	assert(!m_contacts.empty());
	
	for (auto iter = m_contacts.begin(); iter != m_contacts.end(); ++iter)
	{
		if (*iter == c)
		{
			m_contacts.erase(iter);
			break;
		}
	}
	
	const auto fixtureA = c->GetFixtureA();
	const auto fixtureB = c->GetFixtureB();
	const auto bodyA = fixtureA->GetBody();
	const auto bodyB = fixtureB->GetBody();
	
	bodyA->m_contacts.erase(c);
	bodyB->m_contacts.erase(c);
}

void ContactManager::Destroy(Contact* c)
{
	if (m_contactListener && c->IsTouching())
	{
		// EndContact hadn't been called in Collide() since is-touching, so call it now
		m_contactListener->EndContact(*c);
	}

	Remove(c);

	// Call the factory destroy method.
	Contact::Destroy(c, m_allocator);
}

ContactManager::CollideStats ContactManager::Collide()
{
	auto stats = CollideStats{};

	// Update awake contacts.
	auto next = m_contacts.begin();
	for (auto iter = m_contacts.begin(); iter != m_contacts.end(); iter = next)
	{
		const auto c = *iter;
		next = std::next(iter);

		const auto fixtureA = c->GetFixtureA();
		const auto fixtureB = c->GetFixtureB();
		const auto bodyA = fixtureA->GetBody();
		const auto bodyB = fixtureB->GetBody();
		 
		// Is this contact flagged for filtering?
		if (c->NeedsFiltering())
		{
			// Can these bodies collide?
			if (!(bodyB->ShouldCollide(bodyA)))
			{
				Destroy(c);
				++stats.destroyed;
				continue;
			}

			// Check user filtering.
			if (m_contactFilter && !(m_contactFilter->ShouldCollide(fixtureA, fixtureB)))
			{
				Destroy(c);
				++stats.destroyed;
				continue;
			}

			// Clear the filtering flag.
			c->UnflagForFiltering();
		}

		// collidable means is-awake && is-speedable (dynamic or kinematic)
		auto is_collidable = [&](Body* b) {
			constexpr auto awake_and_speedable = Body::e_awakeFlag|Body::e_velocityFlag;
			return (b->m_flags & awake_and_speedable) == awake_and_speedable;
		};

		// At least one body must be collidable
		if (!is_collidable(bodyA) && !is_collidable(bodyB))
		{
			++stats.ignored;
			continue;
		}

		const auto overlap = [&]() {
			const auto indexA = c->GetChildIndexA();
			const auto indexB = c->GetChildIndexB();
			const auto proxyIdA = fixtureA->m_proxies[indexA].proxyId;
			const auto proxyIdB = fixtureB->m_proxies[indexB].proxyId;
			return TestOverlap(m_broadPhase, proxyIdA, proxyIdB);
		}();

		// Here we destroy contacts that cease to overlap in the broad-phase.
		if (!overlap)
		{
			Destroy(c);
			++stats.destroyed;
			continue;
		}

		// The contact persists.

		// Update the contact manifold and notify the listener.
		c->SetEnabled();
		c->Update(m_contactListener);
		++stats.updated;
	}
	
	return stats;
}

contact_count_t ContactManager::FindNewContacts()
{
	// Here m_broadPhase will call this object's ContactManager::AddPair method.
	return m_broadPhase.UpdatePairs(this);
}

static inline bool IsFor(const Contact& contact,
						 const Fixture* fixtureA, child_count_t indexA,
						 const Fixture* fixtureB, child_count_t indexB)
{
	const auto fA = contact.GetFixtureA();
	const auto fB = contact.GetFixtureB();
	const auto iA = contact.GetChildIndexA();
	const auto iB = contact.GetChildIndexB();
	
	return
		((fA == fixtureA) && (fB == fixtureB) && (iA == indexA) && (iB == indexB)) ||
		((fA == fixtureB) && (fB == fixtureA) && (iA == indexB) && (iB == indexA));
}

bool ContactManager::Add(const FixtureProxy& proxyA, const FixtureProxy& proxyB)
{
	const auto fixtureA = proxyA.fixture; ///< Fixture of proxyA (but may get switched with fixtureB).
	const auto fixtureB = proxyB.fixture; ///< Fixture of proxyB (but may get switched with fixtureA).

	const auto bodyA = fixtureA->GetBody();
	const auto bodyB = fixtureB->GetBody();

	// Are the fixtures on the same body?
	if (bodyA == bodyB)
	{
		return false;
	}

	const auto childIndexA = proxyA.childIndex;
	const auto childIndexB = proxyB.childIndex;
	
	// TODO: use a hash table to remove a potential bottleneck when both
	// bodies have a lot of contacts.
	// Does a contact already exist?
	for (auto&& contact: bodyB->m_contacts)
	{
		if (IsFor(*contact, fixtureA, childIndexA, fixtureB, childIndexB))
		{
			// Already have a contact for proxyA with proxyB, bail!
			return false;
		}
	}

	// Does a joint override collision? Is at least one body dynamic?
	if (!bodyB->ShouldCollide(bodyA))
	{
		return false;
	}

	// Check user filtering.
	if (m_contactFilter && !m_contactFilter->ShouldCollide(fixtureA, fixtureB))
	{
		return false;
	}

	assert(GetContacts().size() < MaxContacts);

	// Call the contact factory create method.
	const auto c = Contact::Create(*fixtureA, childIndexA, *fixtureB, childIndexB, m_allocator);
	assert(c);
	if (!c)
	{
		return false;
	}
	
	Add(c);
	return true;
}

void ContactManager::Add(Contact* c)
{
	// Contact creation may swap fixtures.
	const auto fixtureA = c->GetFixtureA();
	const auto fixtureB = c->GetFixtureB();
	const auto bodyA = fixtureA->GetBody();
	const auto bodyB = fixtureB->GetBody();

	// Connect to island graph.

	bodyA->m_contacts.insert(c);
	bodyB->m_contacts.insert(c);

	// Wake up the bodies
	if (!fixtureA->IsSensor() && !fixtureB->IsSensor())
	{
		bodyA->SetAwake();
		bodyB->SetAwake();
	}
	
	// Insert into the world.
	m_contacts.push_front(c);
}
