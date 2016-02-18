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

#include <Box2D/Dynamics/Contacts/b2Contact.h>
#include <Box2D/Dynamics/Contacts/b2CircleContact.h>
#include <Box2D/Dynamics/Contacts/b2PolygonAndCircleContact.h>
#include <Box2D/Dynamics/Contacts/b2PolygonContact.h>
#include <Box2D/Dynamics/Contacts/b2EdgeAndCircleContact.h>
#include <Box2D/Dynamics/Contacts/b2EdgeAndPolygonContact.h>
#include <Box2D/Dynamics/Contacts/b2ChainAndCircleContact.h>
#include <Box2D/Dynamics/Contacts/b2ChainAndPolygonContact.h>
#include <Box2D/Dynamics/Contacts/b2ContactSolver.h>

#include <Box2D/Collision/b2Collision.h>
#include <Box2D/Collision/b2TimeOfImpact.h>
#include <Box2D/Collision/Shapes/b2Shape.h>
#include <Box2D/Common/b2BlockAllocator.h>
#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Dynamics/b2Fixture.h>
#include <Box2D/Dynamics/b2World.h>

using b2ContactCreateFcn = b2Contact* (b2Fixture* fixtureA, int32 indexA,
									   b2Fixture* fixtureB, int32 indexB,
									   b2BlockAllocator* allocator);
using b2ContactDestroyFcn = void (b2Contact* contact, b2BlockAllocator* allocator);

struct b2ContactRegister
{
	b2ContactCreateFcn* const createFcn;
	b2ContactDestroyFcn* const destroyFcn;
	const bool primary;
};

// Order dependent on b2Shape::Type enumeration values
static constexpr b2ContactRegister s_registers[b2Shape::e_typeCount][b2Shape::e_typeCount] =
{
	// circle-* contacts
	{
		{b2CircleContact::Create, b2CircleContact::Destroy, true}, // circle
		{b2EdgeAndCircleContact::Create, b2EdgeAndCircleContact::Destroy, false}, // edge
		{b2PolygonAndCircleContact::Create, b2PolygonAndCircleContact::Destroy, false}, // polygon
		{b2ChainAndCircleContact::Create, b2ChainAndCircleContact::Destroy, false}, // chain
	},
	// edge-* contacts
	{
		{b2EdgeAndCircleContact::Create, b2EdgeAndCircleContact::Destroy, true}, // circle
		{nullptr, nullptr, false}, // edge
		{b2EdgeAndPolygonContact::Create, b2EdgeAndPolygonContact::Destroy, true}, // polygon
		{nullptr, nullptr, false}, // chain
	},
	// polygon-* contacts
	{
		{b2PolygonAndCircleContact::Create, b2PolygonAndCircleContact::Destroy, true}, // circle
		{b2EdgeAndPolygonContact::Create, b2EdgeAndPolygonContact::Destroy, false}, // edge
		{b2PolygonContact::Create, b2PolygonContact::Destroy, true}, // polygon
		{nullptr, nullptr, false}, // chain
	},
	// chain-* contacts
	{
		{b2ChainAndCircleContact::Create, b2ChainAndCircleContact::Destroy, true}, // circle
		{nullptr, nullptr, false}, // edge
		{b2ChainAndPolygonContact::Create, b2ChainAndPolygonContact::Destroy, true}, // polygon
		{nullptr, nullptr, false}, // chain
	},
};

b2Contact* b2Contact::Create(b2Fixture* fixtureA, int32 indexA, b2Fixture* fixtureB, int32 indexB, b2BlockAllocator* allocator)
{
	const auto type1 = fixtureA->GetType();
	const auto type2 = fixtureB->GetType();

	b2Assert(0 <= type1 && type1 < b2Shape::e_typeCount);
	b2Assert(0 <= type2 && type2 < b2Shape::e_typeCount);
	
	auto createFcn = s_registers[type1][type2].createFcn;
	if (createFcn)
	{
		if (s_registers[type1][type2].primary)
		{
			return createFcn(fixtureA, indexA, fixtureB, indexB, allocator);
		}
		else
		{
			return createFcn(fixtureB, indexB, fixtureA, indexA, allocator);
		}
	}
	else
	{
		return nullptr;
	}
}

void b2Contact::Destroy(b2Contact* contact, b2BlockAllocator* allocator)
{
	auto fixtureA = contact->m_fixtureA;
	auto fixtureB = contact->m_fixtureB;

	if (contact->m_manifold.pointCount > 0 &&
		!fixtureA->IsSensor() && !fixtureB->IsSensor())
	{
		fixtureA->GetBody()->SetAwake();
		fixtureB->GetBody()->SetAwake();
	}

	const auto typeA = fixtureA->GetType();
	const auto typeB = fixtureB->GetType();

	b2Assert(0 <= typeA && typeB < b2Shape::e_typeCount);
	b2Assert(0 <= typeA && typeB < b2Shape::e_typeCount);

	auto destroyFcn = s_registers[typeA][typeB].destroyFcn;
	destroyFcn(contact, allocator);
}

b2Contact::b2Contact(b2Fixture* fA, int32 indexA, b2Fixture* fB, int32 indexB) :
	m_fixtureA(fA), m_fixtureB(fB), m_indexA(indexA), m_indexB(indexB),
	m_friction(b2MixFriction(m_fixtureA->m_friction, m_fixtureB->m_friction)),
	m_restitution(b2MixRestitution(m_fixtureA->m_restitution, m_fixtureB->m_restitution))
{
	m_manifold.pointCount = 0;
}

// Update the contact manifold and touching status.
// Note: do not assume the fixture AABBs are overlapping or are valid.
void b2Contact::Update(b2ContactListener* listener)
{
	const auto oldManifold = m_manifold;

	// Re-enable this contact.
	m_flags |= e_enabledFlag;

	auto touching = false;
	auto wasTouching = (m_flags & e_touchingFlag) == e_touchingFlag;

	auto bodyA = m_fixtureA->GetBody();
	auto bodyB = m_fixtureB->GetBody();
	const b2Transform& xfA = bodyA->GetTransform();
	const b2Transform& xfB = bodyB->GetTransform();

	// Is this contact a sensor?
	const auto sensor = m_fixtureA->IsSensor() || m_fixtureB->IsSensor();
	if (sensor)
	{
		const auto shapeA = m_fixtureA->GetShape();
		const auto shapeB = m_fixtureB->GetShape();
		touching = b2TestOverlap(shapeA, m_indexA, shapeB, m_indexB, xfA, xfB);

		// Sensors don't generate manifolds.
		m_manifold.pointCount = 0;
	}
	else
	{
		Evaluate(&m_manifold, xfA, xfB);
		touching = m_manifold.pointCount > 0;

		// Match old contact ids to new contact ids and copy the
		// stored impulses to warm start the solver.
		for (auto i = decltype(m_manifold.pointCount){0}; i < m_manifold.pointCount; ++i)
		{
			auto mp2 = m_manifold.points + i;
			mp2->normalImpulse = 0.0f;
			mp2->tangentImpulse = 0.0f;
			const auto id2 = mp2->id;

			for (auto j = decltype(oldManifold.pointCount){0}; j < oldManifold.pointCount; ++j)
			{
				const auto mp1 = oldManifold.points + j;

				if (mp1->id.key == id2.key)
				{
					mp2->normalImpulse = mp1->normalImpulse;
					mp2->tangentImpulse = mp1->tangentImpulse;
					break;
				}
			}
		}

		if (touching != wasTouching)
		{
			bodyA->SetAwake();
			bodyB->SetAwake();
		}
	}

	if (touching)
	{
		m_flags |= e_touchingFlag;
	}
	else
	{
		m_flags &= ~e_touchingFlag;
	}

	if (!wasTouching && touching && listener)
	{
		listener->BeginContact(this);
	}

	if (wasTouching && !touching && listener)
	{
		listener->EndContact(this);
	}

	if (!sensor && touching && listener)
	{
		listener->PreSolve(this, &oldManifold);
	}
}
