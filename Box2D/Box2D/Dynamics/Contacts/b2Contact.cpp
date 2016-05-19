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

using namespace box2d;

using ContactCreateFcn = Contact* (Fixture* fixtureA, child_count_t indexA,
									   Fixture* fixtureB, child_count_t indexB,
									   BlockAllocator* allocator);
using ContactDestroyFcn = void (Contact* contact, BlockAllocator* allocator);

struct ContactRegister
{
	ContactCreateFcn* const createFcn;
	ContactDestroyFcn* const destroyFcn;
	const bool primary;
};

// Order dependent on Shape::Type enumeration values
static constexpr ContactRegister s_registers[Shape::e_typeCount][Shape::e_typeCount] =
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

Contact* Contact::Create(Fixture* fixtureA, child_count_t indexA,
							 Fixture* fixtureB, child_count_t indexB,
							 BlockAllocator* allocator)
{
	const auto type1 = fixtureA->GetType();
	const auto type2 = fixtureB->GetType();

	assert(0 <= type1 && type1 < Shape::e_typeCount);
	assert(0 <= type2 && type2 < Shape::e_typeCount);
	
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

void Contact::Destroy(Contact* contact, BlockAllocator* allocator)
{
	auto fixtureA = contact->m_fixtureA;
	auto fixtureB = contact->m_fixtureB;

	if ((contact->m_manifold.GetPointCount() > 0) &&
		!fixtureA->IsSensor() && !fixtureB->IsSensor())
	{
		fixtureA->GetBody()->SetAwake();
		fixtureB->GetBody()->SetAwake();
	}

	const auto typeA = fixtureA->GetType();
	const auto typeB = fixtureB->GetType();

	assert(0 <= typeA && typeB < Shape::e_typeCount);
	assert(0 <= typeA && typeB < Shape::e_typeCount);

	auto destroyFcn = s_registers[typeA][typeB].destroyFcn;
	destroyFcn(contact, allocator);
}

Contact::Contact(Fixture* fA, child_count_t indexA, Fixture* fB, child_count_t indexB) :
	m_fixtureA(fA), m_fixtureB(fB), m_indexA(indexA), m_indexB(indexB),
	m_friction(b2MixFriction(m_fixtureA->GetFriction(), m_fixtureB->GetFriction())),
	m_restitution(b2MixRestitution(m_fixtureA->GetRestitution(), m_fixtureB->GetRestitution()))
{
}

// Update the contact manifold and touching status.
// Note: do not assume the fixture AABBs are overlapping or are valid.
void Contact::Update(ContactListener* listener)
{
	const auto oldManifold = m_manifold;

	// Re-enable this contact.
	m_flags |= e_enabledFlag;

	auto touching = false;
	auto wasTouching = IsTouching();

	auto bodyA = m_fixtureA->GetBody();
	auto bodyB = m_fixtureB->GetBody();
	const auto xfA = bodyA->GetTransform();
	const auto xfB = bodyB->GetTransform();

	// Is this contact a sensor?
	const auto sensor = m_fixtureA->IsSensor() || m_fixtureB->IsSensor();
	if (sensor)
	{
		const auto shapeA = m_fixtureA->GetShape();
		const auto shapeB = m_fixtureB->GetShape();

		touching = b2TestOverlap(*shapeA, m_indexA, *shapeB, m_indexB, xfA, xfB);

		// Sensors don't generate manifolds.
		m_manifold = Manifold{};
	}
	else
	{
		m_manifold = Evaluate(xfA, xfB);
		
		const auto old_point_count = oldManifold.GetPointCount();
		const auto new_point_count = m_manifold.GetPointCount();

		touching = new_point_count > 0;

		// Match old contact ids to new contact ids and copy the
		// stored impulses to warm start the solver.
		for (auto i = decltype(new_point_count){0}; i < new_point_count; ++i)
		{
			auto& new_mp = m_manifold.GetPoint(i);
			new_mp.normalImpulse = float_t{0};
			new_mp.tangentImpulse = float_t{0};

			for (auto j = decltype(old_point_count){0}; j < old_point_count; ++j)
			{
				const auto& old_mp = oldManifold.GetPoint(j);
				if (new_mp.cf == old_mp.cf)
				{
					new_mp.normalImpulse = old_mp.normalImpulse;
					new_mp.tangentImpulse = old_mp.tangentImpulse;
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
		SetTouching();
	}
	else
	{
		UnsetTouching();
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

bool Contact::UpdateTOI()
{
	const auto fA = GetFixtureA();
	const auto fB = GetFixtureB();
	
	// Is there a sensor?
	if (fA->IsSensor() || fB->IsSensor())
	{
		return false;
	}
	
	const auto bA = fA->GetBody();
	const auto bB = fB->GetBody();
	
	const auto typeA = bA->m_type;
	const auto typeB = bB->m_type;
	assert((typeA == DynamicBody) || (typeB == DynamicBody));
	
	const auto activeA = bA->IsAwake() && (typeA != StaticBody);
	const auto activeB = bB->IsAwake() && (typeB != StaticBody);
	
	// Is at least one body active (awake and dynamic or kinematic)?
	if ((!activeA) && (!activeB))
	{
		return false;
	}
	
	const auto collideA = bA->IsBullet() || (typeA != DynamicBody);
	const auto collideB = bB->IsBullet() || (typeB != DynamicBody);
	
	// Are these two non-bullet dynamic bodies?
	if ((!collideA) && (!collideB))
	{
		return false;
	}
	
	// Compute the TOI for this contact.
	// Put the sweeps onto the same time interval.
	const auto maxAlpha0 = Max(bA->m_sweep.alpha0, bB->m_sweep.alpha0);
	assert(maxAlpha0 < float_t(1));
	bA->m_sweep.Advance(maxAlpha0);
	bB->m_sweep.Advance(maxAlpha0);
	
	// Compute the time of impact in interval [0, minTOI]
	b2TOIInput input;
	input.proxyA = b2DistanceProxy(*fA->GetShape(), GetChildIndexA());
	input.proxyB = b2DistanceProxy(*fB->GetShape(), GetChildIndexB());
	input.sweepA = bA->m_sweep;
	input.sweepB = bB->m_sweep;
	input.tMax = float_t(1);
	
	const auto output = b2TimeOfImpact(input);
	
	// Beta is the fraction of the remaining portion of the .
	const auto beta = output.get_t();
	const auto alpha = (output.get_state() == b2TOIOutput::e_touching)?
		Min(maxAlpha0 + (float_t{1} - maxAlpha0) * beta, float_t{1}): float_t{1};
	
	SetToi(alpha);
	
	return true;
}
