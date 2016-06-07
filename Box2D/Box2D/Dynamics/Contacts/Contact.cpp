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

#include <Box2D/Dynamics/Contacts/Contact.h>
#include <Box2D/Dynamics/Contacts/CircleContact.h>
#include <Box2D/Dynamics/Contacts/PolygonAndCircleContact.h>
#include <Box2D/Dynamics/Contacts/PolygonContact.h>
#include <Box2D/Dynamics/Contacts/EdgeAndCircleContact.h>
#include <Box2D/Dynamics/Contacts/EdgeAndPolygonContact.h>
#include <Box2D/Dynamics/Contacts/ChainAndCircleContact.h>
#include <Box2D/Dynamics/Contacts/ChainAndPolygonContact.h>
#include <Box2D/Dynamics/Contacts/ContactSolver.h>

#include <Box2D/Collision/Collision.h>
#include <Box2D/Collision/TimeOfImpact.h>
#include <Box2D/Collision/Shapes/Shape.h>
#include <Box2D/Common/BlockAllocator.h>
#include <Box2D/Dynamics/Body.h>
#include <Box2D/Dynamics/Fixture.h>
#include <Box2D/Dynamics/World.h>

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
		{CircleContact::Create, CircleContact::Destroy, true}, // circle
		{EdgeAndCircleContact::Create, EdgeAndCircleContact::Destroy, false}, // edge
		{PolygonAndCircleContact::Create, PolygonAndCircleContact::Destroy, false}, // polygon
		{ChainAndCircleContact::Create, ChainAndCircleContact::Destroy, false}, // chain
	},
	// edge-* contacts
	{
		{EdgeAndCircleContact::Create, EdgeAndCircleContact::Destroy, true}, // circle
		{nullptr, nullptr, false}, // edge
		{EdgeAndPolygonContact::Create, EdgeAndPolygonContact::Destroy, true}, // polygon
		{nullptr, nullptr, false}, // chain
	},
	// polygon-* contacts
	{
		{PolygonAndCircleContact::Create, PolygonAndCircleContact::Destroy, true}, // circle
		{EdgeAndPolygonContact::Create, EdgeAndPolygonContact::Destroy, false}, // edge
		{PolygonContact::Create, PolygonContact::Destroy, true}, // polygon
		{nullptr, nullptr, false}, // chain
	},
	// chain-* contacts
	{
		{ChainAndCircleContact::Create, ChainAndCircleContact::Destroy, true}, // circle
		{nullptr, nullptr, false}, // edge
		{ChainAndPolygonContact::Create, ChainAndPolygonContact::Destroy, true}, // polygon
		{nullptr, nullptr, false}, // chain
	},
};

Contact* Contact::Create(Fixture* fixtureA, child_count_t indexA, Fixture* fixtureB, child_count_t indexB,
						 BlockAllocator* allocator)
{
	const auto type1 = fixtureA->GetType();
	const auto type2 = fixtureB->GetType();

	assert(0 <= type1 && type1 < Shape::e_typeCount);
	assert(0 <= type2 && type2 < Shape::e_typeCount);
	
	const auto createFcn = s_registers[type1][type2].createFcn;
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
	const auto fixtureA = contact->m_fixtureA;
	const auto fixtureB = contact->m_fixtureB;

	if ((contact->m_manifold.GetPointCount() > 0) &&
		!fixtureA->IsSensor() && !fixtureB->IsSensor())
	{
		SetAwake(*fixtureA);
		SetAwake(*fixtureB);
	}

	const auto typeA = fixtureA->GetType();
	const auto typeB = fixtureB->GetType();

	assert(0 <= typeA && typeB < Shape::e_typeCount);
	assert(0 <= typeA && typeB < Shape::e_typeCount);

	const auto destroyFcn = s_registers[typeA][typeB].destroyFcn;
	destroyFcn(contact, allocator);
}

Contact::Contact(Fixture* fA, child_count_t indexA, Fixture* fB, child_count_t indexB) :
	m_fixtureA{fA}, m_fixtureB{fB}, m_indexA{indexA}, m_indexB{indexB},
	m_friction{MixFriction(fA->GetFriction(), fB->GetFriction())},
	m_restitution{MixRestitution(fA->GetRestitution(), fB->GetRestitution())}
{
}

void Contact::Update(ContactListener* listener)
{
	const auto oldManifold = m_manifold;

	// Re-enable this contact.
	m_flags |= e_enabledFlag;

	// Note: do not assume the fixture AABBs are overlapping or are valid.
	auto touching = false;
	auto wasTouching = IsTouching();

	const auto bodyA = m_fixtureA->GetBody();
	const auto bodyB = m_fixtureB->GetBody();
	
	assert(bodyA != nullptr);
	assert(bodyB != nullptr);

	const auto xfA = bodyA->GetTransform();
	const auto xfB = bodyB->GetTransform();

	// Is this contact a sensor?
	const auto sensor = HasSensor();
	if (sensor)
	{
		const auto shapeA = m_fixtureA->GetShape();
		const auto shapeB = m_fixtureB->GetShape();

		touching = TestOverlap(*shapeA, m_indexA, *shapeB, m_indexB, xfA, xfB);

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
				if (new_mp.contactFeature == old_mp.contactFeature)
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

	if (listener)
	{
		if (!wasTouching && touching)
		{
			listener->BeginContact(this);
		}

		if (wasTouching && !touching)
		{
			listener->EndContact(this);
		}

		if (!sensor && touching)
		{
			listener->PreSolve(this, &oldManifold);
		}
	}
}

static inline bool IsValidForTime(TOIOutput::State state)
{
	return state == TOIOutput::e_touching;
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
	assert((typeA == BodyType::Dynamic) || (typeB == BodyType::Dynamic));
	
	const auto activeA = bA->IsAwake() && (typeA != BodyType::Static);
	const auto activeB = bB->IsAwake() && (typeB != BodyType::Static);
	
	// Is at least one body active (awake and dynamic or kinematic)?
	if ((!activeA) && (!activeB))
	{
		return false;
	}
	
	const auto collideA = bA->IsBullet() || (typeA != BodyType::Dynamic);
	const auto collideB = bB->IsBullet() || (typeB != BodyType::Dynamic);
	
	// Are these two non-bullet dynamic bodies?
	if ((!collideA) && (!collideB))
	{
		return false;
	}
	
	// Compute the TOI for this contact.
	// Put the sweeps onto the same time interval.
	const auto maxAlpha0 = Max(bA->m_sweep.GetAlpha0(), bB->m_sweep.GetAlpha0());
	assert(maxAlpha0 < float_t{1});
	bA->m_sweep.Advance(maxAlpha0);
	bB->m_sweep.Advance(maxAlpha0);
	
	// Compute the time of impact in interval [0, 1]
	const auto output = TimeOfImpact(GetDistanceProxy(*fA->GetShape(), GetChildIndexA()), bA->m_sweep,
									 GetDistanceProxy(*fB->GetShape(), GetChildIndexB()), bB->m_sweep);
	
	const auto alpha = IsValidForTime(output.get_state())?
		Min(maxAlpha0 + (float_t{1} - maxAlpha0) * output.get_t(), float_t{1}): float_t{1};
	SetToi(alpha);
	
	return true;
}
