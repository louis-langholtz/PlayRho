/*
* Copyright (c) 2006-2010 Erin Catto http://www.box2d.org
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

#include <Box2D/Dynamics/Contacts/b2ChainAndCircleContact.h>
#include <Box2D/Common/b2BlockAllocator.h>
#include <Box2D/Dynamics/b2Fixture.h>
#include <Box2D/Collision/Shapes/b2ChainShape.h>
#include <Box2D/Collision/Shapes/b2EdgeShape.h>
#include <Box2D/Collision/Shapes/b2CircleShape.h>

#include <new>

b2Contact* b2ChainAndCircleContact::Create(b2Fixture* fixtureA, child_count_t indexA,
										   b2Fixture* fixtureB, child_count_t indexB,
										   b2BlockAllocator* allocator)
{
	void* mem = allocator->Allocate(sizeof(b2ChainAndCircleContact));
	return new (mem) b2ChainAndCircleContact(fixtureA, indexA, fixtureB, indexB);
}

void b2ChainAndCircleContact::Destroy(b2Contact* contact, b2BlockAllocator* allocator)
{
	(static_cast<b2ChainAndCircleContact*>(contact))->~b2ChainAndCircleContact();
	allocator->Free(contact, sizeof(b2ChainAndCircleContact));
}

b2ChainAndCircleContact::b2ChainAndCircleContact(b2Fixture* fixtureA, child_count_t indexA,
												 b2Fixture* fixtureB, child_count_t indexB)
: b2Contact(fixtureA, indexA, fixtureB, indexB)
{
	b2Assert(m_fixtureA->GetType() == b2Shape::e_chain);
	b2Assert(m_fixtureB->GetType() == b2Shape::e_circle);
}

b2Manifold b2ChainAndCircleContact::Evaluate(const b2Transform& xfA, const b2Transform& xfB)
{
	auto chain = static_cast<b2ChainShape*>(m_fixtureA->GetShape());
	b2EdgeShape edge;
	chain->GetChildEdge(&edge, m_indexA);
	return b2CollideShapes(edge, xfA, *static_cast<b2CircleShape*>(m_fixtureB->GetShape()), xfB);
}
