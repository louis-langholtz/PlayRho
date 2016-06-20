/*
* Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
* Modified work Copyright (c) 2016 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

#include <Box2D/Dynamics/Contacts/PolygonContact.h>
#include <Box2D/Common/BlockAllocator.h>
#include <Box2D/Dynamics/Body.h>
#include <Box2D/Dynamics/Fixture.h>
#include <Box2D/Dynamics/WorldCallbacks.h>
#include <Box2D/Collision/Shapes/PolygonShape.h>

#include <new>

using namespace box2d;

Contact* PolygonContact::Create(Fixture* fixtureA, child_count_t,
									Fixture* fixtureB, child_count_t,
									BlockAllocator* allocator)
{
	void* mem = allocator->Allocate(sizeof(PolygonContact));
	return new (mem) PolygonContact(fixtureA, fixtureB);
}

void PolygonContact::Destroy(Contact* contact, BlockAllocator* allocator)
{
	(static_cast<PolygonContact*>(contact))->~PolygonContact();
	allocator->Free(contact, sizeof(PolygonContact));
}

PolygonContact::PolygonContact(Fixture* fixtureA, Fixture* fixtureB)
	: Contact(fixtureA, 0, fixtureB, 0)
{
	assert(m_fixtureA->GetType() == Shape::e_polygon);
	assert(m_fixtureB->GetType() == Shape::e_polygon);
}

Manifold PolygonContact::Evaluate(const Transform& xfA, const Transform& xfB)
{
	return CollideShapes(*static_cast<PolygonShape*>(m_fixtureA->GetShape()), xfA, *static_cast<PolygonShape*>(m_fixtureB->GetShape()), xfB);
}
