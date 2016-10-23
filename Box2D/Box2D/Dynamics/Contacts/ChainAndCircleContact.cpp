/*
* Original work Copyright (c) 2006-2010 Erin Catto http://www.box2d.org
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

#include <Box2D/Dynamics/Contacts/ChainAndCircleContact.h>
#include <Box2D/Common/BlockAllocator.h>
#include <Box2D/Dynamics/Fixture.h>
#include <Box2D/Collision/CollideShapes.hpp>
#include <Box2D/Collision/Shapes/ChainShape.h>
#include <Box2D/Collision/Shapes/EdgeShape.h>
#include <Box2D/Collision/Shapes/CircleShape.h>

#include <new>

using namespace box2d;

Contact* ChainAndCircleContact::Create(Fixture* fixtureA, child_count_t indexA,
										   Fixture* fixtureB, child_count_t indexB,
										   BlockAllocator* allocator)
{
	void* mem = allocator->Allocate(sizeof(ChainAndCircleContact));
	return new (mem) ChainAndCircleContact(fixtureA, indexA, fixtureB, indexB);
}

void ChainAndCircleContact::Destroy(Contact* contact, BlockAllocator* allocator)
{
	Delete(static_cast<ChainAndCircleContact*>(contact), *allocator);
}

ChainAndCircleContact::ChainAndCircleContact(Fixture* fixtureA, child_count_t indexA,
											 Fixture* fixtureB, child_count_t indexB)
: Contact{fixtureA, indexA, fixtureB, indexB}
{
	assert(GetType(*fixtureA) == Shape::e_chain);
	assert(GetType(*fixtureB) == Shape::e_circle);
}

Manifold ChainAndCircleContact::Evaluate(const Transformation& xfA, const Transformation& xfB) const
{
	const auto edge = (static_cast<const ChainShape*>(GetFixtureA()->GetShape()))->GetChildEdge(GetChildIndexA());
	return CollideShapes(edge, xfA, *static_cast<const CircleShape*>(GetFixtureB()->GetShape()), xfB);
}
