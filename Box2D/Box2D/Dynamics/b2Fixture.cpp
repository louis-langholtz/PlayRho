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

#include <Box2D/Dynamics/b2Fixture.h>
#include <Box2D/Dynamics/Contacts/b2Contact.h>
#include <Box2D/Dynamics/b2World.h>
#include <Box2D/Collision/Shapes/b2CircleShape.h>
#include <Box2D/Collision/Shapes/b2EdgeShape.h>
#include <Box2D/Collision/Shapes/b2PolygonShape.h>
#include <Box2D/Collision/Shapes/b2ChainShape.h>
#include <Box2D/Collision/b2BroadPhase.h>
#include <Box2D/Collision/b2Collision.h>
#include <Box2D/Common/b2BlockAllocator.h>

using namespace box2d;

void Fixture::Create(BlockAllocator* allocator, const FixtureDef* def)
{
	m_userData = def->userData;
	m_friction = def->friction;
	m_restitution = def->restitution;
	m_filter = def->filter;
	m_isSensor = def->isSensor;
	m_density = def->density;
	m_shape = def->shape->Clone(allocator);

	// Reserve proxy space
	const auto childCount = m_shape->GetChildCount();
	m_proxies = static_cast<FixtureProxy*>(allocator->Allocate(childCount * sizeof(FixtureProxy)));
	for (auto i = decltype(childCount){0}; i < childCount; ++i)
	{
		m_proxies[i].fixture = nullptr;
		m_proxies[i].proxyId = b2BroadPhase::e_nullProxy;
	}
}

void Fixture::Destroy(BlockAllocator* allocator)
{
	// The proxies must be destroyed before calling this.
	assert(m_proxyCount == 0);

	// Free the proxy array.
	const auto childCount = m_shape->GetChildCount();
	allocator->Free(m_proxies, childCount * sizeof(FixtureProxy));
	m_proxies = nullptr;

	// Free the child shape.
	switch (m_shape->GetType())
	{
	case Shape::e_circle:
		{
			auto s = static_cast<b2CircleShape*>(m_shape);
			s->~b2CircleShape();
			allocator->Free(s, sizeof(b2CircleShape));
		}
		break;

	case Shape::e_edge:
		{
			auto s = static_cast<b2EdgeShape*>(m_shape);
			s->~b2EdgeShape();
			allocator->Free(s, sizeof(b2EdgeShape));
		}
		break;

	case Shape::e_polygon:
		{
			auto s = static_cast<b2PolygonShape*>(m_shape);
			s->~b2PolygonShape();
			allocator->Free(s, sizeof(b2PolygonShape));
		}
		break;

	case Shape::e_chain:
		{
			auto s = static_cast<b2ChainShape*>(m_shape);
			s->~b2ChainShape();
			allocator->Free(s, sizeof(b2ChainShape));
		}
		break;

	default:
		assert(false);
		break;
	}

	m_shape = nullptr;
}

void Fixture::CreateProxies(b2BroadPhase& broadPhase, const Transform& xf)
{
	assert(m_proxyCount == 0);

	// Create proxies in the broad-phase.
	m_proxyCount = m_shape->GetChildCount();

	for (auto i = decltype(m_proxyCount){0}; i < m_proxyCount; ++i)
	{
		auto& proxy = m_proxies[i];
		proxy.aabb = m_shape->ComputeAABB(xf, i);
		proxy.proxyId = broadPhase.CreateProxy(proxy.aabb, &proxy);
		proxy.fixture = this;
		proxy.childIndex = i;
	}
}

void Fixture::DestroyProxies(b2BroadPhase& broadPhase)
{
	// Destroy proxies in the broad-phase.
	for (auto i = decltype(m_proxyCount){0}; i < m_proxyCount; ++i)
	{
		auto& proxy = m_proxies[i];
		broadPhase.DestroyProxy(proxy.proxyId);
		proxy.proxyId = b2BroadPhase::e_nullProxy;
	}

	m_proxyCount = 0;
}

void Fixture::Synchronize(b2BroadPhase& broadPhase, const Transform& transform1, const Transform& transform2)
{
	if (m_proxyCount == 0)
	{	
		return;
	}

	for (auto i = decltype(m_proxyCount){0}; i < m_proxyCount; ++i)
	{
		auto& proxy = m_proxies[i];

		// Compute an AABB that covers the swept shape (may miss some rotation effect).
		const auto aabb1 = m_shape->ComputeAABB(transform1, proxy.childIndex);
		const auto aabb2 = m_shape->ComputeAABB(transform2, proxy.childIndex);
		proxy.aabb = aabb1 + aabb2;

		const auto displacement = transform2.p - transform1.p;

		broadPhase.MoveProxy(proxy.proxyId, proxy.aabb, displacement);
	}
}

void Fixture::SetFilterData(const b2Filter& filter)
{
	m_filter = filter;

	Refilter();
}

void Fixture::Refilter()
{
	if (m_body == nullptr)
	{
		return;
	}

	// Flag associated contacts for filtering.
	auto edge = m_body->GetContactList();
	while (edge)
	{
		auto contact = edge->contact;
		const auto fixtureA = contact->GetFixtureA();
		const auto fixtureB = contact->GetFixtureB();
		if ((fixtureA == this) || (fixtureB == this))
		{
			contact->FlagForFiltering();
		}

		edge = edge->next;
	}

	auto world = m_body->GetWorld();

	if (world == nullptr)
	{
		return;
	}

	// Touch each proxy so that new pairs may be created
	auto broadPhase = &world->m_contactManager.m_broadPhase;
	for (auto i = decltype(m_proxyCount){0}; i < m_proxyCount; ++i)
	{
		broadPhase->TouchProxy(m_proxies[i].proxyId);
	}
}

void Fixture::SetSensor(bool sensor)
{
	if (sensor != m_isSensor)
	{
		m_body->SetAwake();
		m_isSensor = sensor;
	}
}

void Fixture::Dump(island_count_t bodyIndex)
{
	log("    FixtureDef fd;\n");
	log("    fd.friction = %.15lef;\n", m_friction);
	log("    fd.restitution = %.15lef;\n", m_restitution);
	log("    fd.density = %.15lef;\n", m_density);
	log("    fd.isSensor = bool(%d);\n", m_isSensor);
	log("    fd.filter.categoryBits = uint16(%d);\n", m_filter.categoryBits);
	log("    fd.filter.maskBits = uint16(%d);\n", m_filter.maskBits);
	log("    fd.filter.groupIndex = int16(%d);\n", m_filter.groupIndex);

	switch (m_shape->GetType())
	{
	case Shape::e_circle:
		{
			auto s = static_cast<b2CircleShape*>(m_shape);
			log("    b2CircleShape shape;\n");
			log("    shape.m_radius = %.15lef;\n", s->GetRadius());
			log("    shape.m_p = Vec2(%.15lef, %.15lef);\n", s->GetPosition().x, s->GetPosition().y);
		}
		break;

	case Shape::e_edge:
		{
			auto s = static_cast<b2EdgeShape*>(m_shape);
			log("    b2EdgeShape shape;\n");
			log("    shape.m_radius = %.15lef;\n", s->GetRadius());
			log("    shape.m_vertex0.Set(%.15lef, %.15lef);\n", s->GetVertex0().x, s->GetVertex0().y);
			log("    shape.m_vertex1.Set(%.15lef, %.15lef);\n", s->GetVertex1().x, s->GetVertex1().y);
			log("    shape.m_vertex2.Set(%.15lef, %.15lef);\n", s->GetVertex2().x, s->GetVertex2().y);
			log("    shape.m_vertex3.Set(%.15lef, %.15lef);\n", s->GetVertex3().x, s->GetVertex3().y);
			log("    shape.m_hasVertex0 = bool(%d);\n", s->HasVertex0());
			log("    shape.m_hasVertex3 = bool(%d);\n", s->HasVertex3());
		}
		break;

	case Shape::e_polygon:
		{
			auto s = static_cast<b2PolygonShape*>(m_shape);
			log("    b2PolygonShape shape;\n");
			log("    Vec2 vs[%d];\n", MaxPolygonVertices);
			for (auto i = decltype(s->GetVertexCount()){0}; i < s->GetVertexCount(); ++i)
			{
				log("    vs[%d].Set(%.15lef, %.15lef);\n", i, s->GetVertex(i).x, s->GetVertex(i).y);
			}
			log("    shape.Set(vs, %d);\n", s->GetVertexCount());
		}
		break;

	case Shape::e_chain:
		{
			auto s = static_cast<b2ChainShape*>(m_shape);
			log("    b2ChainShape shape;\n");
			log("    Vec2 vs[%d];\n", s->GetVertexCount());
			for (auto i = decltype(s->GetVertexCount()){0}; i < s->GetVertexCount(); ++i)
			{
				log("    vs[%d].Set(%.15lef, %.15lef);\n", i, s->GetVertex(i).x, s->GetVertex(i).y);
			}
			log("    shape.CreateChain(vs, %d);\n", s->GetVertexCount());
			log("    shape.m_prevVertex.Set(%.15lef, %.15lef);\n", s->GetPrevVertex().x, s->GetPrevVertex().y);
			log("    shape.m_nextVertex.Set(%.15lef, %.15lef);\n", s->GetNextVertex().x, s->GetNextVertex().y);
			log("    shape.m_hasPrevVertex = bool(%d);\n", s->HasPrevVertex());
			log("    shape.m_hasNextVertex = bool(%d);\n", s->HasNextVertex());
		}
		break;

	default:
		return;
	}

	log("\n");
	log("    fd.shape = &shape;\n");
	log("\n");
	log("    bodies[%d]->CreateFixture(&fd);\n", bodyIndex);
}
