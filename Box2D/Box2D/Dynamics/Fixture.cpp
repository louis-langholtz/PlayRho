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

#include <Box2D/Dynamics/Fixture.h>
#include <Box2D/Dynamics/FixtureProxy.hpp>
#include <Box2D/Dynamics/Contacts/Contact.h>
#include <Box2D/Dynamics/World.h>
#include <Box2D/Collision/Shapes/CircleShape.h>
#include <Box2D/Collision/Shapes/EdgeShape.h>
#include <Box2D/Collision/Shapes/PolygonShape.h>
#include <Box2D/Collision/Shapes/ChainShape.h>
#include <Box2D/Collision/BroadPhase.h>
#include <Box2D/Common/BlockAllocator.h>

using namespace box2d;

const AABB& Fixture::GetAABB(child_count_t index) const
{
	assert(index < m_proxyCount);
	return m_proxies[index].aabb;
}

const FixtureProxy* Fixture::GetProxy(child_count_t index) const
{
	assert(index < m_proxyCount);
	return (index < m_proxyCount)? m_proxies + index: nullptr;
}

void Fixture::CreateProxies(BlockAllocator& allocator, BroadPhase& broadPhase, const Transformation& xf)
{
	assert(m_proxyCount == 0);
	assert(m_proxies == nullptr);

	const auto shape = GetShape();

	// Reserve proxy space and create proxies in the broad-phase.
	const auto childCount = GetChildCount(*shape);
	const auto proxies = allocator.AllocateArray<FixtureProxy>(childCount);
	const auto aabbExtension = GetAabbExtension(*(GetBody()->GetWorld()));
	const auto extension = Vec2{aabbExtension, aabbExtension};
	for (auto i = decltype(childCount){0}; i < childCount; ++i)
	{
		const auto aabb = ComputeAABB(*shape, xf, i);
		new (proxies + i) FixtureProxy{aabb, broadPhase.CreateProxy(aabb + extension, proxies + i), this, i};
	}
	m_proxies = proxies;
	m_proxyCount = childCount;
}

void Fixture::DestroyProxies(BlockAllocator& allocator, BroadPhase& broadPhase)
{
	// Destroy proxies in the broad-phase.
	const auto childCount = m_proxyCount;
	const auto proxies = m_proxies;
	for (auto i = decltype(childCount){0}; i < childCount; ++i)
	{
		broadPhase.DestroyProxy(proxies[i].proxyId);
		proxies[i].~FixtureProxy();
	}
	allocator.Free(proxies, childCount * sizeof(FixtureProxy));
	m_proxyCount = 0;
	m_proxies = nullptr;
}

void Fixture::TouchProxies(BroadPhase& broadPhase)
{
	for (auto i = decltype(m_proxyCount){0}; i < m_proxyCount; ++i)
	{
		broadPhase.TouchProxy(m_proxies[i].proxyId);
	}
}

void Fixture::Synchronize(BroadPhase& broadPhase, const Transformation& transform1, const Transformation& transform2)
{
	assert(IsValid(transform1));
	assert(IsValid(transform2));

	const auto shape = GetShape();
	const auto aabbExtension = GetAabbExtension(*(GetBody()->GetWorld()));
	const auto extension = Vec2{aabbExtension, aabbExtension};

	for (auto i = decltype(m_proxyCount){0}; i < m_proxyCount; ++i)
	{
		auto& proxy = m_proxies[i];

		// Compute an AABB that covers the swept shape (may miss some rotation effect).
		const auto aabb1 = ComputeAABB(*shape, transform1, proxy.childIndex);
		const auto aabb2 = ComputeAABB(*shape, transform2, proxy.childIndex);
		proxy.aabb = aabb1 + aabb2;

		broadPhase.MoveProxy(proxy.proxyId, proxy.aabb + extension, transform2.p - transform1.p);
	}
}

void Fixture::SetFilterData(const Filter& filter)
{
	m_filter = filter;

	Refilter();
}

void Fixture::Refilter()
{
	const auto body = GetBody();
	if (body)
	{
		// Flag associated contacts for filtering.
		for (auto&& edge: body->GetContactEdges())
		{
			auto contact = edge.contact;
			const auto fixtureA = contact->GetFixtureA();
			const auto fixtureB = contact->GetFixtureB();
			if ((fixtureA == this) || (fixtureB == this))
			{
				contact->FlagForFiltering();
			}
		}
		
		const auto world = body->GetWorld();
		if (world)
		{
			TouchProxies(world->m_contactMgr.m_broadPhase);
		}
	}
}

void Fixture::SetSensor(bool sensor)
{
	if (sensor != m_isSensor)
	{
		m_isSensor = sensor;
		m_body->SetAwake();
	}
}

void box2d::Dump(const Fixture& fixture, size_t bodyIndex)
{
	log("    FixtureDef fd;\n");
	log("    fd.friction = %.15lef;\n", fixture.GetFriction());
	log("    fd.restitution = %.15lef;\n", fixture.GetRestitution());
	log("    fd.density = %.15lef;\n", fixture.GetDensity());
	log("    fd.isSensor = bool(%d);\n", fixture.IsSensor());
	log("    fd.filter.categoryBits = uint16(%d);\n", fixture.GetFilterData().categoryBits);
	log("    fd.filter.maskBits = uint16(%d);\n", fixture.GetFilterData().maskBits);
	log("    fd.filter.groupIndex = int16(%d);\n", fixture.GetFilterData().groupIndex);

	switch (fixture.GetShape()->GetType())
	{
	case Shape::e_circle:
		{
			auto s = static_cast<const CircleShape*>(fixture.GetShape());
			log("    CircleShape shape;\n");
			log("    shape.m_radius = %.15lef;\n", s->GetRadius());
			log("    shape.m_p = Vec2(%.15lef, %.15lef);\n", s->GetPosition().x, s->GetPosition().y);
		}
		break;

	case Shape::e_edge:
		{
			auto s = static_cast<const EdgeShape*>(fixture.GetShape());
			log("    EdgeShape shape;\n");
			log("    shape.m_radius = %.15lef;\n", GetVertexRadius(*s));
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
			auto s = static_cast<const PolygonShape*>(fixture.GetShape());
			log("    PolygonShape shape;\n");
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
			auto s = static_cast<const ChainShape*>(fixture.GetShape());
			log("    ChainShape shape;\n");
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
	log("    bodies[%d]->CreateFixture(fd);\n", bodyIndex);
}
