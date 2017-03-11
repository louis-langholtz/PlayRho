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

#include <Box2D/Dynamics/Fixture.hpp>
#include <Box2D/Dynamics/FixtureProxy.hpp>
#include <Box2D/Dynamics/Contacts/Contact.hpp>
#include <Box2D/Dynamics/World.hpp>
#include <Box2D/Dynamics/Body.hpp>
#include <Box2D/Collision/BroadPhase.hpp>
#include <Box2D/Common/BlockAllocator.hpp>

using namespace box2d;

AABB Fixture::GetAABB(child_count_t index) const noexcept
{
	assert(index < m_proxyCount);
	return m_proxies[index].aabb;
}

const FixtureProxy* Fixture::GetProxy(child_count_t index) const noexcept
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
	const auto aabbExtension = GetBody()->GetWorld()->GetAabbExtension();
	const auto extension = Vec2{aabbExtension, aabbExtension};
	for (auto childIndex = decltype(childCount){0}; childIndex < childCount; ++childIndex)
	{
		const auto aabb = ComputeAABB(*shape, xf, childIndex);
		const auto proxyPtr = proxies + childIndex;
		const auto proxyId = broadPhase.CreateProxy(aabb + extension, proxyPtr);
		new (proxyPtr) FixtureProxy{aabb, proxyId, this, childIndex};
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

child_count_t Fixture::Synchronize(BroadPhase& broadPhase,
								   const Transformation& transform1, const Transformation& transform2)
{
	assert(IsValid(transform1));
	assert(IsValid(transform2));

	const auto shape = GetShape();
	const auto aabbExtension = GetBody()->GetWorld()->GetAabbExtension();
	const auto extension = Vec2{aabbExtension, aabbExtension};

	auto movedCount = child_count_t{0};
	for (auto i = decltype(m_proxyCount){0}; i < m_proxyCount; ++i)
	{
		auto& proxy = m_proxies[i];

		// Compute an AABB that covers the swept shape (may miss some rotation effect).
		const auto aabb1 = ComputeAABB(*shape, transform1, proxy.childIndex);
		const auto aabb2 = ComputeAABB(*shape, transform2, proxy.childIndex);
		proxy.aabb = GetEnclosingAABB(aabb1, aabb2);

		if (broadPhase.MoveProxy(proxy.proxyId, proxy.aabb + extension, transform2.p - transform1.p))
		{
			++movedCount;
		}
	}
	return movedCount;
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
		for (auto&& contact: body->GetContacts())
		{
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
			TouchProxies(world->m_broadPhase);
		}
	}
}

void Fixture::SetSensor(bool sensor)
{
	if (sensor != m_isSensor)
	{
		m_isSensor = sensor;
		const auto body = m_body;
		if (body)
		{
			body->SetAwake();
		}
	}
}

bool box2d::TestPoint(const Fixture& f, const Vec2 p)
{
	return TestPoint(*f.GetShape(), f.GetBody()->GetTransformation(), p);
}

void box2d::SetAwake(Fixture& f) noexcept
{
	const auto b = f.GetBody();
	if (b)
	{
		b->SetAwake();
	}
}
