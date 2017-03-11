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

#include <Box2D/Collision/BroadPhase.hpp>

using namespace box2d;

BroadPhase::BroadPhase(const Conf conf):
	m_pairCapacity{conf.pairCapacity},
	m_moveCapacity{conf.moveCapacity},
	m_pairBuffer(alloc<ProxyIdPair>(conf.pairCapacity)),
	m_moveBuffer(alloc<size_type>(conf.moveCapacity))
{}

BroadPhase::~BroadPhase() noexcept
{
	free(m_moveBuffer);
	free(m_pairBuffer);
}

BroadPhase::size_type BroadPhase::CreateProxy(const AABB& aabb, void* userData)
{
	const auto proxyId = m_tree.CreateProxy(aabb, userData);
	++m_proxyCount;
	BufferMove(proxyId);
	return proxyId;
}

void BroadPhase::DestroyProxy(size_type proxyId)
{
	assert(m_proxyCount > 0);
	UnBufferMove(proxyId);
	--m_proxyCount;
	m_tree.DestroyProxy(proxyId);
}

bool BroadPhase::MoveProxy(size_type proxyId, const AABB& aabb, const Vec2 displacement,
						   const RealNum multiplier, const Vec2 extension)
{
	const auto moved = m_tree.MoveProxy(proxyId, aabb, displacement, multiplier, extension);
	if (moved)
	{
		BufferMove(proxyId);
	}
	return moved;
}

void BroadPhase::TouchProxy(size_type proxyId)
{
	BufferMove(proxyId);
}

void BroadPhase::BufferMove(size_type proxyId)
{
	if (m_moveCount == m_moveCapacity)
	{
		m_moveCapacity *= BufferGrowthRate;
		m_moveBuffer = realloc<size_type>(m_moveBuffer, m_moveCapacity);
	}

	m_moveBuffer[m_moveCount] = proxyId;
	++m_moveCount;
}

void BroadPhase::UnBufferMove(size_type proxyId)
{
	for (auto i = decltype(m_moveCount){0}; i < m_moveCount; ++i)
	{
		if (m_moveBuffer[i] == proxyId)
		{
			m_moveBuffer[i] = e_nullProxy;
		}
	}
}

BroadPhase::size_type BroadPhase::UpdatePairs(std::function<bool(void*,void*)> callback)
{
	// Reset pair buffer
	auto pairCount = size_type{0};
	
	// Perform tree queries for all moving proxies.
	for (auto i = decltype(m_moveCount){0}; i < m_moveCount; ++i)
	{
		const auto queryProxyId = m_moveBuffer[i];
		if (queryProxyId == e_nullProxy)
		{
			continue;
		}
		
		// We have to query the tree with the fat AABB so that
		// we don't fail to create a pair that may touch later.
		const auto aabb = m_tree.GetFatAABB(queryProxyId);
		
		// Query tree for nodes overlapping aabb, create pairs of those & add them pair buffer.
		m_tree.Query(aabb, [&](DynamicTree::size_type nodeId) {
			// A proxy cannot form a pair with itself.
			if (nodeId != queryProxyId)
			{
				// Grow the pair buffer as needed.
				if (m_pairCapacity == pairCount)
				{
					m_pairCapacity *= BufferGrowthRate;
					m_pairBuffer = realloc<ProxyIdPair>(m_pairBuffer, m_pairCapacity);
				}
				m_pairBuffer[pairCount] = ProxyIdPair{Min(nodeId, queryProxyId), Max(nodeId, queryProxyId)};
				++pairCount;
			}
			return true;
		});
	}
	
	// Reset move buffer
	m_moveCount = 0;
	
	// Sort the pair buffer to expose duplicates.
	std::sort(m_pairBuffer, m_pairBuffer + pairCount, [](ProxyIdPair p1, ProxyIdPair p2) {
		return (p1.proxyIdA < p2.proxyIdA) || ((p1.proxyIdA == p2.proxyIdA) && (p1.proxyIdB < p2.proxyIdB));
	});
	
	auto added = size_type{0};
	// Send the pairs back to the client.
	for (auto i = decltype(pairCount){0}; i < pairCount; )
	{
		const auto& primaryPair = m_pairBuffer[i];
		const auto userDataA = m_tree.GetUserData(primaryPair.proxyIdA);
		const auto userDataB = m_tree.GetUserData(primaryPair.proxyIdB);
		
		if (callback(userDataA, userDataB))
		{
			++added;
		}
		++i;
		
		// Skip any duplicate pairs.
		while (i < pairCount)
		{
			const auto& pair = m_pairBuffer[i];
			if (pair != primaryPair)
			{
				break;
			}
			++i;
		}
	}
	
	// Try to keep the tree balanced.
	//m_tree.Rebalance(4);
	return added;
}
