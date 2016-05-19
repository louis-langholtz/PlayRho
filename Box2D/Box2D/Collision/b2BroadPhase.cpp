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

#include <Box2D/Collision/b2BroadPhase.h>

using namespace box2d;

b2BroadPhase::b2BroadPhase():
	m_pairBuffer(static_cast<b2ProxyIdPair*>(alloc(m_pairCapacity * sizeof(b2ProxyIdPair)))),
	m_moveBuffer(static_cast<size_type*>(alloc(m_moveCapacity * sizeof(size_type))))
{}

b2BroadPhase::~b2BroadPhase()
{
	free(m_moveBuffer);
	free(m_pairBuffer);
}

b2BroadPhase::size_type b2BroadPhase::CreateProxy(const b2AABB& aabb, void* userData)
{
	const auto proxyId = m_tree.CreateProxy(aabb, userData);
	++m_proxyCount;
	BufferMove(proxyId);
	return proxyId;
}

void b2BroadPhase::DestroyProxy(size_type proxyId)
{
	assert(m_proxyCount > 0);
	UnBufferMove(proxyId);
	--m_proxyCount;
	m_tree.DestroyProxy(proxyId);
}

void b2BroadPhase::MoveProxy(size_type proxyId, const b2AABB& aabb, const Vec2& displacement)
{
	const auto buffer = m_tree.MoveProxy(proxyId, aabb, displacement);
	if (buffer)
	{
		BufferMove(proxyId);
	}
}

void b2BroadPhase::TouchProxy(size_type proxyId)
{
	BufferMove(proxyId);
}

void b2BroadPhase::BufferMove(size_type proxyId)
{
	if (m_moveCount == m_moveCapacity)
	{
		m_moveCapacity *= BufferGrowthRate;
		m_moveBuffer = static_cast<size_type*>(realloc(m_moveBuffer, m_moveCapacity * sizeof(size_type)));
	}

	m_moveBuffer[m_moveCount] = proxyId;
	++m_moveCount;
}

void b2BroadPhase::UnBufferMove(size_type proxyId)
{
	for (auto i = decltype(m_moveCount){0}; i < m_moveCount; ++i)
	{
		if (m_moveBuffer[i] == proxyId)
		{
			m_moveBuffer[i] = e_nullProxy;
		}
	}
}

// This is called from b2DynamicTree::Query when we are gathering pairs.
bool b2BroadPhase::QueryCallback(size_type proxyId)
{
	// A proxy cannot form a pair with itself.
	if (proxyId == m_queryProxyId)
	{
		return true;
	}

	// Grow the pair buffer as needed.
	if (m_pairCapacity == m_pairCount)
	{
		m_pairCapacity *= BufferGrowthRate;
		m_pairBuffer = static_cast<b2ProxyIdPair*>(realloc(m_pairBuffer, m_pairCapacity * sizeof(b2ProxyIdPair)));
	}

	m_pairBuffer[m_pairCount].proxyIdA = Min(proxyId, m_queryProxyId);
	m_pairBuffer[m_pairCount].proxyIdB = Max(proxyId, m_queryProxyId);
	++m_pairCount;

	return true;
}
