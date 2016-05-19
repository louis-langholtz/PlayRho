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

#ifndef B2_BROAD_PHASE_H
#define B2_BROAD_PHASE_H

#include <Box2D/Common/b2Settings.h>
#include <Box2D/Collision/b2Collision.h>
#include <Box2D/Collision/b2DynamicTree.h>
#include <algorithm>

namespace box2d {

/// Proxy ID pair.
struct ProxyIdPair
{
	using size_type = size_t;
	size_type proxyIdA;
	size_type proxyIdB;
};

constexpr inline bool operator == (ProxyIdPair lhs, ProxyIdPair rhs)
{
	return (lhs.proxyIdA == rhs.proxyIdA) && (lhs.proxyIdB == rhs.proxyIdB);
}

constexpr inline bool operator != (ProxyIdPair lhs, ProxyIdPair rhs)
{
	return !(lhs == rhs);
}

/// The broad-phase is used for computing pairs and performing volume queries and ray casts.
/// This broad-phase does not persist pairs. Instead, this reports potentially new pairs.
/// It is up to the client to consume the new pairs and to track subsequent overlap.
class BroadPhase
{
public:

	using size_type = size_t;

	enum: size_type
	{
		e_nullProxy = static_cast<size_type>(-1)
	};

	BroadPhase();
	~BroadPhase();
	
	BroadPhase(const BroadPhase& copy) = delete;
	BroadPhase& operator=(const BroadPhase&) = delete;

	/// Creates a proxy with an initial AABB. Pairs are not reported until
	/// UpdatePairs is called.
	size_type CreateProxy(const AABB& aabb, void* userData);

	/// Destroys a proxy. It is up to the client to remove any pairs.
	void DestroyProxy(size_type proxyId);

	/// Call MoveProxy as many times as you like, then when you are done
	/// call UpdatePairs to finalized the proxy pairs (for your time step).
	void MoveProxy(size_type proxyId, const AABB& aabb, const Vec2& displacement);

	/// Call to trigger a re-processing of it's pairs on the next call to UpdatePairs.
	void TouchProxy(size_type proxyId);

	/// Gets the fat AABB for a proxy.
	const AABB& GetFatAABB(size_type proxyId) const;

	/// Gets user data from a proxy.
	void* GetUserData(size_type proxyId) const;

	/// Tests overlap of fat AABBs.
	bool TestOverlap(size_type proxyIdA, size_type proxyIdB) const;

	/// Get the number of proxies.
	size_type GetProxyCount() const noexcept;

	/// Update the pairs. This results in pair callbacks. This can only add pairs.
	template <typename T>
	void UpdatePairs(T* callback);

	/// Query an AABB for overlapping proxies. The callback class
	/// is called for each proxy that overlaps the supplied AABB.
	template <typename T>
	void Query(T* callback, const AABB& aabb) const;

	/// Ray-cast against the proxies in the tree. This relies on the callback
	/// to perform an exact ray-cast in the case were the proxy contains a shape.
	/// The callback also performs the any collision filtering. This has performance
	/// roughly equal to k * log(n), where k is the number of collisions and n is the
	/// number of proxies in the tree.
	/// @param input the ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
	/// @param callback a callback class that is called for each proxy that is hit by the ray.
	template <typename T>
	void RayCast(T* callback, const b2RayCastInput& input) const;

	/// Gets the height of the embedded tree.
	size_type GetTreeHeight() const noexcept;

	/// Gets the balance of the embedded tree.
	size_type GetTreeBalance() const;

	/// Gets the quality metric of the embedded tree.
	float_t GetTreeQuality() const;

	/// Shifts the world origin. Useful for large worlds.
	/// The shift formula is: position -= newOrigin
	/// @param newOrigin the new origin with respect to the old origin
	void ShiftOrigin(const Vec2& newOrigin);

private:

	friend class b2DynamicTree;

	void BufferMove(size_type proxyId);
	void UnBufferMove(size_type proxyId);

	bool QueryCallback(size_type proxyId);

	b2DynamicTree m_tree;

	static constexpr size_type BufferGrowthRate = 2;
	
	size_type m_proxyCount = 0;

	size_type m_moveCapacity = 16; ///< Move buffer capacity. The # of elements pointed to by move buffer. @sa m_moveBuffer.
	size_type m_moveCount = 0;

	size_type m_pairCapacity = 16;
	size_type m_pairCount = 0;

	// Initialized on construction
	size_type* m_moveBuffer; ///< Move buffer. @sa size_type. @sa <code>m_moveCapacity</code>. @sa <code>m_moveCount</code>.
	ProxyIdPair* m_pairBuffer;

	// Assigned on calling UpdatePairs
	size_type m_queryProxyId;
};

/// This is used to sort pairs.
inline bool b2PairLessThan(const ProxyIdPair& pair1, const ProxyIdPair& pair2) noexcept
{
	if (pair1.proxyIdA < pair2.proxyIdA)
	{
		return true;
	}

	if (pair1.proxyIdA == pair2.proxyIdA)
	{
		return pair1.proxyIdB < pair2.proxyIdB;
	}

	return false;
}

inline void* BroadPhase::GetUserData(size_type proxyId) const
{
	return m_tree.GetUserData(proxyId);
}

inline bool BroadPhase::TestOverlap(size_type proxyIdA, size_type proxyIdB) const
{
	const AABB& aabbA = m_tree.GetFatAABB(proxyIdA);
	const AABB& aabbB = m_tree.GetFatAABB(proxyIdB);
	return b2TestOverlap(aabbA, aabbB);
}

inline const AABB& BroadPhase::GetFatAABB(size_type proxyId) const
{
	return m_tree.GetFatAABB(proxyId);
}

inline BroadPhase::size_type BroadPhase::GetProxyCount() const noexcept
{
	return m_proxyCount;
}

inline BroadPhase::size_type BroadPhase::GetTreeHeight() const noexcept
{
	return m_tree.GetHeight();
}

inline BroadPhase::size_type BroadPhase::GetTreeBalance() const
{
	return m_tree.GetMaxBalance();
}

inline float_t BroadPhase::GetTreeQuality() const
{
	return m_tree.GetAreaRatio();
}

template <typename T>
void BroadPhase::UpdatePairs(T* callback)
{
	// Reset pair buffer
	m_pairCount = 0;

	// Perform tree queries for all moving proxies.
	for (auto i = decltype(m_moveCount){0}; i < m_moveCount; ++i)
	{
		m_queryProxyId = m_moveBuffer[i];
		if (m_queryProxyId == e_nullProxy)
		{
			continue;
		}

		// We have to query the tree with the fat AABB so that
		// we don't fail to create a pair that may touch later.
		const AABB& fatAABB = m_tree.GetFatAABB(m_queryProxyId);

		// Query tree, create pairs and add them pair buffer.
		m_tree.Query(this, fatAABB);
	}

	// Reset move buffer
	m_moveCount = 0;

	// Sort the pair buffer to expose duplicates.
	std::sort(m_pairBuffer, m_pairBuffer + m_pairCount, b2PairLessThan);

	// Send the pairs back to the client.
	auto i = decltype(m_pairCount){0};
	while (i < m_pairCount)
	{
		const auto& primaryPair = m_pairBuffer[i];
		void* userDataA = m_tree.GetUserData(primaryPair.proxyIdA);
		void* userDataB = m_tree.GetUserData(primaryPair.proxyIdB);

		callback->AddPair(userDataA, userDataB);
		++i;

		// Skip any duplicate pairs.
		while (i < m_pairCount)
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
}

template <typename T>
inline void BroadPhase::Query(T* callback, const AABB& aabb) const
{
	m_tree.Query(callback, aabb);
}

template <typename T>
inline void BroadPhase::RayCast(T* callback, const b2RayCastInput& input) const
{
	m_tree.RayCast(callback, input);
}

inline void BroadPhase::ShiftOrigin(const Vec2& newOrigin)
{
	m_tree.ShiftOrigin(newOrigin);
}
		
} // namespace box2d

#endif
