
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

#ifndef B2_DISTANCE_H
#define B2_DISTANCE_H

#include <Box2D/Common/b2Math.h>

class b2Shape;

/// A distance proxy is used by the GJK algorithm.
/// It encapsulates any shape.
class b2DistanceProxy
{
public:
	using size_type = std::size_t;

	b2DistanceProxy() = default;

	b2DistanceProxy(const b2Shape& shape, size_type index);

	float32 GetRadius() const noexcept { return m_radius; }

	/// Initialize the proxy using the given shape. The shape
	/// must remain in scope while the proxy is in use.
	void Set(const b2Shape& shape, size_type index);

	/// Get the supporting vertex index in the given direction.
	size_type GetSupport(const b2Vec2& d) const noexcept;

	/// Get the supporting vertex in the given direction.
	const b2Vec2& GetSupportVertex(const b2Vec2& d) const;

	/// Get the vertex count.
	inline size_type GetVertexCount() const noexcept { return m_count; }

	/// Get a vertex by index. Used by b2Distance.
	const b2Vec2& GetVertex(size_type index) const;

private:
	b2Vec2 m_buffer[2];
	const b2Vec2* m_vertices = nullptr;
	size_type m_count = 0;
	float32 m_radius = 0.0f;
};

/// Used to warm start b2Distance.
class b2SimplexCache
{
public:
	using size_type = std::size_t;
	using index_t = std::size_t;

	static constexpr auto MaxCount = size_type{3};

	float32 GetMetric() const noexcept { return metric; }
	size_type GetCount() const noexcept { return count; }

	index_t GetIndexA(size_type index) const
	{
		b2Assert(index < count);
		return indexA[index];
	}

	index_t GetIndexB(size_type index) const
	{
		b2Assert(index < count);
		return indexB[index];
	}

	void ClearIndices() noexcept { count = 0; }

	void SetMetric(float32 m) noexcept { metric = m; }

	void AddIndex(index_t a, index_t b)
	{
		b2Assert(count < MaxCount);
		indexA[count] = a;
		indexB[count] = b;
		++count;
	}

private:
	float32 metric;		///< length or area
	size_type count = 0;
	index_t indexA[MaxCount];	///< vertices on shape A
	index_t indexB[MaxCount];	///< vertices on shape B
};

/// Input for b2Distance.
/// You have to option to use the shape radii
/// in the computation. Even 
struct b2DistanceInput
{
	b2DistanceProxy proxyA;
	b2DistanceProxy proxyB;
	b2Transform transformA;
	b2Transform transformB;
	bool useRadii;
};

/// Output for b2Distance.
struct b2DistanceOutput
{
	b2Vec2 pointA;		///< closest point on shapeA
	b2Vec2 pointB;		///< closest point on shapeB
	float32 distance;
	int32 iterations;	///< number of GJK iterations used
};

/// Compute the closest points between two shapes. Supports any combination of:
/// b2CircleShape, b2PolygonShape, b2EdgeShape. The simplex cache is input/output.
/// On the first call, b2SimplexCache.count should be set to zero.
void b2Distance(b2DistanceOutput* output,
				b2SimplexCache* cache, 
				const b2DistanceInput& input);


//////////////////////////////////////////////////////////////////////////

inline const b2Vec2& b2DistanceProxy::GetVertex(size_type index) const
{
	b2Assert(0 <= index && index < m_count);
	return m_vertices[index];
}

inline b2DistanceProxy::size_type b2DistanceProxy::GetSupport(const b2Vec2& d) const noexcept
{
	auto bestIndex = decltype(m_count){0};
	auto bestValue = b2Dot(m_vertices[0], d);
	for (auto i = decltype(m_count){1}; i < m_count; ++i)
	{
		const auto value = b2Dot(m_vertices[i], d);
		if (value > bestValue)
		{
			bestIndex = i;
			bestValue = value;
		}
	}

	return bestIndex;
}

inline const b2Vec2& b2DistanceProxy::GetSupportVertex(const b2Vec2& d) const
{
	auto bestIndex = decltype(m_count){0};
	auto bestValue = b2Dot(m_vertices[0], d);
	for (auto i = decltype(m_count){1}; i < m_count; ++i)
	{
		const auto value = b2Dot(m_vertices[i], d);
		if (value > bestValue)
		{
			bestIndex = i;
			bestValue = value;
		}
	}

	return m_vertices[bestIndex];
}

#endif
