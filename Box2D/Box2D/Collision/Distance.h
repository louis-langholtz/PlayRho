
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

#include <Box2D/Common/Math.h>

namespace box2d
{

class Shape;
	
struct IndexPair
{
	static constexpr size_t InvalidIndex = static_cast<size_t>(-1);

	size_t a;
	size_t b;
};

constexpr inline bool operator == (IndexPair lhs, IndexPair rhs)
{
	return (lhs.a == rhs.a) && (lhs.b == rhs.b);
}

/// A distance proxy is used by the GJK algorithm.
/// It encapsulates any shape.
class DistanceProxy
{
public:
	using size_type = size_t; // must be big enough to hold max posible count of vertices

	static constexpr size_type InvalidIndex = static_cast<size_type>(-1);

	DistanceProxy() noexcept = default;
	
	constexpr DistanceProxy(const DistanceProxy& copy) noexcept = default;

	constexpr DistanceProxy(float_t radius, Vec2 v0) noexcept:
		m_radius{radius}, m_buffer{v0}, m_count{1}
	{}

	constexpr DistanceProxy(float_t radius, Vec2 v0, Vec2 v1) noexcept:
		m_radius{radius}, m_buffer{v0, v1}, m_count{2}
	{}

	constexpr DistanceProxy(float_t radius, const Vec2* vertices, size_type count) noexcept:
		m_radius{radius}, m_buffer{}, m_vertices{vertices}, m_count{count}
	{}

	/// Gets the "radius" of the associated shape.
	/// @return Non-negative distance.
	float_t GetRadius() const noexcept { return m_radius; }

	/// Get the supporting vertex index in the given direction.
	/// @param d Direction vector to find index for.
	/// @return InvalidIndex if the count of vertices is zero or a value from 0 to one less than count.
	/// @sa GetVertexCount().
	size_type GetSupportIndex(const Vec2& d) const noexcept;

	/// Get the vertex count.
	/// @detail This is the count of valid vertex elements that this object provides.
	inline size_type GetVertexCount() const noexcept { return m_count; }

	/// Get a vertex by index. Used by Distance.
	/// @param index A valid index value (must not be InvalidIndex).
	/// @note Behavior is undefined if InvalidIndex is given as the index value.
	/// @return 2D vector value at the given index.
	Vec2 GetVertex(size_type index) const;

private:
	Vec2 m_buffer[2];
	const Vec2* m_vertices = m_buffer;
	size_type m_count = 0; ///< Count of valid elements at m_vertices.
	float_t m_radius = float_t{0}; ///< "Radius" of the associated shape.
};

/// Initialize the proxy using the given shape.
/// @note The shape must remain in scope while the proxy is in use.
DistanceProxy GetDistanceProxy(const Shape& shape, child_count_t index);

/// Used to warm start Distance.
class SimplexCache
{
public:
	/// Maximum count of times this object's add-index method may be called.
	/// @sa AddIndex.
	static constexpr auto MaxCount = unsigned{3};

	using size_type = std::remove_cv<decltype(MaxCount)>::type;

	using index_t = size_t;

	/// Gets the metric that was set.
	/// @note Behavior is undefined if metric was not previously set.
	///   The IsMetricSet() method can be used to check dynamically if unsure.
	/// @sa SetMetric.
	/// @sa IsMetricSet.
	/// @return Value previously set.
	float_t GetMetric() const noexcept
	{
		assert(metric_set);
		return metric;
	}

	/// Gets the count.
 	/// @detail This is the count of times this object's add-index method has been called
	///   since the last clearing of the indexes was done.
	/// @return Value between 0 and MaxCount.
	/// @sa MaxCount.
	/// @sa AddIndex.
	/// @sa ClearIndices.
	size_type GetCount() const noexcept { return count; }

	IndexPair GetIndexPair(size_type index) const noexcept
	{
		assert(index < count);
		return indexPair[index];
	}

	index_t GetIndexA(size_type index) const
	{
		assert(index < count);
		return indexPair[index].a;
	}

	index_t GetIndexB(size_type index) const
	{
		assert(index < count);
		return indexPair[index].b;
	}

	void ClearIndices() noexcept { count = 0; }

	bool IsMetricSet() const noexcept { return metric_set; }

	void SetMetric(float_t m) noexcept
	{
		metric = m;
		metric_set = true;
	}

	void AddIndex(IndexPair ip)
	{
		assert(count < MaxCount);
		indexPair[count] = ip;
		++count;
	}

private:
	bool metric_set = false; ///< Whether the metric has been set or not.
	float_t metric;		///< length or area
	size_type count = 0;
	IndexPair indexPair[MaxCount];	///< Vertices on shape A and B.
};

/// Input for Distance.
/// You have to option to use the shape radii
/// in the computation. Even 
struct DistanceInput
{
	DistanceProxy proxyA;
	DistanceProxy proxyB;
	Transform transformA;
	Transform transformB;
	bool useRadii;
};

/// Output for Distance.
struct DistanceOutput
{
	Vec2 pointA;		///< closest point on shapeA
	Vec2 pointB;		///< closest point on shapeB
	float_t distance;
	unsigned iterations;	///< number of GJK iterations used
};

/// Compute the closest points between two shapes. Supports any combination of:
/// CircleShape, PolygonShape, EdgeShape. The simplex cache is input/output.
/// On the first call, SimplexCache.count should be set to zero.
DistanceOutput Distance(SimplexCache& cache,  const DistanceInput& input);

//////////////////////////////////////////////////////////////////////////

inline Vec2 DistanceProxy::GetVertex(size_type index) const
{
	assert(index != InvalidIndex);
	assert(index < m_count);
	return m_vertices[index];
}

inline DistanceProxy::size_type DistanceProxy::GetSupportIndex(const Vec2& d) const noexcept
{
	auto bestIndex = InvalidIndex;
	auto bestValue = -MaxFloat;
	for (auto i = decltype(m_count){0}; i < m_count; ++i)
	{
		const auto value = Dot(m_vertices[i], d);
		if (bestValue < value)
		{
			bestValue = value;
			bestIndex = i;
		}
	}
	return bestIndex;
}

} /* namespace box2d */

#endif
