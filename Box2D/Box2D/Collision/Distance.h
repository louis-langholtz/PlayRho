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

#ifndef B2_DISTANCE_H
#define B2_DISTANCE_H

#include <Box2D/Common/Math.h>
#include <Box2D/Collision/DistanceProxy.hpp>

namespace box2d
{
	
struct IndexPair
{
	/// Size type.
	/// @detail Data type large enough to hold maximum value usable by this library.
	using size_type = DistanceProxy::size_type;

	static constexpr size_type InvalidIndex = static_cast<size_type>(-1);

	size_type a;
	size_type b;
};

constexpr inline bool operator == (IndexPair lhs, IndexPair rhs)
{
	return (lhs.a == rhs.a) && (lhs.b == rhs.b);
}

/// Used to warm start Distance.
class SimplexCache
{
public:
	/// Maximum count of times this object's add-index method may be called.
	/// @sa AddIndex.
	static constexpr auto MaxCount = unsigned{3};

	using size_type = std::remove_const<decltype(MaxCount)>::type;

	/// Gets the metric that was set.
	/// @note Behavior is undefined if metric was not previously set.
	///   The IsMetricSet() method can be used to check dynamically if unsure.
	/// @sa SetMetric.
	/// @sa IsMetricSet.
	/// @return Value previously set.
	auto GetMetric() const noexcept
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
	auto GetCount() const noexcept { return count; }

	auto GetIndexPair(size_type index) const noexcept
	{
		assert(index < count);
		return indexPair[index];
	}

	auto GetIndexA(size_type index) const
	{
		assert(index < count);
		return indexPair[index].a;
	}

	auto GetIndexB(size_type index) const
	{
		assert(index < count);
		return indexPair[index].b;
	}

	void ClearIndices() noexcept { count = 0; }

	auto IsMetricSet() const noexcept { return metric_set; }

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
	float_t metric; ///< length or area
	size_type count = 0;
	IndexPair indexPair[MaxCount]; ///< Vertices on shape A and B.
};

struct WitnessPoints
{
	Vec2 a;
	Vec2 b;
};
	
/// Output for Distance.
struct DistanceOutput
{
	DistanceOutput() = default;
	DistanceOutput(const DistanceOutput& copy) = default;

	constexpr DistanceOutput(const WitnessPoints& wp, unsigned it) noexcept: witnessPoints{wp}, iterations{it} {}

	WitnessPoints witnessPoints; ///< closest point on shapeA and closest point on shapeB
	unsigned iterations;	///< number of GJK iterations used
};

/// Determines the closest points between two shapes.
/// @detail
/// Supports any combination of:
/// CircleShape, PolygonShape, EdgeShape. The simplex cache is input/output.
/// @note On the first call, the SimplexCache.count should be set to zero.
/// @param cache Simplex cache for assisting the determination.
/// @param proxyA Proxy A.
/// @param transformA Transoform of A.
/// @param proxyB Proxy B.
/// @param transformB Transoform of B.
/// @return Closest points between the two shapes and the count of iterations it took to determine them.
DistanceOutput Distance(SimplexCache& cache,
						const DistanceProxy& proxyA, const Transform& transformA,
						const DistanceProxy& proxyB, const Transform& transformB);

} /* namespace box2d */

#endif
