/*
 * Original work Copyright (c) 2007-2009 Erin Catto http://www.box2d.org
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

#include <Box2D/Common/ArrayList.hpp>
#include <Box2D/Collision/Distance.h>
#include <Box2D/Collision/DistanceProxy.hpp>
#include <Box2D/Collision/SimplexCache.hpp>
#include <Box2D/Collision/Simplex.hpp>
#include <Box2D/Collision/IndexPairList.hpp>
#include <Box2D/Collision/Shapes/CircleShape.h>
#include <Box2D/Collision/Shapes/EdgeShape.h>
#include <Box2D/Collision/Shapes/ChainShape.h>
#include <Box2D/Collision/Shapes/PolygonShape.h>

namespace box2d {

namespace {

inline bool Find(Span<const IndexPair> pairs, IndexPair key)
{
	for (auto&& elem: pairs)
	{
		if (elem == key)
		{
			return true;
		}
	}
	return false;
}

inline DistanceProxy GetDistanceProxy(const CircleShape& shape, child_count_t index)
{
	return DistanceProxy{GetRadius(shape), shape.GetPosition()};
}

inline DistanceProxy GetDistanceProxy(const PolygonShape& shape, child_count_t index)
{
	return DistanceProxy{GetRadius(shape), shape.GetVertices()};		
}

inline DistanceProxy GetDistanceProxy(const ChainShape& shape, child_count_t index)
{
	return DistanceProxy{GetRadius(shape), shape.GetVertex(index), shape.GetVertex(shape.GetNextIndex(index))};
}

inline DistanceProxy GetDistanceProxy(const EdgeShape& shape, child_count_t index)
{
	return DistanceProxy{GetRadius(shape), shape.GetVertex1(), shape.GetVertex2()};
}
	
}

DistanceProxy GetDistanceProxy(const Shape& shape, child_count_t index)
{
	switch (shape.GetType())
	{
		case Shape::e_circle: return GetDistanceProxy(*static_cast<const CircleShape*>(&shape), index);
		case Shape::e_polygon: return GetDistanceProxy(*static_cast<const PolygonShape*>(&shape), index);
		case Shape::e_chain: return GetDistanceProxy(*static_cast<const ChainShape*>(&shape), index);
		case Shape::e_edge: return GetDistanceProxy(*static_cast<const EdgeShape*>(&shape), index);
		case Shape::e_typeCount: break;
	}
	assert(false);
	return DistanceProxy{0, Span<const Vec2>({})};
}

static inline WitnessPoints GetWitnessPoints(const Simplex& simplex) noexcept
{
	auto pointA = Vec2{0, 0};
	auto pointB = Vec2{0, 0};

	const auto size = simplex.GetSize();
	for (auto i = decltype(size){0}; i < size; ++i)
	{
		const auto v = simplex.GetSimplexVertex(i);
		const auto a = simplex.GetCoefficient(i);
		
		pointA += v.GetPointA() * a;
		pointB += v.GetPointB() * a;
	}
#if 0
	// In the 3-simplex case, pointA and pointB are usually equal.
	// XXX: Sometimes in the 3-simplex case, pointA is slightly different than pointB. Why??
	if (size == 3 && pointA != pointB)
	{
		std::cout << "odd: " << pointA << " != " << pointB;
		std::cout << std::endl;
	}
#endif
	return WitnessPoints{pointA, pointB};
}

static inline
SimplexVertex GetSimplexVertex(const DistanceProxy& proxyA, const Transformation& xfA, DistanceProxy::size_type idxA,
							   const DistanceProxy& proxyB, const Transformation& xfB, DistanceProxy::size_type idxB)
{
	const auto wA = Transform(proxyA.GetVertex(idxA), xfA);
	const auto wB = Transform(proxyB.GetVertex(idxB), xfB);
	return SimplexVertex{wA, idxA, wB, idxB};	
}

static inline
SimplexVertices GetSimplexVertices(const IndexPairList& indexPairs,
				   const DistanceProxy& proxyA, const Transformation& xfA,
				   const DistanceProxy& proxyB, const Transformation& xfB)
{
	SimplexVertices simplex;
	for (auto&& indexpair: indexPairs)
	{
		simplex.push_back(GetSimplexVertex(proxyA, xfA, indexpair.a, proxyB, xfB, indexpair.b));
	}
	return simplex;
}
	
inline auto GetSimplexCache(const SimplexVertices& simplex)
{
	return SimplexCache(CalcMetric(simplex), GetIndexPairList(simplex));
}
	
DistanceOutput Distance(const DistanceProxy& proxyA, const Transformation& transformA,
						const DistanceProxy& proxyB, const Transformation& transformB,
						const SimplexCache& cache)
{
	assert(proxyA.GetVertexCount() > 0);
	assert(IsValid(transformA.p));
	assert(proxyB.GetVertexCount() > 0);
	assert(IsValid(transformB.p));
	
	// Initialize the simplex.
	auto simplexVertices = GetSimplexVertices(cache.GetIndices(), proxyA, transformA, proxyB, transformB);

	// Compute the new simplex metric, if it is substantially different than
	// old metric then flush the simplex.
	if (simplexVertices.size() > 1)
	{
		const auto metric1 = cache.GetMetric();
		const auto metric2 = CalcMetric(simplexVertices);
		if ((metric2 < (metric1 / 2)) || (metric2 > (metric1 * 2)) || (metric2 < 0) || almost_zero(metric2))
		{
			simplexVertices.clear();
		}
	}
	
	if (simplexVertices.size() == 0)
	{
		simplexVertices.push_back(GetSimplexVertex(proxyA, transformA, 0, proxyB, transformB, 0));
	}

	auto simplex = Simplex{};

#if defined(DO_COMPUTE_CLOSEST_POINT)
	auto distanceSqr1 = MaxFloat;
#endif

	// Main iteration loop.
	auto iter = std::remove_const<decltype(MaxDistanceIterations)>::type{0};
	while (iter < MaxDistanceIterations)
	{
		++iter;
	
		// Copy simplex so we can identify duplicates and prevent cycling.
		const auto savedIndices = GetIndexPairList(simplexVertices);

		simplex = Simplex::Get(simplexVertices);
		simplexVertices = simplex.GetSimplexVertices();

		// If we have max points (3), then the origin is in the corresponding triangle.
		if (simplexVertices.size() == MaxSimplexVertices)
		{
			break;
		}

#if defined(DO_COMPUTE_CLOSEST_POINT)
		// Compute closest point.
		const auto p = GetClosestPoint(simplexVertices);
		const auto distanceSqr2 = LengthSquared(p);

		// Ensure progress
		if (distanceSqr2 >= distanceSqr1)
		{
			//break;
		}
		distanceSqr1 = distanceSqr2;
#endif
		// Get search direction.
		const auto d = CalcSearchDirection(simplexVertices);
		assert(IsValid(d));

		// Ensure the search direction is numerically fit.
		if (almost_zero(LengthSquared(d)))
		{
			// The origin is probably contained by a line segment
			// or triangle. Thus the shapes are overlapped.

			// We can't return zero here even though there may be overlap.
			// In case the simplex is a point, segment, or triangle it is difficult
			// to determine if the origin is contained in the CSO or very close to it.
			break;
		}

		// Compute a tentative new simplex vertex using support points.
		const auto indexA = GetSupportIndex(proxyA, InverseRotate(-d, transformA.q));
		const auto indexB = GetSupportIndex(proxyB, InverseRotate(d, transformB.q));

		// Check for duplicate support points. This is the main termination criteria.
		// If there's a duplicate support point, code must exit loop to avoid cycling.
		if (Find(savedIndices, IndexPair{indexA, indexB}))
		{
			break;
		}

		// New vertex is ok and needed.
		simplexVertices.push_back(GetSimplexVertex(proxyA, transformA, indexA, proxyB, transformB, indexB));
	}

	return DistanceOutput{GetWitnessPoints(simplex), iter, GetSimplexCache(simplexVertices)};
}
	
} // namespace box2d
