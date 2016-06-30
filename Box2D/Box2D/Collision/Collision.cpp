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

#include <Box2D/Collision/Collision.h>
#include <Box2D/Collision/Distance.h>

namespace box2d {

static inline WorldManifold GetWorldManifoldForCircles(const Manifold& manifold,
													   const Transform& xfA, const float_t radiusA,
													   const Transform& xfB, const float_t radiusB)
{
	assert(manifold.GetPointCount() == 1);
	
	switch (manifold.GetPointCount())
	{
		case 1:
		{
			const auto pointA = Mul(xfA, manifold.GetLocalPoint());
			const auto pointB = Mul(xfB, manifold.GetPoint(0).localPoint);
			const auto delta = pointB - pointA;
			const auto normal = (LengthSquared(delta) > Square(BOX2D_MAGIC(Epsilon)))?
				GetUnitVector(delta): Vec2{float_t{1}, float_t{0}};
			const auto cA = pointA + (radiusA * normal);
			const auto cB = pointB - (radiusB * normal);
			const auto p0 = (cA + cB) / float_t{2};
			const auto s0 = Dot(cB - cA, normal);
			return WorldManifold{normal, PointSeparation{p0, s0}};
		}
		default: break;
	}

	// should never be reached
	return WorldManifold{Vec2{float_t{1}, float_t{0}}};
}

static inline WorldManifold GetWorldManifoldForFaceA(const Manifold& manifold,
													 const Transform& xfA, const float_t radiusA,
													 const Transform& xfB, const float_t radiusB)
{
	const auto normal = Mul(xfA.q, manifold.GetLocalNormal());
	const auto planePoint = Mul(xfA, manifold.GetLocalPoint());
	const auto pointFn = [&](Manifold::size_type index) {
		const auto clipPoint = Mul(xfB, manifold.GetPoint(index).localPoint);
		const auto cA = clipPoint + (radiusA - Dot(clipPoint - planePoint, normal)) * normal;
		const auto cB = clipPoint - (radiusB * normal);
		return PointSeparation{(cA + cB) / float_t{2}, Dot(cB - cA, normal)};
	};
	
	assert(manifold.GetPointCount() <= 2);
	
	switch (manifold.GetPointCount())
	{
		case 0: return WorldManifold{normal};
		case 1: return WorldManifold{normal, pointFn(0)};
		case 2: return WorldManifold{normal, pointFn(0), pointFn(1)};
		default: break; // should never be reached
	}

	// should never be reached
	return WorldManifold{normal};
}

static inline WorldManifold GetWorldManifoldForFaceB(const Manifold& manifold,
													 const Transform& xfA, const float_t radiusA,
													 const Transform& xfB, const float_t radiusB)
{
	const auto normal = Mul(xfB.q, manifold.GetLocalNormal());
	const auto planePoint = Mul(xfB, manifold.GetLocalPoint());
	const auto pointFn = [&](Manifold::size_type index) {
		const auto clipPoint = Mul(xfA, manifold.GetPoint(index).localPoint);
		const auto cB = clipPoint + (radiusB - Dot(clipPoint - planePoint, normal)) * normal;
		const auto cA = clipPoint - (radiusA * normal);
		return PointSeparation{(cA + cB) / float_t{2}, Dot(cA - cB, normal)};
	};
	
	assert(manifold.GetPointCount() <= 2);
	
	// Negate normal given to world manifold constructor to ensure it points from A to B.
	switch (manifold.GetPointCount())
	{
		case 0: return WorldManifold{-normal};
		case 1: return WorldManifold{-normal, pointFn(0)};
		case 2: return WorldManifold{-normal, pointFn(0), pointFn(1)};
		default: break; // should never be reached
	}
	
	// should never be reached
	return WorldManifold{-normal};
}

WorldManifold GetWorldManifold(const Manifold& manifold,
							   const Transform& xfA, const float_t radiusA,
							   const Transform& xfB, const float_t radiusB)
{
	const auto type = manifold.GetType();
	assert((type == Manifold::e_circles) || (type == Manifold::e_faceA) || (type == Manifold::e_faceB) || (type == Manifold::e_unset));
	switch (type)
	{
		case Manifold::e_circles: return GetWorldManifoldForCircles(manifold, xfA, radiusA, xfB, radiusB);
		case Manifold::e_faceA: return GetWorldManifoldForFaceA(manifold, xfA, radiusA, xfB, radiusB);
		case Manifold::e_faceB: return GetWorldManifoldForFaceB(manifold, xfA, radiusA, xfB, radiusB);
		case Manifold::e_unset: return WorldManifold{Vec2_zero};
	}

	// should never be reached
	return WorldManifold{Vec2_zero};
}

void GetPointStates(PointStateArray& state1, PointStateArray& state2,
					const Manifold& manifold1, const Manifold& manifold2)
{
	for (auto i = decltype(MaxManifoldPoints){0}; i < MaxManifoldPoints; ++i)
	{
		state1[i] = PointState::NullState;
		state2[i] = PointState::NullState;
	}

	// Detect persists and removes.
	for (auto i = decltype(manifold1.GetPointCount()){0}; i < manifold1.GetPointCount(); ++i)
	{
		const auto cf = manifold1.GetPoint(i).contactFeature;

		state1[i] = PointState::RemoveState;

		for (auto j = decltype(manifold2.GetPointCount()){0}; j < manifold2.GetPointCount(); ++j)
		{
			if (manifold2.GetPoint(j).contactFeature == cf)
			{
				state1[i] = PointState::PersistState;
				break;
			}
		}
	}

	// Detect persists and adds.
	for (auto i = decltype(manifold2.GetPointCount()){0}; i < manifold2.GetPointCount(); ++i)
	{
		const auto cf = manifold2.GetPoint(i).contactFeature;

		state2[i] = PointState::AddState;

		for (auto j = decltype(manifold1.GetPointCount()){0}; j < manifold1.GetPointCount(); ++j)
		{
			if (manifold1.GetPoint(j).contactFeature == cf)
			{
				state2[i] = PointState::PersistState;
				break;
			}
		}
	}
}

// From Real-time Collision Detection, p179.
bool AABB::RayCast(RayCastOutput* output, const RayCastInput& input) const
{
	auto tmin = -MaxFloat;
	auto tmax = MaxFloat;

	const auto p = input.p1;
	const auto d = input.p2 - input.p1;
	const auto absD = Abs(d);

	Vec2 normal;

	for (auto i = decltype(normal.max_size()){0}; i < normal.max_size(); ++i)
	{
		if (absD[i] < BOX2D_MAGIC(Epsilon))
		{
			// Parallel.
			if ((p[i] < lowerBound[i]) || (upperBound[i] < p[i]))
			{
				return false;
			}
		}
		else
		{
			const auto inv_d = float_t{1} / d[i];
			auto t1 = (lowerBound[i] - p[i]) * inv_d;
			auto t2 = (upperBound[i] - p[i]) * inv_d;
			
			// Sign of the normal vector.
			auto s = float_t{-1};
			
			if (t1 > t2)
			{
				Swap(t1, t2);
				s = float_t{1};
			}
			
			// Push the min up
			if (tmin < t1)
			{
				normal = Vec2_zero;
				normal[i] = s;
				tmin = t1;
			}
			
			// Pull the max down
			tmax = Min(tmax, t2);
			
			if (tmin > tmax)
			{
				return false;
			}
		}
	};

	// Does the ray start inside the box?
	// Does the ray intersect beyond the max fraction?
	if ((tmin < float_t{0}) || (tmin > input.maxFraction))
	{
		return false;
	}

	// Intersection.
	output->fraction = tmin;
	output->normal = normal;
	return true;
}

ClipArray::size_type ClipSegmentToLine(ClipArray& vOut, const ClipArray& vIn,
									   const Vec2& normal, float_t offset, ContactFeature::index_t indexA)
{
	// Use Sutherland-Hodgman clipping (https://en.wikipedia.org/wiki/Sutherland%E2%80%93Hodgman_algorithm ).
	
	// Start with no output points
	auto numOut = ClipArray::size_type{0};

	// Calculate the distance of end points to the line
	const auto distance0 = Dot(normal, vIn[0].v) - offset; ///< Distance of point at vIn[0].v from line defined by normal and offset.
	const auto distance1 = Dot(normal, vIn[1].v) - offset; ///< Distance of point at vIn[1].v from line defined by normal and offset.

	// If the points are behind the plane
	if (distance0 <= float_t{0}) vOut[numOut++] = vIn[0];
	if (distance1 <= float_t{0}) vOut[numOut++] = vIn[1];

	// If the points are on different sides of the plane
	if ((distance0 * distance1) < float_t{0})
	{
		// Find intersection point of edge and plane
		// Vertex A is hitting edge B.
		const auto interp = distance0 / (distance0 - distance1);
		vOut[numOut] = ClipVertex{
			vIn[0].v + (vIn[1].v - vIn[0].v) * interp,
			ContactFeature{ContactFeature::e_vertex, indexA, ContactFeature::e_face, vIn[0].cf.indexB}
		};
		++numOut;
	}

	return numOut;
}

bool TestOverlap(const Shape& shapeA, child_count_t indexA,
				   const Shape& shapeB, child_count_t indexB,
				   const Transform& xfA, const Transform& xfB)
{
	const auto proxyA = GetDistanceProxy(shapeA, indexA);
	const auto proxyB = GetDistanceProxy(shapeB, indexB);

	SimplexCache cache;
	const auto output = Distance(cache, proxyA, xfA, proxyB, xfB);
	
	const auto distanceSquared = DistanceSquared(output.witnessPoints.a, output.witnessPoints.b);
	const auto totalRadiusSquared = Square(proxyA.GetRadius() + proxyB.GetRadius());
	
	return (distanceSquared - totalRadiusSquared) < Square(BOX2D_MAGIC(Epsilon * 10));
}

} // namespace box2d
