/*
* Copyright (c) 2007-2009 Erin Catto http://www.box2d.org
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

#include <Box2D/Collision/b2Collision.h>
#include <Box2D/Collision/b2Distance.h>

void b2WorldManifold::Assign(const b2Manifold& manifold,
						  const b2Transform& xfA, float32 radiusA,
						  const b2Transform& xfB, float32 radiusB)
{
	if (manifold.GetPointCount() == 0)
		return;

	switch (manifold.GetType())
	{
	case b2Manifold::e_unset:
		b2Assert(manifold.GetType() != b2Manifold::e_unset);
		break;

	case b2Manifold::e_circles:
		{
			normal = b2Vec2(1.0f, 0.0f);
			const auto pointA = b2Mul(xfA, manifold.GetLocalPoint());
			const auto pointB = b2Mul(xfB, manifold.GetPoint(0).localPoint);
			if (b2DistanceSquared(pointA, pointB) > (b2_epsilon * b2_epsilon))
				normal = b2Normalize(pointB - pointA);

			const auto cA = pointA + radiusA * normal;
			const auto cB = pointB - radiusB * normal;
			points[0] = 0.5f * (cA + cB);
			separations[0] = b2Dot(cB - cA, normal);
			pointCount = 1;
		}
		break;

	case b2Manifold::e_faceA:
		{
			normal = b2Mul(xfA.q, manifold.GetLocalNormal());
			const auto planePoint = b2Mul(xfA, manifold.GetLocalPoint());
			pointCount = 0;
			for (auto i = decltype(manifold.GetPointCount()){0}; i < manifold.GetPointCount(); ++i)
			{
				const auto clipPoint = b2Mul(xfB, manifold.GetPoint(i).localPoint);
				const auto cA = clipPoint + (radiusA - b2Dot(clipPoint - planePoint, normal)) * normal;
				const auto cB = clipPoint - radiusB * normal;
				points[i] = 0.5f * (cA + cB);
				separations[i] = b2Dot(cB - cA, normal);
				++pointCount;
			}
		}
		break;

	case b2Manifold::e_faceB:
		{
			normal = b2Mul(xfB.q, manifold.GetLocalNormal());
			const auto planePoint = b2Mul(xfB, manifold.GetLocalPoint());

			pointCount = 0;
			for (auto i = decltype(manifold.GetPointCount()){0}; i < manifold.GetPointCount(); ++i)
			{
				const auto clipPoint = b2Mul(xfA, manifold.GetPoint(i).localPoint);
				const auto cB = clipPoint + (radiusB - b2Dot(clipPoint - planePoint, normal)) * normal;
				const auto cA = clipPoint - radiusA * normal;
				points[i] = 0.5f * (cA + cB);
				separations[i] = b2Dot(cA - cB, normal);
				++pointCount;
			}

			// Ensure normal points from A to B.
			normal = -normal;
		}
		break;
	}
}

void b2GetPointStates(b2PointStateArray& state1, b2PointStateArray& state2,
					  const b2Manifold& manifold1, const b2Manifold& manifold2)
{
	for (auto i = decltype(b2_maxManifoldPoints){0}; i < b2_maxManifoldPoints; ++i)
	{
		state1[i] = b2_nullState;
		state2[i] = b2_nullState;
	}

	// Detect persists and removes.
	for (auto i = decltype(manifold1.GetPointCount()){0}; i < manifold1.GetPointCount(); ++i)
	{
		const auto& cf = manifold1.GetPoint(i).cf;

		state1[i] = b2_removeState;

		for (auto j = decltype(manifold2.GetPointCount()){0}; j < manifold2.GetPointCount(); ++j)
		{
			if (manifold2.GetPoint(j).cf == cf)
			{
				state1[i] = b2_persistState;
				break;
			}
		}
	}

	// Detect persists and adds.
	for (auto i = decltype(manifold2.GetPointCount()){0}; i < manifold2.GetPointCount(); ++i)
	{
		const auto& cf = manifold2.GetPoint(i).cf;

		state2[i] = b2_addState;

		for (auto j = decltype(manifold1.GetPointCount()){0}; j < manifold1.GetPointCount(); ++j)
		{
			if (manifold1.GetPoint(j).cf == cf)
			{
				state2[i] = b2_persistState;
				break;
			}
		}
	}
}

// From Real-time Collision Detection, p179.
bool b2AABB::RayCast(b2RayCastOutput* output, const b2RayCastInput& input) const
{
	auto tmin = -b2_maxFloat;
	auto tmax = b2_maxFloat;

	const auto p = input.p1;
	const auto d = input.p2 - input.p1;
	const auto absD = b2Abs(d);

	b2Vec2 normal;

	for (auto i = 0; i < 2; ++i)
	{
		if (absD(i) < b2_epsilon)
		{
			// Parallel.
			if ((p(i) < lowerBound(i)) || (upperBound(i) < p(i)))
				return false;
		}
		else
		{
			const auto inv_d = 1.0f / d(i);
			auto t1 = (lowerBound(i) - p(i)) * inv_d;
			auto t2 = (upperBound(i) - p(i)) * inv_d;

			// Sign of the normal vector.
			auto s = -1.0f;

			if (t1 > t2)
			{
				b2Swap(t1, t2);
				s = 1.0f;
			}

			// Push the min up
			if (t1 > tmin)
			{
				normal = b2Vec2_zero;
				normal(i) = s;
				tmin = t1;
			}

			// Pull the max down
			tmax = b2Min(tmax, t2);

			if (tmin > tmax)
				return false;
		}
	}

	// Does the ray start inside the box?
	// Does the ray intersect beyond the max fraction?
	if ((tmin < 0.0f) || (input.maxFraction < tmin))
		return false;

	// Intersection.
	output->fraction = tmin;
	output->normal = normal;
	return true;
}

// Sutherland-Hodgman clipping.
b2ClipArray::size_type b2ClipSegmentToLine(b2ClipArray& vOut, const b2ClipArray& vIn,
										   const b2Vec2& normal, float32 offset, b2ContactFeature::index_t vertexIndexA)
{
	// Start with no output points
	auto numOut = b2ClipArray::size_type{0};

	// Calculate the distance of end points to the line
	const auto distance0 = b2Dot(normal, vIn[0].v) - offset;
	const auto distance1 = b2Dot(normal, vIn[1].v) - offset;

	// If the points are behind the plane
	if (distance0 <= 0.0f) vOut[numOut++] = vIn[0];
	if (distance1 <= 0.0f) vOut[numOut++] = vIn[1];

	// If the points are on different sides of the plane
	if ((distance0 * distance1) < 0.0f)
	{
		// Find intersection point of edge and plane
		const auto interp = distance0 / (distance0 - distance1);
		vOut[numOut].v = vIn[0].v + interp * (vIn[1].v - vIn[0].v);

		// VertexA is hitting edgeB.
		vOut[numOut].cf.indexA = vertexIndexA;
		vOut[numOut].cf.indexB = vIn[0].cf.indexB;
		vOut[numOut].cf.typeA = b2ContactFeature::e_vertex;
		vOut[numOut].cf.typeB = b2ContactFeature::e_face;
		++numOut;
	}

	return numOut;
}

bool b2TestOverlap(const b2Shape& shapeA, child_count_t indexA,
				   const b2Shape& shapeB, child_count_t indexB,
				   const b2Transform& xfA, const b2Transform& xfB)
{
	b2DistanceInput input;
	input.proxyA.Set(shapeA, indexA);
	input.proxyB.Set(shapeB, indexB);
	input.transformA = xfA;
	input.transformB = xfB;
	input.useRadii = true;

	b2SimplexCache cache;
	b2DistanceOutput output;
	b2Distance(&output, &cache, input);

	return output.distance < (10.0f * b2_epsilon);
}
