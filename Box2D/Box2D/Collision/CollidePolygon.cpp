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

#include <Box2D/Collision/Collision.h>
#include <Box2D/Collision/Shapes/PolygonShape.h>

namespace box2d {

// Find the max separation between shape1 and shape2 using edge normals from shape1.
static float_t FindMaxSeparation(PolygonShape::vertex_count_t& edgeIndex,
								   const PolygonShape& shape1, const Transform& xf1,
								   const PolygonShape& shape2, const Transform& xf2)
{
	auto maxSeparation = -MaxFloat;
	auto shape1_index_of_max_separation = PolygonShape::vertex_count_t{0};
	{
		const auto count1 = shape1.GetVertexCount();
		const auto count2 = shape2.GetVertexCount();
		const auto xf = MulT(xf2, xf1);
		
		for (auto i = decltype(count1){0}; i < count1; ++i)
		{
			// Get shape1 normal in frame2.
			const auto n = Mul(xf.q, shape1.GetNormal(i));
			const auto v1 = Mul(xf, shape1.GetVertex(i));

			// Find deepest point for normal i.
			auto min_sij = MaxFloat;
			for (auto j = decltype(count2){0}; j < count2; ++j)
			{
				const auto sij = Dot(n, shape2.GetVertex(j) - v1);
				min_sij = Min(min_sij, sij);
			}

			if (maxSeparation < min_sij)
			{
				maxSeparation = min_sij;
				shape1_index_of_max_separation = i;
			}
		}
	}
	edgeIndex = shape1_index_of_max_separation;
	return maxSeparation;
}

static inline ClipArray FindIncidentEdge(PolygonShape::vertex_count_t index1,
									  const PolygonShape& shape1, const Transform& xf1,
									  const PolygonShape& shape2, const Transform& xf2)
{
	assert(index1 >= 0);
	assert(index1 < shape1.GetVertexCount());

	const auto count2 = shape2.GetVertexCount();

	// Find the incident edge on shape2.
	auto index_of_min_dot = decltype(count2){0};
	{
		// Get the normal of the reference edge in shape2's frame.
		const auto normal1 = MulT(xf2.q, Mul(xf1.q, shape1.GetNormal(index1)));
		
		auto minDot = MaxFloat;
		for (auto i = decltype(count2){0}; i < count2; ++i)
		{
			const auto dot = Dot(normal1, shape2.GetNormal(i));
			if (minDot > dot)
			{
				minDot = dot;
				index_of_min_dot = i;
			}
		}
	}

	// Build the clip vertices for the incident edge.
	const auto i1 = index_of_min_dot;
	const auto i1_next = static_cast<decltype(i1)>(i1 + 1);
	const auto i2 = (i1_next < count2) ? i1_next: static_cast<decltype(i1)>(0);

	return ClipArray{{
		{Mul(xf2, shape2.GetVertex(i1)), ContactFeature{ContactFeature::e_face, index1, ContactFeature::e_vertex, i1}},
		{Mul(xf2, shape2.GetVertex(i2)), ContactFeature{ContactFeature::e_face, index1, ContactFeature::e_vertex, i2}}
	}};
}

// Find edge normal of max separation on A - return if separating axis is found
// Find edge normal of max separation on B - return if separation axis is found
// Choose reference edge as min(minA, minB)
// Find incident edge
// Clip

// The normal points from 1 to 2
Manifold CollideShapes(const PolygonShape& shapeA, const Transform& xfA, const PolygonShape& shapeB, const Transform& xfB)
{
	const auto totalRadius = shapeA.GetRadius() + shapeB.GetRadius();

	auto edgeIndexA = PolygonShape::vertex_count_t{0};
	const auto separationA = FindMaxSeparation(edgeIndexA, shapeA, xfA, shapeB, xfB);
	if (separationA > totalRadius)
	{
		return Manifold{};
	}

	auto edgeIndexB = PolygonShape::vertex_count_t{0};
	const auto separationB = FindMaxSeparation(edgeIndexB, shapeB, xfB, shapeA, xfA);
	if (separationB > totalRadius)
	{
		return Manifold{};
	}

	const PolygonShape* shape1;	// reference polygon
	const PolygonShape* shape2;	// incident polygon
	Transform xf1, xf2;
	PolygonShape::vertex_count_t edgeIndex1; // reference edge
	bool flip;
	constexpr auto k_tol = LinearSlop / 10;

	auto manifoldType = Manifold::e_unset;
	if (separationB > (separationA + k_tol))
	{
		shape1 = &shapeB;
		shape2 = &shapeA;
		xf1 = xfB;
		xf2 = xfA;
		edgeIndex1 = edgeIndexB;
		manifoldType = Manifold::e_faceB;
		flip = true;
	}
	else
	{
		shape1 = &shapeA;
		shape2 = &shapeB;
		xf1 = xfA;
		xf2 = xfB;
		edgeIndex1 = edgeIndexA;
		manifoldType = Manifold::e_faceA;
		flip = false;
	}

	const auto incidentEdge = FindIncidentEdge(edgeIndex1, *shape1, xf1, *shape2, xf2);

	const auto count1 = shape1->GetVertexCount();

	const auto iv1 = edgeIndex1;
	const auto iv1_next = static_cast<decltype(iv1)>(edgeIndex1 + 1);
	const auto iv2 = (iv1_next < count1)? iv1_next: static_cast<decltype(iv1)>(0);

	auto v11 = shape1->GetVertex(iv1);
	auto v12 = shape1->GetVertex(iv2);

	const auto localTangent = Normalize(v12 - v11);
	
	const auto localNormal = Cross(localTangent, float_t{1});
	const auto planePoint = (v11 + v12) / float_t(2);

	const auto tangent = Mul(xf1.q, localTangent);
	const auto normal = Cross(tangent, float_t{1});
	
	v11 = Mul(xf1, v11);
	v12 = Mul(xf1, v12);

	// Face offset.
	const auto frontOffset = Dot(normal, v11);

	// Side offsets, extended by polytope skin thickness.
	const auto sideOffset1 = -Dot(tangent, v11) + totalRadius;
	const auto sideOffset2 = Dot(tangent, v12) + totalRadius;

	// Clip incident edge against extruded edge1 side edges.

	// Clip to box side 1
	ClipArray clipPoints1;
	if (ClipSegmentToLine(clipPoints1, incidentEdge, -tangent, sideOffset1, iv1) < clipPoints1.size())
	{
		return Manifold{};
	}

	// Clip to negative box side 1
	ClipArray clipPoints2;
	if (ClipSegmentToLine(clipPoints2, clipPoints1,  tangent, sideOffset2, iv2) < clipPoints2.size())
	{
		return Manifold{};
	}

	// Now clipPoints2 contains the clipped points.
	
	auto manifold = Manifold{manifoldType};
	manifold.SetLocalNormal(localNormal);
	manifold.SetLocalPoint(planePoint);
	for (auto i = decltype(clipPoints2.size()){0}; i < clipPoints2.size(); ++i)
	{
		const auto separation = Dot(normal, clipPoints2[i].v) - frontOffset;
		if (separation <= totalRadius)
		{
			const auto cf = flip? Flip(clipPoints2[i].cf): clipPoints2[i].cf;
			manifold.AddPoint(ManifoldPoint{MulT(xf2, clipPoints2[i].v), cf});
		}
	}
	return manifold;
}
	
} // namespace box2d
