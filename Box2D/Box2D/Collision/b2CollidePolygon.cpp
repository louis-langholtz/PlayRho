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

#include <Box2D/Collision/b2Collision.h>
#include <Box2D/Collision/Shapes/b2PolygonShape.h>

namespace box2d {

// Find the max separation between shape1 and shape2 using edge normals from shape1.
static b2Float b2FindMaxSeparation(b2PolygonShape::vertex_count_t& edgeIndex,
								   const b2PolygonShape& shape1, const b2Transform& xf1,
								   const b2PolygonShape& shape2, const b2Transform& xf2)
{
	const auto count1 = shape1.GetVertexCount();
	const auto count2 = shape2.GetVertexCount();
	const auto xf = b2MulT(xf2, xf1);

	auto shape1_index_of_max_separation = decltype(count1){0};
	auto maxSeparation = -b2_maxFloat;
	for (auto i = decltype(count1){0}; i < count1; ++i)
	{
		// Get shape1 normal in frame2.
		const auto n = b2Mul(xf.q, shape1.GetNormal(i));
		const auto v1 = b2Mul(xf, shape1.GetVertex(i));

		// Find deepest point for normal i.
		auto min_sij = b2_maxFloat;
		for (auto j = decltype(count2){0}; j < count2; ++j)
		{
			const auto sij = b2Dot(n, shape2.GetVertex(j) - v1);
			if (min_sij > sij)
			{
				min_sij = sij;
			}
		}

		if (maxSeparation < min_sij)
		{
			maxSeparation = min_sij;
			shape1_index_of_max_separation = i;
		}
	}

	edgeIndex = shape1_index_of_max_separation;
	return maxSeparation;
}

static b2ClipArray b2FindIncidentEdge(b2PolygonShape::vertex_count_t index1,
									  const b2PolygonShape& shape1, const b2Transform& xf1,
									  const b2PolygonShape& shape2, const b2Transform& xf2)
{
	b2Assert(index1 >= 0);
	b2Assert(index1 < shape1.GetVertexCount());

	const auto count2 = shape2.GetVertexCount();

	// Get the normal of the reference edge in shape2's frame.
	const auto normal1 = b2MulT(xf2.q, b2Mul(xf1.q, shape1.GetNormal(index1)));

	// Find the incident edge on shape2.
	auto index_of_min_dot = decltype(count2){0};
	{
		auto minDot = b2_maxFloat;
		for (auto i = decltype(count2){0}; i < count2; ++i)
		{
			const auto dot = b2Dot(normal1, shape2.GetNormal(i));
			if (minDot > dot)
			{
				minDot = dot;
				index_of_min_dot = i;
			}
		}
	}

	// Build the clip vertices for the incident edge.
	const auto i1 = index_of_min_dot;
	const auto i1_next = i1 + 1;
	const auto i2 = (i1_next < count2) ? i1_next: 0;

	return b2ClipArray{{
		{b2Mul(xf2, shape2.GetVertex(i1)), b2ContactFeature(b2ContactFeature::e_face, index1, b2ContactFeature::e_vertex, i1)},
		{b2Mul(xf2, shape2.GetVertex(i2)), b2ContactFeature(b2ContactFeature::e_face, index1, b2ContactFeature::e_vertex, i2)}
	}};
}

// Find edge normal of max separation on A - return if separating axis is found
// Find edge normal of max separation on B - return if separation axis is found
// Choose reference edge as min(minA, minB)
// Find incident edge
// Clip

// The normal points from 1 to 2
b2Manifold b2CollideShapes(const b2PolygonShape& shapeA, const b2Transform& xfA, const b2PolygonShape& shapeB, const b2Transform& xfB)
{
	const auto totalRadius = shapeA.GetRadius() + shapeB.GetRadius();

	auto edgeA = b2PolygonShape::vertex_count_t{0};
	const auto separationA = b2FindMaxSeparation(edgeA, shapeA, xfA, shapeB, xfB);
	if (separationA > totalRadius)
	{
		return b2Manifold{};
	}

	auto edgeB = b2PolygonShape::vertex_count_t{0};
	const auto separationB = b2FindMaxSeparation(edgeB, shapeB, xfB, shapeA, xfA);
	if (separationB > totalRadius)
	{
		return b2Manifold{};
	}

	const b2PolygonShape* shape1;	// reference polygon
	const b2PolygonShape* shape2;	// incident polygon
	b2Transform xf1, xf2;
	b2PolygonShape::vertex_count_t edge1; // reference edge
	bool flip;
	constexpr auto k_tol = b2_linearSlop / 10;

	auto manifoldType = b2Manifold::e_unset;
	if (separationB > (separationA + k_tol))
	{
		shape1 = &shapeB;
		shape2 = &shapeA;
		xf1 = xfB;
		xf2 = xfA;
		edge1 = edgeB;
		manifoldType = b2Manifold::e_faceB;
		flip = true;
	}
	else
	{
		shape1 = &shapeA;
		shape2 = &shapeB;
		xf1 = xfA;
		xf2 = xfB;
		edge1 = edgeA;
		manifoldType = b2Manifold::e_faceA;
		flip = false;
	}

	const auto incidentEdge = b2FindIncidentEdge(edge1, *shape1, xf1, *shape2, xf2);

	const auto count1 = shape1->GetVertexCount();

	const auto iv1 = edge1;
	const auto iv1_next = edge1 + 1;
	const auto iv2 = (iv1_next < count1)? iv1_next: 0;

	auto v11 = shape1->GetVertex(iv1);
	auto v12 = shape1->GetVertex(iv2);

	const auto localTangent = b2Normalize(v12 - v11);
	
	const auto localNormal = b2Cross(localTangent, b2Float(1));
	const auto planePoint = (v11 + v12) / b2Float(2);

	const auto tangent = b2Mul(xf1.q, localTangent);
	const auto normal = b2Cross(tangent, b2Float(1));
	
	v11 = b2Mul(xf1, v11);
	v12 = b2Mul(xf1, v12);

	// Face offset.
	const auto frontOffset = b2Dot(normal, v11);

	// Side offsets, extended by polytope skin thickness.
	const auto sideOffset1 = -b2Dot(tangent, v11) + totalRadius;
	const auto sideOffset2 = b2Dot(tangent, v12) + totalRadius;

	// Clip incident edge against extruded edge1 side edges.

	// Clip to box side 1
	b2ClipArray clipPoints1;
	if (b2ClipSegmentToLine(clipPoints1, incidentEdge, -tangent, sideOffset1, iv1) < clipPoints1.size())
	{
		return b2Manifold{};
	}

	// Clip to negative box side 1
	b2ClipArray clipPoints2;
	if (b2ClipSegmentToLine(clipPoints2, clipPoints1,  tangent, sideOffset2, iv2) < clipPoints2.size())
	{
		return b2Manifold{};
	}

	// Now clipPoints2 contains the clipped points.
	
	auto manifold = b2Manifold{manifoldType};
	manifold.SetLocalNormal(localNormal);
	manifold.SetLocalPoint(planePoint);
	for (auto i = decltype(clipPoints2.size()){0}; i < clipPoints2.size(); ++i)
	{
		const auto separation = b2Dot(normal, clipPoints2[i].v) - frontOffset;
		if (separation <= totalRadius)
		{
			const auto cf = flip? b2Flip(clipPoints2[i].cf): clipPoints2[i].cf;
			manifold.AddPoint(b2MulT(xf2, clipPoints2[i].v), cf);
		}
	}
	return manifold;
}
	
} // namespace box2d
