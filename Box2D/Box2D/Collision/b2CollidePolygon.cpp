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

using index_t = std::size_t;

// Find the max separation between shape1 and shape2 using edge normals from shape1.
static float32 b2FindMaxSeparation(index_t& edgeIndex,
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
				min_sij = sij;
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

static void b2FindIncidentEdge(std::array<b2ClipVertex,2>& c,
							 const b2PolygonShape& shape1, const b2Transform& xf1, index_t index1,
							 const b2PolygonShape& shape2, const b2Transform& xf2)
{
	b2Assert((0 <= index1) && (index1 < shape1.GetVertexCount()));

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
	const auto i2 = ((i1 + 1) < count2) ? i1 + 1 : 0;

	c[0].v = b2Mul(xf2, shape2.GetVertex(i1));
	c[0].id.cf.indexA = (uint8)index1;
	c[0].id.cf.indexB = (uint8)i1;
	c[0].id.cf.typeA = b2ContactFeature::e_face;
	c[0].id.cf.typeB = b2ContactFeature::e_vertex;

	c[1].v = b2Mul(xf2, shape2.GetVertex(i2));
	c[1].id.cf.indexA = (uint8)index1;
	c[1].id.cf.indexB = (uint8)i2;
	c[1].id.cf.typeA = b2ContactFeature::e_face;
	c[1].id.cf.typeB = b2ContactFeature::e_vertex;
}

// Find edge normal of max separation on A - return if separating axis is found
// Find edge normal of max separation on B - return if separation axis is found
// Choose reference edge as min(minA, minB)
// Find incident edge
// Clip

// The normal points from 1 to 2
void b2CollideShapes(b2Manifold* manifold,
					 const b2PolygonShape& shapeA, const b2Transform& xfA,
					 const b2PolygonShape& shapeB, const b2Transform& xfB)
{
	manifold->SetType(b2Manifold::e_unset);
	const auto totalRadius = shapeA.GetRadius() + shapeB.GetRadius();

	auto edgeA = index_t{0};
	const auto separationA = b2FindMaxSeparation(edgeA, shapeA, xfA, shapeB, xfB);
	if (separationA > totalRadius)
		return;

	auto edgeB = index_t{0};
	const auto separationB = b2FindMaxSeparation(edgeB, shapeB, xfB, shapeA, xfA);
	if (separationB > totalRadius)
		return;

	const b2PolygonShape* shape1;	// reference polygon
	const b2PolygonShape* shape2;	// incident polygon
	b2Transform xf1, xf2;
	index_t edge1;					// reference edge
	bool flip;
	constexpr auto k_tol = b2_linearSlop / 10;

	if (separationB > (separationA + k_tol))
	{
		shape1 = &shapeB;
		shape2 = &shapeA;
		xf1 = xfB;
		xf2 = xfA;
		edge1 = edgeB;
		manifold->SetType(b2Manifold::e_faceB);
		flip = true;
	}
	else
	{
		shape1 = &shapeA;
		shape2 = &shapeB;
		xf1 = xfA;
		xf2 = xfB;
		edge1 = edgeA;
		manifold->SetType(b2Manifold::e_faceA);
		flip = false;
	}

	std::array<b2ClipVertex,2> incidentEdge;
	b2FindIncidentEdge(incidentEdge, *shape1, xf1, edge1, *shape2, xf2);

	const auto count1 = shape1->GetVertexCount();

	const auto iv1 = edge1;
	const auto iv2 = ((edge1 + 1) < count1) ? edge1 + 1 : 0;

	auto v11 = b2Vec2(shape1->GetVertex(iv1));
	auto v12 = b2Vec2(shape1->GetVertex(iv2));

	const auto localTangent = b2Normalize(v12 - v11);
	
	const auto localNormal = b2Cross(localTangent, 1.0f);
	const auto planePoint = 0.5f * (v11 + v12);

	manifold->SetLocalNormal(localNormal);
	manifold->SetLocalPoint(planePoint);

	const auto tangent = b2Mul(xf1.q, localTangent);
	const auto normal = b2Cross(tangent, 1.0f);
	
	v11 = b2Mul(xf1, v11);
	v12 = b2Mul(xf1, v12);

	// Face offset.
	const auto frontOffset = b2Dot(normal, v11);

	// Side offsets, extended by polytope skin thickness.
	const auto sideOffset1 = -b2Dot(tangent, v11) + totalRadius;
	const auto sideOffset2 = b2Dot(tangent, v12) + totalRadius;

	// Clip incident edge against extruded edge1 side edges.
	std::array<b2ClipVertex,2> clipPoints1;
	std::array<b2ClipVertex,2> clipPoints2;

	// Clip to box side 1
	if (b2ClipSegmentToLine(clipPoints1, incidentEdge, -tangent, sideOffset1, iv1) < 2)
		return;

	// Clip to negative box side 1
	if (b2ClipSegmentToLine(clipPoints2, clipPoints1,  tangent, sideOffset2, iv2) < 2)
		return;

	// Now clipPoints2 contains the clipped points.
	for (auto i = decltype(b2_maxManifoldPoints){0}; i < b2_maxManifoldPoints; ++i)
	{
		const auto separation = b2Dot(normal, clipPoints2[i].v) - frontOffset;
		if (separation <= totalRadius)
		{
			const auto cf = flip? b2Flip(clipPoints2[i].id.cf): clipPoints2[i].id.cf;
			manifold->AddPoint(b2MulT(xf2, clipPoints2[i].v), cf);
		}
	}
}
