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

// Find the max separation between poly1 and poly2 using edge normals from poly1.
static float32 b2FindMaxSeparation(int32* edgeIndex,
								 const b2PolygonShape* poly1, const b2Transform& xf1,
								 const b2PolygonShape* poly2, const b2Transform& xf2)
{
	const auto count1 = poly1->m_count;
	const auto count2 = poly2->m_count;
	const b2Vec2* n1s = poly1->m_normals;
	const b2Vec2* v1s = poly1->m_vertices;
	const b2Vec2* v2s = poly2->m_vertices;
	b2Transform xf = b2MulT(xf2, xf1);

	int32 bestIndex = 0;
	auto maxSeparation = -b2_maxFloat;
	for (auto i = decltype(count1){0}; i < count1; ++i)
	{
		// Get poly1 normal in frame2.
		const auto n = b2Mul(xf.q, n1s[i]);
		const auto v1 = b2Mul(xf, v1s[i]);

		// Find deepest point for normal i.
		auto si = b2_maxFloat;
		for (auto j = decltype(count2){0}; j < count2; ++j)
		{
			const auto sij = b2Dot(n, v2s[j] - v1);
			if (sij < si)
			{
				si = sij;
			}
		}

		if (si > maxSeparation)
		{
			maxSeparation = si;
			bestIndex = i;
		}
	}

	*edgeIndex = bestIndex;
	return maxSeparation;
}

static void b2FindIncidentEdge(std::array<b2ClipVertex,2>& c,
							 const b2PolygonShape* poly1, const b2Transform& xf1, int32 edge1,
							 const b2PolygonShape* poly2, const b2Transform& xf2)
{
	const b2Vec2* normals1 = poly1->m_normals;

	const auto count2 = poly2->m_count;
	const b2Vec2* vertices2 = poly2->m_vertices;
	const b2Vec2* normals2 = poly2->m_normals;

	b2Assert(0 <= edge1 && edge1 < poly1->m_count);

	// Get the normal of the reference edge in poly2's frame.
	const auto normal1 = b2MulT(xf2.q, b2Mul(xf1.q, normals1[edge1]));

	// Find the incident edge on poly2.
	int32 index = 0;
	auto minDot = b2_maxFloat;
	for (auto i = decltype(count2){0}; i < count2; ++i)
	{
		const auto dot = b2Dot(normal1, normals2[i]);
		if (dot < minDot)
		{
			minDot = dot;
			index = i;
		}
	}

	// Build the clip vertices for the incident edge.
	const auto i1 = index;
	const auto i2 = i1 + 1 < count2 ? i1 + 1 : 0;

	c[0].v = b2Mul(xf2, vertices2[i1]);
	c[0].id.cf.indexA = (uint8)edge1;
	c[0].id.cf.indexB = (uint8)i1;
	c[0].id.cf.typeA = b2ContactFeature::e_face;
	c[0].id.cf.typeB = b2ContactFeature::e_vertex;

	c[1].v = b2Mul(xf2, vertices2[i2]);
	c[1].id.cf.indexA = (uint8)edge1;
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
void b2CollidePolygons(b2Manifold* manifold,
					  const b2PolygonShape* polyA, const b2Transform& xfA,
					  const b2PolygonShape* polyB, const b2Transform& xfB)
{
	manifold->pointCount = 0;
	const auto totalRadius = polyA->m_radius + polyB->m_radius;

	int32 edgeA = 0;
	const auto separationA = b2FindMaxSeparation(&edgeA, polyA, xfA, polyB, xfB);
	if (separationA > totalRadius)
		return;

	int32 edgeB = 0;
	const auto separationB = b2FindMaxSeparation(&edgeB, polyB, xfB, polyA, xfA);
	if (separationB > totalRadius)
		return;

	const b2PolygonShape* poly1;	// reference polygon
	const b2PolygonShape* poly2;	// incident polygon
	b2Transform xf1, xf2;
	int32 edge1;					// reference edge
	bool flip;
	const auto k_tol = 0.1f * b2_linearSlop;

	if (separationB > (separationA + k_tol))
	{
		poly1 = polyB;
		poly2 = polyA;
		xf1 = xfB;
		xf2 = xfA;
		edge1 = edgeB;
		manifold->type = b2Manifold::e_faceB;
		flip = true;
	}
	else
	{
		poly1 = polyA;
		poly2 = polyB;
		xf1 = xfA;
		xf2 = xfB;
		edge1 = edgeA;
		manifold->type = b2Manifold::e_faceA;
		flip = false;
	}

	std::array<b2ClipVertex,2> incidentEdge;
	b2FindIncidentEdge(incidentEdge, poly1, xf1, edge1, poly2, xf2);

	const auto count1 = poly1->m_count;
	const b2Vec2* vertices1 = poly1->m_vertices;

	const auto iv1 = edge1;
	const auto iv2 = ((edge1 + 1) < count1) ? edge1 + 1 : 0;

	b2Vec2 v11 = vertices1[iv1];
	b2Vec2 v12 = vertices1[iv2];

	const auto localTangent = b2Normalize(v12 - v11);
	
	const auto localNormal = b2Cross(localTangent, 1.0f);
	const auto planePoint = 0.5f * (v11 + v12);

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
	{
		return;
	}

	// Clip to negative box side 1
	if (b2ClipSegmentToLine(clipPoints2, clipPoints1,  tangent, sideOffset2, iv2) < 2)
	{
		return;
	}

	// Now clipPoints2 contains the clipped points.
	manifold->localNormal = localNormal;
	manifold->localPoint = planePoint;

	auto pointCount = int32{0};
	for (auto i = decltype(b2_maxManifoldPoints){0}; i < b2_maxManifoldPoints; ++i)
	{
		const auto separation = b2Dot(normal, clipPoints2[i].v) - frontOffset;

		if (separation <= totalRadius)
		{
			auto cp = manifold->points + pointCount;
			cp->localPoint = b2MulT(xf2, clipPoints2[i].v);
			cp->id = clipPoints2[i].id;
			if (flip)
			{
				// Swap features
				const auto cf = cp->id.cf;
				cp->id.cf.indexA = cf.indexB;
				cp->id.cf.indexB = cf.indexA;
				cp->id.cf.typeA = cf.typeB;
				cp->id.cf.typeB = cf.typeA;
			}
			++pointCount;
		}
	}

	manifold->pointCount = pointCount;
}
