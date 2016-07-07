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

struct EdgeSeparation
{
	PolygonShape::vertex_count_t edge;
	float_t separation;
};

// Find the max separation between shape1 and shape2 using edge normals from shape1.
static EdgeSeparation FindMaxSeparation(const PolygonShape& shape1, const Transformation& xf1,
										const PolygonShape& shape2, const Transformation& xf2)
{
	auto maxSeparation = -MaxFloat;
	auto index_of_max = PolygonShape::vertex_count_t{0};
	{
		const auto count1 = shape1.GetVertexCount();
		const auto count2 = shape2.GetVertexCount();
		const auto xf = MulT(xf2, xf1);
		
		for (auto i = decltype(count1){0}; i < count1; ++i)
		{
			// Get shape1 normal in frame2.
			const auto n = Rotate(shape1.GetNormal(i), xf.q);
			const auto v1 = Transform(shape1.GetVertex(i), xf);

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
				index_of_max = i;
			}
		}
	}
	return EdgeSeparation{index_of_max, maxSeparation};
}

static inline ClipArray FindIncidentEdge(PolygonShape::vertex_count_t index1,
									  const PolygonShape& shape1, const Transformation& xf1,
									  const PolygonShape& shape2, const Transformation& xf2)
{
	assert(index1 >= 0);
	assert(index1 < shape1.GetVertexCount());

	const auto count2 = shape2.GetVertexCount();

	// Find the incident edge on shape2.
	auto index_of_min_dot = decltype(count2){0};
	{
		// Get the normal of the reference edge in shape2's frame.
		const auto normal1 = InverseRotate(Rotate(shape1.GetNormal(index1), xf1.q), xf2.q);
		
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
	const auto i2 = static_cast<decltype(i1)>((i1 + 1) % count2);

	return ClipArray{{
		{Transform(shape2.GetVertex(i1), xf2), ContactFeature{ContactFeature::e_face, index1, ContactFeature::e_vertex, i1}},
		{Transform(shape2.GetVertex(i2), xf2), ContactFeature{ContactFeature::e_face, index1, ContactFeature::e_vertex, i2}}
	}};
}

// Find edge normal of max separation on A - return if separating axis is found
// Find edge normal of max separation on B - return if separation axis is found
// Choose reference edge as min(minA, minB)
// Find incident edge
// Clip

// The normal points from 1 to 2
Manifold CollideShapes(const PolygonShape& shapeA, const Transformation& xfA, const PolygonShape& shapeB, const Transformation& xfB)
{
	const auto totalRadius = shapeA.GetRadius() + shapeB.GetRadius();

	const auto edgeSepA = FindMaxSeparation(shapeA, xfA, shapeB, xfB);
	if (edgeSepA.separation > totalRadius)
	{
		return Manifold{};
	}

	const auto edgeSepB = FindMaxSeparation(shapeB, xfB, shapeA, xfA);
	if (edgeSepB.separation > totalRadius)
	{
		return Manifold{};
	}

	const PolygonShape* shape1;	// reference polygon
	const PolygonShape* shape2;	// incident polygon
	Transformation xf1, xf2;
	PolygonShape::vertex_count_t edgeIndex; // reference edge
	bool flip;
	constexpr auto k_tol = LinearSlop / 10;

	Manifold::Type manifoldType;
	if (edgeSepB.separation > (edgeSepA.separation + k_tol))
	{
		shape1 = &shapeB;
		shape2 = &shapeA;
		xf1 = xfB;
		xf2 = xfA;
		edgeIndex = edgeSepB.edge;
		manifoldType = Manifold::e_faceB;
		flip = true;
	}
	else
	{
		shape1 = &shapeA;
		shape2 = &shapeB;
		xf1 = xfA;
		xf2 = xfB;
		edgeIndex = edgeSepA.edge;
		manifoldType = Manifold::e_faceA;
		flip = false;
	}

	const auto incidentEdge = FindIncidentEdge(edgeIndex, *shape1, xf1, *shape2, xf2);

	const auto count1 = shape1->GetVertexCount();

	const auto iv1 = edgeIndex;
	const auto iv2 = static_cast<decltype(iv1)>((edgeIndex + 1) % count1);

	auto v11 = shape1->GetVertex(iv1);
	auto v12 = shape1->GetVertex(iv2);

	const auto localTangent = GetUnitVector(v12 - v11);
	
	const auto localNormal = GetForwardPerpendicular(localTangent);
	const auto planePoint = (v11 + v12) / float_t(2);

	const auto tangent = Rotate(localTangent, xf1.q);
	const auto normal = GetForwardPerpendicular(tangent);
	
	v11 = Transform(v11, xf1);
	v12 = Transform(v12, xf1);

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

	auto manifold = (manifoldType == Manifold::e_faceA)?
		Manifold::GetForFaceA(localNormal, planePoint): Manifold::GetForFaceB(localNormal, planePoint);
	for (auto&& clipPoint: clipPoints2)
	{
		const auto separation = Dot(normal, clipPoint.v) - frontOffset;
		if (separation <= totalRadius)
		{
			const auto cf = flip? Flip(clipPoint.cf): clipPoint.cf;
			manifold.AddPoint(ManifoldPoint{InverseTransform(clipPoint.v, xf2), cf});
		}
	}
	return manifold;
}
	
} // namespace box2d
