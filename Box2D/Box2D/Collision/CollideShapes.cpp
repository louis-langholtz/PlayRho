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

#include <Box2D/Collision/Collision.hpp>
#include <Box2D/Collision/ReferenceFace.hpp>
#include <Box2D/Collision/EdgeInfo.hpp>
#include <Box2D/Collision/CollideShapes.hpp>
#include <Box2D/Collision/Shapes/CircleShape.hpp>
#include <Box2D/Collision/Shapes/PolygonShape.hpp>
#include <Box2D/Collision/Shapes/EdgeShape.hpp>
#include <array>

using namespace box2d;

// This structure is used to keep track of the best separating axis.
struct ShapeSeparation
{
	using index_type = std::remove_const<decltype(MaxShapeVertices)>::type;
	using distance_type = float_t;

	static constexpr index_type InvalidIndex = static_cast<index_type>(-1);
	static constexpr distance_type InvalidDistance = GetInvalid<distance_type>();

	index_type index = InvalidIndex;
	distance_type separation = InvalidDistance;
};

static inline ShapeSeparation GetPolygonSeparation(const PolygonShape& polygon, const EdgeInfo& edge)
{
	auto max_s = -MaxFloat;
	auto index = ShapeSeparation::InvalidIndex;
	{
		const auto totalRadius = GetVertexRadius(polygon) + edge.GetVertexRadius();
		const auto perp = GetRevPerpendicular(edge.GetNormal());
		const auto count = polygon.GetVertexCount();
		const auto edgeVertex1 = edge.GetVertex1();
		const auto edgeVertex2 = edge.GetVertex2();
		for (auto i = decltype(count){0}; i < count; ++i)
		{
			const auto polygonNormal = -polygon.GetNormal(i);
			const auto polygonVertex = polygon.GetVertex(i);
			const auto s = Min(Dot(polygonNormal, polygonVertex - edgeVertex1),
							   Dot(polygonNormal, polygonVertex - edgeVertex2));
			
			if (s > totalRadius) // No collision
			{
				return ShapeSeparation{i, s};
			}
			
			// Adjacency
			if (Dot(polygonNormal, perp) >= 0)
			{
				if (Dot(polygonNormal - edge.GetUpperLimit(), edge.GetNormal()) < BOX2D_MAGIC(-AngularSlop))
				{
					continue;
				}
			}
			else
			{
				if (Dot(polygonNormal - edge.GetLowerLimit(), edge.GetNormal()) < BOX2D_MAGIC(-AngularSlop))
				{
					continue;
				}
			}
			
			if (max_s < s)
			{
				max_s = s;
				index = i;
			}
		}
	}
	return ShapeSeparation{index, max_s};
}

/// Gets the shape separation information for the most opposite vector.
/// @param vectors Collection of 0 or more vectors to find the most anti-parallel vector from and
///    its magnitude from the reference vector.
/// @param refvec Reference vector.
template <typename T>
static inline ShapeSeparation GetMostOppositeSeparation(Span<const T> vectors, const T refvec, const T offset)
{
	// Search for the vector that is most anti-parallel to the reference vector.
	// See: https://en.wikipedia.org/wiki/Antiparallel_(mathematics)#Antiparallel_vectors
	auto bestIndex = ShapeSeparation::InvalidIndex;
	auto minValue = MaxFloat;
	const auto count = vectors.size();
	for (auto i = decltype(count){0}; i < count; ++i)
	{
		// Get cosine of angle between refvec and vectors[i] multiplied by their
		// magnitudes (which will essentially be 1 for any two unit vectors).
		// Get distance from offset to vectors[i] in direction of refvec.
		const auto s = Dot(refvec, vectors[i] - offset);
		if (minValue > s)
		{
			minValue = s;
			bestIndex = static_cast<ShapeSeparation::index_type>(i);
		}
	}
	return ShapeSeparation{bestIndex, minValue};
}

/// Gets the max separation information.
/// @return The index of the vertex and normal from vertices1 and normals1
///   that had the maximum separation distance from any vertex in vertices2 in the
///   direction of that normal and that maximal distance.
static ShapeSeparation
GetMaxSeparation(Span<const Vec2> vertices1, Span<const UnitVec2> normals1, const Transformation& xf1,
				 Span<const Vec2> vertices2, const Transformation& xf2)
{
	assert(vertices1.size() == normals1.size());

	// Find the max separation between shape1 and shape2 using edge normals from shape1.
	auto maxSeparation = -MaxFloat;
	auto index_of_max = ShapeSeparation::index_type{0};
	
	const auto count1 = vertices1.size();
	const auto xf = MulT(xf2, xf1);
	
	for (auto i = decltype(count1){0}; i < count1; ++i)
	{
		// Get shape1 normal and vertex relative to shape2.
		const auto shape1_ni = Rotate(normals1[i], xf.q);
		const auto shape1_vi = Transform(vertices1[i], xf);
		
		const auto s = GetMostOppositeSeparation(vertices2, Vec2{shape1_ni}, shape1_vi).separation;
		if (maxSeparation < s)
		{
			maxSeparation = s;
			index_of_max = static_cast<ShapeSeparation::index_type>(i);
		}
	}
	return ShapeSeparation{index_of_max, maxSeparation};
}

static inline ShapeSeparation GetMaxSeparation(const PolygonShape& shape1, const Transformation& xf1,
											   const PolygonShape& shape2, const Transformation& xf2)
{
	return GetMaxSeparation(shape1.GetVertices(), shape1.GetNormals(), xf1, shape2.GetVertices(), xf2);
}

/// Gets the incident edge clip list.
/// @return Zero one or two face-vertex clip vertices in world coordinates.
static ClipList GetIncidentEdgeClipList(ContactFeature::index_t index1, UnitVec2 normal1, const Transformation& xf1,
										const PolygonShape& shape2, const Transformation& xf2)
{
	const auto count2 = shape2.GetVertexCount();
	if (count2 > 1)
	{
		// Find the incident edge on shape2.
		const auto separation = [&]() {
			// Get the normal of the reference edge in shape2's frame.
			const auto rel_normal1 = InverseRotate(Rotate(normal1, xf1.q), xf2.q);
			return GetMostOppositeSeparation(shape2.GetNormals(), rel_normal1, UnitVec2::GetZero());
		}();
		
		// Build the clip list for the incident edge.
		const auto i1 = separation.index;
		const auto i2 = static_cast<decltype(i1)>((i1 + 1) % count2);
		return ClipList{
			ClipVertex{Transform(shape2.GetVertex(i1), xf2), GetFaceVertexContactFeature(index1, i1)},
			ClipVertex{Transform(shape2.GetVertex(i2), xf2), GetFaceVertexContactFeature(index1, i2)}
		};
	}
	return ClipList{};
}

/// Gets the incident edge clip list.
/// @return Zero one or two face-vertex clip vertices.
static ClipList GetIncidentEdgeClipList(ContactFeature::index_t index1, const UnitVec2 normal1, const PolygonShape& shape2)
{
	const auto separation = GetMostOppositeSeparation(shape2.GetNormals(), normal1, UnitVec2::GetZero());
	const auto i1 = separation.index;
	const auto i2 = static_cast<decltype(i1)>((i1 + 1) % shape2.GetVertexCount());
#if 1
	return ClipList{
		ClipVertex{shape2.GetVertex(i1), GetFaceVertexContactFeature(0, i1)},
		ClipVertex{shape2.GetVertex(i2), GetFaceVertexContactFeature(0, i2)}
	};
#else
	return ClipList{
		ClipVertex{shape2.GetVertex(i1), GetFaceVertexContactFeature(index1, i1)},
		ClipVertex{shape2.GetVertex(i2), GetFaceVertexContactFeature(index1, i2)}
	};
#endif
}

static inline ClipList GetClipPoints(ShapeSeparation::index_type iv1, float_t sideOffset1, UnitVec2 normal1,
									 ShapeSeparation::index_type iv2, float_t sideOffset2, UnitVec2 normal2,
									 const ClipList& incidentEdge)
{
	const auto points = ClipSegmentToLine(incidentEdge, normal1, sideOffset1, iv1);
	return ClipSegmentToLine(points, normal2, sideOffset2, iv2);
}

static Manifold GetManifoldFaceA(const EdgeInfo& edgeInfo,
								 const PolygonShape& localShapeB,
								 const Transformation& xf)
{
	const auto ref_face = GetReferenceFace(edgeInfo);

	// const auto incidentEdge = GetIncidentEdgeClipList(0, edgeInfo.GetNormal(), localShapeB);
	const auto incidentEdge = GetIncidentEdgeClipList(ref_face.GetIndex1(), edgeInfo.GetNormal(), localShapeB);
	
	// Clip incident edge against extruded edge1 side edges.
	const auto clipPoints = GetClipPoints(ref_face.GetIndex1(), ref_face.GetOffset1(), ref_face.GetNormal1(),
										  ref_face.GetIndex2(), ref_face.GetOffset2(), ref_face.GetNormal2(),
										  incidentEdge);
	if (clipPoints.size() != 2)
	{
		return Manifold{};
	}

	auto manifold = Manifold::GetForFaceA(ref_face.GetNormal(), ref_face.GetVertex1());
	const auto totalRadius = edgeInfo.GetVertexRadius() + GetVertexRadius(localShapeB);
	for (auto i = decltype(clipPoints.size()){0}; i < clipPoints.size(); ++i)
	{
		const auto separation = Dot(ref_face.GetNormal(), clipPoints[i].v - ref_face.GetVertex1());
		if (separation <= totalRadius)
		{
			manifold.AddPoint(Manifold::Point{InverseTransform(clipPoints[i].v, xf), clipPoints[i].cf});
		}
	}
	return manifold;
}

static Manifold GetManifoldFaceB(const EdgeInfo& edgeInfo,
								 const PolygonShape& shapeB,
								 const PolygonShape& localShapeB,
								 PolygonShape::vertex_count_t index)
{
#if 1
	const auto incidentEdge = ClipList{
		ClipVertex{edgeInfo.GetVertex1(), GetVertexFaceContactFeature(0, index)},
		ClipVertex{edgeInfo.GetVertex2(), GetVertexFaceContactFeature(0, index)}
	};
#else
	const auto incidentEdge = ClipList{
		ClipVertex{edgeInfo.GetVertex1(), GetFaceVertexContactFeature(index, 0)},
		ClipVertex{edgeInfo.GetVertex2(), GetFaceVertexContactFeature(index, 1)}
	};
#endif
	
	const auto ref_face = GetReferenceFace(localShapeB, index);

	// Clip incident edge against extruded edge1 side edges.
	const auto clipPoints = GetClipPoints(ref_face.GetIndex1(), ref_face.GetOffset1(), ref_face.GetNormal1(),
										  ref_face.GetIndex2(), ref_face.GetOffset2(), ref_face.GetNormal2(),
										  incidentEdge);
	if (clipPoints.size() != 2)
	{
		return Manifold{};
	}
	
	auto manifold = Manifold::GetForFaceB(shapeB.GetNormal(ref_face.GetIndex1()), shapeB.GetVertex(ref_face.GetIndex1()));
	const auto totalRadius = edgeInfo.GetVertexRadius() + GetVertexRadius(shapeB);
	for (auto i = decltype(clipPoints.size()){0}; i < clipPoints.size(); ++i)
	{
		const auto separation = Dot(ref_face.GetNormal(), clipPoints[i].v - ref_face.GetVertex1());
		if (separation <= totalRadius)
		{
			manifold.AddPoint(Manifold::Point{clipPoints[i].v, Flip(clipPoints[i].cf)});
		}
	}
	return manifold;
}

/// @param shape1 Shape 1. This should be shape A for face-A type manifold or shape B for face-B type manifold.
/// @param xf1 Transform 1. This should be transform A for face-A type manifold or transform B for face-B type manifold.
/// @param idx1 Index 1. This should be the index of the vertex and normal of shape1 that had the maximal
///    separation distance from any vertex in shape2.
static inline Manifold GetFaceManifold(const PolygonShape& shape1, const Transformation& xf1,
									   const PolygonShape& shape2, const Transformation& xf2,
									   const ShapeSeparation::index_type idx1,
									   const Manifold::Type type)
{
	const auto totalRadius = GetVertexRadius(shape1) + GetVertexRadius(shape2);
	
	const auto idx2 = static_cast<decltype(idx1)>((idx1 + 1) % (shape1.GetVertexCount()));
	
	const auto shape1_rel_vertex1 = shape1.GetVertex(idx1);
	const auto shape1_rel_vertex2 = shape1.GetVertex(idx2);
	const auto shape1_abs_vertex1 = Transform(shape1_rel_vertex1, xf1);
	const auto shape1_abs_vertex2 = Transform(shape1_rel_vertex2, xf1);
	
	const auto shape1_rel_edge1 = shape1_rel_vertex2 - shape1_rel_vertex1;
	const auto rel_tangent = GetUnitVector(shape1_rel_edge1, UnitVec2::GetZero());
	const auto abs_tangent = Rotate(rel_tangent, xf1.q);
	
	// Clip incident edge against extruded edge1 side edges.
	// Side offsets, extended by polytope skin thickness.
	const auto clipPoints = [&]()
	{
		const auto incidentEdge = GetIncidentEdgeClipList(idx1, shape1.GetNormal(idx1), xf1, shape2, xf2);
		const auto sideOffset1 = -Dot(abs_tangent, shape1_abs_vertex1) + totalRadius;
		const auto sideOffset2 = Dot(abs_tangent, shape1_abs_vertex2) + totalRadius;
		return GetClipPoints(idx1, sideOffset1, -abs_tangent, idx2, sideOffset2, abs_tangent, incidentEdge);
	}();
	if (clipPoints.size() == 2)
	{
		const auto abs_normal = GetFwdPerpendicular(abs_tangent); // Normal points from 1 to 2
		const auto rel_midpoint = (shape1_rel_vertex1 + shape1_rel_vertex2) / float_t(2);
		const auto abs_offset = Dot(abs_normal, shape1_abs_vertex1); ///< Face offset.
		
		switch (type)
		{
			case Manifold::e_faceA:
			{
				auto manifold = Manifold::GetForFaceA(GetFwdPerpendicular(rel_tangent), rel_midpoint);
				for (auto&& cp: clipPoints)
				{
					if ((Dot(abs_normal, cp.v) - abs_offset) <= totalRadius)
					{
						manifold.AddPoint(Manifold::Point{InverseTransform(cp.v, xf2), cp.cf});
					}
				}
				return manifold;
			}
			case Manifold::e_faceB:
			{
				auto manifold = Manifold::GetForFaceB(GetFwdPerpendicular(rel_tangent), rel_midpoint);
				for (auto&& cp: clipPoints)
				{
					if ((Dot(abs_normal, cp.v) - abs_offset) <= totalRadius)
					{
						manifold.AddPoint(Manifold::Point{InverseTransform(cp.v, xf2), Flip(cp.cf)});
					}
				}
				return manifold;
			}
			default:
				break;
		}
	}
	return Manifold{};
}

/*
 * Definition of CollideShapes functions.
 * All CollideShapes functions return a Manifold object.
 */

Manifold box2d::CollideShapes(const CircleShape& shapeA, const Transformation& xfA,
							  const CircleShape& shapeB, const Transformation& xfB)
{
	const auto pA = Transform(shapeA.GetLocation(), xfA);
	const auto pB = Transform(shapeB.GetLocation(), xfB);
	const auto totalRadius = GetVertexRadius(shapeA) + GetVertexRadius(shapeB);
	
	if (GetLengthSquared(pB - pA) > Square(totalRadius))
	{
		return Manifold{};
	}
	return Manifold::GetForCircles(shapeA.GetLocation(), 0, shapeB.GetLocation(), 0);
}

Manifold box2d::CollideShapes(const PolygonShape& shapeA, const Transformation& xfA,
							  const CircleShape& shapeB, const Transformation& xfB)
{
	// Computes the center of the circle in the frame of the polygon.
	const auto cLocal = InverseTransform(Transform(shapeB.GetLocation(), xfB), xfA); ///< Center of circle in frame of polygon.
	
	const auto totalRadius = GetVertexRadius(shapeA) + GetVertexRadius(shapeB);
	const auto vertexCount = shapeA.GetVertexCount();
	
	// Find edge that circle is closest to.
	auto indexOfMax = decltype(vertexCount){0};
	auto maxSeparation = -MaxFloat;
	{
		for (auto i = decltype(vertexCount){0}; i < vertexCount; ++i)
		{
			// Get circle's distance from vertex[i] in direction of normal[i].
			const auto s = Dot(shapeA.GetNormal(i), cLocal - shapeA.GetVertex(i));
			if (s > totalRadius)
			{
				// Early out - no contact.
				return Manifold{};
			}
			if (maxSeparation < s)
			{
				maxSeparation = s;
				indexOfMax = i;
			}
		}
	}
	const auto indexOfMax2 = static_cast<decltype(indexOfMax)>((indexOfMax + 1) % vertexCount);
	assert(maxSeparation <= totalRadius);
	
	// Vertices that subtend the incident face.
	const auto v1 = shapeA.GetVertex(indexOfMax);
	const auto v2 = shapeA.GetVertex(indexOfMax2);
	
	if ((maxSeparation < 0) || almost_zero(maxSeparation))
	{
		// Circle's center is inside the polygon and closest to edge[indexOfMax].
		return Manifold::GetForFaceA(shapeA.GetNormal(indexOfMax), (v1 + v2) / 2, Manifold::Point{shapeB.GetLocation(), GetFaceVertexContactFeature(indexOfMax, 0)});
	}
	
	// Circle's center is outside polygon and closest to edge[indexOfMax].
	// Compute barycentric coordinates.
	
	if (Dot(cLocal - v1, v2 - v1) <= 0)
	{
		// Circle's center right of v1 (in direction of v1 to v2).
		if (GetLengthSquared(cLocal - v1) > Square(totalRadius))
		{
			return Manifold{};
		}
		return Manifold::GetForCircles(v1, indexOfMax, shapeB.GetLocation(), 0);
	}
	
	if (Dot(cLocal - v2, v1 - v2) <= 0)
	{
		// Circle's center left of v2 (in direction of v2 to v1).
		if (GetLengthSquared(cLocal - v2) > Square(totalRadius))
		{
			return Manifold{};
		}
		return Manifold::GetForCircles(v2, indexOfMax2, shapeB.GetLocation(), 0);
	}
	
	// Circle's center is between v1 and v2.
	const auto faceCenter = (v1 + v2) / 2;
	if (Dot(cLocal - faceCenter, shapeA.GetNormal(indexOfMax)) > totalRadius)
	{
		return Manifold{};
	}
	return Manifold::GetForFaceA(shapeA.GetNormal(indexOfMax), faceCenter, Manifold::Point{shapeB.GetLocation(), GetFaceVertexContactFeature(indexOfMax, 0)});
}

Manifold box2d::CollideShapes(const EdgeShape& shapeA, const Transformation& xfA,
							  const CircleShape& shapeB, const Transformation& xfB)
{
	/*
	 * Determine the collision manifold between the edge and the circle. This accounts for
	 * edge connectivity.
	 *
	 * To do this, this code treats the edge like an end-rounded strait-line drawn by an air
	 * brush having the radius that GetVertexRadius(shapeA) returns. As such, the collision is
	 * categorized as either a collision with the first end-point, the second end-point, or the
	 * conceptual rectangle between the end-points.
	 *
	 * Definitions:
	 *   "-" Is an edge parallel to the horizon bounded by vertex A on the left and vertex B
	 *       on the right.
	 *   "|" Is a line perpendicular to the edge.
	 *   "." Is the center of a circle whose position is called Q.
	 *
	 * Then:
	 *   When Q is anywhere to the left of A (as in ".|-"), it's said to be in region A.
	 *   When Q is anywhere to the right of B (as in "-|."), it's said to be in region B.
	 *   Otheriwse (when Q is between A and B), Q is said to be in region AB.
	 */
	
	// Compute circle in frame of edge
	const auto Q = InverseTransform(Transform(shapeB.GetLocation(), xfB), xfA); ///< Circle's position in frame of edge.
	
	const auto A = shapeA.GetVertex1(); ///< Edge shape's vertex 1.
	const auto B = shapeA.GetVertex2(); ///< Edge shape's vertex 2.
	const auto e = B - A; ///< Edge shape's primary edge.
	
	// Barycentric coordinates
	
	const auto totalRadius = GetVertexRadius(shapeA) + GetVertexRadius(shapeB);
	
	// Check if circle's center is relatively left of first vertex of edge - this is "Region A"
	const auto v = Dot(e, Q - A);
	if (v <= 0)
	{
		if (GetLengthSquared(Q - A) > Square(totalRadius))
		{
			return Manifold{};
		}
		
		// Is there an edge connected to A?
		if (shapeA.HasVertex0())
		{
			// Is the circle in Region AB of the previous edge?
			if (Dot(A - shapeA.GetVertex0(), A - Q) > 0)
			{
				return Manifold{};
			}
		}
		return Manifold::GetForCircles(A, 0, shapeB.GetLocation(), 0);
	}
	
	// Check if circle's center is relatively right of second vertex of edge - this is "Region B"
	const auto u = Dot(e, B - Q);
	if (u <= 0)
	{
		if (GetLengthSquared(Q - B) > Square(totalRadius))
		{
			return Manifold{};
		}
		
		// Is there an edge connected to B?
		if (shapeA.HasVertex3())
		{
			// Is the circle in Region AB of the next edge?
			if (Dot(shapeA.GetVertex3() - B, Q - B) > 0)
			{
				return Manifold{};
			}
		}
		return Manifold::GetForCircles(B, 1, shapeB.GetLocation(), 0);
	}
	
	// Region AB
	const auto eLenSquared = GetLengthSquared(e);
	assert(eLenSquared > 0);
	
	if (GetLengthSquared(Q - (u * A + v * B) * (float_t{1} / eLenSquared)) > Square(totalRadius))
	{
		return Manifold{};
	}
	
	const auto ln = GetUnitVector([=]() {
		const auto e_perp = GetRevPerpendicular(e);
		return (Dot(e_perp, Q - A) < 0)? -e_perp: e_perp;
	}(), UnitVec2::GetZero());
	
	return Manifold::GetForFaceA(ln, A, Manifold::Point{shapeB.GetLocation(), GetFaceVertexContactFeature(0, 0)});
}

Manifold box2d::CollideShapes(const EdgeShape& shapeA, const Transformation& xfA,
							  const EdgeShape& shapeB, const Transformation& xfB)
{
	const auto shapeA_v1 = shapeA.GetVertex1(); // p
	const auto shapeA_v2 = shapeA.GetVertex2();
	const auto shapeB_v1 = InverseTransform(Transform(shapeB.GetVertex1(), xfB), xfA); // q
	const auto shapeB_v2 = InverseTransform(Transform(shapeB.GetVertex2(), xfB), xfA);
	const auto shapeA_edge = (shapeA_v2 - shapeA_v1); // r
	const auto shapeB_edge = (shapeB_v2 - shapeB_v1); // s
	const auto shapeA_normal = GetRevPerpendicular(GetUnitVector(shapeA_edge));
	const auto shapeA_len_squared = GetLengthSquared(shapeA_edge); // r . r
	const auto shapeB_len_squared = GetLengthSquared(shapeB_edge); // s . s
	const auto shapeA_extent = GetVertexRadius(shapeA) / Sqrt(shapeA_len_squared);
	const auto shapeB_extent = GetVertexRadius(shapeB) / Sqrt(shapeB_len_squared);
	const auto totalRadius = GetVertexRadius(shapeA) + GetVertexRadius(shapeB);

	// Now solve for:
	// shapeA_v1 + shapeA_c * shapeA_edge == shapeB_v1 + shapeB_c * shapeB_edge

	const auto shapeB_v1_sub_shapeA_v1 = shapeB_v1 - shapeA_v1; // q - p
	const auto cross_edge_A_B = Cross(shapeA_edge, shapeB_edge); // (r × s)
	const auto shapeA_n = Cross(shapeB_v1_sub_shapeA_v1, shapeB_edge); // (q − p) × s
	const auto shapeB_n = Cross(shapeB_v1_sub_shapeA_v1, shapeA_edge); // (q − p) × r
	if (almost_zero(cross_edge_A_B))
	{
		// The two lines are parallel.
		if (almost_zero(shapeB_n))
		{
			// The two lines are collinear (and parallel).
			const auto shapeA_v1_p = shapeA_v1 - (shapeA_extent * shapeA_edge);
			const auto shapeA_v2_p = shapeA_v2 + (shapeA_extent * shapeA_edge);
			const auto shapeA_edge_p = shapeA_v2_p - shapeA_v1_p;
			const auto shapeA_len_squared_p = GetLengthSquared(shapeA_edge_p);
			const auto shapeB_v1_p = shapeB_v1 - (shapeB_extent * shapeB_edge);
			const auto shapeB_v2_p = shapeB_v2 + (shapeB_extent * shapeB_edge);
			const auto shapeB_edge_p = shapeB_v2_p - shapeB_v1_p;
			const auto shapeB_v1_sub_shapeA_v1_p = shapeB_v1_p - shapeA_v1_p;
			const auto dot_edge_B_A = Dot(shapeB_edge_p, shapeA_edge_p);
			const auto shapeA_c0 = Dot(shapeB_v1_sub_shapeA_v1_p, shapeA_edge_p) / shapeA_len_squared_p; // t0
			const auto shapeA_c1 = shapeA_c0 + (dot_edge_B_A / shapeA_len_squared_p); // t1
			const auto interval = (dot_edge_B_A < 0)?
				std::array<float_t, 2>{{shapeA_c1, shapeA_c0}}:
				std::array<float_t, 2>{{shapeA_c0, shapeA_c1}};
			if ((interval[1] >= 0) && (interval[0] <= 1))
			{
				// The line segments are overlapping (and collinear).
				const auto contact_pt = shapeA_v1_p + interval[0] * shapeA_edge_p;
				const auto len_squared_from_shapeA_v1 = GetLengthSquared(shapeA_v1 - contact_pt);
				const auto len_squared_from_shapeA_v2 = GetLengthSquared(shapeA_v2 - contact_pt);
				if (len_squared_from_shapeA_v1 < len_squared_from_shapeA_v2)
				{
					if (len_squared_from_shapeA_v1 >= 0)
					{
						return Manifold::GetForCircles(shapeA_v1, 0, shapeB_v1, 0);						
					}
				}
				else
				{
					if (len_squared_from_shapeA_v2 >= 0)
					{
						return Manifold::GetForCircles(shapeA_v2, 1, shapeB_v1, 0);						
					}
				}
				const auto ln = GetRevPerpendicular(GetUnitVector(shapeA_edge));
				return Manifold::GetForFaceA(ln, contact_pt, Manifold::Point{shapeB_v1, GetFaceFaceContactFeature(0, 0)});
			}
			// The line segments are disjoint (and collinear).
			return Manifold{};
		}
		// The two lines are not collinear (but they are parallel).
		const auto s = Dot(shapeA_normal, shapeB_v1);
		if (Abs(s) > totalRadius)
		{
			return Manifold{};
		}
		GetLengthSquared(shapeA_v1 - shapeB_v1);
		GetLengthSquared(shapeA_v1 - shapeB_v2);
		GetLengthSquared(shapeA_v2 - shapeB_v1);
		GetLengthSquared(shapeA_v2 - shapeB_v2);
	}
	else
	{
		// The two lines are NOT parallel.
		const auto shapeA_c = shapeA_n / cross_edge_A_B; // t = (q − p) × s / (r × s)
		const auto shapeB_c = shapeB_n / cross_edge_A_B; // u = (q − p) × r / (r × s)
		const auto shapeA_c_valid = ((shapeA_c >= 0) && (shapeA_c <= 1));
		const auto shapeB_c_valid = ((shapeB_c >= 0) && (shapeB_c <= 1));
		if (shapeA_c_valid && shapeB_c_valid)
		{
			// The two line segments meet at the point shapeA_v1 + shapeA_c * shapeA_edge
			const auto lp = shapeA_v1 + shapeA_c * shapeA_edge;
			const auto mp = Manifold::Point{shapeB_v1, GetFaceFaceContactFeature(0, 0)};
			return Manifold::GetForFaceA(shapeA_normal, lp, mp);
		}
		else
		{
			// The two line segments are not parallel but do not intersect.
		}
	}
	return Manifold{};
}

Manifold box2d::CollideShapes(const EdgeShape& shapeA, const Transformation& xfA,
							  const PolygonShape& shapeB, const Transformation& xfB)
{
	// Algorithm:
	// 1. Classify v1 and v2
	// 2. Classify polygon centroid as front or back
	// 3. Flip normal if necessary
	// 4. Initialize normal range to [-pi, pi] about face normal
	// 5. Adjust normal range according to adjacent edges
	// 6. Visit each separating axes, only accept axes within the range
	// 7. Return if _any_ axis indicates separation
	// 8. Clip

	const auto xf = MulT(xfA, xfB);
	const auto localShapeB = Transform(shapeB, xf);
	const auto edgeInfo = EdgeInfo{shapeA, localShapeB.GetCentroid()};
	const auto totalRadius = GetVertexRadius(shapeA) + GetVertexRadius(shapeB);

	// If no valid normal can be found then this edge should not collide.
	const auto edgeAxis = GetMostOppositeSeparation(localShapeB.GetVertices(),
													Vec2{edgeInfo.GetNormal()}, edgeInfo.GetVertex1());
	if (edgeAxis.separation > totalRadius)
	{
		return Manifold{};
	}
	
	const auto polygonAxis = GetPolygonSeparation(localShapeB, edgeInfo);
	if (polygonAxis.separation > totalRadius)
	{
		return Manifold{};
	}
	
	// Use hysteresis for jitter reduction.
	constexpr auto k_relativeTol = float_t(0.98);
	constexpr auto k_absoluteTol = LinearSlop / 5; // 0.001
	
	// Now:
	//   (edgeAxis.separation <= MaxSeparation) AND
	//   (polygonAxis.index == EPAxis::InvalidIndex OR polygonAxis.separation <= MaxEPSeparation)
	
	if ((polygonAxis.index != ShapeSeparation::InvalidIndex) &&
		(polygonAxis.separation > ((k_relativeTol * edgeAxis.separation) + k_absoluteTol)))
	{
		return GetManifoldFaceB(edgeInfo, shapeB, localShapeB, polygonAxis.index);
	}
	return GetManifoldFaceA(edgeInfo, localShapeB, xf);	
}

Manifold box2d::CollideShapes(const PolygonShape& shapeA, const Transformation& xfA,
							  const PolygonShape& shapeB, const Transformation& xfB)
{
	// Find edge normal of max separation on A - return if separating axis is found
	// Find edge normal of max separation on B - return if separation axis is found
	// Choose reference edge as min(minA, minB)
	// Find incident edge
	// Clip

	const auto totalRadius = GetVertexRadius(shapeA) + GetVertexRadius(shapeB);
	
	const auto edgeSepA = GetMaxSeparation(shapeA, xfA, shapeB, xfB);
	if (edgeSepA.separation > totalRadius)
	{
		return Manifold{};
	}
	
	const auto edgeSepB = GetMaxSeparation(shapeB, xfB, shapeA, xfA);
	if (edgeSepB.separation > totalRadius)
	{
		return Manifold{};
	}
	
	constexpr auto k_tol = BOX2D_MAGIC(LinearSlop / 10);
	return (edgeSepB.separation > (edgeSepA.separation + k_tol))?
		GetFaceManifold(shapeB, xfB, shapeA, xfA, edgeSepB.index, Manifold::e_faceB):
		GetFaceManifold(shapeA, xfA, shapeB, xfB, edgeSepA.index, Manifold::e_faceA);
}
