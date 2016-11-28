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
#include <Box2D/Collision/CollideShapes.hpp>
#include <Box2D/Collision/Shapes/CircleShape.hpp>
#include <Box2D/Collision/Shapes/PolygonShape.hpp>
#include <Box2D/Collision/Shapes/EdgeShape.hpp>

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

// Reference face used for clipping
struct ReferenceFace
{
	using index_t = PolygonShape::vertex_count_t;
	
	index_t i1, i2;
	
	Vec2 v1, v2;
	
	UnitVec2 normal;
	
	UnitVec2 sideNormal1;
	float_t sideOffset1;
	
	UnitVec2 sideNormal2;
	float_t sideOffset2;
};

class EdgeInfo
{
public:
	EdgeInfo() = default;
	
	EdgeInfo(const EdgeShape& edge, const Vec2 centroid);
	
	Vec2 GetVertex1() const noexcept { return m_vertex1; }
	Vec2 GetVertex2() const noexcept { return m_vertex2; }
	UnitVec2 GetEdge1() const noexcept { return m_edge1; }
	UnitVec2 GetNormal1() const noexcept { return m_normal1; }
	
	bool IsFront() const noexcept { return m_front; }

	/// Gets the normal.
	/// @return Value of normal 1 or the negative of it depending on whether
	///   is-front is true or not (respectively).
	UnitVec2 GetNormal() const noexcept
	{
		// Alternatively:
		//   return m_front? m_normal1: -m_normal1;
		return m_normal;
	}
	
	UnitVec2 GetLowerLimit() const noexcept { return m_lowerLimit; }
	
	UnitVec2 GetUpperLimit() const noexcept { return m_upperLimit; }
	
	float_t GetVertexRadius() const noexcept { return m_vertexRadius; }

private:
	Vec2 m_vertex1;
	Vec2 m_vertex2;
	UnitVec2 m_edge1; ///< Edge 1. @detail A unit vector of edge shape's vertex 2 - vertex 1.
	UnitVec2 m_normal1; ///< Normal 1. @detail Forward perpendicular of edge 1.
	
	bool m_front;

	/// Normal.
	/// @detail This is the cached value of <code>m_normal1</code> or the negative of it depending
	///   on whether <code>m_front</code> is true or not (respectively).
	UnitVec2 m_normal;
	
	UnitVec2 m_lowerLimit;
	UnitVec2 m_upperLimit;
	
	float_t m_vertexRadius;

	void SetNormalLowerUpper(UnitVec2 normal, UnitVec2 lower, UnitVec2 upper) noexcept
	{
		m_normal = normal;
		m_lowerLimit = lower;
		m_upperLimit = upper;
	}
};

inline EdgeInfo::EdgeInfo(const EdgeShape& edge, const Vec2 centroid):
	m_vertex1(edge.GetVertex1()),
	m_vertex2(edge.GetVertex2()),
	m_edge1(GetUnitVector(edge.GetVertex2() - edge.GetVertex1(), UnitVec2::GetZero())),
	m_normal1(GetFwdPerpendicular(m_edge1)),
	m_vertexRadius(box2d::GetVertexRadius(edge))
{
	const auto hasVertex0 = edge.HasVertex0();
	const auto hasVertex3 = edge.HasVertex3();
	
	const auto offset1 = Dot(m_normal1, centroid - m_vertex1);
	
	// Determine front or back collision. Determine collision normal limits.
	if (hasVertex0 && hasVertex3)
	{
		const auto vertex0 = edge.GetVertex0();
		const auto edge0 = GetUnitVector(m_vertex1 - vertex0, UnitVec2::GetZero());
		const auto normal0 = GetFwdPerpendicular(edge0);
		const auto convex1 = Cross(edge0, m_edge1) >= float_t{0};
		const auto offset0 = Dot(normal0, centroid - vertex0);
		
		const auto vertex3 = edge.GetVertex3();
		const auto edge2 = GetUnitVector(vertex3 - m_vertex2, UnitVec2::GetZero());
		const auto normal2 = GetFwdPerpendicular(edge2);
		const auto convex2 = Cross(m_edge1, edge2) > float_t{0};
		const auto offset2 = Dot(normal2, centroid - m_vertex2);
		
		if (convex1 && convex2)
		{
			m_front = (offset0 >= 0) || (offset1 >= 0) || (offset2 >= 0);
			if (m_front)
			{
				SetNormalLowerUpper(m_normal1, normal0, normal2);
			}
			else
			{
				SetNormalLowerUpper(-m_normal1, -m_normal1, -m_normal1);
			}
		}
		else if (convex1)
		{
			m_front = (offset0 >= 0) || ((offset1 >= 0) && (offset2 >= 0));
			if (m_front)
			{
				SetNormalLowerUpper(m_normal1, normal0, m_normal1);
			}
			else
			{
				SetNormalLowerUpper(-m_normal1, -normal2, -m_normal1);
			}
		}
		else if (convex2)
		{
			m_front = (offset2 >= 0) || ((offset0 >= 0) && (offset1 >= 0));
			if (m_front)
			{
				SetNormalLowerUpper(m_normal1, m_normal1, normal2);
			}
			else
			{
				SetNormalLowerUpper(-m_normal1, -m_normal1, -normal0);
			}
		}
		else // !convex1 && !convex2
		{
			m_front = (offset0 >= 0) && (offset1 >= 0) && (offset2 >= 0);
			if (m_front)
			{
				SetNormalLowerUpper(m_normal1, m_normal1, m_normal1);
			}
			else
			{
				SetNormalLowerUpper(-m_normal1, -normal2, -normal0);
			}
		}
	}
	else if (hasVertex0)
	{
		const auto vertex0 = edge.GetVertex0();
		const auto edge0 = GetUnitVector(m_vertex1 - vertex0, UnitVec2::GetZero());
		const auto normal0 = GetFwdPerpendicular(edge0);
		const auto convex1 = Cross(edge0, m_edge1) >= float_t{0};
		const auto offset0 = Dot(normal0, centroid - vertex0);
		
		if (convex1)
		{
			m_front = (offset0 >= 0) || (offset1 >= 0);
			if (m_front)
			{
				SetNormalLowerUpper(m_normal1, normal0, -m_normal1);
			}
			else
			{
				SetNormalLowerUpper(-m_normal1, m_normal1, -m_normal1);
			}
		}
		else // !convex1
		{
			m_front = (offset0 >= 0) && (offset1 >= 0);
			if (m_front)
			{
				SetNormalLowerUpper(m_normal1, m_normal1, -m_normal1);
			}
			else
			{
				SetNormalLowerUpper(-m_normal1, m_normal1, -normal0);
			}
		}
	}
	else if (hasVertex3)
	{
		const auto vertex3 = edge.GetVertex3();
		const auto edge2 = GetUnitVector(vertex3 - m_vertex2, UnitVec2::GetZero());
		const auto normal2 = GetFwdPerpendicular(edge2);
		const auto convex2 = Cross(m_edge1, edge2) > float_t{0};
		const auto offset2 = Dot(normal2, centroid - m_vertex2);
		
		if (convex2)
		{
			m_front = (offset1 >= 0) || (offset2 >= 0);
			if (m_front)
			{
				SetNormalLowerUpper(m_normal1, -m_normal1, normal2);
			}
			else
			{
				SetNormalLowerUpper(-m_normal1, -m_normal1, m_normal1);
			}
		}
		else // !convex2
		{
			m_front = (offset1 >= 0) && (offset2 >= 0);
			if (m_front)
			{
				SetNormalLowerUpper(m_normal1, -m_normal1, m_normal1);
			}
			else
			{
				SetNormalLowerUpper(-m_normal1, -normal2, m_normal1);
			}
		}		
	}
	else // !hasVertex0 && !hasVertex3
	{
		m_front = offset1 >= 0;
		if (m_front)
		{
			SetNormalLowerUpper(m_normal1, -m_normal1, -m_normal1);
		}
		else
		{
			SetNormalLowerUpper(-m_normal1, m_normal1, m_normal1);
		}
	}
}

/// Gets the shape separation information for the most opposite vector.
/// @param vectors Collection of 0 or more vectors to find the most anti-parallel vector from and
///    its magnitude from the reference vector.
/// @param refvec Reference vector.
template <typename T>
static inline ShapeSeparation GetMostOppositeSeparation(Span<const T> vectors, const T refvec, const T offset)
{
	auto bestIndex = ShapeSeparation::InvalidIndex;
	auto minValue = MaxFloat;
	const auto count = vectors.size();
	for (auto i = decltype(count){0}; i < count; ++i)
	{
		// Get cosine of angle between refvec and vectors[i] multiplied by their
		// magnitudes (which will essentially be 1 for any two unit vectors).
		const auto s = Dot(refvec, vectors[i] - offset);
		if (minValue > s)
		{
			minValue = s;
			bestIndex = static_cast<ShapeSeparation::index_type>(i);
		}
	}
	return ShapeSeparation{bestIndex, minValue};
}

static inline ReferenceFace GetReferenceFace(const EdgeInfo& edgeInfo)
{
	ReferenceFace rf;
	if (edgeInfo.IsFront())
	{
		rf.i1 = 0;
		rf.i2 = 1;
		rf.v1 = edgeInfo.GetVertex1();
		rf.v2 = edgeInfo.GetVertex2();
		rf.normal = edgeInfo.GetNormal1();
	}
	else
	{
		rf.i1 = 1;
		rf.i2 = 0;
		rf.v1 = edgeInfo.GetVertex2();
		rf.v2 = edgeInfo.GetVertex1();
		rf.normal = -edgeInfo.GetNormal1();
	}
	rf.sideNormal1 = GetFwdPerpendicular(rf.normal);
	rf.sideNormal2 = -rf.sideNormal1;
	rf.sideOffset1 = Dot(rf.sideNormal1, rf.v1);
	rf.sideOffset2 = Dot(rf.sideNormal2, rf.v2);
	return rf;
}

static inline ReferenceFace GetReferenceFace(const PolygonShape& localShapeB,
											 const PolygonShape::vertex_count_t index)
{
	ReferenceFace rf;
	rf.i1 = index;
	rf.i2 = static_cast<decltype(rf.i1)>((rf.i1 + 1) % localShapeB.GetVertexCount());
	rf.v1 = localShapeB.GetVertex(rf.i1);
	rf.v2 = localShapeB.GetVertex(rf.i2);
	rf.normal = localShapeB.GetNormal(rf.i1);
	rf.sideNormal1 = GetFwdPerpendicular(rf.normal);
	rf.sideNormal2 = -rf.sideNormal1;
	rf.sideOffset1 = Dot(rf.sideNormal1, rf.v1);
	rf.sideOffset2 = Dot(rf.sideNormal2, rf.v2);
	return rf;
}

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
				if (Dot(polygonNormal - edge.GetUpperLimit(), edge.GetNormal()) < -AngularSlop)
				{
					continue;
				}
			}
			else
			{
				if (Dot(polygonNormal - edge.GetLowerLimit(), edge.GetNormal()) < -AngularSlop)
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

static ShapeSeparation GetMaxSeparation(const PolygonShape& shape1, const Transformation& xf1,
										const PolygonShape& shape2, const Transformation& xf2)
{
	// Find the max separation between shape1 and shape2 using edge normals from shape1.
	auto maxSeparation = -MaxFloat;
	auto index_of_max = PolygonShape::vertex_count_t{0};
	{
		const auto count1 = shape1.GetVertexCount();
		const auto count2 = shape2.GetVertexCount();
		const auto xf = MulT(xf2, xf1);
		
		for (auto i = decltype(count1){0}; i < count1; ++i)
		{
			// Get shape1 normal in frame2.
			const auto shape1_ni = Rotate(shape1.GetNormal(i), xf.q);
			const auto shape1_vi = Transform(shape1.GetVertex(i), xf);
			
			// Find deepest point for normal i.
			auto min_sij = MaxFloat;
			for (auto j = decltype(count2){0}; j < count2; ++j)
			{
				const auto sij = Dot(shape1_ni, shape2.GetVertex(j) - shape1_vi);
				min_sij = Min(min_sij, sij);
			}
			
			if (maxSeparation < min_sij)
			{
				maxSeparation = min_sij;
				index_of_max = i;
			}
		}
	}
	return ShapeSeparation{index_of_max, maxSeparation};
}

static ClipList FindIncidentEdge(const PolygonShape::vertex_count_t index1,
								 const PolygonShape& shape1, const Transformation& xf1,
								 const PolygonShape& shape2, const Transformation& xf2)
{
	ClipList list;
	
	const auto count1 = shape1.GetVertexCount();
	const auto count2 = shape2.GetVertexCount();
	if ((index1 < count1) && (count2 > 1))
	{
		// Find the incident edge on shape2.
		auto index_of_min = PolygonShape::InvalidVertex;
		{
			// Get the normal of the reference edge in shape2's frame.
			const auto normal1 = InverseRotate(Rotate(shape1.GetNormal(index1), xf1.q), xf2.q);
			
			auto min_s = MaxFloat;
			for (auto i = decltype(count2){0}; i < count2; ++i)
			{
				const auto s = Dot(normal1, shape2.GetNormal(i));
				if (min_s > s)
				{
					min_s = s;
					index_of_min = i;
				}
			}
		}
		
		// Build the clip vertices for the incident edge.
		const auto i1 = index_of_min;
		const auto i2 = static_cast<decltype(i1)>((i1 + 1) % count2);
		
		list.add(ClipVertex{
			Transform(shape2.GetVertex(i1), xf2),
			ContactFeature{ContactFeature::e_face, index1, ContactFeature::e_vertex, i1}
		});
		list.add(ClipVertex{
			Transform(shape2.GetVertex(i2), xf2),
			ContactFeature{ContactFeature::e_face, index1, ContactFeature::e_vertex, i2}
		});
	}
	
	return list;
}

static Manifold GetManifoldFaceA(const EdgeInfo& edgeInfo,
								 const PolygonShape& localShapeB,
								 const Transformation& xf)
{
	const auto incidentEdge = [&]()
	{
		ClipList list;

		// Search for the polygon normal that is most anti-parallel to the edge normal.
		// See: https://en.wikipedia.org/wiki/Antiparallel_(mathematics)#Antiparallel_vectors
		const auto separation = GetMostOppositeSeparation(localShapeB.GetNormals(),
														  edgeInfo.GetNormal(), UnitVec2::GetZero());
		const auto i1 = separation.index;
		const auto i2 = static_cast<decltype(i1)>((i1 + 1) % localShapeB.GetVertexCount());
		
		list.add(ClipVertex{
			localShapeB.GetVertex(i1),
			ContactFeature{ContactFeature::e_face, 0, ContactFeature::e_vertex, i1}
		});
		list.add(ClipVertex{
			localShapeB.GetVertex(i2),
			ContactFeature{ContactFeature::e_face, 0, ContactFeature::e_vertex, i2}
		});
		return list;
	}();
	
	const auto ref_face = GetReferenceFace(edgeInfo);

	// Clip incident edge against extruded edge1 side edges.
	const auto clipPoints = [&]()
	{
		const auto points = ClipSegmentToLine(incidentEdge, ref_face.sideNormal1, ref_face.sideOffset1, ref_face.i1);
		return ClipSegmentToLine(points, ref_face.sideNormal2, ref_face.sideOffset2, ref_face.i2);
	}();
	if (clipPoints.size() != 2)
	{
		return Manifold{};
	}

	auto manifold = Manifold::GetForFaceA(ref_face.normal, ref_face.v1);
	const auto totalRadius = edgeInfo.GetVertexRadius() + GetVertexRadius(localShapeB);
	for (auto i = decltype(clipPoints.size()){0}; i < clipPoints.size(); ++i)
	{
		const auto separation = Dot(ref_face.normal, clipPoints[i].v - ref_face.v1);
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
	const auto incidentEdge = [&]()
	{
		ClipList list;
		list.add(ClipVertex{
			edgeInfo.GetVertex1(),
			ContactFeature{ContactFeature::e_vertex, 0, ContactFeature::e_face, index}
		});
		list.add(ClipVertex{
			edgeInfo.GetVertex2(),
			ContactFeature{ContactFeature::e_vertex, 0, ContactFeature::e_face, index}
		});
		return list;
	}();
	
	const auto ref_face = GetReferenceFace(localShapeB, index);

	// Clip incident edge against extruded edge1 side edges.
	const auto clipPoints = [&]()
	{
		const auto points = ClipSegmentToLine(incidentEdge, ref_face.sideNormal1, ref_face.sideOffset1, ref_face.i1);
		return ClipSegmentToLine(points, ref_face.sideNormal2, ref_face.sideOffset2, ref_face.i2);
	}();
	if (clipPoints.size() != 2)
	{
		return Manifold{};
	}
	
	auto manifold = Manifold::GetForFaceB(shapeB.GetNormal(ref_face.i1), shapeB.GetVertex(ref_face.i1));
	const auto totalRadius = edgeInfo.GetVertexRadius() + GetVertexRadius(shapeB);
	for (auto i = decltype(clipPoints.size()){0}; i < clipPoints.size(); ++i)
	{
		const auto separation = Dot(ref_face.normal, clipPoints[i].v - ref_face.v1);
		if (separation <= totalRadius)
		{
			manifold.AddPoint(Manifold::Point{clipPoints[i].v, Flip(clipPoints[i].cf)});
		}
	}
	return manifold;
}

static inline Manifold GetManifoldFaceA(const PolygonShape& shape1, const Transformation& xf1,
										const PolygonShape& shape2, const Transformation& xf2,
										const PolygonShape::vertex_count_t iv1)
{
	const auto totalRadius = GetVertexRadius(shape1) + GetVertexRadius(shape2);
	
	const auto iv2 = static_cast<decltype(iv1)>((iv1 + 1) % (shape1.GetVertexCount()));
	
	const auto shape1_rel_vertex_iv1 = shape1.GetVertex(iv1);
	const auto shape1_rel_vertex_iv2 = shape1.GetVertex(iv2);
	const auto shape1_abs_vertex_iv1 = Transform(shape1_rel_vertex_iv1, xf1);
	const auto shape1_abs_vertex_iv2 = Transform(shape1_rel_vertex_iv2, xf1);
	
	const auto localTangent = GetUnitVector(shape1_rel_vertex_iv2 - shape1_rel_vertex_iv1, UnitVec2::GetZero());
	const auto tangent = Rotate(localTangent, xf1.q);
	
	// Clip incident edge against extruded edge1 side edges.
	// Side offsets, extended by polytope skin thickness.
	const auto clipPoints = [&]()
	{
		const auto incidentEdge = FindIncidentEdge(iv1, shape1, xf1, shape2, xf2);
		const auto sideOffset1 = -Dot(tangent, shape1_abs_vertex_iv1) + totalRadius;
		const auto sideOffset2 = Dot(tangent, shape1_abs_vertex_iv2) + totalRadius;
		const auto points = ClipSegmentToLine(incidentEdge, -tangent, sideOffset1, iv1);
		return ClipSegmentToLine(points, tangent, sideOffset2, iv2);
	}();
	if (clipPoints.size() == 2)
	{
		const auto planePoint = (shape1_rel_vertex_iv1 + shape1_rel_vertex_iv2) / float_t(2);
		auto manifold = Manifold::GetForFaceA(GetFwdPerpendicular(localTangent), planePoint);
		
		// The normal points from 1 to 2
		const auto normal = GetFwdPerpendicular(tangent);
		
		const auto frontOffset = Dot(normal, shape1_abs_vertex_iv1); ///< Face offset.
		for (auto&& cp: clipPoints)
		{
			if ((Dot(normal, cp.v) - frontOffset) <= totalRadius)
			{
				manifold.AddPoint(Manifold::Point{InverseTransform(cp.v, xf2), cp.cf});
			}
		}		
		return manifold;
	}
	return Manifold{};	
}

static inline Manifold GetManifoldFaceB(const PolygonShape& shape1, const Transformation& xf1,
										const PolygonShape& shape2, const Transformation& xf2,
										const PolygonShape::vertex_count_t iv1)
{
	const auto totalRadius = GetVertexRadius(shape1) + GetVertexRadius(shape2);
	
	const auto iv2 = static_cast<decltype(iv1)>((iv1 + 1) % (shape1.GetVertexCount()));
	
	const auto shape1_rel_vertex_iv1 = shape1.GetVertex(iv1);
	const auto shape1_rel_vertex_iv2 = shape1.GetVertex(iv2);
	const auto shape1_abs_vertex_iv1 = Transform(shape1_rel_vertex_iv1, xf1);
	const auto shape1_abs_vertex_iv2 = Transform(shape1_rel_vertex_iv2, xf1);
	
	const auto localTangent = GetUnitVector(shape1_rel_vertex_iv2 - shape1_rel_vertex_iv1, UnitVec2::GetZero());
	const auto tangent = Rotate(localTangent, xf1.q);
	
	// Clip incident edge against extruded edge1 side edges.
	// Side offsets, extended by polytope skin thickness.
	const auto clipPoints = [&]()
	{
		const auto incidentEdge = FindIncidentEdge(iv1, shape1, xf1, shape2, xf2);
		const auto sideOffset1 = -Dot(tangent, shape1_abs_vertex_iv1) + totalRadius;
		const auto sideOffset2 = Dot(tangent, shape1_abs_vertex_iv2) + totalRadius;
		const auto points = ClipSegmentToLine(incidentEdge, -tangent, sideOffset1, iv1);
		return ClipSegmentToLine(points, tangent, sideOffset2, iv2);
	}();
	if (clipPoints.size() == 2)
	{
		const auto planePoint = (shape1_rel_vertex_iv1 + shape1_rel_vertex_iv2) / float_t(2);
		auto manifold = Manifold::GetForFaceB(GetFwdPerpendicular(localTangent), planePoint);
		
		// The normal points from 1 to 2
		const auto normal = GetFwdPerpendicular(tangent);
		
		const auto frontOffset = Dot(normal, shape1_abs_vertex_iv1); ///< Face offset.
		for (auto&& cp: clipPoints)
		{
			if ((Dot(normal, cp.v) - frontOffset) <= totalRadius)
			{
				manifold.AddPoint(Manifold::Point{InverseTransform(cp.v, xf2), Flip(cp.cf)});
			}
		}
		return manifold;	
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
	return Manifold::GetForCircles(shapeA.GetLocation(), Manifold::Point{shapeB.GetLocation()});
}

Manifold box2d::CollideShapes(const PolygonShape& shapeA, const Transformation& xfA,
							  const CircleShape& shapeB, const Transformation& xfB)
{
	// Computes the center of the circle in the frame of the polygon.
	const auto cLocal = InverseTransform(Transform(shapeB.GetLocation(), xfB), xfA); ///< Center of the circle in the frame of the polygon.
	
	const auto totalRadius = GetVertexRadius(shapeA) + GetVertexRadius(shapeB);
	const auto totalRadiusSquared = Square(totalRadius);
	const auto vertexCount = shapeA.GetVertexCount();
	
	// Find edge that circle is closest to.
	auto indexOfMax = decltype(vertexCount){0};
	auto maxSeparation = -MaxFloat;
	{
		auto s0 = Dot(shapeA.GetNormal(vertexCount - 1), cLocal - shapeA.GetVertex(vertexCount - 1));
		for (auto i = decltype(vertexCount){0}; i < vertexCount; ++i)
		{
			// Get circle's distance from vertex[i] in direction of normal[i].
			const auto s = Dot(shapeA.GetNormal(i), cLocal - shapeA.GetVertex(i));
			if (s > totalRadius)
			{
				// Early out - no contact.
				return Manifold{};
			}
			if ((s > 0) && (s0 > 0) && (s0 <= totalRadius))
			{
				if (GetLengthSquared(cLocal - shapeA.GetVertex(i)) <= totalRadiusSquared)
				{
					return Manifold::GetForCircles(shapeA.GetVertex(i), Manifold::Point{shapeB.GetLocation()});
				}
			}
			s0 = s;
			if (maxSeparation < s)
			{
				maxSeparation = s;
				indexOfMax = i;
			}
		}
	}
	assert(maxSeparation <= totalRadius);
	
	// Vertices that subtend the incident face.
	const auto v1 = shapeA.GetVertex(indexOfMax);
	const auto v2 = shapeA.GetVertex(static_cast<decltype(indexOfMax)>((indexOfMax + 1) % vertexCount));
	
	if ((maxSeparation < 0) || almost_zero(maxSeparation))
	{
		// Circle's center is inside the polygon and closest to edge[indexOfMax].
		return Manifold::GetForFaceA(shapeA.GetNormal(indexOfMax), (v1 + v2) / 2, Manifold::Point{shapeB.GetLocation()});
	}
	
	// Circle's center is outside polygon and closest to edge[indexOfMax].
	// Compute barycentric coordinates.
	
	if (Dot(cLocal - v1, v2 - v1) <= 0)
	{
		// Circle's center closest to v1 but not between v1 and v2.
		if (GetLengthSquared(cLocal - v1) > Square(totalRadius))
		{
			return Manifold{};
		}
		return Manifold::GetForFaceA(GetUnitVector(cLocal - v1, UnitVec2::GetZero()), v1, Manifold::Point{shapeB.GetLocation()});
	}
	
	if (Dot(cLocal - v2, v1 - v2) <= 0)
	{
		// Circle's center closest to v2 but not between v1 and v2.
		if (GetLengthSquared(cLocal - v2) > Square(totalRadius))
		{
			return Manifold{};
		}
		return Manifold::GetForFaceA(GetUnitVector(cLocal - v2, UnitVec2::GetZero()), v2, Manifold::Point{shapeB.GetLocation()});
	}
	
	// Circle's center is between v1 and v2.
	const auto faceCenter = (v1 + v2) / 2;
	if (Dot(cLocal - faceCenter, shapeA.GetNormal(indexOfMax)) > totalRadius)
	{
		return Manifold{};
	}
	return Manifold::GetForFaceA(shapeA.GetNormal(indexOfMax), faceCenter, Manifold::Point{shapeB.GetLocation()});
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
		return Manifold::GetForCircles(A, Manifold::Point{
			shapeB.GetLocation(),
			ContactFeature{ContactFeature::e_vertex, 0, ContactFeature::e_vertex, 0}
		});
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
		return Manifold::GetForCircles(B, Manifold::Point{
			shapeB.GetLocation(),
			ContactFeature{ContactFeature::e_vertex, 1, ContactFeature::e_vertex, 0}
		});
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
	
	return Manifold::GetForFaceA(ln, A, Manifold::Point{
		shapeB.GetLocation(),
		ContactFeature{ContactFeature::e_face, 0, ContactFeature::e_vertex, 0}
	});
}

Manifold box2d::CollideShapes(const EdgeShape& shapeA, const Transformation& xfA,
							  const EdgeShape& shapeB, const Transformation& xfB)
{
	// Edge-to-edge collisions can result in these types of collision manifolds:
	//   Manifold::e_unset - for non-contacting events.
	//   Manifold::e_circles - for end to end contacts or for end to face contacts.
	//   Manifold::e_faceA - for face A to face contacts.
	//   Manifold::e_faceB - for face B to face contacts.
	
	const auto shapeA_v1 = shapeA.GetVertex1();
	const auto shapeA_v2 = shapeA.GetVertex2();
	const auto shapeA_edge = (shapeA_v2 - shapeA_v1);
	const auto shapeB_v1 = InverseTransform(Transform(shapeB.GetVertex1(), xfB), xfA);
	const auto shapeB_v2 = InverseTransform(Transform(shapeB.GetVertex2(), xfB), xfA);
	const auto shapeB_edge = (shapeB_v2 - shapeB_v1);

	const auto totalRadius = GetVertexRadius(shapeA) + GetVertexRadius(shapeB);

	// Is shape B vertex 1 left of shape A vertex 1?
	const auto a = Dot(shapeA_edge, shapeB_v1 - shapeA_v1);

	// Is shape B vertex 1 right of shape A vertex 2?
	const auto b = Dot(shapeA_edge, shapeA_v2 - shapeB_v1);
	
	// Is shape B vertex 2 left of shape A vertex 1?
	const auto c = Dot(shapeA_edge, shapeB_v2 - shapeA_v1);
	
	// Is shape B vertex 2 right of shape A vertex 2?
	const auto d = Dot(shapeA_edge, shapeA_v2 - shapeB_v2);

	if (a < 0 && c < 0)
	{
		// shape B vertex 1 and 2 are both left of shape A's vertex 1
		if (a > c)
		{
			// shape B vertex 1 is closest to shape A's vertex 1
			if (GetLengthSquared(shapeB_v1 - shapeA_v1) > Square(totalRadius))
			{
				// no contact
				return Manifold{};
			}
			// circle contact with shape A vertex 1
			return Manifold::GetForCircles(shapeA_v1, Manifold::Point{
				shapeB_v1,
				ContactFeature{ContactFeature::e_vertex, 0, ContactFeature::e_vertex, 0}
			});
		}
		else // a <= c
		{
			// shape B vertex 2 is closest to shape A's vertex 1
			if (GetLengthSquared(shapeB_v2 - shapeA_v1) > Square(totalRadius))
			{
				// no contact
				return Manifold{};
			}
			// circle contact with shape A vertex 1
			return Manifold::GetForCircles(shapeA_v1, Manifold::Point{
				shapeB_v2,
				ContactFeature{ContactFeature::e_vertex, 0, ContactFeature::e_vertex, 1}
			});
		}
	}
	if (b < 0 && d < 0)
	{
		// shape B vertex 1 and 2 are both right of shape A's vertex 2
		if (b > d)
		{
			// shape B vertex 1 is closest to shape A's vertex 2
			if (GetLengthSquared(shapeB_v1 - shapeA_v2) > Square(totalRadius))
			{
				// no contact
				return Manifold{};
			}
			// circle contact with shape A vertex 2
			return Manifold::GetForCircles(shapeA_v1, Manifold::Point{
				shapeB_v1,
				ContactFeature{ContactFeature::e_vertex, 0, ContactFeature::e_vertex, 0}
			});
		}
		else // b <= d
		{
			// shape B vertex 2 is closest to shape A's vertex 2			
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
		GetManifoldFaceB(shapeB, xfB, shapeA, xfA, static_cast<PolygonShape::vertex_count_t>(edgeSepB.index)):
		GetManifoldFaceA(shapeA, xfA, shapeB, xfB, static_cast<PolygonShape::vertex_count_t>(edgeSepA.index));
}
