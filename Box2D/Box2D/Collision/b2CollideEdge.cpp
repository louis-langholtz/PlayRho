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
#include <Box2D/Collision/Shapes/b2CircleShape.h>
#include <Box2D/Collision/Shapes/b2EdgeShape.h>
#include <Box2D/Collision/Shapes/b2PolygonShape.h>

#include <type_traits>

// Compute contact points for edge versus circle.
// This accounts for edge connectivity.
b2Manifold b2CollideShapes(const b2EdgeShape& shapeA, const b2Transform& xfA, const b2CircleShape& shapeB, const b2Transform& xfB)
{
	// Compute circle in frame of edge
	const auto Q = b2MulT(xfA, b2Mul(xfB, shapeB.GetPosition()));
	
	const auto A = shapeA.GetVertex1();
	const auto B = shapeA.GetVertex2();
	const auto e = B - A;
	
	// Barycentric coordinates
	const auto u = b2Dot(e, B - Q);
	const auto v = b2Dot(e, Q - A);
	
	const auto totalRadius = shapeA.GetRadius() + shapeB.GetRadius();

	// Region A
	if (v <= 0)
	{
		const auto P = A;
		const auto d = Q - P;
		const auto dd = b2Dot(d, d);
		if (dd > b2Square(totalRadius))
			return b2Manifold{};
		
		// Is there an edge connected to A?
		if (shapeA.HasVertex0())
		{
			const auto A1 = shapeA.GetVertex0();
			const auto B1 = A;
			const auto e1 = B1 - A1;
			const auto u1 = b2Dot(e1, B1 - Q);
			
			// Is the circle in Region AB of the previous edge?
			if (u1 > 0)
				return b2Manifold{};
		}
		
		auto manifold = b2Manifold{b2Manifold::e_circles};
		manifold.SetLocalNormal(b2Vec2_zero);
		manifold.SetLocalPoint(P);
		manifold.AddPoint(shapeB.GetPosition(), b2ContactFeature{b2ContactFeature::e_vertex, 0, b2ContactFeature::e_vertex, 0});
		return manifold;
	}
	
	// Region B
	if (u <= 0)
	{
		const auto P = B;
		const auto d = Q - P;
		const auto dd = b2Dot(d, d);
		if (dd > b2Square(totalRadius))
			return b2Manifold{};
		
		// Is there an edge connected to B?
		if (shapeA.HasVertex3())
		{
			const auto B2 = shapeA.GetVertex3();
			const auto A2 = B;
			const auto e2 = B2 - A2;
			const auto v2 = b2Dot(e2, Q - A2);
			
			// Is the circle in Region AB of the next edge?
			if (v2 > 0)
				return b2Manifold{};
		}
		
		auto manifold = b2Manifold{b2Manifold::e_circles};
		manifold.SetLocalNormal(b2Vec2_zero);
		manifold.SetLocalPoint(P);
		manifold.AddPoint(shapeB.GetPosition(), b2ContactFeature{b2ContactFeature::e_vertex, 1, b2ContactFeature::e_vertex, 0});
		return manifold;
	}
	
	// Region AB
	const auto den = b2Dot(e, e);
	b2Assert(den > 0);
	const auto P = (b2Float(1) / den) * (u * A + v * B);
	const auto d = Q - P;
	const auto dd = b2Dot(d, d);
	if (dd > b2Square(totalRadius))
		return b2Manifold{};
	
	auto n = b2Vec2(-e.y, e.x);
	if (b2Dot(n, Q - A) < 0)
		n = b2Vec2(-n.x, -n.y);
	
	auto manifold = b2Manifold{b2Manifold::e_faceA};
	manifold.SetLocalNormal(b2Normalize(n));
	manifold.SetLocalPoint(A);
	manifold.AddPoint(shapeB.GetPosition(), b2ContactFeature{b2ContactFeature::e_face, 0, b2ContactFeature::e_vertex, 0});
	return manifold;
}

// This structure is used to keep track of the best separating axis.
struct b2EPAxis
{
	using index_t = b2PolygonShape::vertex_count_t;

	enum Type
	{
		e_unknown,
		e_edgeA,
		e_edgeB
	};
	
	static constexpr index_t InvalidIndex = static_cast<index_t>(-1);

	b2EPAxis() = default;

	constexpr b2EPAxis(Type t, index_t i, b2Float s) noexcept: type(t), index(i), separation(s) {}

	Type type;
	index_t index;
	b2Float separation;
};

// This holds polygon B expressed in frame A.
class b2TempPolygon
{
public:
	using size_type = std::remove_cv<decltype(b2_maxPolygonVertices)>::type;

	b2TempPolygon() = default;

	b2TempPolygon(const b2PolygonShape& shape, const b2Transform& xf);

	/// Gets count of appended elements (vertex-normal pairs).
	/// @return value between 0 and b2_maxPolygonVertices inclusive.
	/// @see b2_maxPolygonVertices.
	size_type GetCount() const noexcept { return count; }
	
	b2Vec2 GetVertex(size_type index) const
	{
		b2Assert(index >= 0);
		b2Assert(index < b2_maxPolygonVertices);
		b2Assert(index < count);
		return vertices[index];
	}

	b2Vec2 GetNormal(size_type index) const
	{
		b2Assert(index >= 0);
		b2Assert(index < b2_maxPolygonVertices);
		b2Assert(index < count);
		return normals[index];
	}

	void Append(const b2Vec2& vertex, const b2Vec2& normal)
	{
		b2Assert(count < b2_maxPolygonVertices);
		vertices[count] = vertex;
		normals[count] = normal;
		++count;
	}

private:
	size_type count = 0;
	b2Vec2 vertices[b2_maxPolygonVertices];
	b2Vec2 normals[b2_maxPolygonVertices];
};

/// Gets a b2TempPolygon object from a given shape in terms of a given transform.
inline b2TempPolygon::b2TempPolygon(const b2PolygonShape& shape, const b2Transform& xf)
{
	const auto num_vertices = shape.GetVertexCount();
	for (auto i = decltype(num_vertices){0}; i < num_vertices; ++i)
	{
		Append(b2Mul(xf, shape.GetVertex(i)), b2Mul(xf.q, shape.GetNormal(i)));
	}	
}

// Reference face used for clipping
struct b2ReferenceFace
{
	using index_t = b2PolygonShape::vertex_count_t;

	index_t i1, i2;
	
	b2Vec2 v1, v2;
	
	b2Vec2 normal;
	
	b2Vec2 sideNormal1;
	b2Float sideOffset1;
	
	b2Vec2 sideNormal2;
	b2Float sideOffset2;
};

class b2EdgeInfo
{
public:
	b2EdgeInfo() = default;

	b2EdgeInfo(const b2EdgeShape& edge, const b2Vec2& centroid);

	b2Vec2 GetVertex1() const noexcept { return m_vertex1; }
	b2Vec2 GetVertex2() const noexcept { return m_vertex2; }
	b2Vec2 GetEdge1() const noexcept { return m_edge1; }
	b2Vec2 GetNormal1() const noexcept { return m_normal1; }

	bool IsFront() const noexcept { return m_front; }
	b2Vec2 GetNormal() const noexcept { return m_normal; }
	b2Vec2 GetLowerLimit() const noexcept { return m_lowerLimit; }
	b2Vec2 GetUpperLimit() const noexcept { return m_upperLimit; }

private:
	b2Vec2 m_vertex1;
	b2Vec2 m_vertex2;
	b2Vec2 m_edge1;
	b2Vec2 m_normal1;

	bool m_front;
	b2Vec2 m_normal;
	b2Vec2 m_lowerLimit, m_upperLimit;
};

inline b2EdgeInfo::b2EdgeInfo(const b2EdgeShape& edge, const b2Vec2& centroid):
	m_vertex1(edge.GetVertex1()), m_vertex2(edge.GetVertex2()),
	m_edge1(b2Normalize(m_vertex2 - m_vertex1)), m_normal1(m_edge1.y, -m_edge1.x)
{
	const auto hasVertex0 = edge.HasVertex0();
	const auto hasVertex3 = edge.HasVertex3();

	const auto offset1 = b2Dot(m_normal1, centroid - m_vertex1);
	
	// Determine front or back collision. Determine collision normal limits.
	if (hasVertex0 && hasVertex3)
	{
		const auto vertex0 = edge.GetVertex0();
		const auto edge0 = b2Normalize(m_vertex1 - vertex0);
		const auto normal0 = b2Vec2(edge0.y, -edge0.x);
		const auto convex1 = b2Cross(edge0, m_edge1) >= b2Float{0};
		const auto offset0 = b2Dot(normal0, centroid - vertex0);

		const auto vertex3 = edge.GetVertex3();
		const auto edge2 = b2Normalize(vertex3 - m_vertex2);
		const auto normal2 = b2Vec2(edge2.y, -edge2.x);
		const auto convex2 = b2Cross(m_edge1, edge2) > b2Float{0};
		const auto offset2 = b2Dot(normal2, centroid - m_vertex2);

		if (convex1 && convex2)
		{
			m_front = (offset0 >= 0) || (offset1 >= 0) || (offset2 >= 0);
			if (m_front)
			{
				m_normal = m_normal1;
				m_lowerLimit = normal0;
				m_upperLimit = normal2;
			}
			else
			{
				m_normal = -m_normal1;
				m_lowerLimit = -m_normal1;
				m_upperLimit = -m_normal1;
			}
		}
		else if (convex1)
		{
			m_front = (offset0 >= 0) || ((offset1 >= 0) && (offset2 >= 0));
			if (m_front)
			{
				m_normal = m_normal1;
				m_lowerLimit = normal0;
				m_upperLimit = m_normal1;
			}
			else
			{
				m_normal = -m_normal1;
				m_lowerLimit = -normal2;
				m_upperLimit = -m_normal1;
			}
		}
		else if (convex2)
		{
			m_front = (offset2 >= 0) || ((offset0 >= 0) && (offset1 >= 0));
			if (m_front)
			{
				m_normal = m_normal1;
				m_lowerLimit = m_normal1;
				m_upperLimit = normal2;
			}
			else
			{
				m_normal = -m_normal1;
				m_lowerLimit = -m_normal1;
				m_upperLimit = -normal0;
			}
		}
		else // !convex1 && !convex2
		{
			m_front = (offset0 >= 0) && (offset1 >= 0) && (offset2 >= 0);
			if (m_front)
			{
				m_normal = m_normal1;
				m_lowerLimit = m_normal1;
				m_upperLimit = m_normal1;
			}
			else
			{
				m_normal = -m_normal1;
				m_lowerLimit = -normal2;
				m_upperLimit = -normal0;
			}
		}
	}
	else if (hasVertex0)
	{
		const auto vertex0 = edge.GetVertex0();
		const auto edge0 = b2Normalize(m_vertex1 - vertex0);
		const auto normal0 = b2Vec2(edge0.y, -edge0.x);
		const auto convex1 = b2Cross(edge0, m_edge1) >= b2Float{0};
		const auto offset0 = b2Dot(normal0, centroid - vertex0);

		if (convex1)
		{
			m_front = (offset0 >= 0) || (offset1 >= 0);
			if (m_front)
			{
				m_normal = m_normal1;
				m_lowerLimit = normal0;
				m_upperLimit = -m_normal1;
			}
			else
			{
				m_normal = -m_normal1;
				m_lowerLimit = m_normal1;
				m_upperLimit = -m_normal1;
			}
		}
		else // !convex1
		{
			m_front = (offset0 >= 0) && (offset1 >= 0);
			if (m_front)
			{
				m_normal = m_normal1;
				m_lowerLimit = m_normal1;
				m_upperLimit = -m_normal1;
			}
			else
			{
				m_normal = -m_normal1;
				m_lowerLimit = m_normal1;
				m_upperLimit = -normal0;
			}
		}
	}
	else if (hasVertex3)
	{
		const auto vertex3 = edge.GetVertex3();
		const auto edge2 = b2Normalize(vertex3 - m_vertex2);
		const auto normal2 = b2Vec2(edge2.y, -edge2.x);
		const auto convex2 = b2Cross(m_edge1, edge2) > b2Float{0};
		const auto offset2 = b2Dot(normal2, centroid - m_vertex2);

		if (convex2)
		{
			m_front = (offset1 >= 0) || (offset2 >= 0);
			if (m_front)
			{
				m_normal = m_normal1;
				m_lowerLimit = -m_normal1;
				m_upperLimit = normal2;
			}
			else
			{
				m_normal = -m_normal1;
				m_lowerLimit = -m_normal1;
				m_upperLimit = m_normal1;
			}
		}
		else // !convex2
		{
			m_front = (offset1 >= 0) && (offset2 >= 0);
			if (m_front)
			{
				m_normal = m_normal1;
				m_lowerLimit = -m_normal1;
				m_upperLimit = m_normal1;
			}
			else
			{
				m_normal = -m_normal1;
				m_lowerLimit = -normal2;
				m_upperLimit = m_normal1;
			}
		}		
	}
	else // !hasVertex0 && !hasVertex3
	{
		m_front = offset1 >= 0;
		if (m_front)
		{
			m_normal = m_normal1;
			m_lowerLimit = -m_normal1;
			m_upperLimit = -m_normal1;
		}
		else
		{
			m_normal = -m_normal1;
			m_lowerLimit = m_normal1;
			m_upperLimit = m_normal1;
		}
	}
}

static inline b2TempPolygon::size_type b2GetIndexOfMinimum(const b2TempPolygon& localShapeB, const b2EdgeInfo& edgeInfo)
{
	b2TempPolygon::size_type bestIndex = b2TempPolygon::size_type{0};
	
	auto minValue = b2Dot(edgeInfo.GetNormal(), localShapeB.GetNormal(0));
	const auto count = localShapeB.GetCount();
	for (auto i = decltype(count){1}; i < count; ++i)
	{
		const auto value = b2Dot(edgeInfo.GetNormal(), localShapeB.GetNormal(i));
		if (minValue > value)
		{
			minValue = value;
			bestIndex = i;
		}
	}
	
	return bestIndex;
}

static constexpr b2Float b2MaxEPSeparation = b2_polygonRadius * 2; ///< Maximum separation.

static inline b2EPAxis b2ComputeEdgeSeparation(const b2TempPolygon& shape, const b2EdgeInfo& edgeInfo)
{
	auto min_val = b2_maxFloat;
	const auto count = shape.GetCount();
	for (auto i = decltype(count){0}; i < count; ++i)
	{
		const auto s = b2Dot(edgeInfo.GetNormal(), shape.GetVertex(i) - edgeInfo.GetVertex1());
		if (min_val > s)
			min_val = s;
	}
	return b2EPAxis(b2EPAxis::e_edgeA, edgeInfo.IsFront() ? 0 : 1, min_val);
}

static inline b2EPAxis b2ComputePolygonSeparation(const b2TempPolygon& shape, const b2EdgeInfo& edgeInfo)
{
	auto axis = b2EPAxis{b2EPAxis::e_unknown, b2EPAxis::InvalidIndex, -b2_maxFloat};
	
	const auto normal = edgeInfo.GetNormal();
	const auto perp = b2Vec2(-normal.y, normal.x);
	const auto count = shape.GetCount();
	for (auto i = decltype(count){0}; i < count; ++i)
	{
		const auto polygonNormal = -shape.GetNormal(i);
		const auto polygonVertex = shape.GetVertex(i);
		const auto s1 = b2Dot(polygonNormal, polygonVertex - edgeInfo.GetVertex1());
		const auto s2 = b2Dot(polygonNormal, polygonVertex - edgeInfo.GetVertex2());
		const auto s = b2Min(s1, s2);
		
		if (s > b2MaxEPSeparation) // No collision
			return b2EPAxis(b2EPAxis::e_edgeB, i, s);
		
		// Adjacency
		if (b2Dot(polygonNormal, perp) >= 0)
		{
			if (b2Dot(polygonNormal - edgeInfo.GetUpperLimit(), edgeInfo.GetNormal()) < -b2_angularSlop)
				continue;
		}
		else
		{
			if (b2Dot(polygonNormal - edgeInfo.GetLowerLimit(), edgeInfo.GetNormal()) < -b2_angularSlop)
				continue;
		}
		
		if (axis.separation < s)
		{
			axis = b2EPAxis{b2EPAxis::e_edgeB, i, s};
		}
	}
	
	return axis;
}

/// Edge and polygon collider.
/// This takes into account edge adjacency.
class b2EPCollider
{
public:
	b2EPCollider(const b2Transform& xf): m_xf(xf) {}
	
	b2Manifold Collide(const b2EdgeShape& shapeA, const b2PolygonShape& shapeB) const;
	
private:
	b2Manifold Collide(const b2EdgeInfo& shapeA, const b2PolygonShape& shapeB) const;

	const b2Transform m_xf;
};

b2Manifold b2EPCollider::Collide(const b2EdgeShape& shapeA, const b2PolygonShape& shapeB) const
{
	return Collide(b2EdgeInfo{shapeA, b2Mul(m_xf, shapeB.GetCentroid())}, shapeB);
}

// Algorithm:
// 1. Classify v1 and v2
// 2. Classify polygon centroid as front or back
// 3. Flip normal if necessary
// 4. Initialize normal range to [-pi, pi] about face normal
// 5. Adjust normal range according to adjacent edges
// 6. Visit each separating axes, only accept axes within the range
// 7. Return if _any_ axis indicates separation
// 8. Clip
b2Manifold b2EPCollider::Collide(const b2EdgeInfo& edgeInfo, const b2PolygonShape& shapeB) const
{
	const auto localShapeB = b2TempPolygon{shapeB, m_xf};
	
	const auto edgeAxis = b2ComputeEdgeSeparation(localShapeB, edgeInfo);
	
	// If no valid normal can be found then this edge should not collide.
	b2Assert(edgeAxis.type != b2EPAxis::e_unknown);
	if ((edgeAxis.type == b2EPAxis::e_unknown) || (edgeAxis.separation > b2MaxEPSeparation))
		return b2Manifold{};
	
	const auto polygonAxis = b2ComputePolygonSeparation(localShapeB, edgeInfo);
	if ((polygonAxis.type != b2EPAxis::e_unknown) && (polygonAxis.separation > b2MaxEPSeparation))
		return b2Manifold{};
	
	// Use hysteresis for jitter reduction.
	constexpr auto k_relativeTol = b2Float(0.98);
	constexpr auto k_absoluteTol = b2_linearSlop / 5; // 0.001
	
	// Now:
	//   (edgeAxis.separation <= MaxSeparation) AND
	//   (polygonAxis.type == b2EPAxis::e_unknown OR polygonAxis.separation <= MaxSeparation)
	const auto primaryAxis = (polygonAxis.type == b2EPAxis::e_unknown)?
		edgeAxis: (polygonAxis.separation > ((k_relativeTol * edgeAxis.separation) + k_absoluteTol))?
			polygonAxis: edgeAxis;
	
	auto manifoldType = b2Manifold::e_unset;
	b2ClipArray incidentEdge;
	b2ReferenceFace rf;
	if (primaryAxis.type == b2EPAxis::e_edgeA)
	{
		manifoldType = b2Manifold::e_faceA;
		
		// Search for the polygon normal that is most anti-parallel to the edge normal.
		const auto bestIndex = b2GetIndexOfMinimum(localShapeB, edgeInfo);
		
		const auto i1 = bestIndex;
		const auto i2 = ((i1 + 1) < localShapeB.GetCount()) ? i1 + 1 : 0;
		
		incidentEdge[0].v = localShapeB.GetVertex(i1);
		incidentEdge[0].cf = b2ContactFeature(b2ContactFeature::e_face, 0, b2ContactFeature::e_vertex, i1);
		incidentEdge[1].v = localShapeB.GetVertex(i2);
		incidentEdge[1].cf = b2ContactFeature(b2ContactFeature::e_face, 0, b2ContactFeature::e_vertex, i2);

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
	}
	else
	{
		manifoldType = b2Manifold::e_faceB;
		
		incidentEdge[0].v = edgeInfo.GetVertex1();
		incidentEdge[0].cf = b2ContactFeature(b2ContactFeature::e_vertex, 0, b2ContactFeature::e_face, primaryAxis.index);
		incidentEdge[1].v = edgeInfo.GetVertex2();
		incidentEdge[1].cf = b2ContactFeature(b2ContactFeature::e_vertex, 0, b2ContactFeature::e_face, primaryAxis.index);
		
		rf.i1 = primaryAxis.index;
		rf.i2 = ((rf.i1 + 1) < localShapeB.GetCount()) ? rf.i1 + 1 : 0;
		rf.v1 = localShapeB.GetVertex(rf.i1);
		rf.v2 = localShapeB.GetVertex(rf.i2);
		rf.normal = localShapeB.GetNormal(rf.i1);
	}
	
	rf.sideNormal1 = b2Vec2(rf.normal.y, -rf.normal.x);
	rf.sideNormal2 = -rf.sideNormal1;
	rf.sideOffset1 = b2Dot(rf.sideNormal1, rf.v1);
	rf.sideOffset2 = b2Dot(rf.sideNormal2, rf.v2);
	
	// Clip incident edge against extruded edge1 side edges.
	
	// Clip to box side 1
	b2ClipArray clipPoints1;
	if (b2ClipSegmentToLine(clipPoints1, incidentEdge, rf.sideNormal1, rf.sideOffset1, rf.i1) < clipPoints1.size())
	{
		return b2Manifold{};
	}
	
	// Clip to negative box side 1
	b2ClipArray clipPoints2;
	if (b2ClipSegmentToLine(clipPoints2, clipPoints1, rf.sideNormal2, rf.sideOffset2, rf.i2) < clipPoints2.size())
	{
		return b2Manifold{};
	}
	
	// Now clipPoints2 contains the clipped points.
	
	auto manifold = b2Manifold{manifoldType};

	if (primaryAxis.type == b2EPAxis::e_edgeA)
	{
		manifold.SetLocalNormal(rf.normal);
		manifold.SetLocalPoint(rf.v1);
		for (auto i = decltype(b2_maxManifoldPoints){0}; i < b2_maxManifoldPoints; ++i)
		{
			const auto separation = b2Dot(rf.normal, clipPoints2[i].v - rf.v1);
			if (separation <= b2MaxEPSeparation)
			{
				manifold.AddPoint(b2MulT(m_xf, clipPoints2[i].v), clipPoints2[i].cf);
			}
		}
	}
	else
	{
		manifold.SetLocalNormal(shapeB.GetNormal(rf.i1));
		manifold.SetLocalPoint(shapeB.GetVertex(rf.i1));
		for (auto i = decltype(b2_maxManifoldPoints){0}; i < b2_maxManifoldPoints; ++i)
		{
			const auto separation = b2Dot(rf.normal, clipPoints2[i].v - rf.v1);
			if (separation <= b2MaxEPSeparation)
			{
				manifold.AddPoint(clipPoints2[i].v, b2Flip(clipPoints2[i].cf));
			}
		}
	}

	return manifold;
}

b2Manifold b2CollideShapes(const b2EdgeShape& shapeA, const b2Transform& xfA, const b2PolygonShape& shapeB, const b2Transform& xfB)
{
	const auto collider = b2EPCollider{b2MulT(xfA, xfB)};
	return collider.Collide(shapeA, shapeB);
}
