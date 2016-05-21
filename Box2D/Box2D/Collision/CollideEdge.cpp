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

#include <Box2D/Collision/Collision.h>
#include <Box2D/Collision/Shapes/CircleShape.h>
#include <Box2D/Collision/Shapes/EdgeShape.h>
#include <Box2D/Collision/Shapes/PolygonShape.h>

#include <type_traits>

namespace box2d {

// Compute contact points for edge versus circle.
// This accounts for edge connectivity.
Manifold CollideShapes(const EdgeShape& shapeA, const Transform& xfA, const CircleShape& shapeB, const Transform& xfB)
{
	// Compute circle in frame of edge
	const auto Q = MulT(xfA, Mul(xfB, shapeB.GetPosition()));
	
	const auto A = shapeA.GetVertex1();
	const auto B = shapeA.GetVertex2();
	const auto e = B - A;
	
	// Barycentric coordinates
	const auto u = Dot(e, B - Q);
	const auto v = Dot(e, Q - A);
	
	const auto totalRadius = shapeA.GetRadius() + shapeB.GetRadius();

	// Region A
	if (v <= 0)
	{
		const auto P = A;
		const auto d = Q - P;
		if (d.LengthSquared() > Square(totalRadius))
		{
			return Manifold{};
		}
		
		// Is there an edge connected to A?
		if (shapeA.HasVertex0())
		{
			const auto A1 = shapeA.GetVertex0();
			const auto B1 = A;
			const auto e1 = B1 - A1;
			const auto u1 = Dot(e1, B1 - Q);
			
			// Is the circle in Region AB of the previous edge?
			if (u1 > 0)
			{
				return Manifold{};
			}
		}
		
		auto manifold = Manifold{Manifold::e_circles};
		manifold.SetLocalNormal(Vec2_zero);
		manifold.SetLocalPoint(P);
		manifold.AddPoint(shapeB.GetPosition(), ContactFeature{ContactFeature::e_vertex, 0, ContactFeature::e_vertex, 0});
		return manifold;
	}
	
	// Region B
	if (u <= 0)
	{
		const auto P = B;
		const auto d = Q - P;
		if (d.LengthSquared() > Square(totalRadius))
		{
			return Manifold{};
		}
		
		// Is there an edge connected to B?
		if (shapeA.HasVertex3())
		{
			const auto B2 = shapeA.GetVertex3();
			const auto A2 = B;
			const auto e2 = B2 - A2;
			const auto v2 = Dot(e2, Q - A2);
			
			// Is the circle in Region AB of the next edge?
			if (v2 > 0)
			{
				return Manifold{};
			}
		}
		
		auto manifold = Manifold{Manifold::e_circles};
		manifold.SetLocalNormal(Vec2_zero);
		manifold.SetLocalPoint(P);
		manifold.AddPoint(shapeB.GetPosition(), ContactFeature{ContactFeature::e_vertex, 1, ContactFeature::e_vertex, 0});
		return manifold;
	}
	
	// Region AB
	const auto den = e.LengthSquared();
	assert(den > 0);
	const auto P = (float_t(1) / den) * (u * A + v * B);
	const auto d = Q - P;

	if (d.LengthSquared() > Square(totalRadius))
	{
		return Manifold{};
	}
	
	const auto n = [=]() {
		const auto e_perp = GetReversePerpendicular(e);
		return (Dot(e_perp, Q - A) < 0)? -e_perp: e_perp;
	}();
	
	auto manifold = Manifold{Manifold::e_faceA};
	manifold.SetLocalNormal(Normalize(n));
	manifold.SetLocalPoint(A);
	manifold.AddPoint(shapeB.GetPosition(), ContactFeature{ContactFeature::e_face, 0, ContactFeature::e_vertex, 0});
	return manifold;
}

// This structure is used to keep track of the best separating axis.
struct EPAxis
{
	using index_t = PolygonShape::vertex_count_t;

	enum Type
	{
		e_unknown,
		e_edgeA,
		e_edgeB
	};
	
	static constexpr index_t InvalidIndex = static_cast<index_t>(-1);

	EPAxis() = default;

	constexpr EPAxis(Type t, index_t i, float_t s) noexcept: type(t), index(i), separation(s) {}

	Type type;
	index_t index;
	float_t separation;
};

// This holds polygon B expressed in frame A.
class TempPolygon
{
public:
	using size_type = std::remove_cv<decltype(MaxPolygonVertices)>::type;

	TempPolygon() = default;

	TempPolygon(const PolygonShape& shape, const Transform& xf);

	/// Gets count of appended elements (vertex-normal pairs).
	/// @return value between 0 and MaxPolygonVertices inclusive.
	/// @see MaxPolygonVertices.
	size_type GetCount() const noexcept { return count; }
	
	Vec2 GetVertex(size_type index) const
	{
		assert(index >= 0);
		assert(index < MaxPolygonVertices);
		assert(index < count);
		return vertices[index];
	}

	Vec2 GetNormal(size_type index) const
	{
		assert(index >= 0);
		assert(index < MaxPolygonVertices);
		assert(index < count);
		return normals[index];
	}

	void Append(const Vec2& vertex, const Vec2& normal)
	{
		assert(count < MaxPolygonVertices);
		vertices[count] = vertex;
		normals[count] = normal;
		++count;
	}

private:
	size_type count = 0;
	Vec2 vertices[MaxPolygonVertices];
	Vec2 normals[MaxPolygonVertices];
};

/// Gets a TempPolygon object from a given shape in terms of a given transform.
inline TempPolygon::TempPolygon(const PolygonShape& shape, const Transform& xf)
{
	const auto num_vertices = shape.GetVertexCount();
	for (auto i = decltype(num_vertices){0}; i < num_vertices; ++i)
	{
		Append(Mul(xf, shape.GetVertex(i)), Mul(xf.q, shape.GetNormal(i)));
	}	
}

// Reference face used for clipping
struct ReferenceFace
{
	using index_t = PolygonShape::vertex_count_t;

	index_t i1, i2;
	
	Vec2 v1, v2;
	
	Vec2 normal;
	
	Vec2 sideNormal1;
	float_t sideOffset1;
	
	Vec2 sideNormal2;
	float_t sideOffset2;
};

class EdgeInfo
{
public:
	EdgeInfo() = default;

	EdgeInfo(const EdgeShape& edge, const Vec2& centroid);

	Vec2 GetVertex1() const noexcept { return m_vertex1; }
	Vec2 GetVertex2() const noexcept { return m_vertex2; }
	Vec2 GetEdge1() const noexcept { return m_edge1; }
	Vec2 GetNormal1() const noexcept { return m_normal1; }

	bool IsFront() const noexcept { return m_front; }
	Vec2 GetNormal() const noexcept { return m_normal; }
	Vec2 GetLowerLimit() const noexcept { return m_lowerLimit; }
	Vec2 GetUpperLimit() const noexcept { return m_upperLimit; }

private:
	Vec2 m_vertex1;
	Vec2 m_vertex2;
	Vec2 m_edge1;
	Vec2 m_normal1;

	bool m_front;
	Vec2 m_normal;
	Vec2 m_lowerLimit, m_upperLimit;
};

inline EdgeInfo::EdgeInfo(const EdgeShape& edge, const Vec2& centroid):
	m_vertex1(edge.GetVertex1()), m_vertex2(edge.GetVertex2()),
	m_edge1(Normalize(m_vertex2 - m_vertex1)), m_normal1(m_edge1.y, -m_edge1.x)
{
	const auto hasVertex0 = edge.HasVertex0();
	const auto hasVertex3 = edge.HasVertex3();

	const auto offset1 = Dot(m_normal1, centroid - m_vertex1);
	
	// Determine front or back collision. Determine collision normal limits.
	if (hasVertex0 && hasVertex3)
	{
		const auto vertex0 = edge.GetVertex0();
		const auto edge0 = Normalize(m_vertex1 - vertex0);
		const auto normal0 = GetForwardPerpendicular(edge0);
		const auto convex1 = Cross(edge0, m_edge1) >= float_t{0};
		const auto offset0 = Dot(normal0, centroid - vertex0);

		const auto vertex3 = edge.GetVertex3();
		const auto edge2 = Normalize(vertex3 - m_vertex2);
		const auto normal2 = GetForwardPerpendicular(edge2);
		const auto convex2 = Cross(m_edge1, edge2) > float_t{0};
		const auto offset2 = Dot(normal2, centroid - m_vertex2);

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
		const auto edge0 = Normalize(m_vertex1 - vertex0);
		const auto normal0 = GetForwardPerpendicular(edge0);
		const auto convex1 = Cross(edge0, m_edge1) >= float_t{0};
		const auto offset0 = Dot(normal0, centroid - vertex0);

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
		const auto edge2 = Normalize(vertex3 - m_vertex2);
		const auto normal2 = GetForwardPerpendicular(edge2);
		const auto convex2 = Cross(m_edge1, edge2) > float_t{0};
		const auto offset2 = Dot(normal2, centroid - m_vertex2);

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

static inline TempPolygon::size_type GetIndexOfMinimum(const TempPolygon& localShapeB, const EdgeInfo& edgeInfo)
{
	TempPolygon::size_type bestIndex = TempPolygon::size_type{0};
	{
		auto minValue = Dot(edgeInfo.GetNormal(), localShapeB.GetNormal(0));
		const auto count = localShapeB.GetCount();
		for (auto i = decltype(count){1}; i < count; ++i)
		{
			const auto value = Dot(edgeInfo.GetNormal(), localShapeB.GetNormal(i));
			if (minValue > value)
			{
				minValue = value;
				bestIndex = i;
			}
		}
	}
	return bestIndex;
}

static constexpr float_t MaxEPSeparation = PolygonRadius * 2; ///< Maximum separation.

static inline EPAxis ComputeEdgeSeparation(const TempPolygon& shape, const EdgeInfo& edgeInfo)
{
	auto minValue = MaxFloat;
	{
		const auto count = shape.GetCount();
		for (auto i = decltype(count){0}; i < count; ++i)
		{
			const auto s = Dot(edgeInfo.GetNormal(), shape.GetVertex(i) - edgeInfo.GetVertex1());
			minValue = Min(minValue, s);
		}
	}
	return EPAxis{EPAxis::e_edgeA, edgeInfo.IsFront()? 0u: 1u, minValue};
}

static inline EPAxis ComputePolygonSeparation(const TempPolygon& shape, const EdgeInfo& edgeInfo)
{
	auto axis = EPAxis{EPAxis::e_unknown, EPAxis::InvalidIndex, -MaxFloat};
	
	const auto normal = edgeInfo.GetNormal();
	const auto perp = GetReversePerpendicular(normal);
	const auto count = shape.GetCount();
	for (auto i = decltype(count){0}; i < count; ++i)
	{
		const auto polygonNormal = -shape.GetNormal(i);
		const auto polygonVertex = shape.GetVertex(i);
		const auto s1 = Dot(polygonNormal, polygonVertex - edgeInfo.GetVertex1());
		const auto s2 = Dot(polygonNormal, polygonVertex - edgeInfo.GetVertex2());
		const auto s = Min(s1, s2);
		
		if (s > MaxEPSeparation) // No collision
		{
			return EPAxis{EPAxis::e_edgeB, i, s};
		}
		
		// Adjacency
		if (Dot(polygonNormal, perp) >= 0)
		{
			if (Dot(polygonNormal - edgeInfo.GetUpperLimit(), edgeInfo.GetNormal()) < -AngularSlop)
			{
				continue;
			}
		}
		else
		{
			if (Dot(polygonNormal - edgeInfo.GetLowerLimit(), edgeInfo.GetNormal()) < -AngularSlop)
			{
				continue;
			}
		}
		
		if (axis.separation < s)
		{
			axis = EPAxis{EPAxis::e_edgeB, i, s};
		}
	}
	
	return axis;
}

/// Edge and polygon collider.
/// This takes into account edge adjacency.
class EPCollider
{
public:
	EPCollider(const Transform& xf): m_xf(xf) {}
	
	Manifold Collide(const EdgeShape& shapeA, const PolygonShape& shapeB) const;
	
private:
	Manifold Collide(const EdgeInfo& shapeA, const PolygonShape& shapeB) const;

	const Transform m_xf;
};

Manifold EPCollider::Collide(const EdgeShape& shapeA, const PolygonShape& shapeB) const
{
	return Collide(EdgeInfo{shapeA, Mul(m_xf, shapeB.GetCentroid())}, shapeB);
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
Manifold EPCollider::Collide(const EdgeInfo& edgeInfo, const PolygonShape& shapeB) const
{
	const auto localShapeB = TempPolygon{shapeB, m_xf};
	
	const auto edgeAxis = ComputeEdgeSeparation(localShapeB, edgeInfo);
	
	// If no valid normal can be found then this edge should not collide.
	assert(edgeAxis.type != EPAxis::e_unknown);
	if ((edgeAxis.type == EPAxis::e_unknown) || (edgeAxis.separation > MaxEPSeparation))
	{
		return Manifold{};
	}
	
	const auto polygonAxis = ComputePolygonSeparation(localShapeB, edgeInfo);
	if ((polygonAxis.type != EPAxis::e_unknown) && (polygonAxis.separation > MaxEPSeparation))
	{
		return Manifold{};
	}
	
	// Use hysteresis for jitter reduction.
	constexpr auto k_relativeTol = float_t(0.98);
	constexpr auto k_absoluteTol = LinearSlop / 5; // 0.001
	
	// Now:
	//   (edgeAxis.separation <= MaxSeparation) AND
	//   (polygonAxis.type == EPAxis::e_unknown OR polygonAxis.separation <= MaxSeparation)
	const auto primaryAxis = (polygonAxis.type == EPAxis::e_unknown)?
		edgeAxis: (polygonAxis.separation > ((k_relativeTol * edgeAxis.separation) + k_absoluteTol))?
			polygonAxis: edgeAxis;
	
	auto manifoldType = Manifold::e_unset;
	ClipArray incidentEdge;
	ReferenceFace rf;
	if (primaryAxis.type == EPAxis::e_edgeA)
	{
		manifoldType = Manifold::e_faceA;
		
		// Search for the polygon normal that is most anti-parallel to the edge normal.
		const auto bestIndex = GetIndexOfMinimum(localShapeB, edgeInfo);
		
		const auto i1 = bestIndex;
		const auto i2 = ((i1 + 1) < localShapeB.GetCount()) ? i1 + 1 : 0;
		
		incidentEdge[0].v = localShapeB.GetVertex(i1);
		incidentEdge[0].cf = ContactFeature(ContactFeature::e_face, 0, ContactFeature::e_vertex, i1);
		incidentEdge[1].v = localShapeB.GetVertex(i2);
		incidentEdge[1].cf = ContactFeature(ContactFeature::e_face, 0, ContactFeature::e_vertex, i2);

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
		manifoldType = Manifold::e_faceB;
		
		incidentEdge[0].v = edgeInfo.GetVertex1();
		incidentEdge[0].cf = ContactFeature(ContactFeature::e_vertex, 0, ContactFeature::e_face, primaryAxis.index);
		incidentEdge[1].v = edgeInfo.GetVertex2();
		incidentEdge[1].cf = ContactFeature(ContactFeature::e_vertex, 0, ContactFeature::e_face, primaryAxis.index);
		
		rf.i1 = primaryAxis.index;
		rf.i2 = ((rf.i1 + 1) < localShapeB.GetCount()) ? rf.i1 + 1 : 0;
		rf.v1 = localShapeB.GetVertex(rf.i1);
		rf.v2 = localShapeB.GetVertex(rf.i2);
		rf.normal = localShapeB.GetNormal(rf.i1);
	}
	
	rf.sideNormal1 = GetForwardPerpendicular(rf.normal);
	rf.sideNormal2 = -rf.sideNormal1;
	rf.sideOffset1 = Dot(rf.sideNormal1, rf.v1);
	rf.sideOffset2 = Dot(rf.sideNormal2, rf.v2);
	
	// Clip incident edge against extruded edge1 side edges.
	
	// Clip to box side 1
	ClipArray clipPoints1;
	if (ClipSegmentToLine(clipPoints1, incidentEdge, rf.sideNormal1, rf.sideOffset1, rf.i1) < clipPoints1.size())
	{
		return Manifold{};
	}
	
	// Clip to negative box side 1
	ClipArray clipPoints2;
	if (ClipSegmentToLine(clipPoints2, clipPoints1, rf.sideNormal2, rf.sideOffset2, rf.i2) < clipPoints2.size())
	{
		return Manifold{};
	}
	
	// Now clipPoints2 contains the clipped points.
	
	auto manifold = Manifold{manifoldType};

	if (primaryAxis.type == EPAxis::e_edgeA)
	{
		manifold.SetLocalNormal(rf.normal);
		manifold.SetLocalPoint(rf.v1);
		for (auto i = decltype(clipPoints2.size()){0}; i < clipPoints2.size(); ++i)
		{
			const auto separation = Dot(rf.normal, clipPoints2[i].v - rf.v1);
			if (separation <= MaxEPSeparation)
			{
				manifold.AddPoint(MulT(m_xf, clipPoints2[i].v), clipPoints2[i].cf);
			}
		}
	}
	else
	{
		manifold.SetLocalNormal(shapeB.GetNormal(rf.i1));
		manifold.SetLocalPoint(shapeB.GetVertex(rf.i1));
		for (auto i = decltype(clipPoints2.size()){0}; i < clipPoints2.size(); ++i)
		{
			const auto separation = Dot(rf.normal, clipPoints2[i].v - rf.v1);
			if (separation <= MaxEPSeparation)
			{
				manifold.AddPoint(clipPoints2[i].v, Flip(clipPoints2[i].cf));
			}
		}
	}

	return manifold;
}

Manifold CollideShapes(const EdgeShape& shapeA, const Transform& xfA, const PolygonShape& shapeB, const Transform& xfB)
{
	const auto collider = EPCollider{MulT(xfA, xfB)};
	return collider.Collide(shapeA, shapeB);
}

} // namespace box2d