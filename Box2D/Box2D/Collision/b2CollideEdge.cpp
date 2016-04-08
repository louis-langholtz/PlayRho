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
void b2CollideShapes(b2Manifold* manifold,
					 const b2EdgeShape& shapeA, const b2Transform& xfA,
					 const b2CircleShape& shapeB, const b2Transform& xfB)
{
	manifold->SetType(b2Manifold::e_unset);
	
	// Compute circle in frame of edge
	const auto Q = b2MulT(xfA, b2Mul(xfB, shapeB.GetPosition()));
	
	const auto A = shapeA.GetVertex1();
	const auto B = shapeA.GetVertex2();
	const auto e = B - A;
	
	// Barycentric coordinates
	const auto u = b2Dot(e, B - Q);
	const auto v = b2Dot(e, Q - A);
	
	const auto totalRadius = shapeA.GetRadius() + shapeB.GetRadius();
	
	b2ContactFeature cf;
	cf.indexB = 0;
	cf.typeB = b2ContactFeature::e_vertex;
	
	// Region A
	if (v <= 0)
	{
		const auto P = A;
		const auto d = Q - P;
		const auto dd = b2Dot(d, d);
		if (dd > b2Square(totalRadius))
			return;
		
		// Is there an edge connected to A?
		if (shapeA.HasVertex0())
		{
			const auto A1 = shapeA.GetVertex0();
			const auto B1 = A;
			const auto e1 = B1 - A1;
			const auto u1 = b2Dot(e1, B1 - Q);
			
			// Is the circle in Region AB of the previous edge?
			if (u1 > 0)
				return;
		}
		
		cf.indexA = 0;
		cf.typeA = b2ContactFeature::e_vertex;
		manifold->SetType(b2Manifold::e_circles);
		manifold->SetLocalNormal(b2Vec2_zero);
		manifold->SetLocalPoint(P);
		manifold->AddPoint(shapeB.GetPosition(), cf);
		return;
	}
	
	// Region B
	if (u <= 0)
	{
		const auto P = B;
		const auto d = Q - P;
		const auto dd = b2Dot(d, d);
		if (dd > b2Square(totalRadius))
			return;
		
		// Is there an edge connected to B?
		if (shapeA.HasVertex3())
		{
			const auto B2 = shapeA.GetVertex3();
			const auto A2 = B;
			const auto e2 = B2 - A2;
			const auto v2 = b2Dot(e2, Q - A2);
			
			// Is the circle in Region AB of the next edge?
			if (v2 > 0)
				return;
		}
		
		cf.indexA = 1;
		cf.typeA = b2ContactFeature::e_vertex;
		manifold->SetType(b2Manifold::e_circles);
		manifold->SetLocalNormal(b2Vec2_zero);
		manifold->SetLocalPoint(P);
		manifold->AddPoint(shapeB.GetPosition(), cf);

		return;
	}
	
	// Region AB
	const auto den = b2Dot(e, e);
	b2Assert(den > 0);
	const auto P = (1.0f / den) * (u * A + v * B);
	const auto d = Q - P;
	const auto dd = b2Dot(d, d);
	if (dd > b2Square(totalRadius))
		return;
	
	auto n = b2Vec2(-e.y, e.x);
	if (b2Dot(n, Q - A) < 0)
		n = b2Vec2(-n.x, -n.y);
	
	cf.indexA = 0;
	cf.typeA = b2ContactFeature::e_face;
	manifold->SetType(b2Manifold::e_faceA);
	manifold->SetLocalNormal(b2Normalize(n));
	manifold->SetLocalPoint(A);
	manifold->AddPoint(shapeB.GetPosition(), cf);
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

	constexpr b2EPAxis(Type t, index_t i, float32 s) noexcept: type(t), index(i), separation(s) {}

	Type type;
	index_t index;
	float32 separation;
};

// This holds polygon B expressed in frame A.
class b2TempPolygon
{
public:
	using size_type = std::remove_cv<decltype(b2_maxPolygonVertices)>::type;
	
	size_type GetCount() const noexcept { return count; }
	
	b2Vec2 GetVertex(size_type index) const
	{
		b2Assert((index >= 0) && (index < b2_maxPolygonVertices));
		return vertices[index];
	}

	b2Vec2 GetNormal(size_type index) const
	{
		b2Assert((index >= 0) && (index < b2_maxPolygonVertices));
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

// Reference face used for clipping
struct b2ReferenceFace
{
	using index_t = b2PolygonShape::vertex_count_t;

	index_t i1, i2;
	
	b2Vec2 v1, v2;
	
	b2Vec2 normal;
	
	b2Vec2 sideNormal1;
	float32 sideOffset1;
	
	b2Vec2 sideNormal2;
	float32 sideOffset2;
};

// This class collides an edge and a polygon, taking into account edge adjacency.
class b2EPCollider
{
public:
	void Collide(b2Manifold* manifold,
				 const b2EdgeShape& shapeA, const b2Transform& xfA,
				 const b2PolygonShape& shapeB, const b2Transform& xfB);
	
private:
	enum VertexType
	{
		e_isolated,
		e_concave,
		e_convex
	};

	b2EPAxis ComputeEdgeSeparation() const;
	b2EPAxis ComputePolygonSeparation() const;

	static constexpr float32 MaxSeparation = b2_polygonRadius * 2;

	b2TempPolygon m_shapeB;
	
	b2Transform m_xf;
	b2Vec2 m_centroidB;
	b2Vec2 m_v0, m_v1, m_v2, m_v3;
	b2Vec2 m_normal0, m_normal1, m_normal2;
	b2Vec2 m_normal;
	VertexType m_type1, m_type2;
	b2Vec2 m_lowerLimit, m_upperLimit;
	bool m_front;
};

// Algorithm:
// 1. Classify v1 and v2
// 2. Classify polygon centroid as front or back
// 3. Flip normal if necessary
// 4. Initialize normal range to [-pi, pi] about face normal
// 5. Adjust normal range according to adjacent edges
// 6. Visit each separating axes, only accept axes within the range
// 7. Return if _any_ axis indicates separation
// 8. Clip
void b2EPCollider::Collide(b2Manifold* manifold,
						   const b2EdgeShape& shapeA, const b2Transform& xfA,
						   const b2PolygonShape& shapeB, const b2Transform& xfB)
{
	m_xf = b2MulT(xfA, xfB);
	
	m_centroidB = b2Mul(m_xf, shapeB.GetCentroid());
	
	m_v0 = shapeA.GetVertex0();
	m_v1 = shapeA.GetVertex1();
	m_v2 = shapeA.GetVertex2();
	m_v3 = shapeA.GetVertex3();
	
	const auto hasVertex0 = shapeA.HasVertex0();
	const auto hasVertex3 = shapeA.HasVertex3();
	
	const auto edge1 = b2Normalize(m_v2 - m_v1);
	m_normal1 = b2Vec2(edge1.y, -edge1.x);
	auto offset1 = b2Dot(m_normal1, m_centroidB - m_v1);
	auto offset0 = float32{0};
	auto offset2 = float32{0};
	auto convex1 = false;
	auto convex2 = false;
	
	// Is there a preceding edge?
	if (hasVertex0)
	{
		const auto edge0 = b2Normalize(m_v1 - m_v0);
		m_normal0 = b2Vec2(edge0.y, -edge0.x);
		convex1 = b2Cross(edge0, edge1) >= 0.0f;
		offset0 = b2Dot(m_normal0, m_centroidB - m_v0);
	}
	
	// Is there a following edge?
	if (hasVertex3)
	{
		const auto edge2 = b2Normalize(m_v3 - m_v2);
		m_normal2 = b2Vec2(edge2.y, -edge2.x);
		convex2 = b2Cross(edge1, edge2) > 0.0f;
		offset2 = b2Dot(m_normal2, m_centroidB - m_v2);
	}
	
	// Determine front or back collision. Determine collision normal limits.
	if (hasVertex0 && hasVertex3)
	{
		if (convex1 && convex2)
		{
			m_front = (offset0 >= 0) || (offset1 >= 0) || (offset2 >= 0);
			if (m_front)
			{
				m_normal = m_normal1;
				m_lowerLimit = m_normal0;
				m_upperLimit = m_normal2;
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
				m_lowerLimit = m_normal0;
				m_upperLimit = m_normal1;
			}
			else
			{
				m_normal = -m_normal1;
				m_lowerLimit = -m_normal2;
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
				m_upperLimit = m_normal2;
			}
			else
			{
				m_normal = -m_normal1;
				m_lowerLimit = -m_normal1;
				m_upperLimit = -m_normal0;
			}
		}
		else
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
				m_lowerLimit = -m_normal2;
				m_upperLimit = -m_normal0;
			}
		}
	}
	else if (hasVertex0)
	{
		if (convex1)
		{
			m_front = (offset0 >= 0) || (offset1 >= 0);
			if (m_front)
			{
				m_normal = m_normal1;
				m_lowerLimit = m_normal0;
				m_upperLimit = -m_normal1;
			}
			else
			{
				m_normal = -m_normal1;
				m_lowerLimit = m_normal1;
				m_upperLimit = -m_normal1;
			}
		}
		else
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
				m_upperLimit = -m_normal0;
			}
		}
	}
	else if (hasVertex3)
	{
		if (convex2)
		{
			m_front = (offset1 >= 0) || (offset2 >= 0);
			if (m_front)
			{
				m_normal = m_normal1;
				m_lowerLimit = -m_normal1;
				m_upperLimit = m_normal2;
			}
			else
			{
				m_normal = -m_normal1;
				m_lowerLimit = -m_normal1;
				m_upperLimit = m_normal1;
			}
		}
		else
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
				m_lowerLimit = -m_normal2;
				m_upperLimit = m_normal1;
			}
		}		
	}
	else
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
	
	// Get shapeB in frameA
	for (auto i = decltype(shapeB.GetVertexCount()){0}; i < shapeB.GetVertexCount(); ++i)
	{
		m_shapeB.Append(b2Mul(m_xf, shapeB.GetVertex(i)), b2Mul(m_xf.q, shapeB.GetNormal(i)));
	}

	manifold->SetType(b2Manifold::e_unset);
	
	const auto edgeAxis = ComputeEdgeSeparation();
	
	// If no valid normal can be found then this edge should not collide.
	b2Assert(edgeAxis.type != b2EPAxis::e_unknown);
	if (edgeAxis.type == b2EPAxis::e_unknown)
		return;
	
	if (edgeAxis.separation > MaxSeparation)
		return;
	
	const auto polygonAxis = ComputePolygonSeparation();
	if ((polygonAxis.type != b2EPAxis::e_unknown) && (polygonAxis.separation > MaxSeparation))
		return;
	
	// Use hysteresis for jitter reduction.
	constexpr auto k_relativeTol = 0.98f;
	constexpr auto k_absoluteTol = b2_linearSlop / 5; // 0.001f;
	
	const auto primaryAxis = (polygonAxis.type == b2EPAxis::e_unknown)?
		edgeAxis: (polygonAxis.separation > ((k_relativeTol * edgeAxis.separation) + k_absoluteTol))?
			polygonAxis: edgeAxis;
	
	b2ClipArray ie;
	b2ReferenceFace rf;
	if (primaryAxis.type == b2EPAxis::e_edgeA)
	{
		manifold->SetType(b2Manifold::e_faceA);
		
		// Search for the polygon normal that is most anti-parallel to the edge normal.
		auto bestIndex = decltype(m_shapeB.GetCount()){0};
		{
			auto minValue = b2Dot(m_normal, m_shapeB.GetNormal(0));
			const auto count = m_shapeB.GetCount();
			for (auto i = decltype(count){1}; i < count; ++i)
			{
				const auto value = b2Dot(m_normal, m_shapeB.GetNormal(i));
				if (minValue > value)
				{
					minValue = value;
					bestIndex = i;
				}
			}
		}
		
		const auto i1 = bestIndex;
		const auto i2 = ((i1 + 1) < m_shapeB.GetCount()) ? i1 + 1 : 0;
		
		ie[0].v = m_shapeB.GetVertex(i1);
		ie[0].cf.indexA = 0;
		ie[0].cf.indexB = i1;
		ie[0].cf.typeA = b2ContactFeature::e_face;
		ie[0].cf.typeB = b2ContactFeature::e_vertex;
		
		ie[1].v = m_shapeB.GetVertex(i2);
		ie[1].cf.indexA = 0;
		ie[1].cf.indexB = i2;
		ie[1].cf.typeA = b2ContactFeature::e_face;
		ie[1].cf.typeB = b2ContactFeature::e_vertex;
		
		if (m_front)
		{
			rf.i1 = 0;
			rf.i2 = 1;
			rf.v1 = m_v1;
			rf.v2 = m_v2;
			rf.normal = m_normal1;
		}
		else
		{
			rf.i1 = 1;
			rf.i2 = 0;
			rf.v1 = m_v2;
			rf.v2 = m_v1;
			rf.normal = -m_normal1;
		}		
	}
	else
	{
		manifold->SetType(b2Manifold::e_faceB);
		
		ie[0].v = m_v1;
		ie[0].cf.indexA = 0;
		ie[0].cf.indexB = primaryAxis.index;
		ie[0].cf.typeA = b2ContactFeature::e_vertex;
		ie[0].cf.typeB = b2ContactFeature::e_face;
		
		ie[1].v = m_v2;
		ie[1].cf.indexA = 0;
		ie[1].cf.indexB = primaryAxis.index;		
		ie[1].cf.typeA = b2ContactFeature::e_vertex;
		ie[1].cf.typeB = b2ContactFeature::e_face;
		
		rf.i1 = primaryAxis.index;
		rf.i2 = ((rf.i1 + 1) < m_shapeB.GetCount()) ? rf.i1 + 1 : 0;
		rf.v1 = m_shapeB.GetVertex(rf.i1);
		rf.v2 = m_shapeB.GetVertex(rf.i2);
		rf.normal = m_shapeB.GetNormal(rf.i1);
	}
	
	rf.sideNormal1 = b2Vec2(rf.normal.y, -rf.normal.x);
	rf.sideNormal2 = -rf.sideNormal1;
	rf.sideOffset1 = b2Dot(rf.sideNormal1, rf.v1);
	rf.sideOffset2 = b2Dot(rf.sideNormal2, rf.v2);
	
	if (primaryAxis.type == b2EPAxis::e_edgeA)
	{
		manifold->SetLocalNormal(rf.normal);
		manifold->SetLocalPoint(rf.v1);
	}
	else
	{
		manifold->SetLocalNormal(shapeB.GetNormal(rf.i1));
		manifold->SetLocalPoint(shapeB.GetVertex(rf.i1));
	}

	// Clip incident edge against extruded edge1 side edges.
	b2ClipArray clipPoints1;
	b2ClipArray clipPoints2;
	
	// Clip to box side 1
	if (b2ClipSegmentToLine(clipPoints1, ie, rf.sideNormal1, rf.sideOffset1, rf.i1) < b2_maxManifoldPoints)
		return;
	
	// Clip to negative box side 1
	if (b2ClipSegmentToLine(clipPoints2, clipPoints1, rf.sideNormal2, rf.sideOffset2, rf.i2) < b2_maxManifoldPoints)
		return;
	
	// Now clipPoints2 contains the clipped points.
	
	for (auto i = decltype(b2_maxManifoldPoints){0}; i < b2_maxManifoldPoints; ++i)
	{
		const auto separation = b2Dot(rf.normal, clipPoints2[i].v - rf.v1);
		if (separation <= MaxSeparation)
		{
			if (primaryAxis.type == b2EPAxis::e_edgeA)
				manifold->AddPoint(b2MulT(m_xf, clipPoints2[i].v), clipPoints2[i].cf);
			else
				manifold->AddPoint(clipPoints2[i].v, b2Flip(clipPoints2[i].cf));
		}
	}
}

b2EPAxis b2EPCollider::ComputeEdgeSeparation() const
{
	auto min_val = b2_maxFloat;
	const auto count = m_shapeB.GetCount();
	for (auto i = decltype(count){0}; i < count; ++i)
	{
		const auto s = b2Dot(m_normal, m_shapeB.GetVertex(i) - m_v1);
		if (min_val > s)
			min_val = s;
	}
	return b2EPAxis(b2EPAxis::e_edgeA, m_front ? 0 : 1, min_val);
}

b2EPAxis b2EPCollider::ComputePolygonSeparation() const
{
	b2EPAxis axis;
	axis.type = b2EPAxis::e_unknown;
	axis.index = b2EPAxis::InvalidIndex;
	axis.separation = -b2_maxFloat;

	const auto perp = b2Vec2(-m_normal.y, m_normal.x);
	for (auto i = decltype(m_shapeB.GetCount()){0}; i < m_shapeB.GetCount(); ++i)
	{
		const auto n = -m_shapeB.GetNormal(i);
		
		const auto vertexB = m_shapeB.GetVertex(i);
		const auto s1 = b2Dot(n, vertexB - m_v1);
		const auto s2 = b2Dot(n, vertexB - m_v2);
		const auto s = b2Min(s1, s2);
		
		if (s > MaxSeparation) // No collision
			return b2EPAxis(b2EPAxis::e_edgeB, i, s);
		
		// Adjacency
		if (b2Dot(n, perp) >= 0)
		{
			if (b2Dot(n - m_upperLimit, m_normal) < -b2_angularSlop)
				continue;
		}
		else
		{
			if (b2Dot(n - m_lowerLimit, m_normal) < -b2_angularSlop)
				continue;
		}
		
		if (axis.separation < s)
		{
			axis.type = b2EPAxis::e_edgeB;
			axis.index = i;
			axis.separation = s;
		}
	}
	
	return axis;
}

void b2CollideShapes(b2Manifold* manifold,
					 const b2EdgeShape& shapeA, const b2Transform& xfA,
					 const b2PolygonShape& shapeB, const b2Transform& xfB)
{
	b2EPCollider collider;
	collider.Collide(manifold, shapeA, xfA, shapeB, xfB);
}
