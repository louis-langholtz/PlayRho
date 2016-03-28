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


// Compute contact points for edge versus circle.
// This accounts for edge connectivity.
void b2CollideEdgeAndCircle(b2Manifold* manifold,
							const b2EdgeShape* edgeA, const b2Transform& xfA,
							const b2CircleShape* circleB, const b2Transform& xfB)
{
	manifold->ClearPoints();
	
	// Compute circle in frame of edge
	const auto Q = b2MulT(xfA, b2Mul(xfB, circleB->GetPosition()));
	
	const auto A = edgeA->GetVertex1();
	const auto B = edgeA->GetVertex2();
	const auto e = B - A;
	
	// Barycentric coordinates
	const auto u = b2Dot(e, B - Q);
	const auto v = b2Dot(e, Q - A);
	
	const auto radius = edgeA->GetRadius() + circleB->GetRadius();
	
	b2ContactFeature cf;
	cf.indexB = 0;
	cf.typeB = b2ContactFeature::e_vertex;
	
	// Region A
	if (v <= 0.0f)
	{
		const auto P = A;
		const auto d = Q - P;
		const auto dd = b2Dot(d, d);
		if (dd > (radius * radius))
			return;
		
		// Is there an edge connected to A?
		if (edgeA->HasVertex0())
		{
			const auto A1 = edgeA->GetVertex0();
			const auto B1 = A;
			const auto e1 = B1 - A1;
			const auto u1 = b2Dot(e1, B1 - Q);
			
			// Is the circle in Region AB of the previous edge?
			if (u1 > 0.0f)
				return;
		}
		
		cf.indexA = 0;
		cf.typeA = b2ContactFeature::e_vertex;
		manifold->SetType(b2Manifold::e_circles);
		manifold->SetLocalNormal(b2Vec2_zero);
		manifold->SetLocalPoint(P);
		manifold->AddPoint(circleB->GetPosition(), cf);
		return;
	}
	
	// Region B
	if (u <= 0.0f)
	{
		const auto P = B;
		const auto d = Q - P;
		const auto dd = b2Dot(d, d);
		if (dd > (radius * radius))
			return;
		
		// Is there an edge connected to B?
		if (edgeA->HasVertex3())
		{
			const auto B2 = edgeA->GetVertex3();
			const auto A2 = B;
			const auto e2 = B2 - A2;
			const auto v2 = b2Dot(e2, Q - A2);
			
			// Is the circle in Region AB of the next edge?
			if (v2 > 0.0f)
				return;
		}
		
		cf.indexA = 1;
		cf.typeA = b2ContactFeature::e_vertex;
		manifold->SetType(b2Manifold::e_circles);
		manifold->SetLocalNormal(b2Vec2_zero);
		manifold->SetLocalPoint(P);
		manifold->AddPoint(circleB->GetPosition(), cf);

		return;
	}
	
	// Region AB
	const auto den = b2Dot(e, e);
	b2Assert(den > 0.0f);
	const auto P = (1.0f / den) * (u * A + v * B);
	const auto d = Q - P;
	const auto dd = b2Dot(d, d);
	if (dd > (radius * radius))
		return;
	
	auto n = b2Vec2(-e.y, e.x);
	if (b2Dot(n, Q - A) < 0.0f)
		n = b2Vec2(-n.x, -n.y);
	
	cf.indexA = 0;
	cf.typeA = b2ContactFeature::e_face;
	manifold->SetType(b2Manifold::e_faceA);
	manifold->SetLocalNormal(b2Normalize(n));
	manifold->SetLocalPoint(A);
	manifold->AddPoint(circleB->GetPosition(), cf);
}

// This structure is used to keep track of the best separating axis.
struct b2EPAxis
{
	enum Type
	{
		e_unknown,
		e_edgeA,
		e_edgeB
	};
	
	Type type;
	int32 index;
	float32 separation;
};

// This holds polygon B expressed in frame A.
class b2TempPolygon
{
public:
	using count_t = int32;
	
	count_t getCount() const noexcept { return count; }
	
	b2Vec2 getVertex(count_t index) const
	{
		b2Assert(index < b2_maxPolygonVertices);
		return vertices[index];
	}

	b2Vec2 getNormal(count_t index) const
	{
		b2Assert(index < b2_maxPolygonVertices);
		return normals[index];
	}

	void append(b2Vec2 vertex, b2Vec2 normal)
	{
		b2Assert(count < b2_maxPolygonVertices);
		vertices[count] = vertex;
		normals[count] = normal;
		++count;
	}

private:
	b2Vec2 vertices[b2_maxPolygonVertices];
	b2Vec2 normals[b2_maxPolygonVertices];
	count_t count = 0;
};

// Reference face used for clipping
struct b2ReferenceFace
{
	int32 i1, i2;
	
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
	void Collide(b2Manifold* manifold, const b2EdgeShape* edgeA, const b2Transform& xfA,
				 const b2PolygonShape* polygonB, const b2Transform& xfB);
	
private:
	enum VertexType
	{
		e_isolated,
		e_concave,
		e_convex
	};

	b2EPAxis ComputeEdgeSeparation() const;
	b2EPAxis ComputePolygonSeparation() const;

	b2TempPolygon m_polygonB;
	
	b2Transform m_xf;
	b2Vec2 m_centroidB;
	b2Vec2 m_v0, m_v1, m_v2, m_v3;
	b2Vec2 m_normal0, m_normal1, m_normal2;
	b2Vec2 m_normal;
	VertexType m_type1, m_type2;
	b2Vec2 m_lowerLimit, m_upperLimit;
	float32 m_radius;
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
void b2EPCollider::Collide(b2Manifold* manifold, const b2EdgeShape* edgeA, const b2Transform& xfA,
						   const b2PolygonShape* polygonB, const b2Transform& xfB)
{
	m_xf = b2MulT(xfA, xfB);
	
	m_centroidB = b2Mul(m_xf, polygonB->GetCentroid());
	
	m_v0 = edgeA->GetVertex0();
	m_v1 = edgeA->GetVertex1();
	m_v2 = edgeA->GetVertex2();
	m_v3 = edgeA->GetVertex3();
	
	const auto hasVertex0 = edgeA->HasVertex0();
	const auto hasVertex3 = edgeA->HasVertex3();
	
	const auto edge1 = b2Normalize(m_v2 - m_v1);
	m_normal1 = b2Vec2(edge1.y, -edge1.x);
	auto offset1 = b2Dot(m_normal1, m_centroidB - m_v1);
	auto offset0 = 0.0f;
	auto offset2 = 0.0f;
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
			m_front = (offset0 >= 0.0f) || (offset1 >= 0.0f) || (offset2 >= 0.0f);
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
			m_front = (offset0 >= 0.0f) || ((offset1 >= 0.0f) && (offset2 >= 0.0f));
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
			m_front = (offset2 >= 0.0f) || ((offset0 >= 0.0f) && (offset1 >= 0.0f));
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
			m_front = (offset0 >= 0.0f) && (offset1 >= 0.0f) && (offset2 >= 0.0f);
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
			m_front = (offset0 >= 0.0f) || (offset1 >= 0.0f);
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
			m_front = (offset0 >= 0.0f) && (offset1 >= 0.0f);
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
			m_front = (offset1 >= 0.0f) || (offset2 >= 0.0f);
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
			m_front = (offset1 >= 0.0f) && (offset2 >= 0.0f);
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
		m_front = offset1 >= 0.0f;
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
	
	// Get polygonB in frameA
	for (auto i = decltype(polygonB->GetVertexCount()){0}; i < polygonB->GetVertexCount(); ++i)
	{
		m_polygonB.append(b2Mul(m_xf, polygonB->GetVertex(i)), b2Mul(m_xf.q, polygonB->GetNormal(i)));
	}
	
	m_radius = 2.0f * b2_polygonRadius;
	
	manifold->ClearPoints();
	
	const auto edgeAxis = ComputeEdgeSeparation();
	
	// If no valid normal can be found than this edge should not collide.
	if (edgeAxis.type == b2EPAxis::e_unknown)
	{
		return;
	}
	
	if (edgeAxis.separation > m_radius)
	{
		return;
	}
	
	const auto polygonAxis = ComputePolygonSeparation();
	if ((polygonAxis.type != b2EPAxis::e_unknown) && (polygonAxis.separation > m_radius))
	{
		return;
	}
	
	// Use hysteresis for jitter reduction.
	const auto k_relativeTol = 0.98f;
	const auto k_absoluteTol = 0.001f;
	
	b2EPAxis primaryAxis;
	if (polygonAxis.type == b2EPAxis::e_unknown)
	{
		primaryAxis = edgeAxis;
	}
	else if (polygonAxis.separation > ((k_relativeTol * edgeAxis.separation) + k_absoluteTol))
	{
		primaryAxis = polygonAxis;
	}
	else
	{
		primaryAxis = edgeAxis;
	}
	
	std::array<b2ClipVertex,2> ie;
	b2ReferenceFace rf;
	if (primaryAxis.type == b2EPAxis::e_edgeA)
	{
		manifold->SetType(b2Manifold::e_faceA);
		
		// Search for the polygon normal that is most anti-parallel to the edge normal.
		auto bestIndex = decltype(m_polygonB.getCount()){0};
		auto bestValue = b2Dot(m_normal, m_polygonB.getNormal(0));
		for (auto i = decltype(m_polygonB.getCount()){1}; i < m_polygonB.getCount(); ++i)
		{
			const auto value = b2Dot(m_normal, m_polygonB.getNormal(i));
			if (value < bestValue)
			{
				bestValue = value;
				bestIndex = i;
			}
		}
		
		const auto i1 = bestIndex;
		const auto i2 = ((i1 + 1) < m_polygonB.getCount()) ? i1 + 1 : 0;
		
		ie[0].v = m_polygonB.getVertex(i1);
		ie[0].id.cf.indexA = 0;
		ie[0].id.cf.indexB = static_cast<uint8>(i1);
		ie[0].id.cf.typeA = b2ContactFeature::e_face;
		ie[0].id.cf.typeB = b2ContactFeature::e_vertex;
		
		ie[1].v = m_polygonB.getVertex(i2);
		ie[1].id.cf.indexA = 0;
		ie[1].id.cf.indexB = static_cast<uint8>(i2);
		ie[1].id.cf.typeA = b2ContactFeature::e_face;
		ie[1].id.cf.typeB = b2ContactFeature::e_vertex;
		
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
		ie[0].id.cf.indexA = 0;
		ie[0].id.cf.indexB = static_cast<uint8>(primaryAxis.index);
		ie[0].id.cf.typeA = b2ContactFeature::e_vertex;
		ie[0].id.cf.typeB = b2ContactFeature::e_face;
		
		ie[1].v = m_v2;
		ie[1].id.cf.indexA = 0;
		ie[1].id.cf.indexB = static_cast<uint8>(primaryAxis.index);		
		ie[1].id.cf.typeA = b2ContactFeature::e_vertex;
		ie[1].id.cf.typeB = b2ContactFeature::e_face;
		
		rf.i1 = primaryAxis.index;
		rf.i2 = ((rf.i1 + 1) < m_polygonB.getCount()) ? rf.i1 + 1 : 0;
		rf.v1 = m_polygonB.getVertex(rf.i1);
		rf.v2 = m_polygonB.getVertex(rf.i2);
		rf.normal = m_polygonB.getNormal(rf.i1);
	}
	
	rf.sideNormal1 = b2Vec2(rf.normal.y, -rf.normal.x);
	rf.sideNormal2 = -rf.sideNormal1;
	rf.sideOffset1 = b2Dot(rf.sideNormal1, rf.v1);
	rf.sideOffset2 = b2Dot(rf.sideNormal2, rf.v2);
	
	// Clip incident edge against extruded edge1 side edges.
	std::array<b2ClipVertex,b2_maxManifoldPoints> clipPoints1;
	std::array<b2ClipVertex,b2_maxManifoldPoints> clipPoints2;
	
	// Clip to box side 1
	if (b2ClipSegmentToLine(clipPoints1, ie, rf.sideNormal1, rf.sideOffset1, rf.i1) < b2_maxManifoldPoints)
	{
		return;
	}
	
	// Clip to negative box side 1
	if (b2ClipSegmentToLine(clipPoints2, clipPoints1, rf.sideNormal2, rf.sideOffset2, rf.i2) < b2_maxManifoldPoints)
	{
		return;
	}
	
	// Now clipPoints2 contains the clipped points.
	if (primaryAxis.type == b2EPAxis::e_edgeA)
	{
		manifold->SetLocalNormal(rf.normal);
		manifold->SetLocalPoint(rf.v1);
	}
	else
	{
		manifold->SetLocalNormal(polygonB->GetNormal(rf.i1));
		manifold->SetLocalPoint(polygonB->GetVertex(rf.i1));
	}
	
	for (auto i = decltype(b2_maxManifoldPoints){0}; i < b2_maxManifoldPoints; ++i)
	{
		const auto separation = b2Dot(rf.normal, clipPoints2[i].v - rf.v1);
		
		if (separation <= m_radius)
		{
			if (primaryAxis.type == b2EPAxis::e_edgeA)
			{
				manifold->AddPoint(b2MulT(m_xf, clipPoints2[i].v), clipPoints2[i].id);
			}
			else
			{
				b2ContactFeature cf;
				cf.typeA = clipPoints2[i].id.cf.typeB;
				cf.typeB = clipPoints2[i].id.cf.typeA;
				cf.indexA = clipPoints2[i].id.cf.indexB;
				cf.indexB = clipPoints2[i].id.cf.indexA;
				manifold->AddPoint(clipPoints2[i].v, cf);
			}
		}
	}
}

b2EPAxis b2EPCollider::ComputeEdgeSeparation() const
{
	b2EPAxis axis;
	axis.type = b2EPAxis::e_edgeA;
	axis.index = m_front ? 0 : 1;
	axis.separation = b2_maxFloat;
	
	for (auto i = decltype(m_polygonB.getCount()){0}; i < m_polygonB.getCount(); ++i)
	{
		const auto s = b2Dot(m_normal, m_polygonB.getVertex(i) - m_v1);
		if (axis.separation > s)
		{
			axis.separation = s;
		}
	}
	
	return axis;
}

b2EPAxis b2EPCollider::ComputePolygonSeparation() const
{
	b2EPAxis axis;
	axis.type = b2EPAxis::e_unknown;
	axis.index = -1;
	axis.separation = -b2_maxFloat;

	const auto perp = b2Vec2(-m_normal.y, m_normal.x);

	for (auto i = decltype(m_polygonB.getCount()){0}; i < m_polygonB.getCount(); ++i)
	{
		const auto n = -m_polygonB.getNormal(i);
		
		const auto s1 = b2Dot(n, m_polygonB.getVertex(i) - m_v1);
		const auto s2 = b2Dot(n, m_polygonB.getVertex(i) - m_v2);
		const auto s = b2Min(s1, s2);
		
		if (s > m_radius)
		{
			// No collision
			axis.type = b2EPAxis::e_edgeB;
			axis.index = i;
			axis.separation = s;
			return axis;
		}
		
		// Adjacency
		if (b2Dot(n, perp) >= 0.0f)
		{
			if (b2Dot(n - m_upperLimit, m_normal) < -b2_angularSlop)
			{
				continue;
			}
		}
		else
		{
			if (b2Dot(n - m_lowerLimit, m_normal) < -b2_angularSlop)
			{
				continue;
			}
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

void b2CollideEdgeAndPolygon(	b2Manifold* manifold,
							 const b2EdgeShape* edgeA, const b2Transform& xfA,
							 const b2PolygonShape* polygonB, const b2Transform& xfB)
{
	b2EPCollider collider;
	collider.Collide(manifold, edgeA, xfA, polygonB, xfB);
}
