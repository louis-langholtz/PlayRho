/*
 * Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

#include <Box2D/Collision/EdgeInfo.hpp>
#include <Box2D/Collision/Shapes/EdgeShape.hpp>

using namespace box2d;

EdgeInfo::EdgeInfo(const EdgeShape& edge, const Vec2 centroid) noexcept:
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
		const auto convex1 = Cross(edge0, m_edge1) >= RealNum{0};
		const auto offset0 = Dot(normal0, centroid - vertex0);
		
		const auto vertex3 = edge.GetVertex3();
		const auto edge2 = GetUnitVector(vertex3 - m_vertex2, UnitVec2::GetZero());
		const auto normal2 = GetFwdPerpendicular(edge2);
		const auto convex2 = Cross(m_edge1, edge2) > RealNum{0};
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
		const auto convex1 = Cross(edge0, m_edge1) >= RealNum{0};
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
		const auto convex2 = Cross(m_edge1, edge2) > RealNum{0};
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

