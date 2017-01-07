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

#include <Box2D/Collision/Shapes/PolygonShape.hpp>
#include <Box2D/Common/VertexSet.hpp>

using namespace box2d;

PolygonShape::PolygonShape(float_t hx, float_t hy) noexcept:
	Shape{e_polygon, PolygonShape::GetDefaultVertexRadius()}
{
	SetAsBox(hx, hy);
}

PolygonShape::PolygonShape(Span<const Vec2> points) noexcept:
	Shape{e_polygon, PolygonShape::GetDefaultVertexRadius()}
{
	Set(points);
}

void PolygonShape::SetAsBox(float_t hx, float_t hy) noexcept
{
	m_count = 4;
	m_centroid = Vec2_zero;

	// vertices must be counter-clockwise

	const auto btm_rgt = Vec2{+hx, -hy};
	const auto top_rgt = Vec2{ hx,  hy};
	const auto top_lft = Vec2{-hx, +hy};
	const auto btm_lft = Vec2{-hx, -hy};
	
	m_vertices[0] = btm_rgt;
	m_vertices[1] = top_rgt;
	m_vertices[2] = top_lft;
	m_vertices[3] = btm_lft;

	m_normals[0] = UnitVec2::GetRight();
	m_normals[1] = UnitVec2::GetTop();
	m_normals[2] = UnitVec2::GetLeft();
	m_normals[3] = UnitVec2::GetBottom();
}

void box2d::SetAsBox(PolygonShape& shape, float_t hx, float_t hy, const Vec2& center, Angle angle) noexcept
{
	shape.SetAsBox(hx, hy);
	shape.Transform(Transformation{center, UnitVec2{angle}});
}

void PolygonShape::Transform(box2d::Transformation xf) noexcept
{
	for (auto i = decltype(m_count){0}; i < m_count; ++i)
	{
		m_vertices[i] = box2d::Transform(m_vertices[i], xf);
		m_normals[i] = Rotate(m_normals[i], xf.q);
	}
	m_centroid = box2d::Transform(m_centroid, xf);
}

void PolygonShape::Set(Span<const Vec2> points) noexcept
{
	// Perform welding and copy vertices into local buffer.
	auto point_set = VertexSet<MaxPolygonVertices>(LinearSlop);
	{
		const auto clampedCount = static_cast<vertex_count_t>(Min(points.size(), size_t{MaxPolygonVertices}));
		for (auto i = decltype(clampedCount){0}; i < clampedCount; ++i)
		{
			point_set.add(points[i]);
		}
	}
	Set(point_set);
}

void PolygonShape::Set(const VertexSet<MaxPolygonVertices>& point_set) noexcept
{
	const auto n = static_cast<vertex_count_t>(point_set.size());

	// Create the convex hull using the Gift wrapping algorithm
	// http://en.wikipedia.org/wiki/Gift_wrapping_algorithm

	const auto index0 = static_cast<decltype(n)>(FindLowestRightMostVertex(point_set));

	vertex_count_t hull[MaxPolygonVertices];
	auto m = decltype(m_count){0};
	auto ih = index0;

	for (;;)
	{
		hull[m] = ih;

		auto ie = decltype(n){0};
		for (auto j = decltype(n){1}; j < n; ++j)
		{
			if (ie == ih)
			{
				ie = j;
				continue;
			}

			const auto r = point_set[ie] - point_set[hull[m]];
			const auto v = point_set[j] - point_set[hull[m]];
			const auto c = Cross(r, v);
			if ((c < 0) || ((c == 0) && (GetLengthSquared(v) > GetLengthSquared(r))))
			{
				ie = j;
			}
		}

		++m;
		ih = ie;

		if (ie == index0)
		{
			break;
		}
	}
	
	m_count = m;

	// Copy vertices.
	for (auto i = decltype(m){0}; i < m; ++i)
	{
		m_vertices[i] = point_set[hull[i]];
	}

	if (m > 1)
	{
		// Compute normals.
		for (auto i = decltype(m){0}; i < m; ++i)
		{
			m_normals[i] = GetUnitVector(GetFwdPerpendicular(GetEdge(*this, i)));
		}
	}

	// Compute the polygon centroid.
	switch (m)
	{
		case 0:
			m_centroid = GetInvalid<Vec2>();
			break;
		case 1:
			m_centroid = m_vertices[0];
			break;
		case 2:
			m_centroid = (m_vertices[0] + m_vertices[1]) / 2;
			break;
		default:
			m_centroid = ComputeCentroid(Span<const Vec2>(m_vertices, m));
			break;
	}
}

size_t box2d::FindLowestRightMostVertex(Span<const Vec2> vertices)
{
	assert(vertices.size() > 0);
	auto i0 = decltype(vertices.size()){0};
	auto max_x = vertices[0].x;
	for (auto i = decltype(vertices.size()){1}; i < vertices.size(); ++i)
	{
		const auto x = vertices[i].x;
		if ((max_x < x) || ((max_x == x) && (vertices[i].y < vertices[i0].y)))
		{
			max_x = x;
			i0 = i;
		}
	}
	return i0;
}

Vec2 box2d::GetEdge(const PolygonShape& shape, PolygonShape::vertex_count_t index)
{
	assert(shape.GetVertexCount() > 1);

	const auto i0 = index;
	const auto i1 = GetModuloNext(index, shape.GetVertexCount());
	return shape.GetVertex(i1) - shape.GetVertex(i0);
}

bool box2d::Validate(const PolygonShape& shape)
{
	const auto count = shape.GetVertexCount();
	for (auto i = decltype(count){0}; i < count; ++i)
	{
		const auto i1 = i;
		const auto i2 = GetModuloNext(i1, count);
		const auto p = shape.GetVertex(i1);
		const auto e = shape.GetVertex(i2) - p;
		
		for (auto j = decltype(count){0}; j < count; ++j)
		{
			if ((j == i1) || (j == i2))
			{
				continue;
			}
			
			const auto v = shape.GetVertex(j) - p;
			const auto c = Cross(e, v);
			if (c < 0)
			{
				return false;
			}
		}
	}
	
	return true;
}

child_count_t box2d::GetChildCount(const PolygonShape& shape)
{
	return 1;
}

bool box2d::TestPoint(const PolygonShape& shape, const Transformation& xf, const Vec2& p)
{
	const auto pLocal = InverseRotate(p - xf.p, xf.q);
	const auto vr = shape.GetVertexRadius();
	const auto count = shape.GetVertexCount();
	auto maxDot = -MaxFloat;
	auto maxIdx = PolygonShape::InvalidVertex;
	for (auto i = decltype(count){0}; i < count; ++i)
	{
		const auto dot = Dot(shape.GetNormal(i), pLocal - shape.GetVertex(i));
		if (dot > vr)
		{
			return false;
		}
		if (maxDot < dot)
		{
			maxDot = dot;
			maxIdx = i;
		}
	}

	const auto v0 = shape.GetVertex(maxIdx);
	const auto v1 = shape.GetVertex(GetModuloNext(maxIdx, count));
	const auto edge = v1 - v0;
	const auto d0 = Dot(edge, v0 - pLocal);
	if (d0 >= 0)
	{
		// point is nearest v0 and not within edge
		return GetLengthSquared(v0 - pLocal) <= Square(vr);
	}
	const auto d1 = Dot(edge, pLocal - v1);
	if (d1 >= 0)
	{
		// point is nearest v1 and not within edge
		return GetLengthSquared(pLocal - v1) <= Square(vr);
	}
	return true;
}
