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

#include <Box2D/Collision/Shapes/PolygonShape.hpp>
#include <Box2D/Common/VertexSet.hpp>

using namespace box2d;

PolygonShape::PolygonShape(RealNum hx, RealNum hy, const Conf& conf) noexcept:
	Shape{e_polygon, conf}
{
	SetAsBox(hx, hy);
}

PolygonShape::PolygonShape(Span<const Vec2> points, const Conf& conf) noexcept:
	Shape{e_polygon, conf}
{
	Set(points);
}

void PolygonShape::SetAsBox(RealNum hx, RealNum hy) noexcept
{
	m_centroid = Vec2_zero;

	// vertices must be counter-clockwise

	const auto btm_rgt = Vec2{+hx, -hy};
	const auto top_rgt = Vec2{ hx,  hy};
	const auto top_lft = Vec2{-hx, +hy};
	const auto btm_lft = Vec2{-hx, -hy};
	
	m_vertices.clear();
	m_vertices.emplace_back(btm_rgt);
	m_vertices.emplace_back(top_rgt);
	m_vertices.emplace_back(top_lft);
	m_vertices.emplace_back(btm_lft);

	m_normals.clear();
	m_normals.emplace_back(UnitVec2::GetRight());
	m_normals.emplace_back(UnitVec2::GetTop());
	m_normals.emplace_back(UnitVec2::GetLeft());
	m_normals.emplace_back(UnitVec2::GetBottom());
}

void box2d::SetAsBox(PolygonShape& shape, RealNum hx, RealNum hy, const Vec2 center, Angle angle) noexcept
{
	shape.SetAsBox(hx, hy);
	shape.Transform(Transformation{center, UnitVec2{angle}});
}

void PolygonShape::Transform(box2d::Transformation xf) noexcept
{
	for (auto i = decltype(GetVertexCount()){0}; i < GetVertexCount(); ++i)
	{
		m_vertices[i] = box2d::Transform(m_vertices[i], xf);
		m_normals[i] = Rotate(m_normals[i], xf.q);
	}
	m_centroid = box2d::Transform(m_centroid, xf);
}

void PolygonShape::Set(Span<const Vec2> points) noexcept
{
	// Perform welding and copy vertices into local buffer.
	auto point_set = VertexSet(DefaultLinearSlop);
	for (auto&& p: points)
	{
		point_set.add(p);
	}
	Set(point_set);
}

std::vector<Vec2> box2d::GetConvexHullAsVector(Span<const Vec2> vertices)
{
	std::vector<Vec2> result;
	
	// Create the convex hull using the Gift wrapping algorithm
	// http://en.wikipedia.org/wiki/Gift_wrapping_algorithm
	
	const auto index0 = FindLowestRightMostVertex(vertices);
	if (index0 != static_cast<decltype(index0)>(-1))
	{
		const auto size = vertices.size();
		auto hull = std::vector<decltype(vertices.size())>();

		auto ih = index0;
		for (;;)
		{
			hull.push_back(ih);
			
			auto ie = decltype(size){0};
			for (auto j = decltype(size){1}; j < size; ++j)
			{
				if (ie == ih)
				{
					ie = j;
					continue;
				}
				
				const auto r = vertices[ie] - vertices[ih];
				const auto v = vertices[j] - vertices[ih];
				const auto c = Cross(r, v);
				if ((c < 0) || ((c == 0) && (GetLengthSquared(v) > GetLengthSquared(r))))
				{
					ie = j;
				}
			}
			
			ih = ie;
			if (ie == index0)
			{
				break;
			}
		}
		
		const auto count = hull.size();
		for (auto i = decltype(count){0}; i < count; ++i)
		{
			result.emplace_back(vertices[hull[i]]);
		}
	}

	return result;
}

void PolygonShape::Set(const VertexSet& point_set) noexcept
{
#ifndef NDEBUG
	const auto point_set_size = point_set.size();
	assert(point_set_size > 0 && point_set_size < std::numeric_limits<vertex_count_t>::max());
#endif

	m_vertices = GetConvexHullAsVector(point_set);
	assert(m_vertices.size() > 0 && m_vertices.size() < std::numeric_limits<vertex_count_t>::max());
	
	const auto count = static_cast<vertex_count_t>(m_vertices.size());

	m_normals.clear();
	if (count > 1)
	{
		// Compute normals.
		for (auto i = decltype(count){0}; i < count; ++i)
		{
			m_normals.emplace_back(GetUnitVector(GetFwdPerpendicular(GetEdge(*this, i))));
		}
	}
	else if (count == 1)
	{
		m_normals.emplace_back(UnitVec2{});
	}

	// Compute the polygon centroid.
	switch (count)
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
			m_centroid = ComputeCentroid(GetVertices());
			break;
	}
}

size_t box2d::FindLowestRightMostVertex(Span<const Vec2> vertices)
{
	const auto size = vertices.size();
	if (size > 0)
	{
		auto i0 = decltype(size){0};
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
	return static_cast<size_t>(-1);
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

child_count_t box2d::GetChildCount(const PolygonShape&)
{
	return 1;
}

bool box2d::TestPoint(const PolygonShape& shape, const Transformation& xf, const Vec2 p)
{
	const auto pLocal = InverseRotate(p - xf.p, xf.q);
	const auto vr = shape.GetVertexRadius();
	const auto count = shape.GetVertexCount();
	
	if (count == 1)
	{
		const auto center = xf.p + Rotate(shape.GetVertex(0), xf.q);
		return GetLengthSquared(p - center) <= Square(vr);
	}

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
