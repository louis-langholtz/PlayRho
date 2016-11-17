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

#include <Box2D/Collision/Shapes/PolygonShape.h>
#include <Box2D/Common/VertexSet.hpp>
#include <new>

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
	assert((points.size() >= 3) && (points.size() <= MaxPolygonVertices));
	if (points.size() < 3)
	{
		SetAsBox(float_t{1}, float_t{1});
		return;
	}
	
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

	assert(n >= 3);
	if (n < 3)
	{
		// Polygon is degenerate.
		SetAsBox(float_t{1}, float_t{1});
		return;
	}

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
			if ((c < 0) || ((c == 0) && (LengthSquared(v) > LengthSquared(r))))
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
	
	assert(m >= 3);
	if (m < 3)
	{
		// Polygon is degenerate.
		SetAsBox(float_t{1}, float_t{1});
		return;
	}

	m_count = m;

	// Copy vertices.
	for (auto i = decltype(m){0}; i < m; ++i)
	{
		m_vertices[i] = point_set[hull[i]];
	}

	// Compute normals.
	for (auto i = decltype(m){0}; i < m; ++i)
	{
		m_normals[i] = GetUnitVector(GetFwdPerpendicular(GetEdge(*this, i)));
	}

	// Compute the polygon centroid.
	m_centroid = ComputeCentroid(Span<const Vec2>(m_vertices, m));
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
	const auto i0 = index;
	const auto i1 = static_cast<PolygonShape::vertex_count_t>((index + 1) % shape.GetVertexCount());
	return shape.GetVertex(i1) - shape.GetVertex(i0);
}

bool box2d::Validate(const PolygonShape& shape)
{
	const auto count = shape.GetVertexCount();
	for (auto i = decltype(count){0}; i < count; ++i)
	{
		const auto i1 = i;
		const auto i2 = static_cast<decltype(count)>((i1 + 1) % count);
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
	const auto count = shape.GetVertexCount();
	for (auto i = decltype(count){0}; i < count; ++i)
	{
		const auto dot = Dot(shape.GetNormal(i), pLocal - shape.GetVertex(i));
		if (dot > float_t{0})
		{
			return false;
		}
	}

	return true;
}

RayCastOutput box2d::RayCast(const PolygonShape& shape, const RayCastInput& input,
							 const Transformation& xf, child_count_t childIndex)
{
	BOX2D_NOT_USED(childIndex);

	// Put the ray into the polygon's frame of reference.
	const auto p1 = InverseRotate(input.p1 - xf.p, xf.q);
	const auto p2 = InverseRotate(input.p2 - xf.p, xf.q);
	const auto d = p2 - p1;

	auto lower = float_t{0};
	auto upper = input.maxFraction;
	constexpr auto InvalidIndex = static_cast<PolygonShape::vertex_count_t>(-1);
	auto index = InvalidIndex;
	const auto count = shape.GetVertexCount();
	for (auto i = decltype(count){0}; i < count; ++i)
	{
		// p = p1 + a * d
		// dot(normal, p - v) = 0
		// dot(normal, p1 - v) + a * dot(normal, d) = 0
		const auto numerator = Dot(shape.GetNormal(i), shape.GetVertex(i) - p1);
		const auto denominator = Dot(shape.GetNormal(i), d);

		if (denominator == float_t{0})
		{	
			if (numerator < float_t{0})
			{
				return RayCastOutput{};
			}
		}
		else
		{
			// Note: we want this predicate without division:
			// lower < numerator / denominator, where denominator < 0
			// Since denominator < 0, we have to flip the inequality:
			// lower < numerator / denominator <==> denominator * lower > numerator.
			if (denominator < float_t{0} && numerator < lower * denominator)
			{
				// Increase lower.
				// The segment enters this half-space.
				lower = numerator / denominator;
				index = i;
			}
			else if (denominator > float_t{0} && numerator < upper * denominator)
			{
				// Decrease upper.
				// The segment exits this half-space.
				upper = numerator / denominator;
			}
		}

		if (upper < lower)
		{
			return RayCastOutput{};
		}
	}

	assert((float_t{0} <= lower) && (lower <= input.maxFraction));

	if (index != InvalidIndex)
	{
		return RayCastOutput{Rotate(shape.GetNormal(index), xf.q), lower};
	}

	return RayCastOutput{};
}

AABB box2d::ComputeAABB(const PolygonShape& shape, const Transformation& xf, child_count_t childIndex)
{
	BOX2D_NOT_USED(childIndex);
	
	assert(shape.GetVertexCount() > 0);

	auto lower = Transform(shape.GetVertex(0), xf);
	auto upper = lower;

	const auto count = shape.GetVertexCount();
	for (auto i = decltype(count){1}; i < count; ++i)
	{
		const auto v = Transform(shape.GetVertex(i), xf);
		lower = Min(lower, v);
		upper = Max(upper, v);
	}

	const auto r = Vec2{GetVertexRadius(shape), GetVertexRadius(shape)};
	return AABB{lower - r, upper + r};
}

MassData box2d::ComputeMass(const PolygonShape& shape, float_t density)
{
	assert(density >= 0);

	// Polygon mass, centroid, and inertia.
	// Let rho be the polygon density in mass per unit area.
	// Then:
	// mass = rho * int(dA)
	// centroid.x = (1/mass) * rho * int(x * dA)
	// centroid.y = (1/mass) * rho * int(y * dA)
	// I = rho * int((x*x + y*y) * dA)
	//
	// We can compute these integrals by summing all the integrals
	// for each triangle of the polygon. To evaluate the integral
	// for a single triangle, we make a change of variables to
	// the (u,v) coordinates of the triangle:
	// x = x0 + e1x * u + e2x * v
	// y = y0 + e1y * u + e2y * v
	// where 0 <= u && 0 <= v && u + v <= 1.
	//
	// We integrate u from [0,1-v] and then v from [0,1].
	// We also need to use the Jacobian of the transformation:
	// D = cross(e1, e2)
	//
	// Simplification: triangle centroid = (1/3) * (p1 + p2 + p3)
	//
	// The rest of the derivation is handled by computer algebra.

	const auto count = shape.GetVertexCount();
	assert(count >= 3);

	auto center = Vec2_zero;
	auto area = float_t{0};
	auto I = float_t{0};

	// s is the reference point for forming triangles.
	// It's location doesn't change the result (except for rounding error).
	// This code puts the reference point inside the polygon.
	const auto s = Average(shape.GetVertices());

	constexpr auto k_inv3 = float_t{1} / float_t{3};

	for (auto i = decltype(count){0}; i < count; ++i)
	{
		// Triangle vertices.
		const auto e1 = shape.GetVertex(i) - s;
		const auto e2 = shape.GetVertex((i + 1) % count) - s;

		const auto D = Cross(e1, e2);

		const auto triangleArea = D / 2;
		area += triangleArea;

		// Area weighted centroid
		center += triangleArea * k_inv3 * (e1 + e2);

		const auto intx2 = e1.x * e1.x + e2.x * e1.x + e2.x * e2.x;
		const auto inty2 = e1.y * e1.y + e2.y * e1.y + e2.y * e2.y;

		I += (D * k_inv3 / 4) * (intx2 + inty2);
	}

	// Total mass
	const auto mass = density * area;

	// Center of mass
	assert((area > 0) && !almost_zero(area));
	center *= float_t{1} / area;
	const auto massDataCenter = center + s;

	// Inertia tensor relative to the local origin (point s).
	// Shift to center of mass then to original body origin.
	const auto massDataI = (density * I) + (mass * (LengthSquared(massDataCenter) - LengthSquared(center)));
	
	return MassData{mass, massDataCenter, massDataI};
}
