/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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

#include <Box2D/Collision/Shapes/b2PolygonShape.h>
#include <new>

using namespace box2d;

b2Shape* b2PolygonShape::Clone(b2BlockAllocator* allocator) const
{
	void* mem = allocator->Allocate(sizeof(b2PolygonShape));
	return new (mem) b2PolygonShape(*this);
}

void b2PolygonShape::SetAsBox(float_t hx, float_t hy) noexcept
{
	m_count = 4;
	m_vertices[0] = b2Vec2(-hx, -hy);
	m_vertices[1] = b2Vec2( hx, -hy);
	m_vertices[2] = b2Vec2( hx,  hy);
	m_vertices[3] = b2Vec2(-hx,  hy);
	m_normals[0] = b2Vec2(float_t{0}, float_t{-1});
	m_normals[1] = b2Vec2(float_t{1}, float_t{0});
	m_normals[2] = b2Vec2(float_t{0}, float_t{1});
	m_normals[3] = b2Vec2(float_t{-1}, float_t{0});
	m_centroid = b2Vec2_zero;
}

void b2PolygonShape::SetAsBox(float_t hx, float_t hy, const b2Vec2& center, float_t angle)
{
	m_count = 4;
	m_vertices[0] = b2Vec2(-hx, -hy);
	m_vertices[1] = b2Vec2( hx, -hy);
	m_vertices[2] = b2Vec2( hx,  hy);
	m_vertices[3] = b2Vec2(-hx,  hy);
	m_normals[0] = b2Vec2(float_t{0}, -float_t(1));
	m_normals[1] = b2Vec2(float_t(1), float_t{0});
	m_normals[2] = b2Vec2(float_t{0}, float_t(1));
	m_normals[3] = b2Vec2(-float_t(1), float_t{0});
	m_centroid = center;

	const auto xf = b2Transform{center, b2Rot{angle}};

	// Transform vertices and normals.
	for (auto i = decltype(m_count){0}; i < m_count; ++i)
	{
		m_vertices[i] = b2Mul(xf, m_vertices[i]);
		m_normals[i] = b2Mul(xf.q, m_normals[i]);
	}
}

child_count_t b2PolygonShape::GetChildCount() const
{
	return 1;
}

static b2Vec2 ComputeCentroid(const b2Vec2 vs[], b2PolygonShape::vertex_count_t count)
{
	assert(count >= 3);

	auto c = b2Vec2_zero;
	auto area = float_t{0};

	// pRef is the reference point for forming triangles.
	// It's location doesn't change the result (except for rounding error).
	const auto pRef = b2Vec2_zero;
#if 0
	// This code would put the reference point inside the polygon.
	for (auto i = decltype(count){0}; i < count; ++i)
	{
		pRef += vs[i];
	}
	pRef *= float_t(1) / count;
#endif

	const auto inv3 = float_t(1) / float_t(3);

	for (auto i = decltype(count){0}; i < count; ++i)
	{
		// Triangle vertices.
		const auto p1 = pRef;
		const auto p2 = vs[i];
		const auto p3 = (i + 1 < count) ? vs[i+1] : vs[0];

		const auto e1 = p2 - p1;
		const auto e2 = p3 - p1;

		const auto D = b2Cross(e1, e2);

		const auto triangleArea = D / float_t(2);
		area += triangleArea;

		// Area weighted centroid
		c += triangleArea * inv3 * (p1 + p2 + p3);
	}

	// Centroid
	assert(area > Epsilon);
	c *= float_t(1) / area;
	return c;
}

void b2PolygonShape::Set(const b2Vec2 vertices[], vertex_count_t count)
{
	assert((count >= 3) && (count <= MaxPolygonVertices));
	if (count < 3)
	{
		SetAsBox(float_t(1), float_t(1));
		return;
	}
	
	auto n = b2Min(count, MaxPolygonVertices);

	// Perform welding and copy vertices into local buffer.
	b2Vec2 ps[MaxPolygonVertices];
	auto tempCount = decltype(n){0};
	for (auto i = decltype(n){0}; i < n; ++i)
	{
		const auto v = vertices[i];

		auto unique = true;
		for (auto j = decltype(tempCount){0}; j < tempCount; ++j)
		{
			if (b2DistanceSquared(v, ps[j]) < b2Square(LinearSlop / 2))
			{
				unique = false;
				break;
			}
		}

		if (unique)
		{
			ps[tempCount++] = v;
		}
	}

	n = tempCount;
	if (n < 3)
	{
		// Polygon is degenerate.
		assert(false);
		SetAsBox(float_t(1), float_t(1));
		return;
	}

	// Create the convex hull using the Gift wrapping algorithm
	// http://en.wikipedia.org/wiki/Gift_wrapping_algorithm

	// Find the right most point on the hull
	auto i0 = decltype(n){0};
	auto x0 = ps[0].x;
	for (auto i = decltype(n){1}; i < n; ++i)
	{
		const auto x = ps[i].x;
		if ((x > x0) || ((x == x0) && (ps[i].y < ps[i0].y)))
		{
			i0 = i;
			x0 = x;
		}
	}

	vertex_count_t hull[MaxPolygonVertices];
	auto m = decltype(m_count){0};
	auto ih = i0;

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

			const auto r = ps[ie] - ps[hull[m]];
			const auto v = ps[j] - ps[hull[m]];
			const auto c = b2Cross(r, v);
			if (c < float_t{0})
			{
				ie = j;
			}

			// Collinearity check
			if ((c == float_t{0}) && (v.LengthSquared() > r.LengthSquared()))
			{
				ie = j;
			}
		}

		++m;
		ih = ie;

		if (ie == i0)
		{
			break;
		}
	}
	
	if (m < 3)
	{
		// Polygon is degenerate.
		assert(false);
		SetAsBox(float_t(1), float_t(1));
		return;
	}

	m_count = m;

	// Copy vertices.
	for (auto i = decltype(m){0}; i < m; ++i)
	{
		m_vertices[i] = ps[hull[i]];
	}

	// Compute normals. Ensure the edges have non-zero length.
	for (auto i = decltype(m){0}; i < m; ++i)
	{
		const auto i1 = i;
		const auto i2 = ((i + 1) < m) ? i + 1 : 0;
		const auto edge = m_vertices[i2] - m_vertices[i1];
		assert(edge.LengthSquared() > b2Square(Epsilon));
		m_normals[i] = b2Normalize(b2Cross(edge, float_t(1)));
	}

	// Compute the polygon centroid.
	m_centroid = ComputeCentroid(m_vertices, m);
}

bool b2PolygonShape::TestPoint(const b2Transform& xf, const b2Vec2& p) const
{
	const auto pLocal = b2MulT(xf.q, p - xf.p);

	for (auto i = decltype(m_count){0}; i < m_count; ++i)
	{
		const auto dot = b2Dot(m_normals[i], pLocal - m_vertices[i]);
		if (dot > float_t{0})
		{
			return false;
		}
	}

	return true;
}

bool b2PolygonShape::RayCast(b2RayCastOutput* output, const b2RayCastInput& input,
								const b2Transform& xf, child_count_t childIndex) const
{
	BOX2D_NOT_USED(childIndex);

	// Put the ray into the polygon's frame of reference.
	const auto p1 = b2MulT(xf.q, input.p1 - xf.p);
	const auto p2 = b2MulT(xf.q, input.p2 - xf.p);
	const auto d = p2 - p1;

	auto lower = float_t{0};
	auto upper = input.maxFraction;
	constexpr auto InvalidIndex = static_cast<child_count_t>(-1);
	auto index = InvalidIndex;
	for (auto i = decltype(m_count){0}; i < m_count; ++i)
	{
		// p = p1 + a * d
		// dot(normal, p - v) = 0
		// dot(normal, p1 - v) + a * dot(normal, d) = 0
		const auto numerator = b2Dot(m_normals[i], m_vertices[i] - p1);
		const auto denominator = b2Dot(m_normals[i], d);

		if (denominator == float_t{0})
		{	
			if (numerator < float_t{0})
			{
				return false;
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

		// The use of epsilon here causes the assert on lower to trip
		// in some cases. Apparently the use of epsilon was to make edge
		// shapes work, but now those are handled separately.
		//if (upper < lower - Epsilon)
		if (upper < lower)
		{
			return false;
		}
	}

	assert((float_t{0} <= lower) && (lower <= input.maxFraction));

	if (index != InvalidIndex)
	{
		output->fraction = lower;
		output->normal = b2Mul(xf.q, m_normals[index]);
		return true;
	}

	return false;
}

b2AABB b2PolygonShape::ComputeAABB(const b2Transform& xf, child_count_t childIndex) const
{
	BOX2D_NOT_USED(childIndex);
	
	assert(m_count > 0);

	auto lower = b2Mul(xf, m_vertices[0]);
	auto upper = lower;

	for (auto i = decltype(m_count){1}; i < m_count; ++i)
	{
		const auto v = b2Mul(xf, m_vertices[i]);
		lower = b2Min(lower, v);
		upper = b2Max(upper, v);
	}

	const auto r = b2Vec2(GetRadius(), GetRadius());
	return b2AABB{lower - r, upper + r};
}

b2MassData b2PolygonShape::ComputeMass(float_t density) const
{
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

	assert(m_count >= 3);

	auto center = b2Vec2_zero;
	auto area = float_t{0};
	auto I = float_t{0};

	// s is the reference point for forming triangles.
	// It's location doesn't change the result (except for rounding error).
	auto s = b2Vec2_zero;

	// This code would put the reference point inside the polygon.
	for (auto i = decltype(m_count){0}; i < m_count; ++i)
	{
		s += m_vertices[i];
	}
	s *= float_t(1) / m_count;

	constexpr auto k_inv3 = float_t(1) / float_t(3);

	for (auto i = decltype(m_count){0}; i < m_count; ++i)
	{
		// Triangle vertices.
		const auto e1 = m_vertices[i] - s;
		const auto e2 = ((i + 1) < m_count) ? m_vertices[i+1] - s : m_vertices[0] - s;

		const auto D = b2Cross(e1, e2);

		const auto triangleArea = D / float_t(2);
		area += triangleArea;

		// Area weighted centroid
		center += triangleArea * k_inv3 * (e1 + e2);

		const auto ex1 = e1.x, ey1 = e1.y;
		const auto ex2 = e2.x, ey2 = e2.y;

		const auto intx2 = ex1*ex1 + ex2*ex1 + ex2*ex2;
		const auto inty2 = ey1*ey1 + ey2*ey1 + ey2*ey2;

		I += (D * k_inv3 / 4) * (intx2 + inty2);
	}

	// Total mass
	const auto mass = density * area;

	// Center of mass
	assert(area > Epsilon);
	center *= float_t(1) / area;
	const auto massDataCenter = center + s;

	// Inertia tensor relative to the local origin (point s).
	// Shift to center of mass then to original body origin.
	const auto massDataI = (density * I) + (mass * (massDataCenter.LengthSquared() - center.LengthSquared()));
	
	return b2MassData{mass, massDataCenter, massDataI};
}

bool b2PolygonShape::Validate() const
{
	for (auto i = decltype(m_count){0}; i < m_count; ++i)
	{
		const auto i1 = i;
		const auto i2 = (i < (m_count - 1)) ? i1 + 1 : 0;
		const auto p = m_vertices[i1];
		const auto e = m_vertices[i2] - p;

		for (auto j = decltype(m_count){0}; j < m_count; ++j)
		{
			if ((j == i1) || (j == i2))
			{
				continue;
			}

			const auto v = m_vertices[j] - p;
			const auto c = b2Cross(e, v);
			if (c < float_t{0})
			{
				return false;
			}
		}
	}

	return true;
}
