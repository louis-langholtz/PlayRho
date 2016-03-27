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

#include <Box2D/Collision/b2Distance.h>
#include <Box2D/Collision/Shapes/b2CircleShape.h>
#include <Box2D/Collision/Shapes/b2EdgeShape.h>
#include <Box2D/Collision/Shapes/b2ChainShape.h>
#include <Box2D/Collision/Shapes/b2PolygonShape.h>

#if defined(DO_GJK_PROFILING)
// GJK using Voronoi regions (Christer Ericson) and Barycentric coordinates.
int32 b2_gjkCalls, b2_gjkIters, b2_gjkMaxIters;
#endif

void b2DistanceProxy::Set(const b2Shape* shape, int32 index)
{
	switch (shape->GetType())
	{
	case b2Shape::e_circle:
		{
			const auto circle = static_cast<const b2CircleShape*>(shape);
			m_buffer[0] = circle->GetPosition();
			m_vertices = m_buffer;
			m_count = 1;
			m_radius = circle->GetRadius();
		}
		break;

	case b2Shape::e_polygon:
		{
			const auto polygon = static_cast<const b2PolygonShape*>(shape);
			m_vertices = polygon->GetVertices();
			m_count = polygon->GetVertexCount();
			m_radius = polygon->GetRadius();
		}
		break;

	case b2Shape::e_chain:
		{
			const auto chain = static_cast<const b2ChainShape*>(shape);
			b2Assert(0 <= index && index < chain->GetVertexCount());

			m_buffer[0] = chain->GetVertex(index);
			if ((index + 1) < chain->GetVertexCount())
			{
				m_buffer[1] = chain->GetVertex(index + 1);
			}
			else
			{
				m_buffer[1] = chain->GetVertex(0);
			}

			m_vertices = m_buffer;
			m_count = 2;
			m_radius = chain->GetRadius();
		}
		break;

	case b2Shape::e_edge:
		{
			const auto edge = static_cast<const b2EdgeShape*>(shape);
			m_buffer[0] = edge->GetVertex1();
			m_buffer[1] = edge->GetVertex2();
			m_vertices = m_buffer;
			m_count = 2;
			m_radius = edge->GetRadius();
		}
		break;

	default:
		b2Assert(false);
	}
}


struct b2SimplexVertex
{
	b2Vec2 wA;		// support point in proxyA
	b2Vec2 wB;		// support point in proxyB
	b2Vec2 w;		// wB - wA
	float32 a;		// barycentric coordinate for closest point
	int32 indexA;	// wA index
	int32 indexB;	// wB index
};

class b2Simplex
{
public:
	using vertices_count_t = uint32;
	static constexpr vertices_count_t maxVertices = 3;

	vertices_count_t GetCount() const noexcept
	{
		return m_count;
	}

	const b2SimplexVertex* GetVertices() const noexcept
	{
		return m_vertices;
	}

	void AddVertex(const b2SimplexVertex& vertex) noexcept
	{
		b2Assert(m_count < maxVertices);
		m_vertices[m_count] = vertex;
		++m_count;
	}

	void ReadCache(	const b2SimplexCache& cache,
					const b2DistanceProxy& proxyA, const b2Transform& transformA,
					const b2DistanceProxy& proxyB, const b2Transform& transformB)
	{
		b2Assert(cache.count <= maxVertices);
		
		// Copy data from cache.
		m_count = cache.count;
		for (auto i = decltype(m_count){0}; i < m_count; ++i)
		{
			auto& v = m_vertices[i];
			v.indexA = cache.indexA[i];
			v.indexB = cache.indexB[i];
			v.wA = b2Mul(transformA, proxyA.GetVertex(v.indexA));
			v.wB = b2Mul(transformB, proxyB.GetVertex(v.indexB));
			v.w = v.wB - v.wA;
			v.a = 0.0f;
		}

		// Compute the new simplex metric, if it is substantially different than
		// old metric then flush the simplex.
		if (m_count > 1)
		{
			const auto metric1 = cache.metric;
			const auto metric2 = GetMetric();
			if ((metric2 < 0.5f * metric1) || (2.0f * metric1 < metric2) || (metric2 < b2_epsilon))
			{
				// Reset the simplex.
				m_count = 0;
			}
		}

		// If the cache is empty or invalid ...
		if (m_count == 0)
		{
			auto& v = m_vertices[0];
			v.indexA = 0;
			v.indexB = 0;
			v.wA = b2Mul(transformA, proxyA.GetVertex(0));
			v.wB = b2Mul(transformB, proxyB.GetVertex(0));
			v.w = v.wB - v.wA;
			v.a = 1.0f;
			m_count = 1;
		}
	}

	void WriteCache(b2SimplexCache& cache) const
	{
		cache.metric = GetMetric();
		cache.count = uint16(m_count);
		const auto vertices = m_vertices;
		for (auto i = decltype(m_count){0}; i < m_count; ++i)
		{
			cache.indexA[i] = uint8(vertices[i].indexA);
			cache.indexB[i] = uint8(vertices[i].indexB);
		}
	}

	b2Vec2 GetSearchDirection() const
	{
		switch (m_count)
		{
		case 1:
			return -m_vertices[0].w;

		case 2:
			{
				const auto e12 = m_vertices[1].w - m_vertices[0].w;
				const auto sgn = b2Cross(e12, -m_vertices[0].w);
				if (sgn > 0.0f)
				{
					// Origin is left of e12.
					return b2Cross(1.0f, e12);
				}
				else
				{
					// Origin is right of e12.
					return b2Cross(e12, 1.0f);
				}
			}

		default:
			b2Assert(false);
			return b2Vec2_zero;
		}
	}

	b2Vec2 GetClosestPoint() const
	{
		switch (m_count)
		{
		case 1:
			return m_vertices[0].w;

		case 2:
			return m_vertices[0].a * m_vertices[0].w + m_vertices[1].a * m_vertices[1].w;

		case 3:
			return b2Vec2_zero;

		default:
			b2Assert(false);
			return b2Vec2_zero;
		}
	}

	void GetWitnessPoints(b2Vec2* pA, b2Vec2* pB) const
	{
		switch (m_count)
		{
		case 1:
			*pA = m_vertices[0].wA;
			*pB = m_vertices[0].wB;
			break;

		case 2:
			*pA = m_vertices[0].a * m_vertices[0].wA + m_vertices[1].a * m_vertices[1].wA;
			*pB = m_vertices[0].a * m_vertices[0].wB + m_vertices[1].a * m_vertices[1].wB;
			break;

		case 3:
			*pA = m_vertices[0].a * m_vertices[0].wA + m_vertices[1].a * m_vertices[1].wA + m_vertices[2].a * m_vertices[2].wA;
			*pB = *pA;
			break;

		default:
			b2Assert(false);
			break;
		}
	}

	float32 GetMetric() const
	{
		switch (m_count)
		{
		case 1:
			return 0.0f;

		case 2:
			return b2Distance(m_vertices[0].w, m_vertices[1].w);

		case 3:
			return b2Cross(m_vertices[1].w - m_vertices[0].w, m_vertices[2].w - m_vertices[0].w);

		default:
			b2Assert(false);
			return 0.0f;
		}
	}

	void Solve2() noexcept;
	void Solve3() noexcept;

private:
	vertices_count_t m_count = 0;
	b2SimplexVertex m_vertices[maxVertices];
};


// Solve a line segment using barycentric coordinates.
//
// p = a1 * w1 + a2 * w2
// a1 + a2 = 1
//
// The vector from the origin to the closest point on the line is
// perpendicular to the line.
// e12 = w2 - w1
// dot(p, e) = 0
// a1 * dot(w1, e) + a2 * dot(w2, e) = 0
//
// 2-by-2 linear system
// [1      1     ][a1] = [1]
// [w1.e12 w2.e12][a2] = [0]
//
// Define
// d12_1 =  dot(w2, e12)
// d12_2 = -dot(w1, e12)
// d12 = d12_1 + d12_2
//
// Solution
// a1 = d12_1 / d12
// a2 = d12_2 / d12
void b2Simplex::Solve2() noexcept
{
	const auto w1 = m_vertices[0].w;
	const auto w2 = m_vertices[1].w;
	const auto e12 = w2 - w1;

	// w1 region
	const auto d12_2 = -b2Dot(w1, e12);
	if (d12_2 <= 0.0f)
	{
		// a2 <= 0, so we clamp it to 0
		m_vertices[0].a = 1.0f;
		m_count = 1;
		return;
	}

	// w2 region
	const auto d12_1 = b2Dot(w2, e12);
	if (d12_1 <= 0.0f)
	{
		// a1 <= 0, so we clamp it to 0
		m_vertices[1].a = 1.0f;
		m_vertices[0] = m_vertices[1];
		m_count = 1;
		return;
	}

	// Must be in e12 region.
	const auto inv_d12 = 1.0f / (d12_1 + d12_2);
	m_vertices[0].a = d12_1 * inv_d12;
	m_vertices[1].a = d12_2 * inv_d12;
	m_count = 2;
}

// Possible regions:
// - points[2]
// - edge points[0]-points[2]
// - edge points[1]-points[2]
// - inside the triangle
void b2Simplex::Solve3() noexcept
{
	const auto w1 = m_vertices[0].w;
	const auto w2 = m_vertices[1].w;
	const auto w3 = m_vertices[2].w;

	// Edge12
	// [1      1     ][a1] = [1]
	// [w1.e12 w2.e12][a2] = [0]
	// a3 = 0
	const auto e12 = w2 - w1;
	const auto w1e12 = b2Dot(w1, e12);
	const auto w2e12 = b2Dot(w2, e12);
	const auto d12_1 = w2e12;
	const auto d12_2 = -w1e12;

	// Edge13
	// [1      1     ][a1] = [1]
	// [w1.e13 w3.e13][a3] = [0]
	// a2 = 0
	const auto e13 = w3 - w1;
	const auto w1e13 = b2Dot(w1, e13);
	const auto w3e13 = b2Dot(w3, e13);
	const auto d13_1 = w3e13;
	const auto d13_2 = -w1e13;

	// Edge23
	// [1      1     ][a2] = [1]
	// [w2.e23 w3.e23][a3] = [0]
	// a1 = 0
	const auto e23 = w3 - w2;
	const auto w2e23 = b2Dot(w2, e23);
	const auto w3e23 = b2Dot(w3, e23);
	const auto d23_1 = w3e23;
	const auto d23_2 = -w2e23;
	
	// Triangle123
	const auto n123 = b2Cross(e12, e13);

	const auto d123_1 = n123 * b2Cross(w2, w3);
	const auto d123_2 = n123 * b2Cross(w3, w1);
	const auto d123_3 = n123 * b2Cross(w1, w2);

	// w1 region
	if ((d12_2 <= 0.0f) && (d13_2 <= 0.0f))
	{
		m_vertices[0].a = 1.0f;
		m_count = 1;
		return;
	}

	// e12
	if ((d12_1 > 0.0f) && (d12_2 > 0.0f) && (d123_3 <= 0.0f))
	{
		const auto inv_d12 = 1.0f / (d12_1 + d12_2);
		m_vertices[0].a = d12_1 * inv_d12;
		m_vertices[1].a = d12_2 * inv_d12;
		m_count = 2;
		return;
	}

	// e13
	if ((d13_1 > 0.0f) && (d13_2 > 0.0f) && (d123_2 <= 0.0f))
	{
		const auto inv_d13 = 1.0f / (d13_1 + d13_2);
		m_vertices[0].a = d13_1 * inv_d13;
		m_vertices[2].a = d13_2 * inv_d13;
		m_count = 2;
		m_vertices[1] = m_vertices[2];
		return;
	}

	// w2 region
	if ((d12_1 <= 0.0f) && (d23_2 <= 0.0f))
	{
		m_vertices[1].a = 1.0f;
		m_count = 1;
		m_vertices[0] = m_vertices[1];
		return;
	}

	// w3 region
	if ((d13_1 <= 0.0f) && (d23_1 <= 0.0f))
	{
		m_vertices[2].a = 1.0f;
		m_count = 1;
		m_vertices[0] = m_vertices[2];
		return;
	}

	// e23
	if ((d23_1 > 0.0f) && (d23_2 > 0.0f) && (d123_1 <= 0.0f))
	{
		const auto inv_d23 = 1.0f / (d23_1 + d23_2);
		m_vertices[1].a = d23_1 * inv_d23;
		m_vertices[2].a = d23_2 * inv_d23;
		m_count = 2;
		m_vertices[0] = m_vertices[2];
		return;
	}

	// Must be in triangle123
	const auto inv_d123 = 1.0f / (d123_1 + d123_2 + d123_3);
	m_vertices[0].a = d123_1 * inv_d123;
	m_vertices[1].a = d123_2 * inv_d123;
	m_vertices[2].a = d123_3 * inv_d123;
	m_count = 3;
}

void b2Distance(b2DistanceOutput* output,
				b2SimplexCache* cache,
				const b2DistanceInput* input)
{
#if defined(DO_GJK_PROFILING)
	++b2_gjkCalls;
#endif

	const auto& proxyA = input->proxyA;
	const auto& proxyB = input->proxyB;

	const auto transformA = input->transformA;
	const auto transformB = input->transformB;

	// Initialize the simplex.
	b2Simplex simplex;
	simplex.ReadCache(*cache, proxyA, transformA, proxyB, transformB);

	// Get simplex vertices as an array.
	const auto vertices = simplex.GetVertices();
	constexpr auto k_maxIters = int32{20};

	// These store the vertices of the last simplex so that we
	// can check for duplicates and prevent cycling.
	int32 saveA[3], saveB[3];

#if defined(DO_COMPUTE_CLOSEST_POINT)
	auto distanceSqr1 = b2_maxFloat;
#endif

	// Main iteration loop.
	auto iter = decltype(k_maxIters){0};
	while (iter < k_maxIters)
	{
		// Copy simplex so we can identify duplicates.
		const auto saveCount = simplex.GetCount();
		for (auto i = decltype(saveCount){0}; i < saveCount; ++i)
		{
			saveA[i] = vertices[i].indexA;
			saveB[i] = vertices[i].indexB;
		}

		switch (simplex.GetCount())
		{
		case 1:
			break;

		case 2:
			simplex.Solve2();
			break;

		case 3:
			simplex.Solve3();
			break;

		default:
			b2Assert(false);
		}

		// If we have max points (3), then the origin is in the corresponding triangle.
		if (simplex.GetCount() == b2Simplex::maxVertices)
		{
			break;
		}

#if defined(DO_COMPUTE_CLOSEST_POINT)
		// Compute closest point.
		const auto p = simplex.GetClosestPoint();
		const auto distanceSqr2 = p.LengthSquared();

		// Ensure progress
		if (distanceSqr2 >= distanceSqr1)
		{
			//break;
		}
		distanceSqr1 = distanceSqr2;
#endif
		// Get search direction.
		const auto d = simplex.GetSearchDirection();

		// Ensure the search direction is numerically fit.
		if (d.LengthSquared() < b2_epsilon * b2_epsilon)
		{
			// The origin is probably contained by a line segment
			// or triangle. Thus the shapes are overlapped.

			// We can't return zero here even though there may be overlap.
			// In case the simplex is a point, segment, or triangle it is difficult
			// to determine if the origin is contained in the CSO or very close to it.
			break;
		}

		// Compute a tentative new simplex vertex using support points.
		b2SimplexVertex vertex;
		//auto vertex = vertices + simplex.m_count;
		vertex.indexA = proxyA.GetSupport(b2MulT(transformA.q, -d));
		vertex.wA = b2Mul(transformA, proxyA.GetVertex(vertex.indexA));
		vertex.indexB = proxyB.GetSupport(b2MulT(transformB.q, d));
		vertex.wB = b2Mul(transformB, proxyB.GetVertex(vertex.indexB));
		vertex.w = vertex.wB - vertex.wA;

		// Iteration count is equated to the number of support point calls.
		++iter;
#if defined(DO_GJK_PROFILING)
		++b2_gjkIters;
#endif

		// Check for duplicate support points. This is the main termination criteria.
		auto duplicate = false;
		for (auto i = decltype(saveCount){0}; i < saveCount; ++i)
		{
			if (vertex.indexA == saveA[i] && vertex.indexB == saveB[i])
			{
				duplicate = true;
				break;
			}
		}

		// If we found a duplicate support point we must exit to avoid cycling.
		if (duplicate)
		{
			break;
		}

		// New vertex is ok and needed.
		simplex.AddVertex(vertex);
	}

#if defined(DO_GJK_PROFILING)
	b2_gjkMaxIters = b2Max(b2_gjkMaxIters, iter);
#endif

	// Prepare output.
	simplex.GetWitnessPoints(&output->pointA, &output->pointB);
	output->distance = b2Distance(output->pointA, output->pointB);
	output->iterations = iter;

	// Cache the simplex.
	simplex.WriteCache(*cache);

	// Apply radii if requested.
	if (input->useRadii)
	{
		const auto rA = proxyA.m_radius;
		const auto rB = proxyB.m_radius;

		if ((output->distance > (rA + rB)) && (output->distance > b2_epsilon))
		{
			// Shapes are still no overlapped.
			// Move the witness points to the outer surface.
			output->distance -= rA + rB;
			const auto normal = b2Normalize(output->pointB - output->pointA);
			output->pointA += rA * normal;
			output->pointB -= rB * normal;
		}
		else
		{
			// Shapes are overlapped when radii are considered.
			// Move the witness points to the middle.
			const auto p = 0.5f * (output->pointA + output->pointB);
			output->pointA = p;
			output->pointB = p;
			output->distance = 0.0f;
		}
	}
}
