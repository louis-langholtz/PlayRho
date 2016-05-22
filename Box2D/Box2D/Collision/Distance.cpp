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

#include <Box2D/Collision/Distance.h>
#include <Box2D/Collision/Shapes/CircleShape.h>
#include <Box2D/Collision/Shapes/EdgeShape.h>
#include <Box2D/Collision/Shapes/ChainShape.h>
#include <Box2D/Collision/Shapes/PolygonShape.h>

namespace box2d {

#if defined(DO_GJK_PROFILING)
// GJK using Voronoi regions (Christer Ericson) and Barycentric coordinates.
uint32 gjkCalls, gjkIters, gjkMaxIters;
#endif

DistanceProxy::DistanceProxy(const Shape& shape, child_count_t index)
{
	switch (shape.GetType())
	{
		case Shape::e_circle:
		{
			const auto& circle = *static_cast<const CircleShape*>(&shape);
			m_buffer[0] = circle.GetPosition();
			m_vertices = m_buffer;
			m_count = 1;
			m_radius = circle.GetRadius();
		}
			break;
			
		case Shape::e_polygon:
		{
			const auto& polygon = *static_cast<const PolygonShape*>(&shape);
			m_vertices = polygon.GetVertices();
			m_count = polygon.GetVertexCount();
			m_radius = polygon.GetRadius();
		}
			break;
			
		case Shape::e_chain:
		{
			const auto& chain = *static_cast<const ChainShape*>(&shape);
			m_buffer[0] = chain.GetVertex(index);
			m_buffer[1] = chain.GetVertex(chain.GetNextIndex(index));
			m_vertices = m_buffer;
			m_count = 2;
			m_radius = chain.GetRadius();
		}
			break;
			
		case Shape::e_edge:
		{
			const auto& edge = *static_cast<const EdgeShape*>(&shape);
			m_buffer[0] = edge.GetVertex1();
			m_buffer[1] = edge.GetVertex2();
			m_vertices = m_buffer;
			m_count = 2;
			m_radius = edge.GetRadius();
		}
			break;
			
		default:
			assert(false);
			break;
	}
}

class SimplexVertex
{
public:
	using size_type = DistanceProxy::size_type;

	SimplexVertex() = default;

	constexpr SimplexVertex(const SimplexVertex& copy) noexcept = default;

	constexpr SimplexVertex(Vec2 sA, size_type iA, Vec2 sB, size_type iB, float_t a_) noexcept:
		wA(sA), wB(sB), indexA(iA), indexB(iB), w(sB - sA), a(a_)
	{
		assert(a_ >= 0 && a_ <= 1);
	}

	constexpr SimplexVertex(const SimplexVertex& copy, float_t newA) noexcept:
		wA(copy.wA), wB(copy.wB), indexA(copy.indexA), indexB(copy.indexB), w(copy.w), a(newA)
	{
		assert(newA >= 0 && newA <= 1);
	}

	Vec2 get_wA() const noexcept { return wA; }
	
	Vec2 get_wB() const noexcept { return wB; }

	/// Gets "w".
	/// @return 2D vector value of wB minus wA.
	Vec2 get_w() const noexcept { return w; }

	/// Gets "A".
	/// @detail This is the "Barycentric coordinate for closest point".
	/// @return Scalar value between 0 and 1 inclusive.
	float_t get_a() const noexcept { return a; }

	/// Sets "A" to the given value.
	/// @note The given value must be between 0 and 1 inclusively. Behavior is undefined otherwise.
	/// @param value Value between 0 and 1 to set "A" to.
	void set_a(float_t value) noexcept
	{
		assert(value >= 0 && value <= 1);
		a = value;
	}

	size_type indexA; ///< wA index
	size_type indexB; ///< wB index
	
private:
	Vec2 wA; ///< Support point in proxy A.
	Vec2 wB; ///< Support point in proxy B.
	Vec2 w; ///< wB - wA. @see wA. @see wB.
	float_t a; ///< Barycentric coordinate for closest point
};

class Simplex
{
public:
	/// Maximum number of supportable vertices.
	static constexpr auto MaxVertices = unsigned{3};

	using size_type = std::remove_cv<decltype(MaxVertices)>::type;

	Simplex() = default;
	
	Simplex(const SimplexVertex& sv1) noexcept:
		m_count{1}, m_vertices{sv1} {}
	Simplex(const SimplexVertex& sv1, const SimplexVertex& sv2) noexcept:
		m_count{2}, m_vertices{sv1, sv2} {}
	Simplex(const SimplexVertex& sv1, const SimplexVertex& sv2, const SimplexVertex& sv3) noexcept:
		m_count{3}, m_vertices{sv1, sv2, sv3} {}

	/// Gets count of valid vertices.
 	/// @return Value between 0 and MaxVertices.
	/// @see MaxVertices
	size_type GetCount() const noexcept
	{
		return m_count;
	}

	const SimplexVertex* GetVertices() const noexcept
	{
		return m_vertices;
	}

	void AddVertex(const SimplexVertex& vertex) noexcept
	{
		assert(m_count < MaxVertices);
		m_vertices[m_count] = vertex;
		++m_count;
	}

	void ReadCache(const SimplexCache& cache,
				   const DistanceProxy& proxyA, const Transform& transformA,
				   const DistanceProxy& proxyB, const Transform& transformB)
	{
		assert(cache.GetCount() <= MaxVertices);
		
		// Copy data from cache.
		const auto count = cache.GetCount();
		for (auto i = decltype(count){0}; i < count; ++i)
		{
			const auto indexA = cache.GetIndexA(i);
			const auto indexB = cache.GetIndexB(i);
			const auto wA = Mul(transformA, proxyA.GetVertex(indexA));
			const auto wB = Mul(transformB, proxyB.GetVertex(indexB));
			m_vertices[i] = SimplexVertex{wA, indexA, wB, indexB, float_t{0}};
		}
		m_count = count;

		// Compute the new simplex metric, if it is substantially different than
		// old metric then flush the simplex.
		if (m_count > 1)
		{
			const auto metric1 = cache.GetMetric();
			const auto metric2 = GetMetric();
			if ((metric2 < (metric1 / float_t{2})) || (metric2 > (metric1 * float_t{2})) || (metric2 < Epsilon))
			{
				// Reset the simplex.
				m_count = 0;
			}
		}

		// If the cache is empty or invalid ...
		if (m_count == 0)
		{
			const auto indexA = SimplexCache::index_t{0};
			const auto indexB = SimplexCache::index_t{0};
			const auto wA = Mul(transformA, proxyA.GetVertex(indexA));
			const auto wB = Mul(transformB, proxyB.GetVertex(indexB));
			m_vertices[0] = SimplexVertex{wA, indexA, wB, indexB, float_t{1}};
			m_count = 1;
		}
	}

	void WriteCache(SimplexCache& cache) const
	{
		cache.SetMetric(GetMetric());
		cache.ClearIndices();
		for (auto i = decltype(m_count){0}; i < m_count; ++i)
		{
			cache.AddIndex(m_vertices[i].indexA, m_vertices[i].indexB);
		}
	}

	Vec2 GetSearchDirection() const
	{
		assert((m_count == 1) || (m_count == 2));
		switch (m_count)
		{
		case 1:
			return -m_vertices[0].get_w();

		case 2:
			{
				const auto e12 = m_vertices[1].get_w() - m_vertices[0].get_w();
				const auto sgn = Cross(e12, -m_vertices[0].get_w());
				// If sgn > 0, then origin is left of e12, else origin is right of e12.
				return (sgn > float_t{0})? Cross(float_t{1}, e12): Cross(e12, float_t{1});
			}

		default:
			return Vec2_zero;
		}
	}

	Vec2 GetClosestPoint() const
	{
		assert(m_count == 1 || m_count == 2 || m_count == 3);
		
		switch (m_count)
		{
		case 1:
			return m_vertices[0].get_w();

		case 2:
			return m_vertices[0].get_a() * m_vertices[0].get_w() + m_vertices[1].get_a() * m_vertices[1].get_w();

		case 3:
			return Vec2_zero;

		default:
			return Vec2_zero;
		}
	}

	void GetWitnessPoints(Vec2* pA, Vec2* pB) const
	{
		assert(m_count == 1 || m_count == 2 || m_count == 3);

		switch (m_count)
		{
		case 1:
			*pA = m_vertices[0].get_wA();
			*pB = m_vertices[0].get_wB();
			break;

		case 2:
			*pA = m_vertices[0].get_a() * m_vertices[0].get_wA() + m_vertices[1].get_a() * m_vertices[1].get_wA();
			*pB = m_vertices[0].get_a() * m_vertices[0].get_wB() + m_vertices[1].get_a() * m_vertices[1].get_wB();
			break;

		case 3:
			*pA = m_vertices[0].get_a() * m_vertices[0].get_wA()
				+ m_vertices[1].get_a() * m_vertices[1].get_wA()
				+ m_vertices[2].get_a() * m_vertices[2].get_wA();
			*pB = *pA;
			break;

		default:
			break;
		}
	}

	float_t GetMetric() const
	{
		assert(m_count == 1 || m_count == 2 || m_count == 3);
		switch (m_count)
		{
		case 1:	return float_t{0};
		case 2:	return Distance(m_vertices[0].get_w(), m_vertices[1].get_w());
		case 3:	return Cross(m_vertices[1].get_w() - m_vertices[0].get_w(), m_vertices[2].get_w() - m_vertices[0].get_w());
		default: return float_t{0};
		}
	}

	void Solve() noexcept;

private:
	void Solve2() noexcept;
	void Solve3() noexcept;

	size_type m_count = 0; ///< Count of valid vertex entries in m_vertices. Value between 0 and MaxVertices. @see m_vertices.
	SimplexVertex m_vertices[MaxVertices]; ///< Vertices. Only elements < m_count are valid. @see m_count.
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
void Simplex::Solve2() noexcept
{
	const auto w1 = m_vertices[0].get_w();
	const auto w2 = m_vertices[1].get_w();
	const auto e12 = w2 - w1;

	// w1 region
	const auto d12_2 = -Dot(w1, e12);
	if (d12_2 <= float_t{0})
	{
		// a2 <= 0, so we clamp it to 0
		m_vertices[0].set_a(float_t{1});
		m_count = 1;
		return;
	}

	// w2 region
	const auto d12_1 = Dot(w2, e12);
	if (d12_1 <= float_t{0})
	{
		// a1 <= 0, so we clamp it to 0
		m_vertices[1].set_a(float_t{1});
		m_vertices[0] = m_vertices[1];
		m_count = 1;
		return;
	}

	// Must be in e12 region.
	const auto inv_d12 = float_t{1} / (d12_1 + d12_2);
	m_vertices[0].set_a(d12_1 * inv_d12);
	m_vertices[1].set_a(d12_2 * inv_d12);
	m_count = 2;
}

// Possible regions:
// - points[2]
// - edge points[0]-points[2]
// - edge points[1]-points[2]
// - inside the triangle
void Simplex::Solve3() noexcept
{
	const auto w1 = m_vertices[0].get_w();
	const auto w2 = m_vertices[1].get_w();
	const auto w3 = m_vertices[2].get_w();

	// Edge12
	// [1      1     ][a1] = [1]
	// [w1.e12 w2.e12][a2] = [0]
	// a3 = 0
	const auto e12 = w2 - w1;
	const auto w1e12 = Dot(w1, e12);
	const auto w2e12 = Dot(w2, e12);
	const auto d12_1 = w2e12;
	const auto d12_2 = -w1e12;

	// Edge13
	// [1      1     ][a1] = [1]
	// [w1.e13 w3.e13][a3] = [0]
	// a2 = 0
	const auto e13 = w3 - w1;
	const auto w1e13 = Dot(w1, e13);
	const auto w3e13 = Dot(w3, e13);
	const auto d13_1 = w3e13;
	const auto d13_2 = -w1e13;

	// Edge23
	// [1      1     ][a2] = [1]
	// [w2.e23 w3.e23][a3] = [0]
	// a1 = 0
	const auto e23 = w3 - w2;
	const auto w2e23 = Dot(w2, e23);
	const auto w3e23 = Dot(w3, e23);
	const auto d23_1 = w3e23;
	const auto d23_2 = -w2e23;
	
	// Triangle123
	const auto n123 = Cross(e12, e13);

	const auto d123_1 = n123 * Cross(w2, w3);
	const auto d123_2 = n123 * Cross(w3, w1);
	const auto d123_3 = n123 * Cross(w1, w2);

	// w1 region
	if ((d12_2 <= float_t{0}) && (d13_2 <= float_t{0}))
	{
		m_vertices[0].set_a(float_t{1});
		m_count = 1;
		return;
	}

	// e12
	if ((d12_1 > float_t{0}) && (d12_2 > float_t{0}) && (d123_3 <= float_t{0}))
	{
		const auto inv_d12 = float_t{1} / (d12_1 + d12_2);
		m_vertices[0].set_a(d12_1 * inv_d12);
		m_vertices[1].set_a(d12_2 * inv_d12);
		m_count = 2;
		return;
	}

	// e13
	if ((d13_1 > float_t{0}) && (d13_2 > float_t{0}) && (d123_2 <= float_t{0}))
	{
		const auto inv_d13 = float_t{1} / (d13_1 + d13_2);
		m_vertices[0].set_a(d13_1 * inv_d13);
		m_vertices[2].set_a(d13_2 * inv_d13);
		m_count = 2;
		m_vertices[1] = m_vertices[2];
		return;
	}

	// w2 region
	if ((d12_1 <= float_t{0}) && (d23_2 <= float_t{0}))
	{
		m_vertices[1].set_a(float_t{1});
		m_count = 1;
		m_vertices[0] = m_vertices[1];
		return;
	}

	// w3 region
	if ((d13_1 <= float_t{0}) && (d23_1 <= float_t{0}))
	{
		m_vertices[2].set_a(float_t{1});
		m_count = 1;
		m_vertices[0] = m_vertices[2];
		return;
	}

	// e23
	if ((d23_1 > float_t{0}) && (d23_2 > float_t{0}) && (d123_1 <= float_t{0}))
	{
		const auto inv_d23 = float_t{1} / (d23_1 + d23_2);
		m_vertices[1].set_a(d23_1 * inv_d23);
		m_vertices[2].set_a(d23_2 * inv_d23);
		m_count = 2;
		m_vertices[0] = m_vertices[2];
		return;
	}

	// Must be in triangle123
	const auto inv_d123 = float_t{1} / (d123_1 + d123_2 + d123_3);
	m_vertices[0].set_a(d123_1 * inv_d123);
	m_vertices[1].set_a(d123_2 * inv_d123);
	m_vertices[2].set_a(d123_3 * inv_d123);
	m_count = 3;
}

void Simplex::Solve() noexcept
{
	assert(m_count == 1 || m_count == 2 || m_count == 3);
	
	switch (m_count)
	{
		case 1:
			break;
			
		case 2:
			Solve2();
			break;
			
		case 3:
			Solve3();
			break;
			
		default:
			break;
	}
}

DistanceOutput Distance(SimplexCache& cache, const DistanceInput& input)
{
#if defined(DO_GJK_PROFILING)
	++gjkCalls;
#endif

	// Initialize the simplex.
	Simplex simplex;
	simplex.ReadCache(cache, input.proxyA, input.transformA, input.proxyB, input.transformB);

	// Get simplex vertices as an array.
	const auto vertices = simplex.GetVertices();
	constexpr auto k_maxIters = uint32{20}; ///< Max number of support point calls.

	// These store the vertices of the last simplex so that we
	// can check for duplicates and prevent cycling.
	SimplexVertex::size_type saveA[Simplex::MaxVertices], saveB[Simplex::MaxVertices];

#if defined(DO_COMPUTE_CLOSEST_POINT)
	auto distanceSqr1 = MaxFloat;
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

		simplex.Solve();

		// If we have max points (3), then the origin is in the corresponding triangle.
		if (simplex.GetCount() == Simplex::MaxVertices)
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
		if (d.LengthSquared() < Square(Epsilon))
		{
			// The origin is probably contained by a line segment
			// or triangle. Thus the shapes are overlapped.

			// We can't return zero here even though there may be overlap.
			// In case the simplex is a point, segment, or triangle it is difficult
			// to determine if the origin is contained in the CSO or very close to it.
			break;
		}

		// Compute a tentative new simplex vertex using support points.
		const auto indexA = input.proxyA.GetSupport(MulT(input.transformA.q, -d));
		const auto indexB = input.proxyB.GetSupport(MulT(input.transformB.q, d));

		// Iteration count is equated to the number of support point calls.
		++iter;

		// Check for duplicate support points. This is the main termination criteria.
		auto duplicate = false;
		for (auto i = decltype(saveCount){0}; i < saveCount; ++i)
		{
			if ((indexA == saveA[i]) && (indexB == saveB[i]))
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
		const auto wA = Mul(input.transformA, input.proxyA.GetVertex(indexA));
		const auto wB = Mul(input.transformB, input.proxyB.GetVertex(indexB));
		simplex.AddVertex(SimplexVertex{wA, indexA, wB, indexB, 0});
	}

#if defined(DO_GJK_PROFILING)
	gjkIters += iter;
	gjkMaxIters = Max(gjkMaxIters, iter);
#endif

	// Prepare output.
	DistanceOutput output;
	simplex.GetWitnessPoints(&output.pointA, &output.pointB);
	output.distance = Distance(output.pointA, output.pointB);
	output.iterations = iter;

	// Cache the simplex.
	simplex.WriteCache(cache);

	// Apply radii if requested.
	if (input.useRadii)
	{
		const auto rA = input.proxyA.GetRadius();
		const auto rB = input.proxyB.GetRadius();
		const auto totalRadius = rA + rB;

		if ((output.distance > totalRadius) && (output.distance > Epsilon))
		{
			// Shapes are still no overlapped.
			// Move the witness points to the outer surface.
			output.distance -= totalRadius;
			const auto normal = Normalize(output.pointB - output.pointA);
			output.pointA += rA * normal;
			output.pointB -= rB * normal;
		}
		else
		{
			// Shapes are overlapped when radii are considered.
			// Move the witness points to the middle.
			const auto p = (output.pointA + output.pointB) / float_t{2};
			output.pointA = p;
			output.pointB = p;
			output.distance = float_t{0};
		}
	}
	return output;
}

} // namespace box2d