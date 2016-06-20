/*
* Original work Copyright (c) 2007-2009 Erin Catto http://www.box2d.org
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

namespace {

/// Maximum number of supportable vertices.
constexpr auto MaxSimplexVertices = unsigned{3};

using IndexPairArray = std::array<IndexPair, MaxSimplexVertices>;

bool Find(const IndexPairArray& array, IndexPair key, std::size_t count)
{
	assert(count <= array.size());
	for (std::size_t i = std::size_t{0}; i < count; ++i)
	{
		if (array[i] == key)
		{
			return true;
		}
	}
	return false;
}

inline DistanceProxy GetDistanceProxy(const CircleShape& shape, child_count_t index)
{
	return DistanceProxy{shape.GetRadius(), shape.GetPosition()};
}

inline DistanceProxy GetDistanceProxy(const PolygonShape& shape, child_count_t index)
{
	return DistanceProxy{shape.GetRadius(), shape.GetVertices(), shape.GetVertexCount()};		
}

inline DistanceProxy GetDistanceProxy(const ChainShape& shape, child_count_t index)
{
	return DistanceProxy{shape.GetRadius(), shape.GetVertex(index), shape.GetVertex(shape.GetNextIndex(index))};
}

inline DistanceProxy GetDistanceProxy(const EdgeShape& shape, child_count_t index)
{
	return DistanceProxy{shape.GetRadius(), shape.GetVertex1(), shape.GetVertex2()};
}
	
}
	
DistanceProxy GetDistanceProxy(const Shape& shape, child_count_t index)
{
	switch (shape.GetType())
	{
		case Shape::e_circle: return GetDistanceProxy(*static_cast<const CircleShape*>(&shape), index);
		case Shape::e_polygon: return GetDistanceProxy(*static_cast<const PolygonShape*>(&shape), index);
		case Shape::e_chain: return GetDistanceProxy(*static_cast<const ChainShape*>(&shape), index);
		case Shape::e_edge: return GetDistanceProxy(*static_cast<const EdgeShape*>(&shape), index);
		case Shape::e_typeCount: break;
	}
	assert(false);
	return DistanceProxy{0, nullptr, 0};
}

class SimplexVertex
{
public:
	using size_type = DistanceProxy::size_type;

	SimplexVertex() = default;

	constexpr SimplexVertex(const SimplexVertex& copy) noexcept = default;

	constexpr SimplexVertex(Vec2 sA, size_type iA, Vec2 sB, size_type iB, float_t a_) noexcept:
		wA{sA}, wB{sB}, indexPair{iA,iB}, w{sB - sA}, a{a_}
	{
		assert(a_ >= 0 && a_ <= 1);
	}

	constexpr SimplexVertex(const SimplexVertex& copy, float_t newA) noexcept:
		wA{copy.wA}, wB{copy.wB}, indexPair{copy.indexPair}, w{copy.w}, a{newA}
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

	IndexPair indexPair; ///< Indexes of wA and wB.
	
private:
	Vec2 wA; ///< Support point in proxy A.
	Vec2 wB; ///< Support point in proxy B.
	Vec2 w; ///< wB - wA. @see wA. @see wB.
	float_t a; ///< Barycentric coordinate for closest point
};

class Simplex
{
public:
	using size_type = std::remove_const<decltype(MaxSimplexVertices)>::type;

	Simplex() = default;
	
	Simplex(const SimplexVertex& sv1) noexcept:
		m_count{1}, m_vertices{sv1}
	{}

	Simplex(const SimplexVertex& sv1, const SimplexVertex& sv2) noexcept:
		m_count{2}, m_vertices{sv1, sv2}, m_metric{Distance(m_vertices[0].get_w(), m_vertices[1].get_w())}
	{}
	
	Simplex(const SimplexVertex& sv1, const SimplexVertex& sv2, const SimplexVertex& sv3) noexcept:
		m_count{3}, m_vertices{sv1, sv2, sv3}, m_metric{Cross(m_vertices[1].get_w() - m_vertices[0].get_w(), m_vertices[2].get_w() - m_vertices[0].get_w())}
	{}

	/// Gets count of valid vertices.
 	/// @return Value between 0 and MaxSimplexVertices.
	/// @see MaxSimplexVertices
	size_type GetCount() const noexcept
	{
		return m_count;
	}

	const SimplexVertex& GetVertex(size_type index) const noexcept
	{
		assert(index < m_count);
		return m_vertices[index];
	}

	const SimplexVertex* GetVertices() const noexcept
	{
		return m_vertices;
	}

	void AddVertex(const SimplexVertex& vertex) noexcept
	{
		assert(m_count < MaxSimplexVertices);
		m_vertices[m_count] = vertex;
		++m_count;
	}

	void ReadCache(const SimplexCache& cache,
				   const DistanceProxy& proxyA, const Transform& transformA,
				   const DistanceProxy& proxyB, const Transform& transformB)
	{
		assert(cache.GetCount() <= MaxSimplexVertices);
		
		// Copy data from cache.
		{
			const auto count = cache.GetCount();
			for (auto i = decltype(count){0}; i < count; ++i)
			{
				const auto indexPair = cache.GetIndexPair(i);
				const auto wA = Mul(transformA, proxyA.GetVertex(indexPair.a));
				const auto wB = Mul(transformB, proxyB.GetVertex(indexPair.b));
				m_vertices[i] = SimplexVertex{wA, indexPair.a, wB, indexPair.b, float_t{0}};
			}
			m_count = count;
		}

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
			cache.AddIndex(m_vertices[i].indexPair);
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

	/// Gets the "closest point".
	/// @note This uses the vertices "A" values when count is 2.
	Vec2 GetClosestPoint() const
	{
		assert(m_count == 1 || m_count == 2 || m_count == 3);
		
		switch (m_count)
		{
		case 1: return m_vertices[0].get_w();
		case 2: return m_vertices[0].get_a() * m_vertices[0].get_w() + m_vertices[1].get_a() * m_vertices[1].get_w();
		case 3: return Vec2_zero;
		default: return Vec2_zero;
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

	/// "Solves" the simplex.
	/// @detail This updates the simplex vertexes - setting the "A" value of each and possibly changing the simplex vertex count.
	/// @sa GetCount().
	void Solve() noexcept;

private:
	void Solve2() noexcept;
	void Solve3() noexcept;

	size_type m_count = 0; ///< Count of valid vertex entries in m_vertices. Value between 0 and MaxVertices. @see m_vertices.
	SimplexVertex m_vertices[MaxSimplexVertices]; ///< Vertices. Only elements < m_count are valid. @see m_count.
	float_t m_metric = 0;
};

static SimplexVertex GetSimplexVertex(IndexPair indexPair,
									  const DistanceProxy& proxyA, const Transform& xfA,
									  const DistanceProxy& proxyB, const Transform& xfB)
{
	const auto wA = Mul(xfA, proxyA.GetVertex(indexPair.a));
	const auto wB = Mul(xfB, proxyB.GetVertex(indexPair.b));
	return SimplexVertex{wA, indexPair.a, wB, indexPair.b, float_t{0}};	
}

static Simplex GetSimplex(const SimplexCache& cache,
						  const DistanceProxy& proxyA, const Transform& xfA,
						  const DistanceProxy& proxyB, const Transform& xfB)
{
	const auto count = cache.GetCount();
	assert(count <= 3);
	switch (count)
	{
		case 1:
			return Simplex{
				GetSimplexVertex(cache.GetIndexPair(0), proxyA, xfA, proxyB, xfB)
			};
		case 2:
			return Simplex{
				GetSimplexVertex(cache.GetIndexPair(0), proxyA, xfA, proxyB, xfB),
				GetSimplexVertex(cache.GetIndexPair(1), proxyA, xfA, proxyB, xfB)
			};
		case 3:
			return Simplex{
				GetSimplexVertex(cache.GetIndexPair(0), proxyA, xfA, proxyB, xfB),
				GetSimplexVertex(cache.GetIndexPair(1), proxyA, xfA, proxyB, xfB),
				GetSimplexVertex(cache.GetIndexPair(2), proxyA, xfA, proxyB, xfB)
			};
		default: break;
	}
	return Simplex{};
}
	
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

static inline auto CopyIndexPairs(IndexPairArray& dst, const Simplex& src)
{
	const auto count = src.GetCount();
	for (auto i = decltype(count){0}; i < count; ++i)
	{
		dst[i] = src.GetVertex(i).indexPair;
	}
	return count;
}

DistanceOutput Distance(SimplexCache& cache, const DistanceInput& input)
{
#if defined(DO_GJK_PROFILING)
	++gjkCalls;
#endif

	assert(input.proxyA.GetVertexCount() > 0);
	assert(input.proxyB.GetVertexCount() > 0);

	// Initialize the simplex.
	Simplex simplex; // = GetSimplex(cache, input.proxyA, input.transformA, input.proxyB, input.transformB);
	simplex.ReadCache(cache, input.proxyA, input.transformA, input.proxyB, input.transformB);

	// Get simplex vertices as an array.
	constexpr auto k_maxIters = unsigned{20}; ///< Max number of support point calls.

	// These store the vertices of the last simplex so that we
	// can check for duplicates and prevent cycling.
	IndexPairArray savedIndices;

#if defined(DO_COMPUTE_CLOSEST_POINT)
	auto distanceSqr1 = MaxFloat;
#endif

	// Main iteration loop.
	auto iter = decltype(k_maxIters){0};
	while (iter < k_maxIters)
	{
		// Copy simplex so we can identify duplicates.
		const auto savedCount = CopyIndexPairs(savedIndices, simplex);

		simplex.Solve();

		// If we have max points (3), then the origin is in the corresponding triangle.
		if (simplex.GetCount() == MaxSimplexVertices)
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

		// Iteration count is equated to the number of support point calls.
		++iter;
		
		// Compute a tentative new simplex vertex using support points.
		const auto indexA = input.proxyA.GetSupportIndex(MulT(input.transformA.q, -d));
		const auto indexB = input.proxyB.GetSupportIndex(MulT(input.transformB.q, d));

		// Check for duplicate support points. This is the main termination criteria.
		// If there's a duplicate support point, code must exit loop to avoid cycling.
		if (Find(savedIndices, IndexPair{indexA, indexB}, savedCount))
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