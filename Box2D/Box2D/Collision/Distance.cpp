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
#include <Box2D/Collision/DistanceProxy.hpp>
#include <Box2D/Collision/SimplexCache.hpp>
#include <Box2D/Collision/Shapes/CircleShape.h>
#include <Box2D/Collision/Shapes/EdgeShape.h>
#include <Box2D/Collision/Shapes/ChainShape.h>
#include <Box2D/Collision/Shapes/PolygonShape.h>

namespace box2d {

namespace {

/// Maximum number of supportable vertices.
constexpr auto MaxSimplexVertices = unsigned{3};

using IndexPairArray = std::array<IndexPair, MaxSimplexVertices>;

inline bool Find(const IndexPairArray& array, IndexPair key, std::size_t count)
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
	return DistanceProxy{GetRadius(shape), shape.GetPosition()};
}

inline DistanceProxy GetDistanceProxy(const PolygonShape& shape, child_count_t index)
{
	return DistanceProxy{GetRadius(shape), shape.GetVertices(), shape.GetVertexCount()};		
}

inline DistanceProxy GetDistanceProxy(const ChainShape& shape, child_count_t index)
{
	return DistanceProxy{GetRadius(shape), shape.GetVertex(index), shape.GetVertex(shape.GetNextIndex(index))};
}

inline DistanceProxy GetDistanceProxy(const EdgeShape& shape, child_count_t index)
{
	return DistanceProxy{GetRadius(shape), shape.GetVertex1(), shape.GetVertex2()};
}
	
}

class Simplex;
static float_t GetMetric(const Simplex& simplex);

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
	using size_type = IndexPair::size_type;

	SimplexVertex() = default;

	constexpr SimplexVertex(const SimplexVertex& copy) noexcept = default;

	constexpr SimplexVertex(Vec2 sA, size_type iA, Vec2 sB, size_type iB, float_t a_) noexcept:
		wA{sA}, wB{sB}, indexPair{iA,iB}, a{a_}
	{
		assert(a_ >= 0 && a_ <= 1);
	}

	constexpr SimplexVertex(const SimplexVertex& copy, float_t newA) noexcept:
		wA{copy.wA}, wB{copy.wB}, indexPair{copy.indexPair}, a{newA}
	{
		assert(newA >= 0 && newA <= 1);
	}

	constexpr Vec2 get_wA() const noexcept { return wA; }
	
	constexpr Vec2 get_wB() const noexcept { return wB; }

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
	float_t a; ///< Barycentric coordinate for closest point
};

/// Gets "w".
/// @return 2D vector value of wB minus wA.
static constexpr inline Vec2 GetW(const SimplexVertex sv)
{
	return sv.get_wB() - sv.get_wA();
}
	
static inline Vec2 GetScaledPointA(const SimplexVertex sv)
{
	return sv.get_wA() * sv.get_a();
}

static inline Vec2 GetScaledPointB(const SimplexVertex sv)
{
	return sv.get_wB() * sv.get_a();
}

#if defined(DO_COMPUTE_CLOSEST_POINT)

static inline Vec2 GetScaledDelta(const SimplexVertex sv)
{
	return GetW(sv) * sv.get_a();
}

#endif

/// Simplex.
/// @detail
/// An encapsulation of a point, line segment, or triangle.
/// These are defined respectively as: a 0-simplex, a 1-simplex, and a 2-simplex.
/// Used in doing GJK collision detection.
/// @sa https://en.wikipedia.org/wiki/Simplex
/// @sa https://en.wikipedia.org/wiki/Gilbert%2DJohnson%2DKeerthi_distance_algorithm
class Simplex
{
public:
	using size_type = std::remove_const<decltype(MaxSimplexVertices)>::type;

	Simplex() = default;
	
	Simplex(const SimplexVertex& sv1) noexcept:
		m_count{1}, m_vertices{sv1}
	{}

	Simplex(const SimplexVertex& sv1, const SimplexVertex& sv2) noexcept:
		m_count{2}, m_vertices{sv1, sv2}
	{}
	
	Simplex(const SimplexVertex& sv1, const SimplexVertex& sv2, const SimplexVertex& sv3) noexcept:
		m_count{3}, m_vertices{sv1, sv2, sv3}
	{}

	/// Gets count of valid vertices.
 	/// @return Value between 0 and MaxSimplexVertices.
	/// @see MaxSimplexVertices
	size_type size() const noexcept
	{
		return m_count;
	}

	const SimplexVertex& operator[](size_type index) const noexcept
	{
		assert(index < m_count);
		return m_vertices[index];
	}

	void clear() noexcept
	{
		m_count = 0;
	}

	void push_back(const SimplexVertex& vertex) noexcept
	{
		assert(m_count < MaxSimplexVertices);
		m_vertices[m_count] = vertex;
		++m_count;
	}

private:
	size_type m_count = 0; ///< Count of valid vertex entries in m_vertices. Value between 0 and MaxVertices. @see m_vertices.
	SimplexVertex m_vertices[MaxSimplexVertices]; ///< Vertices. Only elements < m_count are valid. @see m_count.
};

/// Gets the "search direction" for the given simplex.
/// @param simplex A one or two vertex simplex.
/// @warning Behavior is undefined if the given simplex has zero vertices.
/// @return "search direction" vector.
static inline Vec2 GetSearchDirection(const Simplex& simplex) noexcept
{
	const auto count = simplex.size();
	assert((count == 1) || (count == 2));
	switch (count)
	{
		case 1:
			return -GetW(simplex[0]);
			
		case 2:
		{
			const auto e12 = GetW(simplex[1]) - GetW(simplex[0]);
			const auto sgn = Cross(e12, -GetW(simplex[0]));
			// If sgn > 0, then origin is left of e12, else origin is right of e12.
			return (sgn > float_t{0})? GetRevPerpendicular(e12): GetFwdPerpendicular(e12);
		}
			
		default:
			return Vec2_zero;
	}
}

#if defined(DO_COMPUTE_CLOSEST_POINT)

/// Gets the "closest point".
/// @note This uses the vertices "a" values when count is 2.
static inline Vec2 GetClosestPoint(const Simplex& simplex)
{
	const auto count = simplex.size();
	assert(count < 4);
	switch (count)
	{
		case 1: return GetW(simplex[0]);
		case 2: return GetScaledDelta(simplex[0]) + GetScaledDelta(simplex[1]);
		case 3: return Vec2_zero;
		default: return Vec2_zero;
	}
}

#endif
				
static float_t GetMetric(const Simplex& simplex)
{
	assert(simplex.size() < 4);
	switch (simplex.size())
	{
		case 0: return float_t{0};
		case 1: return float_t{0};
		case 2:	return Sqrt(LengthSquared(GetW(simplex[0]) - GetW(simplex[1])));
		case 3:	return Cross(GetW(simplex[1]) - GetW(simplex[0]), GetW(simplex[2]) - GetW(simplex[0]));
		default: break; // should not be reached
	}
	return float_t{0};
}

static inline WitnessPoints GetWitnessPoints(const Simplex& simplex) noexcept
{
	const auto count = simplex.size();

	assert(count == 1 || count == 2 || count == 3);
	
	switch (count)
	{
		case 1:
			return WitnessPoints{
				simplex[0].get_wA(), 
				simplex[0].get_wB()
			};
		case 2:
			return WitnessPoints{
				GetScaledPointA(simplex[0]) + GetScaledPointA(simplex[1]),
				GetScaledPointB(simplex[0]) + GetScaledPointB(simplex[1])
			};
		case 3:
		{
			const auto point =
				GetScaledPointA(simplex[0]) +
				GetScaledPointA(simplex[1]) +
				GetScaledPointA(simplex[2]);
			return WitnessPoints{point, point};
		}
		default: // should not be reached!
			break;
	}
	return WitnessPoints{};
}

static SimplexVertex GetSimplexVertex(IndexPair indexPair,
									  const DistanceProxy& proxyA, const Transformation& xfA,
									  const DistanceProxy& proxyB, const Transformation& xfB)
{
	const auto wA = Transform(proxyA.GetVertex(indexPair.a), xfA);
	const auto wB = Transform(proxyB.GetVertex(indexPair.b), xfB);
	return SimplexVertex{wA, indexPair.a, wB, indexPair.b, float_t{0}};	
}

static inline Simplex GetSimplex(const SimplexCache& cache,
						  const DistanceProxy& proxyA, const Transformation& xfA,
						  const DistanceProxy& proxyB, const Transformation& xfB)
{
	Simplex simplex;
	const auto count = cache.GetCount();
	assert(count <= MaxSimplexVertices);
	for (auto i = decltype(count){0}; i < count; ++i)
	{
		simplex.push_back(GetSimplexVertex(cache.GetIndexPair(i), proxyA, xfA, proxyB, xfB));
	}
	return simplex;
}
	
/// Solves the given line segment simplex using barycentric coordinates.
///
/// @note The given simplex must have two simplex vertices.
/// @warning Behavior is undefined if the given simplex doesn't have two vertices.
///
/// @detail
/// p = a1 * w1 + a2 * w2
/// a1 + a2 = 1
///
/// The vector from the origin to the closest point on the line is
/// perpendicular to the line.
/// e12 = w2 - w1
/// dot(p, e) = 0
/// a1 * dot(w1, e) + a2 * dot(w2, e) = 0
///
/// 2-by-2 linear system
/// [1      1     ][a1] = [1]
/// [w1.e12 w2.e12][a2] = [0]
///
/// Define
/// d12_1 =  dot(w2, e12)
/// d12_2 = -dot(w1, e12)
/// d12 = d12_1 + d12_2
///
/// Solution
/// a1 = d12_1 / d12
/// a2 = d12_2 / d12
///
/// @param simplex Two-vertex simplex to provide a "solution" for.
/// @result One or two vertex "solution".
static inline Simplex Solve2(const Simplex& simplex) noexcept
{
	const auto w1 = GetW(simplex[0]);
	const auto w2 = GetW(simplex[1]);
	const auto e12 = w2 - w1;

	// w1 region
	const auto d12_2 = -Dot(w1, e12);
	if (d12_2 <= float_t{0})
	{
		// a2 <= 0, so we clamp it to 0
		return Simplex{SimplexVertex{simplex[0], float_t{1}}};
	}

	// w2 region
	const auto d12_1 = Dot(w2, e12);
	if (d12_1 <= float_t{0})
	{
		// a1 <= 0, so we clamp it to 0
		return Simplex{SimplexVertex{simplex[1], float_t{1}}};
	}

	// Must be in e12 region.
	const auto inv_d12 = float_t{1} / (d12_1 + d12_2);
	return Simplex{SimplexVertex{simplex[0], d12_1 * inv_d12}, SimplexVertex{simplex[1], d12_2 * inv_d12}};
}

/// Solves the given 3-vertex simplex.
///
/// @note The given simplex must have three simplex vertices.
/// @warning Behavior is undefined if the given simplex doesn't have three vertices.
///
/// @detail
/// Possible regions:
/// - points[2]
/// - edge points[0]-points[2]
/// - edge points[1]-points[2]
/// - inside the triangle
///
/// @param simplex Three-vertex simplex to provide a "solution" for.
/// @result One, two, or three vertex "solution".
static inline Simplex Solve3(const Simplex& simplex) noexcept
{
	const auto w1 = GetW(simplex[0]);
	const auto w2 = GetW(simplex[1]);
	const auto w3 = GetW(simplex[2]);

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
		return Simplex{SimplexVertex{simplex[0], float_t{1}}};
	}

	// e12
	if ((d12_1 > float_t{0}) && (d12_2 > float_t{0}) && (d123_3 <= float_t{0}))
	{
		const auto inv_d12 = float_t{1} / (d12_1 + d12_2);
		return Simplex{
			SimplexVertex{simplex[0], d12_1 * inv_d12},
			SimplexVertex{simplex[1], d12_2 * inv_d12}
		};
	}

	// e13
	if ((d13_1 > float_t{0}) && (d13_2 > float_t{0}) && (d123_2 <= float_t{0}))
	{
		const auto inv_d13 = float_t{1} / (d13_1 + d13_2);
		return Simplex{
			SimplexVertex{simplex[0], d13_1 * inv_d13},
			SimplexVertex{simplex[2], d13_2 * inv_d13}
		};
	}

	// w2 region
	if ((d12_1 <= float_t{0}) && (d23_2 <= float_t{0}))
	{
		return Simplex{SimplexVertex{simplex[1], float_t{1}}};
	}

	// w3 region
	if ((d13_1 <= float_t{0}) && (d23_1 <= float_t{0}))
	{
		return Simplex{SimplexVertex{simplex[2], float_t{1}}};
	}

	// e23
	if ((d23_1 > float_t{0}) && (d23_2 > float_t{0}) && (d123_1 <= float_t{0}))
	{
		const auto inv_d23 = float_t{1} / (d23_1 + d23_2);
		return Simplex{
			SimplexVertex{simplex[2], d23_2 * inv_d23},
			SimplexVertex{simplex[1], d23_1 * inv_d23}
		};
	}

	// Must be in triangle123
	const auto inv_d123 = float_t{1} / (d123_1 + d123_2 + d123_3);
	return Simplex{
		SimplexVertex{simplex[0], d123_1 * inv_d123},
		SimplexVertex{simplex[1], d123_2 * inv_d123},
		SimplexVertex{simplex[2], d123_3 * inv_d123}
	};
}

/// Solves the given simplex.
/// @param simplex A one, two, or three vertex simplex.
/// @warning Behavior is undefined if the given simplex has zero vertices.
/// @return One, two, or three vertex "solution".
static inline Simplex Solve(const Simplex& simplex) noexcept
{
	const auto count = simplex.size();
	assert(count == 1 || count == 2 || count == 3);
	switch (count)
	{
		case 1: return simplex;
		case 2: return Solve2(simplex);
		case 3: return Solve3(simplex);
		default: break;
	}
	return simplex;
}

static inline auto CopyIndexPairs(IndexPairArray& dst, const Simplex& src) noexcept
{
	const auto count = src.size();
	for (auto i = decltype(count){0}; i < count; ++i)
	{
		dst[i] = src[i].indexPair;
	}
	return count;
}

DistanceOutput Distance(SimplexCache& cache,
						const DistanceProxy& proxyA, const Transformation& transformA,
						const DistanceProxy& proxyB, const Transformation& transformB)
{
	assert(proxyA.GetVertexCount() > 0);
	assert(IsValid(transformA.p));
	assert(proxyB.GetVertexCount() > 0);
	assert(IsValid(transformB.p));
	
	// Initialize the simplex.
	auto simplex = GetSimplex(cache, proxyA, transformA, proxyB, transformB);

	// Compute the new simplex metric, if it is substantially different than
	// old metric then flush the simplex.
	if (simplex.size() > 1)
	{
		const auto metric1 = cache.GetMetric();
		const auto metric2 = GetMetric(simplex);
		if ((metric2 < (metric1 / 2)) || (metric2 > (metric1 * 2)) || (metric2 < float_t{0}) || almost_equal(metric2, float_t{0}))
		{
			simplex.clear();
		}
	}
	
	if (simplex.size() == 0)
	{
		simplex.push_back(GetSimplexVertex(IndexPair{0, 0}, proxyA, transformA, proxyB, transformB));
	}

	// Get simplex vertices as an array.
	// These store the vertices of the last simplex so that we
	// can check for duplicates and prevent cycling.
	IndexPairArray savedIndices;

#if defined(DO_COMPUTE_CLOSEST_POINT)
	auto distanceSqr1 = MaxFloat;
#endif

	// Main iteration loop.
	auto iter = std::remove_const<decltype(MaxDistanceIterations)>::type{0};
	while (iter < MaxDistanceIterations)
	{
		++iter;
	
		// Copy simplex so we can identify duplicates.
		const auto savedCount = CopyIndexPairs(savedIndices, simplex);

		simplex = Solve(simplex);

		// If we have max points (3), then the origin is in the corresponding triangle.
		if (simplex.size() == MaxSimplexVertices)
		{
			break;
		}

#if defined(DO_COMPUTE_CLOSEST_POINT)
		// Compute closest point.
		const auto p = GetClosestPoint(simplex);
		const auto distanceSqr2 = LengthSquared(p);

		// Ensure progress
		if (distanceSqr2 >= distanceSqr1)
		{
			//break;
		}
		distanceSqr1 = distanceSqr2;
#endif
		// Get search direction.
		const auto d = GetSearchDirection(simplex);
		assert(IsValid(d));

		// Ensure the search direction is numerically fit.
		if (almost_equal(LengthSquared(d), float_t{0}))
		{
			// The origin is probably contained by a line segment
			// or triangle. Thus the shapes are overlapped.

			// We can't return zero here even though there may be overlap.
			// In case the simplex is a point, segment, or triangle it is difficult
			// to determine if the origin is contained in the CSO or very close to it.
			break;
		}

		// Compute a tentative new simplex vertex using support points.
		const auto indexA = GetSupportIndex(proxyA, InverseRotate(-d, transformA.q));
		const auto indexB = GetSupportIndex(proxyB, InverseRotate(d, transformB.q));

		// Check for duplicate support points. This is the main termination criteria.
		// If there's a duplicate support point, code must exit loop to avoid cycling.
		if (Find(savedIndices, IndexPair{indexA, indexB}, savedCount))
		{
			break;
		}

		// New vertex is ok and needed.
		const auto wA = Transform(proxyA.GetVertex(indexA), transformA);
		const auto wB = Transform(proxyB.GetVertex(indexB), transformB);
		simplex.push_back(SimplexVertex{wA, indexA, wB, indexB, 0});
	}

	// Cache the simplex.
	cache.SetMetric(GetMetric(simplex));
	cache.ClearIndices();
	for (auto i = decltype(simplex.size()){0}; i < simplex.size(); ++i)
	{
		cache.AddIndex(simplex[i].indexPair);
	}

	return DistanceOutput{GetWitnessPoints(simplex), iter};
}
	
} // namespace box2d