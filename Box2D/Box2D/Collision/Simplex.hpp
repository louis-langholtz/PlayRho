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

#ifndef Simplex_hpp
#define Simplex_hpp

#include <Box2D/Common/ArrayList.hpp>
#include <Box2D/Collision/SimplexEdge.hpp>

namespace box2d
{

	/// Simplex.
	///
	/// @detail An encapsulation of a point, line segment, or triangle.
	///   These are defined respectively as: a 0-simplex, a 1-simplex, and a 2-simplex.
	///   Used in doing GJK collision detection.
	///
	/// @note This data structure is 104-bytes large.
	///
	/// @invariant Vertex's for the same index must have the same point locations.
	/// @invariant There may not be more than one entry for the same index pair.
	///
	/// @sa https://en.wikipedia.org/wiki/Simplex
	/// @sa https://en.wikipedia.org/wiki/Gilbert%2DJohnson%2DKeerthi_distance_algorithm
	///
	class Simplex
	{
	public:
		/// Simplex edge collection.
		///
		/// @note This data structure may be 28 * 3 + 4 = 88-bytes large.
		///
		using Edges = ArrayList<SimplexEdge, MaxSimplexEdges, std::remove_const<decltype(MaxSimplexEdges)>::type>;

		using size_type = Edges::size_type;

		/// Coefficients.
		///
		/// @detail Collection of coefficient values.
		///
		/// @note This data structure is 4 * 3 + 4 = 16-bytes large.
		///
		using Coefficients = ArrayList<float_t, MaxSimplexEdges, std::remove_const<decltype(MaxSimplexEdges)>::type>;

		/// Calculates the "search direction" for the given simplex edge list.
		/// @param simplexEdges A one or two edge list.
		/// @warning Behavior is undefined if the given edge list has zero edges.
		/// @return "search direction" vector.
		static constexpr Vec2 CalcSearchDirection(const Edges& simplexEdges) noexcept;
		
		/// Gets the given simplex's "metric".
		static inline float_t CalcMetric(const Edges& simplexEdges);

		static Simplex Get(const SimplexEdge& s0) noexcept;

		/// Gets the simplex for the given 2 edges.
		///
		/// @note The given simplex vertices must have different index pairs or be of the same values.
		/// @warning Behavior is undefined if the given simplex edges index pairs are the same
		///    and the whole edges values are not also the same.
		///
		/// @param s0 Simplex edge 0.
		/// @param s1 Simplex edge 1.
		///
		/// @result One or two edge simplex.
		///
		static Simplex Get(const SimplexEdge& s0, const SimplexEdge& s1) noexcept;
		
		/// Gets the simplex for the given 3 edges.
		///
		/// @result One, two, or three edge simplex.
		///
		static Simplex Get(const SimplexEdge& s0, const SimplexEdge& s1, const SimplexEdge& s2) noexcept;
		
		/// Gets the simplex for the given collection of vertices.
		/// @param edges Collection of zero, one, two, or three simplex edges.
		/// @warning Behavior is undefined if the given collection has more than 3 edges.
		/// @return Zero, one, two, or three edge simplex.
		static Simplex Get(const Edges& edges) noexcept;

		Simplex() = default;

		Edges GetEdges() const noexcept { return m_simplexEdges; }

		const SimplexEdge& GetSimplexEdge(size_type index) const noexcept
		{
			return m_simplexEdges[index];
		}

		float_t GetCoefficient(size_type index) const noexcept
		{
			return m_normalizedWeights[index];
		}

		size_type GetSize() const noexcept { return m_simplexEdges.size(); }

	private:
		Simplex(const Edges& simplexEdges, const Coefficients& normalizedWeights):
			m_simplexEdges{simplexEdges}, m_normalizedWeights{normalizedWeights}
		{
			assert(simplexEdges.size() == normalizedWeights.size());
			assert(almost_equal(1, [&]()
			{
				auto sum = float_t(0);
				for (auto&& elem: normalizedWeights)
				{
					assert(elem > 0);
					sum += elem;
				}
				return sum;
			}()));
		}

		Edges m_simplexEdges; ///< Collection of valid simplex edges. 88-bytes.

		/// Normalized weights.
		///
		/// @detail Collection of coefficients (ranging from greater than 0 to less than 1).
		/// A.k.a.: barycentric coordinates.
		///
		/// @note This member variable is 16-bytes.
		///
		Coefficients m_normalizedWeights;
	};

	constexpr inline Vec2 Simplex::CalcSearchDirection(const Edges& simplexEdges) noexcept
	{
		assert((simplexEdges.size() == 1) || (simplexEdges.size() == 2));
		switch (simplexEdges.size())
		{
			case 1:
				return -GetPointDelta(simplexEdges[0]);
				
			case 2:
			{
				const auto e12 = GetPointDelta(simplexEdges[1]) - GetPointDelta(simplexEdges[0]);
				const auto sgn = Cross(e12, -GetPointDelta(simplexEdges[0]));
				// If sgn > 0, then origin is left of e12, else origin is right of e12.
				return (sgn > float_t{0})? GetRevPerpendicular(e12): GetFwdPerpendicular(e12);
			}
				
			default:
				return Vec2_zero;
		}
	}

	inline float_t Simplex::CalcMetric(const Edges& simplexEdges)
	{
		assert(simplexEdges.size() < 4);
		switch (simplexEdges.size())
		{
			case 0: return float_t{0};
			case 1: return float_t{0};
			case 2:	return Sqrt(GetLengthSquared(GetPointDelta(simplexEdges[1]) - GetPointDelta(simplexEdges[0])));
			case 3:	return Cross(GetPointDelta(simplexEdges[1]) - GetPointDelta(simplexEdges[0]),
								 GetPointDelta(simplexEdges[2]) - GetPointDelta(simplexEdges[0]));
			default: break; // should not be reached
		}
		return float_t{0};
	}

	inline Vec2 GetScaledDelta(const Simplex& simplex, Simplex::size_type index)
	{
		return simplex.GetSimplexEdge(index).GetPointDelta() * simplex.GetCoefficient(index);
	}

	/// Gets the "closest point".
	/// @note This uses the vertices "a" values when count is 2.
	constexpr inline Vec2 GetClosestPoint(const Simplex& simplex)
	{
		switch (simplex.GetSize())
		{
			case 1: return GetScaledDelta(simplex, 0);
			case 2: return GetScaledDelta(simplex, 0) + GetScaledDelta(simplex, 1);
			case 3: return Vec2_zero;
			default: return Vec2_zero;
		}
	}

}	

#endif /* Simplex_hpp */
