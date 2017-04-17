/*
 * Original work Copyright (c) 2007-2009 Erin Catto http://www.box2d.org
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
		/// Maximum number of supportable edges in a simplex.
		static constexpr auto MaxEdges = uint8{3};

		/// Simplex edge collection.
		///
		/// @note This data is 28 * 3 + 4 = 88-bytes large (on at least one 64-bit platform).
		///
		using Edges = ArrayList<SimplexEdge, MaxEdges, std::remove_const<decltype(MaxEdges)>::type>;

		/// Size type.
		///
		/// @note This data type is explicitly set to 1-byte large.
		using size_type = Edges::size_type;

		/// Coefficients.
		///
		/// @detail Collection of coefficient values.
		///
		/// @note This data structure is 4 * 3 + 4 = 16-bytes large.
		///
		using Coefficients = ArrayList<RealNum, MaxEdges, std::remove_const<decltype(MaxEdges)>::type>;

		/// Index pairs.
		///
		/// @note This data type is 7-bytes large (on at least one 64-bit platform).
		using IndexPairs = ArrayList<IndexPair, MaxEdges, std::remove_const<decltype(MaxEdges)>::type>;
		
		/// Simplex cache.
		///
		/// @detail Used to warm start Distance.
		/// Caches particular information from a simplex - a related metric and up-to 3 index pairs.
		///
		/// @invariant As the metric and list of index pairs should be values from a snapshot of a
		///   simplex, the mertic and list of index pairs must not vary independent of each other.
		///   As such, this data structure only allows these values to be changed in unision via object
		///   construction or object assignment.
		///
		/// @note This data structure is 12-bytes large.
		///
		class Cache
		{
		public:
			Cache() = default;
			
			Cache(const Cache& copy) = default;
			
			BOX2D_CONSTEXPR Cache(RealNum metric, IndexPairs indices) noexcept;
			
			/// Gets the metric that was set.
			/// @note Behavior is undefined if metric was not previously set.
			///   The IsMetricSet() method can be used to check dynamically if unsure.
			/// @sa SetMetric.
			/// @sa IsMetricSet.
			/// @return Value previously set.
			RealNum GetMetric() const noexcept;
			
			bool IsMetricSet() const noexcept;
			
			BOX2D_CONSTEXPR IndexPairs GetIndices() const noexcept;
			
			BOX2D_CONSTEXPR size_type GetNumIndices() const noexcept;
			
			BOX2D_CONSTEXPR IndexPair GetIndexPair(size_type index) const noexcept;
			
		private:
			RealNum m_metric = GetInvalid<RealNum>(); ///< Metric. @detail This is a length or area value.			
			IndexPairs m_indices; ///< Indices. @detail Collection of index-pairs.
		};

		static Cache GetCache(const Simplex::Edges& edges) noexcept;
		
		/// Gets index pairs for the given edges collection.
		///
		static IndexPairs GetIndexPairs(const Edges& collection) noexcept;

		/// Calculates the "search direction" for the given simplex edge list.
		/// @param simplexEdges A one or two edge list.
		/// @warning Behavior is undefined if the given edge list has zero edges.
		/// @return "search direction" vector.
		static constexpr Length2D CalcSearchDirection(const Edges& simplexEdges) noexcept;
		
		/// Gets the given simplex's "metric".
		static inline RealNum CalcMetric(const Edges& simplexEdges);

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

		BOX2D_CONSTEXPR Edges GetEdges() const noexcept;

		const SimplexEdge& GetSimplexEdge(size_type index) const noexcept;

		BOX2D_CONSTEXPR RealNum GetCoefficient(size_type index) const noexcept;

		BOX2D_CONSTEXPR size_type GetSize() const noexcept;

	private:
		BOX2D_CONSTEXPR Simplex(const Edges& simplexEdges, const Coefficients& normalizedWeights) noexcept;

		/// Collection of valid simplex edges.
		///
		/// @note This member variable is 88-bytes.
		///
		Edges m_simplexEdges;

		/// Normalized weights.
		///
		/// @detail Collection of coefficients (ranging from greater than 0 to less than 1).
		/// A.k.a.: barycentric coordinates.
		///
		/// @note This member variable is 16-bytes.
		///
		Coefficients m_normalizedWeights;
	};

	BOX2D_CONSTEXPR inline Simplex::Cache::Cache(RealNum metric, IndexPairs indices) noexcept:
		m_metric{metric}, m_indices{indices}
	{
		// Intentionally empty
	}

	inline RealNum Simplex::Cache::GetMetric() const noexcept
	{
		assert(IsValid(m_metric));
		return m_metric;
	}
	
	inline bool Simplex::Cache::IsMetricSet() const noexcept
	{
		return IsValid(m_metric);
	}
	
	BOX2D_CONSTEXPR inline Simplex::IndexPairs Simplex::Cache::GetIndices() const noexcept
	{
		return m_indices;
	}
	
	BOX2D_CONSTEXPR inline Simplex::size_type Simplex::Cache::GetNumIndices() const noexcept
	{
		return m_indices.size();
	}
	
	BOX2D_CONSTEXPR inline IndexPair Simplex::Cache::GetIndexPair(size_type index) const noexcept
	{
		return m_indices[index];
	}

	inline Simplex::Cache Simplex::GetCache(const Simplex::Edges& edges) noexcept
	{
		return Simplex::Cache{Simplex::CalcMetric(edges), Simplex::GetIndexPairs(edges)};
	}

	inline Simplex::IndexPairs Simplex::GetIndexPairs(const Edges& collection) noexcept
	{
		IndexPairs list;
		for (auto&& element: collection)
		{
			list.push_back(element.GetIndexPair());
		}
		return list;
	}

	BOX2D_CONSTEXPR inline Length2D Simplex::CalcSearchDirection(const Edges& simplexEdges) noexcept
	{
		assert((simplexEdges.size() == 1) || (simplexEdges.size() == 2));
		switch (simplexEdges.size())
		{
			case 1:
			{
				return -GetPointDelta(simplexEdges[0]);
			}

			case 2:
			{
				const auto e12 = GetPointDelta(simplexEdges[1]) - GetPointDelta(simplexEdges[0]);
				const auto e0 = GetPointDelta(simplexEdges[0]);
				const auto sgn = Cross(e12, -e0);
				// If sgn > 0, then origin is left of e12, else origin is right of e12.
				return (sgn > RealNum{0} * SquareMeter)? GetRevPerpendicular(e12): GetFwdPerpendicular(e12);
			}
				
			default:
				return Vec2_zero * Meter;
		}
	}

	inline RealNum Simplex::CalcMetric(const Edges& simplexEdges)
	{
		assert(simplexEdges.size() < 4);
		switch (simplexEdges.size())
		{
			case 0: return RealNum{0};
			case 1: return RealNum{0};
			case 2:
			{
				const auto delta = GetPointDelta(simplexEdges[1]) - GetPointDelta(simplexEdges[0]);
				return Sqrt(GetLengthSquared(StripUnits(delta)));
			}
			case 3:
			{
				const auto delta10 = GetPointDelta(simplexEdges[1]) - GetPointDelta(simplexEdges[0]);
				const auto delta20 = GetPointDelta(simplexEdges[2]) - GetPointDelta(simplexEdges[0]);
				return Cross(StripUnits(delta10), StripUnits(delta20));
			}
			default: break; // should not be reached
		}
		return RealNum{0};
	}

	BOX2D_CONSTEXPR inline Simplex::Simplex(const Edges& simplexEdges, const Coefficients& normalizedWeights) noexcept:
		m_simplexEdges{simplexEdges}, m_normalizedWeights{normalizedWeights}
	{
		assert(simplexEdges.size() == normalizedWeights.size());
		assert(almost_equal(1, [&]() {
			auto sum = RealNum(0);
			for (auto&& elem: normalizedWeights)
			{
				assert(elem >= 0);
				sum += elem;
			}
			return sum;
		}()));
	}

	BOX2D_CONSTEXPR inline Simplex::Edges Simplex::GetEdges() const noexcept
	{
		return m_simplexEdges;
	}
	
	const inline SimplexEdge& Simplex::GetSimplexEdge(size_type index) const noexcept
	{
		return m_simplexEdges[index];
	}
	
	BOX2D_CONSTEXPR inline RealNum Simplex::GetCoefficient(size_type index) const noexcept
	{
		return m_normalizedWeights[index];
	}
	
	BOX2D_CONSTEXPR inline Simplex::size_type Simplex::GetSize() const noexcept
	{
		return m_simplexEdges.size();
	}

	inline Length2D GetScaledDelta(const Simplex& simplex, Simplex::size_type index)
	{
		return simplex.GetSimplexEdge(index).GetPointDelta() * simplex.GetCoefficient(index);
	}

	/// Gets the "closest point".
	BOX2D_CONSTEXPR inline Length2D GetClosestPoint(const Simplex& simplex)
	{
		switch (simplex.GetSize())
		{
			case 1: return GetScaledDelta(simplex, 0);
			case 2: return GetScaledDelta(simplex, 0) + GetScaledDelta(simplex, 1);
			case 3: return Vec2_zero * Meter;
			default: return Vec2_zero * Meter;
		}
	}

}	

#endif /* Simplex_hpp */
