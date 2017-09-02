/*
 * Original work Copyright (c) 2007-2009 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#include <PlayRho/Common/ArrayList.hpp>
#include <PlayRho/Common/Vector.hpp>
#include <PlayRho/Collision/SimplexEdge.hpp>

namespace playrho
{

    /// Simplex.
    ///
    /// @details An encapsulation of a point, line segment, or triangle.
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
        /// @note This data is 28 * 3 + 4 = 88-bytes large (on at least one 64-bit platform).
        ///
        using Edges = ArrayList<SimplexEdge, MaxSimplexEdges, std::remove_const<decltype(MaxSimplexEdges)>::type>;

        /// Size type.
        ///
        /// @note This data type is explicitly set to 1-byte large.
        using size_type = Edges::size_type;

        /// Coefficients.
        ///
        /// @details Collection of coefficient values.
        ///
        /// @note This data structure is 4 * 3 + 4 = 16-bytes large.
        ///
        using Coefficients = ArrayList<Real, MaxSimplexEdges, std::remove_const<decltype(MaxSimplexEdges)>::type>;
        
        /// Simplex cache.
        ///
        /// @details Used to warm start Distance.
        /// Caches particular information from a simplex - a related metric and up-to 3 index pairs.
        ///
        /// @invariant As the metric and list of index pairs should be values from a snapshot of a
        ///   simplex, the metric and list of index pairs must not vary independent of each other.
        ///   As such, this data structure only allows these values to be changed in unison via object
        ///   construction or object assignment.
        ///
        /// @note This data structure is 12-bytes large.
        ///
        class Cache
        {
        public:
            Cache() = default;
            
            Cache(const Cache& copy) = default;
            
            constexpr Cache(Real metric, IndexPair3 indices) noexcept;
            
            /// Gets the metric that was set.
            /// @warning Behavior is undefined if metric was not previously set.
            ///   The IsMetricSet() method can be used to check dynamically if unsure.
            /// @sa SetMetric.
            /// @sa IsMetricSet.
            /// @return Value previously set.
            Real GetMetric() const noexcept;
            
            bool IsMetricSet() const noexcept;
            
            constexpr IndexPair3 GetIndices() const noexcept;
            
            constexpr IndexPair GetIndexPair(size_type index) const noexcept;
            
        private:
            Real m_metric = GetInvalid<Real>(); ///< Metric. @details This is a length or area value.            
            IndexPair3 m_indices{InvalidIndexPair, InvalidIndexPair, InvalidIndexPair}; ///< Indices. @details Collection of index-pairs.
        };

        static Cache GetCache(const Simplex::Edges& edges) noexcept;
        
        /// Gets index pairs for the given edges collection.
        ///
        static IndexPair3 GetIndexPairs(const Edges& collection) noexcept;

        /// Calculates the "search direction" for the given simplex edge list.
        /// @param simplexEdges A one or two edge list.
        /// @warning Behavior is undefined if the given edge list has zero edges.
        /// @return "search direction" vector.
        static constexpr Length2D CalcSearchDirection(const Edges& simplexEdges) noexcept;
        
        /// Gets the given simplex's "metric".
        static inline Real CalcMetric(const Edges& simplexEdges);

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

        constexpr Edges GetEdges() const noexcept;

        const SimplexEdge& GetSimplexEdge(size_type index) const noexcept;

        constexpr Real GetCoefficient(size_type index) const noexcept;

        constexpr size_type GetSize() const noexcept;

    private:
        Simplex(const Edges& simplexEdges, const Coefficients& normalizedWeights) noexcept;

        /// Collection of valid simplex edges.
        ///
        /// @note This member variable is 88-bytes.
        ///
        Edges m_simplexEdges;

        /// Normalized weights.
        ///
        /// @details Collection of coefficients (ranging from greater than 0 to less than 1).
        /// A.k.a.: barycentric coordinates.
        ///
        /// @note This member variable is 16-bytes.
        ///
        Coefficients m_normalizedWeights;
    };

    constexpr inline Simplex::Cache::Cache(Real metric, IndexPair3 indices) noexcept:
        m_metric{metric}, m_indices{indices}
    {
        // Intentionally empty
    }

    inline Real Simplex::Cache::GetMetric() const noexcept
    {
        assert(IsMetricSet());
        return m_metric;
    }
    
    inline bool Simplex::Cache::IsMetricSet() const noexcept
    {
        return GetNumIndices(m_indices) > std::size_t{0};
    }
    
    constexpr inline IndexPair3 Simplex::Cache::GetIndices() const noexcept
    {
        return m_indices;
    }
    
    constexpr inline IndexPair Simplex::Cache::GetIndexPair(size_type index) const noexcept
    {
        return m_indices[index];
    }

    inline Simplex::Cache Simplex::GetCache(const Simplex::Edges& edges) noexcept
    {
        return Simplex::Cache{Simplex::CalcMetric(edges), Simplex::GetIndexPairs(edges)};
    }

    inline IndexPair3 Simplex::GetIndexPairs(const Edges& collection) noexcept
    {
        auto list = IndexPair3{InvalidIndexPair, InvalidIndexPair, InvalidIndexPair};
        switch (collection.size())
        {
            case 3: list[2] = collection[2].GetIndexPair(); // fallthrough
            case 2: list[1] = collection[1].GetIndexPair(); // fallthrough
            case 1: list[0] = collection[0].GetIndexPair(); // fallthrough
        }
        return list;
    }

    constexpr inline Length2D Simplex::CalcSearchDirection(const Edges& simplexEdges) noexcept
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
                return (sgn > Real{0} * SquareMeter)? GetRevPerpendicular(e12): GetFwdPerpendicular(e12);
            }
                
            default:
                return Length2D{Real(0) * Meter, Real(0) * Meter};
        }
    }

    inline Real Simplex::CalcMetric(const Edges& simplexEdges)
    {
        assert(simplexEdges.size() < 4);
        switch (simplexEdges.size())
        {
            case 0: return Real{0};
            case 1: return Real{0};
            case 2:
            {
                const auto delta = GetPointDelta(simplexEdges[1]) - GetPointDelta(simplexEdges[0]);
                return Sqrt(GetLengthSquared(GetVec2(delta)));
            }
            case 3:
            {
                const auto delta10 = GetPointDelta(simplexEdges[1]) - GetPointDelta(simplexEdges[0]);
                const auto delta20 = GetPointDelta(simplexEdges[2]) - GetPointDelta(simplexEdges[0]);
                return Cross(GetVec2(delta10), GetVec2(delta20));
            }
            default: break; // should not be reached
        }
        return Real{0};
    }

    inline Simplex::Simplex(const Edges& simplexEdges,
                                            const Coefficients& normalizedWeights) noexcept:
        m_simplexEdges{simplexEdges}, m_normalizedWeights{normalizedWeights}
    {
        assert(simplexEdges.size() == normalizedWeights.size());
#ifndef NDEBUG
        const auto sum = std::accumulate(std::begin(normalizedWeights), std::end(normalizedWeights),
                                         Real(0));
        assert(AlmostEqual(1, sum));
#endif
    }

    constexpr inline Simplex::Edges Simplex::GetEdges() const noexcept
    {
        return m_simplexEdges;
    }
    
    const inline SimplexEdge& Simplex::GetSimplexEdge(size_type index) const noexcept
    {
        return m_simplexEdges[index];
    }
    
    constexpr inline Real Simplex::GetCoefficient(size_type index) const noexcept
    {
        return m_normalizedWeights[index];
    }
    
    /// @brief Gets the size in number of valid edges of this Simplex.
    /// @return Value between 0 and <code>MaxEdges</code> (inclusive).
    constexpr inline Simplex::size_type Simplex::GetSize() const noexcept
    {
        return m_simplexEdges.size();
    }

    inline Length2D GetScaledDelta(const Simplex& simplex, Simplex::size_type index)
    {
        return GetPointDelta(simplex.GetSimplexEdge(index)) * simplex.GetCoefficient(index);
    }

    /// Gets the "closest point".
    constexpr inline Length2D GetClosestPoint(const Simplex& simplex)
    {
        switch (simplex.GetSize())
        {
            case 1: return GetScaledDelta(simplex, 0);
            case 2: return GetScaledDelta(simplex, 0) + GetScaledDelta(simplex, 1);
            case 3: return Length2D{Real(0) * Meter, Real(0) * Meter};
            default: return Length2D{Real(0) * Meter, Real(0) * Meter};
        }
    }
}

#endif /* Simplex_hpp */
