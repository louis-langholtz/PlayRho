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

#ifndef ShapeSeparation_hpp
#define ShapeSeparation_hpp

#include <Box2D/Common/Math.hpp>
#include <Box2D/Common/Span.hpp>

namespace box2d
{
    /// Index separation.
    /// @details This structure is used to keep track of the best separating axis.
    struct IndexSeparation
    {
        using distance_type = Length;
        using index_type = std::remove_const<decltype(MaxShapeVertices)>::type;
        
        static constexpr distance_type GetInvalidDistance() noexcept
        {
            return std::numeric_limits<RealNum>::max() * Meter;
        }

        static constexpr index_type InvalidIndex = static_cast<index_type>(-1);
        
        distance_type separation = GetInvalidDistance();
        index_type index = InvalidIndex;
    };
    
    /// Index pair separation.
    /// @details This structure is used to keep track of the best separating axis.
    struct IndexPairSeparation
    {
        using distance_type = Length;
        using index_type = std::remove_const<decltype(MaxShapeVertices)>::type;
        
        static constexpr distance_type GetInvalidDistance() noexcept
        {
            return std::numeric_limits<RealNum>::max() * Meter;
        }

        static constexpr index_type InvalidIndex = static_cast<index_type>(-1);
        
        distance_type separation = GetInvalidDistance();
        index_type index1 = InvalidIndex;
        index_type index2 = InvalidIndex;
    };

    /// Gets the shape separation information for the most anti-parallel vector.
    /// @param points Collection of 0 or more points to find the most anti-parallel vector from and
    ///    its magnitude from the reference vector.
    /// @param refvec Reference vector.
    template <typename T1, typename T2>
    static inline IndexSeparation GetMostAntiParallelSeparation(Span<const T1> points,
                                                                const T2 refvec, const T1 offset)
    {
        // Search for the vector that is most anti-parallel to the reference vector.
        // See: https://en.wikipedia.org/wiki/Antiparallel_(mathematics)#Antiparallel_vectors
        auto result = IndexSeparation{};
        const auto count = points.size();
        for (auto i = decltype(count){0}; i < count; ++i)
        {
            // Get cosine of angle between refvec and vectors[i] multiplied by their
            // magnitudes (which will essentially be 1 for any two unit vectors).
            // Get distance from offset to vectors[i] in direction of refvec.
            const auto s = Dot(refvec, points[i] - offset);
            if (result.separation > s)
            {
                result.separation = s;
                result.index = static_cast<IndexSeparation::index_type>(i);
            }
        }
        return result;
    }

    /// Gets the max separation information.
    /// @return The index of the vertex and normal from <code>verts1</code> and <code>norms1</code>,
    ///   the index of the vertex from <code>verts2</code> (that had the maximum separation
    ///   distance from each other in the direction of that normal), and the maximal distance.
    IndexPairSeparation GetMaxSeparation(Span<const Length2D> verts1, Span<const UnitVec2> norms1,
                                         const Transformation& xf1,
                                         Span<const Length2D> verts2, const Transformation& xf2,
                                         Length stop = MaxFloat * Meter);

} // namespace box2d

#endif /* ShapeSeparation_hpp */
