/*
 * Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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

#ifndef ShapeSeparation_hpp
#define ShapeSeparation_hpp

#include <PlayRho/Common/Math.hpp>
#include <PlayRho/Common/Span.hpp>

namespace playrho
{
    class DistanceProxy;

    /// Index separation.
    /// @details This structure is used to keep track of the best separating axis.
    struct IndexSeparation
    {
        using distance_type = Length;
        using index_type = std::remove_const<decltype(MaxShapeVertices)>::type;
        
        static constexpr distance_type GetInvalidDistance() noexcept
        {
            return std::numeric_limits<Real>::max() * Meter;
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
            return std::numeric_limits<Real>::max() * Meter;
        }

        static constexpr index_type InvalidIndex = static_cast<index_type>(-1);
        
        distance_type separation = GetInvalidDistance();
        index_type index1 = InvalidIndex;
        index_type index2 = InvalidIndex;
    };

    /// @brief Gets the max separation information.
    /// @return Index of the vertex and normal from <code>proxy1</code>,
    ///   index of the vertex from <code>proxy2</code> (that had the maximum separation
    ///   distance from each other in the direction of that normal), and the maximal distance.
    IndexPairSeparation GetMaxSeparation(const DistanceProxy& proxy1, const Transformation xf1,
                                         const DistanceProxy& proxy2, const Transformation xf2,
                                         Length stop = MaxFloat * Meter);

    /// @brief Gets the max separation information.
    /// @return Index of the vertex and normal from <code>proxy1</code>,
    ///   index of the vertex from <code>proxy2</code> (that had the maximum separation
    ///   distance from each other in the direction of that normal), and the maximal distance.
    IndexPairSeparation GetMaxSeparation(const DistanceProxy& proxy1,
                                         const DistanceProxy& proxy2,
                                         Length stop = MaxFloat * Meter);
    
} // namespace playrho

#endif /* ShapeSeparation_hpp */
