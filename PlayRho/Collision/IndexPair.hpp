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

#ifndef PLAYRHO_COLLISION_INDEXPAIR_HPP
#define PLAYRHO_COLLISION_INDEXPAIR_HPP

#include <PlayRho/Common/Settings.hpp>
#include <PlayRho/Common/Vector.hpp>

namespace playrho {
    
    /// Index pair.
    /// @note This data structure is at least 2-bytes large.
    struct IndexPair
    {
        /// Size type.
        /// @details Must be big enough to hold max posible count of vertices.
        using size_type = std::remove_const<decltype(MaxShapeVertices)>::type;
        
        /// @brief Invalid index.
        static constexpr size_type InvalidIndex = static_cast<size_type>(-1);
        
        size_type a; ///< Index of vertex from shape A.
        size_type b; ///< Index of vertex from shape B.
    };
    
    /// @brief Invalid index pair value.
    constexpr auto InvalidIndexPair = IndexPair{IndexPair::InvalidIndex, IndexPair::InvalidIndex};
    
    /// @brief Determines whether the two given index pairs are equal.
    /// @relatedalso IndexPair
    constexpr inline bool operator== (IndexPair lhs, IndexPair rhs)
    {
        return (lhs.a == rhs.a) && (lhs.b == rhs.b);
    }
    
    /// @brief Determines whether the two given index pairs are not equal.
    /// @relatedalso IndexPair
    constexpr inline bool operator!= (IndexPair lhs, IndexPair rhs)
    {
        return !(lhs == rhs);
    }
    
    /// @brief Index pairs.
    /// @note This data type is 6-bytes large (on at least one 64-bit platform).
    using IndexPair3 = Vector<MaxSimplexEdges, IndexPair>;
    
    static_assert(MaxSimplexEdges == 3, "Invalid assumption about size of MaxSimplexEdges");

    /// @brief Gets the number of valid indices in the given collection of index pairs.
    constexpr inline std::size_t GetNumIndices(IndexPair3 pairs) noexcept
    {
        return std::size_t{3}
        - ((pairs[0] == InvalidIndexPair)? 1u: 0u)
        - ((pairs[1] == InvalidIndexPair)? 1u: 0u)
        - ((pairs[2] == InvalidIndexPair)? 1u: 0u);
    }

} // namespace playrho

#endif // PLAYRHO_COLLISION_INDEXPAIR_HPP
