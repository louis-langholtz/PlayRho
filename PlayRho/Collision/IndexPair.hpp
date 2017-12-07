/*
 * Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

#ifndef PLAYRHO_COLLISION_INDEXPAIR_HPP
#define PLAYRHO_COLLISION_INDEXPAIR_HPP

#include <PlayRho/Common/Settings.hpp>
#include <array>
#include <utility>

namespace playrho {
    
    /// Index pair.
    /// @note This data structure is at least 2-bytes large.
    using IndexPair = std::pair<VertexCounter, VertexCounter>;
    
    /// @brief Invalid index pair value.
    PLAYRHO_CONSTEXPR const auto InvalidIndexPair = IndexPair{
        InvalidVertex, InvalidVertex
    };
    
    /// @brief Index pairs.
    /// @note This data type is 6-bytes large (on at least one 64-bit platform).
    using IndexPair3 = std::array<IndexPair, MaxSimplexEdges>;
    
    static_assert(MaxSimplexEdges == 3, "Invalid assumption about size of MaxSimplexEdges");

    /// @brief Gets the number of valid indices in the given collection of index pairs.
    PLAYRHO_CONSTEXPR inline std::size_t GetNumIndices(IndexPair3 pairs) noexcept
    {
        return std::size_t{3}
        - ((std::get<0>(pairs) == InvalidIndexPair)? 1u: 0u)
        - ((std::get<1>(pairs) == InvalidIndexPair)? 1u: 0u)
        - ((std::get<2>(pairs) == InvalidIndexPair)? 1u: 0u);
    }
    
    /// Index pair distance.
    /// @details This structure is used to keep track of the best separating axis.
    struct IndexPairDistance
    {
        Length distance = GetInvalid<Length>(); ///< Separation.
        IndexPair indices = InvalidIndexPair; ///< Index pair.
    };
    
} // namespace playrho

#endif // PLAYRHO_COLLISION_INDEXPAIR_HPP
