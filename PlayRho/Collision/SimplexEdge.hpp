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

#ifndef PLAYRHO_SIMPLEX_EDGE_HPP
#define PLAYRHO_SIMPLEX_EDGE_HPP

#include <PlayRho/Common/Math.hpp>
#include <PlayRho/Collision/IndexPair.hpp>

namespace playrho
{
    /// Simplex edge.
    ///
    /// @details This is the locations (in world coordinates) and indices of a pair of vertices
    /// from two shapes (shape A and shape B).
    ///
    /// @note This data structure is 28-bytes large (on at least one 64-bit platform).
    ///
    class SimplexEdge
    {
    public:
        
        /// @brief Index type.
        using index_type = IndexPair::size_type;
        
        /// Default constructor.
        SimplexEdge() = default;
        
        /// @brief Copy constructor.
        constexpr SimplexEdge(const SimplexEdge& copy) = default;
        
        /// Initializing constructor.
        /// @param pA Point A in world coordinates.
        /// @param iA Index of point A within the shape that it comes from.
        /// @param pB Point B in world coordinates.
        /// @param iB Index of point B within the shape that it comes from.
        constexpr SimplexEdge(Length2D pA, index_type iA, Length2D pB, index_type iB) noexcept;
        
        /// Gets point A (in world coordinates).
        constexpr auto GetPointA() const noexcept { return m_wA; }
        
        /// Gets point B (in world coordinates).
        constexpr auto GetPointB() const noexcept { return m_wB; }

        /// @brief Gets index A.
        constexpr auto GetIndexA() const noexcept { return m_indexPair.a; }
        
        /// @brief Gets index B.
        constexpr auto GetIndexB() const noexcept { return m_indexPair.b; }

        /// @brief Gets the index pair.
        constexpr auto GetIndexPair() const noexcept { return m_indexPair; }

    private:
        Length2D m_wA; ///< Point A in world coordinates. This is the support point in proxy A. 8-bytes.
        Length2D m_wB; ///< Point B in world coordinates. This is the support point in proxy B. 8-bytes.
        IndexPair m_indexPair; ///< Index pair. @details Indices of points A and B. 2-bytes.
    };
    
    constexpr inline SimplexEdge::SimplexEdge(Length2D pA, index_type iA, Length2D pB, index_type iB) noexcept:
        m_wA{pA}, m_wB{pB}, m_indexPair{iA, iB}
    {
    }
    
    /// Gets "w".
    /// @return 2D vector value of wB minus wA.
    constexpr inline Length2D GetPointDelta(const SimplexEdge& sv)
    {
        return sv.GetPointB() - sv.GetPointA();
    }
    
    /// @brief SimplexEdge equality operator.
    constexpr inline bool operator== (const SimplexEdge& lhs, const SimplexEdge& rhs)
    {
        return (lhs.GetPointA() == rhs.GetPointA())
            && (lhs.GetPointB() == rhs.GetPointB())
            && (lhs.GetIndexPair() == rhs.GetIndexPair());
    }
    
    /// @brief SimplexEdge inequality operator.
    constexpr inline bool operator!= (const SimplexEdge& lhs, const SimplexEdge& rhs)
    {
        return !(lhs == rhs);
    }
}

#endif /* PLAYRHO_SIMPLEX_EDGE_HPP */
