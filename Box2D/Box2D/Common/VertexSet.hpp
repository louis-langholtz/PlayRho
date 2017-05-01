/*
 * Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

#ifndef VertexSet_hpp
#define VertexSet_hpp

#include <vector>
#include <Box2D/Common/Math.hpp>

namespace box2d
{
    /// Vertex Set.
    ///
    /// @details This is a container that enforces the invariant that no two
    /// vertices can be closer together than the minimum separation distance.
    ///
    class VertexSet
    {
    public:
        using const_pointer = const Length2D*;

        static Vec2::data_type GetDefaultMinSeparationSquared()
        {
            return Sqrt(std::numeric_limits<Vec2::data_type>::min()) * 2;
        }
        
        VertexSet(Vec2::data_type minSepSquared = GetDefaultMinSeparationSquared()):
            m_minSepSquared{std::abs(minSepSquared)}
        {
            assert(minSepSquared >= 0);
        }

        Vec2::data_type GetMinSeparationSquared() const noexcept { return m_minSepSquared; }

        bool add(Length2D value)
        {
            if (find(value) != end())
            {
                return false;
            }
            m_elements.push_back(value);
            return true;
        }

        size_t size() const noexcept { return m_elements.size(); }
        
        const_pointer begin() const { return &m_elements[0]; }
        
        const_pointer end() const { return &m_elements[m_elements.size()]; }

        /// Finds contained point whose delta with the given point has a squared length less
        /// than or equal to this set's minimum length squared value.
        const_pointer find(Length2D value) const
        {
            // squaring anything smaller than the sqrt(std::numeric_limits<Vec2::data_type>::min())
            // won't be reversible.
            // i.e. won't obey the property that square(sqrt(a)) == a and sqrt(square(a)) == a.
            for (auto&& elem: *this)
            {
                // length squared must be large enough to have a reasonable enough unit vector.
                const auto delta = value - elem;
                if (GetLengthSquared(StripUnits(delta)) <= m_minSepSquared)
                {
                    // found or delta poorly conditioned
                    return &elem;
                }
            }
            return end();
        }

        Length2D operator[](size_t index) const noexcept
        {
            return m_elements[index];
        }

    private:
        std::vector<Length2D> m_elements; ///< Elements.
        const Vec2::data_type m_minSepSquared; ///< Minimum length squared. sizeof(Vec2)/2 or 4-bytes.
    };
}

#endif /* VertexSet_hpp */
