/*
 * Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#include <PlayRho/Common/Math.hpp>

#include <vector>
#include <algorithm>

namespace playrho
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

        static Area GetDefaultMinSeparationSquared()
        {
            return Sqrt((std::numeric_limits<Vec2::value_type>::min)()) * SquareMeter;
        }
        
        VertexSet(Area minSepSquared = GetDefaultMinSeparationSquared()):
            m_minSepSquared{minSepSquared}
        {
            assert(minSepSquared >= Area{0});
        }

        Area GetMinSeparationSquared() const noexcept { return m_minSepSquared; }

        bool add(Length2D value)
        {
            if (find(value) != end())
            {
                return false;
            }
            m_elements.push_back(value);
            return true;
        }
        
        void clear() noexcept
        {
            m_elements.clear();
        }

        std::size_t size() const noexcept { return m_elements.size(); }
        
        const_pointer begin() const { return m_elements.data(); }
        
        const_pointer end() const { return m_elements.data() + m_elements.size(); }

        /// Finds contained point whose delta with the given point has a squared length less
        /// than or equal to this set's minimum length squared value.
        const_pointer find(Length2D value) const
        {
            // squaring anything smaller than the sqrt(std::numeric_limits<Vec2::data_type>::min())
            // won't be reversible.
            // i.e. won't obey the property that square(sqrt(a)) == a and sqrt(square(a)) == a.
            return std::find_if(begin(), end(), [&](Length2D elem) {
                // length squared must be large enough to have a reasonable enough unit vector.
                return GetLengthSquared(value - elem) <= m_minSepSquared;
            });
        }

        Length2D operator[](std::size_t index) const noexcept
        {
            return m_elements[index];
        }

    private:
        std::vector<Length2D> m_elements; ///< Elements.
        const Area m_minSepSquared; ///< Minimum length squared. sizeof(Vec2)/2 or 4-bytes.
    };
}

#endif /* VertexSet_hpp */
