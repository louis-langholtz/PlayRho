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

#ifndef Vector_hpp
#define Vector_hpp

#include <cstddef>

namespace playrho
{
    /// @brief Vector.
    /// @details Basically a constexpr enhanced std::array for C++14.
    /// @note This type should be drop-in replacable with std::array in C++17.
    template <typename T, std::size_t N>
    struct Vector
    {
        using value_type = T;
        using size_type = std::size_t;
        using difference_type = std::ptrdiff_t;
        using reference = value_type&;
        using const_reference = const value_type&;
        using pointer = value_type*;
        using const_pointer = const value_type*;
        
        constexpr size_type max_size() const noexcept { return N; }
        constexpr size_type size() const noexcept { return N; }
        constexpr size_type empty() const noexcept { return N == 0; }
        
        constexpr reference operator[](size_type pos);
        constexpr const_reference operator[](size_type pos) const;
    };
}

#endif /* Vector_hpp */
