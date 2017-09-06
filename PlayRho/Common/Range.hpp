/*
 * Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#ifndef Range_hpp
#define Range_hpp

#include <cstddef>

namespace playrho
{
    
    /// @brief Template range value class.
    template <typename IT>
    class Range
    {
    public:
        using iterator_type = IT;

        constexpr Range(iterator_type iter_begin, iterator_type iter_end) noexcept:
        	m_begin{iter_begin}, m_end{iter_end}
        {
            // Intentionally empty.
        }

        iterator_type begin() const noexcept
        {
            return m_begin;
        }

        iterator_type end() const noexcept
        {
            return m_end;
        }

        bool empty() const noexcept
        {
            return m_begin == m_end;
        }

    private:
        iterator_type m_begin;
        iterator_type m_end;
    };

    /// @brief Template sized range value class.
    template <typename IT>
    class SizedRange: public Range<IT>
    {
    public:
        using size_type = std::size_t;

        constexpr SizedRange(typename Range<IT>::iterator_type iter_begin,
                             typename Range<IT>::iterator_type iter_end,
                             size_type size) noexcept:
        	Range<IT>{iter_begin, iter_end}, m_size{size}
        {
            // Intentionally empty.
        }

        size_type size() const noexcept
        {
            return m_size;
        }

    private:
        size_type m_size;
    };

}

#endif /* Range_hpp */
