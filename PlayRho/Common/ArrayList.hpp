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

#ifndef ArrayList_hpp
#define ArrayList_hpp

#include <PlayRho/Defines.hpp>

#include <type_traits>
#include <initializer_list>
#include <cassert>
#include <array>

namespace playrho
{
    /// Array list.
    template <typename VALUE_TYPE, std::size_t MAXSIZE, typename SIZE_TYPE = std::size_t>
    class ArrayList
    {
    public:
        using size_type = SIZE_TYPE;
        using value_type = VALUE_TYPE;
        using reference = value_type&;
        using const_reference = const value_type&;
        using pointer = value_type*;
        using const_pointer = const value_type*;

        constexpr ArrayList() noexcept
        {
            // Intentionally empty.
        }

        template <std::size_t COPY_MAXSIZE, typename COPY_SIZE_TYPE, typename = std::enable_if_t< COPY_MAXSIZE <= MAXSIZE >>
        constexpr ArrayList(const ArrayList<VALUE_TYPE, COPY_MAXSIZE, SIZE_TYPE>& copy):
            m_size{copy.size()},
            m_elements{copy.data()}
        {
            // Intentionally empty
        }

        template <std::size_t COPY_MAXSIZE, typename COPY_SIZE_TYPE, typename = std::enable_if_t< COPY_MAXSIZE <= MAXSIZE >>
        ArrayList& operator= (const ArrayList<VALUE_TYPE, COPY_MAXSIZE, COPY_SIZE_TYPE>& copy)
        {
            m_size = static_cast<SIZE_TYPE>(copy.size());
            m_elements = copy.data();
            return *this;
        }

        template <std::size_t SIZE, typename = std::enable_if_t< SIZE <= MAXSIZE >>
        ArrayList(value_type (&value)[SIZE]) noexcept
        {
            for (auto&& elem: value)
            {
                push_back(elem);
            }
        }

        ArrayList(std::initializer_list<value_type> list)
        {
            for (auto&& elem: list)
            {
                push_back(elem);
            }
        }
        
        constexpr ArrayList& Append(const value_type& value)
        {
            push_back(value);
            return *this;
        }

        constexpr void push_back(const value_type& value) noexcept
        {
            assert(m_size < MAXSIZE);
            m_elements[m_size] = value;
            ++m_size;
        }
        
        void size(size_type value) noexcept
        {
            assert(value <= MAXSIZE);
            m_size = value;
        }

        void clear() noexcept
        {
            m_size = 0;
        }
        
        bool empty() const noexcept { return m_size == 0; }

        bool add(value_type value) noexcept
        {
            if (m_size < MAXSIZE)
            {
                m_elements[m_size] = value;
                ++m_size;
                return true;
            }
            return false;
        }
        
        reference operator[](size_type index) noexcept
        {
            assert(index < MAXSIZE);
            return m_elements[index];
        }
        
        constexpr const_reference operator[](size_type index) const noexcept
        {
            assert(index < MAXSIZE);
            return m_elements[index];
        }

        /// Gets the size of this collection.
        /// @details This is the number of elements that have been added to this collection.
        /// @return Value between 0 and the maximum size for this collection.
        /// @sa max_size().
        constexpr size_type size() const noexcept { return m_size; }
        
        /// Gets the maximum size that this collection can be.
        /// @details This is the maximum number of elements that can be contained in this collection.
        constexpr size_type max_size() const noexcept { return MAXSIZE; }

        auto data() const noexcept { return m_elements.data(); }

        pointer begin() noexcept { return m_elements.data(); }
        pointer end() noexcept { return m_elements.data() + m_size; }
        
        const_pointer begin() const noexcept { return m_elements.data(); }
        const_pointer end() const noexcept { return m_elements.data() + m_size; }
        
    private:
        size_type m_size = size_type{0};
        std::array<value_type,MAXSIZE> m_elements;
    };
    
    template <typename T, std::size_t S>
    ArrayList<T, S>& operator+= (ArrayList<T, S>& lhs, const typename ArrayList<T, S>::data_type& rhs)
    {
        lhs.push_back(rhs);
        return lhs;
    }

    template <typename T, std::size_t S>
    ArrayList<T, S> operator+ (ArrayList<T, S> lhs, const typename ArrayList<T, S>::data_type& rhs)
    {
        lhs.push_back(rhs);
        return lhs;
    }

} /* namespace playrho */

namespace std
{
    /// Tuple size specialization for ArrayList classes.
    template< class T, size_t N, typename SIZE_TYPE >
    class tuple_size< playrho::ArrayList<T, N, SIZE_TYPE> >: public integral_constant<size_t, N>
    {
        // Intentionally empty.
    };
}

#endif /* ArrayList_hpp */
