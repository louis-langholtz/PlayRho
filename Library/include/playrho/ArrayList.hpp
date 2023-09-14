/*
 * Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2023 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#ifndef PLAYRHO_ARRAYLIST_HPP
#define PLAYRHO_ARRAYLIST_HPP

#include <array>
#include <cassert>
#include <initializer_list>
#include <type_traits>
#include <utility> // for std::move

#include <playrho/Defines.hpp>

namespace playrho {

/// @brief Array list.
/// @details This is a <code>std::array</code> backed <code>std::vector</code> like container. It
///   provides vector like behavior whose max size is capped at the size given by the template max
///   size parameter without using dynamic storage.
template <typename VALUE_TYPE, std::size_t MAXSIZE, typename SIZE_TYPE = std::size_t>
class ArrayList
{
public:
    /// @brief Size type.
    using size_type = SIZE_TYPE;

    /// @brief Value type.
    using value_type = VALUE_TYPE;

    /// @brief Reference type.
    using reference = value_type&;

    /// @brief Constant reference type.
    using const_reference = const value_type&;

    /// @brief Pointer type.
    using pointer = value_type*;

    /// @brief Constant pointer type.
    using const_pointer = const value_type*;

    /// @brief Iterator type.
    using iterator = value_type*;

    /// @brief Constant iterator type.
    using const_iterator = const value_type*;

    /// @brief Default constructor.
    /// @note Some older versions of gcc have issues with this being defaulted.
    constexpr ArrayList() noexcept = default;

    template <std::size_t COPY_MAXSIZE, typename COPY_SIZE_TYPE,
              typename = std::enable_if_t<COPY_MAXSIZE <= MAXSIZE>>
    constexpr explicit ArrayList(const ArrayList<VALUE_TYPE, COPY_MAXSIZE, SIZE_TYPE>& copy)
        : m_size{size(copy)}, m_elements{data(copy)}
    {
        // Intentionally empty
    }

    /// @brief Assignment operator.
    template <std::size_t COPY_MAXSIZE, typename COPY_SIZE_TYPE,
              typename = std::enable_if_t<COPY_MAXSIZE <= MAXSIZE>>
    ArrayList& operator=(const ArrayList<VALUE_TYPE, COPY_MAXSIZE, COPY_SIZE_TYPE>& copy)
    {
        m_size = static_cast<SIZE_TYPE>(size(copy));
        m_elements = data(copy);
        return *this;
    }

    template <std::size_t SIZE, typename = std::enable_if_t<SIZE <= MAXSIZE>>
    explicit ArrayList(value_type (&value)[SIZE]) noexcept(std::is_nothrow_copy_assignable_v<VALUE_TYPE>)
    {
        for (auto&& elem : value) {
            push_back(elem);
        }
    }

    ArrayList(std::initializer_list<value_type> list) noexcept(std::is_nothrow_copy_assignable_v<VALUE_TYPE>)
    {
        for (auto&& elem : list) {
            push_back(elem);
        }
    }

    /// @brief Appends the given value onto back.
    /// @return Reference to this instance.
    constexpr ArrayList& Append(value_type value) noexcept(std::is_nothrow_move_assignable_v<VALUE_TYPE>)
    {
        push_back(std::move(value));
        return *this;
    }

    /// @brief Pushes given value onto back.
    /// @pre <code>size()</code> is less than <code>max_size()</code>.
    /// @post <code>size()</code> is one greater than before.
    /// @post <code>empty()</code> returns false.
    constexpr void push_back(value_type value) noexcept(std::is_nothrow_move_assignable_v<VALUE_TYPE>)
    {
        assert(m_size < max_size());
        m_elements[m_size] = std::move(value); // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
        ++m_size;
    }

    /// @brief Sets the size to the given value.
    /// @pre @p value is less-than or equal-to <code>max_size()</code>.
    /// @post <code>size()</code> returns the value given.
    /// @see size().
    void size(size_type value) noexcept
    {
        assert(value <= max_size());
        m_size = value;
    }

    /// @brief Resets size to zero.
    /// @see size().
    void clear() noexcept
    {
        m_size = 0;
    }

    /// @brief Gets whether this object has no elements.
    /// @return true if <code>size()</code> is zero, false otherwise.
    /// @see size().
    bool empty() const noexcept
    {
        return m_size == 0;
    }

    /// @brief Adds given value if space available.
    /// @post On successful addition, <code>size()</code> returns value one greater than before.
    /// @return true if value was added, false otherwise.
    bool add(value_type value) noexcept(std::is_nothrow_move_assignable_v<VALUE_TYPE>)
    {
        if (m_size < max_size()) {
            m_elements[m_size] = std::move(value);
            ++m_size;
            return true;
        }
        return false;
    }

    /// @brief Accesses element at given index.
    /// @pre @p index is less than <code>max_size()</code>.
    reference operator[](size_type index) noexcept
    {
        assert(index < max_size());
        return m_elements[index]; // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
    }

    /// @brief Accesses element at given index.
    /// @pre @p index is less than <code>max_size()</code>.
    constexpr const_reference operator[](size_type index) const noexcept
    {
        assert(index < max_size());
        return m_elements[index]; // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
    }

    /// Gets the size of this collection.
    /// @details This is the number of elements that have been added to this collection.
    /// @return Value between 0 and the maximum size for this collection.
    /// @see max_size().
    constexpr size_type size() const noexcept
    {
        return m_size;
    }

    /// @brief Gets the maximum size that this collection can be.
    /// @details This is the maximum number of elements that can be contained in this collection.
    constexpr size_type max_size() const noexcept
    {
        return MAXSIZE;
    }

    /// @brief Gets pointer to underlying data array.
    pointer data() noexcept
    {
        return m_elements.data();
    }

    /// @brief Gets pointer to underlying data array.
    const_pointer data() const noexcept
    {
        return m_elements.data();
    }

    iterator begin() noexcept
    {
        return data();
    }

    iterator end() noexcept
    {
        return data() + size();
    }

    const_iterator begin() const noexcept
    {
        return data();
    }

    const_iterator end() const noexcept
    {
        return data() + size();
    }

private:
    size_type m_size = size_type{0};
    std::array<value_type, MAXSIZE> m_elements = {};
};

/// @brief <code>ArrayList</code> append operator.
template <typename T, std::size_t S>
ArrayList<T, S>& operator+=(ArrayList<T, S>& lhs, const typename ArrayList<T, S>::data_type& rhs)
{
    lhs.push_back(rhs);
    return lhs;
}

/// @brief <code>ArrayList</code> add operator.
template <typename T, std::size_t S>
ArrayList<T, S> operator+(ArrayList<T, S> lhs, const typename ArrayList<T, S>::data_type& rhs)
{
    lhs.push_back(rhs);
    return lhs;
}

} /* namespace playrho */

/// Tuple size specialization for <code>ArrayList</code> classes.
template <class T, std::size_t N, typename SIZE_TYPE>
class std::tuple_size<playrho::ArrayList<T, N, SIZE_TYPE>> : public integral_constant<std::size_t, N>
{
    // Intentionally empty.
};

#endif // PLAYRHO_ARRAYLIST_HPP
