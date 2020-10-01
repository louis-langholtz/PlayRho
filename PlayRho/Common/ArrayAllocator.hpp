/*
 * Copyright (c) 2020 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#ifndef PLAYRHO_COMMON_ARRAYALLOCATOR_HPP
#define PLAYRHO_COMMON_ARRAYALLOCATOR_HPP

#include <utility>
#include <vector>
#include <type_traits>

namespace playrho {

/// @brief Array allocator.
template <typename T>
class ArrayAllocator
{
public:
    /// @brief Element type.
    using value_type = T;

    /// @brief Size type.
    using size_type = typename std::vector<value_type>::size_type;

    using reference = typename std::vector<value_type>::reference;

    using const_reference = typename std::vector<value_type>::const_reference;

    size_type GetIndex(value_type* ptr) const
    {
        const auto i = ptr - m_data.data();
        return static_cast<size_type>(((i >= 0) && (static_cast<size_type>(i) < m_data.size()))? i: -1);
    }

    template< class... Args >
    size_type Allocate(Args&&... args)
    {
        if (!m_free.empty())
        {
            const auto index = m_free.back();
            m_data[index] = value_type{std::forward<Args>(args)...};
            m_free.pop_back();
            return index;
        }
        const auto index = m_data.size();
        m_data.emplace_back(std::forward<Args>(args)...);
        return index;
    }

    size_type Allocate(const value_type& copy)
    {
        if (!m_free.empty())
        {
            const auto index = m_free.back();
            m_data[index] = copy;
            m_free.pop_back();
            return index;
        }
        const auto index = m_data.size();
        m_data.push_back(copy);
        return index;
    }

    void Free(size_type index)
    {
        if (index != static_cast<size_type>(-1))
        {
            m_data[index] = value_type{};
            m_free.push_back(index);
        }
    }

    reference operator[](size_type pos)
    {
        return m_data[pos];
    }

    const_reference operator[](size_type pos) const
    {
        return m_data[pos];
    }

    reference at(size_type pos)
    {
        return m_data.at(pos);
    }

    const_reference at(size_type pos) const
    {
        return m_data.at(pos);
    }

    size_type size() const noexcept
    {
        return m_data.size();
    }

    size_type max_size() const noexcept
    {
        return m_data.max_size();
    }

    size_type free() const noexcept
    {
        return m_free.size();
    }

    void reserve(size_type value)
    {
        m_data.reserve(value);
    }

private:
    std::vector<value_type> m_data;
    std::vector<typename std::vector<value_type>::size_type> m_free;
};

template <typename T>
typename ArrayAllocator<T>::size_type used(const ArrayAllocator<T>& array) noexcept
{
    return array.size() - array.free();
}

} // namespace playrho

#endif // PLAYRHO_COMMON_ARRAYALLOCATOR_HPP
