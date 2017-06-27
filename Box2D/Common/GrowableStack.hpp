/*
 * Original work Copyright (c) 2010 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

#ifndef B2_GROWABLE_STACK_H
#define B2_GROWABLE_STACK_H
#include <Box2D/Common/Settings.hpp>
#include <cstring>

namespace box2d {

/// This is a growable LIFO stack with an initial capacity of N.
/// If the stack size exceeds the initial capacity, the heap is used
/// to increase the size of the stack.
template <typename T, std::size_t N>
class GrowableStack
{
public:
    using size_type = std::size_t;
    static constexpr auto BufferGrowthRate = size_type{2};

    GrowableStack() = default;

    ~GrowableStack() noexcept
    {
        if (m_stack != m_array)
        {
            Free(m_stack);
            m_stack = nullptr;
        }
    }

    void Push(const T& element)
    {
        if (m_count == m_capacity)
        {
            T* old = m_stack;
            m_capacity *= BufferGrowthRate;
            m_stack = Alloc<T>(m_capacity);
            std::memcpy(m_stack, old, m_count * sizeof(T));
            if (old != m_array)
            {
                Free(old);
            }
        }

        m_stack[m_count] = element;
        ++m_count;
    }

    T Pop()
    {
        assert(m_count > 0);
        --m_count;
        return m_stack[m_count];
    }

    constexpr size_type GetCount() const noexcept
    {
        return m_count;
    }
    
    constexpr bool Empty() const noexcept
    {
        return m_count == 0;
    }

private:
    T m_array[N];
    T* m_stack = m_array;
    size_type m_count = 0;
    size_type m_capacity = N;
};

} /* namespace box2d */

#endif
