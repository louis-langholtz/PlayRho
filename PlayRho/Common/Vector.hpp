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
#include <type_traits>
#include <PlayRho/Common/InvalidArgument.hpp>

namespace playrho
{

/// @brief Vector.
/// @details Basically a constexpr and constructor enhanced std::array for C++14.
/// @note This type is trivially default constructible - i.e. default construction
///   performs no actions (no initialization).
/// @note This type should be drop-in replacable with std::array in C++17.
template <std::size_t N, typename T>
struct Vector
{
    static_assert(N > 0, "Dimension must be greater than 0");

    /// @brief Value type.
    using value_type = T;

    /// @brief Size type.
    using size_type = std::size_t;
    
    /// @brief Difference type.
    using difference_type = std::ptrdiff_t;
    
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
    /// @note Defaulted explicitly.
    /// @note This constructor performs no action.
    constexpr Vector() = default;
    
    /// @brief Initializing constructor.
    template<typename... Tail>
    constexpr Vector(typename std::enable_if<sizeof...(Tail)+1 == N, T>::type
                     head, Tail... tail) noexcept: elements{head, T(tail)...}
    {
        //static_assert(sizeof...(args) == N, "Invalid number of arguments");
    }

    /// @brief Gets the max size.
    constexpr size_type max_size() const noexcept { return N; }
    
    /// @brief Gets the size.
    constexpr size_type size() const noexcept { return N; }
    
    /// @brief Whether empty.
    /// @note Always false for N > 0.
    constexpr size_type empty() const noexcept { return N == 0; }
    
    /// @brief Gets a "begin" iterator.
    iterator begin() noexcept { return iterator(elements); }

    /// @brief Gets an "end" iterator.
    iterator end() noexcept { return iterator(elements + N); }
    
    /// @brief Gets a "begin" iterator.
    const_iterator begin() const noexcept { return const_iterator(elements); }
    
    /// @brief Gets an "end" iterator.
    const_iterator end() const noexcept { return const_iterator(elements + N); }
    
    /// @brief Gets a "begin" iterator.
    const_iterator cbegin() const noexcept { return begin(); }
    
    /// @brief Gets an "end" iterator.
    const_iterator cend() const noexcept { return end(); }

    /// @brief Gets a reference to the requested element.
    /// @note No bounds checking is performed.
    /// @warning Behavior is undefined if given a position equal to or greater than size().
    constexpr reference operator[](size_type pos) noexcept
    {
        assert(pos < size());
        return elements[pos];
    }

    
    /// @brief Gets a constant reference to the requested element.
    /// @note No bounds checking is performed.
    /// @warning Behavior is undefined if given a position equal to or greater than size().
    constexpr const_reference operator[](size_type pos) const noexcept
    {
        assert(pos < size());
        return elements[pos];
    }
    
    /// @brief Gets a reference to the requested element.
    /// @throws InvalidArgument if given a position that's >= size().
    constexpr reference at(size_type pos)
    {
        if (pos >= size())
        {
            throw InvalidArgument("Vector::at: position >= size()");
        }
        return elements[pos];
    }
    
    /// @brief Gets a constant reference to the requested element.
    /// @throws InvalidArgument if given a position that's >= size().
    constexpr const_reference at(size_type pos) const
    {
        if (pos >= size())
        {
            throw InvalidArgument("Vector::at: position >= size()");
        }
        return elements[pos];
    }
    
    /// @brief Direct access to data.
    constexpr pointer data() noexcept
    {
        return elements;
    }
    
    /// @brief Direct access to data.
    constexpr const_pointer data() const noexcept
    {
        return elements;
    }
    
    /// @brief Elements.
    /// @warning Don't access this directly!
    /// @warning Data is not initialized on default construction. This is intentional
    ///   to avoid any performance overhead that default initialization might incur.
    value_type elements[N];
};

/// @brief Gets the I'th element of the given collection.
template <size_t I, size_t N, typename T>
constexpr auto& Get(Vector<N, T>& v) noexcept
{
    static_assert(I < N, "Index out of bounds in playrho::Get<> (playrho::Vector)");
    return v[I];
}

/// @brief Gets the I'th element of the given collection.
template <size_t I, size_t N, typename T>
constexpr auto Get(const Vector<N, T>& v) noexcept
{
    static_assert(I < N, "Index out of bounds in playrho::Get<> (playrho::Vector)");
    return v[I];
}

} // namespace playrho

#endif /* Vector_hpp */
