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

#ifndef PLAYRHO_COMMON_VECTOR_HPP
#define PLAYRHO_COMMON_VECTOR_HPP

#include <cassert>
#include <cstddef>
#include <type_traits>
#include <iterator>
#include <algorithm>
#include <functional>
#include <iostream>
#include <PlayRho/Common/InvalidArgument.hpp>
#include <PlayRho/Common/Real.hpp>
#include <PlayRho/Common/Templates.hpp>

namespace playrho
{

/// @brief Vector.
/// @details This is a <code>constexpr</code> and constructor enhanced
///   <code>std::array</code>-like template class for types supporting the +, -, *, /
///   arithmetic operators ("arithmetic types" as defined by the <code>IsArithmetic</code>
///   type trait) that itself comes with non-member arithmetic operator support making
///   Vector instances arithmetic types as well.
/// @note This type is trivially default constructible - i.e. default construction
///   performs no actions (no initialization).
/// @sa IsArithmetic
template <typename T, std::size_t N>
struct Vector
{
    static_assert(N > 0, "Number of elements must be greater than 0");
    static_assert(IsArithmetic<T>::value, "Type must be arithmetic");

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
    
    /// @brief Reverse iterator type.
    using reverse_iterator = std::reverse_iterator<iterator>;
    
    /// @brief Constant reverse iterator type.
    using const_reverse_iterator = std::reverse_iterator<const_iterator>;
    
    /// @brief Default constructor.
    /// @note Defaulted explicitly.
    /// @note This constructor performs no action.
    constexpr Vector() = default;
    
    /// @brief Initializing constructor.
    template<typename... Tail>
    constexpr explicit Vector(typename std::enable_if<sizeof...(Tail)+1 == N, T>::type
                     head, Tail... tail) noexcept: elements{head, T(tail)...}
    {
        // Intentionally empty.
    }

    /// @brief Gets the max size.
    constexpr size_type max_size() const noexcept { return N; }
    
    /// @brief Gets the size.
    constexpr size_type size() const noexcept { return N; }
    
    /// @brief Whether empty.
    /// @note Always false for N > 0.
    constexpr bool empty() const noexcept { return N == 0; }
    
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

    /// @brief Gets a reverse "begin" iterator.
    reverse_iterator rbegin() noexcept { return reverse_iterator{elements + N}; }

    /// @brief Gets a reverse "end" iterator.
    reverse_iterator rend() noexcept { return reverse_iterator{elements}; }
    
    /// @brief Gets a reverse "begin" iterator.
    const_reverse_iterator crbegin() const noexcept
    {
        return const_reverse_iterator{elements + N};
    }
    
    /// @brief Gets a reverse "end" iterator.
    const_reverse_iterator crend() const noexcept
    {
        return const_reverse_iterator{elements};
    }

    /// @brief Gets a reverse "begin" iterator.
    const_reverse_iterator rbegin() const noexcept
    {
        return crbegin();
    }
    
    /// @brief Gets a reverse "end" iterator.
    const_reverse_iterator rend() const noexcept
    {
        return crend();
    }

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

/// @brief Equality operator.
/// @relatedalso Vector
template <typename T, std::size_t N>
constexpr bool operator== (const Vector<T, N>& lhs, const Vector<T, N>& rhs) noexcept
{
    for (auto i = static_cast<size_t>(0); i < N; ++i)
    {
        if (lhs[i] != rhs[i])
        {
            return false;
        }
    }
    return true;
}

/// @brief Inequality operator.
/// @relatedalso Vector
template <typename T, std::size_t N>
constexpr bool operator!= (const Vector<T, N>& lhs, const Vector<T, N>& rhs) noexcept
{
    return !(lhs == rhs);
}

/// @brief Unary plus operator.
/// @relatedalso Vector
template <typename T, std::size_t N>
constexpr auto operator+ (Vector<T, N> v) noexcept
{
    return v;
}

/// @brief Unary negation operator.
/// @relatedalso Vector
template <typename T, std::size_t N>
constexpr auto operator- (Vector<T, N> v) noexcept
{
    for (auto i = static_cast<size_t>(0); i < N; ++i)
    {
        v[i] = -v[i];
    }
    return v;
}

/// @brief Increments the left hand side value by the right hand side value.
/// @relatedalso Vector
template <typename T, std::size_t N>
constexpr auto& operator+= (Vector<T, N>& lhs, const Vector<T, N> rhs) noexcept
{
    for (auto i = static_cast<size_t>(0); i < N; ++i)
    {
        lhs[i] += rhs[i];
    }
    return lhs;
}

/// @brief Decrements the left hand side value by the right hand side value.
/// @relatedalso Vector
template <typename T, std::size_t N>
constexpr auto& operator-= (Vector<T, N>& lhs, const Vector<T, N> rhs) noexcept
{
    for (auto i = static_cast<size_t>(0); i < N; ++i)
    {
        lhs[i] -= rhs[i];
    }
    return lhs;
}

/// @brief Adds two vectors component-wise.
/// @relatedalso Vector
template <typename T, std::size_t N>
constexpr auto operator+ (Vector<T, N> lhs, const Vector<T, N> rhs) noexcept
{
    return lhs += rhs;
}

/// @brief Subtracts two vectors component-wise.
/// @relatedalso Vector
template <typename T, std::size_t N>
constexpr auto operator- (Vector<T, N> lhs, const Vector<T, N> rhs) noexcept
{
    return lhs -= rhs;
}

/// @brief Multiplication assignment operator.
/// @relatedalso Vector
template <typename T, std::size_t N>
constexpr auto& operator*= (Vector<T, N>& lhs, const Real rhs) noexcept
{
    for (auto i = static_cast<size_t>(0); i < N; ++i)
    {
        lhs[i] *= rhs;
    }
    return lhs;
}

/// @brief Division assignment operator.
/// @relatedalso Vector
template <typename T, std::size_t N>
constexpr auto& operator/= (Vector<T, N>& lhs, const Real rhs) noexcept
{
    for (auto i = static_cast<size_t>(0); i < N; ++i)
    {
        lhs[i] /= rhs;
    }
    return lhs;
}

/// @brief Multiplication operator.
/// @relatedalso Vector
template <std::size_t N, typename T1, typename T2, typename OT = decltype(T1{} * T2{})>
constexpr auto operator* (const T1 s, const Vector<T2, N>& a) noexcept
{
    auto result = Vector<OT, N>{};
    for (auto i = static_cast<size_t>(0); i < N; ++i)
    {
        result[i] = a[i] * s;
    }
    return result;
}

/// @brief Multiplication operator.
/// @relatedalso Vector
template <std::size_t N, typename T1, typename T2, typename OT = decltype(T1{} * T2{})>
constexpr auto operator* (const Vector<T1, N>& a, const T2 s) noexcept
{
    auto result = Vector<OT, N>{};
    for (auto i = static_cast<size_t>(0); i < N; ++i)
    {
        result[i] = a[i] * s;
    }
    return result;
}

/// @brief Division operator.
/// @relatedalso Vector
template <std::size_t N, typename T1, typename T2, typename OT = decltype(T1{} / T2{})>
constexpr auto operator/ (const Vector<T1, N>& a, const T2 s) noexcept
{
    auto result = Vector<OT, N>{};
    for (auto i = static_cast<size_t>(0); i < N; ++i)
    {
        result[i] = a[i] / s;
    }
    return result;
}

/// @brief Multiplication operator.
/// @relatedalso Vector
template <std::size_t N, typename T1, typename T2, typename OT = decltype(T1{} * T2{})>
constexpr auto operator* (const Vector<T1, N>& lhs, const Vector<T2, N>& rhs) noexcept
{
    auto result = Vector<OT, N>{};
    for (auto i = static_cast<size_t>(0); i < N; ++i)
    {
        result[i] = lhs[i] * rhs[i];
    }
    return result;
}

/// @brief Division operator.
/// @relatedalso Vector
template <std::size_t N, typename T1, typename T2, typename OT = decltype(T1{} / T2{})>
constexpr auto operator/ (const Vector<T1, N>& lhs, const Vector<T2, N>& rhs) noexcept
{
    auto result = Vector<OT, N>{};
    for (auto i = static_cast<size_t>(0); i < N; ++i)
    {
        result[i] = lhs[i] / rhs[i];
    }
    return result;
}

/// @brief Lexicographical less-than operator.
/// @relatedalso Vector
template <typename T, std::size_t N>
constexpr bool operator< (const Vector<T, N>& lhs, const Vector<T, N>& rhs) noexcept
{
    return std::lexicographical_compare(lhs.cbegin(), lhs.cend(), rhs.cbegin(), rhs.cend(),
                                        std::less<T>{});
}

/// @brief Lexicographical less-than or equal-to operator.
/// @relatedalso Vector
template <typename T, std::size_t N>
constexpr bool operator<= (const Vector<T, N>& lhs, const Vector<T, N>& rhs) noexcept
{
    return std::lexicographical_compare(lhs.cbegin(), lhs.cend(), rhs.cbegin(), rhs.cend(),
                                        std::less_equal<T>{});
}

/// @brief Lexicographical greater-than operator.
/// @relatedalso Vector
template <typename T, std::size_t N>
constexpr bool operator> (const Vector<T, N>& lhs, const Vector<T, N>& rhs) noexcept
{
    return std::lexicographical_compare(lhs.cbegin(), lhs.cend(), rhs.cbegin(), rhs.cend(),
                                        std::greater<T>{});
}

/// @brief Lexicographical greater-than or equal-to operator.
/// @relatedalso Vector
template <typename T, std::size_t N>
constexpr bool operator>= (const Vector<T, N>& lhs, const Vector<T, N>& rhs) noexcept
{
    return std::lexicographical_compare(lhs.cbegin(), lhs.cend(), rhs.cbegin(), rhs.cend(),
                                        std::greater_equal<T>{});
}

/// @brief Gets the I'th element of the given collection.
/// @relatedalso Vector
template <size_t I, size_t N, typename T>
constexpr auto& Get(Vector<T, N>& v) noexcept
{
    static_assert(I < N, "Index out of bounds in playrho::Get<> (playrho::Vector)");
    return v[I];
}

/// @brief Gets the I'th element of the given collection.
template <size_t I, size_t N, typename T>
constexpr auto Get(const Vector<T, N>& v) noexcept
{
    static_assert(I < N, "Index out of bounds in playrho::Get<> (playrho::Vector)");
    return v[I];
}

/// @brief Output stream operator.
/// @relatedalso Vector
template <typename T, std::size_t N>
::std::ostream& operator<< (::std::ostream& os, const Vector<T, N>& value)
{
    os << "{";
    for (auto i = static_cast<size_t>(0); i < N; ++i)
    {
        if (i > static_cast<size_t>(0))
        {
            os << ',';
        }
        os << value[i];
    }
    os << "}";
    return os;
}

} // namespace playrho

#endif // PLAYRHO_COMMON_VECTOR_HPP
