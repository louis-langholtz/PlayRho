/*
 * Based on work by Jonathan Boccara and Jonathan Müller.
 * Modified work Copyright (c) 2020 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#ifndef PLAYRHO_COMMON_STRONGTYPE_HPP
#define PLAYRHO_COMMON_STRONGTYPE_HPP

#include <utility>
#include <functional> // for std::hash
#include <type_traits> // for std::is_nothrow_default_constructible

namespace playrho {
namespace strongtype {

/// @brief Named "strong type" template class.
/// @details A template class for wrapping types into more special-purposed types. Wrapping
///   types this way is often referred to as more "strongly typing" the underlying type.
/// @note This comes from pulling together code found from various sites on the Internet.
/// @see https://www.fluentcpp.com/2016/12/08/strong-types-for-strong-interfaces/
/// @see https://foonathan.net/blog/2016/10/19/strong-typedefs.html
/// @code{.cpp}
/// using Width = NamedType<double, struct WidthParameter>;
/// @endcode
template <typename T, typename Tag>
class NamedType
{
public:
    using underlying_type = T;

    /// @brief Default constructor.
    /// @note This causes default initialization of the underlying type.
    constexpr explicit NamedType()
    noexcept(std::is_nothrow_default_constructible<underlying_type>::value): value_{} {}

    /// @brief Copy constructor.
    constexpr explicit NamedType(const underlying_type& value)
    noexcept(std::is_nothrow_copy_constructible<underlying_type>::value): value_(value) {}

    /// @brief Move constructor.
    constexpr explicit NamedType(underlying_type&& value)
    noexcept(std::is_nothrow_move_constructible<underlying_type>::value):
        value_(std::move(value)) {}

    /// @brief Underlying type cast operator support.
    constexpr explicit operator underlying_type&() noexcept
    {
        return value_;
    }

    /// @brief Underlying type cast operator support.
    constexpr explicit operator const underlying_type&() const noexcept
    {
        return value_;
    }

    /// @brief Swap function.
    friend void swap(NamedType& a, NamedType& b) noexcept
    {
        using std::swap;
        swap(static_cast<underlying_type&>(a), static_cast<underlying_type&>(b));
    }

private:
    underlying_type value_; ///< Underlying value.
};

template <typename T, typename Tag>
constexpr T UnderlyingTypeImpl(NamedType<T, Tag>);

template <typename T>
using UnderlyingType = decltype(UnderlyingTypeImpl(std::declval<T>()));

/// @brief Gets the underlying value.
template <typename T, typename Tag>
constexpr T& UnderlyingValue(NamedType<T, Tag>& o) noexcept
{
    return static_cast<T&>(o);
}

/// @brief Gets the underlying value.
template <typename T, typename Tag>
constexpr const T& UnderlyingValue(const NamedType<T, Tag>& o) noexcept
{
    return static_cast<const T&>(o);
}

/// @brief Equality comparable type.
template <class NamedType>
struct EqualityComparable
{
    /// @brief Equality operator.
    friend constexpr bool operator== (const NamedType& lhs, const NamedType& rhs)
    {
        using type = UnderlyingType<NamedType>;
        return static_cast<type>(lhs) == static_cast<type>(rhs);
    }
};

/// @brief Inequality comparable type.
template <class NamedType>
struct InequalityComparable
{
    /// @brief Inequality operator.
    friend constexpr bool operator!= (const NamedType& lhs, const NamedType& rhs)
    {
        using type = UnderlyingType<NamedType>;
        return static_cast<type>(lhs) != static_cast<type>(rhs);
    }
};

/// @brief Less-than comparable type.
template <class NamedType>
struct LessThanComparable
{
    /// @brief Less-than operator.
    friend constexpr bool operator< (const NamedType& lhs, const NamedType& rhs)
    {
        using type = UnderlyingType<NamedType>;
        return static_cast<type>(lhs) < static_cast<type>(rhs);
    }
};

/// @brief Greater-than comparable type.
template <class NamedType>
struct GreaterThanComparable
{
    /// @brief Greater-than operator.
    friend constexpr bool operator> (const NamedType& lhs, const NamedType& rhs)
    {
        using type = UnderlyingType<NamedType>;
        return static_cast<type>(lhs) > static_cast<type>(rhs);
    }
};

/// @brief Less-than-or-equal-to comparable type.
template <class NamedType>
struct LessThanEqualComparable
{
    /// @brief Less-than-or-equal-to operator.
    friend constexpr bool operator<= (const NamedType& lhs, const NamedType& rhs)
    {
        using type = UnderlyingType<NamedType>;
        return static_cast<type>(lhs) <= static_cast<type>(rhs);
    }
};

/// @brief Greater-than-or-equal-to comparable type.
template <class NamedType>
struct GreaterThanEqualComparable
{
    /// @brief Greater-than-or-equal-to operator.
    friend constexpr bool operator>= (const NamedType& lhs, const NamedType& rhs)
    {
        using type = UnderlyingType<NamedType>;
        return static_cast<type>(lhs) >= static_cast<type>(rhs);
    }
};

template <class NamedType>
struct Hashable
{
    using type = UnderlyingType<NamedType>;

    friend ::std::size_t hash(const NamedType& v) noexcept
    {
        return ::std::hash<type>(UnderlyingValue(v));
    }
};

// Composite strong types...

template <typename T, typename Tag>
struct IdentifyingNamedType: NamedType<T, Tag>,
    EqualityComparable<NamedType<T, Tag>>, InequalityComparable<NamedType<T, Tag>>,
    Hashable<NamedType<T, Tag>>
{
    using NamedType<T, Tag>::NamedType;
};

template <typename T, typename Tag>
struct IndexingNamedType:
    NamedType<T, Tag>,
    EqualityComparable<NamedType<T, Tag>>, InequalityComparable<NamedType<T, Tag>>,
    LessThanComparable<NamedType<T, Tag>>, GreaterThanComparable<NamedType<T, Tag>>,
    LessThanEqualComparable<NamedType<T, Tag>>, GreaterThanEqualComparable<NamedType<T, Tag>>,
    Hashable<NamedType<T, Tag>>
{
    using NamedType<T, Tag>::NamedType;
};

} // namespace strongtype
} // namespace playrho

namespace std {

/// @brief Custom specialization of std::hash for <code>::playrho::strongtype::NamedType</code>.
template <typename T, typename Tag>
struct hash<::playrho::strongtype::IdentifyingNamedType<T, Tag>>
{
    ::std::size_t operator()(const ::playrho::strongtype::IdentifyingNamedType<T, Tag>& v) const noexcept
    {
        using type = ::playrho::strongtype::UnderlyingType<::playrho::strongtype::IdentifyingNamedType<T, Tag>>;
        return ::std::hash<type>()(::playrho::strongtype::UnderlyingValue(v));;
    }
};

/// @brief Custom specialization of std::hash for <code>::playrho::strongtype::NamedType</code>.
template <typename T, typename Tag>
struct hash<::playrho::strongtype::IndexingNamedType<T, Tag>>
{
    ::std::size_t operator()(const ::playrho::strongtype::IndexingNamedType<T, Tag>& v) const noexcept
    {
        using type = ::playrho::strongtype::UnderlyingType<::playrho::strongtype::IndexingNamedType<T, Tag>>;
        return ::std::hash<type>()(::playrho::strongtype::UnderlyingValue(v));;
    }
};

} // namespace std

#endif // PLAYRHO_COMMON_STRONGTYPE_HPP
