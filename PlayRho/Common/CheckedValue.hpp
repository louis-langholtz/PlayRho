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

#ifndef PLAYRHO_COMMON_CHECKEDVALUE_HPP
#define PLAYRHO_COMMON_CHECKEDVALUE_HPP

#include <PlayRho/Common/Templates.hpp>

#include <cmath>
#include <limits>
#include <type_traits>
#include <iostream>
#include <utility>

namespace playrho {

namespace detail {

template <typename T, typename=void>
struct CheckExceptionType{};

template <typename T>
struct CheckExceptionType<T, detail::VoidT<typename T::exception_type>> {
    using exception_type = typename T::exception_type;
};

} // namespace detail

/// @brief Checked value.
/// @note Prefer to use this type instead of <code>BoundedValue</code>.
/// @see BoundedValue.
template <typename ValueType, typename CheckerType>
class CheckedValue: public detail::CheckExceptionType<CheckerType>
{
public:
    static_assert(HasUnaryFunctor<CheckerType, ValueType, ValueType>::value,
                  "Checker type doesn't provide acceptable unary functor!");

    /// @brief Value type.
    using value_type = ValueType;

    /// @brief Remove pointer type.
    using remove_pointer_type = typename std::remove_pointer<ValueType>::type;

    /// @brief Checker type.
    using checker_type = CheckerType;

    /// Default constructor available for checker types with acceptable nullary functors.
    template <bool B = HasNullaryFunctor<CheckerType,ValueType>::value, typename std::enable_if_t<B, int> = 0>
    constexpr CheckedValue() noexcept(noexcept(CheckerType{}())): m_value{CheckerType{}()}
    {
    }

    /// @brief Initializing constructor.
    constexpr CheckedValue(value_type value) noexcept(noexcept(checker_type{}(value))):
        m_value{CheckerType{}(value)}
    {
    }

    /// @brief Gets the underlying value.
    constexpr value_type get() const noexcept
    {
        return m_value;
    }

    /// @brief Gets the underlying value.
    /// @todo Mark this function "explicit".
    constexpr operator value_type () const noexcept
    {
        return m_value;
    }

    /// @brief Member of pointer operator available for pointer <code>ValueType</code>.
    template <typename U = ValueType>
    constexpr std::enable_if_t<std::is_pointer<U>::value, U> operator-> () const
    {
        return m_value;
    }

    /// @brief Indirection operator available for pointer <code>ValueType</code>.
    template <typename U = ValueType>
    constexpr std::enable_if_t<std::is_pointer<U>::value, remove_pointer_type>&
    operator* () const
    {
        return *m_value;
    }

private:
    value_type m_value; ///< Underlying value.
};

// Common operations.

/// @brief Constrained value stream output operator.
template <typename ValueType, typename CheckerType>
auto operator<<(::std::ostream& os, const CheckedValue<ValueType, CheckerType>& value) ->
    decltype(os << ValueType{value})
{
    return os << ValueType{value};
}

/// @brief Constrained value equality operator.
template <typename LhsValueType, typename LhsCheckerType, typename RhsValueType, typename RhsCheckerType>
constexpr auto operator== (const CheckedValue<LhsValueType, LhsCheckerType>& lhs,
                           const CheckedValue<RhsValueType, RhsCheckerType>& rhs)
-> decltype(LhsValueType{lhs} == RhsValueType{rhs})
{
    return LhsValueType{lhs} == RhsValueType{rhs};
}

/// @brief Constrained value greater-than or equal-to operator.
template <typename ValueType, typename CheckerType, typename Other>
constexpr auto operator== (const CheckedValue<ValueType, CheckerType>& lhs,
                           const Other& rhs)
-> decltype(ValueType{lhs} == rhs)
{
    return ValueType{lhs} == rhs;
}

/// @brief Constrained value greater-than or equal-to operator.
template <typename ValueType, typename CheckerType, typename Other>
constexpr auto operator== (const Other& lhs,
                           const CheckedValue<ValueType, CheckerType>& rhs)
-> decltype(lhs == ValueType{rhs})
{
    return lhs == ValueType{rhs};
}

/// @brief Constrained value inequality operator.
template <typename LhsValueType, typename LhsCheckerType, typename RhsValueType, typename RhsCheckerType>
constexpr auto operator!= (const CheckedValue<LhsValueType, LhsCheckerType>& lhs,
                           const CheckedValue<RhsValueType, RhsCheckerType>& rhs)
-> decltype(LhsValueType{lhs} != RhsValueType{rhs})
{
    return LhsValueType{lhs} != RhsValueType{rhs};
}

/// @brief Constrained value greater-than or equal-to operator.
template <typename ValueType, typename CheckerType, typename Other>
constexpr auto operator!= (const CheckedValue<ValueType, CheckerType>& lhs,
                           const Other& rhs)
-> decltype(ValueType{lhs} != rhs)
{
    return ValueType{lhs} != rhs;
}

/// @brief Constrained value greater-than or equal-to operator.
template <typename ValueType, typename CheckerType, typename Other>
constexpr auto operator!= (const Other& lhs,
                           const CheckedValue<ValueType, CheckerType>& rhs)
-> decltype(lhs != ValueType{rhs})
{
    return lhs != ValueType{rhs};
}

/// @brief Constrained value less-than or equal-to operator.
template <typename LhsValueType, typename LhsCheckerType, typename RhsValueType, typename RhsCheckerType>
constexpr auto operator<= (const CheckedValue<LhsValueType, LhsCheckerType>& lhs,
                           const CheckedValue<RhsValueType, RhsCheckerType>& rhs)
-> decltype(LhsValueType{lhs} <= RhsValueType{rhs})
{
    return LhsValueType{lhs} <= RhsValueType{rhs};
}

/// @brief Constrained value greater-than or equal-to operator.
template <typename ValueType, typename CheckerType, typename Other>
constexpr auto operator<= (const CheckedValue<ValueType, CheckerType>& lhs,
                           const Other& rhs)
-> decltype(ValueType{lhs} <= rhs)
{
    return ValueType{lhs} <= rhs;
}

/// @brief Constrained value greater-than or equal-to operator.
template <typename ValueType, typename CheckerType, typename Other>
constexpr auto operator<= (const Other& lhs,
                           const CheckedValue<ValueType, CheckerType>& rhs)
-> decltype(lhs <= ValueType{rhs})
{
    return lhs <= ValueType{rhs};
}

/// @brief Constrained value greater-than or equal-to operator.
template <typename LhsValueType, typename LhsCheckerType, typename RhsValueType, typename RhsCheckerType>
constexpr auto operator>= (const CheckedValue<LhsValueType, LhsCheckerType>& lhs,
                           const CheckedValue<RhsValueType, RhsCheckerType>& rhs)
-> decltype(LhsValueType{lhs} >= RhsValueType{rhs})
{
    return LhsValueType{lhs} >= RhsValueType{rhs};
}

/// @brief Constrained value greater-than or equal-to operator.
template <typename ValueType, typename CheckerType, typename Other>
constexpr auto operator>= (const CheckedValue<ValueType, CheckerType>& lhs,
                           const Other& rhs)
-> decltype(ValueType{lhs} >= rhs)
{
    return ValueType{lhs} >= rhs;
}

/// @brief Constrained value greater-than or equal-to operator.
template <typename ValueType, typename CheckerType, typename Other>
constexpr auto operator>= (const Other& lhs,
                           const CheckedValue<ValueType, CheckerType>& rhs)
-> decltype(lhs >= ValueType{rhs})
{
    return lhs >= ValueType{rhs};
}

/// @brief Constrained value less-than operator.
template <typename LhsValueType, typename LhsCheckerType, typename RhsValueType, typename RhsCheckerType>
constexpr auto operator< (const CheckedValue<LhsValueType, LhsCheckerType>& lhs,
                          const CheckedValue<RhsValueType, RhsCheckerType>& rhs)
-> decltype(LhsValueType{lhs} < RhsValueType{rhs})
{
    return LhsValueType{lhs} < RhsValueType{rhs};
}

/// @brief Constrained value greater-than or equal-to operator.
template <typename ValueType, typename CheckerType, typename Other>
constexpr auto operator< (const CheckedValue<ValueType, CheckerType>& lhs,
                          const Other& rhs)
-> decltype(ValueType{lhs} < rhs)
{
    return ValueType{lhs} < rhs;
}

/// @brief Constrained value greater-than or equal-to operator.
template <typename ValueType, typename CheckerType, typename Other>
constexpr auto operator< (const Other& lhs,
                          const CheckedValue<ValueType, CheckerType>& rhs)
-> decltype(lhs < ValueType{rhs})
{
    return lhs < ValueType{rhs};
}

/// @brief Constrained value greater-than operator.
template <typename LhsValueType, typename LhsCheckerType, typename RhsValueType, typename RhsCheckerType>
constexpr auto operator> (const CheckedValue<LhsValueType, LhsCheckerType>& lhs,
                          const CheckedValue<RhsValueType, RhsCheckerType>& rhs)
-> decltype(LhsValueType{lhs} > RhsValueType{rhs})
{
    return LhsValueType{lhs} > RhsValueType{rhs};
}

/// @brief Constrained value greater-than or equal-to operator.
template <typename ValueType, typename CheckerType, typename Other>
constexpr auto operator> (const CheckedValue<ValueType, CheckerType>& lhs,
                          const Other& rhs)
-> decltype(ValueType{lhs} > rhs)
{
    return ValueType{lhs} > rhs;
}

/// @brief Constrained value greater-than or equal-to operator.
template <typename ValueType, typename CheckerType, typename Other>
constexpr auto operator> (const Other& lhs,
                          const CheckedValue<ValueType, CheckerType>& rhs)
-> decltype(lhs > ValueType{rhs})
{
    return lhs > ValueType{rhs};
}

/// @brief Constrained value multiplication operator.
template <typename LhsValueType, typename LhsCheckerType, typename RhsValueType, typename RhsCheckerType>
constexpr auto operator* (const CheckedValue<LhsValueType, LhsCheckerType>& lhs,
                          const CheckedValue<RhsValueType, RhsCheckerType>& rhs)
-> decltype(LhsValueType{lhs} * RhsValueType{rhs})
{
    return LhsValueType{lhs} * RhsValueType{rhs};
}

/// @brief Constrained value multiplication operator.
template <typename ValueType, typename CheckerType, typename Other>
constexpr std::enable_if_t<
    !IsMultipliable<CheckedValue<ValueType, CheckerType>, Other>::value &&
    IsMultipliable<ValueType, Other>::value,
    decltype(ValueType{}*Other{})>
operator* (const CheckedValue<ValueType, CheckerType>& lhs, const Other& rhs)
{
    return ValueType{lhs} * rhs;
}

/// @brief Constrained value multiplication operator.
template <typename ValueType, typename CheckerType, typename Other>
constexpr std::enable_if_t<
    !IsMultipliable<Other, CheckedValue<ValueType, CheckerType>>::value &&
    IsMultipliable<Other, ValueType>::value,
    decltype(Other{}*ValueType{})>
operator* (const Other& lhs, const CheckedValue<ValueType, CheckerType>& rhs)
{
    return lhs * ValueType{rhs};
}

/// @brief Constrained value division operator.
template <typename LhsValueType, typename LhsCheckerType, typename RhsValueType, typename RhsCheckerType>
constexpr auto operator/ (const CheckedValue<LhsValueType, LhsCheckerType>& lhs,
                          const CheckedValue<RhsValueType, RhsCheckerType>& rhs)
-> decltype(LhsValueType{lhs} / RhsValueType{rhs})
{
    return LhsValueType{lhs} / RhsValueType{rhs};
}

/// @brief Constrained value division operator.
template <typename ValueType, typename CheckerType, typename Other>
constexpr auto operator/ (const CheckedValue<ValueType, CheckerType>& lhs,
                          const Other& rhs)
-> decltype(ValueType{lhs} / rhs)
{
    return ValueType{lhs} / rhs;
}

/// @brief Constrained value division operator.
template <typename ValueType, typename CheckerType, typename Other>
constexpr auto operator/ (const Other& lhs,
                          const CheckedValue<ValueType, CheckerType>& rhs)
-> decltype(lhs / ValueType{rhs})
{
    return lhs / ValueType{rhs};
}

/// @brief Constrained value addition operator.
template <typename LhsValueType, typename LhsCheckerType, typename RhsValueType, typename RhsCheckerType>
constexpr auto operator+ (const CheckedValue<LhsValueType, LhsCheckerType>& lhs,
                          const CheckedValue<RhsValueType, RhsCheckerType>& rhs)
-> decltype(LhsValueType{lhs} + RhsValueType{rhs})
{
    return LhsValueType{lhs} + RhsValueType{rhs};
}

/// @brief Constrained value addition operator.
template <typename ValueType, typename CheckerType, typename Other>
constexpr auto operator+ (const CheckedValue<ValueType, CheckerType>& lhs,
                          const Other& rhs)
-> decltype(ValueType{lhs} + rhs)
{
    return ValueType{lhs} + rhs;
}

/// @brief Constrained value addition operator.
template <typename ValueType, typename CheckerType, typename Other>
constexpr auto operator+ (const Other& lhs,
                          const CheckedValue<ValueType, CheckerType>& rhs)
-> decltype(lhs + ValueType{rhs})
{
    return lhs + ValueType{rhs};
}

/// @brief Constrained value subtraction operator.
template <typename LhsValueType, typename LhsCheckerType, typename RhsValueType, typename RhsCheckerType>
constexpr auto operator- (const CheckedValue<LhsValueType, LhsCheckerType>& lhs,
                          const CheckedValue<RhsValueType, RhsCheckerType>& rhs)
-> decltype(LhsValueType{lhs} - RhsValueType{rhs})
{
    return LhsValueType{lhs} - RhsValueType{rhs};
}

/// @brief Constrained value subtraction operator.
template <typename ValueType, typename CheckerType, typename Other>
constexpr auto operator- (const CheckedValue<ValueType, CheckerType>& lhs,
                          const Other& rhs)
-> decltype(ValueType{lhs} - rhs)
{
    return ValueType{lhs} - rhs;
}

/// @brief Constrained value subtraction operator.
template <typename ValueType, typename CheckerType, typename Other>
constexpr auto operator- (const Other& lhs,
                          const CheckedValue<ValueType, CheckerType>& rhs)
-> decltype(lhs - ValueType{rhs})
{
    return lhs - ValueType{rhs};
}

/// @defgroup CheckedValue Constrained Value Types
/// @brief Types for constrained values.
/// @details Type aliases for constrained values via on-construction checks that
///   throw the <code>InvalidArgument</code> exception if an attempt is made
///   to construct the constrained value type with a value not allowed by the specific
///   alias.
/// @sa CheckedValue, InvalidArgument
/// @{

/// @}

} // namespace playrho

#endif // PLAYRHO_COMMON_CHECKEDVALUE_HPP
