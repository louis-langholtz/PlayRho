/*
 * Copyright (c) 2023 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#ifndef PLAYRHO_CHECKEDVALUE_HPP
#define PLAYRHO_CHECKEDVALUE_HPP

/// @file
/// @brief Declarations of the @c Checked class template and closely related code.

#include <iostream>
#include <type_traits>
#include <utility> // for std::declval

#include <playrho/InvalidArgument.hpp>
#include <playrho/Templates.hpp>

namespace playrho::detail {

/// @defgroup Checkers Checker Types
/// @brief Types for checking values.
/// @details Types for checking values for use in types like @c Checked.
///   Valid checkers must minimally provide a one-parameter functor taking
///   the value to be checked and returning @c nullptr if **valid** or a non-null
///   pointer to a C-style nul-terminated ASCII string in static storage
///   indicating why the given value is **invalid**.
///   Additionally, checkers can optionally provide a no-parameter functor returning
///   a default value for the type and that checker.
/// @note It's expected that these functors are non-throwing.
/// @note An example conforming checker is the <code>NoOpChecker</code>.
/// @see Checked, NoOpChecker.

/// @brief No-op value checker.
/// @details Provides functors for a no-operation style checker. This simply always
///   returns @c nullptr in the one-parameter value checking functor and returns the
///   template specified type's default value from the no-parameter functor.
/// @ingroup Checkers
/// @tparam T Value type to check.
/// @note This is meant to be used as a checker with types like <code>Checked</code>.
/// @see Checked.
template <class T>
struct NoOpChecker
{
    /// @brief Default value supplying functor.
    /// @return Always returns defaulted value of class template's given type parameter.
    constexpr auto operator()() noexcept -> decltype(T())
    {
        return T();
    }

    /// @brief Value checking functor.
    /// @return Always @c nullptr, in a conceivably no-operation style.
    constexpr auto operator()(const T&) noexcept -> const char *
    {
        return nullptr;
    }
};

/// @brief Class template for construction-time constraining a type's value.
/// @note Conceptually, this is to values what concepts are to types. One difference
///   however, is this - as a mechanism - operates at construction-time rather than
///   compile-time (like concepts do). From a design perspective, this can serve as
///   an efficient and scalable mechanism for defensive/offensive programming or
///   for pre/post conditions of contract oriented programming so long as the
///   value-type this is used with has only value semantics.
/// @invariant The value of an object of this type is always valid for the checker
///   of the type.
/// @tparam ValueType Type of the underlying value that will get checked. Note that
///   this is the only template parameter that effects the size of objects of this
///   class.
/// @tparam Checker Checker type to check or possibly default initialize values with.
///   See the @ref Checkers "Checkers" topic for more information on what checkers
///   are already available or how to design your own. Note that whatever the checker
///   is, it gets constructed only for single use of either its one parameter value
///   checking functor, or its zero parameter default value supplying functor.
/// @tparam NoExcept Whether to terminate or just throw on being invalid. The value
///   chosen is used for some member functions' @c noexcept specifier as
///   @c noexcept(NoExcept) . By using the value of @c true , those functions that
///   would have otherwise thrown an exception, become terminating functions in debug
///   builds or pass-through functions in non-debug builds. As a guide, use @c false
///   for types to validate data from outside the program (like from the user, as a
///   defensive programming mechanism), or @c true for types only validating data
///   from within the program (that should be valid, and as an offensive programming
///   mechanism).
/// @see https://en.cppreference.com/w/cpp/language/noexcept_spec.
/// @see https://en.cppreference.com/w/cpp/concepts/regular.
template <class ValueType, class Checker = NoOpChecker<ValueType>, bool NoExcept = false>
class Checked
{
public:
    static_assert(HasUnaryFunctor<Checker, bool, ValueType>::value,
                  "Checker type doesn't provide acceptable unary functor!");

    /// @brief Alias for the value type this class template was instantiated for.
    using value_type = ValueType;

    /// @brief Alias for the removed pointer type.
    /// @note This is the same as <code>value_type</code> unless it's actually
    ///   a pointer type in which case then this is the type pointed to.
    using remove_pointer_type = std::remove_pointer_t<ValueType>;

    /// @brief Alias for checker type this class template was instantiated for.
    using checker_type = Checker;

    /// @brief Alias for the exception type possibly thrown by this class.
    using exception_type = InvalidArgument;

    /// @brief Throws this class's exception type if the given value is invalid.
    /// @throws exception_type Constructed with the returned error - if the checker
    ///   returned a non-null explanatory string.
    /// @see exception_type.
    static constexpr auto ThrowIfInvalid(const value_type& value)
        -> decltype((void)exception_type(Checker{}(value)), std::declval<void>())
    {
        if (const auto error = Checker{}(value)) {
            throw exception_type(error);
        }
    }

    /// @brief Validator function.
    /// @details Validates the given value using the @c checker_type type and returns
    ///   the value given if it checks out.
    /// @pre @p value must be valid if @c NoExcept is true.
    /// @throws exception_type Constructed with the returned error - if the checker
    ///   returned a non-null explanatory string - and @c NoExcept is @c false.
    /// @return Value given.
    static constexpr auto Validate(const value_type& value) noexcept(NoExcept)
        -> decltype(ThrowIfInvalid(value), value_type{})
    {
        if constexpr (NoExcept) {
#ifndef NDEBUG // Only verify condition & add that overhead in debug builds!
            ThrowIfInvalid(value);
#endif
        }
        else {
            ThrowIfInvalid(value);
        }
        return value;
    }

    /// @brief Default constructor.
    /// @details Constructs a defaulted checked type if the checker type has a nullary
    ///   functor that returns a value which the
    ///   <code>Validate(const value_type& value)</code> function validates.
    /// @note This is only available for checker types with acceptable nullary functors.
    /// @post Calling <code>get()</code> or casting to the underlying type, results
    ///   in the value given by the checker's nullary functor.
    /// @see Validate.
    template <bool B = HasNullaryFunctor<Checker,ValueType>::value, std::enable_if_t<B, int> = 0>
    constexpr Checked() noexcept(NoExcept):
        m_value{Validate(Checker{}())}
    {
        // Intentionally empty.
    }

    /// @brief Initializing constructor.
    /// @details Constructs a checked type of the given value if the
    ///   <code>Validate(const value_type& value)</code> function validates.
    /// @post Calling <code>get()</code> or casting to the underlying type, results
    ///   in the value given.
    /// @todo Consider marking this function "explicit".
    /// @see Validate.
    constexpr Checked(value_type value) noexcept(NoExcept):
        m_value{Validate(value)}
    {
        // Intentionally empty.
    }

    /// @brief Copying constructor.
    /// @note This also supports direct non-checked implicit conversion between
    ///   @c Checked types that differ only in their non-type @c NoExcept template
    ///   paramaeter value.
    /// @post The constructed object compares equally to @c other .
    template <bool OtherNoExcept>
    constexpr Checked(const Checked<ValueType, Checker, OtherNoExcept>& other) noexcept:
        m_value{other.get()}
    {
        // Intentionally empty.
    }

    /// @brief Explicitly gets the underlying value.
    /// @note This can also be cast to the underlying value.
    constexpr value_type get() const noexcept
    {
        return m_value;
    }

    /// @brief Gets the underlying value via a cast or implicit conversion.
    /// @see get.
    /// @todo Consider marking this function "explicit".
    constexpr operator value_type () const noexcept
    {
        return m_value;
    }

    /// @brief Member-of pointer operator available for pointer <code>ValueType</code>.
    /// @note This is only available for pointer <code>ValueType</code>s.
    template <class U = ValueType>
    constexpr std::enable_if_t<std::is_pointer_v<U>, U> operator-> () const
    {
        return m_value;
    }

    /// @brief Indirection operator available for pointer <code>ValueType</code>.
    /// @note This is only available for pointer <code>ValueType</code>s.
    template <class U = ValueType>
    constexpr std::enable_if_t<std::is_pointer_v<U>, remove_pointer_type>&
    operator* () const
    {
        return *m_value;
    }

private:
    value_type m_value; ///< Underlying value.
};

// Common operations.

/// @brief Constrained value stream output operator for value types which support it.
/// @tparam ValueType Type of the value used by the checked value.
/// @tparam Checker Type of the checker used by the checked value.
/// @tparam NoExcept Whether the checked value terminates or just throws on being invalid.
/// @relatedalso Checked
template <class ValueType, class Checker, bool NoExcept>
auto operator<<(::std::ostream& os, const Checked<ValueType, Checker, NoExcept>& value) ->
    decltype(os << ValueType(value))
{
    return os << ValueType(value);
}

/// @brief Constrained value equality operator for value types which support it.
/// @tparam LhsValueType Type of the value used by the left hand side checked value.
/// @tparam LhsChecker Type of the checker used by the left hand side checked value.
/// @tparam LhsNoExcept Whether the left hand side checked value terminates or just throws on being invalid.
/// @tparam RhsValueType Type of the value used by the right hand side checked value.
/// @tparam RhsChecker Type of the checker used by the right hand side checked value.
/// @tparam RhsNoExcept Whether the right hand side checked value terminates or just throws on being invalid.
/// @relatedalso Checked
template <class LhsValueType, class LhsChecker, bool LhsNoExcept, // force newline
          class RhsValueType, class RhsChecker, bool RhsNoExcept>
constexpr auto operator== (const Checked<LhsValueType, LhsChecker, LhsNoExcept>& lhs,
                           const Checked<RhsValueType, RhsChecker, RhsNoExcept>& rhs)
    noexcept(noexcept(std::declval<LhsValueType>() == std::declval<RhsValueType>()))
-> decltype(LhsValueType(lhs) == RhsValueType(rhs))
{
    return LhsValueType(lhs) == RhsValueType(rhs);
}

/// @brief Constrained value inequality operator for value types which support it.
/// @tparam LhsValueType Type of the value used by the left hand side checked value.
/// @tparam LhsChecker Type of the checker used by the left hand side checked value.
/// @tparam LhsNoExcept Whether the left hand side checked value terminates or just throws on being invalid.
/// @tparam RhsValueType Type of the value used by the right hand side checked value.
/// @tparam RhsChecker Type of the checker used by the right hand side checked value.
/// @tparam RhsNoExcept Whether the right hand side checked value terminates or just throws on being invalid.
/// @relatedalso Checked
template <class LhsValueType, class LhsChecker, bool LhsNoExcept, // force newline
          class RhsValueType, class RhsChecker, bool RhsNoExcept>
constexpr auto operator!= (const Checked<LhsValueType, LhsChecker, LhsNoExcept>& lhs,
                           const Checked<RhsValueType, RhsChecker, RhsNoExcept>& rhs)
    noexcept(noexcept(std::declval<LhsValueType>() != std::declval<RhsValueType>()))
-> decltype(LhsValueType(lhs) != RhsValueType(rhs))
{
    return LhsValueType(lhs) != RhsValueType(rhs);
}

/// @brief Constrained value less-than or equal-to operator.
/// @tparam LhsValueType Type of the value used by the left hand side checked value.
/// @tparam LhsChecker Type of the checker used by the left hand side checked value.
/// @tparam LhsNoExcept Whether the left hand side checked value terminates or just throws on being invalid.
/// @tparam RhsValueType Type of the value used by the right hand side checked value.
/// @tparam RhsChecker Type of the checker used by the right hand side checked value.
/// @tparam RhsNoExcept Whether the right hand side checked value terminates or just throws on being invalid.
/// @relatedalso Checked
template <class LhsValueType, class LhsChecker, bool LhsNoExcept, // force newline
          class RhsValueType, class RhsChecker, bool RhsNoExcept>
constexpr auto operator<= (const Checked<LhsValueType, LhsChecker, LhsNoExcept>& lhs,
                           const Checked<RhsValueType, RhsChecker, RhsNoExcept>& rhs)
-> decltype(LhsValueType(lhs) <= RhsValueType(rhs))
{
    return LhsValueType(lhs) <= RhsValueType(rhs);
}

/// @brief Constrained value greater-than or equal-to operator.
/// @tparam LhsValueType Type of the value used by the left hand side checked value.
/// @tparam LhsChecker Type of the checker used by the left hand side checked value.
/// @tparam LhsNoExcept Whether the left hand side checked value terminates or just throws on being invalid.
/// @tparam RhsValueType Type of the value used by the right hand side checked value.
/// @tparam RhsChecker Type of the checker used by the right hand side checked value.
/// @tparam RhsNoExcept Whether the right hand side checked value terminates or just throws on being invalid.
/// @relatedalso Checked
template <class LhsValueType, class LhsChecker, bool LhsNoExcept, // force newline
          class RhsValueType, class RhsChecker, bool RhsNoExcept>
constexpr auto operator>= (const Checked<LhsValueType, LhsChecker, LhsNoExcept>& lhs,
                           const Checked<RhsValueType, RhsChecker, RhsNoExcept>& rhs)
-> decltype(LhsValueType(lhs) >= RhsValueType(rhs))
{
    return LhsValueType(lhs) >= RhsValueType(rhs);
}

/// @brief Constrained value less-than operator.
/// @tparam LhsValueType Type of the value used by the left hand side checked value.
/// @tparam LhsChecker Type of the checker used by the left hand side checked value.
/// @tparam LhsNoExcept Whether the left hand side checked value terminates or just throws on being invalid.
/// @tparam RhsValueType Type of the value used by the right hand side checked value.
/// @tparam RhsChecker Type of the checker used by the right hand side checked value.
/// @tparam RhsNoExcept Whether the right hand side checked value terminates or just throws on being invalid.
/// @relatedalso Checked
template <class LhsValueType, class LhsChecker, bool LhsNoExcept, // force newline
          class RhsValueType, class RhsChecker, bool RhsNoExcept>
constexpr auto operator< (const Checked<LhsValueType, LhsChecker, LhsNoExcept>& lhs,
                          const Checked<RhsValueType, RhsChecker, RhsNoExcept>& rhs)
-> decltype(LhsValueType(lhs) < RhsValueType(rhs))
{
    return LhsValueType(lhs) < RhsValueType(rhs);
}

/// @brief Constrained value greater-than operator.
/// @tparam LhsValueType Type of the value used by the left hand side checked value.
/// @tparam LhsChecker Type of the checker used by the left hand side checked value.
/// @tparam LhsNoExcept Whether the left hand side checked value terminates or just throws on being invalid.
/// @tparam RhsValueType Type of the value used by the right hand side checked value.
/// @tparam RhsChecker Type of the checker used by the right hand side checked value.
/// @tparam RhsNoExcept Whether the right hand side checked value terminates or just throws on being invalid.
/// @relatedalso Checked
template <class LhsValueType, class LhsChecker, bool LhsNoExcept, // force newline
          class RhsValueType, class RhsChecker, bool RhsNoExcept>
constexpr auto operator> (const Checked<LhsValueType, LhsChecker, LhsNoExcept>& lhs,
                          const Checked<RhsValueType, RhsChecker, RhsNoExcept>& rhs)
-> decltype(LhsValueType(lhs) > RhsValueType(rhs))
{
    return LhsValueType(lhs) > RhsValueType(rhs);
}

/// @brief Constrained value multiplication operator.
/// @tparam LhsValueType Type of the value used by the left hand side checked value.
/// @tparam LhsChecker Type of the checker used by the left hand side checked value.
/// @tparam LhsNoExcept Whether the left hand side checked value terminates or just throws on being invalid.
/// @tparam RhsValueType Type of the value used by the right hand side checked value.
/// @tparam RhsChecker Type of the checker used by the right hand side checked value.
/// @tparam RhsNoExcept Whether the right hand side checked value terminates or just throws on being invalid.
/// @relatedalso Checked
template <class LhsValueType, class LhsChecker, bool LhsNoExcept, // force newline
          class RhsValueType, class RhsChecker, bool RhsNoExcept>
constexpr auto operator* (const Checked<LhsValueType, LhsChecker, LhsNoExcept>& lhs,
                          const Checked<RhsValueType, RhsChecker, RhsNoExcept>& rhs)
-> decltype(LhsValueType(lhs) * RhsValueType(rhs))
{
    return LhsValueType(lhs) * RhsValueType(rhs);
}

/// @brief Constrained value division operator.
/// @tparam LhsValueType Type of the value used by the left hand side checked value.
/// @tparam LhsChecker Type of the checker used by the left hand side checked value.
/// @tparam LhsNoExcept Whether the left hand side checked value terminates or just throws on being invalid.
/// @tparam RhsValueType Type of the value used by the right hand side checked value.
/// @tparam RhsChecker Type of the checker used by the right hand side checked value.
/// @tparam RhsNoExcept Whether the right hand side checked value terminates or just throws on being invalid.
/// @relatedalso Checked
template <class LhsValueType, class LhsChecker, bool LhsNoExcept, // force newline
          class RhsValueType, class RhsChecker, bool RhsNoExcept>
constexpr auto operator/ (const Checked<LhsValueType, LhsChecker, LhsNoExcept>& lhs,
                          const Checked<RhsValueType, RhsChecker, RhsNoExcept>& rhs)
-> decltype(LhsValueType(lhs) / RhsValueType(rhs))
{
    return LhsValueType(lhs) / RhsValueType(rhs);
}

/// @brief Constrained value addition operator.
/// @tparam LhsValueType Type of the value used by the left hand side checked value.
/// @tparam LhsChecker Type of the checker used by the left hand side checked value.
/// @tparam LhsNoExcept Whether the left hand side checked value terminates or just throws on being invalid.
/// @tparam RhsValueType Type of the value used by the right hand side checked value.
/// @tparam RhsChecker Type of the checker used by the right hand side checked value.
/// @tparam RhsNoExcept Whether the right hand side checked value terminates or just throws on being invalid.
/// @relatedalso Checked
template <class LhsValueType, class LhsChecker, bool LhsNoExcept, // force newline
          class RhsValueType, class RhsChecker, bool RhsNoExcept>
constexpr auto operator+ (const Checked<LhsValueType, LhsChecker, LhsNoExcept>& lhs,
                          const Checked<RhsValueType, RhsChecker, RhsNoExcept>& rhs)
-> decltype(LhsValueType(lhs) + RhsValueType(rhs))
{
    return LhsValueType(lhs) + RhsValueType(rhs);
}

/// @brief Constrained value subtraction operator.
/// @tparam LhsValueType Type of the value used by the left hand side checked value.
/// @tparam LhsChecker Type of the checker used by the left hand side checked value.
/// @tparam LhsNoExcept Whether the left hand side checked value terminates or just throws on being invalid.
/// @tparam RhsValueType Type of the value used by the right hand side checked value.
/// @tparam RhsChecker Type of the checker used by the right hand side checked value.
/// @tparam RhsNoExcept Whether the right hand side checked value terminates or just throws on being invalid.
/// @relatedalso Checked
template <class LhsValueType, class LhsChecker, bool LhsNoExcept, // force newline
          class RhsValueType, class RhsChecker, bool RhsNoExcept>
constexpr auto operator- (const Checked<LhsValueType, LhsChecker, LhsNoExcept>& lhs,
                          const Checked<RhsValueType, RhsChecker, RhsNoExcept>& rhs)
-> decltype(LhsValueType(lhs) - RhsValueType(rhs))
{
    return LhsValueType(lhs) - RhsValueType(rhs);
}

/// @brief Constrained value equality operator.
/// @tparam ValueType Type of the value used by the checked value.
/// @tparam Checker Type of the checker used by the checked value.
/// @tparam NoExcept Whether the checked value terminates or just throws on being invalid.
/// @tparam Other Type of the other value that this operation will operator with.
/// @relatedalso Checked
template <class ValueType, class Checker, bool NoExcept, class Other>
constexpr auto operator== (const Checked<ValueType, Checker, NoExcept>& lhs,
                           const Other& rhs)
-> decltype(ValueType(lhs) == rhs)
{
    return ValueType(lhs) == rhs;
}

/// @brief Constrained value equality operator.
/// @tparam ValueType Type of the value used by the checked value.
/// @tparam Checker Type of the checker used by the checked value.
/// @tparam NoExcept Whether the checked value terminates or just throws on being invalid.
/// @tparam Other Type of the other value that this operation will operator with.
/// @relatedalso Checked
template <class ValueType, class Checker, bool NoExcept, class Other>
constexpr auto operator== (const Other& lhs,
                           const Checked<ValueType, Checker, NoExcept>& rhs)
-> decltype(lhs == ValueType(rhs))
{
    return lhs == ValueType(rhs);
}

/// @brief Constrained value inequality operator.
/// @tparam ValueType Type of the value used by the checked value.
/// @tparam Checker Type of the checker used by the checked value.
/// @tparam NoExcept Whether the checked value terminates or just throws on being invalid.
/// @tparam Other Type of the other value that this operation will operator with.
/// @relatedalso Checked
template <class ValueType, class Checker, bool NoExcept, class Other>
constexpr auto operator!= (const Checked<ValueType, Checker, NoExcept>& lhs,
                           const Other& rhs)
-> decltype(ValueType(lhs) != rhs)
{
    return ValueType(lhs) != rhs;
}

/// @brief Constrained value inequality operator.
/// @tparam ValueType Type of the value used by the checked value.
/// @tparam Checker Type of the checker used by the checked value.
/// @tparam NoExcept Whether the checked value terminates or just throws on being invalid.
/// @tparam Other Type of the other value that this operation will operator with.
/// @relatedalso Checked
template <class ValueType, class Checker, bool NoExcept, class Other>
constexpr auto operator!= (const Other& lhs,
                           const Checked<ValueType, Checker, NoExcept>& rhs)
-> decltype(lhs != ValueType(rhs))
{
    return lhs != ValueType(rhs);
}

/// @brief Constrained value less-than or equal-to operator.
/// @tparam ValueType Type of the value used by the checked value.
/// @tparam Checker Type of the checker used by the checked value.
/// @tparam NoExcept Whether the checked value terminates or just throws on being invalid.
/// @tparam Other Type of the other value that this operation will operator with.
/// @relatedalso Checked
template <class ValueType, class Checker, bool NoExcept, class Other>
constexpr auto operator<= (const Checked<ValueType, Checker, NoExcept>& lhs,
                           const Other& rhs)
-> decltype(ValueType(lhs) <= ValueType(rhs))
{
    return ValueType(lhs) <= ValueType(rhs);
}

/// @brief Constrained value less-than or equal-to operator.
/// @tparam ValueType Type of the value used by the checked value.
/// @tparam Checker Type of the checker used by the checked value.
/// @tparam NoExcept Whether the checked value terminates or just throws on being invalid.
/// @tparam Other Type of the other value that this operation will operator with.
/// @relatedalso Checked
template <class ValueType, class Checker, bool NoExcept, class Other>
constexpr auto operator<= (const Other& lhs,
                           const Checked<ValueType, Checker, NoExcept>& rhs)
-> decltype(ValueType(lhs) <= ValueType(rhs))
{
    return ValueType(lhs) <= ValueType(rhs);
}

/// @brief Constrained value greater-than or equal-to operator.
/// @tparam ValueType Type of the value used by the checked value.
/// @tparam Checker Type of the checker used by the checked value.
/// @tparam NoExcept Whether the checked value terminates or just throws on being invalid.
/// @tparam Other Type of the other value that this operation will operator with.
/// @relatedalso Checked
template <class ValueType, class Checker, bool NoExcept, class Other>
constexpr auto operator>= (const Checked<ValueType, Checker, NoExcept>& lhs,
                           const Other& rhs)
-> decltype(ValueType(lhs) >= ValueType(rhs))
{
    return ValueType(lhs) >= ValueType(rhs);
}

/// @brief Constrained value greater-than or equal-to operator.
/// @tparam ValueType Type of the value used by the checked value.
/// @tparam Checker Type of the checker used by the checked value.
/// @tparam NoExcept Whether the checked value terminates or just throws on being invalid.
/// @tparam Other Type of the other value that this operation will operator with.
/// @relatedalso Checked
template <class ValueType, class Checker, bool NoExcept, class Other>
constexpr auto operator>= (const Other& lhs,
                           const Checked<ValueType, Checker, NoExcept>& rhs)
-> decltype(ValueType(lhs) >= ValueType(rhs))
{
    return ValueType(lhs) >= ValueType(rhs);
}


/// @brief Constrained value less-than operator.
/// @tparam ValueType Type of the value used by the checked value.
/// @tparam Checker Type of the checker used by the checked value.
/// @tparam NoExcept Whether the checked value terminates or just throws on being invalid.
/// @tparam Other Type of the other value that this operation will operator with.
/// @relatedalso Checked
template <class ValueType, class Checker, bool NoExcept, class Other>
constexpr auto operator< (const Checked<ValueType, Checker, NoExcept>& lhs,
                          const Other& rhs)
-> decltype(ValueType(lhs) < ValueType(rhs))
{
    return ValueType(lhs) < ValueType(rhs);
}

/// @brief Constrained value less-than operator.
/// @tparam ValueType Type of the value used by the checked value.
/// @tparam Checker Type of the checker used by the checked value.
/// @tparam NoExcept Whether the checked value terminates or just throws on being invalid.
/// @tparam Other Type of the other value that this operation will operator with.
/// @relatedalso Checked
template <class ValueType, class Checker, bool NoExcept, class Other>
constexpr auto operator< (const Other& lhs,
                          const Checked<ValueType, Checker, NoExcept>& rhs)
-> decltype(ValueType(lhs) < ValueType(rhs))
{
    return ValueType(lhs) < ValueType(rhs);
}

/// @brief Constrained value greater-than operator.
/// @tparam ValueType Type of the value used by the checked value.
/// @tparam Checker Type of the checker used by the checked value.
/// @tparam NoExcept Whether the checked value terminates or just throws on being invalid.
/// @tparam Other Type of the other value that this operation will operator with.
/// @relatedalso Checked
template <class ValueType, class Checker, bool NoExcept, class Other>
constexpr auto operator> (const Checked<ValueType, Checker, NoExcept>& lhs,
                          const Other& rhs)
-> decltype(ValueType(lhs) > ValueType(rhs))
{
    return ValueType(lhs) > ValueType(rhs);
}

/// @brief Constrained value greater-than ooperator.
/// @tparam ValueType Type of the value used by the checked value.
/// @tparam Checker Type of the checker used by the checked value.
/// @tparam NoExcept Whether the checked value terminates or just throws on being invalid.
/// @tparam Other Type of the other value that this operation will operator with.
/// @relatedalso Checked
template <class ValueType, class Checker, bool NoExcept, class Other>
constexpr auto operator> (const Other& lhs,
                          const Checked<ValueType, Checker, NoExcept>& rhs)
-> decltype(ValueType(lhs) > ValueType(rhs))
{
    return ValueType(lhs) > ValueType(rhs);
}

/// @brief Constrained value multiplication operator.
/// @tparam ValueType Type of the value used by the checked value.
/// @tparam Checker Type of the checker used by the checked value.
/// @tparam NoExcept Whether the checked value terminates or just throws on being invalid.
/// @tparam Other Type of the other value that this operation will operator with.
/// @relatedalso Checked
template <class ValueType, class Checker, bool NoExcept, class Other>
constexpr auto operator* (const Checked<ValueType, Checker, NoExcept>& lhs, const Other& rhs)
-> std::enable_if_t<!IsMultipliableV<Checked<ValueType, Checker, NoExcept>, Other>,
decltype(ValueType()*Other())>
{
    return ValueType(lhs) * rhs;
}

/// @brief Constrained value multiplication operator.
/// @tparam ValueType Type of the value used by the checked value.
/// @tparam Checker Type of the checker used by the checked value.
/// @tparam NoExcept Whether the checked value terminates or just throws on being invalid.
/// @tparam Other Type of the other value that this operation will operator with.
/// @relatedalso Checked
template <class ValueType, class Checker, bool NoExcept, class Other>
constexpr auto operator* (const Other& lhs, const Checked<ValueType, Checker, NoExcept>& rhs)
-> std::enable_if_t<!IsMultipliableV<Other, Checked<ValueType, Checker, NoExcept>>,
decltype(Other()*ValueType())>
{
    return lhs * ValueType(rhs);
}

/// @brief Constrained value division operator.
/// @tparam ValueType Type of the value used by the checked value.
/// @tparam Checker Type of the checker used by the checked value.
/// @tparam NoExcept Whether the checked value terminates or just throws on being invalid.
/// @tparam Other Type of the other value that this operation will operator with.
/// @relatedalso Checked
template <class ValueType, class Checker, bool NoExcept, class Other>
constexpr auto operator/ (const Checked<ValueType, Checker, NoExcept>& lhs,
                          const Other& rhs)
-> decltype(ValueType(lhs) / rhs)
{
    return ValueType(lhs) / rhs;
}

/// @brief Constrained value division operator.
/// @tparam ValueType Type of the value used by the checked value.
/// @tparam Checker Type of the checker used by the checked value.
/// @tparam NoExcept Whether the checked value terminates or just throws on being invalid.
/// @tparam Other Type of the other value that this operation will operator with.
/// @relatedalso Checked
template <class ValueType, class Checker, bool NoExcept, class Other>
constexpr auto operator/ (const Other& lhs,
                          const Checked<ValueType, Checker, NoExcept>& rhs)
-> decltype(lhs / ValueType(rhs))
{
    return lhs / ValueType(rhs);
}

/// @brief Constrained value addition operator.
/// @tparam ValueType Type of the value used by the checked value.
/// @tparam Checker Type of the checker used by the checked value.
/// @tparam NoExcept Whether the checked value terminates or just throws on being invalid.
/// @tparam Other Type of the other value that this operation will operator with.
/// @relatedalso Checked
template <class ValueType, class Checker, bool NoExcept, class Other>
constexpr auto operator+ (const Checked<ValueType, Checker, NoExcept>& lhs,
                          const Other& rhs)
-> decltype(ValueType(lhs) + rhs)
{
    return ValueType(lhs) + rhs;
}

/// @brief Constrained value addition operator.
/// @tparam ValueType Type of the value used by the checked value.
/// @tparam Checker Type of the checker used by the checked value.
/// @tparam NoExcept Whether the checked value terminates or just throws on being invalid.
/// @tparam Other Type of the other value that this operation will operator with.
/// @relatedalso Checked
template <class ValueType, class Checker, bool NoExcept, class Other>
constexpr auto operator+ (const Other& lhs,
                          const Checked<ValueType, Checker, NoExcept>& rhs)
-> decltype(lhs + ValueType(rhs))
{
    return lhs + ValueType(rhs);
}

/// @brief Constrained value subtraction operator.
/// @tparam ValueType Type of the value used by the checked value.
/// @tparam Checker Type of the checker used by the checked value.
/// @tparam NoExcept Whether the checked value terminates or just throws on being invalid.
/// @tparam Other Type of the other value that this operation will operator with.
/// @relatedalso Checked
template <class ValueType, class Checker, bool NoExcept, class Other>
constexpr auto operator- (const Checked<ValueType, Checker, NoExcept>& lhs,
                          const Other& rhs)
-> decltype(ValueType(lhs) - rhs)
{
    return ValueType(lhs) - rhs;
}

/// @brief Constrained value subtraction operator.
/// @tparam ValueType Type of the value used by the checked value.
/// @tparam Checker Type of the checker used by the checked value.
/// @tparam NoExcept Whether the checked value terminates or just throws on being invalid.
/// @tparam Other Type of the other value that this operation will operator with.
/// @relatedalso Checked
template <class ValueType, class Checker, bool NoExcept, class Other>
constexpr auto operator- (const Other& lhs,
                          const Checked<ValueType, Checker, NoExcept>& rhs)
-> decltype(lhs - ValueType(rhs))
{
    return lhs - ValueType(rhs);
}

/// @defgroup CheckedTypes Checked Value Types
/// @brief Types for holding checked valid values.
/// @details Type aliases for checked values via on-construction checks that
///   may throw an exception if an attempt is made to construct the checked value
///   type with a value not allowed by the specific alias.
/// @see Checked

/// @ingroup CheckedTypes
/// @brief Default checked value type.
/// @details A checked value type using the default checker type.
/// @note This is basically a no-op for base line testing and demonstration purposes.
template <class T>
using DefaultCheckedValue = Checked<T>;

} // namespace playrho::detail

#endif // PLAYRHO_CHECKEDVALUE_HPP