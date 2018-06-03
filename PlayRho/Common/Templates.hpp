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

#ifndef PLAYRHO_COMMON_TEMPLATES_HPP
#define PLAYRHO_COMMON_TEMPLATES_HPP

#include <PlayRho/Defines.hpp>

#include <algorithm>
#include <functional>
#include <iterator>
#include <limits>
#include <typeinfo>
#include <type_traits>
#include <tuple>

namespace playrho {

    /// @brief "Not used" annotator.
    template<class... T> void NOT_USED(T&&...){}

    /// @brief Gets an invalid value for the type.
    template <typename T>
    PLAYRHO_CONSTEXPR inline T GetInvalid() noexcept
    {
        static_assert(sizeof(T) == 0, "No available specialization");
    }

    /// @brief Determines if the given value is valid.
    template <typename T>
    PLAYRHO_CONSTEXPR inline bool IsValid(const T& value) noexcept
    {
        // Note: This is not necessarily a no-op!! But it is a "PLAYRHO_CONSTEXPR inline".
        //
        // From http://en.cppreference.com/w/cpp/numeric/math/isnan:
        //   "Another way to test if a floating-point value is NaN is
        //    to compare it with itself:
        //      bool is_nan(double x) { return x != x; }
        //
        // So for all T, for which isnan() is implemented, this should work
        // correctly and quite usefully!
        //
        return value == value;
    }

    // GetInvalid template specializations.
    
    /// @brief Gets an invalid value for the float type.
    template <>
    PLAYRHO_CONSTEXPR inline float GetInvalid() noexcept
    {
        return std::numeric_limits<float>::signaling_NaN();
    }
    
    /// @brief Gets an invalid value for the double type.
    template <>
    PLAYRHO_CONSTEXPR inline double GetInvalid() noexcept
    {
        return std::numeric_limits<double>::signaling_NaN();
    }
    
    /// @brief Gets an invalid value for the long double type.
    template <>
    PLAYRHO_CONSTEXPR inline long double GetInvalid() noexcept
    {
        return std::numeric_limits<long double>::signaling_NaN();
    }
    
    /// @brief Gets an invalid value for the std::size_t type.
    template <>
    PLAYRHO_CONSTEXPR inline std::size_t GetInvalid() noexcept
    {
        return static_cast<std::size_t>(-1);
    }
    
    // IsValid template specializations.
    
    /// @brief Determines if the given value is valid.
    template <>
    PLAYRHO_CONSTEXPR inline bool IsValid(const std::size_t& value) noexcept
    {
        return value != GetInvalid<std::size_t>();
    }
    
    // Other templates.
    
    /// @brief Gets a pointer for the given variable.
    template <class T>
    PLAYRHO_CONSTEXPR const T* GetPtr(const T* value) noexcept
    {
        return value;
    }
    
    /// @brief Gets a pointer for the given variable.
    template <class T>
    PLAYRHO_CONSTEXPR inline T* GetPtr(T* value) noexcept
    {
        return value;
    }
    
    /// @brief Gets a pointer for the given variable.
    template <class T>
    PLAYRHO_CONSTEXPR const T* GetPtr(const T& value) noexcept
    {
        return &value;
    }
    
    /// @brief Gets a pointer for the given variable.
    template <class T>
    PLAYRHO_CONSTEXPR inline T* GetPtr(T& value) noexcept
    {
        return &value;
    }

    /// @brief Gets a reference for the given variable.
    template <class T>
    PLAYRHO_CONSTEXPR const T& GetRef(const T* value) noexcept
    {
        return *value;
    }
    
    /// @brief Gets a reference for the given variable.
    template <class T>
    PLAYRHO_CONSTEXPR inline T& GetRef(T* value) noexcept
    {
        return *value;
    }
    
    /// @brief Gets a reference for the given variable.
    template <class T>
    PLAYRHO_CONSTEXPR const T& GetRef(const T& value) noexcept
    {
        return value;
    }
    
    /// @brief Gets a reference for the given variable.
    template <class T>
    PLAYRHO_CONSTEXPR inline T& GetRef(T& value) noexcept
    {
        return value;
    }
    
    /// @brief Template function for visiting objects.
    /// @note Specialize this function to tie in application specific handling for types
    ///   which don't already have specialized handling. Specializations should always
    ///   return <code>true</code>.
    /// @note First parameter is the object to visit.
    /// @note Second parameter is user data or the <code>nullptr</code>.
    /// @sa https://en.wikipedia.org/wiki/Visitor_pattern
    template <typename T>
    bool Visit(const T& /*object*/, void* /*userData*/)
    {
        return false;
    }
    
    /// @brief Gets the library defined name for the given type.
    /// @details Provides an interface to a function that can be specialized for getting
    ///   a C-style null-terminated array of characters that names the type.
    /// @return Non-null pointer to C-style string name of specified type.
    template <typename T>
    inline const char* GetTypeName() noexcept
    {
        // No gaurantee of what the following returns. Could be mangled!
        // See http://en.cppreference.com/w/cpp/types/type_info/name
        return typeid(T).name();
    }
    
    /// @brief Gets a human recognizable name for the float type.
    template <>
    inline const char* GetTypeName<float>() noexcept
    {
        return "float";
    }
    
    /// @brief Gets a human recognizable name for the double type.
    template <>
    inline const char* GetTypeName<double>() noexcept
    {
        return "double";
    }
    
    /// @brief Gets a human recognizable name for the long double type.
    template <>
    inline const char* GetTypeName<long double>() noexcept
    {
        return "long double";
    }

    /// @brief Voiding template class.
    template<class...> struct Voidify {
        /// @brief Type alias.
        using type = void;
    };

    /// @brief Void type templated alias.
    template<class... Ts> using VoidT = typename Voidify<Ts...>::type;
    
    /// @brief Template for determining if the given type is an equality comparable type.
    /// @note This isn't exactly the same as the "EqualityComparable" concept.
    /// @see http://en.cppreference.com/w/cpp/concept/EqualityComparable
    template<class T1, class T2, class = void>
    struct IsEqualityComparable: std::false_type {};
    
    /// @brief Template specialization for equality comparable types.
    template<class T1, class T2>
    struct IsEqualityComparable<T1, T2, VoidT<decltype(T1{} == T2{})> >: std::true_type {};
    
    /// @brief Template for determining if the given type is an inequality comparable type.
    template<class T1, class T2, class = void>
    struct IsInequalityComparable: std::false_type {};
    
    /// @brief Template specialization for inequality comparable types.
    template<class T1, class T2>
    struct IsInequalityComparable<T1, T2, VoidT<decltype(T1{} != T2{})> >: std::true_type {};

    template<class T1, class T2, class = void>
    struct IsMultipliable: std::false_type {};
    
    template<class T1, class T2>
    struct IsMultipliable<T1, T2, VoidT<decltype(T1{} * T2{})> >: std::true_type {};
    
    template<class T1, class T2, class = void>
    struct IsDivisable: std::false_type {};
    
    template<class T1, class T2>
    struct IsDivisable<T1, T2, VoidT<decltype(T1{} / T2{})> >: std::true_type {};

    /// @brief Template for determining if the given type is an "arithmetic" type.
    /// @note In the context of this library, "arithmetic" types are all types which
    ///   have +, -, *, / arithmetic operator support.
    template<class T, class = void>
    struct IsArithmetic: std::false_type {};
    
    /// @brief Template specialization for valid/acceptable "arithmetic" types.
    template<class T>
    struct IsArithmetic<T, VoidT<
        decltype(T{} + T{}), decltype(T{} - T{}), decltype(T{} * T{}), decltype(T{} / T{})
    > >: std::true_type {};
    
    /// @brief Has-type trait template class.
    /// @note This is from Piotr Skotnicki's answer on the <em>StackOverflow</em> website
    ///   to the question of: "How do I find out if a tuple contains a type?".
    /// @sa https://stackoverflow.com/a/25958302/7410358
    template <typename T, typename Tuple>
    struct HasType;
    
    /// @brief Has-type trait template class specialized for <code>std::tuple</code> classes.
    /// @note This is from Piotr Skotnicki's answer on the <em>StackOverflow</em> website
    ///   to the question of: "How do I find out if a tuple contains a type?".
    /// @sa https://stackoverflow.com/a/25958302/7410358
    template <typename T>
    struct HasType<T, std::tuple<>> : std::false_type {};
    
    /// @brief Has-type trait true class.
    /// @note This is from Piotr Skotnicki's answer on the <em>StackOverflow</em> website
    ///   to the question of: "How do I find out if a tuple contains a type?".
    /// @sa https://stackoverflow.com/a/25958302/7410358
    template <typename T, typename... Ts>
    struct HasType<T, std::tuple<T, Ts...>> : std::true_type {};
    
    /// @brief Has-type trait template super class.
    /// @note This is from Piotr Skotnicki's answer on the <em>StackOverflow</em> website
    ///   to the question of: "How do I find out if a tuple contains a type?".
    /// @sa https://stackoverflow.com/a/25958302/7410358
    template <typename T, typename U, typename... Ts>
    struct HasType<T, std::tuple<U, Ts...>> : HasType<T, std::tuple<Ts...>> {};

    /// @brief Tuple contains type alias.
    /// @details Alias in case the trait itself should be <code>std::true_type</code> or
    ///   <code>std::false_type</code>.
    /// @note This is from Piotr Skotnicki's answer on the <em>StackOverflow</em> website
    ///   to the question of: "How do I find out if a tuple contains a type?".
    /// @sa https://stackoverflow.com/a/25958302/7410358
    template <typename T, typename Tuple>
    using TupleContainsType = typename HasType<T, Tuple>::type;
    
    /// @brief Computes the absolute value of the given value.
    template <typename T>
    PLAYRHO_CONSTEXPR inline T Abs(T a)
    {
        return (a >= T{0}) ? a : -a;
    }

    /// @brief Checks whether the given container is empty.
    /// @note This is from <code>std::empty</code> for C++17.
    /// @sa http://en.cppreference.com/w/cpp/iterator/empty
    template <class T>
    PLAYRHO_CONSTEXPR inline auto IsEmpty(const T& arg) -> decltype(arg.empty())
    {
        return arg.empty();
    }

    /// @brief Gets the current size of the given container.
    /// @note This is from <code>std::size</code> for C++17.
    /// @sa http://en.cppreference.com/w/cpp/iterator/size
    template <class T>
    PLAYRHO_CONSTEXPR inline auto GetSize(const T& arg) -> decltype(arg.size())
    {
        return arg.size();
    }
    
    /// @brief Gets the maximum size of the given container.
    template <class T>
    PLAYRHO_CONSTEXPR inline auto GetMaxSize(const T& arg) -> decltype(arg.max_size())
    {
        return arg.max_size();
    }

    /// @brief Checks whether the given container is full.
    template <class T>
    PLAYRHO_CONSTEXPR inline auto IsFull(const T& arg) -> decltype(GetSize(arg) == arg.max_size())
    {
        return GetSize(arg) == GetMaxSize(arg);
    }

    /// @brief Gets the compile-time size of a C-style array.
    template <class T, std::size_t N>
    PLAYRHO_CONSTEXPR inline std::size_t GetSize(T (&)[N]) { return N; }
    
    /// @brief Function object for performing lexicographical less-than
    ///   comparisons of containers.
    /// @sa http://en.cppreference.com/w/cpp/algorithm/lexicographical_compare
    /// @sa http://en.cppreference.com/w/cpp/utility/functional/less
    template <typename T>
    struct LexicographicalLess
    {
        /// @brief Checks whether the first argument is lexicographically less-than the
        ///   second argument.
        constexpr bool operator()(const T& lhs, const T& rhs) const
        {
            using std::begin;
            using std::end;
            using std::lexicographical_compare;
            using std::less;
            using ElementType = decltype(*begin(lhs));
            return lexicographical_compare(begin(lhs), end(lhs), begin(rhs), end(rhs),
                                           less<ElementType>{});
        }
    };
    
    /// @brief Function object for performing lexicographical greater-than
    ///   comparisons of containers.
    /// @sa http://en.cppreference.com/w/cpp/algorithm/lexicographical_compare
    /// @sa http://en.cppreference.com/w/cpp/utility/functional/greater
    template <typename T>
    struct LexicographicalGreater
    {
        /// @brief Checks whether the first argument is lexicographically greater-than the
        ///   second argument.
        constexpr bool operator()(const T& lhs, const T& rhs) const
        {
            using std::begin;
            using std::end;
            using std::lexicographical_compare;
            using std::greater;
            using ElementType = decltype(*begin(lhs));
            return lexicographical_compare(begin(lhs), end(lhs), begin(rhs), end(rhs),
                                           greater<ElementType>{});
        }
    };

    /// @brief Function object for performing lexicographical less-than or equal-to
    ///   comparisons of containers.
    /// @sa http://en.cppreference.com/w/cpp/algorithm/lexicographical_compare
    /// @sa http://en.cppreference.com/w/cpp/utility/functional/less_equal
    template <typename T>
    struct LexicographicalLessEqual
    {
        /// @brief Checks whether the first argument is lexicographically less-than or
        ///   equal-to the second argument.
        constexpr bool operator()(const T& lhs, const T& rhs) const
        {
            using std::begin;
            using std::end;
            using std::mismatch;
            using std::less;
            using std::get;
            using ElementType = decltype(*begin(lhs));
            const auto lhsEnd = end(lhs);
            const auto diff = mismatch(begin(lhs), lhsEnd, begin(rhs), end(rhs));
            return (get<0>(diff) == lhsEnd) || less<ElementType>{}(*get<0>(diff), *get<1>(diff));
        }
    };

    /// @brief Function object for performing lexicographical greater-than or equal-to
    ///   comparisons of containers.
    /// @sa http://en.cppreference.com/w/cpp/algorithm/lexicographical_compare
    /// @sa http://en.cppreference.com/w/cpp/utility/functional/greater_equal
    template <typename T>
    struct LexicographicalGreaterEqual
    {
        /// @brief Checks whether the first argument is lexicographically greater-than or
        ///   equal-to the second argument.
        constexpr bool operator()(const T& lhs, const T& rhs) const
        {
            using std::begin;
            using std::end;
            using std::mismatch;
            using std::greater;
            using std::get;
            using ElementType = decltype(*begin(lhs));
            const auto lhsEnd = end(lhs);
            const auto diff = mismatch(begin(lhs), lhsEnd, begin(rhs), end(rhs));
            return (get<0>(diff) == lhsEnd) || greater<ElementType>{}(*get<0>(diff), *get<1>(diff));
        }
    };

} // namespace playrho

#endif // PLAYRHO_COMMON_TEMPLATES_HPP
