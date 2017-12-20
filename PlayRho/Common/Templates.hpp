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

#include <limits>
#include <typeinfo>
#include <type_traits>
#include <tuple>

namespace playrho
{

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
    
    /// @brief Gets the library defined name for the given type.
    /// @details Provides an interface to a specializable function for getting C-style
    ///   null-terminated array of characters that names the type.
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
    /// @note This is from Piotr Skotnicki's answer on StackOverflow to the question of
    ///   "How do I find out if a tuple contains a type?".
    /// @sa https://stackoverflow.com/a/25958302/7410358
    template <typename T, typename Tuple>
    struct HasType;
    
    /// @brief Has-type trait template class specialized for std::tuple classes.
    /// @note This is from Piotr Skotnicki's answer on StackOverflow to the question of
    ///   "How do I find out if a tuple contains a type?".
    /// @sa https://stackoverflow.com/a/25958302/7410358
    template <typename T>
    struct HasType<T, std::tuple<>> : std::false_type {};
    
    /// @brief Has-type trait true class.
    /// @note This is from Piotr Skotnicki's answer on StackOverflow to the question of
    ///   "How do I find out if a tuple contains a type?".
    /// @sa https://stackoverflow.com/a/25958302/7410358
    template <typename T, typename... Ts>
    struct HasType<T, std::tuple<T, Ts...>> : std::true_type {};
    
    /// @brief Has-type trait template super class.
    /// @note This is from Piotr Skotnicki's answer on StackOverflow to the question of
    ///   "How do I find out if a tuple contains a type?".
    /// @sa https://stackoverflow.com/a/25958302/7410358
    template <typename T, typename U, typename... Ts>
    struct HasType<T, std::tuple<U, Ts...>> : HasType<T, std::tuple<Ts...>> {};

    /// @brief Tuple contains type alias.
    /// @details Alias in case the trait itself should be std::true_type or std::false_type.
    /// @note This is from Piotr Skotnicki's answer on StackOverflow to the question of
    ///   "How do I find out if a tuple contains a type?".
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

} // namespace playrho

#endif // PLAYRHO_COMMON_TEMPLATES_HPP
