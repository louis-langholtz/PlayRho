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

#include <limits>
#include <typeinfo>

namespace playrho
{

    /// @brief "Not used" annotator.
    template<class... T> void NOT_USED(T&&...){}

    /// @brief Gets an invalid value for the type.
    template <typename T>
    constexpr T GetInvalid() noexcept
    {
        static_assert(sizeof(T) == 0, "No available specialization");
    }

    /// @brief Determines if the given value is valid.
    template <typename T>
    constexpr bool IsValid(const T& value) noexcept
    {
        // Note: This is not necessarily a no-op!! But it is a "constexpr".
        //
        // From http://en.cppreference.com/w/cpp/numeric/math/isnan:
        //   "Another way to test if a floating-point value is NaN is
        //    to compare it with itself:
        //      bool is_nan(double x) { return x != x; }
        //
        // So for all T, for which std::isnan() is implemented, this should work
        // correctly and quite usefully!
        //
        return value == value;
    }

    // GetInvalid template specializations.
    
    /// @brief Gets an invalid value for the float type.
    template <>
    constexpr float GetInvalid() noexcept
    {
        return std::numeric_limits<float>::signaling_NaN();
    }
    
    /// @brief Gets an invalid value for the double type.
    template <>
    constexpr double GetInvalid() noexcept
    {
        return std::numeric_limits<double>::signaling_NaN();
    }
    
    /// @brief Gets an invalid value for the long double type.
    template <>
    constexpr long double GetInvalid() noexcept
    {
        return std::numeric_limits<long double>::signaling_NaN();
    }
    
    /// @brief Gets an invalid value for the std::size_t type.
    template <>
    constexpr std::size_t GetInvalid() noexcept
    {
        return static_cast<std::size_t>(-1);
    }
    
    // IsValid template specializations.
    
    /// @brief Determines if the given value is valid.
    template <>
    constexpr inline bool IsValid(const std::size_t& value) noexcept
    {
        return value != GetInvalid<std::size_t>();
    }
    
    // Other templates.
    
    /// @brief Gets a pointer for the given variable.
    template <class T>
    constexpr const T* GetPtr(const T* value) noexcept
    {
        return value;
    }
    
    /// @brief Gets a pointer for the given variable.
    template <class T>
    constexpr T* GetPtr(T* value) noexcept
    {
        return value;
    }
    
    /// @brief Gets a pointer for the given variable.
    template <class T>
    constexpr const T* GetPtr(const T& value) noexcept
    {
        return &value;
    }
    
    /// @brief Gets a pointer for the given variable.
    template <class T>
    constexpr T* GetPtr(T& value) noexcept
    {
        return &value;
    }

    /// @brief Gets a reference for the given variable.
    template <class T>
    constexpr const T& GetRef(const T* value) noexcept
    {
        return *value;
    }
    
    /// @brief Gets a reference for the given variable.
    template <class T>
    constexpr T& GetRef(T* value) noexcept
    {
        return *value;
    }
    
    /// @brief Gets a reference for the given variable.
    template <class T>
    constexpr const T& GetRef(const T& value) noexcept
    {
        return value;
    }
    
    /// @brief Gets a reference for the given variable.
    template <class T>
    constexpr T& GetRef(T& value) noexcept
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

} // namespace playrho

#endif // PLAYRHO_COMMON_TEMPLATES_HPP
