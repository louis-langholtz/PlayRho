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

#ifndef PLAYRHO_COMMON_TYPEID_HPP
#define PLAYRHO_COMMON_TYPEID_HPP

#include <PlayRho/Common/Templates.hpp>

#include <functional> // for std::reference_wrapper
#include <regex>
#include <string>
#include <typeindex>
#include <typeinfo>

namespace playrho {
namespace detail {

template <typename T>
std::string TypeNameAsString()
{
    // Ideally return string unique to the type T...
#if defined(__clang__)
    // Use __PRETTY_FUNCTION__. **Note that despite its appearance, this is an identifier; it's not a macro**!
    // template <typename T> string Name() { return string{__PRETTY_FUNCTION__}; }
    // enum class Fruit {APPLE, PEAR};
    // std::cout << Name<Fruit>() << '\n';
    // produces: std::string Name() [T = Fruit]
    return std::regex_replace(__PRETTY_FUNCTION__, std::regex(".*T = (.*)\\].*"), "$1");
#elif defined(__GNUC__)
    // Use __PRETTY_FUNCTION__. **Note that despite its appearance, this is an identifier; it's not a macro**!
    // template <typename T> string Name() { return string{__PRETTY_FUNCTION__}; }
    // enum class Fruit {APPLE, PEAR};
    // std::cout << Name<Fruit>() << '\n';
    // produces: std::string Name() [with T = Fruit; std::string = std::__cxx11::basic_string<char>]
    return std::regex_replace(__PRETTY_FUNCTION__, std::regex(".*T = (.*);.*"), "$1");
#elif defined(__FUNCSIG__)
    // Assume this is Microsoft Visual C++ or compatible compiler and format.
    // enum class Fruit {APPLE, PEAR};
    // std::cout << Name<Fruit>() << '\n';
    // produces:
    // class std::basic_string<char,struct std::char_traits<char>,class std::allocator<char> > __cdecl Name<enum Fruit>(void)
    return std::regex_replace(__FUNCSIG__, std::regex(".* __cdecl [^<]+<(.*)>\\(void\\)"), "$1");
#else
    return {}; // not unique but maybe still helpful at avoiding compiler issues
#endif
}

/// @brief Gets a null-terminated byte string identifying this function.
/// @note Intended for use by <code>TypeInfo</code> to set the value of its
///    <code>name</code> variable to something dependent on the type and avoid issues
///    like <code>TypeInfo::name</code> being a non-unique address like happens on MSVC
///    when whole program is turned on. Such an issue is documented in Issue #370.
/// @see https://github.com/louis-langholtz/PlayRho/issues/370
template <typename T>
static const char* GetNameForTypeInfo() noexcept
{
    static const std::string buffer = TypeNameAsString<T>();
    return buffer.c_str();
}

} // namespace detail

/// @brief Type information.
/// @note Users may specialize this for their own types.
template <typename T>
struct TypeInfo
{
    /// @brief The name of the templated type.
    /// @note This is also a static member providing a unique ID, via its address, for
    ///   the type T without resorting to using C++ run-time type information (RTTI).
    /// @note Setting this to a null-terminated byte string that's unique to at least the
    ///   template's type <code>T</code> prevents issue #370. Credit for this technique
    ///   goes to Li Jin (github user pigpigyyy).
    /// @see https://github.com/louis-langholtz/PlayRho/issues/370
    static inline const char* name = detail::GetNameForTypeInfo<T>();
};

class TypeID;

/// @brief Gets the type ID for the function's template parameter type with its name demangled.
template <typename T>
TypeID GetTypeID() noexcept;

/// @brief Gets the type ID for the function parameter type with its name demangled.
template <typename T>
TypeID GetTypeID(const T&) noexcept;

/// @brief Type identifier.
/// @note This provides value semantics like being copyable, assignable, and equality comparable.
class TypeID
{
public:
    /// Default constructor.
    /// @post A type identifier equivalent to the value returned by <code>GetTypeID<void>()</code>.
    TypeID() noexcept = default;

    /// Gets demangled name of the type this was generated for as a a non-null, null terminated string buffer.
    constexpr const char* GetName() const noexcept
    {
        return m_name;
    }

    /// Equality operator support via "hidden friend" function.
    inline friend bool operator==(const TypeID& lhs, const TypeID& rhs) noexcept
    {
        return lhs.m_info.get() == rhs.m_info.get();
    }

    /// Inequality operator support via "hidden friend" function.
    inline friend bool operator!=(const TypeID& lhs, const TypeID& rhs) noexcept
    {
        return !(lhs == rhs);
    }

    /// Less-than operator support via "hidden friend" function.
    /// @note The ordering of type IDs is unspecified. This is provided anyway to support things like associative containers.
    inline friend bool operator<(const TypeID& lhs, const TypeID& rhs) noexcept
    {
        return std::type_index{lhs.m_info.get()} < std::type_index{rhs.m_info.get()};
    }

    /// Less-than-or-equal operator support via "hidden friend" function.
    /// @note The ordering of type IDs is unspecified. This is provided anyway to support things like associative containers.
    inline friend bool operator<=(const TypeID& lhs, const TypeID& rhs) noexcept
    {
        return std::type_index{lhs.m_info.get()} <= std::type_index{rhs.m_info.get()};
    }

    /// Greater-than operator support via "hidden friend" function.
    /// @note The ordering of type IDs is unspecified. This is provided anyway to support things like associative containers.
    inline friend bool operator>(const TypeID& lhs, const TypeID& rhs) noexcept
    {
        return std::type_index{lhs.m_info.get()} > std::type_index{rhs.m_info.get()};
    }

    /// Greater-than-or-equal operator support via "hidden friend" function.
    /// @note The ordering of type IDs is unspecified. This is provided anyway to support things like associative containers.
    inline friend bool operator>=(const TypeID& lhs, const TypeID& rhs) noexcept
    {
        return std::type_index{lhs.m_info.get()} >= std::type_index{rhs.m_info.get()};
    }

    template <typename T>
    friend TypeID GetTypeID() noexcept;

    template <typename T>
    friend TypeID GetTypeID(const T&) noexcept;

private:
    explicit TypeID(const std::type_info& info, const char* name) noexcept:
        m_info{info}, m_name{name}
    {
        // Intentionally empty.
    }

    std::reference_wrapper<const std::type_info> m_info{typeid(void)};
    const char* m_name{TypeInfo<void>::name};
};

template <typename T>
TypeID GetTypeID() noexcept
{
    return TypeID{typeid(std::decay_t<T>), TypeInfo<std::decay_t<T>>::name};
}

template <typename T>
TypeID GetTypeID(const T&) noexcept
{
    return TypeID{typeid(std::decay_t<T>), TypeInfo<std::decay_t<T>>::name};
}

/// @brief Gets the name associated with the given type ID.
inline const char* GetName(const TypeID& id) noexcept
{
    return id.GetName();
}

/// @brief Gets the name associated with the given template parameter type.
template <typename T>
constexpr const char* GetTypeName() noexcept
{
    return GetTypeID<std::decay_t<T>>().GetName();
}

} // namespace playrho

#endif // PLAYRHO_COMMON_TYPEID_HPP
