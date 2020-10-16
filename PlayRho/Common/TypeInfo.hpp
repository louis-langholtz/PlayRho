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

#include <PlayRho/Common/StrongType.hpp>
#include <PlayRho/Common/Templates.hpp> // for GetInvalid, IsValid

#ifdef USE_RTTI
#include <typeinfo>
#endif // USE_RTTI

namespace playrho {

/// @brief Type information.
/// @note Users may specialize this for their own types.
template <typename T>
struct TypeInfo
{
    /// @brief Gets the name of the templated type.
    /// @note This is also a static member providing a unique ID, via its address, for
    ///   the type T without resorting to using C++ run-time type information (RTTI).
    static const char* name() noexcept {
#ifdef USE_RTTI
        // No gaurantee of what the following returns. Could be mangled!
        // See http://en.cppreference.com/w/cpp/types/type_info/name
        return typeid(T).name();
#else // !USE_RTTI
        return nullptr;
#endif // USE_RTTI
    }
};

#ifndef USE_RTTI

/// @brief Type info specialization for <code>float</code>.
template <>
struct TypeInfo<float>
{
    /// @brief Provides name of the type as a null-terminated string.
    static const char* name() noexcept {
        return "float";
    }
};

/// @brief Type info specialization for <code>double</code>.
template <>
struct TypeInfo<double>
{
    /// @brief Provides name of the type as a null-terminated string.
    static const char* name() noexcept {
        return "double";
    }
};

/// @brief Type info specialization for <code>long double</code>.
template <>
struct TypeInfo<long double>
{
    /// @brief Provides name of the type as a null-terminated string.
    static const char* name() noexcept {
        return "long double";
    }
};

#endif // USE_RTTI

/// @brief Type identifier.
using TypeID = strongtype::IndexingNamedType<const char *(*)() noexcept, struct TypeIdentifier>;

static_assert(sizeof(TypeID) == sizeof(void*), "TypeID size not that of a pointer?!");

/// @brief Invalid type ID value.
constexpr auto InvalidTypeID =
    static_cast<TypeID>(static_cast<TypeID::underlying_type>(nullptr));

/// @brief Gets an invalid value for the TypeID type.
template <>
constexpr TypeID GetInvalid() noexcept
{
    return InvalidTypeID;
}

/// @brief Determines if the given value is valid.
template <>
constexpr bool IsValid(const TypeID& value) noexcept
{
    return value != GetInvalid<TypeID>();
}

/// @brief Gets the type ID for the template parameter type.
template <typename T>
constexpr TypeID GetTypeID()
{
    return TypeID{&TypeInfo<std::decay_t<T>>::name};
}

/// @brief Gets the type ID for the function parameter type.
template <typename T>
TypeID GetTypeID(T)
{
    return TypeID{&TypeInfo<std::decay_t<T>>::name};
}

/// @brief Gets the name associated with the given type ID.
inline const char* GetName(TypeID id) noexcept
{
    return (*UnderlyingValue(id))();
}

/// @brief Gets the name associated with the given template parameter type.
template <typename T>
const char* GetTypeName() noexcept
{
    return TypeInfo<T>::name();
}

} // namespace playrho

#endif // PLAYRHO_COMMON_TYPEID_HPP
