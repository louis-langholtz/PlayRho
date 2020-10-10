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

template <>
struct TypeInfo<float>
{
    static const char* name() noexcept {
        return "float";
    }
};

template <>
struct TypeInfo<double>
{
    static const char* name() noexcept {
        return "double";
    }
};

template <>
struct TypeInfo<long double>
{
    static const char* name() noexcept {
        return "long double";
    }
};

#endif // USE_RTTI

/// @brief Type identifier.
using TypeID = strongtype::IndexingNamedType<const char *(*)() noexcept, struct TypeIdentifier>;

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

template <typename T>
constexpr TypeID GetTypeID()
{
    return TypeID{&TypeInfo<std::decay_t<T>>::name};
}

template <typename T>
TypeID GetTypeID(T)
{
    return TypeID{&TypeInfo<std::decay_t<T>>::name};
}

inline const char* GetName(TypeID id) noexcept
{
    return (*UnderlyingValue(id))();
}

template <typename T>
const char* GetTypeName() noexcept
{
    return TypeInfo<T>::name();
}

} // namespace playrho

#endif // PLAYRHO_COMMON_TYPEID_HPP
