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

namespace playrho {
namespace detail {

template <typename T>
struct Type
{
    /// Function providing a unique ID - its address - for the type T
    /// without resorting to using C++ run-time type information (RTTI).
    static void Id() {
    }
};

} // namespace detail

/// @brief Type identifier.
using TypeID = strongtype::IndexingNamedType<void*, struct TypeIdentifier>;

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
TypeID GetTypeID()
{
    return TypeID{reinterpret_cast<void*>(&detail::Type<std::decay_t<T>>::Id)};
}

template <typename T>
TypeID GetTypeID(T)
{
    return TypeID{reinterpret_cast<void*>(&detail::Type<std::decay_t<T>>::Id)};
}

} // namespace playrho

#endif // PLAYRHO_COMMON_TYPEID_HPP
