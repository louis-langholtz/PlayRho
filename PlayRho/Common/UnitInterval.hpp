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

#ifndef PLAYRHO_COMMON_UNITINTERVAL_HPP
#define PLAYRHO_COMMON_UNITINTERVAL_HPP

#include <PlayRho/Common/CheckedValue.hpp>

namespace playrho {

template <typename T>
struct UnitIntervalChecker {
    using exception_type = std::invalid_argument;

    constexpr auto operator()() noexcept -> decltype(static_cast<T>(0))
    {
        return static_cast<T>(0);
    }

    constexpr auto operator()(const T& v) -> decltype((v >= static_cast<T>(0) && v <= static_cast<T>(1)), T{v})
    {
        if (!(v >= static_cast<T>(0))) {
            throw exception_type("value not greater than nor equal to zero");
        }
        if (!(v <= static_cast<T>(1))) {
            throw exception_type("value not less than nor equal to one");
        }
        return v;
    }
};

/// @brief Unit interval constrained value type.
template <typename T>
using UnitInterval = CheckedValue<T, UnitIntervalChecker<T>>;

static_assert(std::is_default_constructible<UnitInterval<int>>::value);

} // namespace playrho

#endif // PLAYRHO_COMMON_UNITINTERVAL_HPP
