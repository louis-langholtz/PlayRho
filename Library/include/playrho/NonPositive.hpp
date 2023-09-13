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

#ifndef PLAYRHO_NONPOSITIVE_HPP
#define PLAYRHO_NONPOSITIVE_HPP

#include <playrho/Checked.hpp>

namespace playrho {

/// @brief Non-positive constrained value checker.
template <typename T>
struct NonPositiveChecker {

    /// @brief Default value supplying functor.
    constexpr auto operator()() noexcept -> decltype(static_cast<T>(0))
    {
        return static_cast<T>(0);
    }

    /// @brief Value checking functor.
    constexpr auto operator()(const T& v) noexcept
        -> decltype(v <= static_cast<T>(0), static_cast<const char*>(nullptr))
    {
        if (!(v <= static_cast<T>(0))) {
            return "value not lesser than nor equal to zero";
        }
        return {};
    }
};

/// @ingroup CheckedTypes
/// @brief Non-positive constrained value type.
template <typename T>
using NonPositive = Checked<T, NonPositiveChecker<T>>;

/// @ingroup CheckedTypes
/// @brief Fast failing non-positive constrained value type.
template <typename T>
using NonPositiveFF = Checked<T, NonPositiveChecker<T>, true>;

static_assert(std::is_default_constructible<NonPositive<int>>::value);

} // namespace playrho

#endif // PLAYRHO_NONPOSITIVE_HPP
