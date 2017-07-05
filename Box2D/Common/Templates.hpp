/*
 * Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

#ifndef Templates_hpp
#define Templates_hpp

#include <limits>

namespace box2d
{

    template <typename T>
    constexpr T GetInvalid() noexcept
    {
        static_assert(sizeof(T) == 0, "No available specialization");
    }

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
    
    template <>
    constexpr float GetInvalid() noexcept
    {
        return std::numeric_limits<float>::signaling_NaN();
    }
    
    template <>
    constexpr double GetInvalid() noexcept
    {
        return std::numeric_limits<double>::signaling_NaN();
    }
    
    template <>
    constexpr long double GetInvalid() noexcept
    {
        return std::numeric_limits<long double>::signaling_NaN();
    }
    
    template <>
    constexpr std::size_t GetInvalid() noexcept
    {
        return static_cast<std::size_t>(-1);
    }
    
    // IsValid template specializations.
    
    template <>
    constexpr inline bool IsValid(const std::size_t& x) noexcept
    {
        return x != GetInvalid<std::size_t>();
    }

} // namespace box2d

#endif /* Templates_hpp */
