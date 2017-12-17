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

#ifndef PLAYRHO_COMMON_FIXEDMATH_HPP
#define PLAYRHO_COMMON_FIXEDMATH_HPP

#include <PlayRho/Common/Fixed.hpp>
#include <cmath>

namespace playrho {
    
    /// @defgroup FixedMath Mathematical Functions For Fixed Types
    /// @brief Common Mathematical Functions For Fixed Types.
    /// @sa Fixed
    /// @sa http://en.cppreference.com/w/cpp/numeric/math
    /// @{

    /// @brief Square root's the given value.
    /// @note This implementation isn't meant to be fast, only correct enough.
    /// @sa http://en.cppreference.com/w/cpp/numeric/math/sqrt
    template <typename BT, unsigned int FB>
    inline auto sqrt(Fixed<BT, FB> arg)
    {
        return static_cast<Fixed<BT, FB>>(std::sqrt(static_cast<double>(arg)));
    }
    
    /// @brief Gets whether the given value is normal - i.e. not 0 nor infinite.
    /// @sa http://en.cppreference.com/w/cpp/numeric/math/isnormal
    template <typename BT, unsigned int FB>
    inline bool isnormal(Fixed<BT, FB> arg)
    {
        return arg != Fixed<BT, FB>{0} && arg.isfinite();
    }
    
    /// @brief Computes the sine of the argument for Fixed types.
    /// @sa http://en.cppreference.com/w/cpp/numeric/math/sin
    template <typename BT, unsigned int FB>
    inline Fixed<BT, FB> sin(Fixed<BT, FB> arg)
    {
        return static_cast<Fixed<BT, FB>>(std::sin(static_cast<double>(arg)));
    }
    
    /// @brief Computes the cosine of the argument for Fixed types.
    /// @sa http://en.cppreference.com/w/cpp/numeric/math/cos
    template <typename BT, unsigned int FB>
    inline Fixed<BT, FB> cos(Fixed<BT, FB> arg)
    {
        return static_cast<Fixed<BT, FB>>(std::cos(static_cast<double>(arg)));
    }
    
    /// @brief Computes the arc tangent.
    /// @sa http://en.cppreference.com/w/cpp/numeric/math/atan2
    template <typename BT, unsigned int FB>
    inline Fixed<BT, FB> atan2(Fixed<BT, FB> y, Fixed<BT, FB> x)
    {
        return static_cast<Fixed<BT, FB>>(std::atan2(static_cast<double>(y), static_cast<double>(x)));
    }
    
    /// @brief Computes the value of the base number raised to the power of the exponent.
    /// @sa http://en.cppreference.com/w/cpp/numeric/math/pow
    template <typename BT, unsigned int FB>
    inline Fixed<BT, FB> pow(Fixed<BT, FB> base, Fixed<BT, FB> exp)
    {
        return static_cast<Fixed<BT, FB>>(std::pow(static_cast<double>(base),
                                                   static_cast<double>(exp)));
    }
    
    /// @brief Computes the value of the base number raised to the power of the exponent.
    /// @sa http://en.cppreference.com/w/cpp/numeric/math/pow
    template <typename BT, unsigned int FB>
    inline Fixed<BT, FB> pow(Fixed<BT, FB> base, double exp)
    {
        return static_cast<Fixed<BT, FB>>(std::pow(static_cast<double>(base), exp));
    }
    
    /// @brief Computes the square root of the sum of the squares.
    /// @sa http://en.cppreference.com/w/cpp/numeric/math/hypot
    template <typename BT, unsigned int FB>
    inline Fixed<BT, FB> hypot(Fixed<BT, FB> x, Fixed<BT, FB> y)
    {
        return static_cast<Fixed<BT, FB>>(std::hypot(static_cast<double>(x), static_cast<double>(y)));
    }
    
    /// @brief Rounds the given value.
    /// @sa http://en.cppreference.com/w/cpp/numeric/math/round
    template <typename BT, unsigned int FB>
    inline Fixed<BT, FB> round(Fixed<BT, FB> value) noexcept
    {
        const auto tmp = value + (Fixed<BT, FB>{1} / Fixed<BT, FB>{2});
        const auto truncated = static_cast<typename Fixed<BT, FB>::value_type>(tmp);
        return Fixed<BT, FB>{truncated, 0};
    }
    
    /// @brief Truncates the given value.
    /// @sa http://en.cppreference.com/w/c/numeric/math/trunc
    template <typename BT, unsigned int FB>
    inline Fixed<BT, FB> trunc(Fixed<BT, FB> arg)
    {
        return static_cast<Fixed<BT, FB>>(static_cast<long long>(arg));
    }
    
    /// @brief Determines whether the given value is negative.
    /// @sa http://en.cppreference.com/w/cpp/numeric/math/signbit
    template <typename BT, unsigned int FB>
    inline bool signbit(Fixed<BT, FB> value) noexcept
    {
        return value.getsign() < 0;
    }
    
    /// @brief Gets whether the given value is not-a-number.
    /// @sa http://en.cppreference.com/w/cpp/numeric/math/isnan
    template <typename BT, unsigned int FB>
    PLAYRHO_CONSTEXPR inline bool isnan(Fixed<BT, FB> value) noexcept
    {
        return value.Compare(0) == Fixed<BT, FB>::CmpResult::Incomparable;
    }
    
    /// @brief Gets whether the given value is finite.
    /// @sa http://en.cppreference.com/w/cpp/numeric/math/isfinite
    template <typename BT, unsigned int FB>
    inline bool isfinite(Fixed<BT, FB> value) noexcept
    {
        return (value > Fixed<BT, FB>::GetNegativeInfinity())
        && (value < Fixed<BT, FB>::GetInfinity());
    }
    
    /// @brief Next after function for Fixed types.
    /// @sa http://en.cppreference.com/w/cpp/numeric/math/nextafter
    template <typename BT, unsigned int FB>
    inline Fixed<BT, FB> nextafter(Fixed<BT, FB> from, Fixed<BT, FB> to) noexcept
    {
        if (from < to)
        {
            return static_cast<Fixed<BT, FB>>(from + Fixed<BT,FB>::GetMin());
        }
        if (from > to)
        {
            return static_cast<Fixed<BT, FB>>(from - Fixed<BT,FB>::GetMin());
        }
        return static_cast<Fixed<BT, FB>>(to);
    }
    
    /// @}

} // namespace playrho

#endif // PLAYRHO_COMMON_FIXEDMATH_HPP
