/*
 * Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#ifndef PLAYRHO_COMMON_VECTOR2_HPP
#define PLAYRHO_COMMON_VECTOR2_HPP

#include <PlayRho/Common/Settings.hpp>
#include <PlayRho/Common/InvalidArgument.hpp>
#include <PlayRho/Common/Vector.hpp>

namespace playrho
{
    /// @brief Vector with 2-elements.
    /// @note This is just a C++11 alias template for 2-element uses of the Vector template.
    template <typename T>
    using Vector2 = Vector<2, T>;
    
    /// @brief Vector 2D of Real.
    /// @note This data structure is two-times the size of the <code>Real</code> type
    ///   (or 8 using Real of float).
    using Vec2 = Vector2<Real>;
    
    /// An all zero Vec2 value.
    /// @see Vec2.
    constexpr auto Vec2_zero = Vec2{0, 0};

    /// @brief 2D vector for the Length unit-type.
    /// @details A 2-dimensional location vector.
    using Length2D = Vector2<Length>;

    /// @brief 2D vector for the LinearVelocity unit-type.
    /// @details A 2-dimensional speed vector.
    using LinearVelocity2D = Vector2<LinearVelocity>;
    
    /// @brief 2D vector for the LinearAcceleration unit-type.
    /// @details A 2-dimensional acceleration vector.
    using LinearAcceleration2D = Vector2<LinearAcceleration>;
    
    /// @brief 2D vector for the Force unit-type.
    /// @details A 2-dimensional force vector.
    using Force2D = Vector2<Force>;
    
    /// @brief 2D vector for the Momentum unit-type.
    /// @details A 2-dimensional momentum vector.
    using Momentum2D = Vector2<Momentum>;
        
    /// @brief Earthly gravity.
    /// @details An approximation of Earth's average gravity at sea-level in 2-dimensions.
    constexpr auto EarthlyGravity = LinearAcceleration2D{
        Real{0} * MeterPerSquareSecond,
        Real{-9.8f} * MeterPerSquareSecond
    };

    /// @brief Gets the given value as a Vec2.
    constexpr inline Vec2 GetVec2(const Vector2<Real> value)
    {
        return {value};
    }

    /// @brief Gets an invalid value for the Vec2 type.
    template <>
    constexpr inline Vec2 GetInvalid() noexcept
    {
        return Vec2{GetInvalid<Real>(), GetInvalid<Real>()};
    }

    /// @brief Determines whether the given vector contains finite coordinates.
    template <typename TYPE>
    constexpr inline bool IsValid(const Vector2<TYPE>& value) noexcept
    {
        return IsValid(Get<0>(value)) && IsValid(Get<1>(value));
    }
    
    /// @brief Equality operator.
    template <typename TYPE>
    constexpr bool operator== (const Vector2<TYPE> a, const Vector2<TYPE> b) noexcept
    {
        return (Get<0>(a) == Get<0>(b)) && (Get<1>(a) == Get<1>(b));
    }
    
    /// @brief Inequality operator.
    template <typename TYPE>
    constexpr bool operator!= (const Vector2<TYPE> a, const Vector2<TYPE> b) noexcept
    {
        return !(a == b);
    }
    
    /// Increment the left hand side value by the right hand side value.
    template <typename TYPE>
    constexpr Vector2<TYPE>& operator += (Vector2<TYPE>& lhs, const Vector2<TYPE> rhs) noexcept
    {
        Get<0>(lhs) += Get<0>(rhs);
        Get<1>(lhs) += Get<1>(rhs);
        return lhs;
    }
    
    /// Decrement the left hand side value by the right hand side value.
    template <typename TYPE>
    constexpr Vector2<TYPE>& operator -= (Vector2<TYPE>& lhs, const Vector2<TYPE> rhs) noexcept
    {
        Get<0>(lhs) -= Get<0>(rhs);
        Get<1>(lhs) -= Get<1>(rhs);
        return lhs;
    }

    /// @brief Multiplication assignment operator.
    template <typename TYPE>
    constexpr Vector2<TYPE>& operator*= (Vector2<TYPE>& lhs, const Real rhs) noexcept
    {
        Get<0>(lhs) *= rhs;
        Get<1>(lhs) *= rhs;
        return lhs;
    }
    
    /// @brief Division assignment operator.
    template <typename TYPE>
    constexpr Vector2<TYPE>& operator/= (Vector2<TYPE>& lhs, const Real rhs) noexcept
    {
        Get<0>(lhs) /= rhs;
        Get<1>(lhs) /= rhs;
        return lhs;
    }
    
    /// @brief Positive operator.
    template <typename T>
    constexpr auto operator+ (const Vector2<T> v) noexcept
    {
        return Vector2<T>{+Get<0>(v), +Get<1>(v)};
    }

    /// @brief Negation operator.
    template <typename T>
    constexpr auto operator- (const Vector2<T> v) noexcept
    {
        return Vector2<T>{-Get<0>(v), -Get<1>(v)};
    }

    /// Add two vectors component-wise.
    template <typename TYPE>
    constexpr Vector2<TYPE> operator + (Vector2<TYPE> lhs, const Vector2<TYPE> rhs) noexcept
    {
        return lhs += rhs;
    }
    
    /// Subtract two vectors component-wise.
    template <typename TYPE>
    constexpr Vector2<TYPE> operator - (Vector2<TYPE> lhs, const Vector2<TYPE> rhs) noexcept
    {
        return lhs -= rhs;
    }
    
    /// @brief Multiplication operator.
    template <typename TYPE1, typename TYPE2, typename OUT_TYPE = decltype(TYPE1{} * TYPE2{})>
    constexpr inline Vector2<OUT_TYPE> operator* (const TYPE1 s, Vector2<TYPE2> a) noexcept
    {
        return Vector2<OUT_TYPE>{Get<0>(a) * s, Get<1>(a) * s};
    }
    
    /// @brief Multiplication operator.
    template <typename TYPE1, typename TYPE2, typename OUT_TYPE = decltype(TYPE1{} * TYPE2{})>
    constexpr inline Vector2<OUT_TYPE> operator* (Vector2<TYPE1> a, const TYPE2 s) noexcept
    {
        return Vector2<OUT_TYPE>{Get<0>(a) * s, Get<1>(a) * s};
    }
    
    /// @brief Division operator.
    template <typename TYPE1, typename TYPE2, typename OUT_TYPE = decltype(TYPE1{} / TYPE2{})>
    constexpr Vector2<OUT_TYPE> operator/ (Vector2<TYPE1> a, const TYPE2 s) noexcept
    {
        return Vector2<OUT_TYPE>{Get<0>(a) / s, Get<1>(a) / s};
    }
    
#ifdef USE_BOOST_UNITS
    /// @brief Gets an invalid value for the Length2D type.
    template <>
    constexpr Length2D GetInvalid() noexcept
    {
        return Length2D{GetInvalid<Length>(), GetInvalid<Length>()};
    }
    
    /// @brief Gets an invalid value for the LinearVelocity2D type.
    template <>
    constexpr LinearVelocity2D GetInvalid() noexcept
    {
        return LinearVelocity2D{GetInvalid<LinearVelocity>(), GetInvalid<LinearVelocity>()};
    }
    
    /// @brief Gets an invalid value for the Force2D type.
    template <>
    constexpr Force2D GetInvalid() noexcept
    {
        return Force2D{GetInvalid<Force>(), GetInvalid<Force>()};
    }
    
    /// @brief Gets an invalid value for the Momentum2D type.
    template <>
    constexpr Momentum2D GetInvalid() noexcept
    {
        return Momentum2D{GetInvalid<Momentum>(), GetInvalid<Momentum>()};
    }
    
    constexpr inline Vec2 GetVec2(const Length2D value)
    {
        return Vec2{
            Get<0>(value) / Meter,
            Get<1>(value) / Meter
        };
    }
    
    constexpr inline Vec2 GetVec2(const LinearVelocity2D value)
    {
        return Vec2{
            Get<0>(value) / MeterPerSecond,
            Get<1>(value) / MeterPerSecond
        };
    }
    
    constexpr inline Vec2 GetVec2(const Momentum2D value)
    {
        return Vec2{
            Get<0>(value) / (Kilogram * MeterPerSecond),
            Get<1>(value) / (Kilogram * MeterPerSecond)
        };
    }
    
    constexpr inline Vec2 GetVec2(const Force2D value)
    {
        return Vec2{
            Get<0>(value) / Newton,
            Get<1>(value) / Newton
        };
    }
#endif
} // namespace playrho

#endif // PLAYRHO_COMMON_VECTOR2_HPP
