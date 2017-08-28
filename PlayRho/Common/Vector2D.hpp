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

#ifndef Vec2_hpp
#define Vec2_hpp

#include <PlayRho/Common/Settings.hpp>
#include <PlayRho/Common/InvalidArgument.hpp>
#include <PlayRho/Common/Vector.hpp>

namespace playrho
{
    /// @brief Vector with 2-dimensions.
    /// @note This is just a C++11 alias template for 2-dimensional uses of the Vector template.
    template <typename T>
    using Vector2D = Vector<2, T>;
    
    /// @brief Vector 2D of Real.
    /// @note This data structure is 2-times the size of the <code>Real</code> type
    ///   (or 8 using Real of float).
    using Real2 = Vector2D<Real>;

    /// @brief Vector 2D of Precise.
    /// @note This data structure is 2-times the size of the <code>Precise</code> type
    ///   (or 16 using Precise of double).
    using Precise2 = Vector2D<Precise>;

    /// @brief Vector 2D of Integer.
    /// @note This data structure is 2-times the size of the <code>Integer</code> type
    ///   (or 8 using Integer of long).
    using Integer2 = Vector2D<Integer>;

    /// An all zero Real2 value.
    /// @see Real2.
    constexpr auto Real2Zero = Real2{ 0, 0 };

    /// An all zero Precise2 value.
    /// @see Precise2.
    constexpr auto Precise2Zero = Precise2{ 0, 0 };

    /// An all zero Integer2 value.
    /// @see Integer2.
    constexpr auto Integer2Zero = Integer2{ 0, 0 };

    /// @brief 2D vector for the Length unit-type.
    /// @details A 2-dimensional location vector.
    using Length2D = Vector2D<Length>;

    /// @brief 2D vector for the LinearVelocity unit-type.
    /// @details A 2-dimensional speed vector.
    using LinearVelocity2D = Vector2D<LinearVelocity>;
    
    /// @brief 2D vector for the LinearAcceleration unit-type.
    /// @details A 2-dimensional acceleration vector.
    using LinearAcceleration2D = Vector2D<LinearAcceleration>;
    
    /// @brief 2D vector for the Force unit-type.
    /// @details A 2-dimensional force vector.
    using Force2D = Vector2D<Force>;
    
    /// @brief 2D vector for the Momentum unit-type.
    /// @details A 2-dimensional momentum vector.
    using Momentum2D = Vector2D<Momentum>;
        
    /// @brief Earthly gravity.
    /// @details An approximation of Earth's average gravity at sea-level in 2-dimensions.
    constexpr auto EarthlyGravity = LinearAcceleration2D{
        Real{0} * MeterPerSquareSecond,
        Real{-9.8f} * MeterPerSquareSecond
    };

    constexpr inline Real2 GetVec2(const Vector2D<Real> value)
    {
        return Real2(value);
    }

    template <>
    constexpr inline Real2 GetInvalid() noexcept
    {
        return Real2{GetInvalid<Real>(), GetInvalid<Real>()};
    }

    /// Does this vector contain finite coordinates?
    template <typename TYPE>
    constexpr inline bool IsValid(const Vector2D<TYPE>& value) noexcept
    {
        return IsValid(Get<0>(value)) && IsValid(Get<1>(value));
    }
    
    template <typename TYPE>
    constexpr bool operator == (const Vector2D<TYPE> a, const Vector2D<TYPE> b) noexcept
    {
        return (Get<0>(a) == Get<0>(b)) && (Get<1>(a) == Get<1>(b));
    }
    
    template <typename TYPE>
    constexpr bool operator != (const Vector2D<TYPE> a, const Vector2D<TYPE> b) noexcept
    {
        return !(a == b);
    }
    
    /// Increment the left hand side value by the right hand side value.
    template <typename TYPE>
    constexpr Vector2D<TYPE>& operator += (Vector2D<TYPE>& lhs, const Vector2D<TYPE> rhs) noexcept
    {
        Get<0>(lhs) += Get<0>(rhs);
        Get<1>(lhs) += Get<1>(rhs);
        return lhs;
    }
    
    /// Decrement the left hand side value by the right hand side value.
    template <typename TYPE>
    constexpr Vector2D<TYPE>& operator -= (Vector2D<TYPE>& lhs, const Vector2D<TYPE> rhs) noexcept
    {
        Get<0>(lhs) -= Get<0>(rhs);
        Get<1>(lhs) -= Get<1>(rhs);
        return lhs;
    }

    template <typename TYPE>
    constexpr Vector2D<TYPE>& operator *= (Vector2D<TYPE>& lhs, const Real rhs) noexcept
    {
        Get<0>(lhs) *= rhs;
        Get<1>(lhs) *= rhs;
        return lhs;
    }
    
    template <typename TYPE>
    constexpr Vector2D<TYPE>& operator /= (Vector2D<TYPE>& lhs, const Real rhs) noexcept
    {
        Get<0>(lhs) /= rhs;
        Get<1>(lhs) /= rhs;
        return lhs;
    }
    
    template <typename T>
    constexpr auto operator+ (const Vector2D<T> v) noexcept
    {
        return Vector2D<T>{+Get<0>(v), +Get<1>(v)};
    }

    template <typename T>
    constexpr auto operator- (const Vector2D<T> v) noexcept
    {
        return Vector2D<T>{-Get<0>(v), -Get<1>(v)};
    }

    /// Add two vectors component-wise.
    template <typename TYPE>
    constexpr Vector2D<TYPE> operator + (Vector2D<TYPE> lhs, const Vector2D<TYPE> rhs) noexcept
    {
        return lhs += rhs;
    }
    
    /// Subtract two vectors component-wise.
    template <typename TYPE>
    constexpr Vector2D<TYPE> operator - (Vector2D<TYPE> lhs, const Vector2D<TYPE> rhs) noexcept
    {
        return lhs -= rhs;
    }
    
    template <typename TYPE1, typename TYPE2, typename OUT_TYPE = decltype(TYPE1{} * TYPE2{})>
    constexpr inline Vector2D<OUT_TYPE> operator * (const TYPE1 s, Vector2D<TYPE2> a) noexcept
    {
        return Vector2D<OUT_TYPE>{Get<0>(a) * s, Get<1>(a) * s};
    }
    
    template <typename TYPE1, typename TYPE2, typename OUT_TYPE = decltype(TYPE1{} * TYPE2{})>
    constexpr inline Vector2D<OUT_TYPE> operator * (Vector2D<TYPE1> a, const TYPE2 s) noexcept
    {
        return Vector2D<OUT_TYPE>{Get<0>(a) * s, Get<1>(a) * s};
    }
    
    template <typename TYPE1, typename TYPE2, typename OUT_TYPE = decltype(TYPE1{} / TYPE2{})>
    constexpr Vector2D<OUT_TYPE> operator/ (Vector2D<TYPE1> a, const TYPE2 s) noexcept
    {
        return Vector2D<OUT_TYPE>{Get<0>(a) / s, Get<1>(a) / s};
    }
    
#ifdef USE_BOOST_UNITS
    template <>
    constexpr Length2D GetInvalid() noexcept
    {
        return Length2D{GetInvalid<Length>(), GetInvalid<Length>()};
    }
    
    template <>
    constexpr LinearVelocity2D GetInvalid() noexcept
    {
        return LinearVelocity2D{GetInvalid<LinearVelocity>(), GetInvalid<LinearVelocity>()};
    }
    
    template <>
    constexpr Force2D GetInvalid() noexcept
    {
        return Force2D{GetInvalid<Force>(), GetInvalid<Force>()};
    }
    
    template <>
    constexpr Momentum2D GetInvalid() noexcept
    {
        return Momentum2D{GetInvalid<Momentum>(), GetInvalid<Momentum>()};
    }
    
    constexpr inline Real2 GetVec2(const Length2D value)
    {
        return Real2{
            Get<0>(value) / Meter,
            Get<1>(value) / Meter
        };
    }
    
    constexpr inline Real2 GetVec2(const LinearVelocity2D value)
    {
        return Real2{
            Get<0>(value) / MeterPerSecond,
            Get<1>(value) / MeterPerSecond
        };
    }
    
    constexpr inline Real2 GetVec2(const Momentum2D value)
    {
        return Real2{
            Get<0>(value) / (Kilogram * MeterPerSecond),
            Get<1>(value) / (Kilogram * MeterPerSecond)
        };
    }
    
    constexpr inline Real2 GetVec2(const Force2D value)
    {
        return Real2{
            Get<0>(value) / Newton,
            Get<1>(value) / Newton
        };
    }
#endif
} // namespace playrho

#endif /* Vec2_hpp */
