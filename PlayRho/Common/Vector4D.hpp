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

#ifndef Vector4D_hpp
#define Vector4D_hpp

#include <PlayRho/Common/Settings.hpp>

namespace playrho
{
    /// @brief Vector with 4-dimensions.
    /// @note This is just a C++11 alias template for 4-dimensional uses of the Vector template.
    template <typename T>
    using Vector4D = Vector<4, T>;

    /// @brief Vector 4D of Real.
    /// @note This data structure is 4-times the size of the <code>Real</code> type
    ///   (or 16 using Real of float).
    using Vec4 = Vector4D<Real>;

    /// An all zero Real4 value.
    /// @see Real4.
    constexpr auto Real4Zero = Vec4{ 0, 0, 0, 0 };
    
    template <typename TYPE>
    constexpr inline bool operator == (const Vector4D<TYPE> a, const Vector4D<TYPE> b) noexcept
    {
        return (Get<0>(a) == Get<0>(b)) && (Get<1>(a) == Get<1>(b))
            && (Get<2>(a) == Get<2>(b) && (Get<3>(a) == Get<3>(b)));
    }
    
    template <typename TYPE>
    constexpr inline bool operator != (const Vector4D<TYPE> a, const Vector4D<TYPE> b) noexcept
    {
        return !(a == b);
    }
    
    /// Increment the left hand side value by the right hand side value.
    template <typename TYPE>
    constexpr Vector4D<TYPE>& operator += (Vector4D<TYPE>& lhs, const Vector4D<TYPE> rhs) noexcept
    {
        Get<0>(lhs) += Get<0>(rhs);
        Get<1>(lhs) += Get<1>(rhs);
        Get<2>(lhs) += Get<2>(rhs);
        Get<3>(lhs) += Get<3>(rhs);
        return lhs;
    }
    
    /// Decrement the left hand side value by the right hand side value.
    template <typename TYPE>
    constexpr Vector4D<TYPE>& operator -= (Vector4D<TYPE>& lhs, const Vector4D<TYPE> rhs) noexcept
    {
        Get<0>(lhs) -= Get<0>(rhs);
        Get<1>(lhs) -= Get<1>(rhs);
        Get<2>(lhs) -= Get<2>(rhs);
        Get<3>(lhs) -= Get<3>(rhs);
        return lhs;
    }
    
    template <typename TYPE>
    constexpr Vector4D<TYPE>& operator *= (Vector4D<TYPE>& lhs, const Real rhs) noexcept
    {
        Get<0>(lhs) *= rhs;
        Get<1>(lhs) *= rhs;
        Get<2>(lhs) *= rhs;
        Get<3>(lhs) *= rhs;
        return lhs;
    }
    
    template <typename TYPE>
    constexpr Vector4D<TYPE>& operator /= (Vector4D<TYPE>& lhs, const Real rhs) noexcept
    {
        Get<0>(lhs) /= rhs;
        Get<1>(lhs) /= rhs;
        Get<2>(lhs) /= rhs;
        Get<3>(lhs) /= rhs;
        return lhs;
    }
    
    template <typename T>
    constexpr auto operator+ (const Vector4D<T> v) noexcept
    {
        return Vector4D<T>{+Get<0>(v), +Get<1>(v), +Get<2>(v), +Get<3>(v)};
    }
    
    template <typename T>
    constexpr auto operator- (const Vector4D<T> v) noexcept
    {
        return Vector4D<T>{-Get<0>(v), -Get<1>(v), -Get<2>(v), -Get<3>(v)};
    }
    
    /// Add two vectors component-wise.
    template <typename TYPE>
    constexpr Vector4D<TYPE> operator + (Vector4D<TYPE> lhs, const Vector4D<TYPE> rhs) noexcept
    {
        return lhs += rhs;
    }
    
    /// Subtract two vectors component-wise.
    template <typename TYPE>
    constexpr Vector4D<TYPE> operator - (Vector4D<TYPE> lhs, const Vector4D<TYPE> rhs) noexcept
    {
        return lhs -= rhs;
    }
    
    template <typename TYPE1, typename TYPE2, typename OUT_TYPE = decltype(TYPE1{0} * TYPE2{0})>
    constexpr inline Vector4D<OUT_TYPE> operator * (const TYPE1 s, Vector4D<TYPE2> a) noexcept
    {
        return a *= s;
    }
    
    template <typename TYPE1, typename TYPE2, typename OUT_TYPE = decltype(TYPE1{0} * TYPE2{0})>
    constexpr inline Vector4D<OUT_TYPE> operator * (Vector4D<TYPE1> a, const TYPE2 s) noexcept
    {
        return a *= s;
    }
    
    template <typename TYPE1, typename TYPE2, typename OUT_TYPE = decltype(TYPE1{0} / TYPE2{0})>
    constexpr Vector4D<OUT_TYPE> operator/ (Vector4D<TYPE1> a, const TYPE2 s) noexcept
    {
        return a /= s;
    }
    
    template <>
    constexpr inline Vec4 GetInvalid() noexcept
    {
        return Vec4{GetInvalid<Real>(), GetInvalid<Real>(), GetInvalid<Real>() , GetInvalid<Real>() };
    }
    
    /// Does this vector contain finite coordinates?
    template <>
    constexpr inline bool IsValid(const Vec4& value) noexcept
    {
        return IsValid(Get<0>(value)) && IsValid(Get<1>(value)) && IsValid(Get<2>(value) && IsValid(Get<3>(value)));
    }
    
} // namespace playrho

#endif /* Vector4D_hpp */
