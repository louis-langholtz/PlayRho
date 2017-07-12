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

#ifndef Vector3D_hpp
#define Vector3D_hpp

#include <PlayRho/Common/Settings.hpp>

namespace playrho
{
    /// Vector 3D.
    template <typename TYPE>
    struct Vector3D
    {
        using size_type = std::size_t;
        using data_type = TYPE;

        constexpr auto operator+ () const noexcept { return Vector3D<data_type>{+x, +y, +z}; }

        /// Negate this vector.
        constexpr auto operator- () const noexcept { return Vector3D<data_type>{-x, -y, -z}; }

        data_type x, y, z;
    };
    
    template <typename TYPE>
    constexpr inline typename Vector3D<TYPE>::data_type GetX(const Vector3D<TYPE> value)
    {
        return value.x;
    }
    
    template <typename TYPE>
    constexpr inline typename Vector3D<TYPE>::data_type GetY(const Vector3D<TYPE> value)
    {
        return value.y;
    }

    template <typename TYPE>
    constexpr inline typename Vector3D<TYPE>::data_type GetZ(const Vector3D<TYPE> value)
    {
        return value.z;
    }
    
    template <typename TYPE>
    constexpr inline bool operator == (const Vector3D<TYPE> a, const Vector3D<TYPE> b) noexcept
    {
        return (a.x == b.x) && (a.y == b.y) && (a.z == b.z);
    }
    
    template <typename TYPE>
    constexpr inline bool operator != (const Vector3D<TYPE> a, const Vector3D<TYPE> b) noexcept
    {
        return !(a == b);
    }
    
    /// Add two vectors component-wise.
    template <typename TYPE>
    constexpr inline Vector3D<TYPE> operator + (const Vector3D<TYPE> a, const Vector3D<TYPE> b) noexcept
    {
        return Vector3D<TYPE>{a.x + b.x, a.y + b.y, a.z + b.z};
    }
    
    /// Subtract two vectors component-wise.
    template <typename TYPE>
    constexpr inline Vector3D<TYPE> operator - (const Vector3D<TYPE> a, const Vector3D<TYPE> b) noexcept
    {
        return Vector3D<TYPE>{a.x - b.x, a.y - b.y, a.z - b.z};
    }
    
    /// Increment the left hand side value by the right hand side value.
    template <typename TYPE>
    constexpr Vector3D<TYPE>& operator += (Vector3D<TYPE>& lhs, const Vector3D<TYPE> rhs) noexcept
    {
        lhs.x += rhs.x;
        lhs.y += rhs.y;
        lhs.z += rhs.z;
        return lhs;
    }
    
    /// Decrement the left hand side value by the right hand side value.
    template <typename TYPE>
    constexpr Vector3D<TYPE>& operator -= (Vector3D<TYPE>& lhs, const Vector3D<TYPE> rhs) noexcept
    {
        lhs.x -= rhs.x;
        lhs.y -= rhs.y;
        lhs.z -= rhs.z;
        return lhs;
    }
    
    template <typename TYPE>
    constexpr Vector3D<TYPE>& operator *= (Vector3D<TYPE>& lhs, const Real rhs) noexcept
    {
        lhs.x *= rhs;
        lhs.y *= rhs;
        lhs.z *= rhs;
        return lhs;
    }
    
    template <typename TYPE>
    constexpr Vector3D<TYPE>& operator /= (Vector3D<TYPE>& lhs, const Real rhs) noexcept
    {
        lhs.x /= rhs;
        lhs.y /= rhs;
        lhs.z /= rhs;
        return lhs;
    }
    
    template <typename TYPE1, typename TYPE2, typename OUT_TYPE = decltype(TYPE1{0} * TYPE2{0})>
    constexpr inline Vector3D<OUT_TYPE> operator * (const TYPE1 s, const Vector3D<TYPE2> a) noexcept
    {
        return Vector3D<OUT_TYPE>{s * a.x, s * a.y, s * a.z};
    }
    
    template <typename TYPE1, typename TYPE2, typename OUT_TYPE = decltype(TYPE1{0} * TYPE2{0})>
    constexpr inline Vector3D<OUT_TYPE> operator * (const Vector3D<TYPE1> a, const TYPE2 s) noexcept
    {
        return Vector3D<OUT_TYPE>{a.x * s, a.y * s, a.z * s};
    }
    
    template <typename TYPE1, typename TYPE2, typename OUT_TYPE = decltype(TYPE1{0} / TYPE2{0})>
    constexpr Vector3D<OUT_TYPE> operator/ (const Vector3D<TYPE1> a, const TYPE2 s) noexcept
    {
        return Vector3D<OUT_TYPE>{a.x / s, a.y / s, a.z / s};
    }

    /// A 3D column vector with 3 elements.
    /// @note This data structure is 3 times the size of <code>Real</code> -
    ///   i.e. 12-bytes (with 4-byte Real).
    using Vec3 = Vector3D<Real>;

    /// An all zero Vec3 value.
    /// @see Vec3.
    constexpr auto Vec3_zero = Vec3{0, 0, 0};
    
    template <>
    constexpr inline Vec3 GetInvalid() noexcept
    {
        return Vec3{GetInvalid<Real>(), GetInvalid<Real>(), GetInvalid<Real>()};
    }
    
    /// Does this vector contain finite coordinates?
    template <>
    constexpr inline bool IsValid(const Vec3& value) noexcept
    {
        return IsValid(value.x) && IsValid(value.y) && IsValid(value.z);
    }
    
} // namespace playrho

#endif /* Vector3D_hpp */
