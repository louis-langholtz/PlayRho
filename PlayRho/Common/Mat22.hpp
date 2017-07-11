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

#ifndef Mat22_hpp
#define Mat22_hpp

#include <PlayRho/Common/Settings.hpp>
#include <PlayRho/Common/Vector2D.hpp>

namespace box2d
{
    
    /// @brief A 2-by-2 matrix.
    /// @details Stored in column-major order.
    /// @note This structure is likely about 16-bytes large.
    struct Mat22
    {
        /// The default constructor does nothing (for performance).
        Mat22() noexcept = default;
        
        /// Construct this matrix using columns.
        constexpr Mat22(const Vec2 c1, const Vec2 c2) noexcept: ex{c1}, ey{c2} {}
        
        /// Construct this matrix using scalars.
        constexpr Mat22(Real a11, Real a12, Real a21, Real a22) noexcept: ex{a11, a21}, ey{a12, a22} {}
        
        Vec2 ex, ey;
    };
    
    template <>
    constexpr inline bool IsValid(const Mat22& value) noexcept
    {
        return IsValid(value.ex) && IsValid(value.ey);
    }
    
    template <>
    constexpr inline Mat22 GetInvalid() noexcept
    {
        return Mat22{GetInvalid<Vec2>(), GetInvalid<Vec2>()};
    }
    
    /// An all zero Mat22 value.
    /// @see Mat22.
    constexpr auto Mat22_zero = Mat22(Vec2_zero, Vec2_zero);

    /// Identity value for Mat22 objects.
    /// @see Mat22.
    constexpr auto Mat22_identity = Mat22(Vec2{1, 0}, Vec2{0, 1});
    
    constexpr inline Mat22 operator + (const Mat22 A, const Mat22 B) noexcept
    {
        return Mat22{A.ex + B.ex, A.ey + B.ey};
    }

}

#endif /* Mat22_hpp */
