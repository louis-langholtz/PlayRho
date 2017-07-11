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

#ifndef Mat33_hpp
#define Mat33_hpp

#include <PlayRho/Common/Settings.hpp>
#include <PlayRho/Common/Vector3D.hpp>

namespace box2d
{
    
    /// @brief A 3-by-3 matrix. Stored in column-major order.
    /// @note This data structure is 36-bytes large (on at least one 64-bit platform with 4-byte Real).
    struct Mat33
    {
        /// The default constructor does nothing (for performance).
        Mat33() noexcept = default;
        
        /// Construct this matrix using columns.
        constexpr Mat33(const Vec3 c1, const Vec3 c2, const Vec3 c3) noexcept:
        	ex{c1}, ey{c2}, ez{c3}
        {
            // Intentionally empty.
        }
        
        Vec3 ex, ey, ez;
    };
    
    constexpr auto Mat33_zero = Mat33(Vec3_zero, Vec3_zero, Vec3_zero);
    
}

#endif /* Mat33_hpp */
