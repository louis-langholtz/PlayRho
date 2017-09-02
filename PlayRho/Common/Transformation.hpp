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

#ifndef Transformation_hpp
#define Transformation_hpp

#include <PlayRho/Common/Settings.hpp>
#include <PlayRho/Common/Vector2D.hpp>
#include <PlayRho/Common/UnitVec2.hpp>

namespace playrho
{
    
    /// Transformation.
    /// @details
    /// A transform contains translation and rotation. It is used to represent
    /// the position and orientation of rigid frames.
    /// @note This data structure is 16-bytes large (on at least one 64-bit platform).
    struct Transformation
    {
        Length2D p; ///< Translational portion of the transformation. 8-bytes.
        UnitVec2 q; ///< Rotational portion of the transformation. 8-bytes.
    };
    
    auto Transform_identity = Transformation{
        Length2D{ Real(0) * Meter, Real(0) * Meter }, UnitVec2::GetRight()
    };
    
    template <>
    constexpr inline bool IsValid(const Transformation& value) noexcept
    {
        return IsValid(value.p) && IsValid(value.q);
    }
    
    constexpr inline bool operator == (Transformation lhs, Transformation rhs) noexcept
    {
        return (lhs.p == rhs.p) && (lhs.q == rhs.q);
    }
    
    constexpr inline bool operator != (Transformation lhs, Transformation rhs) noexcept
    {
        return (lhs.p != rhs.p) || (lhs.q != rhs.q);
    }

}

#endif /* Transformation_hpp */
