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

#ifndef PLAYRHO_COMMON_TRANSFORMATION_HPP
#define PLAYRHO_COMMON_TRANSFORMATION_HPP

#include <PlayRho/Common/Settings.hpp>
#include <PlayRho/Common/Vector2D.hpp>
#include <PlayRho/Common/UnitVec2.hpp>

/// @file
/// Definition of the Transformation class and free functions directly associated with it.

namespace playrho
{
    
    /// @brief Describes a geometric transformation.
    /// @details A transform contains translation and rotation. It is used to represent
    ///   the position and orientation of rigid frames.
    /// @note The default transformation is the identity transformation - the transformation
    ///   which neither translates nor rotates a location.
    /// @note This data structure is 16-bytes large (on at least one 64-bit platform).
    struct Transformation
    {
        Length2D p = Length2D{}; ///< Translational portion of the transformation. 8-bytes.
        UnitVec2 q = UnitVec2::GetRight(); ///< Rotational portion of the transformation. 8-bytes.
    };
    
    /// @brief Identity transformation value.
    constexpr auto Transform_identity = Transformation{
        Length2D{Real(0) * Meter, Real(0) * Meter}, UnitVec2::GetRight()
    };
    
    /// @brief Determines if the given value is valid.
    /// @relatedalso Transformation
    template <>
    constexpr inline bool IsValid(const Transformation& value) noexcept
    {
        return IsValid(value.p) && IsValid(value.q);
    }
    
    /// @brief Equality operator.
    /// @relatedalso Transformation
    constexpr inline bool operator== (Transformation lhs, Transformation rhs) noexcept
    {
        return (lhs.p == rhs.p) && (lhs.q == rhs.q);
    }
    
    /// @brief Inequality operator.
    /// @relatedalso Transformation
    constexpr inline bool operator!= (Transformation lhs, Transformation rhs) noexcept
    {
        return (lhs.p != rhs.p) || (lhs.q != rhs.q);
    }

} // namespace playrho

#endif // PLAYRHO_COMMON_TRANSFORMATION_HPP
