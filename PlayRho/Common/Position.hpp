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

#ifndef PLAYRHO_COMMON_POSITION_HPP
#define PLAYRHO_COMMON_POSITION_HPP

#include <PlayRho/Common/Templates.hpp>
#include <PlayRho/Common/Settings.hpp>
#include <PlayRho/Common/Vector2.hpp>

namespace playrho {
    
    /// @brief 2-D positional data structure.
    /// @details A 2-element length and angle pair suitable for representing a linear and
    ///   angular position in 2-D.
    /// @note This structure is likely to be 12-bytes large (at least on 64-bit platforms).
    struct Position2D
    {
        Length2 linear; ///< Linear position.
        Angle angular; ///< Angular position.
    };
    
    /// @brief Determines if the given value is valid.
    /// @relatedalso Position2D
    template <>
    PLAYRHO_CONSTEXPR inline bool IsValid(const Position2D& value) noexcept
    {
        return IsValid(value.linear) && IsValid(value.angular);
    }
    
    /// @brief Equality operator.
    /// @relatedalso Position2D
    PLAYRHO_CONSTEXPR inline bool operator==(const Position2D& lhs, const Position2D& rhs)
    {
        return (lhs.linear == rhs.linear) && (lhs.angular == rhs.angular);
    }
    
    /// @brief Inequality operator.
    /// @relatedalso Position2D
    PLAYRHO_CONSTEXPR inline bool operator!=(const Position2D& lhs, const Position2D& rhs)
    {
        return (lhs.linear != rhs.linear) || (lhs.angular != rhs.angular);
    }
    
    /// @brief Negation operator.
    /// @relatedalso Position2D
    PLAYRHO_CONSTEXPR inline Position2D operator- (const Position2D& value)
    {
        return Position2D{-value.linear, -value.angular};
    }
    
    /// @brief Positive operator.
    /// @relatedalso Position2D
    PLAYRHO_CONSTEXPR inline Position2D operator+ (const Position2D& value)
    {
        return value;
    }
    
    /// @brief Addition assignment operator.
    /// @relatedalso Position2D
    PLAYRHO_CONSTEXPR inline Position2D& operator+= (Position2D& lhs, const Position2D& rhs)
    {
        lhs.linear += rhs.linear;
        lhs.angular += rhs.angular;
        return lhs;
    }
    
    /// @brief Addition operator.
    /// @relatedalso Position2D
    PLAYRHO_CONSTEXPR inline Position2D operator+ (const Position2D& lhs, const Position2D& rhs)
    {
        return Position2D{lhs.linear + rhs.linear, lhs.angular + rhs.angular};
    }
    
    /// @brief Subtraction assignment operator.
    /// @relatedalso Position2D
    PLAYRHO_CONSTEXPR inline Position2D& operator-= (Position2D& lhs, const Position2D& rhs)
    {
        lhs.linear -= rhs.linear;
        lhs.angular -= rhs.angular;
        return lhs;
    }
    
    /// @brief Subtraction operator.
    /// @relatedalso Position2D
    PLAYRHO_CONSTEXPR inline Position2D operator- (const Position2D& lhs, const Position2D& rhs)
    {
        return Position2D{lhs.linear - rhs.linear, lhs.angular - rhs.angular};
    }
    
    /// @brief Multiplication operator.
    PLAYRHO_CONSTEXPR inline Position2D operator* (const Position2D& pos, const Real scalar)
    {
        return Position2D{pos.linear * scalar, pos.angular * scalar};
    }
    
    /// @brief Multiplication operator.
    /// @relatedalso Position2D
    PLAYRHO_CONSTEXPR inline Position2D operator* (const Real scalar, const Position2D& pos)
    {
        return Position2D{pos.linear * scalar, pos.angular * scalar};
    }
    
    /// Gets the position between two positions at a given unit interval.
    /// @param pos0 Position at unit interval value of 0.
    /// @param pos1 Position at unit interval value of 1.
    /// @param beta Unit interval (value between 0 and 1) of travel between position 0 and
    ///   position 1.
    /// @return position 0 if <code>pos0 == pos1</code> or <code>beta == 0</code>,
    ///   position 1 if <code>beta == 1</code>, or at the given unit interval value
    ///   between position 0 and position 1.
    /// @relatedalso Position2D
    PLAYRHO_CONSTEXPR inline Position2D GetPosition(const Position2D pos0, const Position2D pos1,
                                   const Real beta) noexcept
    {
        assert(IsValid(pos0));
        assert(IsValid(pos1));
        assert(IsValid(beta));

        // Note: have to be careful how this is done.
        //   If pos0 == pos1 then return value should always be equal to pos0 too.
        //   But if Real is float, pos0 * (1 - beta) + pos1 * beta can fail this requirement.
        //   Meanwhile, pos0 + (pos1 - pos0) * beta always works.
        
        // pos0 * (1 - beta) + pos1 * beta
        // pos0 - pos0 * beta + pos1 * beta
        // pos0 + (pos1 * beta - pos0 * beta)
        // pos0 + (pos1 - pos0) * beta
        return pos0 + (pos1 - pos0) * beta;
    }
    
} // namespace playrho

#endif // PLAYRHO_COMMON_POSITION_HPP
