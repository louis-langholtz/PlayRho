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

#ifndef Position_hpp
#define Position_hpp

#include <PlayRho/Common/Settings.hpp>
#include <PlayRho/Common/Vector2D.hpp>

namespace playrho
{
    
    /// @brief Positional data structure.
    /// @note This structure is likely to be 12-bytes large (at least on 64-bit platforms).
    struct Position
    {
        Length2D linear; ///< Linear position.
        Angle angular; ///< Angular position.
    };
    
    template <>
    constexpr inline bool IsValid(const Position& value) noexcept
    {
        return IsValid(value.linear) && IsValid(value.angular);
    }
    
    constexpr inline bool operator==(const Position& lhs, const Position& rhs)
    {
        return (lhs.linear == rhs.linear) && (lhs.angular == rhs.angular);
    }
    
    constexpr inline bool operator!=(const Position& lhs, const Position& rhs)
    {
        return (lhs.linear != rhs.linear) || (lhs.angular != rhs.angular);
    }
    
    constexpr inline Position operator- (const Position& value)
    {
        return Position{-value.linear, -value.angular};
    }
    
    constexpr inline Position operator+ (const Position& value)
    {
        return value;
    }
    
    constexpr inline Position& operator+= (Position& lhs, const Position& rhs)
    {
        lhs.linear += rhs.linear;
        lhs.angular += rhs.angular;
        return lhs;
    }
    
    constexpr inline Position operator+ (const Position& lhs, const Position& rhs)
    {
        return Position{lhs.linear + rhs.linear, lhs.angular + rhs.angular};
    }
    
    constexpr inline Position& operator-= (Position& lhs, const Position& rhs)
    {
        lhs.linear -= rhs.linear;
        lhs.angular -= rhs.angular;
        return lhs;
    }
    
    constexpr inline Position operator- (const Position& lhs, const Position& rhs)
    {
        return Position{lhs.linear - rhs.linear, lhs.angular - rhs.angular};
    }
    
    constexpr inline Position operator* (const Position& pos, const Real scalar)
    {
        return Position{{pos.linear.x * scalar, pos.linear.y * scalar}, pos.angular * scalar};
    }
    
    constexpr inline Position operator* (const Real scalar, const Position& pos)
    {
        return Position{{pos.linear.x * scalar, pos.linear.y * scalar}, pos.angular * scalar};
    }
    
    /// Gets the position between two positions at a given unit interval.
    /// @param pos0 Position at unit interval value of 0.
    /// @param pos1 Position at unit interval value of 1.
    /// @param beta Unit interval (value between 0 and 1) of travel between pos0 and pos1.
    /// @return pos0 if pos0 == pos1 or beta == 0, pos1 if beta == 1, or at the given
    ///   unit interval value between pos0 and pos1.
    inline Position GetPosition(const Position pos0, const Position pos1, const Real beta) noexcept
    {
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

}

#endif /* Position_hpp */
