/*
 * Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

#ifndef Sweep_hpp
#define Sweep_hpp

#include <Box2D/Common/Settings.hpp>
#include <Box2D/Common/Position.hpp>
#include <Box2D/Common/Vector2D.hpp>

namespace box2d
{
    
    /// @brief Sweep.
    /// @details This describes the motion of a body/shape for TOI computation.
    ///   Shapes are defined with respect to the body origin, which may
    ///   not coincide with the center of mass. However, to support dynamics
    ///   we must interpolate the center of mass position.
    /// @note This data structure is likely 36-bytes (at least on 64-bit platforms).
    class Sweep
    {
    public:
        /// Default constructor.
        Sweep() = default;
        
        /// Copy constructor.
        constexpr Sweep(const Sweep& copy) = default;
        
        /// Initializing constructor.
        constexpr Sweep(const Position p0, const Position p1,
                        const Length2D lc = Vec2_zero * Meter, RealNum a0 = 0) noexcept:
        	pos0{p0}, pos1{p1}, localCenter{lc}, alpha0{a0}
        {
            assert(a0 >= 0);
            assert(a0 < 1);
        }
        
        /// Initializing constructor.
        constexpr explicit Sweep(const Position p, const Length2D lc = Vec2_zero * Meter): Sweep{p, p, lc, 0} {}
        
        /// Gets the local center of mass position.
        /// @note This value can only be set via a sweep constructed using an initializing constructor.
        Length2D GetLocalCenter() const noexcept { return localCenter; }
        
        /// Gets the alpha0 for this sweep.
        /// @return Value between 0 and less than 1.
        RealNum GetAlpha0() const noexcept { return alpha0; }
        
        /// Advances the sweep by a factor of the difference between the given time alpha and the sweep's alpha0.
        /// @details
        /// This advances position 0 (<code>pos0</code>) of the sweep towards position 1 (<code>pos1</code>)
        /// by a factor of the difference between the given alpha and the alpha0.
        /// @param alpha Valid new time factor in [0,1) to update the sweep to. Behavior is undefined if value is invalid.
        void Advance0(RealNum alpha) noexcept;
        
        void ResetAlpha0() noexcept;
        
        Position pos0; ///< Center world position and world angle at time "0". 12-bytes.
        Position pos1; ///< Center world position and world angle at time "1". 12-bytes.
        
    private:
        Length2D localCenter = Vec2_zero * Meter; ///< Local center of mass position. 8-bytes.
        
        /// Fraction of the current time step in the range [0,1]
        /// pos0.linear and pos0.angular are the positions at alpha0.
        /// @note 4-bytes.
        RealNum alpha0 = 0;
    };
    
    inline void Sweep::Advance0(const RealNum alpha) noexcept
    {
        assert(IsValid(alpha));
        assert(alpha >= 0);
        assert(alpha < 1);
        assert(alpha0 < 1);
        
        const auto beta = (alpha - alpha0) / (1 - alpha0);
        pos0 = GetPosition(pos0, pos1, beta);
        alpha0 = alpha;
    }
    
    inline void Sweep::ResetAlpha0() noexcept
    {
        alpha0 = 0;
    }
    
}

#endif /* Sweep_hpp */
