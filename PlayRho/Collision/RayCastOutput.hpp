/*
 * Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

#ifndef RayCastOutput_hpp
#define RayCastOutput_hpp

/// @file
/// Declaration of the RayCastOutput struct and related free functions.

#include <PlayRho/Common/Math.hpp>

namespace box2d
{
    struct RayCastInput;
    class AABB;
    class Shape;
    class DistanceProxy;

    /// @brief Ray-cast output data.
    /// @details The ray hits at p1 + fraction * (p2 - p1), where p1 and p2 come from RayCastInput.
    struct RayCastOutput
    {
        RayCastOutput() = default;
        
        /// @brief Initializing constructor.
        ///
        /// @note This intentionally default initializes the hit parameter as <code>true</code>.
        ///
        /// @param n Normal.
        /// @param f Fraction. A unit interval value or NaN (a value between 0 and 1 inclusive
        ///   or NaN).
        /// @param h Hit (or not).
        ///
        constexpr RayCastOutput(UnitVec2 n, Real f, bool h = true) noexcept:
            normal{n}, fraction{f}, hit{h}
        {
            // Check against out-of-range values of f while accepting NaN.
            assert(!(f < 0) && !(f > 1));
        }
        
        /// @brief Surface normal in world coordinates at the point of contact.
        /// @note This value is meaningless unless the ray hit.
        /// @sa hit.
        UnitVec2 normal = GetInvalid<decltype(normal)>();

        /// @brief Fraction.
        /// @note This is a unit interval value - a value between 0 and 1 - or it's invalid.
        /// @note This value is meaningless unless the ray hit.
        /// @sa hit.
        Real fraction = GetInvalid<decltype(fraction)>();
        
        /// @brief Hit flag.
        /// @note <code>true</code> if the ray hit and the normal and fraction values should be
        ///   valid, <code>false</code> otherwise.
        bool hit = false;
    };

    /// @brief Cast a ray against a circle of a given radius at the given location.
    /// @param radius Radius of the circle.
    /// @param location Location in world coordinates of the circle.
    /// @param input Ray-cast input parameters.
    RayCastOutput RayCast(const Length radius, const Length2D location,
                          const RayCastInput& input) noexcept;

    /// @brief Cast a ray against the given AABB.
    /// @param aabb Axis Aligned Bounding Box.
    /// @param input the ray-cast input parameters.
    RayCastOutput RayCast(const AABB& aabb, const RayCastInput& input) noexcept;
    
    /// @brief Cast a ray against the distance proxy.
    /// @param proxy Distance-proxy object (in local coordinates).
    /// @param input Ray-cast input parameters.
    /// @param transform Transform to be applied to the distance-proxy to get world coordinates.
    RayCastOutput RayCast(const DistanceProxy& proxy, const RayCastInput& input,
                          const Transformation& transform) noexcept;
    
    /// @brief Cast a ray against the child of the given shape.
    /// @note This is a convenience function for calling the raycast against a distance-proxy.
    /// @param shape Shape.
    /// @param childIndex Child index.
    /// @param input the ray-cast input parameters.
    /// @param transform Transform to be applied to the child of the shape.
    RayCastOutput RayCast(const Shape& shape, ChildCounter childIndex,
                          const RayCastInput& input, const Transformation& transform) noexcept;

} // namespace box2d

#endif /* RayCastOutput_hpp */
