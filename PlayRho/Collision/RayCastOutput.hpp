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

#ifndef PLAYRHO_COLLISION_RAYCASTOUTPUT_HPP
#define PLAYRHO_COLLISION_RAYCASTOUTPUT_HPP

/// @file
/// Declaration of the RayCastOutput struct and related free functions.

#include <PlayRho/Common/Math.hpp>
#include <PlayRho/Common/BoundedValue.hpp>
#include <PlayRho/Common/OptionalValue.hpp>

namespace playrho
{
    struct RayCastInput;
    class AABB;
    class Shape;
    class DistanceProxy;

    /// @brief Ray-cast hit data.
    /// @details The ray hits at p1 + fraction * (p2 - p1), where p1 and p2 come from RayCastInput.
    struct RayCastHit
    {        
        /// @brief Surface normal in world coordinates at the point of contact.
        UnitVec2 normal;

        /// @brief Fraction.
        /// @note This is a unit interval value - a value between 0 and 1 - or it's invalid.
        UnitInterval<Real> fraction = UnitInterval<Real>{0};
    };

    /// @brief Ray cast output.
    /// @details This is a type alias for an optional RayCastHit instance.
    /// @sa RayCast
    /// @sa Optional
    /// @sa RayCastHit
    using RayCastOutput = Optional<RayCastHit>;
    
    /// @defgroup RayCastGroup Ray Casting Functions
    /// @details Collection of functions that do ray casting.
    /// @image html raycast.png
    /// @{

    /// @brief Cast a ray against a circle of a given radius at the given location.
    /// @param radius Radius of the circle.
    /// @param location Location in world coordinates of the circle.
    /// @param input Ray-cast input parameters.
    RayCastOutput RayCast(Length radius, Length2D location, const RayCastInput& input) noexcept;

    /// @brief Cast a ray against the given AABB.
    /// @param aabb Axis Aligned Bounding Box.
    /// @param input the ray-cast input parameters.
    /// @relatedalso AABB
    RayCastOutput RayCast(const AABB& aabb, const RayCastInput& input) noexcept;
    
    /// @brief Cast a ray against the distance proxy.
    /// @param proxy Distance-proxy object (in local coordinates).
    /// @param input Ray-cast input parameters.
    /// @param transform Transform to be applied to the distance-proxy to get world coordinates.
    /// @relatedalso DistanceProxy
    RayCastOutput RayCast(const DistanceProxy& proxy, const RayCastInput& input,
                          const Transformation& transform) noexcept;
    
    /// @brief Cast a ray against the child of the given shape.
    /// @note This is a convenience function for calling the raycast against a distance-proxy.
    /// @param shape Shape.
    /// @param childIndex Child index.
    /// @param input the ray-cast input parameters.
    /// @param transform Transform to be applied to the child of the shape.
    /// @relatedalso Shape
    RayCastOutput RayCast(const Shape& shape, ChildCounter childIndex,
                          const RayCastInput& input, const Transformation& transform) noexcept;

    /// @}

} // namespace playrho

#endif // PLAYRHO_COLLISION_RAYCASTOUTPUT_HPP
