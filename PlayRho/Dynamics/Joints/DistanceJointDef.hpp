/*
 * Original work Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
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

#ifndef PLAYRHO_DYNAMICS_JOINTS_DISTANCEJOINTDEF_HPP
#define PLAYRHO_DYNAMICS_JOINTS_DISTANCEJOINTDEF_HPP

#include <PlayRho/Dynamics/Joints/JointDef.hpp>
#include <PlayRho/Common/BoundedValue.hpp>
#include <PlayRho/Common/Math.hpp>

namespace playrho {

class Body;
class DistanceJoint;

/// @brief Distance joint definition.
/// @details This requires defining an anchor point on both bodies and the non-zero
/// length of the distance joint. The definition uses local anchor points so that
/// the initial configuration can violate the constraint slightly. This helps when
//  saving and loading a game.
/// @warning Do not use a zero or short length.
struct DistanceJointDef : public JointBuilder<DistanceJointDef>
{
    
    /// @brief Super type.
    using super = JointBuilder<DistanceJointDef>;
    
    constexpr DistanceJointDef() noexcept: super{JointType::Distance} {}
    
    /// @brief Copy constructor.
    DistanceJointDef(const DistanceJointDef& copy) = default;
    
    /// @brief Initializing constructor.
    /// @details Initialize the bodies, anchors, and length using the world anchors.
    DistanceJointDef(NonNull<Body*> bodyA, NonNull<Body*> bodyB,
                     Length2D anchorA = Length2D{},
                     Length2D anchorB = Length2D{}) noexcept;
    
    /// @brief Uses the given length.
    constexpr DistanceJointDef& UseLength(Length v) noexcept;
    
    /// @brief Uses the given frequency.
    constexpr DistanceJointDef& UseFrequency(NonNegative<Frequency> v) noexcept;
    
    /// @brief Uses the given damping ratio.
    constexpr DistanceJointDef& UseDampingRatio(Real v) noexcept;
    
    /// @brief Local anchor point relative to bodyA's origin.
    Length2D localAnchorA = Length2D{};
    
    /// @brief Local anchor point relative to bodyB's origin.
    Length2D localAnchorB = Length2D{};
    
    /// @brief Natural length between the anchor points.
    Length length = Real{1} * Meter;
    
    /// @brief Mass-spring-damper frequency.
    /// @note 0 disables softness.
    NonNegative<Frequency> frequency = NonNegative<Frequency>{0};
    
    /// @brief Damping ratio.
    /// @note 0 = no damping, 1 = critical damping.
    Real dampingRatio = 0;
};

constexpr DistanceJointDef& DistanceJointDef::UseLength(Length v) noexcept
{
    length = v;
    return *this;
}

constexpr DistanceJointDef& DistanceJointDef::UseFrequency(NonNegative<Frequency> v) noexcept
{
    frequency = v;
    return *this;
}

constexpr DistanceJointDef& DistanceJointDef::UseDampingRatio(Real v) noexcept
{
    dampingRatio = v;
    return *this;
}

/// @brief Gets the definition data for the given joint.
DistanceJointDef GetDistanceJointDef(const DistanceJoint& joint) noexcept;

} // namespace playrho

#endif // PLAYRHO_DYNAMICS_JOINTS_DISTANCEJOINTDEF_HPP
