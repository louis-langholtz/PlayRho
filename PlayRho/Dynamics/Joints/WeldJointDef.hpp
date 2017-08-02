/*
 * Original work Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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

#ifndef WeldJointDef_hpp
#define WeldJointDef_hpp

#include <PlayRho/Dynamics/Joints/JointDef.hpp>
#include <PlayRho/Common/BoundedValue.hpp>
#include <PlayRho/Common/Math.hpp>

namespace playrho {

class WeldJoint;

/// @brief Weld joint definition.
/// @details You need to specify local anchor points
/// where they are attached and the relative body angle. The position
/// of the anchor points is important for computing the reaction torque.
struct WeldJointDef : public JointBuilder<WeldJointDef>
{
    using super = JointBuilder<WeldJointDef>;
    
    constexpr WeldJointDef() noexcept: super{JointType::Weld} {}
    
    /// Initialize the bodies, anchors, and reference angle using a world
    /// anchor point.
    WeldJointDef(NonNull<Body*> bodyA, NonNull<Body*> bodyB, const Length2D anchor) noexcept;
    
    constexpr WeldJointDef& UseFrequency(Frequency v) noexcept;
    
    constexpr WeldJointDef& UseDampingRatio(Real v) noexcept;
    
    /// The local anchor point relative to bodyA's origin.
    Length2D localAnchorA = Length2D{};
    
    /// The local anchor point relative to bodyB's origin.
    Length2D localAnchorB = Length2D{};
    
    /// The bodyB angle minus bodyA angle in the reference state (radians).
    Angle referenceAngle = Angle{0};
    
    /// @brief Mass-spring-damper frequency.
    /// @note Rotation only.
    /// @note Disable softness with a value of 0.
    Frequency frequency = Frequency{0};
    
    /// @brief Damping ratio.
    /// @note 0 = no damping, 1 = critical damping.
    Real dampingRatio = 0;
};

constexpr WeldJointDef& WeldJointDef::UseFrequency(Frequency v) noexcept
{
    frequency = v;
    return *this;
}

constexpr WeldJointDef& WeldJointDef::UseDampingRatio(Real v) noexcept
{
    dampingRatio = v;
    return *this;
}

WeldJointDef GetWeldJointDef(const WeldJoint& joint) noexcept;

} // namespace playrho

#endif /* WeldJointDef_hpp */
