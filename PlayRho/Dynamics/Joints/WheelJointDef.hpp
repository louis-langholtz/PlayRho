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

#ifndef PLAYRHO_DYNAMICS_JOINTS_WHEELJOINTDEF_HPP
#define PLAYRHO_DYNAMICS_JOINTS_WHEELJOINTDEF_HPP

#include <PlayRho/Dynamics/Joints/JointDef.hpp>
#include <PlayRho/Common/BoundedValue.hpp>
#include <PlayRho/Common/Math.hpp>

namespace playrho {

class WheelJoint;

/// @brief Wheel joint definition.
/// @details This requires defining a line of
///   motion using an axis and an anchor point. The definition uses local
///   anchor points and a local axis so that the initial configuration
///   can violate the constraint slightly. The joint translation is zero
///   when the local anchor points coincide in world space. Using local
///   anchors and a local axis helps when saving and loading a game.
struct WheelJointDef : public JointBuilder<WheelJointDef>
{
    /// @brief Super type.
    using super = JointBuilder<WheelJointDef>;
    
    constexpr WheelJointDef() noexcept: super{JointType::Wheel} {}
    
    /// Initialize the bodies, anchors, axis, and reference angle using the world
    /// anchor and world axis.
    WheelJointDef(NonNull<Body*> bodyA, NonNull<Body*> bodyB, const Length2D anchor,
                  const UnitVec2 axis) noexcept;
    
    /// @brief Uses the given enable motor state value.
    constexpr WheelJointDef& UseEnableMotor(bool v) noexcept;
    
    /// @brief Uses the given max motor toque value.
    constexpr WheelJointDef& UseMaxMotorTorque(Torque v) noexcept;
    
    /// @brief Uses the given motor speed value.
    constexpr WheelJointDef& UseMotorSpeed(AngularVelocity v) noexcept;
    
    /// @brief Uses the given frequency value.
    constexpr WheelJointDef& UseFrequency(Frequency v) noexcept;
    
    /// @brief Uses the given damping ratio value.
    constexpr WheelJointDef& UseDampingRatio(Real v) noexcept;
    
    /// The local anchor point relative to bodyA's origin.
    Length2D localAnchorA = Length2D{};
    
    /// The local anchor point relative to bodyB's origin.
    Length2D localAnchorB = Length2D{};
    
    /// The local translation axis in bodyA.
    UnitVec2 localAxisA = UnitVec2::GetRight();
    
    /// Enable/disable the joint motor.
    bool enableMotor = false;
    
    /// The maximum motor torque.
    Torque maxMotorTorque = Torque{0};
    
    /// The desired angular motor speed.
    AngularVelocity motorSpeed = AngularVelocity{0};
    
    /// Suspension frequency, zero indicates no suspension
    Frequency frequency = Real{2} * Hertz;
    
    /// Suspension damping ratio, one indicates critical damping
    Real dampingRatio = 0.7f;
};

constexpr WheelJointDef& WheelJointDef::UseEnableMotor(bool v) noexcept
{
    enableMotor = v;
    return *this;
}

constexpr WheelJointDef& WheelJointDef::UseMaxMotorTorque(Torque v) noexcept
{
    maxMotorTorque = v;
    return *this;
}

constexpr WheelJointDef& WheelJointDef::UseMotorSpeed(AngularVelocity v) noexcept
{
    motorSpeed = v;
    return *this;
}

constexpr WheelJointDef& WheelJointDef::UseFrequency(Frequency v) noexcept
{
    frequency = v;
    return *this;
}

constexpr WheelJointDef& WheelJointDef::UseDampingRatio(Real v) noexcept
{
    dampingRatio = v;
    return *this;
}

/// @brief Gets the definition data for the given joint.
WheelJointDef GetWheelJointDef(const WheelJoint& joint) noexcept;

} // namespace playrho

#endif // PLAYRHO_DYNAMICS_JOINTS_WHEELJOINTDEF_HPP
