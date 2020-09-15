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

#ifndef PLAYRHO_DYNAMICS_JOINTS_WHEELJOINTCONF_HPP
#define PLAYRHO_DYNAMICS_JOINTS_WHEELJOINTCONF_HPP

#include <PlayRho/Dynamics/Joints/JointConf.hpp>
#include <PlayRho/Common/Math.hpp>

namespace playrho {
namespace d2 {

class WheelJoint;
class World;

/// @brief Wheel joint definition.
/// @details This requires defining a line of
///   motion using an axis and an anchor point. The definition uses local
///   anchor points and a local axis so that the initial configuration
///   can violate the constraint slightly. The joint translation is zero
///   when the local anchor points coincide in world space. Using local
///   anchors and a local axis helps when saving and loading a game.
struct WheelJointConf : public JointBuilder<WheelJointConf>
{
    /// @brief Super type.
    using super = JointBuilder<WheelJointConf>;
    
    constexpr WheelJointConf() noexcept: super{JointType::Wheel} {}
    
    /// Initialize the bodies, anchors, axis, and reference angle using the world
    /// anchor and world axis.
    WheelJointConf(BodyID bA, BodyID bB,
                   Length2 laA = Length2{}, Length2 laB = Length2{},
                   UnitVec axis = UnitVec::GetRight()) noexcept;
    
    /// @brief Uses the given enable motor state value.
    constexpr WheelJointConf& UseEnableMotor(bool v) noexcept;
    
    /// @brief Uses the given max motor toque value.
    constexpr WheelJointConf& UseMaxMotorTorque(Torque v) noexcept;
    
    /// @brief Uses the given motor speed value.
    constexpr WheelJointConf& UseMotorSpeed(AngularVelocity v) noexcept;
    
    /// @brief Uses the given frequency value.
    constexpr WheelJointConf& UseFrequency(Frequency v) noexcept;
    
    /// @brief Uses the given damping ratio value.
    constexpr WheelJointConf& UseDampingRatio(Real v) noexcept;
    
    /// The local anchor point relative to body A's origin.
    Length2 localAnchorA = Length2{};
    
    /// The local anchor point relative to body B's origin.
    Length2 localAnchorB = Length2{};
    
    /// The local translation axis in body-A.
    UnitVec localAxisA = UnitVec::GetRight();
    
    /// Enable/disable the joint motor.
    bool enableMotor = false;
    
    /// The maximum motor torque.
    Torque maxMotorTorque = Torque{0};
    
    /// The desired angular motor speed.
    AngularVelocity motorSpeed = 0_rpm;
    
    /// Suspension frequency, zero indicates no suspension
    Frequency frequency = 2_Hz;
    
    /// Suspension damping ratio, one indicates critical damping
    Real dampingRatio = 0.7f;
};

constexpr WheelJointConf& WheelJointConf::UseEnableMotor(bool v) noexcept
{
    enableMotor = v;
    return *this;
}

constexpr WheelJointConf& WheelJointConf::UseMaxMotorTorque(Torque v) noexcept
{
    maxMotorTorque = v;
    return *this;
}

constexpr WheelJointConf& WheelJointConf::UseMotorSpeed(AngularVelocity v) noexcept
{
    motorSpeed = v;
    return *this;
}

constexpr WheelJointConf& WheelJointConf::UseFrequency(Frequency v) noexcept
{
    frequency = v;
    return *this;
}

constexpr WheelJointConf& WheelJointConf::UseDampingRatio(Real v) noexcept
{
    dampingRatio = v;
    return *this;
}

/// @brief Gets the definition data for the given joint.
/// @relatedalso WheelJoint
WheelJointConf GetWheelJointConf(const WheelJoint& joint) noexcept;

WheelJointConf GetWheelJointConf(const World& world, BodyID bodyA, BodyID bodyB,
                                 Length2 anchor, UnitVec axis = UnitVec::GetRight());

} // namespace d2
} // namespace playrho

#endif // PLAYRHO_DYNAMICS_JOINTS_WHEELJOINTCONF_HPP
