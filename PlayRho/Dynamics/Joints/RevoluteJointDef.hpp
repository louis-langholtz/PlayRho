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

#ifndef RevoluteJointDef_hpp
#define RevoluteJointDef_hpp

#include <PlayRho/Dynamics/Joints/JointDef.hpp>
#include <PlayRho/Common/BoundedValue.hpp>
#include <PlayRho/Common/Math.hpp>

namespace playrho {

class RevoluteJoint;

/// @brief Revolute joint definition.
/// @details This requires defining an
/// anchor point where the bodies are joined. The definition
/// uses local anchor points so that the initial configuration
/// can violate the constraint slightly. You also need to
/// specify the initial relative angle for joint limits. This
/// helps when saving and loading a game.
/// @note The local anchor points are measured from the body's origin
///   rather than the center of mass because:
///    1. you might not know where the center of mass will be;
///    2. if you add/remove shapes from a body and recompute the mass,
///       the joints will be broken.
struct RevoluteJointDef : public JointBuilder<RevoluteJointDef>
{
    using super = JointBuilder<RevoluteJointDef>;
    
    constexpr RevoluteJointDef() noexcept: super{JointType::Revolute} {}
    
    /// @brief Initialize the bodies, anchors, and reference angle using a world anchor point.
    RevoluteJointDef(NonNull<Body*> bodyA, NonNull<Body*> bodyB, const Length2D anchor) noexcept;
    
    constexpr RevoluteJointDef& UseEnableLimit(bool v) noexcept;
    
    constexpr RevoluteJointDef& UseLowerAngle(Angle v) noexcept;
    
    constexpr RevoluteJointDef& UseUpperAngle(Angle v) noexcept;
    
    constexpr RevoluteJointDef& UseEnableMotor(bool v) noexcept;

    /// @brief Local anchor point relative to bodyA's origin.
    Length2D localAnchorA = Length2D{};
    
    /// @brief Local anchor point relative to bodyB's origin.
    Length2D localAnchorB = Length2D{};
    
    /// @brief Reference angle.
    /// @details This is the bodyB angle minus bodyA angle in the reference state (radians).
    Angle referenceAngle = Angle{0};
    
    /// @brief Flag to enable joint limits.
    bool enableLimit = false;
    
    /// @brief Lower angle for the joint limit.
    Angle lowerAngle = Angle{0};
    
    /// @brief Upper angle for the joint limit.
    Angle upperAngle = Angle{0};
    
    /// @brief Flag to enable the joint motor.
    bool enableMotor = false;
    
    /// @brief Desired motor speed.
    AngularVelocity motorSpeed = AngularVelocity{0};
    
    /// @brief Maximum motor torque used to achieve the desired motor speed.
    Torque maxMotorTorque = 0;
};

constexpr RevoluteJointDef& RevoluteJointDef::UseEnableLimit(bool v) noexcept
{
    enableLimit = v;
    return *this;
}

constexpr RevoluteJointDef& RevoluteJointDef::UseLowerAngle(Angle v) noexcept
{
    lowerAngle = v;
    return *this;
}

constexpr RevoluteJointDef& RevoluteJointDef::UseUpperAngle(Angle v) noexcept
{
    upperAngle = v;
    return *this;
}

constexpr RevoluteJointDef& RevoluteJointDef::UseEnableMotor(bool v) noexcept
{
    enableMotor = v;
    return *this;
}

RevoluteJointDef GetRevoluteJointDef(const RevoluteJoint& joint) noexcept;

} // namespace playrho

#endif /* RevoluteJointDef_hpp */
