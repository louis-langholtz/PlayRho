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

#ifndef PLAYRHO_DYNAMICS_JOINTS_PRISMATICJOINTDEF_HPP
#define PLAYRHO_DYNAMICS_JOINTS_PRISMATICJOINTDEF_HPP

#include <PlayRho/Dynamics/Joints/JointDef.hpp>
#include <PlayRho/Common/BoundedValue.hpp>
#include <PlayRho/Common/Math.hpp>

namespace playrho {

class PrismaticJoint;

/// @brief Prismatic joint definition.
/// @details This requires defining a line of
/// motion using an axis and an anchor point. The definition uses local
/// anchor points and a local axis so that the initial configuration
/// can violate the constraint slightly. The joint translation is zero
/// when the local anchor points coincide in world space. Using local
/// anchors and a local axis helps when saving and loading a game.
struct PrismaticJointDef : public JointBuilder<PrismaticJointDef>
{
    /// @brief Super type.
    using super = JointBuilder<PrismaticJointDef>;
    
    constexpr PrismaticJointDef() noexcept: super{JointType::Prismatic} {}
    
    /// @brief Copy constructor.
    PrismaticJointDef(const PrismaticJointDef& copy) = default;
    
    /// @brief Initializing constructor.
    /// @details Initializes the bodies, anchors, axis, and reference angle using the world
    ///   anchor and unit world axis.
    PrismaticJointDef(NonNull<Body*> bodyA, NonNull<Body*> bodyB, const Length2D anchor,
                      const UnitVec2 axis) noexcept;
    
    /// @brief Uses the given enable limit state value.
    PrismaticJointDef& UseEnableLimit(bool v) noexcept;
    
    /// @brief Uses the given lower translation value.
    PrismaticJointDef& UseLowerTranslation(Length v) noexcept;
    
    /// @brief Uses the given upper translation value.
    PrismaticJointDef& UseUpperTranslation(Length v) noexcept;
    
    /// @brief Uses the given enable motor state value.
    PrismaticJointDef& UseEnableMotor(bool v) noexcept;
    
    /// The local anchor point relative to bodyA's origin.
    Length2D localAnchorA = Length2D{};
    
    /// The local anchor point relative to bodyB's origin.
    Length2D localAnchorB = Length2D{};
    
    /// The local translation unit axis in bodyA.
    UnitVec2 localAxisA = UnitVec2::GetRight();
    
    /// The constrained angle between the bodies: bodyB_angle - bodyA_angle.
    Angle referenceAngle = Angle{0};
    
    /// Enable/disable the joint limit.
    bool enableLimit = false;
    
    /// The lower translation limit.
    Length lowerTranslation = Length{0};
    
    /// The upper translation limit.
    Length upperTranslation = Length{0};
    
    /// Enable/disable the joint motor.
    bool enableMotor = false;
    
    /// The maximum motor force.
    Force maxMotorForce = Force{0};
    
    /// The desired angular motor speed.
    AngularVelocity motorSpeed = AngularVelocity{0};
};

inline PrismaticJointDef& PrismaticJointDef::UseEnableLimit(bool v) noexcept
{
    enableLimit = v;
    return *this;
}

inline PrismaticJointDef& PrismaticJointDef::UseLowerTranslation(Length v) noexcept
{
    lowerTranslation = v;
    return *this;
}

inline PrismaticJointDef& PrismaticJointDef::UseUpperTranslation(Length v) noexcept
{
    upperTranslation = v;
    return *this;
}

inline PrismaticJointDef& PrismaticJointDef::UseEnableMotor(bool v) noexcept
{
    enableMotor = v;
    return *this;
}

/// @brief Gets the definition data for the given joint.
PrismaticJointDef GetPrismaticJointDef(const PrismaticJoint& joint) noexcept;

} // namespace playrho

#endif // PLAYRHO_DYNAMICS_JOINTS_PRISMATICJOINTDEF_HPP
