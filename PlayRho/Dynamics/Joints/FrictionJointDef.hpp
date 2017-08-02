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

#ifndef FrictionJointDef_hpp
#define FrictionJointDef_hpp

#include <PlayRho/Dynamics/Joints/JointDef.hpp>
#include <PlayRho/Common/BoundedValue.hpp>
#include <PlayRho/Common/Math.hpp>

namespace playrho {

class Body;
class FrictionJoint;

/// @brief Friction joint definition.
struct FrictionJointDef : public JointBuilder<FrictionJointDef>
{
    using super = JointBuilder<FrictionJointDef>;
    
    constexpr FrictionJointDef() noexcept: super{JointType::Friction} {}
    
    /// @brief Initializing constructor.
    /// @details Initialize the bodies, anchors, axis, and reference angle using the world
    ///   anchor and world axis.
    FrictionJointDef(Body* bodyA, Body* bodyB, const Length2D anchor) noexcept;
    
    constexpr FrictionJointDef& UseMaxForce(NonNegative<Force> v) noexcept;
    
    constexpr FrictionJointDef& UseMaxTorque(NonNegative<Torque> v) noexcept;
    
    /// @brief Local anchor point relative to bodyA's origin.
    Length2D localAnchorA = Length2D{};
    
    /// @brief Local anchor point relative to bodyB's origin.
    Length2D localAnchorB = Length2D{};
    
    /// @brief Maximum friction force.
    NonNegative<Force> maxForce = NonNegative<Force>{0};
    
    /// @brief Maximum friction torque.
    NonNegative<Torque> maxTorque = NonNegative<Torque>{0};
};

constexpr FrictionJointDef& FrictionJointDef::UseMaxForce(NonNegative<Force> v) noexcept
{
    maxForce = v;
    return *this;
}

constexpr FrictionJointDef& FrictionJointDef::UseMaxTorque(NonNegative<Torque> v) noexcept
{
    maxTorque = v;
    return *this;
}

FrictionJointDef GetFrictionJointDef(const FrictionJoint& joint) noexcept;

} // namespace playrho

#endif /* FrictionJointDef_hpp */
