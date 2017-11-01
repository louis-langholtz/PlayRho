/*
 * Original work Copyright (c) 2006-2012 Erin Catto http://www.box2d.org
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

#ifndef PLAYRHO_DYNAMICS_JOINTS_MOTORJOINTDEF_HPP
#define PLAYRHO_DYNAMICS_JOINTS_MOTORJOINTDEF_HPP

#include <PlayRho/Dynamics/Joints/JointDef.hpp>
#include <PlayRho/Common/BoundedValue.hpp>
#include <PlayRho/Common/Math.hpp>

namespace playrho {

class Body;
class MotorJoint;
    
/// @brief Motor joint definition.
struct MotorJointDef : public JointBuilder<MotorJointDef>
{
    /// @brief Super type.
    using super = JointBuilder<MotorJointDef>;
    
    constexpr MotorJointDef() noexcept: super{JointType::Motor} {}
    
    /// @brief Initialize the bodies and offsets using the current transforms.
    MotorJointDef(NonNull<Body*> bodyA, NonNull<Body*> bodyB) noexcept;
    
    /// @brief Uses the given linear offset value.
    MotorJointDef& UseLinearOffset(Length2 v) noexcept;

    /// @brief Uses the given angular offset value.
    MotorJointDef& UseAngularOffset(Angle v) noexcept;

    /// @brief Uses the given maximum force value.
    MotorJointDef& UseMaxForce(NonNegative<Force> v) noexcept;
    
    /// @brief Uses the given max torque value.
    MotorJointDef& UseMaxTorque(NonNegative<Torque> v) noexcept;
    
    /// @brief Uses the given correction factor.
    MotorJointDef& UseCorrectionFactor(Real v) noexcept;
    
    /// @brief Position of bodyB minus the position of bodyA, in bodyA's frame.
    Length2 linearOffset = Length2{};
    
    /// @brief Angle of bodyB minus angle of bodyA.
    Angle angularOffset = Angle{0};
    
    /// @brief Maximum motor force.
    NonNegative<Force> maxForce = NonNegative<Force>(1_N);
    
    /// @brief Maximum motor torque.
    NonNegative<Torque> maxTorque = NonNegative<Torque>(1_Nm);
    
    /// @brief Position correction factor in the range [0,1].
    Real correctionFactor = Real(0.3);
};

inline MotorJointDef& MotorJointDef::UseLinearOffset(Length2 v) noexcept
{
    linearOffset = v;
    return *this;
}

inline MotorJointDef& MotorJointDef::UseAngularOffset(Angle v) noexcept
{
    angularOffset = v;
    return *this;
}

inline MotorJointDef& MotorJointDef::UseMaxForce(NonNegative<Force> v) noexcept
{
    maxForce = v;
    return *this;
}

inline MotorJointDef& MotorJointDef::UseMaxTorque(NonNegative<Torque> v) noexcept
{
    maxTorque = v;
    return *this;
}

inline MotorJointDef& MotorJointDef::UseCorrectionFactor(Real v) noexcept
{
    correctionFactor = v;
    return *this;
}

/// @brief Gets the definition data for the given joint.
/// @relatedalso MotorJoint
MotorJointDef GetMotorJointDef(const MotorJoint& joint) noexcept;

} // namespace playrho

#endif // PLAYRHO_DYNAMICS_JOINTS_MOTORJOINTDEF_HPP
