/*
* Original work Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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

#ifndef B2_WHEEL_JOINT_H
#define B2_WHEEL_JOINT_H

#include <PlayRho/Dynamics/Joints/Joint.hpp>
#include <PlayRho/Common/BoundedValue.hpp>

namespace playrho {

/// @brief Wheel joint definition.
/// @details This requires defining a line of
///   motion using an axis and an anchor point. The definition uses local
///   anchor points and a local axis so that the initial configuration
///   can violate the constraint slightly. The joint translation is zero
///   when the local anchor points coincide in world space. Using local
///   anchors and a local axis helps when saving and loading a game.
struct WheelJointDef : public JointBuilder<WheelJointDef>
{
    using super = JointBuilder<WheelJointDef>;
    
    constexpr WheelJointDef() noexcept: super{JointType::Wheel} {}

    /// Initialize the bodies, anchors, axis, and reference angle using the world
    /// anchor and world axis.
    WheelJointDef(NonNull<Body*> bodyA, NonNull<Body*> bodyB, const Length2D anchor,
                  const UnitVec2 axis) noexcept;

    /// The local anchor point relative to bodyA's origin.
    Length2D localAnchorA = Length2D(0, 0);

    /// The local anchor point relative to bodyB's origin.
    Length2D localAnchorB = Length2D(0, 0);

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

/// @brief Wheel joint.
/// @details This joint provides two degrees of freedom: translation
///   along an axis fixed in bodyA and rotation in the plane. In other words, it is a point to
///   line constraint with a rotational motor and a linear spring/damper.
///   This joint is designed for vehicle suspensions.
class WheelJoint : public Joint
{
public:
    WheelJoint(const WheelJointDef& def);
    
    Length2D GetAnchorA() const override;
    Length2D GetAnchorB() const override;

    Force2D GetReactionForce(Frequency inv_dt) const override;
    Torque GetReactionTorque(Frequency inv_dt) const override;

    /// The local anchor point relative to bodyA's origin.
    Length2D GetLocalAnchorA() const { return m_localAnchorA; }

    /// The local anchor point relative to bodyB's origin.
    Length2D GetLocalAnchorB() const  { return m_localAnchorB; }

    /// The local joint axis relative to bodyA.
    UnitVec2 GetLocalAxisA() const { return m_localXAxisA; }

    /// Is the joint motor enabled?
    bool IsMotorEnabled() const noexcept { return m_enableMotor; }

    /// Enable/disable the joint motor.
    void EnableMotor(bool flag);

    /// Set the angular motor speed.
    void SetMotorSpeed(AngularVelocity speed);

    /// Get the angular motor speed.
    AngularVelocity GetMotorSpeed() const;

    /// Set/Get the maximum motor force.
    void SetMaxMotorTorque(Torque torque);
    Torque GetMaxMotorTorque() const;

    /// Get the current motor torque given the inverse time step.
    Torque GetMotorTorque(Frequency inv_dt) const;

    /// @brief Sets the spring frequency.
    /// @note Setting the frequency to zero disables the spring.
    void SetSpringFrequency(Frequency frequency);

    /// @brief Gets the spring frequency.
    Frequency GetSpringFrequency() const;

    /// Set/Get the spring damping ratio
    void SetSpringDampingRatio(Real ratio);
    Real GetSpringDampingRatio() const;

private:

    void InitVelocityConstraints(BodyConstraintsMap& bodies, const StepConf& step,
                                 const ConstraintSolverConf& conf) override;
    bool SolveVelocityConstraints(BodyConstraintsMap& bodies, const StepConf& step) override;
    bool SolvePositionConstraints(BodyConstraintsMap& bodies,
                                  const ConstraintSolverConf& conf) const override;

    Frequency m_frequency;
    Real m_dampingRatio;

    // Solver shared
    Length2D m_localAnchorA;
    Length2D m_localAnchorB;
    UnitVec2 m_localXAxisA;
    UnitVec2 m_localYAxisA;

    Momentum m_impulse = 0;
    AngularMomentum m_motorImpulse = 0;
    Momentum m_springImpulse = 0;

    Torque m_maxMotorTorque;
    AngularVelocity m_motorSpeed;
    bool m_enableMotor;

    // Solver temp    
    UnitVec2 m_ax;
    UnitVec2 m_ay;

    Length m_sAx;
    Length m_sBx;
    Length m_sAy;
    Length m_sBy;

    Mass m_mass = Mass{0};
    RotInertia m_motorMass = RotInertia{0};
    Mass m_springMass = Mass{0};

    LinearVelocity m_bias = LinearVelocity{0};
    InvMass m_gamma = InvMass{0};
};

inline AngularVelocity WheelJoint::GetMotorSpeed() const
{
    return m_motorSpeed;
}

inline Torque WheelJoint::GetMaxMotorTorque() const
{
    return m_maxMotorTorque;
}

inline void WheelJoint::SetSpringFrequency(Frequency hz)
{
    m_frequency = hz;
}

inline Frequency WheelJoint::GetSpringFrequency() const
{
    return m_frequency;
}

inline void WheelJoint::SetSpringDampingRatio(Real ratio)
{
    m_dampingRatio = ratio;
}

inline Real WheelJoint::GetSpringDampingRatio() const
{
    return m_dampingRatio;
}

// Free functions on WheelJoint instances.

/// Get the current joint translation.
Length GetJointTranslation(const WheelJoint& joint) noexcept;

/// Get the current joint translation speed.
AngularVelocity GetAngularVelocity(const WheelJoint& joint) noexcept;

WheelJointDef GetWheelJointDef(const WheelJoint& joint) noexcept;

} // namespace playrho

#endif
