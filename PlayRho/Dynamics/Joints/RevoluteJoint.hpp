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

#ifndef B2_REVOLUTE_JOINT_H
#define B2_REVOLUTE_JOINT_H

#include <PlayRho/Dynamics/Joints/Joint.hpp>

namespace box2d {

/// Revolute joint definition. This requires defining an
/// anchor point where the bodies are joined. The definition
/// uses local anchor points so that the initial configuration
/// can violate the constraint slightly. You also need to
/// specify the initial relative angle for joint limits. This
/// helps when saving and loading a game.
/// The local anchor points are measured from the body's origin
/// rather than the center of mass because:
/// 1. you might not know where the center of mass will be.
/// 2. if you add/remove shapes from a body and recompute the mass,
///    the joints will be broken.
struct RevoluteJointDef : public JointDef
{
    constexpr RevoluteJointDef() noexcept: JointDef{JointType::Revolute} {}

    /// Initialize the bodies, anchors, and reference angle using a world
    /// anchor point.
    RevoluteJointDef(Body* bodyA, Body* bodyB, const Length2D anchor, bool cc = false);

    /// The local anchor point relative to bodyA's origin.
    Length2D localAnchorA = Length2D(0, 0);

    /// The local anchor point relative to bodyB's origin.
    Length2D localAnchorB = Length2D(0, 0);

    /// The bodyB angle minus bodyA angle in the reference state (radians).
    Angle referenceAngle = Angle{0};

    /// A flag to enable joint limits.
    bool enableLimit = false;

    /// The lower angle for the joint limit (radians).
    Angle lowerAngle = Angle{0};

    /// The upper angle for the joint limit (radians).
    Angle upperAngle = Angle{0};

    /// A flag to enable the joint motor.
    bool enableMotor = false;

    /// The desired motor speed.
    AngularVelocity motorSpeed = AngularVelocity{0};

    /// The maximum motor torque used to achieve the desired motor speed.
    Torque maxMotorTorque = 0;
};

/// Revolute Joint.
///
/// @details A revolute joint constrains two bodies to share a common point while they
/// are free to rotate about the point. The relative rotation about the shared
/// point is the joint angle.
///
/// @note You can limit the relative rotation with a joint limit that specifies a
/// lower and upper angle. You can use a motor
/// to drive the relative rotation about the shared point. A maximum motor torque
/// is provided so that infinite forces are not generated.
///
class RevoluteJoint : public Joint
{
public:
    RevoluteJoint(const RevoluteJointDef& def);

    Length2D GetAnchorA() const override;
    Length2D GetAnchorB() const override;

    /// The local anchor point relative to bodyA's origin.
    Length2D GetLocalAnchorA() const noexcept { return m_localAnchorA; }

    /// The local anchor point relative to bodyB's origin.
    Length2D GetLocalAnchorB() const noexcept { return m_localAnchorB; }

    /// Get the reference angle.
    Angle GetReferenceAngle() const noexcept { return m_referenceAngle; }

    /// Is the joint limit enabled?
    bool IsLimitEnabled() const noexcept;

    /// Enable/disable the joint limit.
    void EnableLimit(bool flag);

    /// Get the lower joint limit.
    Angle GetLowerLimit() const noexcept;

    /// Get the upper joint limit.
    Angle GetUpperLimit() const noexcept;

    /// Set the joint limits.
    void SetLimits(Angle lower, Angle upper);

    /// Is the joint motor enabled?
    bool IsMotorEnabled() const noexcept;

    /// Enable/disable the joint motor.
    void EnableMotor(bool flag);

    /// Set the angular motor speed.
    void SetMotorSpeed(AngularVelocity speed);

    /// Gets the angular motor speed.
    AngularVelocity GetMotorSpeed() const noexcept;

    /// Set the maximum motor torque.
    void SetMaxMotorTorque(Torque torque);

    Torque GetMaxMotorTorque() const noexcept { return m_maxMotorTorque; }

    /// Get the reaction force given the inverse time step.
    /// Unit is N.
    Force2D GetReactionForce(Frequency inv_dt) const override;

    /// Get the reaction torque due to the joint limit given the inverse time step.
    /// Unit is N*m.
    Torque GetReactionTorque(Frequency inv_dt) const override;

    /// Get the current motor torque given the inverse time step.
    /// Unit is N*m.
    Torque GetMotorTorque(Frequency inv_dt) const;

private:
    
    void InitVelocityConstraints(BodyConstraintsMap& bodies,
                                 const StepConf& step, const ConstraintSolverConf& conf) override;

    bool SolveVelocityConstraints(BodyConstraintsMap& bodies, const StepConf& step) override;
    
    bool SolvePositionConstraints(BodyConstraintsMap& bodies, const ConstraintSolverConf& conf) const override;

    // Solver shared
    Length2D m_localAnchorA;
    Length2D m_localAnchorB;
    Vec3 m_impulse = Vec3_zero; ///< Impulse. Mofified by: InitVelocityConstraints, SolveVelocityConstraints.
    AngularMomentum m_motorImpulse = 0; ///< Motor impulse. Modified by: InitVelocityConstraints, SolveVelocityConstraints.

    bool m_enableMotor;
    Torque m_maxMotorTorque;
    AngularVelocity m_motorSpeed;

    bool m_enableLimit;
    Angle m_referenceAngle;
    Angle m_lowerAngle;
    Angle m_upperAngle;

    // Solver cached temporary data. Values set by by InitVelocityConstraints.

    Length2D m_rA; ///< Rotated delta of body A's local center from local anchor A.
    Length2D m_rB; ///< Rotated delta of body B's local center from local anchor B.
    Mat33 m_mass; ///< Effective mass for point-to-point constraint.
    RotInertia m_motorMass; ///< Effective mass for motor/limit angular constraint.
    LimitState m_limitState = e_inactiveLimit; ///< Limit state.
};

inline bool RevoluteJoint::IsLimitEnabled() const noexcept
{
    return m_enableLimit;
}

inline Angle RevoluteJoint::GetLowerLimit() const noexcept
{
    return m_lowerAngle;
}

inline Angle RevoluteJoint::GetUpperLimit() const noexcept
{
    return m_upperAngle;
}

inline bool RevoluteJoint::IsMotorEnabled() const noexcept
{
    return m_enableMotor;
}

inline AngularVelocity RevoluteJoint::GetMotorSpeed() const noexcept
{
    return m_motorSpeed;
}

// Free functions...

/// Gets the current joint angle.
Angle GetJointAngle(const RevoluteJoint& joint);
    
/// Gets the current joint angle speed.
AngularVelocity GetJointSpeed(const RevoluteJoint& joint);

RevoluteJointDef GetRevoluteJointDef(const RevoluteJoint& joint) noexcept;

} // namespace box2d

#endif
