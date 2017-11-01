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

#ifndef PLAYRHO_DYNAMICS_JOINTS_REVOLUTEJOINT_HPP
#define PLAYRHO_DYNAMICS_JOINTS_REVOLUTEJOINT_HPP

#include <PlayRho/Dynamics/Joints/Joint.hpp>
#include <PlayRho/Dynamics/Joints/RevoluteJointDef.hpp>

namespace playrho {

/// @brief Revolute Joint.
///
/// @details A revolute joint constrains two bodies to share a common point while they
/// are free to rotate about the point. The relative rotation about the shared
/// point is the joint angle.
///
/// @note You can limit the relative rotation with a joint limit that specifies a
///   lower and upper angle. You can use a motor to drive the relative rotation about
///   the shared point. A maximum motor torque is provided so that infinite forces are
///   not generated.
///
/// @ingroup JointsGroup
///
/// @image html revoluteJoint.gif
///
class RevoluteJoint : public Joint
{
public:
    
    /// @brief Initializing constructor.
    RevoluteJoint(const RevoluteJointDef& def);
    
    void Accept(JointVisitor& visitor) const override;

    Length2 GetAnchorA() const override;
    Length2 GetAnchorB() const override;

    /// The local anchor point relative to bodyA's origin.
    Length2 GetLocalAnchorA() const noexcept { return m_localAnchorA; }

    /// The local anchor point relative to bodyB's origin.
    Length2 GetLocalAnchorB() const noexcept { return m_localAnchorB; }

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

    /// @brief Gets the max motor torque.
    Torque GetMaxMotorTorque() const noexcept { return m_maxMotorTorque; }

    /// Get the linear reaction.
    Momentum2 GetLinearReaction() const override;

    /// Get the angular reaction due to the joint limit.
    AngularMomentum GetAngularReaction() const override;

    /// Get the current motor torque given the inverse time step.
    Torque GetMotorTorque(Frequency inv_dt) const;
    
    /// @brief Gets the current limit state.
    /// @note This will be <code>e_inactiveLimit</code> unless the joint limit has been
    ///   enabled.
    LimitState GetLimitState() const noexcept;

private:
    
    void InitVelocityConstraints(BodyConstraintsMap& bodies,
                                 const StepConf& step, const ConstraintSolverConf& conf) override;

    bool SolveVelocityConstraints(BodyConstraintsMap& bodies, const StepConf& step) override;
    
    bool SolvePositionConstraints(BodyConstraintsMap& bodies,
                                  const ConstraintSolverConf& conf) const override;

    // Solver shared
    Length2 m_localAnchorA; ///< Local anchor A.
    Length2 m_localAnchorB; ///< Local anchor B.
    Vec3 m_impulse = Vec3_zero; ///< Impulse. Mofified by: InitVelocityConstraints, SolveVelocityConstraints.
    AngularMomentum m_motorImpulse = 0; ///< Motor impulse. Modified by: InitVelocityConstraints, SolveVelocityConstraints.

    bool m_enableMotor; ///< Enable motor. <code>true</code> if motor is enabled.
    Torque m_maxMotorTorque; ///< Max motor torque.
    AngularVelocity m_motorSpeed; ///< Motor speed.

    bool m_enableLimit; ///< Enable limit. <code>true</code> if limit is enabled.
    Angle m_referenceAngle; ///< Reference angle.
    Angle m_lowerAngle; ///< Lower angle.
    Angle m_upperAngle; ///< Upper angle.

    // Solver cached temporary data. Values set by by InitVelocityConstraints.

    Length2 m_rA; ///< Rotated delta of body A's local center from local anchor A.
    Length2 m_rB; ///< Rotated delta of body B's local center from local anchor B.
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

inline Joint::LimitState RevoluteJoint::GetLimitState() const noexcept
{
    return m_limitState;
}

// Free functions...

/// @brief Gets the current joint angle.
/// @relatedalso RevoluteJoint
Angle GetJointAngle(const RevoluteJoint& joint);
    
/// @brief Gets the current joint angle speed.
/// @relatedalso RevoluteJoint
AngularVelocity GetAngularVelocity(const RevoluteJoint& joint);

} // namespace playrho

#endif // PLAYRHO_DYNAMICS_JOINTS_REVOLUTEJOINT_HPP
