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

#ifndef B2_PRISMATIC_JOINT_H
#define B2_PRISMATIC_JOINT_H

#include <PlayRho/Dynamics/Joints/Joint.hpp>
#include <PlayRho/Dynamics/Joints/PrismaticJointDef.hpp>
#include <PlayRho/Common/BoundedValue.hpp>

namespace playrho {

/// @brief Prismatic Joint.
///
/// @details This joint provides one degree of freedom: translation along an axis fixed
///   in bodyA. Relative rotation is prevented.
///
/// @note You can use a joint limit to restrict the range of motion and a joint motor
///   to drive the motion or to model joint friction.
///
class PrismaticJoint : public Joint
{
public:
    PrismaticJoint(const PrismaticJointDef& def);

    Length2D GetAnchorA() const override;
    Length2D GetAnchorB() const override;

    Momentum2D GetLinearReaction() const override;
    AngularMomentum GetAngularReaction() const override;

    /// @brief Gets the local anchor point relative to bodyA's origin.
    Length2D GetLocalAnchorA() const { return m_localAnchorA; }

    /// @brief Gets the local anchor point relative to bodyB's origin.
    Length2D GetLocalAnchorB() const  { return m_localAnchorB; }

    /// @brief Gets local joint axis relative to bodyA.
    UnitVec2 GetLocalAxisA() const { return m_localXAxisA; }

    /// @brief Gets the reference angle.
    Angle GetReferenceAngle() const { return m_referenceAngle; }

    /// @brief Is the joint limit enabled?
    bool IsLimitEnabled() const noexcept;

    /// Enable/disable the joint limit.
    void EnableLimit(bool flag) noexcept;

    /// @brief Gets the lower joint limit.
    Length GetLowerLimit() const noexcept;

    /// @brief Gets the upper joint limit.
    Length GetUpperLimit() const noexcept;

    /// @brief Sets the joint limits.
    void SetLimits(Length lower, Length upper) noexcept;

    /// Is the joint motor enabled?
    bool IsMotorEnabled() const noexcept;

    /// Enable/disable the joint motor.
    void EnableMotor(bool flag) noexcept;

    /// @brief Sets the motor speed.
    void SetMotorSpeed(AngularVelocity speed) noexcept;

    /// @brief Gets the motor speed.
    AngularVelocity GetMotorSpeed() const noexcept;

    /// @brief Sets the maximum motor force.
    void SetMaxMotorForce(Force force) noexcept;
    Force GetMaxMotorForce() const noexcept { return m_maxMotorForce; }

    /// @brief Gets the current motor force given the inverse time step.
    Force GetMotorForce(Frequency inv_dt) const noexcept;

    /// @brief Gets the current limit state.
    /// @note This will be <code>e_inactiveLimit</code> unless the joint limit has been
    ///   enabled.
    LimitState GetLimitState() const noexcept;
    
private:
    void InitVelocityConstraints(BodyConstraintsMap& bodies, const StepConf& step,
                                 const ConstraintSolverConf& conf) override;
    bool SolveVelocityConstraints(BodyConstraintsMap& bodies, const StepConf& step) override;
    bool SolvePositionConstraints(BodyConstraintsMap& bodies,
                                  const ConstraintSolverConf& conf) const override;

    // Solver shared
    Length2D m_localAnchorA;
    Length2D m_localAnchorB;
    UnitVec2 m_localXAxisA;
    UnitVec2 m_localYAxisA;
    Angle m_referenceAngle;
    Vec3 m_impulse = Vec3_zero;
    Momentum m_motorImpulse = 0;
    Length m_lowerTranslation;
    Length m_upperTranslation;
    Force m_maxMotorForce;
    AngularVelocity m_motorSpeed;
    bool m_enableLimit;
    bool m_enableMotor;
    LimitState m_limitState = e_inactiveLimit;

    // Solver temp
    UnitVec2 m_axis = UnitVec2::GetZero();
    UnitVec2 m_perp = UnitVec2::GetZero();
    Length m_s1;
    Length m_s2;
    Length m_a1;
    Length m_a2;
    Mat33 m_K;
    Mass m_motorMass = Mass{0};
};

inline Length PrismaticJoint::GetLowerLimit() const noexcept
{
    return m_lowerTranslation;
}

inline Length PrismaticJoint::GetUpperLimit() const noexcept
{
    return m_upperTranslation;
}

inline bool PrismaticJoint::IsLimitEnabled() const noexcept
{
    return m_enableLimit;
}

inline bool PrismaticJoint::IsMotorEnabled() const noexcept
{
    return m_enableMotor;
}

inline AngularVelocity PrismaticJoint::GetMotorSpeed() const noexcept
{
    return m_motorSpeed;
}

inline Joint::LimitState PrismaticJoint::GetLimitState() const noexcept
{
    return m_limitState;
}

/// Get the current joint translation.
Length GetJointTranslation(const PrismaticJoint& joint) noexcept;

/// Get the current joint translation speed.
LinearVelocity GetLinearVelocity(const PrismaticJoint& joint) noexcept;

} // namespace playrho

#endif
