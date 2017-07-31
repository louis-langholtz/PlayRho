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

#ifndef B2_MOTOR_JOINT_H
#define B2_MOTOR_JOINT_H

#include <PlayRho/Dynamics/Joints/Joint.hpp>
#include <PlayRho/Dynamics/Joints/MotorJointDef.hpp>

namespace playrho {

/// @brief Motor joint.
/// @details A motor joint is used to control the relative motion
/// between two bodies. A typical usage is to control the movement
/// of a dynamic body with respect to the ground.
class MotorJoint : public Joint
{
public:
    MotorJoint(const MotorJointDef& def);

    Length2D GetAnchorA() const override;
    Length2D GetAnchorB() const override;

    Force2D GetReactionForce(Frequency inv_dt) const override;
    Torque GetReactionTorque(Frequency inv_dt) const override;

    /// Set/get the target linear offset, in frame A.
    void SetLinearOffset(const Length2D linearOffset);
    const Length2D GetLinearOffset() const;

    /// Set/get the target angular offset.
    void SetAngularOffset(Angle angularOffset);
    Angle GetAngularOffset() const;

    /// @brief Sets the maximum friction force.
    void SetMaxForce(NonNegative<Force> force);

    /// @brief Gets the maximum friction force.
    NonNegative<Force> GetMaxForce() const;

    /// Set the maximum friction torque.
    void SetMaxTorque(NonNegative<Torque> torque);

    /// Get the maximum friction torque.
    NonNegative<Torque> GetMaxTorque() const;

    /// Set the position correction factor in the range [0,1].
    void SetCorrectionFactor(Real factor);

    /// Get the position correction factor in the range [0,1].
    Real GetCorrectionFactor() const;

private:

    void InitVelocityConstraints(BodyConstraintsMap& bodies, const StepConf& step, const ConstraintSolverConf& conf) override;
    bool SolveVelocityConstraints(BodyConstraintsMap& bodies, const StepConf& step) override;
    bool SolvePositionConstraints(BodyConstraintsMap& bodies, const ConstraintSolverConf& conf) const override;

    // Solver shared
    Length2D m_linearOffset;
    Angle m_angularOffset;
    Momentum2D m_linearImpulse = Momentum2D{0, 0};
    AngularMomentum m_angularImpulse = AngularMomentum{0};
    NonNegative<Force> m_maxForce;
    NonNegative<Torque> m_maxTorque;
    Real m_correctionFactor;

    // Solver temp
    Length2D m_rA;
    Length2D m_rB;
    Length2D m_linearError;
    Angle m_angularError;
    Mat22 m_linearMass; ///< 2x2 linear mass matrix in kilograms.
    RotInertia m_angularMass;
};

inline void MotorJoint::SetMaxForce(NonNegative<Force> force)
{
    m_maxForce = force;
}

inline NonNegative<Force> MotorJoint::GetMaxForce() const
{
    return m_maxForce;
}

inline void MotorJoint::SetMaxTorque(NonNegative<Torque> torque)
{
    m_maxTorque = torque;
}

inline NonNegative<Torque> MotorJoint::GetMaxTorque() const
{
    return m_maxTorque;
}

} // namespace playrho

#endif
