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

#ifndef B2_MOUSE_JOINT_H
#define B2_MOUSE_JOINT_H

#include <PlayRho/Dynamics/Joints/Joint.hpp>
#include <PlayRho/Dynamics/Joints/MouseJointDef.hpp>

namespace playrho {

/// @brief Mouse Joint.
///
/// @details A mouse joint is used to make a point on a body track a
///   specified world point. This a soft constraint with a maximum
///   force. This allows the constraint to stretch and without
///   applying huge forces.
/// @note This joint is not documented in the manual because it was
///   developed to be used in the testbed. If you want to learn how to
///   use the mouse joint, look at the testbed.
/// @note This structure is 120-bytes large (using a 4-byte Real on at least one 64-bit
///   architecture/build).
///
class MouseJoint : public Joint
{
public:

    /// @brief Is the given definition okay.
    static bool IsOkay(const MouseJointDef& def) noexcept;

    /// @brief Initializing constructor.
    MouseJoint(const MouseJointDef& def);

    Length2D GetAnchorA() const override;

    Length2D GetAnchorB() const override;

    Momentum2D GetLinearReaction() const override;

    AngularMomentum GetAngularReaction() const override;

    /// @brief Gets the local anchor B.
    Length2D GetLocalAnchorB() const noexcept;

    /// @brief Sets the target point.
    void SetTarget(const Length2D target) noexcept;

    /// @brief Gets the target point.
    Length2D GetTarget() const noexcept;

    /// @brief Sets the maximum force.
    void SetMaxForce(NonNegative<Force> force) noexcept;

    /// @brief Gets the maximum force.
    NonNegative<Force> GetMaxForce() const noexcept;

    /// @brief Sets the frequency.
    void SetFrequency(NonNegative<Frequency> hz) noexcept;

    /// @brief Gets the frequency.
    NonNegative<Frequency> GetFrequency() const noexcept;

    /// @brief Sets the damping ratio.
    void SetDampingRatio(NonNegative<Real> ratio) noexcept;

    /// @brief Gets the damping ratio.
    NonNegative<Real> GetDampingRatio() const noexcept;

    /// Implement Joint::ShiftOrigin
    void ShiftOrigin(const Length2D newOrigin) override;

private:
    void InitVelocityConstraints(BodyConstraintsMap& bodies, const StepConf& step, const ConstraintSolverConf& conf) override;
    bool SolveVelocityConstraints(BodyConstraintsMap& bodies, const StepConf& step) override;
    bool SolvePositionConstraints(BodyConstraintsMap& bodies, const ConstraintSolverConf& conf) const override;

    Mass22 GetEffectiveMassMatrix(const BodyConstraint& body) const noexcept;

    Length2D m_targetA;
    Length2D m_localAnchorB;
    NonNegative<Frequency> m_frequency = NonNegative<Frequency>{0};
    NonNegative<Real> m_dampingRatio = NonNegative<Real>{0};
    
    // Solver shared
    Momentum2D m_impulse = Momentum2D{};
    NonNegative<Force> m_maxForce = NonNegative<Force>{0};
    InvMass m_gamma = InvMass{0};

    // Solver variables. These are only valid after InitVelocityConstraints called.
    Length2D m_rB;
    Mass22 m_mass; ///< 2x2 mass matrix in kilograms.
    LinearVelocity2D m_C;
};

inline Length2D MouseJoint::GetLocalAnchorB() const noexcept
{
    return m_localAnchorB;
}

inline Length2D MouseJoint::GetTarget() const noexcept
{
    return m_targetA;
}

inline void MouseJoint::SetMaxForce(NonNegative<Force> force) noexcept
{
    m_maxForce = force;
}

inline NonNegative<Force> MouseJoint::GetMaxForce() const noexcept
{
    return m_maxForce;
}

inline void MouseJoint::SetFrequency(NonNegative<Frequency> hz) noexcept
{
    m_frequency = hz;
}

inline NonNegative<Frequency> MouseJoint::GetFrequency() const noexcept
{
    return m_frequency;
}

inline void MouseJoint::SetDampingRatio(NonNegative<Real> ratio) noexcept
{
    m_dampingRatio = ratio;
}

inline NonNegative<Real> MouseJoint::GetDampingRatio() const noexcept
{
    return m_dampingRatio;
}

} // namespace playrho

#endif
