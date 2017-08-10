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

#ifndef B2_DISTANCE_JOINT_H
#define B2_DISTANCE_JOINT_H

#include <PlayRho/Dynamics/Joints/Joint.hpp>
#include <PlayRho/Dynamics/Joints/DistanceJointDef.hpp>

namespace playrho {

/// @brief Distance Joint.
/// @details
/// A distance joint constrains two points on two bodies
/// to remain at a fixed distance from each other. You can view
/// this as a massless, rigid rod.
class DistanceJoint : public Joint
{
public:
    static bool IsOkay(const DistanceJointDef& data) noexcept;

    DistanceJoint(const DistanceJointDef& data);

    Length2D GetAnchorA() const override;
    Length2D GetAnchorB() const override;

    /// @brief Get the reaction force given the inverse time step.
    Force2D GetReactionForce(Frequency inv_dt) const override;

    /// @brief Gets the reaction torque given the inverse time step.
    /// @note This is always zero for a distance joint.
    Torque GetReactionTorque(Frequency inv_dt) const override;

    /// @brief Gets the local anchor point relative to bodyA's origin.
    Length2D GetLocalAnchorA() const noexcept { return m_localAnchorA; }

    /// @brief Gets the local anchor point relative to bodyB's origin.
    Length2D GetLocalAnchorB() const noexcept { return m_localAnchorB; }

    /// Set/get the natural length.
    /// Manipulating the length can lead to non-physical behavior when the frequency is zero.
    void SetLength(Length length) noexcept;
    Length GetLength() const noexcept;

    /// @brief Sets frequency.
    void SetFrequency(NonNegative<Frequency> frequency) noexcept;

    /// @brief Gets the frequency.
    NonNegative<Frequency> GetFrequency() const noexcept;

    /// Set/get damping ratio.
    void SetDampingRatio(Real ratio) noexcept;
    Real GetDampingRatio() const noexcept;

private:

    void InitVelocityConstraints(BodyConstraintsMap& bodies, const StepConf& step, const
                                 ConstraintSolverConf&) override;
    bool SolveVelocityConstraints(BodyConstraintsMap& bodies, const StepConf& step) override;
    bool SolvePositionConstraints(BodyConstraintsMap& bodies,
                                  const ConstraintSolverConf& conf) const override;

    Length2D m_localAnchorA;
    Length2D m_localAnchorB;
    Length m_length;
    NonNegative<Frequency> m_frequency = NonNegative<Frequency>{0};
    Real m_dampingRatio;

    // Solver shared
    Momentum m_impulse = Momentum{0};

    // Solver temp
    InvMass m_invGamma;
    LinearVelocity m_bias;
    Mass m_mass;
    UnitVec2 m_u;
    Length2D m_rA;
    Length2D m_rB;
};

inline void DistanceJoint::SetLength(Length length) noexcept
{
    m_length = length;
}

inline Length DistanceJoint::GetLength() const noexcept
{
    return m_length;
}

inline void DistanceJoint::SetFrequency(NonNegative<Frequency> hz) noexcept
{
    m_frequency = hz;
}

inline NonNegative<Frequency> DistanceJoint::GetFrequency() const noexcept
{
    return m_frequency;
}

inline void DistanceJoint::SetDampingRatio(Real ratio) noexcept
{
    m_dampingRatio = ratio;
}

inline Real DistanceJoint::GetDampingRatio() const noexcept
{
    return m_dampingRatio;
}

} // namespace playrho

#endif
