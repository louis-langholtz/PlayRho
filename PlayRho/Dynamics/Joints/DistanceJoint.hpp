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

#ifndef PLAYRHO_DYNAMICS_JOINTS_DISTANCEJOINT_HPP
#define PLAYRHO_DYNAMICS_JOINTS_DISTANCEJOINT_HPP

#include <PlayRho/Dynamics/Joints/Joint.hpp>
#include <PlayRho/Dynamics/Joints/DistanceJointDef.hpp>

namespace playrho {

/// @brief Distance Joint.
///
/// @details A distance joint constrains two points on two bodies to remain at a
///   fixed distance from each other. You can view this as a massless, rigid rod.
///
/// @ingroup JointsGroup
///
/// @image html distanceJoint.gif
///
class DistanceJoint : public Joint
{
public:
    
    /// @brief Is the given definition okay.
    static bool IsOkay(const DistanceJointDef& data) noexcept;

    /// @brief Initializing constructor.
    DistanceJoint(const DistanceJointDef& data);

    void Accept(JointVisitor& visitor) const override;
    void Accept(JointVisitor& visitor) override;

    Length2 GetAnchorA() const override;

    Length2 GetAnchorB() const override;

    /// @brief Get the linear reaction.
    Momentum2 GetLinearReaction() const override;

    /// @brief Gets the angular reaction.
    /// @note This is always zero for a distance joint.
    AngularMomentum GetAngularReaction() const override;

    /// @brief Gets the local anchor point relative to bodyA's origin.
    Length2 GetLocalAnchorA() const noexcept { return m_localAnchorA; }

    /// @brief Gets the local anchor point relative to bodyB's origin.
    Length2 GetLocalAnchorB() const noexcept { return m_localAnchorB; }

    /// @brief Sets the natural length.
    /// @note Manipulating the length can lead to non-physical behavior when the frequency is zero.
    void SetLength(Length length) noexcept;
    
    /// @brief Gets the length.
    Length GetLength() const noexcept;

    /// @brief Sets frequency.
    void SetFrequency(NonNegative<Frequency> frequency) noexcept;

    /// @brief Gets the frequency.
    NonNegative<Frequency> GetFrequency() const noexcept;

    /// @brief Sets the damping ratio.
    void SetDampingRatio(Real ratio) noexcept;

    /// @brief Gets damping ratio.
    Real GetDampingRatio() const noexcept;

private:

    void InitVelocityConstraints(BodyConstraintsMap& bodies, const StepConf& step, const
                                 ConstraintSolverConf&) override;
    bool SolveVelocityConstraints(BodyConstraintsMap& bodies, const StepConf& step) override;
    bool SolvePositionConstraints(BodyConstraintsMap& bodies,
                                  const ConstraintSolverConf& conf) const override;

    Length2 m_localAnchorA; ///< Local anchor A.
    Length2 m_localAnchorB; ///< Local anchor B.
    Length m_length; ///< Length.
    NonNegative<Frequency> m_frequency = NonNegative<Frequency>{0}; ///< Frequency.
    Real m_dampingRatio; ///< Damping ratio.

    // Solver shared
    Momentum m_impulse = Momentum{0}; ///< Impulse.

    // Solver temp
    InvMass m_invGamma; ///< Inverse gamma.
    LinearVelocity m_bias; ///< Bias.
    Mass m_mass; ///< Mass.
    UnitVec2 m_u; ///< "u" directional.
    Length2 m_rA; ///< Relative A position.
    Length2 m_rB; ///< Relative B position.
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

#endif // PLAYRHO_DYNAMICS_JOINTS_DISTANCEJOINT_HPP
