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

#ifndef B2_WELD_JOINT_H
#define B2_WELD_JOINT_H

#include <PlayRho/Dynamics/Joints/Joint.hpp>
#include <PlayRho/Dynamics/Joints/WeldJointDef.hpp>

namespace playrho {

/// @brief Weld joint.
/// @details A weld joint essentially glues two bodies together. A weld joint may
/// distort somewhat because the island constraint solver is approximate.
class WeldJoint : public Joint
{
public:
    WeldJoint(const WeldJointDef& def);

    Length2D GetAnchorA() const override;
    Length2D GetAnchorB() const override;

    Momentum2D GetLinearReaction() const override;
    AngularMomentum GetAngularReaction() const override;

    /// The local anchor point relative to bodyA's origin.
    Length2D GetLocalAnchorA() const { return m_localAnchorA; }

    /// The local anchor point relative to bodyB's origin.
    Length2D GetLocalAnchorB() const  { return m_localAnchorB; }

    /// Get the reference angle.
    Angle GetReferenceAngle() const { return m_referenceAngle; }

    /// @brief Sets frequency.
    void SetFrequency(Frequency frequency) { m_frequency = frequency; }

    /// @brief Gets the frequency.
    Frequency GetFrequency() const { return m_frequency; }

    /// Set/get damping ratio.
    void SetDampingRatio(Real ratio) { m_dampingRatio = ratio; }
    Real GetDampingRatio() const { return m_dampingRatio; }

private:

    void InitVelocityConstraints(BodyConstraintsMap& bodies, const StepConf& step,
                                 const ConstraintSolverConf&) override;
    bool SolveVelocityConstraints(BodyConstraintsMap& bodies, const StepConf& step) override;
    bool SolvePositionConstraints(BodyConstraintsMap& bodies,
                                  const ConstraintSolverConf& conf) const override;

    Length2D m_localAnchorA;
    Length2D m_localAnchorB;
    Angle m_referenceAngle;
    Frequency m_frequency;
    Real m_dampingRatio;

    // Solver shared
    Vec3 m_impulse = Vec3_zero;

    // Solver temp
    InvRotInertia m_gamma;
    AngularVelocity m_bias;
    Length2D m_rA;
    Length2D m_rB;
    Mat33 m_mass;
};

} // namespace playrho

#endif
