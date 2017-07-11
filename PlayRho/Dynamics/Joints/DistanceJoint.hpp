/*
 * Original work Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

#ifndef B2_DISTANCE_JOINT_H
#define B2_DISTANCE_JOINT_H

#include <PlayRho/Dynamics/Joints/Joint.hpp>

namespace box2d {

/// Distance joint definition. This requires defining an
/// anchor point on both bodies and the non-zero length of the
/// distance joint. The definition uses local anchor points
/// so that the initial configuration can violate the constraint
/// slightly. This helps when saving and loading a game.
/// @warning Do not use a zero or short length.
struct DistanceJointDef : public JointDef
{
    constexpr DistanceJointDef() noexcept: JointDef{JointType::Distance} {}
    DistanceJointDef(const DistanceJointDef& copy) = default;

    /// Initialize the bodies, anchors, and length using the world anchors.
    DistanceJointDef(Body* bodyA, Body* bodyB,
                     const Length2D anchorA = Length2D(0, 0),
                     const Length2D anchorB = Length2D(0, 0),
                     Frequency freq = Frequency{0}, Real damp = 0) noexcept;

    /// The local anchor point relative to bodyA's origin.
    Length2D localAnchorA = Length2D(0, 0);

    /// The local anchor point relative to bodyB's origin.
    Length2D localAnchorB = Length2D(0, 0);

    /// The natural length between the anchor points.
    Length length = Real{1} * Meter;

    /// Mass-spring-damper frequency in Hertz.
    /// @note 0 disables softness.
    /// @note Should be 0 or greater.
    Frequency frequency = Frequency{0};

    /// The damping ratio. 0 = no damping, 1 = critical damping.
    Real dampingRatio = 0;
};

/// Distance Joint.
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

    /// Get the reaction force given the inverse time step.
    /// Unit is N.
    Force2D GetReactionForce(Frequency inv_dt) const override;

    /// Get the reaction torque given the inverse time step.
    /// Unit is N*m. This is always zero for a distance joint.
    Torque GetReactionTorque(Frequency inv_dt) const override;

    /// The local anchor point relative to bodyA's origin.
    Length2D GetLocalAnchorA() const noexcept { return m_localAnchorA; }

    /// The local anchor point relative to bodyB's origin.
    Length2D GetLocalAnchorB() const noexcept { return m_localAnchorB; }

    /// Set/get the natural length.
    /// Manipulating the length can lead to non-physical behavior when the frequency is zero.
    void SetLength(Length length) noexcept;
    Length GetLength() const noexcept;

    /// Set/get frequency.
    void SetFrequency(Frequency frequency) noexcept;
    Frequency GetFrequency() const noexcept;

    /// Set/get damping ratio.
    void SetDampingRatio(Real ratio) noexcept;
    Real GetDampingRatio() const noexcept;

private:

    void InitVelocityConstraints(BodyConstraintsMap& bodies, const StepConf& step, const ConstraintSolverConf& conf) override;
    bool SolveVelocityConstraints(BodyConstraintsMap& bodies, const StepConf& step) override;
    bool SolvePositionConstraints(BodyConstraintsMap& bodies, const ConstraintSolverConf& conf) const override;

    Length2D m_localAnchorA;
    Length2D m_localAnchorB;
    Length m_length;
    Frequency m_frequency;
    Real m_dampingRatio;

    // Solver temp
    InvMass m_invGamma;
    Momentum m_impulse = Momentum{0};
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

inline void DistanceJoint::SetFrequency(Frequency hz) noexcept
{
    m_frequency = hz;
}

inline Frequency DistanceJoint::GetFrequency() const noexcept
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
    
DistanceJointDef GetDistanceJointDef(const DistanceJoint& joint) noexcept;

} // namespace box2d

#endif
