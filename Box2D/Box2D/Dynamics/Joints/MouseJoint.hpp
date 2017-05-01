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

#ifndef B2_MOUSE_JOINT_H
#define B2_MOUSE_JOINT_H

#include <Box2D/Dynamics/Joints/Joint.hpp>

namespace box2d {

/// Mouse joint definition. This requires a world target point,
/// tuning parameters, and the time step.
struct MouseJointDef : public JointDef
{
    constexpr MouseJointDef() noexcept: JointDef(JointType::Mouse) {}

    /// The initial world target point. This is assumed
    /// to coincide with the body anchor initially.
    Length2D target = Vec2_zero * Meter;

    /// Max force.
    /// @details
    /// The maximum constraint force that can be exerted
    /// to move the candidate body. Usually you will express
    /// as some multiple of the weight (multiplier * mass * gravity).
    /// @note This may not be negative.
    /// @warning Behavior is undefined if this is a negative value.
    Force maxForce = Force{0};

    /// Frequency.
    /// @details The has to do with the response speed.
    /// @note This value may not be negative.
    /// @warning Behavior is undefined if this is a negative value.
    Frequency frequencyHz = RealNum{5} * Hertz;

    /// The damping ratio. 0 = no damping, 1 = critical damping.
    RealNum dampingRatio = 0.7f;
};

/// Mouse Joint.
///
/// @details
/// A mouse joint is used to make a point on a body track a
///   specified world point. This a soft constraint with a maximum
///   force. This allows the constraint to stretch and without
///   applying huge forces.
/// @note This joint is not documented in the manual because it was
///   developed to be used in the testbed. If you want to learn how to
///   use the mouse joint, look at the testbed.
/// @note This structure is 120-bytes large (using a 4-byte RealNum on at least one 64-bit
///   architecture/build).
///
class MouseJoint : public Joint
{
public:
    static bool IsOkay(const MouseJointDef& def) noexcept;

    MouseJoint(const MouseJointDef& def);

    /// Implements Joint.
    Length2D GetAnchorA() const override;

    /// Implements Joint.
    Length2D GetAnchorB() const override;

    /// Implements Joint.
    Force2D GetReactionForce(Frequency inv_dt) const override;

    /// Implements Joint.
    Torque GetReactionTorque(Frequency inv_dt) const override;

    Length2D GetLocalAnchorB() const noexcept;

    /// Use this to update the target point.
    void SetTarget(const Length2D target) noexcept;
    Length2D GetTarget() const noexcept;

    /// Set/get the maximum force in Newtons.
    void SetMaxForce(Force force) noexcept;
    Force GetMaxForce() const noexcept;

    /// Set/get the frequency in Hertz.
    void SetFrequency(Frequency hz) noexcept;
    Frequency GetFrequency() const noexcept;

    /// Set/get the damping ratio (dimensionless).
    void SetDampingRatio(RealNum ratio) noexcept;
    RealNum GetDampingRatio() const noexcept;

    /// Implement Joint::ShiftOrigin
    void ShiftOrigin(const Length2D newOrigin) override;

private:
    void InitVelocityConstraints(BodyConstraints& bodies, const StepConf& step, const ConstraintSolverConf& conf) override;
    RealNum SolveVelocityConstraints(BodyConstraints& bodies, const StepConf& step) override;
    bool SolvePositionConstraints(BodyConstraints& bodies, const ConstraintSolverConf& conf) const override;

    Mat22 GetEffectiveMassMatrix(const BodyConstraint& body) const noexcept;

    Length2D m_targetA;
    Length2D m_localAnchorB;
    Frequency m_frequencyHz;
    RealNum m_dampingRatio;
    
    // Solver shared
    Momentum2D m_impulse = Vec2_zero * Kilogram * MeterPerSecond;
    Force m_maxForce;
    InvMass m_gamma = InvMass{0};

    // Solver variables. These are only valid after InitVelocityConstraints called.
    Length2D m_rB;
    Mat22 m_mass;
    LinearVelocity2D m_C;
};

inline Length2D MouseJoint::GetLocalAnchorB() const noexcept
{
    return m_localAnchorB;
}

inline Length2D MouseJoint::GetAnchorA() const
{
    return m_targetA;
}

inline Length2D MouseJoint::GetTarget() const noexcept
{
    return m_targetA;
}

inline void MouseJoint::SetMaxForce(Force force) noexcept
{
    m_maxForce = force;
}

inline Force MouseJoint::GetMaxForce() const noexcept
{
    return m_maxForce;
}

inline void MouseJoint::SetFrequency(Frequency hz) noexcept
{
    m_frequencyHz = hz;
}

inline Frequency MouseJoint::GetFrequency() const noexcept
{
    return m_frequencyHz;
}

inline void MouseJoint::SetDampingRatio(RealNum ratio) noexcept
{
    m_dampingRatio = ratio;
}

inline RealNum MouseJoint::GetDampingRatio() const noexcept
{
    return m_dampingRatio;
}

} // namespace box2d

#endif
