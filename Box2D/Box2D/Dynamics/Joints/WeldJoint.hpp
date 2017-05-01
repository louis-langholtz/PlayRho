/*
* Original work Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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

#ifndef B2_WELD_JOINT_H
#define B2_WELD_JOINT_H

#include <Box2D/Dynamics/Joints/Joint.hpp>

namespace box2d {

/// Weld joint definition. You need to specify local anchor points
/// where they are attached and the relative body angle. The position
/// of the anchor points is important for computing the reaction torque.
struct WeldJointDef : public JointDef
{
    constexpr WeldJointDef() noexcept: JointDef(JointType::Weld) {}

    /// Initialize the bodies, anchors, and reference angle using a world
    /// anchor point.
    void Initialize(Body* bodyA, Body* bodyB, const Length2D anchor);

    /// The local anchor point relative to bodyA's origin.
    Length2D localAnchorA = Vec2_zero * Meter;

    /// The local anchor point relative to bodyB's origin.
    Length2D localAnchorB = Vec2_zero * Meter;

    /// The bodyB angle minus bodyA angle in the reference state (radians).
    Angle referenceAngle = Angle{0};
    
    /// The mass-spring-damper frequency in Hertz. Rotation only.
    /// Disable softness with a value of 0.
    Frequency frequencyHz = Frequency{0};

    /// The damping ratio. 0 = no damping, 1 = critical damping.
    RealNum dampingRatio = 0;
};

/// A weld joint essentially glues two bodies together. A weld joint may
/// distort somewhat because the island constraint solver is approximate.
class WeldJoint : public Joint
{
public:
    WeldJoint(const WeldJointDef& def);

    Length2D GetAnchorA() const override;
    Length2D GetAnchorB() const override;

    Force2D GetReactionForce(Frequency inv_dt) const override;
    Torque GetReactionTorque(Frequency inv_dt) const override;

    /// The local anchor point relative to bodyA's origin.
    Length2D GetLocalAnchorA() const { return m_localAnchorA; }

    /// The local anchor point relative to bodyB's origin.
    Length2D GetLocalAnchorB() const  { return m_localAnchorB; }

    /// Get the reference angle.
    Angle GetReferenceAngle() const { return m_referenceAngle; }

    /// Set/get frequency in Hz.
    void SetFrequency(Frequency hz) { m_frequencyHz = hz; }
    Frequency GetFrequency() const { return m_frequencyHz; }

    /// Set/get damping ratio.
    void SetDampingRatio(RealNum ratio) { m_dampingRatio = ratio; }
    RealNum GetDampingRatio() const { return m_dampingRatio; }

private:

    void InitVelocityConstraints(BodyConstraints& bodies, const StepConf& step, const ConstraintSolverConf& conf) override;
    RealNum SolveVelocityConstraints(BodyConstraints& bodies, const StepConf& step) override;
    bool SolvePositionConstraints(BodyConstraints& bodies, const ConstraintSolverConf& conf) const override;

    Frequency m_frequencyHz;
    RealNum m_dampingRatio;
    AngularVelocity m_bias;

    // Solver shared
    Length2D m_localAnchorA;
    Length2D m_localAnchorB;
    Angle m_referenceAngle;
    InvRotInertia m_gamma;
    Vec3 m_impulse = Vec3_zero;

    // Solver temp
    Length2D m_rA;
    Length2D m_rB;
    Mat33 m_mass;
};

} // namespace box2d

#endif
