/*
* Original work Copyright (c) 2006-2012 Erin Catto http://www.box2d.org
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

#ifndef B2_MOTOR_JOINT_H
#define B2_MOTOR_JOINT_H

#include <Box2D/Dynamics/Joints/Joint.hpp>

namespace box2d {

/// Motor joint definition.
struct MotorJointDef : public JointDef
{
    constexpr MotorJointDef() noexcept: JointDef(JointType::Motor) {}

    /// Initialize the bodies and offsets using the current transforms.
    MotorJointDef(Body* bodyA, Body* bodyB) noexcept;

    /// Position of bodyB minus the position of bodyA, in bodyA's frame.
    Length2D linearOffset = Length2D(0, 0);

    /// The bodyB angle minus bodyA angle.
    Angle angularOffset = Angle{0};
    
    /// The maximum motor force.
    Force maxForce = Real{1} * Newton;

    /// The maximum motor torque.
    Torque maxTorque = Real{1} * NewtonMeter;

    /// Position correction factor in the range [0,1].
    Real correctionFactor = Real(0.3);
};

/// A motor joint is used to control the relative motion
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

    /// Set the maximum friction force.
    void SetMaxForce(Force force);

    /// Get the maximum friction force.
    Force GetMaxForce() const;

    /// Set the maximum friction torque.
    void SetMaxTorque(Torque torque);

    /// Get the maximum friction torque.
    Torque GetMaxTorque() const;

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
    Force m_maxForce;
    Torque m_maxTorque;
    Real m_correctionFactor;

    // Solver temp
    Length2D m_rA;
    Length2D m_rB;
    Length2D m_linearError;
    Angle m_angularError;
    Mat22 m_linearMass;
    RotInertia m_angularMass;
};

MotorJointDef GetMotorJointDef(const MotorJoint& joint) noexcept;

} // namespace box2d

#endif
