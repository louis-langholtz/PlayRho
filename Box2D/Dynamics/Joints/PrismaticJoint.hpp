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

#ifndef B2_PRISMATIC_JOINT_H
#define B2_PRISMATIC_JOINT_H

#include <Box2D/Dynamics/Joints/Joint.hpp>

namespace box2d {

/// Prismatic joint definition. This requires defining a line of
/// motion using an axis and an anchor point. The definition uses local
/// anchor points and a local axis so that the initial configuration
/// can violate the constraint slightly. The joint translation is zero
/// when the local anchor points coincide in world space. Using local
/// anchors and a local axis helps when saving and loading a game.
struct PrismaticJointDef : public JointDef
{
    constexpr PrismaticJointDef() noexcept: JointDef(JointType::Prismatic) {}

    PrismaticJointDef(const PrismaticJointDef& copy) = default;
    
    /// Initialize the bodies, anchors, axis, and reference angle using the world
    /// anchor and unit world axis.
    PrismaticJointDef(Body* bodyA, Body* bodyB, const Length2D anchor, const UnitVec2 axis) noexcept;

    /// The local anchor point relative to bodyA's origin.
    Length2D localAnchorA = Vec2_zero * Meter;

    /// The local anchor point relative to bodyB's origin.
    Length2D localAnchorB = Vec2_zero * Meter;

    /// The local translation unit axis in bodyA.
    UnitVec2 localAxisA = UnitVec2::GetRight();

    /// The constrained angle between the bodies: bodyB_angle - bodyA_angle.
    Angle referenceAngle = Angle{0};

    /// Enable/disable the joint limit.
    bool enableLimit = false;

    /// The lower translation limit.
    Length lowerTranslation = Length{0};

    /// The upper translation limit.
    Length upperTranslation = Length{0};

    /// Enable/disable the joint motor.
    bool enableMotor = false;

    /// The maximum motor torque.
    Torque maxMotorTorque = Torque{0};

    /// The desired angular motor speed.
    AngularVelocity motorSpeed = AngularVelocity{0};
};

/// Prismatic Joint.
///
/// @details This joint provides one degree of freedom: translation along an axis fixed
/// in bodyA. Relative rotation is prevented.
///
/// @note You can use a joint limit to restrict the range of motion and a joint motor
/// to drive the motion or to model joint friction.
///
class PrismaticJoint : public Joint
{
public:
    PrismaticJoint(const PrismaticJointDef& def);

    Length2D GetAnchorA() const override;
    Length2D GetAnchorB() const override;

    Force2D GetReactionForce(Frequency inv_dt) const override;
    Torque GetReactionTorque(Frequency inv_dt) const override;

    /// The local anchor point relative to bodyA's origin.
    Length2D GetLocalAnchorA() const { return m_localAnchorA; }

    /// The local anchor point relative to bodyB's origin.
    Length2D GetLocalAnchorB() const  { return m_localAnchorB; }

    /// The local joint axis relative to bodyA.
    UnitVec2 GetLocalAxisA() const { return m_localXAxisA; }

    /// Get the reference angle.
    Angle GetReferenceAngle() const { return m_referenceAngle; }

    /// Get the current joint translation.
    Length GetJointTranslation() const;

    /// Get the current joint translation speed.
    LinearVelocity GetJointSpeed() const;

    /// Is the joint limit enabled?
    bool IsLimitEnabled() const noexcept;

    /// Enable/disable the joint limit.
    void EnableLimit(bool flag) noexcept;

    /// Get the lower joint limit.
    Length GetLowerLimit() const noexcept;

    /// Get the upper joint limit.
    Length GetUpperLimit() const noexcept;

    /// Set the joint limits.
    void SetLimits(Length lower, Length upper);

    /// Is the joint motor enabled?
    bool IsMotorEnabled() const noexcept;

    /// Enable/disable the joint motor.
    void EnableMotor(bool flag) noexcept;

    /// Set the motor speed.
    void SetMotorSpeed(AngularVelocity speed) noexcept;

    /// Get the motor speed.
    AngularVelocity GetMotorSpeed() const noexcept;

    /// Set the maximum motor force.
    void SetMaxMotorForce(Force force) noexcept;
    Force GetMaxMotorForce() const noexcept { return m_maxMotorForce; }

    /// Get the current motor force given the inverse time step.
    Force GetMotorForce(Frequency inv_dt) const noexcept;

private:
    void InitVelocityConstraints(BodyConstraints& bodies, const StepConf& step, const ConstraintSolverConf& conf) override;
    bool SolveVelocityConstraints(BodyConstraints& bodies, const StepConf& step) override;
    bool SolvePositionConstraints(BodyConstraints& bodies, const ConstraintSolverConf& conf) const override;

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

inline AngularVelocity PrismaticJoint::GetMotorSpeed() const noexcept
{
    return m_motorSpeed;
}

PrismaticJointDef GetPrismaticJointDef(const PrismaticJoint& joint) noexcept;

} // namespace box2d

#endif
