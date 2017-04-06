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

#ifndef B2_REVOLUTE_JOINT_H
#define B2_REVOLUTE_JOINT_H

#include <Box2D/Dynamics/Joints/Joint.hpp>

namespace box2d {

/// Revolute joint definition. This requires defining an
/// anchor point where the bodies are joined. The definition
/// uses local anchor points so that the initial configuration
/// can violate the constraint slightly. You also need to
/// specify the initial relative angle for joint limits. This
/// helps when saving and loading a game.
/// The local anchor points are measured from the body's origin
/// rather than the center of mass because:
/// 1. you might not know where the center of mass will be.
/// 2. if you add/remove shapes from a body and recompute the mass,
///    the joints will be broken.
struct RevoluteJointDef : public JointDef
{
	constexpr RevoluteJointDef() noexcept: JointDef{JointType::Revolute} {}

	/// Initialize the bodies, anchors, and reference angle using a world
	/// anchor point.
	RevoluteJointDef(Body* bodyA, Body* bodyB, const Vec2 anchor, bool cc = false);

	/// The local anchor point relative to bodyA's origin.
	Vec2 localAnchorA = Vec2_zero;

	/// The local anchor point relative to bodyB's origin.
	Vec2 localAnchorB = Vec2_zero;

	/// The bodyB angle minus bodyA angle in the reference state (radians).
	Angle referenceAngle = Angle{0};

	/// A flag to enable joint limits.
	bool enableLimit = false;

	/// The lower angle for the joint limit (radians).
	Angle lowerAngle = Angle{0};

	/// The upper angle for the joint limit (radians).
	Angle upperAngle = Angle{0};

	/// A flag to enable the joint motor.
	bool enableMotor = false;

	/// The desired motor speed.
	AngularVelocity motorSpeed = AngularVelocity{0};

	/// The maximum motor torque used to achieve the desired motor speed.
	/// Usually in N-m.
	RealNum maxMotorTorque = 0;
};

/// Revolute Joint.
///
/// @detail A revolute joint constrains two bodies to share a common point while they
/// are free to rotate about the point. The relative rotation about the shared
/// point is the joint angle.
///
/// @note You can limit the relative rotation with a joint limit that specifies a
/// lower and upper angle. You can use a motor
/// to drive the relative rotation about the shared point. A maximum motor torque
/// is provided so that infinite forces are not generated.
///
class RevoluteJoint : public Joint
{
public:
	RevoluteJoint(const RevoluteJointDef& def);

	Vec2 GetAnchorA() const override;
	Vec2 GetAnchorB() const override;

	/// The local anchor point relative to bodyA's origin.
	Vec2 GetLocalAnchorA() const noexcept { return m_localAnchorA; }

	/// The local anchor point relative to bodyB's origin.
	Vec2 GetLocalAnchorB() const noexcept { return m_localAnchorB; }

	/// Get the reference angle.
	Angle GetReferenceAngle() const noexcept { return m_referenceAngle; }

	/// Is the joint limit enabled?
	bool IsLimitEnabled() const noexcept;

	/// Enable/disable the joint limit.
	void EnableLimit(bool flag);

	/// Get the lower joint limit in radians.
	Angle GetLowerLimit() const noexcept;

	/// Get the upper joint limit in radians.
	Angle GetUpperLimit() const noexcept;

	/// Set the joint limits in radians.
	void SetLimits(Angle lower, Angle upper);

	/// Is the joint motor enabled?
	bool IsMotorEnabled() const noexcept;

	/// Enable/disable the joint motor.
	void EnableMotor(bool flag);

	/// Set the motor speed in radians per second.
	void SetMotorSpeed(AngularVelocity speed);

	/// Get the motor speed in radians per second.
	AngularVelocity GetMotorSpeed() const noexcept;

	/// Set the maximum motor torque, usually in N-m.
	void SetMaxMotorTorque(RealNum torque);

	RealNum GetMaxMotorTorque() const noexcept { return m_maxMotorTorque; }

	/// Get the reaction force given the inverse time step.
	/// Unit is N.
	Vec2 GetReactionForce(Frequency inv_dt) const override;

	/// Get the reaction torque due to the joint limit given the inverse time step.
	/// Unit is N*m.
	RealNum GetReactionTorque(Frequency inv_dt) const override;

	/// Get the current motor torque given the inverse time step.
	/// Unit is N*m.
	RealNum GetMotorTorque(RealNum inv_dt) const;

private:
	
	void InitVelocityConstraints(BodyConstraints& bodies,
								 const StepConf& step, const ConstraintSolverConf& conf) override;

	RealNum SolveVelocityConstraints(BodyConstraints& bodies, const StepConf& step) override;
	
	bool SolvePositionConstraints(BodyConstraints& bodies, const ConstraintSolverConf& conf) const override;

	// Solver shared
	Vec2 m_localAnchorA;
	Vec2 m_localAnchorB;
	Vec3 m_impulse = Vec3_zero; ///< Impulse. Mofified by: InitVelocityConstraints, SolveVelocityConstraints.
	RealNum m_motorImpulse = 0; ///< Motor impulse. Modified by: InitVelocityConstraints, SolveVelocityConstraints.

	bool m_enableMotor;
	RealNum m_maxMotorTorque;
	AngularVelocity m_motorSpeed;

	bool m_enableLimit;
	Angle m_referenceAngle;
	Angle m_lowerAngle;
	Angle m_upperAngle;

	// Solver cached temporary data. Values set by by InitVelocityConstraints.

	Vec2 m_rA; ///< Rotated delta of body A's local center from local anchor A.
	Vec2 m_rB; ///< Rotated delta of body B's local center from local anchor B.
	Vec2 m_localCenterA; ///< Local center of body A.
	Vec2 m_localCenterB; ///< Local center of body B.
	RealNum m_invMassA; ///< Inverse mass of body A.
	RealNum m_invMassB; ///< Inverse mass of body B.
	RealNum m_invIA; ///< Inverse inertia of body A.
	RealNum m_invIB; ///< Inverse inertia of body B.
	Mat33 m_mass; ///< Effective mass for point-to-point constraint.
	RealNum m_motorMass; ///< Effective mass for motor/limit angular constraint.
	LimitState m_limitState = e_inactiveLimit; ///< Limit state.
};

inline bool RevoluteJoint::IsLimitEnabled() const noexcept
{
	return m_enableLimit;
}

inline Angle RevoluteJoint::GetLowerLimit() const noexcept
{
	return m_lowerAngle;
}

inline Angle RevoluteJoint::GetUpperLimit() const noexcept
{
	return m_upperAngle;
}

inline bool RevoluteJoint::IsMotorEnabled() const noexcept
{
	return m_enableMotor;
}

inline AngularVelocity RevoluteJoint::GetMotorSpeed() const noexcept
{
	return m_motorSpeed;
}

/// Get the current joint angle in radians.
Angle GetJointAngle(const RevoluteJoint& joint);
	
/// Get the current joint angle speed in radians per second.
AngularVelocity GetJointSpeed(const RevoluteJoint& joint);

} // namespace box2d

#endif
