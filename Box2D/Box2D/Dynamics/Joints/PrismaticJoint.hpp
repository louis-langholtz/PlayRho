/*
* Original work Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
* Modified work Copyright (c) 2016 Louis Langholtz https://github.com/louis-langholtz/Box2D
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
	PrismaticJointDef(Body* bodyA, Body* bodyB, const Vec2 anchor, const Vec2 axis) noexcept;

	/// The local anchor point relative to bodyA's origin.
	Vec2 localAnchorA = Vec2_zero;

	/// The local anchor point relative to bodyB's origin.
	Vec2 localAnchorB = Vec2_zero;

	/// The local translation unit axis in bodyA.
	Vec2 localAxisA = Vec2{realnum{1}, realnum{0}};

	/// The constrained angle between the bodies: bodyB_angle - bodyA_angle.
	Angle referenceAngle = 0_rad;

	/// Enable/disable the joint limit.
	bool enableLimit = false;

	/// The lower translation limit, usually in meters.
	realnum lowerTranslation = realnum{0};

	/// The upper translation limit, usually in meters.
	realnum upperTranslation = realnum{0};

	/// Enable/disable the joint motor.
	bool enableMotor = false;

	/// The maximum motor torque, usually in N-m.
	realnum maxMotorForce = realnum{0};

	/// The desired motor speed in radians per second.
	realnum motorSpeed = realnum{0};
};

/// A prismatic joint. This joint provides one degree of freedom: translation
/// along an axis fixed in bodyA. Relative rotation is prevented. You can
/// use a joint limit to restrict the range of motion and a joint motor to
/// drive the motion or to model joint friction.
class PrismaticJoint : public Joint
{
public:
	PrismaticJoint(const PrismaticJointDef& def);

	Vec2 GetAnchorA() const override;
	Vec2 GetAnchorB() const override;

	Vec2 GetReactionForce(realnum inv_dt) const override;
	realnum GetReactionTorque(realnum inv_dt) const override;

	/// The local anchor point relative to bodyA's origin.
	Vec2 GetLocalAnchorA() const { return m_localAnchorA; }

	/// The local anchor point relative to bodyB's origin.
	Vec2 GetLocalAnchorB() const  { return m_localAnchorB; }

	/// The local joint axis relative to bodyA.
	UnitVec2 GetLocalAxisA() const { return m_localXAxisA; }

	/// Get the reference angle.
	Angle GetReferenceAngle() const { return m_referenceAngle; }

	/// Get the current joint translation, usually in meters.
	realnum GetJointTranslation() const;

	/// Get the current joint translation speed, usually in meters per second.
	realnum GetJointSpeed() const;

	/// Is the joint limit enabled?
	bool IsLimitEnabled() const noexcept;

	/// Enable/disable the joint limit.
	void EnableLimit(bool flag) noexcept;

	/// Get the lower joint limit, usually in meters.
	realnum GetLowerLimit() const noexcept;

	/// Get the upper joint limit, usually in meters.
	realnum GetUpperLimit() const noexcept;

	/// Set the joint limits, usually in meters.
	void SetLimits(realnum lower, realnum upper);

	/// Is the joint motor enabled?
	bool IsMotorEnabled() const noexcept;

	/// Enable/disable the joint motor.
	void EnableMotor(bool flag) noexcept;

	/// Set the motor speed, usually in meters per second.
	void SetMotorSpeed(realnum speed) noexcept;

	/// Get the motor speed, usually in meters per second.
	realnum GetMotorSpeed() const noexcept;

	/// Set the maximum motor force, usually in N.
	void SetMaxMotorForce(realnum force) noexcept;
	realnum GetMaxMotorForce() const noexcept { return m_maxMotorForce; }

	/// Get the current motor force given the inverse time step, usually in N.
	realnum GetMotorForce(realnum inv_dt) const noexcept;

private:
	friend class GearJoint;

	void InitVelocityConstraints(Span<Velocity> velocities, Span<const Position> positions, const StepConf& step, const ConstraintSolverConf& conf) override;
	void SolveVelocityConstraints(Span<Velocity> velocities, const StepConf& step) override;
	bool SolvePositionConstraints(Span<Position> positions, const ConstraintSolverConf& conf) override;

	// Solver shared
	Vec2 m_localAnchorA;
	Vec2 m_localAnchorB;
	UnitVec2 m_localXAxisA;
	UnitVec2 m_localYAxisA;
	Angle m_referenceAngle;
	Vec3 m_impulse;
	realnum m_motorImpulse;
	realnum m_lowerTranslation;
	realnum m_upperTranslation;
	realnum m_maxMotorForce;
	realnum m_motorSpeed;
	bool m_enableLimit;
	bool m_enableMotor;
	LimitState m_limitState;

	// Solver temp
	index_t m_indexA;
	index_t m_indexB;
	Vec2 m_localCenterA;
	Vec2 m_localCenterB;
	realnum m_invMassA;
	realnum m_invMassB;
	realnum m_invIA;
	realnum m_invIB;
	UnitVec2 m_axis;
	UnitVec2 m_perp;
	realnum m_s1, m_s2;
	realnum m_a1, m_a2;
	Mat33 m_K;
	realnum m_motorMass;
};

inline realnum PrismaticJoint::GetMotorSpeed() const noexcept
{
	return m_motorSpeed;
}

} // namespace box2d

#endif
