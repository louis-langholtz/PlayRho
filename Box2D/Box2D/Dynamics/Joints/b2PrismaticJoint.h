/*
* Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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

#include <Box2D/Dynamics/Joints/b2Joint.h>

namespace box2d {

/// Prismatic joint definition. This requires defining a line of
/// motion using an axis and an anchor point. The definition uses local
/// anchor points and a local axis so that the initial configuration
/// can violate the constraint slightly. The joint translation is zero
/// when the local anchor points coincide in world space. Using local
/// anchors and a local axis helps when saving and loading a game.
struct b2PrismaticJointDef : public b2JointDef
{
	constexpr b2PrismaticJointDef() noexcept: b2JointDef(e_prismaticJoint) {}

	/// Initialize the bodies, anchors, axis, and reference angle using the world
	/// anchor and unit world axis.
	void Initialize(b2Body* bodyA, b2Body* bodyB, const b2Vec2& anchor, const b2Vec2& axis);

	/// The local anchor point relative to bodyA's origin.
	b2Vec2 localAnchorA = b2Vec2_zero;

	/// The local anchor point relative to bodyB's origin.
	b2Vec2 localAnchorB = b2Vec2_zero;

	/// The local translation unit axis in bodyA.
	b2Vec2 localAxisA = b2Vec2{b2Float(1), b2Float{0}};

	/// The constrained angle between the bodies: bodyB_angle - bodyA_angle.
	b2Float referenceAngle = b2Float{0};

	/// Enable/disable the joint limit.
	bool enableLimit = false;

	/// The lower translation limit, usually in meters.
	b2Float lowerTranslation = b2Float{0};

	/// The upper translation limit, usually in meters.
	b2Float upperTranslation = b2Float{0};

	/// Enable/disable the joint motor.
	bool enableMotor = false;

	/// The maximum motor torque, usually in N-m.
	b2Float maxMotorForce = b2Float{0};

	/// The desired motor speed in radians per second.
	b2Float motorSpeed = b2Float{0};
};

/// A prismatic joint. This joint provides one degree of freedom: translation
/// along an axis fixed in bodyA. Relative rotation is prevented. You can
/// use a joint limit to restrict the range of motion and a joint motor to
/// drive the motion or to model joint friction.
class b2PrismaticJoint : public b2Joint
{
public:
	b2Vec2 GetAnchorA() const override;
	b2Vec2 GetAnchorB() const override;

	b2Vec2 GetReactionForce(b2Float inv_dt) const override;
	b2Float GetReactionTorque(b2Float inv_dt) const override;

	/// The local anchor point relative to bodyA's origin.
	b2Vec2 GetLocalAnchorA() const { return m_localAnchorA; }

	/// The local anchor point relative to bodyB's origin.
	b2Vec2 GetLocalAnchorB() const  { return m_localAnchorB; }

	/// The local joint axis relative to bodyA.
	b2Vec2 GetLocalAxisA() const { return m_localXAxisA; }

	/// Get the reference angle.
	b2Float GetReferenceAngle() const { return m_referenceAngle; }

	/// Get the current joint translation, usually in meters.
	b2Float GetJointTranslation() const;

	/// Get the current joint translation speed, usually in meters per second.
	b2Float GetJointSpeed() const;

	/// Is the joint limit enabled?
	bool IsLimitEnabled() const noexcept;

	/// Enable/disable the joint limit.
	void EnableLimit(bool flag) noexcept;

	/// Get the lower joint limit, usually in meters.
	b2Float GetLowerLimit() const noexcept;

	/// Get the upper joint limit, usually in meters.
	b2Float GetUpperLimit() const noexcept;

	/// Set the joint limits, usually in meters.
	void SetLimits(b2Float lower, b2Float upper);

	/// Is the joint motor enabled?
	bool IsMotorEnabled() const noexcept;

	/// Enable/disable the joint motor.
	void EnableMotor(bool flag) noexcept;

	/// Set the motor speed, usually in meters per second.
	void SetMotorSpeed(b2Float speed) noexcept;

	/// Get the motor speed, usually in meters per second.
	b2Float GetMotorSpeed() const noexcept;

	/// Set the maximum motor force, usually in N.
	void SetMaxMotorForce(b2Float force) noexcept;
	b2Float GetMaxMotorForce() const noexcept { return m_maxMotorForce; }

	/// Get the current motor force given the inverse time step, usually in N.
	b2Float GetMotorForce(b2Float inv_dt) const noexcept;

	/// Dump to b2Log
	void Dump() override;

protected:
	friend class b2Joint;
	friend class b2GearJoint;
	b2PrismaticJoint(const b2PrismaticJointDef* def);

	void InitVelocityConstraints(const b2SolverData& data) override;
	void SolveVelocityConstraints(const b2SolverData& data) override;
	bool SolvePositionConstraints(const b2SolverData& data) override;

	// Solver shared
	b2Vec2 m_localAnchorA;
	b2Vec2 m_localAnchorB;
	b2Vec2 m_localXAxisA;
	b2Vec2 m_localYAxisA;
	b2Float m_referenceAngle;
	b2Vec3 m_impulse;
	b2Float m_motorImpulse;
	b2Float m_lowerTranslation;
	b2Float m_upperTranslation;
	b2Float m_maxMotorForce;
	b2Float m_motorSpeed;
	bool m_enableLimit;
	bool m_enableMotor;
	b2LimitState m_limitState;

	// Solver temp
	index_t m_indexA;
	index_t m_indexB;
	b2Vec2 m_localCenterA;
	b2Vec2 m_localCenterB;
	b2Float m_invMassA;
	b2Float m_invMassB;
	b2Float m_invIA;
	b2Float m_invIB;
	b2Vec2 m_axis, m_perp;
	b2Float m_s1, m_s2;
	b2Float m_a1, m_a2;
	b2Mat33 m_K;
	b2Float m_motorMass;
};

inline b2Float b2PrismaticJoint::GetMotorSpeed() const noexcept
{
	return m_motorSpeed;
}

} // namespace box2d

#endif
