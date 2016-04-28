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

#ifndef B2_REVOLUTE_JOINT_H
#define B2_REVOLUTE_JOINT_H

#include <Box2D/Dynamics/Joints/b2Joint.h>

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
struct b2RevoluteJointDef : public b2JointDef
{
	constexpr b2RevoluteJointDef() noexcept: b2JointDef(e_revoluteJoint) {}

	/// Initialize the bodies, anchors, and reference angle using a world
	/// anchor point.
	void Initialize(b2Body* bodyA, b2Body* bodyB, const b2Vec2& anchor);

	/// The local anchor point relative to bodyA's origin.
	b2Vec2 localAnchorA = b2Vec2_zero;

	/// The local anchor point relative to bodyB's origin.
	b2Vec2 localAnchorB = b2Vec2_zero;

	/// The bodyB angle minus bodyA angle in the reference state (radians).
	b2Float referenceAngle = b2Float{0};

	/// A flag to enable joint limits.
	bool enableLimit = false;

	/// The lower angle for the joint limit (radians).
	b2Float lowerAngle = b2Float{0};

	/// The upper angle for the joint limit (radians).
	b2Float upperAngle = b2Float{0};

	/// A flag to enable the joint motor.
	bool enableMotor = false;

	/// The desired motor speed. Usually in radians per second.
	b2Float motorSpeed = b2Float{0};

	/// The maximum motor torque used to achieve the desired motor speed.
	/// Usually in N-m.
	b2Float maxMotorTorque = b2Float{0};
};

/// A revolute joint constrains two bodies to share a common point while they
/// are free to rotate about the point. The relative rotation about the shared
/// point is the joint angle. You can limit the relative rotation with
/// a joint limit that specifies a lower and upper angle. You can use a motor
/// to drive the relative rotation about the shared point. A maximum motor torque
/// is provided so that infinite forces are not generated.
class b2RevoluteJoint : public b2Joint
{
public:
	b2Vec2 GetAnchorA() const override;
	b2Vec2 GetAnchorB() const override;

	/// The local anchor point relative to bodyA's origin.
	const b2Vec2& GetLocalAnchorA() const noexcept { return m_localAnchorA; }

	/// The local anchor point relative to bodyB's origin.
	const b2Vec2& GetLocalAnchorB() const noexcept { return m_localAnchorB; }

	/// Get the reference angle.
	b2Float GetReferenceAngle() const noexcept { return m_referenceAngle; }

	/// Get the current joint angle in radians.
	b2Float GetJointAngle() const;

	/// Get the current joint angle speed in radians per second.
	b2Float GetJointSpeed() const;

	/// Is the joint limit enabled?
	bool IsLimitEnabled() const;

	/// Enable/disable the joint limit.
	void EnableLimit(bool flag);

	/// Get the lower joint limit in radians.
	b2Float GetLowerLimit() const;

	/// Get the upper joint limit in radians.
	b2Float GetUpperLimit() const;

	/// Set the joint limits in radians.
	void SetLimits(b2Float lower, b2Float upper);

	/// Is the joint motor enabled?
	bool IsMotorEnabled() const;

	/// Enable/disable the joint motor.
	void EnableMotor(bool flag);

	/// Set the motor speed in radians per second.
	void SetMotorSpeed(b2Float speed);

	/// Get the motor speed in radians per second.
	b2Float GetMotorSpeed() const;

	/// Set the maximum motor torque, usually in N-m.
	void SetMaxMotorTorque(b2Float torque);
	b2Float GetMaxMotorTorque() const noexcept { return m_maxMotorTorque; }

	/// Get the reaction force given the inverse time step.
	/// Unit is N.
	b2Vec2 GetReactionForce(b2Float inv_dt) const override;

	/// Get the reaction torque due to the joint limit given the inverse time step.
	/// Unit is N*m.
	b2Float GetReactionTorque(b2Float inv_dt) const override;

	/// Get the current motor torque given the inverse time step.
	/// Unit is N*m.
	b2Float GetMotorTorque(b2Float inv_dt) const;

	/// Dump to b2Log.
	void Dump() override;

protected:
	
	friend class b2Joint;
	friend class b2GearJoint;

	b2RevoluteJoint(const b2RevoluteJointDef* def);

	void InitVelocityConstraints(const b2SolverData& data) override;
	void SolveVelocityConstraints(const b2SolverData& data) override;
	bool SolvePositionConstraints(const b2SolverData& data) override;

	// Solver shared
	b2Vec2 m_localAnchorA;
	b2Vec2 m_localAnchorB;
	b2Vec3 m_impulse;
	b2Float m_motorImpulse;

	bool m_enableMotor;
	b2Float m_maxMotorTorque;
	b2Float m_motorSpeed;

	bool m_enableLimit;
	b2Float m_referenceAngle;
	b2Float m_lowerAngle;
	b2Float m_upperAngle;

	// Solver temp
	index_t m_indexA;
	index_t m_indexB;
	b2Vec2 m_rA;
	b2Vec2 m_rB;
	b2Vec2 m_localCenterA;
	b2Vec2 m_localCenterB;
	b2Float m_invMassA;
	b2Float m_invMassB;
	b2Float m_invIA;
	b2Float m_invIB;
	b2Mat33 m_mass;			// effective mass for point-to-point constraint.
	b2Float m_motorMass;	// effective mass for motor/limit angular constraint.
	b2LimitState m_limitState;
};

inline b2Float b2RevoluteJoint::GetMotorSpeed() const
{
	return m_motorSpeed;
}

#endif
