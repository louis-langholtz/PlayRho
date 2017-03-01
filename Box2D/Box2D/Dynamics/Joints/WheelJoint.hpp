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

#ifndef B2_WHEEL_JOINT_H
#define B2_WHEEL_JOINT_H

#include <Box2D/Dynamics/Joints/Joint.hpp>

namespace box2d {

/// Wheel joint definition. This requires defining a line of
/// motion using an axis and an anchor point. The definition uses local
/// anchor points and a local axis so that the initial configuration
/// can violate the constraint slightly. The joint translation is zero
/// when the local anchor points coincide in world space. Using local
/// anchors and a local axis helps when saving and loading a game.
struct WheelJointDef : public JointDef
{
	constexpr WheelJointDef() noexcept: JointDef(JointType::Wheel) {}

	/// Initialize the bodies, anchors, axis, and reference angle using the world
	/// anchor and world axis.
	void Initialize(Body* bodyA, Body* bodyB, const Vec2 anchor, const Vec2 axis);

	/// The local anchor point relative to bodyA's origin.
	Vec2 localAnchorA = Vec2_zero;

	/// The local anchor point relative to bodyB's origin.
	Vec2 localAnchorB = Vec2_zero;

	/// The local translation axis in bodyA.
	Vec2 localAxisA = Vec2{1, 0};

	/// Enable/disable the joint motor.
	bool enableMotor = false;

	/// The maximum motor torque, usually in N-m.
	RealNum maxMotorTorque = 0;

	/// The desired motor speed in radians per second.
	Angle motorSpeed = 0_rad;

	/// Suspension frequency, zero indicates no suspension
	RealNum frequencyHz = 2;

	/// Suspension damping ratio, one indicates critical damping
	RealNum dampingRatio = 0.7f;
};

/// A wheel joint. This joint provides two degrees of freedom: translation
/// along an axis fixed in bodyA and rotation in the plane. In other words, it is a point to
/// line constraint with a rotational motor and a linear spring/damper.
/// This joint is designed for vehicle suspensions.
class WheelJoint : public Joint
{
public:
	WheelJoint(const WheelJointDef& def);
	
	Vec2 GetAnchorA() const override;
	Vec2 GetAnchorB() const override;

	Vec2 GetReactionForce(RealNum inv_dt) const override;
	RealNum GetReactionTorque(RealNum inv_dt) const override;

	/// The local anchor point relative to bodyA's origin.
	Vec2 GetLocalAnchorA() const { return m_localAnchorA; }

	/// The local anchor point relative to bodyB's origin.
	Vec2 GetLocalAnchorB() const  { return m_localAnchorB; }

	/// The local joint axis relative to bodyA.
	Vec2 GetLocalAxisA() const { return m_localXAxisA; }

	/// Get the current joint translation, usually in meters.
	RealNum GetJointTranslation() const;

	/// Get the current joint translation speed, in Angle per second.
	Angle GetJointSpeed() const;

	/// Is the joint motor enabled?
	bool IsMotorEnabled() const noexcept { return m_enableMotor; }

	/// Enable/disable the joint motor.
	void EnableMotor(bool flag);

	/// Set the motor speed, usually in radians per second.
	void SetMotorSpeed(Angle speed);

	/// Get the motor speed, usually in radians per second.
	Angle GetMotorSpeed() const;

	/// Set/Get the maximum motor force, usually in N-m.
	void SetMaxMotorTorque(RealNum torque);
	RealNum GetMaxMotorTorque() const;

	/// Get the current motor torque given the inverse time step, usually in N-m.
	RealNum GetMotorTorque(RealNum inv_dt) const;

	/// Set/Get the spring frequency in hertz. Setting the frequency to zero disables the spring.
	void SetSpringFrequencyHz(RealNum hz);
	RealNum GetSpringFrequencyHz() const;

	/// Set/Get the spring damping ratio
	void SetSpringDampingRatio(RealNum ratio);
	RealNum GetSpringDampingRatio() const;

private:

	void InitVelocityConstraints(Span<BodyConstraint> bodies, const StepConf& step, const ConstraintSolverConf& conf) override;
	RealNum SolveVelocityConstraints(Span<BodyConstraint> bodies, const StepConf& step) override;
	bool SolvePositionConstraints(Span<BodyConstraint> bodies, const ConstraintSolverConf& conf) const override;

	RealNum m_frequencyHz;
	RealNum m_dampingRatio;

	// Solver shared
	Vec2 m_localAnchorA;
	Vec2 m_localAnchorB;
	Vec2 m_localXAxisA;
	Vec2 m_localYAxisA;

	RealNum m_impulse = 0;
	RealNum m_motorImpulse = 0;
	RealNum m_springImpulse = 0;

	RealNum m_maxMotorTorque;
	Angle m_motorSpeed;
	bool m_enableMotor;

	// Solver temp
	index_t m_indexA;
	index_t m_indexB;
	Vec2 m_localCenterA;
	Vec2 m_localCenterB;
	
	RealNum m_invMassA;
	RealNum m_invMassB;
	RealNum m_invIA;
	RealNum m_invIB;

	Vec2 m_ax = Vec2_zero;
	Vec2 m_ay = Vec2_zero;

	RealNum m_sAx, m_sBx;
	RealNum m_sAy, m_sBy;

	RealNum m_mass = 0;
	RealNum m_motorMass = 0;
	RealNum m_springMass = 0;

	RealNum m_bias = 0;
	RealNum m_gamma = 0;
};

inline Angle WheelJoint::GetMotorSpeed() const
{
	return m_motorSpeed;
}

inline RealNum WheelJoint::GetMaxMotorTorque() const
{
	return m_maxMotorTorque;
}

inline void WheelJoint::SetSpringFrequencyHz(RealNum hz)
{
	m_frequencyHz = hz;
}

inline RealNum WheelJoint::GetSpringFrequencyHz() const
{
	return m_frequencyHz;
}

inline void WheelJoint::SetSpringDampingRatio(RealNum ratio)
{
	m_dampingRatio = ratio;
}

inline RealNum WheelJoint::GetSpringDampingRatio() const
{
	return m_dampingRatio;
}

} // namespace box2d

#endif
