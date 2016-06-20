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

#ifndef B2_WHEEL_JOINT_H
#define B2_WHEEL_JOINT_H

#include <Box2D/Dynamics/Joints/Joint.h>

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
	void Initialize(Body* bodyA, Body* bodyB, const Vec2& anchor, const Vec2& axis);

	/// The local anchor point relative to bodyA's origin.
	Vec2 localAnchorA = Vec2_zero;

	/// The local anchor point relative to bodyB's origin.
	Vec2 localAnchorB = Vec2_zero;

	/// The local translation axis in bodyA.
	Vec2 localAxisA = Vec2{float_t{1}, float_t{0}};

	/// Enable/disable the joint motor.
	bool enableMotor = false;

	/// The maximum motor torque, usually in N-m.
	float_t maxMotorTorque = float_t{0};

	/// The desired motor speed in radians per second.
	float_t motorSpeed = float_t{0};

	/// Suspension frequency, zero indicates no suspension
	float_t frequencyHz = float_t(2);

	/// Suspension damping ratio, one indicates critical damping
	float_t dampingRatio = float_t(0.7);
};

/// A wheel joint. This joint provides two degrees of freedom: translation
/// along an axis fixed in bodyA and rotation in the plane. In other words, it is a point to
/// line constraint with a rotational motor and a linear spring/damper.
/// This joint is designed for vehicle suspensions.
class WheelJoint : public Joint
{
public:
	Vec2 GetAnchorA() const override;
	Vec2 GetAnchorB() const override;

	Vec2 GetReactionForce(float_t inv_dt) const override;
	float_t GetReactionTorque(float_t inv_dt) const override;

	/// The local anchor point relative to bodyA's origin.
	const Vec2& GetLocalAnchorA() const { return m_localAnchorA; }

	/// The local anchor point relative to bodyB's origin.
	const Vec2& GetLocalAnchorB() const  { return m_localAnchorB; }

	/// The local joint axis relative to bodyA.
	const Vec2& GetLocalAxisA() const { return m_localXAxisA; }

	/// Get the current joint translation, usually in meters.
	float_t GetJointTranslation() const;

	/// Get the current joint translation speed, usually in meters per second.
	float_t GetJointSpeed() const;

	/// Is the joint motor enabled?
	bool IsMotorEnabled() const;

	/// Enable/disable the joint motor.
	void EnableMotor(bool flag);

	/// Set the motor speed, usually in radians per second.
	void SetMotorSpeed(float_t speed);

	/// Get the motor speed, usually in radians per second.
	float_t GetMotorSpeed() const;

	/// Set/Get the maximum motor force, usually in N-m.
	void SetMaxMotorTorque(float_t torque);
	float_t GetMaxMotorTorque() const;

	/// Get the current motor torque given the inverse time step, usually in N-m.
	float_t GetMotorTorque(float_t inv_dt) const;

	/// Set/Get the spring frequency in hertz. Setting the frequency to zero disables the spring.
	void SetSpringFrequencyHz(float_t hz);
	float_t GetSpringFrequencyHz() const;

	/// Set/Get the spring damping ratio
	void SetSpringDampingRatio(float_t ratio);
	float_t GetSpringDampingRatio() const;

	/// Dump to log
	void Dump() override;

protected:

	friend class Joint;
	WheelJoint(const WheelJointDef& def);

	void InitVelocityConstraints(const SolverData& data) override;
	void SolveVelocityConstraints(const SolverData& data) override;
	bool SolvePositionConstraints(const SolverData& data) override;

	float_t m_frequencyHz;
	float_t m_dampingRatio;

	// Solver shared
	Vec2 m_localAnchorA;
	Vec2 m_localAnchorB;
	Vec2 m_localXAxisA;
	Vec2 m_localYAxisA;

	float_t m_impulse;
	float_t m_motorImpulse;
	float_t m_springImpulse;

	float_t m_maxMotorTorque;
	float_t m_motorSpeed;
	bool m_enableMotor;

	// Solver temp
	index_t m_indexA;
	index_t m_indexB;
	Vec2 m_localCenterA;
	Vec2 m_localCenterB;
	float_t m_invMassA;
	float_t m_invMassB;
	float_t m_invIA;
	float_t m_invIB;

	Vec2 m_ax, m_ay;
	float_t m_sAx, m_sBx;
	float_t m_sAy, m_sBy;

	float_t m_mass;
	float_t m_motorMass;
	float_t m_springMass;

	float_t m_bias;
	float_t m_gamma;
};

inline float_t WheelJoint::GetMotorSpeed() const
{
	return m_motorSpeed;
}

inline float_t WheelJoint::GetMaxMotorTorque() const
{
	return m_maxMotorTorque;
}

inline void WheelJoint::SetSpringFrequencyHz(float_t hz)
{
	m_frequencyHz = hz;
}

inline float_t WheelJoint::GetSpringFrequencyHz() const
{
	return m_frequencyHz;
}

inline void WheelJoint::SetSpringDampingRatio(float_t ratio)
{
	m_dampingRatio = ratio;
}

inline float_t WheelJoint::GetSpringDampingRatio() const
{
	return m_dampingRatio;
}

} // namespace box2d

#endif
