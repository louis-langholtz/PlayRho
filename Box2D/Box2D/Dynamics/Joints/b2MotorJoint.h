/*
* Copyright (c) 2006-2012 Erin Catto http://www.box2d.org
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

#include <Box2D/Dynamics/Joints/b2Joint.h>

namespace box2d {

/// Motor joint definition.
struct MotorJointDef : public JointDef
{
	constexpr MotorJointDef() noexcept: JointDef(e_motorJoint) {}

	/// Initialize the bodies and offsets using the current transforms.
	void Initialize(Body* bodyA, Body* bodyB);

	/// Position of bodyB minus the position of bodyA, in bodyA's frame, in meters.
	Vec2 linearOffset = Vec2_zero;

	/// The bodyB angle minus bodyA angle in radians.
	float_t angularOffset = float_t{0};
	
	/// The maximum motor force in N.
	float_t maxForce = float_t(1);

	/// The maximum motor torque in N-m.
	float_t maxTorque = float_t(1);

	/// Position correction factor in the range [0,1].
	float_t correctionFactor = float_t(0.3);
};

/// A motor joint is used to control the relative motion
/// between two bodies. A typical usage is to control the movement
/// of a dynamic body with respect to the ground.
class MotorJoint : public Joint
{
public:
	Vec2 GetAnchorA() const override;
	Vec2 GetAnchorB() const override;

	Vec2 GetReactionForce(float_t inv_dt) const override;
	float_t GetReactionTorque(float_t inv_dt) const override;

	/// Set/get the target linear offset, in frame A, in meters.
	void SetLinearOffset(const Vec2& linearOffset);
	const Vec2& GetLinearOffset() const;

	/// Set/get the target angular offset, in radians.
	void SetAngularOffset(float_t angularOffset);
	float_t GetAngularOffset() const;

	/// Set the maximum friction force in N.
	void SetMaxForce(float_t force);

	/// Get the maximum friction force in N.
	float_t GetMaxForce() const;

	/// Set the maximum friction torque in N*m.
	void SetMaxTorque(float_t torque);

	/// Get the maximum friction torque in N*m.
	float_t GetMaxTorque() const;

	/// Set the position correction factor in the range [0,1].
	void SetCorrectionFactor(float_t factor);

	/// Get the position correction factor in the range [0,1].
	float_t GetCorrectionFactor() const;

	/// Dump to log
	void Dump() override;

protected:

	friend class Joint;

	MotorJoint(const MotorJointDef* def);

	void InitVelocityConstraints(const b2SolverData& data) override;
	void SolveVelocityConstraints(const b2SolverData& data) override;
	bool SolvePositionConstraints(const b2SolverData& data) override;

	// Solver shared
	Vec2 m_linearOffset;
	float_t m_angularOffset;
	Vec2 m_linearImpulse;
	float_t m_angularImpulse;
	float_t m_maxForce;
	float_t m_maxTorque;
	float_t m_correctionFactor;

	// Solver temp
	index_t m_indexA;
	index_t m_indexB;
	Vec2 m_rA;
	Vec2 m_rB;
	Vec2 m_localCenterA;
	Vec2 m_localCenterB;
	Vec2 m_linearError;
	float_t m_angularError;
	float_t m_invMassA;
	float_t m_invMassB;
	float_t m_invIA;
	float_t m_invIB;
	Mat22 m_linearMass;
	float_t m_angularMass;
};

} // namespace box2d

#endif
