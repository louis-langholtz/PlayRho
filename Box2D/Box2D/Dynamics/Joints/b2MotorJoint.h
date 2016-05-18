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
struct b2MotorJointDef : public b2JointDef
{
	constexpr b2MotorJointDef() noexcept: b2JointDef(e_motorJoint) {}

	/// Initialize the bodies and offsets using the current transforms.
	void Initialize(b2Body* bodyA, b2Body* bodyB);

	/// Position of bodyB minus the position of bodyA, in bodyA's frame, in meters.
	b2Vec2 linearOffset = b2Vec2_zero;

	/// The bodyB angle minus bodyA angle in radians.
	b2Float angularOffset = b2Float{0};
	
	/// The maximum motor force in N.
	b2Float maxForce = b2Float(1);

	/// The maximum motor torque in N-m.
	b2Float maxTorque = b2Float(1);

	/// Position correction factor in the range [0,1].
	b2Float correctionFactor = b2Float(0.3);
};

/// A motor joint is used to control the relative motion
/// between two bodies. A typical usage is to control the movement
/// of a dynamic body with respect to the ground.
class b2MotorJoint : public b2Joint
{
public:
	b2Vec2 GetAnchorA() const override;
	b2Vec2 GetAnchorB() const override;

	b2Vec2 GetReactionForce(b2Float inv_dt) const override;
	b2Float GetReactionTorque(b2Float inv_dt) const override;

	/// Set/get the target linear offset, in frame A, in meters.
	void SetLinearOffset(const b2Vec2& linearOffset);
	const b2Vec2& GetLinearOffset() const;

	/// Set/get the target angular offset, in radians.
	void SetAngularOffset(b2Float angularOffset);
	b2Float GetAngularOffset() const;

	/// Set the maximum friction force in N.
	void SetMaxForce(b2Float force);

	/// Get the maximum friction force in N.
	b2Float GetMaxForce() const;

	/// Set the maximum friction torque in N*m.
	void SetMaxTorque(b2Float torque);

	/// Get the maximum friction torque in N*m.
	b2Float GetMaxTorque() const;

	/// Set the position correction factor in the range [0,1].
	void SetCorrectionFactor(b2Float factor);

	/// Get the position correction factor in the range [0,1].
	b2Float GetCorrectionFactor() const;

	/// Dump to b2Log
	void Dump() override;

protected:

	friend class b2Joint;

	b2MotorJoint(const b2MotorJointDef* def);

	void InitVelocityConstraints(const b2SolverData& data) override;
	void SolveVelocityConstraints(const b2SolverData& data) override;
	bool SolvePositionConstraints(const b2SolverData& data) override;

	// Solver shared
	b2Vec2 m_linearOffset;
	b2Float m_angularOffset;
	b2Vec2 m_linearImpulse;
	b2Float m_angularImpulse;
	b2Float m_maxForce;
	b2Float m_maxTorque;
	b2Float m_correctionFactor;

	// Solver temp
	index_t m_indexA;
	index_t m_indexB;
	b2Vec2 m_rA;
	b2Vec2 m_rB;
	b2Vec2 m_localCenterA;
	b2Vec2 m_localCenterB;
	b2Vec2 m_linearError;
	b2Float m_angularError;
	b2Float m_invMassA;
	b2Float m_invMassB;
	b2Float m_invIA;
	b2Float m_invIB;
	b2Mat22 m_linearMass;
	b2Float m_angularMass;
};

} // namespace box2d

#endif
