/*
* Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
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

#ifndef B2_FRICTION_JOINT_H
#define B2_FRICTION_JOINT_H

#include <Box2D/Dynamics/Joints/b2Joint.h>

namespace box2d {

/// Friction joint definition.
struct b2FrictionJointDef : public b2JointDef
{
	constexpr b2FrictionJointDef() noexcept: b2JointDef(e_frictionJoint) {}

	/// Initialize the bodies, anchors, axis, and reference angle using the world
	/// anchor and world axis.
	void Initialize(b2Body* bodyA, b2Body* bodyB, const b2Vec2& anchor);

	/// The local anchor point relative to bodyA's origin.
	b2Vec2 localAnchorA = b2Vec2_zero;

	/// The local anchor point relative to bodyB's origin.
	b2Vec2 localAnchorB = b2Vec2_zero;

	/// The maximum friction force in N.
	float_t maxForce = float_t{0};

	/// The maximum friction torque in N-m.
	float_t maxTorque = float_t{0};
};

/// Friction joint. This is used for top-down friction.
/// It provides 2D translational friction and angular friction.
class b2FrictionJoint : public b2Joint
{
public:
	b2Vec2 GetAnchorA() const override;
	b2Vec2 GetAnchorB() const override;

	b2Vec2 GetReactionForce(float_t inv_dt) const override;
	float_t GetReactionTorque(float_t inv_dt) const override;

	/// The local anchor point relative to bodyA's origin.
	const b2Vec2& GetLocalAnchorA() const { return m_localAnchorA; }

	/// The local anchor point relative to bodyB's origin.
	const b2Vec2& GetLocalAnchorB() const  { return m_localAnchorB; }

	/// Set the maximum friction force in N.
	void SetMaxForce(float_t force);

	/// Get the maximum friction force in N.
	float_t GetMaxForce() const;

	/// Set the maximum friction torque in N*m.
	void SetMaxTorque(float_t torque);

	/// Get the maximum friction torque in N*m.
	float_t GetMaxTorque() const;

	/// Dump joint to dmLog
	void Dump() override;

protected:

	friend class b2Joint;

	b2FrictionJoint(const b2FrictionJointDef* def);

	void InitVelocityConstraints(const b2SolverData& data) override;
	void SolveVelocityConstraints(const b2SolverData& data) override;
	bool SolvePositionConstraints(const b2SolverData& data) override;

	b2Vec2 m_localAnchorA;
	b2Vec2 m_localAnchorB;

	// Solver shared
	b2Vec2 m_linearImpulse = b2Vec2_zero;
	float_t m_angularImpulse = float_t{0};
	float_t m_maxForce;
	float_t m_maxTorque;

	// Solver temp
	index_t m_indexA;
	index_t m_indexB;
	b2Vec2 m_rA;
	b2Vec2 m_rB;
	b2Vec2 m_localCenterA;
	b2Vec2 m_localCenterB;
	float_t m_invMassA;
	float_t m_invMassB;
	float_t m_invIA;
	float_t m_invIB;
	b2Mat22 m_linearMass;
	float_t m_angularMass;
};

} // namespace box2d

#endif
