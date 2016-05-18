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

#ifndef B2_MOUSE_JOINT_H
#define B2_MOUSE_JOINT_H

#include <Box2D/Dynamics/Joints/b2Joint.h>

namespace box2d {

/// Mouse joint definition. This requires a world target point,
/// tuning parameters, and the time step.
struct b2MouseJointDef : public b2JointDef
{
	constexpr b2MouseJointDef() noexcept: b2JointDef(e_mouseJoint) {}

	/// The initial world target point. This is assumed
	/// to coincide with the body anchor initially.
	b2Vec2 target = b2Vec2_zero;

	/// The maximum constraint force that can be exerted
	/// to move the candidate body. Usually you will express
	/// as some multiple of the weight (multiplier * mass * gravity).
	b2Float maxForce = b2Float{0};

	/// The response speed.
	b2Float frequencyHz = b2Float(5);

	/// The damping ratio. 0 = no damping, 1 = critical damping.
	b2Float dampingRatio = b2Float(0.7);
};

/// A mouse joint is used to make a point on a body track a
/// specified world point. This a soft constraint with a maximum
/// force. This allows the constraint to stretch and without
/// applying huge forces.
/// NOTE: this joint is not documented in the manual because it was
/// developed to be used in the testbed. If you want to learn how to
/// use the mouse joint, look at the testbed.
class b2MouseJoint : public b2Joint
{
public:

	/// Implements b2Joint.
	b2Vec2 GetAnchorA() const override;

	/// Implements b2Joint.
	b2Vec2 GetAnchorB() const override;

	/// Implements b2Joint.
	b2Vec2 GetReactionForce(b2Float inv_dt) const override;

	/// Implements b2Joint.
	b2Float GetReactionTorque(b2Float inv_dt) const override;

	/// Use this to update the target point.
	void SetTarget(const b2Vec2& target);
	const b2Vec2& GetTarget() const;

	/// Set/get the maximum force in Newtons.
	void SetMaxForce(b2Float force);
	b2Float GetMaxForce() const;

	/// Set/get the frequency in Hertz.
	void SetFrequency(b2Float hz);
	b2Float GetFrequency() const;

	/// Set/get the damping ratio (dimensionless).
	void SetDampingRatio(b2Float ratio);
	b2Float GetDampingRatio() const;

	/// The mouse joint does not support dumping.
	void Dump() override;

	/// Implement b2Joint::ShiftOrigin
	void ShiftOrigin(const b2Vec2& newOrigin) override;

protected:
	friend class b2Joint;

	b2MouseJoint(const b2MouseJointDef* def);

	void InitVelocityConstraints(const b2SolverData& data) override;
	void SolveVelocityConstraints(const b2SolverData& data) override;
	bool SolvePositionConstraints(const b2SolverData& data) override;

	b2Vec2 m_localAnchorB;
	b2Vec2 m_targetA;
	b2Float m_frequencyHz;
	b2Float m_dampingRatio;
	b2Float m_beta = b2Float{0};
	
	// Solver shared
	b2Vec2 m_impulse = b2Vec2_zero;
	b2Float m_maxForce;
	b2Float m_gamma = b2Float{0};

	// Solver temp
	index_t m_indexA;
	index_t m_indexB;
	b2Vec2 m_rB;
	b2Vec2 m_localCenterB;
	b2Float m_invMassB;
	b2Float m_invIB;
	b2Mat22 m_mass;
	b2Vec2 m_C;
};

} // namespace box2d

#endif
