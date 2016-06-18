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

#include <Box2D/Dynamics/Joints/Joint.h>

namespace box2d {

/// Mouse joint definition. This requires a world target point,
/// tuning parameters, and the time step.
struct MouseJointDef : public JointDef
{
	constexpr MouseJointDef() noexcept: JointDef(JointType::e_mouseJoint) {}

	/// The initial world target point. This is assumed
	/// to coincide with the body anchor initially.
	Vec2 target = Vec2_zero;

	/// The maximum constraint force that can be exerted
	/// to move the candidate body. Usually you will express
	/// as some multiple of the weight (multiplier * mass * gravity).
	float_t maxForce = float_t{0};

	/// The response speed.
	float_t frequencyHz = float_t(5);

	/// The damping ratio. 0 = no damping, 1 = critical damping.
	float_t dampingRatio = float_t(0.7);
};

/// A mouse joint is used to make a point on a body track a
/// specified world point. This a soft constraint with a maximum
/// force. This allows the constraint to stretch and without
/// applying huge forces.
/// NOTE: this joint is not documented in the manual because it was
/// developed to be used in the testbed. If you want to learn how to
/// use the mouse joint, look at the testbed.
class MouseJoint : public Joint
{
public:

	/// Implements Joint.
	Vec2 GetAnchorA() const override;

	/// Implements Joint.
	Vec2 GetAnchorB() const override;

	/// Implements Joint.
	Vec2 GetReactionForce(float_t inv_dt) const override;

	/// Implements Joint.
	float_t GetReactionTorque(float_t inv_dt) const override;

	/// Use this to update the target point.
	void SetTarget(const Vec2& target);
	const Vec2& GetTarget() const;

	/// Set/get the maximum force in Newtons.
	void SetMaxForce(float_t force);
	float_t GetMaxForce() const;

	/// Set/get the frequency in Hertz.
	void SetFrequency(float_t hz);
	float_t GetFrequency() const;

	/// Set/get the damping ratio (dimensionless).
	void SetDampingRatio(float_t ratio);
	float_t GetDampingRatio() const;

	/// The mouse joint does not support dumping.
	void Dump() override;

	/// Implement Joint::ShiftOrigin
	void ShiftOrigin(const Vec2& newOrigin) override;

protected:
	friend class Joint;

	MouseJoint(const MouseJointDef& def);

	void InitVelocityConstraints(const SolverData& data) override;
	void SolveVelocityConstraints(const SolverData& data) override;
	bool SolvePositionConstraints(const SolverData& data) override;

	Vec2 m_localAnchorB;
	Vec2 m_targetA;
	float_t m_frequencyHz;
	float_t m_dampingRatio;
	float_t m_beta = float_t{0};
	
	// Solver shared
	Vec2 m_impulse = Vec2_zero;
	float_t m_maxForce;
	float_t m_gamma = float_t{0};

	// Solver temp
	index_t m_indexA;
	index_t m_indexB;
	Vec2 m_rB;
	Vec2 m_localCenterB;
	float_t m_invMassB;
	float_t m_invIB;
	Mat22 m_mass;
	Vec2 m_C;
};

} // namespace box2d

#endif
