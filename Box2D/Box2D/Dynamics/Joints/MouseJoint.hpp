/*
* Original work Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
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

#ifndef B2_MOUSE_JOINT_H
#define B2_MOUSE_JOINT_H

#include <Box2D/Dynamics/Joints/Joint.hpp>

namespace box2d {

/// Mouse joint definition. This requires a world target point,
/// tuning parameters, and the time step.
struct MouseJointDef : public JointDef
{
	constexpr MouseJointDef() noexcept: JointDef(JointType::Mouse) {}

	/// The initial world target point. This is assumed
	/// to coincide with the body anchor initially.
	Vec2 target = Vec2_zero;

	/// Max force.
	/// @detail
	/// The maximum constraint force that can be exerted
	/// to move the candidate body. Usually you will express
	/// as some multiple of the weight (multiplier * mass * gravity).
	/// @note This may not be negative.
	/// @warning Behavior is undefined if this is a negative value.
	float_t maxForce = float_t{0};

	/// Frequency.
	/// @detail The has to do with the response speed.
	/// @note This value may not be negative.
	/// @warning Behavior is undefined if this is a negative value.
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
	MouseJoint(const MouseJointDef& def);

	/// Implements Joint.
	Vec2 GetAnchorA() const override;

	/// Implements Joint.
	Vec2 GetAnchorB() const override;

	/// Implements Joint.
	Vec2 GetReactionForce(float_t inv_dt) const override;

	/// Implements Joint.
	float_t GetReactionTorque(float_t inv_dt) const override;

	Vec2 GetLocalAnchorB() const noexcept;

	/// Use this to update the target point.
	void SetTarget(const Vec2 target) noexcept;
	Vec2 GetTarget() const noexcept;

	/// Set/get the maximum force in Newtons.
	void SetMaxForce(float_t force) noexcept;
	float_t GetMaxForce() const noexcept;

	/// Set/get the frequency in Hertz.
	void SetFrequency(float_t hz) noexcept;
	float_t GetFrequency() const noexcept;

	/// Set/get the damping ratio (dimensionless).
	void SetDampingRatio(float_t ratio) noexcept;
	float_t GetDampingRatio() const noexcept;

	/// Implement Joint::ShiftOrigin
	void ShiftOrigin(const Vec2 newOrigin) override;

private:
	void InitVelocityConstraints(Span<Velocity> velocities, Span<const Position> positions, const StepConf& step, const ConstraintSolverConf& conf) override;
	void SolveVelocityConstraints(Span<Velocity> velocities, const StepConf& step) override;
	bool SolvePositionConstraints(Span<Position> positions, const ConstraintSolverConf& conf) override;

	Vec2 m_localAnchorB;
	Vec2 m_targetA;
	float_t m_frequencyHz;
	float_t m_dampingRatio;
	
	// Solver shared
	Vec2 m_impulse = Vec2_zero;
	float_t m_maxForce;
	float_t m_gamma = float_t{0};

	// Solver variables. These are only valid after InitVelocityConstraints called.
	index_t m_indexB;
	Vec2 m_rB;
	Vec2 m_localCenterB;
	float_t m_invMassB;
	float_t m_invIB;
	Mat22 m_mass;
	Vec2 m_C;
};

inline Vec2 MouseJoint::GetLocalAnchorB() const noexcept
{
	return m_localAnchorB;
}

inline Vec2 MouseJoint::GetAnchorA() const
{
	return m_targetA;
}

inline Vec2 MouseJoint::GetTarget() const noexcept
{
	return m_targetA;
}

inline void MouseJoint::SetMaxForce(float_t force) noexcept
{
	m_maxForce = force;
}

inline float_t MouseJoint::GetMaxForce() const noexcept
{
	return m_maxForce;
}

inline void MouseJoint::SetFrequency(float_t hz) noexcept
{
	m_frequencyHz = hz;
}

inline float_t MouseJoint::GetFrequency() const noexcept
{
	return m_frequencyHz;
}

inline void MouseJoint::SetDampingRatio(float_t ratio) noexcept
{
	m_dampingRatio = ratio;
}

inline float_t MouseJoint::GetDampingRatio() const noexcept
{
	return m_dampingRatio;
}

} // namespace box2d

#endif
