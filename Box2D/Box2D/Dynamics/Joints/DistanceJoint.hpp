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

#ifndef B2_DISTANCE_JOINT_H
#define B2_DISTANCE_JOINT_H

#include <Box2D/Dynamics/Joints/Joint.hpp>

namespace box2d {

/// Distance joint definition. This requires defining an
/// anchor point on both bodies and the non-zero length of the
/// distance joint. The definition uses local anchor points
/// so that the initial configuration can violate the constraint
/// slightly. This helps when saving and loading a game.
/// @warning Do not use a zero or short length.
struct DistanceJointDef : public JointDef
{
	constexpr DistanceJointDef() noexcept: JointDef{JointType::Distance} {}
	DistanceJointDef(const DistanceJointDef& copy) = default;

	/// Initialize the bodies, anchors, and length using the world anchors.
	DistanceJointDef(Body* bodyA, Body* bodyB,
					 const Vec2 anchorA = Vec2_zero, const Vec2 anchorB = Vec2_zero,
					 realnum freq = 0, realnum damp = 0) noexcept;

	/// The local anchor point relative to bodyA's origin.
	Vec2 localAnchorA = Vec2_zero;

	/// The local anchor point relative to bodyB's origin.
	Vec2 localAnchorB = Vec2_zero;

	/// The natural length between the anchor points.
	realnum length = 1;

	/// The mass-spring-damper frequency in Hertz. A value of 0
	/// disables softness.
	realnum frequencyHz = 0;

	/// The damping ratio. 0 = no damping, 1 = critical damping.
	realnum dampingRatio = 0;
};

/// A distance joint constrains two points on two bodies
/// to remain at a fixed distance from each other. You can view
/// this as a massless, rigid rod.
class DistanceJoint : public Joint
{
public:
	DistanceJoint(const DistanceJointDef& data);

	Vec2 GetAnchorA() const override;
	Vec2 GetAnchorB() const override;

	/// Get the reaction force given the inverse time step.
	/// Unit is N.
	Vec2 GetReactionForce(realnum inv_dt) const override;

	/// Get the reaction torque given the inverse time step.
	/// Unit is N*m. This is always zero for a distance joint.
	realnum GetReactionTorque(realnum inv_dt) const override;

	/// The local anchor point relative to bodyA's origin.
	Vec2 GetLocalAnchorA() const noexcept { return m_localAnchorA; }

	/// The local anchor point relative to bodyB's origin.
	Vec2 GetLocalAnchorB() const noexcept { return m_localAnchorB; }

	/// Set/get the natural length.
	/// Manipulating the length can lead to non-physical behavior when the frequency is zero.
	void SetLength(realnum length);
	realnum GetLength() const noexcept;

	/// Set/get frequency in Hz.
	void SetFrequency(realnum hz);
	realnum GetFrequency() const noexcept;

	/// Set/get damping ratio.
	void SetDampingRatio(realnum ratio);
	realnum GetDampingRatio() const noexcept;

private:

	void InitVelocityConstraints(Span<Velocity> velocities, Span<const Position> positions, const StepConf& step, const ConstraintSolverConf& conf) override;
	void SolveVelocityConstraints(Span<Velocity> velocities, const StepConf& step) override;
	bool SolvePositionConstraints(Span<Position> positions, const ConstraintSolverConf& conf) override;

	realnum m_frequencyHz;
	realnum m_dampingRatio;
	realnum m_bias = 0;

	// Solver shared
	Vec2 m_localAnchorA;
	Vec2 m_localAnchorB;
	realnum m_gamma = 0;
	realnum m_impulse = 0;
	realnum m_length;

	// Solver temp
	index_t m_indexA;
	index_t m_indexB;
	Vec2 m_u;
	Vec2 m_rA;
	Vec2 m_rB;
	Vec2 m_localCenterA;
	Vec2 m_localCenterB;
	realnum m_invMassA;
	realnum m_invMassB;
	realnum m_invIA;
	realnum m_invIB;
	realnum m_mass;
};

inline void DistanceJoint::SetLength(realnum length)
{
	m_length = length;
}

inline realnum DistanceJoint::GetLength() const noexcept
{
	return m_length;
}

inline void DistanceJoint::SetFrequency(realnum hz)
{
	m_frequencyHz = hz;
}

inline realnum DistanceJoint::GetFrequency() const noexcept
{
	return m_frequencyHz;
}

inline void DistanceJoint::SetDampingRatio(realnum ratio)
{
	m_dampingRatio = ratio;
}

inline realnum DistanceJoint::GetDampingRatio() const noexcept
{
	return m_dampingRatio;
}
	
} // namespace box2d

#endif
