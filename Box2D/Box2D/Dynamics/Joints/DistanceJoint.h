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

#include <Box2D/Dynamics/Joints/Joint.h>

namespace box2d {

/// Distance joint definition. This requires defining an
/// anchor point on both bodies and the non-zero length of the
/// distance joint. The definition uses local anchor points
/// so that the initial configuration can violate the constraint
/// slightly. This helps when saving and loading a game.
/// @warning Do not use a zero or short length.
struct DistanceJointDef : public JointDef
{
	constexpr DistanceJointDef() noexcept: JointDef(JointType::Distance) {}

	/// Initialize the bodies, anchors, and length using the world
	/// anchors.
	void Initialize(Body* bodyA, Body* bodyB,
					const Vec2& anchorA, const Vec2& anchorB);

	/// The local anchor point relative to bodyA's origin.
	Vec2 localAnchorA = Vec2_zero;

	/// The local anchor point relative to bodyB's origin.
	Vec2 localAnchorB = Vec2_zero;

	/// The natural length between the anchor points.
	float_t length = float_t{1};

	/// The mass-spring-damper frequency in Hertz. A value of 0
	/// disables softness.
	float_t frequencyHz = float_t{0};

	/// The damping ratio. 0 = no damping, 1 = critical damping.
	float_t dampingRatio = float_t{0};
};

/// A distance joint constrains two points on two bodies
/// to remain at a fixed distance from each other. You can view
/// this as a massless, rigid rod.
class DistanceJoint : public Joint
{
public:

	Vec2 GetAnchorA() const override;
	Vec2 GetAnchorB() const override;

	/// Get the reaction force given the inverse time step.
	/// Unit is N.
	Vec2 GetReactionForce(float_t inv_dt) const override;

	/// Get the reaction torque given the inverse time step.
	/// Unit is N*m. This is always zero for a distance joint.
	float_t GetReactionTorque(float_t inv_dt) const override;

	/// The local anchor point relative to bodyA's origin.
	const Vec2& GetLocalAnchorA() const noexcept { return m_localAnchorA; }

	/// The local anchor point relative to bodyB's origin.
	const Vec2& GetLocalAnchorB() const noexcept { return m_localAnchorB; }

	/// Set/get the natural length.
	/// Manipulating the length can lead to non-physical behavior when the frequency is zero.
	void SetLength(float_t length);
	float_t GetLength() const noexcept;

	/// Set/get frequency in Hz.
	void SetFrequency(float_t hz);
	float_t GetFrequency() const noexcept;

	/// Set/get damping ratio.
	void SetDampingRatio(float_t ratio);
	float_t GetDampingRatio() const noexcept;

	/// Dump joint to dmLog
	void Dump() override;

protected:

	friend class Joint;
	DistanceJoint(const DistanceJointDef& data);

	void InitVelocityConstraints(const SolverData& data) override;
	void SolveVelocityConstraints(const SolverData& data) override;
	bool SolvePositionConstraints(const SolverData& data) override;

	float_t m_frequencyHz;
	float_t m_dampingRatio;
	float_t m_bias = float_t{0};

	// Solver shared
	Vec2 m_localAnchorA;
	Vec2 m_localAnchorB;
	float_t m_gamma = float_t{0};
	float_t m_impulse = float_t{0};
	float_t m_length;

	// Solver temp
	index_t m_indexA;
	index_t m_indexB;
	Vec2 m_u;
	Vec2 m_rA;
	Vec2 m_rB;
	Vec2 m_localCenterA;
	Vec2 m_localCenterB;
	float_t m_invMassA;
	float_t m_invMassB;
	float_t m_invIA;
	float_t m_invIB;
	float_t m_mass;
};

inline void DistanceJoint::SetLength(float_t length)
{
	m_length = length;
}

inline float_t DistanceJoint::GetLength() const noexcept
{
	return m_length;
}

inline void DistanceJoint::SetFrequency(float_t hz)
{
	m_frequencyHz = hz;
}

inline float_t DistanceJoint::GetFrequency() const noexcept
{
	return m_frequencyHz;
}

inline void DistanceJoint::SetDampingRatio(float_t ratio)
{
	m_dampingRatio = ratio;
}

inline float_t DistanceJoint::GetDampingRatio() const noexcept
{
	return m_dampingRatio;
}

} // namespace box2d

#endif
