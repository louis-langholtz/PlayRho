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

#ifndef B2_PULLEY_JOINT_H
#define B2_PULLEY_JOINT_H

#include <Box2D/Dynamics/Joints/Joint.hpp>

namespace box2d {

/// Pulley joint definition. This requires two ground anchors,
/// two dynamic body anchor points, and a pulley ratio.
struct PulleyJointDef : public JointDef
{
	PulleyJointDef() noexcept: JointDef(JointType::Pulley)
	{
		collideConnected = true;
	}

	/// Initialize the bodies, anchors, lengths, max lengths, and ratio using the world anchors.
	void Initialize(Body* bodyA, Body* bodyB,
					const Vec2 groundAnchorA, const Vec2 groundAnchorB,
					const Vec2 anchorA, const Vec2 anchorB,
					RealNum ratio);

	/// The first ground anchor in world coordinates. This point never moves.
	Vec2 groundAnchorA = Vec2{-RealNum{1}, RealNum{1}};

	/// The second ground anchor in world coordinates. This point never moves.
	Vec2 groundAnchorB = Vec2{RealNum{1}, RealNum{1}};

	/// The local anchor point relative to bodyA's origin.
	Vec2 localAnchorA = Vec2{-RealNum{1}, RealNum{0}};

	/// The local anchor point relative to bodyB's origin.
	Vec2 localAnchorB = Vec2{RealNum{1}, RealNum{0}};

	/// The a reference length for the segment attached to bodyA.
	RealNum lengthA = RealNum{0};

	/// The a reference length for the segment attached to bodyB.
	RealNum lengthB = RealNum{0};

	/// The pulley ratio, used to simulate a block-and-tackle.
	RealNum ratio = RealNum{1};
};

/// The pulley joint is connected to two bodies and two fixed ground points.
/// The pulley supports a ratio such that:
/// length1 + ratio * length2 <= constant
/// Yes, the force transmitted is scaled by the ratio.
/// Warning: the pulley joint can get a bit squirrelly by itself. They often
/// work better when combined with prismatic joints. You should also cover the
/// the anchor points with static shapes to prevent one side from going to
/// zero length.
class PulleyJoint : public Joint
{
public:
	PulleyJoint(const PulleyJointDef& data);

	Vec2 GetLocalAnchorA() const noexcept;
	Vec2 GetLocalAnchorB() const noexcept;

	Vec2 GetAnchorA() const override;
	Vec2 GetAnchorB() const override;

	Vec2 GetReactionForce(RealNum inv_dt) const override;
	RealNum GetReactionTorque(RealNum inv_dt) const override;

	/// Get the first ground anchor.
	Vec2 GetGroundAnchorA() const noexcept;

	/// Get the second ground anchor.
	Vec2 GetGroundAnchorB() const noexcept;

	/// Get the current length of the segment attached to bodyA.
	RealNum GetLengthA() const noexcept;

	/// Get the current length of the segment attached to bodyB.
	RealNum GetLengthB() const noexcept;

	/// Get the pulley ratio.
	RealNum GetRatio() const noexcept;

	/// Implement Joint::ShiftOrigin
	void ShiftOrigin(const Vec2 newOrigin) override;

private:

	void InitVelocityConstraints(Span<Velocity> velocities, Span<const Position> positions, const StepConf& step, const ConstraintSolverConf& conf) override;
	void SolveVelocityConstraints(Span<Velocity> velocities, const StepConf& step) override;
	bool SolvePositionConstraints(Span<Position> positions, const ConstraintSolverConf& conf) const override;

	Vec2 m_groundAnchorA;
	Vec2 m_groundAnchorB;
	RealNum m_lengthA;
	RealNum m_lengthB;
	
	// Solver shared
	Vec2 m_localAnchorA;
	Vec2 m_localAnchorB;
	RealNum m_constant;
	RealNum m_ratio;
	RealNum m_impulse = 0;

	// Solver temp
	index_t m_indexA;
	index_t m_indexB;
	Vec2 m_uA;
	Vec2 m_uB;
	Vec2 m_rA;
	Vec2 m_rB;
	Vec2 m_localCenterA;
	Vec2 m_localCenterB;
	RealNum m_invMassA;
	RealNum m_invMassB;
	RealNum m_invIA;
	RealNum m_invIB;
	RealNum m_mass;
};
	
inline Vec2 PulleyJoint::GetLocalAnchorA() const noexcept
{
	return m_localAnchorA;
}

inline Vec2 PulleyJoint::GetLocalAnchorB() const noexcept
{
	return m_localAnchorB;
}

inline Vec2 PulleyJoint::GetGroundAnchorA() const noexcept
{
	return m_groundAnchorA;
}

inline Vec2 PulleyJoint::GetGroundAnchorB() const noexcept
{
	return m_groundAnchorB;
}
	
inline RealNum PulleyJoint::GetLengthA() const noexcept
{
	return m_lengthA;
}

inline RealNum PulleyJoint::GetLengthB() const noexcept
{
	return m_lengthB;
}

inline RealNum PulleyJoint::GetRatio() const noexcept
{
	return m_ratio;
}

/// Get the current length of the segment attached to bodyA.
RealNum GetCurrentLengthA(const PulleyJoint& joint);

/// Get the current length of the segment attached to bodyB.
RealNum GetCurrentLengthB(const PulleyJoint& joint);

} // namespace box2d

#endif
