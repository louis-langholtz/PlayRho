/*
* Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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

#include <Box2D/Dynamics/Joints/b2Joint.h>

namespace box2d {

const auto b2_minPulleyLength = float_t(2);

/// Pulley joint definition. This requires two ground anchors,
/// two dynamic body anchor points, and a pulley ratio.
struct b2PulleyJointDef : public b2JointDef
{
	b2PulleyJointDef() noexcept: b2JointDef(e_pulleyJoint)
	{
		collideConnected = true;
	}

	/// Initialize the bodies, anchors, lengths, max lengths, and ratio using the world anchors.
	void Initialize(b2Body* bodyA, b2Body* bodyB,
					const b2Vec2& groundAnchorA, const b2Vec2& groundAnchorB,
					const b2Vec2& anchorA, const b2Vec2& anchorB,
					float_t ratio);

	/// The first ground anchor in world coordinates. This point never moves.
	b2Vec2 groundAnchorA = b2Vec2{-float_t(1), float_t(1)};

	/// The second ground anchor in world coordinates. This point never moves.
	b2Vec2 groundAnchorB = b2Vec2{float_t(1), float_t(1)};

	/// The local anchor point relative to bodyA's origin.
	b2Vec2 localAnchorA = b2Vec2{-float_t(1), float_t{0}};

	/// The local anchor point relative to bodyB's origin.
	b2Vec2 localAnchorB = b2Vec2{float_t(1), float_t{0}};

	/// The a reference length for the segment attached to bodyA.
	float_t lengthA = float_t{0};

	/// The a reference length for the segment attached to bodyB.
	float_t lengthB = float_t{0};

	/// The pulley ratio, used to simulate a block-and-tackle.
	float_t ratio = float_t(1);
};

/// The pulley joint is connected to two bodies and two fixed ground points.
/// The pulley supports a ratio such that:
/// length1 + ratio * length2 <= constant
/// Yes, the force transmitted is scaled by the ratio.
/// Warning: the pulley joint can get a bit squirrelly by itself. They often
/// work better when combined with prismatic joints. You should also cover the
/// the anchor points with static shapes to prevent one side from going to
/// zero length.
class b2PulleyJoint : public b2Joint
{
public:
	b2Vec2 GetAnchorA() const override;
	b2Vec2 GetAnchorB() const override;

	b2Vec2 GetReactionForce(float_t inv_dt) const override;
	float_t GetReactionTorque(float_t inv_dt) const override;

	/// Get the first ground anchor.
	b2Vec2 GetGroundAnchorA() const;

	/// Get the second ground anchor.
	b2Vec2 GetGroundAnchorB() const;

	/// Get the current length of the segment attached to bodyA.
	float_t GetLengthA() const;

	/// Get the current length of the segment attached to bodyB.
	float_t GetLengthB() const;

	/// Get the pulley ratio.
	float_t GetRatio() const;

	/// Get the current length of the segment attached to bodyA.
	float_t GetCurrentLengthA() const;

	/// Get the current length of the segment attached to bodyB.
	float_t GetCurrentLengthB() const;

	/// Dump joint to dmLog
	void Dump() override;

	/// Implement b2Joint::ShiftOrigin
	void ShiftOrigin(const b2Vec2& newOrigin) override;

protected:

	friend class b2Joint;
	b2PulleyJoint(const b2PulleyJointDef* data);

	void InitVelocityConstraints(const b2SolverData& data) override;
	void SolveVelocityConstraints(const b2SolverData& data) override;
	bool SolvePositionConstraints(const b2SolverData& data) override;

	b2Vec2 m_groundAnchorA;
	b2Vec2 m_groundAnchorB;
	float_t m_lengthA;
	float_t m_lengthB;
	
	// Solver shared
	b2Vec2 m_localAnchorA;
	b2Vec2 m_localAnchorB;
	float_t m_constant;
	float_t m_ratio;
	float_t m_impulse;

	// Solver temp
	index_t m_indexA;
	index_t m_indexB;
	b2Vec2 m_uA;
	b2Vec2 m_uB;
	b2Vec2 m_rA;
	b2Vec2 m_rB;
	b2Vec2 m_localCenterA;
	b2Vec2 m_localCenterB;
	float_t m_invMassA;
	float_t m_invMassB;
	float_t m_invIA;
	float_t m_invIB;
	float_t m_mass;
};

} // namespace box2d

#endif
