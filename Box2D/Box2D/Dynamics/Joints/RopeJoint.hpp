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

#ifndef B2_ROPE_JOINT_H
#define B2_ROPE_JOINT_H

#include <Box2D/Dynamics/Joints/Joint.hpp>

namespace box2d {

/// Rope joint definition. This requires two body anchor points and
/// a maximum lengths.
/// Note: by default the connected objects will not collide.
/// see collideConnected in JointDef.
struct RopeJointDef : public JointDef
{
	constexpr RopeJointDef() noexcept: JointDef(JointType::Rope) {}

	constexpr RopeJointDef(Body* bodyA, Body* bodyB) noexcept: JointDef(JointType::Rope, bodyA, bodyB) {}

	/// The local anchor point relative to bodyA's origin.
	Vec2 localAnchorA = Vec2{-float_t{1}, float_t{0}};

	/// The local anchor point relative to bodyB's origin.
	Vec2 localAnchorB = Vec2{float_t{1}, float_t{0}};

	/// The maximum length of the rope.
	float_t maxLength = float_t{0};
};

/// A rope joint enforces a maximum distance between two points
/// on two bodies. It has no other effect.
/// Warning: if you attempt to change the maximum length during
/// the simulation you will get some non-physical behavior.
/// A model that would allow you to dynamically modify the length
/// would have some sponginess, so I chose not to implement it
/// that way. See DistanceJoint if you want to dynamically
/// control length.
class RopeJoint : public Joint
{
public:
	RopeJoint(const RopeJointDef& data);

	Vec2 GetAnchorA() const override;
	Vec2 GetAnchorB() const override;

	Vec2 GetReactionForce(float_t inv_dt) const override;
	float_t GetReactionTorque(float_t inv_dt) const override;

	/// The local anchor point relative to bodyA's origin.
	Vec2 GetLocalAnchorA() const { return m_localAnchorA; }

	/// The local anchor point relative to bodyB's origin.
	Vec2 GetLocalAnchorB() const  { return m_localAnchorB; }

	/// Set/Get the maximum length of the rope.
	void SetMaxLength(float_t length) { m_maxLength = length; }
	float_t GetMaxLength() const;

	LimitState GetLimitState() const;

private:

	void InitVelocityConstraints(Span<Velocity> velocities, Span<const Position> positions, const TimeStep& step, const ConstraintSolverConf& conf) override;
	void SolveVelocityConstraints(Span<Velocity> velocities, const TimeStep& step) override;
	bool SolvePositionConstraints(Span<Position> positions, const ConstraintSolverConf& conf) override;

	// Solver shared
	Vec2 m_localAnchorA;
	Vec2 m_localAnchorB;
	float_t m_maxLength;
	float_t m_length;
	float_t m_impulse;

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
	LimitState m_state;
};

} // namespace box2d

#endif
