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

#ifndef B2_WELD_JOINT_H
#define B2_WELD_JOINT_H

#include <Box2D/Dynamics/Joints/b2Joint.h>

namespace box2d {

/// Weld joint definition. You need to specify local anchor points
/// where they are attached and the relative body angle. The position
/// of the anchor points is important for computing the reaction torque.
struct b2WeldJointDef : public b2JointDef
{
	constexpr b2WeldJointDef() noexcept: b2JointDef(e_weldJoint) {}

	/// Initialize the bodies, anchors, and reference angle using a world
	/// anchor point.
	void Initialize(b2Body* bodyA, b2Body* bodyB, const b2Vec2& anchor);

	/// The local anchor point relative to bodyA's origin.
	b2Vec2 localAnchorA = b2Vec2_zero;

	/// The local anchor point relative to bodyB's origin.
	b2Vec2 localAnchorB = b2Vec2_zero;

	/// The bodyB angle minus bodyA angle in the reference state (radians).
	b2Float referenceAngle = b2Float{0};
	
	/// The mass-spring-damper frequency in Hertz. Rotation only.
	/// Disable softness with a value of 0.
	b2Float frequencyHz = b2Float{0};

	/// The damping ratio. 0 = no damping, 1 = critical damping.
	b2Float dampingRatio = b2Float{0};
};

/// A weld joint essentially glues two bodies together. A weld joint may
/// distort somewhat because the island constraint solver is approximate.
class b2WeldJoint : public b2Joint
{
public:
	b2Vec2 GetAnchorA() const override;
	b2Vec2 GetAnchorB() const override;

	b2Vec2 GetReactionForce(b2Float inv_dt) const override;
	b2Float GetReactionTorque(b2Float inv_dt) const override;

	/// The local anchor point relative to bodyA's origin.
	const b2Vec2& GetLocalAnchorA() const { return m_localAnchorA; }

	/// The local anchor point relative to bodyB's origin.
	const b2Vec2& GetLocalAnchorB() const  { return m_localAnchorB; }

	/// Get the reference angle.
	b2Float GetReferenceAngle() const { return m_referenceAngle; }

	/// Set/get frequency in Hz.
	void SetFrequency(b2Float hz) { m_frequencyHz = hz; }
	b2Float GetFrequency() const { return m_frequencyHz; }

	/// Set/get damping ratio.
	void SetDampingRatio(b2Float ratio) { m_dampingRatio = ratio; }
	b2Float GetDampingRatio() const { return m_dampingRatio; }

	/// Dump to b2Log
	void Dump() override;

protected:

	friend class b2Joint;

	b2WeldJoint(const b2WeldJointDef* def);

	void InitVelocityConstraints(const b2SolverData& data) override;
	void SolveVelocityConstraints(const b2SolverData& data) override;
	bool SolvePositionConstraints(const b2SolverData& data) override;

	b2Float m_frequencyHz;
	b2Float m_dampingRatio;
	b2Float m_bias;

	// Solver shared
	b2Vec2 m_localAnchorA;
	b2Vec2 m_localAnchorB;
	b2Float m_referenceAngle;
	b2Float m_gamma;
	b2Vec3 m_impulse;

	// Solver temp
	index_t m_indexA;
	index_t m_indexB;
	b2Vec2 m_rA;
	b2Vec2 m_rB;
	b2Vec2 m_localCenterA;
	b2Vec2 m_localCenterB;
	b2Float m_invMassA;
	b2Float m_invMassB;
	b2Float m_invIA;
	b2Float m_invIB;
	b2Mat33 m_mass;
};

} // namespace box2d

#endif
