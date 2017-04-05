/*
* Original work Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
* Modified work Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

#include <Box2D/Dynamics/Joints/Joint.hpp>

namespace box2d {

/// Weld joint definition. You need to specify local anchor points
/// where they are attached and the relative body angle. The position
/// of the anchor points is important for computing the reaction torque.
struct WeldJointDef : public JointDef
{
	constexpr WeldJointDef() noexcept: JointDef(JointType::Weld) {}

	/// Initialize the bodies, anchors, and reference angle using a world
	/// anchor point.
	void Initialize(Body* bodyA, Body* bodyB, const Vec2 anchor);

	/// The local anchor point relative to bodyA's origin.
	Vec2 localAnchorA = Vec2_zero;

	/// The local anchor point relative to bodyB's origin.
	Vec2 localAnchorB = Vec2_zero;

	/// The bodyB angle minus bodyA angle in the reference state (radians).
	Angle referenceAngle = Angle{0};
	
	/// The mass-spring-damper frequency in Hertz. Rotation only.
	/// Disable softness with a value of 0.
	RealNum frequencyHz = 0;

	/// The damping ratio. 0 = no damping, 1 = critical damping.
	RealNum dampingRatio = 0;
};

/// A weld joint essentially glues two bodies together. A weld joint may
/// distort somewhat because the island constraint solver is approximate.
class WeldJoint : public Joint
{
public:
	WeldJoint(const WeldJointDef& def);

	Vec2 GetAnchorA() const override;
	Vec2 GetAnchorB() const override;

	Vec2 GetReactionForce(RealNum inv_dt) const override;
	RealNum GetReactionTorque(RealNum inv_dt) const override;

	/// The local anchor point relative to bodyA's origin.
	Vec2 GetLocalAnchorA() const { return m_localAnchorA; }

	/// The local anchor point relative to bodyB's origin.
	Vec2 GetLocalAnchorB() const  { return m_localAnchorB; }

	/// Get the reference angle.
	Angle GetReferenceAngle() const { return m_referenceAngle; }

	/// Set/get frequency in Hz.
	void SetFrequency(RealNum hz) { m_frequencyHz = hz; }
	RealNum GetFrequency() const { return m_frequencyHz; }

	/// Set/get damping ratio.
	void SetDampingRatio(RealNum ratio) { m_dampingRatio = ratio; }
	RealNum GetDampingRatio() const { return m_dampingRatio; }

private:

	void InitVelocityConstraints(BodyConstraints& bodies, const StepConf& step, const ConstraintSolverConf& conf) override;
	RealNum SolveVelocityConstraints(BodyConstraints& bodies, const StepConf& step) override;
	bool SolvePositionConstraints(BodyConstraints& bodies, const ConstraintSolverConf& conf) const override;

	RealNum m_frequencyHz;
	RealNum m_dampingRatio;
	RealNum m_bias;

	// Solver shared
	Vec2 m_localAnchorA;
	Vec2 m_localAnchorB;
	Angle m_referenceAngle;
	RealNum m_gamma;
	Vec3 m_impulse = Vec3_zero;

	// Solver temp
	Vec2 m_rA;
	Vec2 m_rB;
	Vec2 m_localCenterA;
	Vec2 m_localCenterB;
	RealNum m_invMassA;
	RealNum m_invMassB;
	RealNum m_invIA;
	RealNum m_invIB;
	Mat33 m_mass;
};

} // namespace box2d

#endif
