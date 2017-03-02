/*
* Original work Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
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

#ifndef B2_FRICTION_JOINT_H
#define B2_FRICTION_JOINT_H

#include <Box2D/Dynamics/Joints/Joint.hpp>

namespace box2d {

/// Friction joint definition.
struct FrictionJointDef : public JointDef
{
	constexpr FrictionJointDef() noexcept: JointDef(JointType::Friction) {}

	/// Initialize the bodies, anchors, axis, and reference angle using the world
	/// anchor and world axis.
	void Initialize(Body* bodyA, Body* bodyB, const Vec2 anchor);

	/// The local anchor point relative to bodyA's origin.
	Vec2 localAnchorA = Vec2_zero;

	/// The local anchor point relative to bodyB's origin.
	Vec2 localAnchorB = Vec2_zero;

	/// The maximum friction force in N.
	RealNum maxForce = 0;

	/// The maximum friction torque in N-m.
	RealNum maxTorque = 0;
};

/// Friction joint. This is used for top-down friction.
/// It provides 2D translational friction and angular friction.
class FrictionJoint : public Joint
{
public:
	FrictionJoint(const FrictionJointDef& def);

	Vec2 GetAnchorA() const override;
	Vec2 GetAnchorB() const override;

	Vec2 GetReactionForce(RealNum inv_dt) const override;
	RealNum GetReactionTorque(RealNum inv_dt) const override;

	/// The local anchor point relative to bodyA's origin.
	Vec2 GetLocalAnchorA() const { return m_localAnchorA; }

	/// The local anchor point relative to bodyB's origin.
	Vec2 GetLocalAnchorB() const  { return m_localAnchorB; }

	/// Set the maximum friction force in N.
	void SetMaxForce(RealNum force);

	/// Get the maximum friction force in N.
	RealNum GetMaxForce() const;

	/// Set the maximum friction torque in N*m.
	void SetMaxTorque(RealNum torque);

	/// Get the maximum friction torque in N*m.
	RealNum GetMaxTorque() const;

private:

	void InitVelocityConstraints(BodyConstraints& bodies, const StepConf& step, const ConstraintSolverConf& conf) override;
	RealNum SolveVelocityConstraints(BodyConstraints& bodies, const StepConf& step) override;
	bool SolvePositionConstraints(BodyConstraints& bodies, const ConstraintSolverConf& conf) const override;

	Vec2 m_localAnchorA;
	Vec2 m_localAnchorB;

	// Solver shared
	Vec2 m_linearImpulse = Vec2_zero; ///< Linear impulse.
	RealNum m_angularImpulse = 0;
	RealNum m_maxForce;
	RealNum m_maxTorque;

	// Solver temp
	Vec2 m_rA;
	Vec2 m_rB;
	Vec2 m_localCenterA;
	Vec2 m_localCenterB;
	RealNum m_invMassA;
	RealNum m_invMassB;
	RealNum m_invIA;
	RealNum m_invIB;
	Mat22 m_linearMass;
	RealNum m_angularMass;
};

} // namespace box2d

#endif
