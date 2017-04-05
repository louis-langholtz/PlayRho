/*
* Original work Copyright (c) 2006-2012 Erin Catto http://www.box2d.org
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

#ifndef B2_MOTOR_JOINT_H
#define B2_MOTOR_JOINT_H

#include <Box2D/Dynamics/Joints/Joint.hpp>

namespace box2d {

/// Motor joint definition.
struct MotorJointDef : public JointDef
{
	constexpr MotorJointDef() noexcept: JointDef(JointType::Motor) {}

	/// Initialize the bodies and offsets using the current transforms.
	void Initialize(Body* bodyA, Body* bodyB);

	/// Position of bodyB minus the position of bodyA, in bodyA's frame, in meters.
	Vec2 linearOffset = Vec2_zero;

	/// The bodyB angle minus bodyA angle in radians.
	Angle angularOffset = Angle{0};
	
	/// The maximum motor force in N.
	RealNum maxForce = RealNum{1};

	/// The maximum motor torque in N-m.
	RealNum maxTorque = RealNum{1};

	/// Position correction factor in the range [0,1].
	RealNum correctionFactor = RealNum(0.3);
};

/// A motor joint is used to control the relative motion
/// between two bodies. A typical usage is to control the movement
/// of a dynamic body with respect to the ground.
class MotorJoint : public Joint
{
public:
	MotorJoint(const MotorJointDef& def);

	Vec2 GetAnchorA() const override;
	Vec2 GetAnchorB() const override;

	Vec2 GetReactionForce(RealNum inv_dt) const override;
	RealNum GetReactionTorque(RealNum inv_dt) const override;

	/// Set/get the target linear offset, in frame A, in meters.
	void SetLinearOffset(const Vec2 linearOffset);
	const Vec2 GetLinearOffset() const;

	/// Set/get the target angular offset, in radians.
	void SetAngularOffset(Angle angularOffset);
	Angle GetAngularOffset() const;

	/// Set the maximum friction force in N.
	void SetMaxForce(RealNum force);

	/// Get the maximum friction force in N.
	RealNum GetMaxForce() const;

	/// Set the maximum friction torque in N*m.
	void SetMaxTorque(RealNum torque);

	/// Get the maximum friction torque in N*m.
	RealNum GetMaxTorque() const;

	/// Set the position correction factor in the range [0,1].
	void SetCorrectionFactor(RealNum factor);

	/// Get the position correction factor in the range [0,1].
	RealNum GetCorrectionFactor() const;

private:

	void InitVelocityConstraints(BodyConstraints& bodies, const StepConf& step, const ConstraintSolverConf& conf) override;
	RealNum SolveVelocityConstraints(BodyConstraints& bodies, const StepConf& step) override;
	bool SolvePositionConstraints(BodyConstraints& bodies, const ConstraintSolverConf& conf) const override;

	// Solver shared
	Vec2 m_linearOffset;
	Angle m_angularOffset;
	Vec2 m_linearImpulse = Vec2_zero;
	RealNum m_angularImpulse = 0;
	RealNum m_maxForce;
	RealNum m_maxTorque;
	RealNum m_correctionFactor;

	// Solver temp
	Vec2 m_rA;
	Vec2 m_rB;
	Vec2 m_localCenterA;
	Vec2 m_localCenterB;
	Vec2 m_linearError;
	Angle m_angularError;
	RealNum m_invMassA;
	RealNum m_invMassB;
	RealNum m_invIA;
	RealNum m_invIB;
	Mat22 m_linearMass;
	RealNum m_angularMass;
};

} // namespace box2d

#endif
