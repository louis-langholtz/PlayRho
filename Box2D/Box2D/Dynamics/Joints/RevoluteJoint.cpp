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

#include <Box2D/Dynamics/Joints/RevoluteJoint.hpp>
#include <Box2D/Dynamics/Body.hpp>
#include <Box2D/Dynamics/StepConf.hpp>
#include <Box2D/Dynamics/Contacts/ContactSolver.hpp>
#include <Box2D/Dynamics/Contacts/BodyConstraint.hpp>

using namespace box2d;

// Point-to-point constraint
// C = p2 - p1
// Cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-I -r1_skew I r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

// Motor constraint
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2

RevoluteJointDef::RevoluteJointDef(Body* bA, Body* bB, const Vec2 anchor, bool cc):
	JointDef{JointType::Revolute, bA, bB, cc},
	localAnchorA{GetLocalPoint(*bA, anchor)},
	localAnchorB{GetLocalPoint(*bB, anchor)},
	referenceAngle{bB->GetAngle() - bA->GetAngle()}
{
	// Intentionally empty.
}

RevoluteJoint::RevoluteJoint(const RevoluteJointDef& def):
	Joint{def},
	m_localAnchorA{def.localAnchorA},
	m_localAnchorB{def.localAnchorB},
	m_enableMotor{def.enableMotor},
	m_maxMotorTorque{def.maxMotorTorque},
	m_motorSpeed{def.motorSpeed},
	m_enableLimit{def.enableLimit},
	m_referenceAngle{def.referenceAngle},
	m_lowerAngle{def.lowerAngle},
	m_upperAngle{def.upperAngle}
{
	// Intentionally empty.
}

void RevoluteJoint::InitVelocityConstraints(BodyConstraints& bodies,
											const StepConf& step,
											const ConstraintSolverConf& conf)
{
	auto& bodiesA = bodies.at(GetBodyA());
	auto& bodiesB = bodies.at(GetBodyB());

	m_localCenterA = bodiesA.GetLocalCenter();
	m_invMassA = bodiesA.GetInvMass();
	m_invIA = bodiesA.GetInvRotInertia();
	const auto aA = bodiesA.GetPosition().angular;
	auto velA = bodiesA.GetVelocity();

	m_localCenterB = bodiesB.GetLocalCenter();
	m_invMassB = bodiesB.GetInvMass();
	m_invIB = bodiesB.GetInvRotInertia();
	const auto aB = bodiesB.GetPosition().angular;
	auto velB = bodiesB.GetVelocity();

	const auto qA = UnitVec2(aA);
	const auto qB = UnitVec2(aB);

	m_rA = Rotate(m_localAnchorA - m_localCenterA, qA);
	m_rB = Rotate(m_localAnchorB - m_localCenterB, qB);

	// J = [-I -r1_skew I r2_skew]
	//     [ 0       -1 0       1]
	// r_skew = [-ry; rx]

	// Matlab
	// K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
	//     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
	//     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

	const auto mA = m_invMassA;
	const auto mB = m_invMassB;
	const auto iA = m_invIA;
	const auto iB = m_invIB;
	const auto totInvI = iA + iB;

	const auto fixedRotation = (totInvI == 0);

	m_mass.ex.x = mA + mB + (m_rA.y * m_rA.y * iA) + (m_rB.y * m_rB.y * iB);
	m_mass.ey.x = (-m_rA.y * m_rA.x * iA) + (-m_rB.y * m_rB.x * iB);
	m_mass.ez.x = (-m_rA.y * iA) + (-m_rB.y * iB);
	m_mass.ex.y = m_mass.ey.x;
	m_mass.ey.y = mA + mB + (m_rA.x * m_rA.x * iA) + (m_rB.x * m_rB.x * iB);
	m_mass.ez.y = (m_rA.x * iA) + (m_rB.x * iB);
	m_mass.ex.z = m_mass.ez.x;
	m_mass.ey.z = m_mass.ez.y;
	m_mass.ez.z = totInvI;

	m_motorMass = (totInvI > 0)? 1 / totInvI: totInvI;

	if (!m_enableMotor || fixedRotation)
	{
		m_motorImpulse = 0;
	}

	if (m_enableLimit && !fixedRotation)
	{
		const auto jointAngle = aB - aA - GetReferenceAngle();
		if (Abs(m_upperAngle - m_lowerAngle) < (2_rad * conf.angularSlop))
		{
			m_limitState = e_equalLimits;
		}
		else if (jointAngle <= m_lowerAngle)
		{
			if (m_limitState != e_atLowerLimit)
			{
				m_impulse.z = 0;
			}
			m_limitState = e_atLowerLimit;
		}
		else if (jointAngle >= m_upperAngle)
		{
			if (m_limitState != e_atUpperLimit)
			{
				m_impulse.z = 0;
			}
			m_limitState = e_atUpperLimit;
		}
		else
		{
			m_limitState = e_inactiveLimit;
			m_impulse.z = 0;
		}
	}
	else
	{
		m_limitState = e_inactiveLimit;
	}

	if (step.doWarmStart)
	{
		// Scale impulses to support a variable time step.
		m_impulse *= step.dtRatio;
		m_motorImpulse *= step.dtRatio;

		const auto P = Vec2{m_impulse.x, m_impulse.y};

		velA -= Velocity{mA * P, 1_rad * iA * (Cross(m_rA, P) + m_motorImpulse + m_impulse.z)};
		velB += Velocity{mB * P, 1_rad * iB * (Cross(m_rB, P) + m_motorImpulse + m_impulse.z)};
	}
	else
	{
		m_impulse = Vec3_zero;
		m_motorImpulse = 0;
	}

	bodiesA.SetVelocity(velA);
	bodiesB.SetVelocity(velB);
}

RealNum RevoluteJoint::SolveVelocityConstraints(BodyConstraints& bodies, const StepConf& step)
{
	auto& bodiesA = bodies.at(GetBodyA());
	auto& bodiesB = bodies.at(GetBodyB());

	auto velA = bodiesA.GetVelocity();
	auto velB = bodiesB.GetVelocity();

	const auto mA = m_invMassA;
	const auto mB = m_invMassB;
	const auto iA = m_invIA;
	const auto iB = m_invIB;

	const auto fixedRotation = (iA + iB == 0);

	// Solve motor constraint.
	if (m_enableMotor && (m_limitState != e_equalLimits) && !fixedRotation)
	{
		const auto difSpeed = (velB.angular - velA.angular).ToRadians() - m_motorSpeed;
		const auto impulse = -m_motorMass * difSpeed;
		const auto oldImpulse = m_motorImpulse;
		const auto maxImpulse = step.get_dt() * m_maxMotorTorque;
		m_motorImpulse = Clamp(m_motorImpulse + impulse, -maxImpulse, maxImpulse);
		const auto incImpulse = m_motorImpulse - oldImpulse;

		velA.angular -= 1_rad * iA * incImpulse;
		velB.angular += 1_rad * iB * incImpulse;
	}

	// Solve limit constraint.
	if (m_enableLimit && (m_limitState != e_inactiveLimit) && !fixedRotation)
	{
		const auto Cdot1 = velB.linear + (GetRevPerpendicular(m_rB) * velB.angular.ToRadians()) - velA.linear - (GetRevPerpendicular(m_rA) * velA.angular.ToRadians());
		const auto Cdot2 = (velB.angular - velA.angular).ToRadians();
		const auto Cdot = Vec3(Cdot1.x, Cdot1.y, Cdot2);

		auto impulse = -Solve33(m_mass, Cdot);

		if (m_limitState == e_equalLimits)
		{
			m_impulse += impulse;
		}
		else if (m_limitState == e_atLowerLimit)
		{
			const auto newImpulse = m_impulse.z + impulse.z;
			if (newImpulse < 0)
			{
				const auto rhs = -Cdot1 + m_impulse.z * Vec2{m_mass.ez.x, m_mass.ez.y};
				const auto reduced = Solve22(m_mass, rhs);
				impulse.x = reduced.x;
				impulse.y = reduced.y;
				impulse.z = -m_impulse.z;
				m_impulse.x += reduced.x;
				m_impulse.y += reduced.y;
				m_impulse.z = 0;
			}
			else
			{
				m_impulse += impulse;
			}
		}
		else if (m_limitState == e_atUpperLimit)
		{
			const auto newImpulse = m_impulse.z + impulse.z;
			if (newImpulse > 0)
			{
				const auto rhs = -Cdot1 + m_impulse.z * Vec2{m_mass.ez.x, m_mass.ez.y};
				const auto reduced = Solve22(m_mass, rhs);
				impulse.x = reduced.x;
				impulse.y = reduced.y;
				impulse.z = -m_impulse.z;
				m_impulse.x += reduced.x;
				m_impulse.y += reduced.y;
				m_impulse.z = 0;
			}
			else
			{
				m_impulse += impulse;
			}
		}

		const auto P = Vec2{impulse.x, impulse.y};

		velA -= Velocity{mA * P, 1_rad * iA * (Cross(m_rA, P) + impulse.z)};
		velB += Velocity{mB * P, 1_rad * iB * (Cross(m_rB, P) + impulse.z)};
	}
	else
	{
		// Solve point-to-point constraint
		const auto Cdot = (velB.linear + (GetRevPerpendicular(m_rB) * velB.angular.ToRadians()))
		                - (velA.linear + (GetRevPerpendicular(m_rA) * velA.angular.ToRadians()));
		const auto impulse = Solve22(m_mass, -Cdot);

		m_impulse.x += impulse.x;
		m_impulse.y += impulse.y;

		velA -= Velocity{mA * impulse, 1_rad * iA * Cross(m_rA, impulse)};
		velB += Velocity{mB * impulse, 1_rad * iB * Cross(m_rB, impulse)};
	}

	bodiesA.SetVelocity(velA);
	bodiesB.SetVelocity(velB);
	
	return GetInvalid<RealNum>(); // TODO
}

bool RevoluteJoint::SolvePositionConstraints(BodyConstraints& bodies, const ConstraintSolverConf& conf) const
{
	auto& bodiesA = bodies.at(GetBodyA());
	auto& bodiesB = bodies.at(GetBodyB());

	auto posA = bodiesA.GetPosition();
	auto posB = bodiesB.GetPosition();

	auto angularError = RealNum{0};
	auto positionError = RealNum{0};

	const auto iA = m_invIA;
	const auto iB = m_invIB;

	const auto fixedRotation = ((iA + iB) == 0);

	// Solve angular limit constraint.
	if (m_enableLimit && m_limitState != e_inactiveLimit && !fixedRotation)
	{
		const auto angle = posB.angular - posA.angular - GetReferenceAngle();
		auto limitImpulse = RealNum{0};

		if (m_limitState == e_equalLimits)
		{
			// Prevent large angular corrections
			const auto C = Clamp((angle - m_lowerAngle).ToRadians(), -conf.maxLinearCorrection, conf.maxLinearCorrection);
			limitImpulse = -m_motorMass * C;
			angularError = Abs(C);
		}
		else if (m_limitState == e_atLowerLimit)
		{
			auto C = (angle - m_lowerAngle).ToRadians();
			angularError = -C;

			// Prevent large angular corrections and allow some slop.
			C = Clamp(C + conf.angularSlop, -conf.maxAngularCorrection, RealNum{0});
			limitImpulse = -m_motorMass * C;
		}
		else if (m_limitState == e_atUpperLimit)
		{
			auto C = (angle - m_upperAngle).ToRadians();
			angularError = C;

			// Prevent large angular corrections and allow some slop.
			C = Clamp(C - conf.angularSlop, RealNum{0}, conf.maxAngularCorrection);
			limitImpulse = -m_motorMass * C;
		}

		posA.angular -= 1_rad * iA * limitImpulse;
		posB.angular += 1_rad * iB * limitImpulse;
	}

	// Solve point-to-point constraint.
	{
		const auto qA = UnitVec2(posA.angular);
		const auto qB = UnitVec2(posB.angular);

		const auto rA = Rotate(m_localAnchorA - m_localCenterA, qA);
		const auto rB = Rotate(m_localAnchorB - m_localCenterB, qB);

		const auto C = (posB.linear + rB) - (posA.linear + rA);
		positionError = GetLength(C);

		const auto mA = m_invMassA;
		const auto mB = m_invMassB;

		Mat22 K;
		K.ex.x = mA + mB + (iA * rA.y * rA.y) + (iB * rB.y * rB.y);
		K.ex.y = (-iA * rA.x * rA.y) + (-iB * rB.x * rB.y);
		K.ey.x = K.ex.y;
		K.ey.y = mA + mB + (iA * rA.x * rA.x) + (iB * rB.x * rB.x);

		const auto impulse = -Solve(K, C);

		posA -= Position{mA * impulse, 1_rad * iA * Cross(rA, impulse)};
		posB += Position{mB * impulse, 1_rad * iB * Cross(rB, impulse)};
	}

	bodiesA.SetPosition(posA);
	bodiesB.SetPosition(posB);
	
	return (positionError <= conf.linearSlop) && (angularError <= conf.angularSlop);
}

Vec2 RevoluteJoint::GetAnchorA() const
{
	return GetWorldPoint(*GetBodyA(), GetLocalAnchorA());
}

Vec2 RevoluteJoint::GetAnchorB() const
{
	return GetWorldPoint(*GetBodyB(), GetLocalAnchorB());
}

Vec2 RevoluteJoint::GetReactionForce(RealNum inv_dt) const
{
	return inv_dt * Vec2{m_impulse.x, m_impulse.y};
}

RealNum RevoluteJoint::GetReactionTorque(RealNum inv_dt) const
{
	return inv_dt * m_impulse.z;
}

void RevoluteJoint::EnableMotor(bool flag)
{
	GetBodyA()->SetAwake();
	GetBodyB()->SetAwake();
	m_enableMotor = flag;
}

RealNum RevoluteJoint::GetMotorTorque(RealNum inv_dt) const
{
	return inv_dt * m_motorImpulse;
}

void RevoluteJoint::SetMotorSpeed(RealNum speed)
{
	GetBodyA()->SetAwake();
	GetBodyB()->SetAwake();
	m_motorSpeed = speed;
}

void RevoluteJoint::SetMaxMotorTorque(RealNum torque)
{
	GetBodyA()->SetAwake();
	GetBodyB()->SetAwake();
	m_maxMotorTorque = torque;
}

void RevoluteJoint::EnableLimit(bool flag)
{
	if (flag != m_enableLimit)
	{
		GetBodyA()->SetAwake();
		GetBodyB()->SetAwake();
		m_enableLimit = flag;
		m_impulse.z = 0;
	}
}

void RevoluteJoint::SetLimits(Angle lower, Angle upper)
{
	assert(lower <= upper);
	
	if ((lower != m_lowerAngle) || (upper != m_upperAngle))
	{
		GetBodyA()->SetAwake();
		GetBodyB()->SetAwake();
		m_impulse.z = 0;
		m_lowerAngle = lower;
		m_upperAngle = upper;
	}
}

Angle box2d::GetJointAngle(const RevoluteJoint& joint)
{
	return joint.GetBodyB()->GetAngle() - joint.GetBodyA()->GetAngle() - joint.GetReferenceAngle();
}

Angle box2d::GetJointSpeed(const RevoluteJoint& joint)
{
	return joint.GetBodyB()->GetVelocity().angular - joint.GetBodyA()->GetVelocity().angular;
}
