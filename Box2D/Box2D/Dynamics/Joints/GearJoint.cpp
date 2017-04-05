/*
* Original work Copyright (c) 2007-2011 Erin Catto http://www.box2d.org
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

#include <Box2D/Dynamics/Joints/GearJoint.hpp>
#include <Box2D/Dynamics/Joints/RevoluteJoint.hpp>
#include <Box2D/Dynamics/Joints/PrismaticJoint.hpp>
#include <Box2D/Dynamics/Body.hpp>
#include <Box2D/Dynamics/StepConf.hpp>
#include <Box2D/Dynamics/Contacts/ContactSolver.hpp>
#include <Box2D/Dynamics/Contacts/BodyConstraint.hpp>

using namespace box2d;

// Gear Joint:
// C0 = (coordinate1 + ratio * coordinate2)_initial
// C = (coordinate1 + ratio * coordinate2) - C0 = 0
// J = [J1 ratio * J2]
// K = J * invM * JT
//   = J1 * invM1 * J1T + ratio * ratio * J2 * invM2 * J2T
//
// Revolute:
// coordinate = rotation
// Cdot = angularVelocity
// J = [0 0 1]
// K = J * invM * JT = invI
//
// Prismatic:
// coordinate = dot(p - pg, ug)
// Cdot = dot(v + cross(w, r), ug)
// J = [ug cross(r, ug)]
// K = J * invM * JT = invMass + invI * cross(r, ug)^2

GearJoint::GearJoint(const GearJointDef& def):
	Joint(def),
	m_joint1(def.joint1),
	m_joint2(def.joint2),
	m_typeA(def.joint1->GetType()),
	m_typeB(def.joint2->GetType()),
	m_ratio(def.ratio)
{
	assert(m_typeA == JointType::Revolute || m_typeA == JointType::Prismatic);
	assert(m_typeB == JointType::Revolute || m_typeB == JointType::Prismatic);

	// TODO_ERIN there might be some problem with the joint edges in Joint.

	m_bodyC = m_joint1->GetBodyA();
	SetBodyA(m_joint1->GetBodyB());

	// Get geometry of joint1
	const auto xfA = GetBodyA()->GetTransformation();
	const auto aA = GetBodyA()->GetAngle();
	const auto xfC = m_bodyC->GetTransformation();
	const auto aC = m_bodyC->GetAngle();

	Angle coordinateA;
	if (m_typeA == JointType::Revolute)
	{
		const auto revolute = static_cast<RevoluteJoint*>(def.joint1);
		m_localAnchorC = revolute->GetLocalAnchorA();
		m_localAnchorA = revolute->GetLocalAnchorB();
		m_referenceAngleA = revolute->GetReferenceAngle();
		m_localAxisC = UnitVec2::GetZero();

		coordinateA = aA - aC - m_referenceAngleA;
	}
	else
	{
		const auto prismatic = static_cast<PrismaticJoint*>(def.joint1);
		m_localAnchorC = prismatic->GetLocalAnchorA();
		m_localAnchorA = prismatic->GetLocalAnchorB();
		m_referenceAngleA = prismatic->GetReferenceAngle();
		m_localAxisC = prismatic->GetLocalAxisA();

		const auto pC = m_localAnchorC;
		const auto pA = InverseRotate(Rotate(m_localAnchorA, xfA.q) + (xfA.p - xfC.p), xfC.q);
		coordinateA = Dot(pA - pC, m_localAxisC) * Radian;
	}

	m_bodyD = m_joint2->GetBodyA();
	SetBodyB(m_joint2->GetBodyB());

	// Get geometry of joint2
	const auto xfB = GetBodyB()->GetTransformation();
	const auto aB = GetBodyB()->GetAngle();
	const auto xfD = m_bodyD->GetTransformation();
	const auto aD = m_bodyD->GetAngle();

	Angle coordinateB;
	if (m_typeB == JointType::Revolute)
	{
		const auto revolute = static_cast<RevoluteJoint*>(def.joint2);
		m_localAnchorD = revolute->GetLocalAnchorA();
		m_localAnchorB = revolute->GetLocalAnchorB();
		m_referenceAngleB = revolute->GetReferenceAngle();
		m_localAxisD = UnitVec2::GetZero();

		coordinateB = aB - aD - m_referenceAngleB;
	}
	else
	{
		const auto prismatic = static_cast<PrismaticJoint*>(def.joint2);
		m_localAnchorD = prismatic->GetLocalAnchorA();
		m_localAnchorB = prismatic->GetLocalAnchorB();
		m_referenceAngleB = prismatic->GetReferenceAngle();
		m_localAxisD = prismatic->GetLocalAxisA();

		const auto pD = m_localAnchorD;
		const auto pB = InverseRotate(Rotate(m_localAnchorB, xfB.q) + (xfB.p - xfD.p), xfD.q);
		coordinateB = Dot(pB - pD, m_localAxisD) * Radian;
	}

	m_constant = coordinateA + m_ratio * coordinateB;
}

void GearJoint::InitVelocityConstraints(BodyConstraints& bodies, const StepConf& step, const ConstraintSolverConf&)
{
	auto& bodiesA = bodies.at(GetBodyA());
	auto& bodiesB = bodies.at(GetBodyB());
	auto& bodiesC = bodies.at(m_bodyC);
	auto& bodiesD = bodies.at(m_bodyD);

	m_lcA = bodiesA.GetLocalCenter();
	m_mA = RealNum{bodiesA.GetInvMass() * Kilogram};
	m_iA = bodiesA.GetInvRotInertia() * (SquareMeter * Kilogram / SquareRadian);
	auto velA = bodiesA.GetVelocity();
	const auto aA = bodiesA.GetPosition().angular;

	m_lcB = bodiesB.GetLocalCenter();
	m_mB = RealNum{bodiesB.GetInvMass() * Kilogram};
	m_iB = bodiesB.GetInvRotInertia() * (SquareMeter * Kilogram / SquareRadian);
	auto velB = bodiesB.GetVelocity();
	const auto aB = bodiesB.GetPosition().angular;

	m_lcC = bodiesC.GetLocalCenter();
	m_mC = RealNum{bodiesC.GetInvMass() * Kilogram};
	m_iC = bodiesC.GetInvRotInertia() * (SquareMeter * Kilogram / SquareRadian);
	auto velC = bodiesC.GetVelocity();
	const auto aC = bodiesC.GetPosition().angular;

	m_lcD = bodiesD.GetLocalCenter();
	m_mD = RealNum{bodiesD.GetInvMass() * Kilogram};
	m_iD = bodiesD.GetInvRotInertia() * (SquareMeter * Kilogram / SquareRadian);
	auto velD = bodiesD.GetVelocity();
	const auto aD = bodiesD.GetPosition().angular;

	const auto qA = UnitVec2(aA);
	const auto qB = UnitVec2(aB);
	const auto qC = UnitVec2(aC);
	const auto qD = UnitVec2(aD);

	m_mass = 0;

	if (m_typeA == JointType::Revolute)
	{
		m_JvAC = Vec2_zero;
		m_JwA = 1;
		m_JwC = 1;
		m_mass += m_iA + m_iC;
	}
	else
	{
		const auto u = Rotate(m_localAxisC, qC);
		const auto rC = Rotate(m_localAnchorC - m_lcC, qC);
		const auto rA = Rotate(m_localAnchorA - m_lcA, qA);
		m_JvAC = u * 1;
		m_JwC = Cross(rC, u);
		m_JwA = Cross(rA, u);
		m_mass += m_mC + m_mA + m_iC * Square(m_JwC) + m_iA * Square(m_JwA);
	}

	if (m_typeB == JointType::Revolute)
	{
		m_JvBD = Vec2_zero;
		m_JwB = m_ratio;
		m_JwD = m_ratio;
		m_mass += Square(m_ratio) * (m_iB + m_iD);
	}
	else
	{
		const auto u = Rotate(m_localAxisD, qD);
		const auto rD = Rotate(m_localAnchorD - m_lcD, qD);
		const auto rB = Rotate(m_localAnchorB - m_lcB, qB);
		m_JvBD = m_ratio * u;
		m_JwD = m_ratio * Cross(rD, u);
		m_JwB = m_ratio * Cross(rB, u);
		m_mass += Square(m_ratio) * (m_mD + m_mB) + m_iD * Square(m_JwD) + m_iB * Square(m_JwB);
	}

	// Compute effective mass.
	m_mass = (m_mass > 0) ? RealNum{1} / m_mass : RealNum{0};

	if (step.doWarmStart)
	{
		velA += Velocity{(m_mA * m_impulse) * m_JvAC, Radian * m_iA * m_impulse * m_JwA};
		velB += Velocity{(m_mB * m_impulse) * m_JvBD, Radian * m_iB * m_impulse * m_JwB};
		velC -= Velocity{(m_mC * m_impulse) * m_JvAC, Radian * m_iC * m_impulse * m_JwC};
		velD -= Velocity{(m_mD * m_impulse) * m_JvBD, Radian * m_iD * m_impulse * m_JwD};
	}
	else
	{
		m_impulse = 0;
	}

	bodiesA.SetVelocity(velA);
	bodiesB.SetVelocity(velB);
	bodiesC.SetVelocity(velC);
	bodiesD.SetVelocity(velD);
}

RealNum GearJoint::SolveVelocityConstraints(BodyConstraints& bodies, const StepConf&)
{
	auto& bodiesA = bodies.at(GetBodyA());
	auto& bodiesB = bodies.at(GetBodyB());
	auto& bodiesC = bodies.at(m_bodyC);
	auto& bodiesD = bodies.at(m_bodyD);

	auto velA = bodiesA.GetVelocity();
	auto velB = bodiesB.GetVelocity();
	auto velC = bodiesC.GetVelocity();
	auto velD = bodiesD.GetVelocity();

	const auto deltaVelAC = velA.linear - velC.linear;
	const auto deltaVelBD = velB.linear - velD.linear;
	const auto Cdot = Dot(m_JvAC, deltaVelAC) + Dot(m_JvBD, deltaVelBD)
		+ RealNum{(m_JwA * velA.angular - m_JwC * velC.angular) / Radian}
		+ RealNum{(m_JwB * velB.angular - m_JwD * velD.angular) / Radian};

	const auto impulse = -m_mass * Cdot;
	m_impulse += impulse;

	velA += Velocity{(m_mA * impulse) * m_JvAC, Radian * m_iA * impulse * m_JwA};
	velB += Velocity{(m_mB * impulse) * m_JvBD, Radian * m_iB * impulse * m_JwB};
	velC -= Velocity{(m_mC * impulse) * m_JvAC, Radian * m_iC * impulse * m_JwC};
	velD -= Velocity{(m_mD * impulse) * m_JvBD, Radian * m_iD * impulse * m_JwD};

	bodiesA.SetVelocity(velA);
	bodiesB.SetVelocity(velB);
	bodiesC.SetVelocity(velC);
	bodiesD.SetVelocity(velD);
	
	return impulse;
}

bool GearJoint::SolvePositionConstraints(BodyConstraints& bodies, const ConstraintSolverConf& conf) const
{
	auto& bodiesA = bodies.at(GetBodyA());
	auto& bodiesB = bodies.at(GetBodyB());
	auto& bodiesC = bodies.at(m_bodyC);
	auto& bodiesD = bodies.at(m_bodyD);

	auto posA = bodiesA.GetPosition();
	auto posB = bodiesB.GetPosition();
	auto posC = bodiesC.GetPosition();
	auto posD = bodiesD.GetPosition();

	const UnitVec2 qA(posA.angular), qB(posB.angular), qC(posC.angular), qD(posD.angular);

	const auto linearError = RealNum{0};

	Angle coordinateA, coordinateB;

	Vec2 JvAC, JvBD;
	RealNum JwA, JwB, JwC, JwD;
	auto mass = RealNum{0};

	if (m_typeA == JointType::Revolute)
	{
		JvAC = Vec2_zero;
		JwA = 1;
		JwC = 1;
		mass += m_iA + m_iC;

		coordinateA = posA.angular - posC.angular - m_referenceAngleA;
	}
	else
	{
		const auto u = Rotate(m_localAxisC, qC);
		const auto rC = Rotate(m_localAnchorC - m_lcC, qC);
		const auto rA = Rotate(m_localAnchorA - m_lcA, qA);
		JvAC = u * 1;
		JwC = Cross(rC, u);
		JwA = Cross(rA, u);
		mass += m_mC + m_mA + m_iC * Square(JwC) + m_iA * Square(JwA);

		const auto pC = m_localAnchorC - m_lcC;
		const auto pA = InverseRotate(rA + (posA.linear - posC.linear), qC);
		coordinateA = Radian * Dot(pA - pC, m_localAxisC);
	}

	if (m_typeB == JointType::Revolute)
	{
		JvBD = Vec2_zero;
		JwB = m_ratio;
		JwD = m_ratio;
		mass += Square(m_ratio) * (m_iB + m_iD);

		coordinateB = posB.angular - posD.angular - m_referenceAngleB;
	}
	else
	{
		const auto u = Rotate(m_localAxisD, qD);
		const auto rD = Rotate(m_localAnchorD - m_lcD, qD);
		const auto rB = Rotate(m_localAnchorB - m_lcB, qB);
		JvBD = m_ratio * u;
		JwD = m_ratio * Cross(rD, u);
		JwB = m_ratio * Cross(rB, u);
		mass += Square(m_ratio) * (m_mD + m_mB) + m_iD * Square(JwD) + m_iB * Square(JwB);

		const auto pD = m_localAnchorD - m_lcD;
		const auto pB = InverseRotate(rB + (posB.linear - posD.linear), qD);
		coordinateB = Radian * Dot(pB - pD, m_localAxisD);
	}

	const auto C = RealNum{((coordinateA + m_ratio * coordinateB) - m_constant) / Radian};

	auto impulse = RealNum{0};
	if (mass > 0)
	{
		impulse = -C / mass;
	}

	posA += Position{m_mA * impulse * JvAC, Radian * m_iA * impulse * JwA};
	posB += Position{m_mB * impulse * JvBD, Radian * m_iB * impulse * JwB};
	posC -= Position{m_mC * impulse * JvAC, Radian * m_iC * impulse * JwC};
	posD -= Position{m_mD * impulse * JvBD, Radian * m_iD * impulse * JwD};

	bodiesA.SetPosition(posA);
	bodiesB.SetPosition(posB);
	bodiesC.SetPosition(posC);
	bodiesD.SetPosition(posD);

	// TODO_ERIN not implemented
	return linearError < conf.linearSlop;
}

Vec2 GearJoint::GetAnchorA() const
{
	return GetWorldPoint(*GetBodyA(), GetLocalAnchorA());
}

Vec2 GearJoint::GetAnchorB() const
{
	return GetWorldPoint(*GetBodyB(), GetLocalAnchorB());
}

Vec2 GearJoint::GetReactionForce(RealNum inv_dt) const
{
	return inv_dt * m_impulse * m_JvAC;
}

RealNum GearJoint::GetReactionTorque(RealNum inv_dt) const
{
	return inv_dt * m_impulse * m_JwA;
}

void GearJoint::SetRatio(RealNum ratio)
{
	assert(IsValid(ratio));
	m_ratio = ratio;
}

RealNum GearJoint::GetRatio() const
{
	return m_ratio;
}
