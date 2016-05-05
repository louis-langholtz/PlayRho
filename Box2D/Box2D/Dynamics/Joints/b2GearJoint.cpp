/*
* Copyright (c) 2007-2011 Erin Catto http://www.box2d.org
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

#include <Box2D/Dynamics/Joints/b2GearJoint.h>
#include <Box2D/Dynamics/Joints/b2RevoluteJoint.h>
#include <Box2D/Dynamics/Joints/b2PrismaticJoint.h>
#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Dynamics/b2TimeStep.h>

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

b2GearJoint::b2GearJoint(const b2GearJointDef* def)
: b2Joint(def)
{
	m_joint1 = def->joint1;
	m_joint2 = def->joint2;

	m_typeA = m_joint1->GetType();
	m_typeB = m_joint2->GetType();

	b2Assert(m_typeA == e_revoluteJoint || m_typeA == e_prismaticJoint);
	b2Assert(m_typeB == e_revoluteJoint || m_typeB == e_prismaticJoint);

	b2Float coordinateA, coordinateB;

	// TODO_ERIN there might be some problem with the joint edges in b2Joint.

	m_bodyC = m_joint1->GetBodyA();
	m_bodyA = m_joint1->GetBodyB();

	// Get geometry of joint1
	const auto xfA = m_bodyA->m_xf;
	const auto aA = m_bodyA->m_sweep.a;
	const auto xfC = m_bodyC->m_xf;
	const auto aC = m_bodyC->m_sweep.a;

	if (m_typeA == e_revoluteJoint)
	{
		const auto revolute = static_cast<b2RevoluteJoint*>(def->joint1);
		m_localAnchorC = revolute->m_localAnchorA;
		m_localAnchorA = revolute->m_localAnchorB;
		m_referenceAngleA = revolute->m_referenceAngle;
		m_localAxisC = b2Vec2_zero;

		coordinateA = aA - aC - m_referenceAngleA;
	}
	else
	{
		const auto prismatic = static_cast<b2PrismaticJoint*>(def->joint1);
		m_localAnchorC = prismatic->m_localAnchorA;
		m_localAnchorA = prismatic->m_localAnchorB;
		m_referenceAngleA = prismatic->m_referenceAngle;
		m_localAxisC = prismatic->m_localXAxisA;

		const auto pC = m_localAnchorC;
		const auto pA = b2MulT(xfC.q, b2Mul(xfA.q, m_localAnchorA) + (xfA.p - xfC.p));
		coordinateA = b2Dot(pA - pC, m_localAxisC);
	}

	m_bodyD = m_joint2->GetBodyA();
	m_bodyB = m_joint2->GetBodyB();

	// Get geometry of joint2
	const auto xfB = m_bodyB->m_xf;
	const auto aB = m_bodyB->m_sweep.a;
	const auto xfD = m_bodyD->m_xf;
	const auto aD = m_bodyD->m_sweep.a;

	if (m_typeB == e_revoluteJoint)
	{
		const auto revolute = static_cast<b2RevoluteJoint*>(def->joint2);
		m_localAnchorD = revolute->m_localAnchorA;
		m_localAnchorB = revolute->m_localAnchorB;
		m_referenceAngleB = revolute->m_referenceAngle;
		m_localAxisD = b2Vec2_zero;

		coordinateB = aB - aD - m_referenceAngleB;
	}
	else
	{
		const auto prismatic = static_cast<b2PrismaticJoint*>(def->joint2);
		m_localAnchorD = prismatic->m_localAnchorA;
		m_localAnchorB = prismatic->m_localAnchorB;
		m_referenceAngleB = prismatic->m_referenceAngle;
		m_localAxisD = prismatic->m_localXAxisA;

		const auto pD = m_localAnchorD;
		const auto pB = b2MulT(xfD.q, b2Mul(xfB.q, m_localAnchorB) + (xfB.p - xfD.p));
		coordinateB = b2Dot(pB - pD, m_localAxisD);
	}

	m_ratio = def->ratio;

	m_constant = coordinateA + m_ratio * coordinateB;

	m_impulse = b2Float{0};
}

void b2GearJoint::InitVelocityConstraints(const b2SolverData& data)
{
	m_indexA = m_bodyA->m_islandIndex;
	m_indexB = m_bodyB->m_islandIndex;
	m_indexC = m_bodyC->m_islandIndex;
	m_indexD = m_bodyD->m_islandIndex;
	m_lcA = m_bodyA->m_sweep.localCenter;
	m_lcB = m_bodyB->m_sweep.localCenter;
	m_lcC = m_bodyC->m_sweep.localCenter;
	m_lcD = m_bodyD->m_sweep.localCenter;
	m_mA = m_bodyA->m_invMass;
	m_mB = m_bodyB->m_invMass;
	m_mC = m_bodyC->m_invMass;
	m_mD = m_bodyD->m_invMass;
	m_iA = m_bodyA->m_invI;
	m_iB = m_bodyB->m_invI;
	m_iC = m_bodyC->m_invI;
	m_iD = m_bodyD->m_invI;

	const auto aA = data.positions[m_indexA].a;
	auto vA = data.velocities[m_indexA].v;
	auto wA = data.velocities[m_indexA].w;

	const auto aB = data.positions[m_indexB].a;
	auto vB = data.velocities[m_indexB].v;
	auto wB = data.velocities[m_indexB].w;

	const auto aC = data.positions[m_indexC].a;
	auto vC = data.velocities[m_indexC].v;
	auto wC = data.velocities[m_indexC].w;

	const auto aD = data.positions[m_indexD].a;
	auto vD = data.velocities[m_indexD].v;
	auto wD = data.velocities[m_indexD].w;

	const auto qA = b2Rot(aA);
	const auto qB = b2Rot(aB);
	const auto qC = b2Rot(aC);
	const auto qD = b2Rot(aD);

	m_mass = b2Float{0};

	if (m_typeA == e_revoluteJoint)
	{
		m_JvAC = b2Vec2_zero;
		m_JwA = b2Float(1);
		m_JwC = b2Float(1);
		m_mass += m_iA + m_iC;
	}
	else
	{
		const auto u = b2Mul(qC, m_localAxisC);
		const auto rC = b2Mul(qC, m_localAnchorC - m_lcC);
		const auto rA = b2Mul(qA, m_localAnchorA - m_lcA);
		m_JvAC = u;
		m_JwC = b2Cross(rC, u);
		m_JwA = b2Cross(rA, u);
		m_mass += m_mC + m_mA + m_iC * b2Square(m_JwC) + m_iA * b2Square(m_JwA);
	}

	if (m_typeB == e_revoluteJoint)
	{
		m_JvBD = b2Vec2_zero;
		m_JwB = m_ratio;
		m_JwD = m_ratio;
		m_mass += b2Square(m_ratio) * (m_iB + m_iD);
	}
	else
	{
		b2Vec2 u = b2Mul(qD, m_localAxisD);
		b2Vec2 rD = b2Mul(qD, m_localAnchorD - m_lcD);
		b2Vec2 rB = b2Mul(qB, m_localAnchorB - m_lcB);
		m_JvBD = m_ratio * u;
		m_JwD = m_ratio * b2Cross(rD, u);
		m_JwB = m_ratio * b2Cross(rB, u);
		m_mass += b2Square(m_ratio) * (m_mD + m_mB) + m_iD * b2Square(m_JwD) + m_iB * b2Square(m_JwB);
	}

	// Compute effective mass.
	m_mass = (m_mass > b2Float{0}) ? b2Float(1) / m_mass : b2Float{0};

	if (data.step.warmStarting)
	{
		vA += (m_mA * m_impulse) * m_JvAC;
		wA += m_iA * m_impulse * m_JwA;
		vB += (m_mB * m_impulse) * m_JvBD;
		wB += m_iB * m_impulse * m_JwB;
		vC -= (m_mC * m_impulse) * m_JvAC;
		wC -= m_iC * m_impulse * m_JwC;
		vD -= (m_mD * m_impulse) * m_JvBD;
		wD -= m_iD * m_impulse * m_JwD;
	}
	else
	{
		m_impulse = b2Float{0};
	}

	data.velocities[m_indexA].v = vA;
	data.velocities[m_indexA].w = wA;
	data.velocities[m_indexB].v = vB;
	data.velocities[m_indexB].w = wB;
	data.velocities[m_indexC].v = vC;
	data.velocities[m_indexC].w = wC;
	data.velocities[m_indexD].v = vD;
	data.velocities[m_indexD].w = wD;
}

void b2GearJoint::SolveVelocityConstraints(const b2SolverData& data)
{
	auto vA = data.velocities[m_indexA].v;
	auto wA = data.velocities[m_indexA].w;
	auto vB = data.velocities[m_indexB].v;
	auto wB = data.velocities[m_indexB].w;
	auto vC = data.velocities[m_indexC].v;
	auto wC = data.velocities[m_indexC].w;
	auto vD = data.velocities[m_indexD].v;
	auto wD = data.velocities[m_indexD].w;

	auto Cdot = b2Dot(m_JvAC, vA - vC) + b2Dot(m_JvBD, vB - vD);
	Cdot += (m_JwA * wA - m_JwC * wC) + (m_JwB * wB - m_JwD * wD);

	const auto impulse = -m_mass * Cdot;
	m_impulse += impulse;

	vA += (m_mA * impulse) * m_JvAC;
	wA += m_iA * impulse * m_JwA;
	vB += (m_mB * impulse) * m_JvBD;
	wB += m_iB * impulse * m_JwB;
	vC -= (m_mC * impulse) * m_JvAC;
	wC -= m_iC * impulse * m_JwC;
	vD -= (m_mD * impulse) * m_JvBD;
	wD -= m_iD * impulse * m_JwD;

	data.velocities[m_indexA].v = vA;
	data.velocities[m_indexA].w = wA;
	data.velocities[m_indexB].v = vB;
	data.velocities[m_indexB].w = wB;
	data.velocities[m_indexC].v = vC;
	data.velocities[m_indexC].w = wC;
	data.velocities[m_indexD].v = vD;
	data.velocities[m_indexD].w = wD;
}

bool b2GearJoint::SolvePositionConstraints(const b2SolverData& data)
{
	auto cA = data.positions[m_indexA].c;
	auto aA = data.positions[m_indexA].a;
	auto cB = data.positions[m_indexB].c;
	auto aB = data.positions[m_indexB].a;
	auto cC = data.positions[m_indexC].c;
	auto aC = data.positions[m_indexC].a;
	auto cD = data.positions[m_indexD].c;
	auto aD = data.positions[m_indexD].a;

	const b2Rot qA(aA), qB(aB), qC(aC), qD(aD);

	const auto linearError = b2Float{0};

	b2Float coordinateA, coordinateB;

	b2Vec2 JvAC, JvBD;
	b2Float JwA, JwB, JwC, JwD;
	auto mass = b2Float{0};

	if (m_typeA == e_revoluteJoint)
	{
		JvAC = b2Vec2_zero;
		JwA = b2Float(1);
		JwC = b2Float(1);
		mass += m_iA + m_iC;

		coordinateA = aA - aC - m_referenceAngleA;
	}
	else
	{
		const auto u = b2Mul(qC, m_localAxisC);
		const auto rC = b2Mul(qC, m_localAnchorC - m_lcC);
		const auto rA = b2Mul(qA, m_localAnchorA - m_lcA);
		JvAC = u;
		JwC = b2Cross(rC, u);
		JwA = b2Cross(rA, u);
		mass += m_mC + m_mA + m_iC * b2Square(JwC) + m_iA * b2Square(JwA);

		const auto pC = m_localAnchorC - m_lcC;
		const auto pA = b2MulT(qC, rA + (cA - cC));
		coordinateA = b2Dot(pA - pC, m_localAxisC);
	}

	if (m_typeB == e_revoluteJoint)
	{
		JvBD = b2Vec2_zero;
		JwB = m_ratio;
		JwD = m_ratio;
		mass += b2Square(m_ratio) * (m_iB + m_iD);

		coordinateB = aB - aD - m_referenceAngleB;
	}
	else
	{
		const auto u = b2Mul(qD, m_localAxisD);
		const auto rD = b2Mul(qD, m_localAnchorD - m_lcD);
		const auto rB = b2Mul(qB, m_localAnchorB - m_lcB);
		JvBD = m_ratio * u;
		JwD = m_ratio * b2Cross(rD, u);
		JwB = m_ratio * b2Cross(rB, u);
		mass += b2Square(m_ratio) * (m_mD + m_mB) + m_iD * b2Square(JwD) + m_iB * b2Square(JwB);

		const auto pD = m_localAnchorD - m_lcD;
		const auto pB = b2MulT(qD, rB + (cB - cD));
		coordinateB = b2Dot(pB - pD, m_localAxisD);
	}

	const auto C = (coordinateA + m_ratio * coordinateB) - m_constant;

	auto impulse = b2Float{0};
	if (mass > b2Float{0})
	{
		impulse = -C / mass;
	}

	cA += m_mA * impulse * JvAC;
	aA += m_iA * impulse * JwA;
	cB += m_mB * impulse * JvBD;
	aB += m_iB * impulse * JwB;
	cC -= m_mC * impulse * JvAC;
	aC -= m_iC * impulse * JwC;
	cD -= m_mD * impulse * JvBD;
	aD -= m_iD * impulse * JwD;

	data.positions[m_indexA].c = cA;
	data.positions[m_indexA].a = aA;
	data.positions[m_indexB].c = cB;
	data.positions[m_indexB].a = aB;
	data.positions[m_indexC].c = cC;
	data.positions[m_indexC].a = aC;
	data.positions[m_indexD].c = cD;
	data.positions[m_indexD].a = aD;

	// TODO_ERIN not implemented
	return linearError < b2_linearSlop;
}

b2Vec2 b2GearJoint::GetAnchorA() const
{
	return m_bodyA->GetWorldPoint(m_localAnchorA);
}

b2Vec2 b2GearJoint::GetAnchorB() const
{
	return m_bodyB->GetWorldPoint(m_localAnchorB);
}

b2Vec2 b2GearJoint::GetReactionForce(b2Float inv_dt) const
{
	return inv_dt * m_impulse * m_JvAC;
}

b2Float b2GearJoint::GetReactionTorque(b2Float inv_dt) const
{
	return inv_dt * m_impulse * m_JwA;
}

void b2GearJoint::SetRatio(b2Float ratio)
{
	b2Assert(b2IsValid(ratio));
	m_ratio = ratio;
}

b2Float b2GearJoint::GetRatio() const
{
	return m_ratio;
}

void b2GearJoint::Dump()
{
	const auto indexA = m_bodyA->m_islandIndex;
	const auto indexB = m_bodyB->m_islandIndex;

	const auto index1 = m_joint1->m_index;
	const auto index2 = m_joint2->m_index;

	b2Log("  b2GearJointDef jd;\n");
	b2Log("  jd.bodyA = bodies[%d];\n", indexA);
	b2Log("  jd.bodyB = bodies[%d];\n", indexB);
	b2Log("  jd.collideConnected = bool(%d);\n", m_collideConnected);
	b2Log("  jd.joint1 = joints[%d];\n", index1);
	b2Log("  jd.joint2 = joints[%d];\n", index2);
	b2Log("  jd.ratio = %.15lef;\n", m_ratio);
	b2Log("  joints[%d] = m_world->CreateJoint(&jd);\n", m_index);
}
