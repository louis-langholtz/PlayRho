/*
* Copyright (c) 2007 Erin Catto http://www.box2d.org
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

#include <Box2D/Dynamics/Joints/b2PulleyJoint.h>
#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Dynamics/b2TimeStep.h>

// Pulley:
// length1 = norm(p1 - s1)
// length2 = norm(p2 - s2)
// C0 = (length1 + ratio * length2)_initial
// C = C0 - (length1 + ratio * length2)
// u1 = (p1 - s1) / norm(p1 - s1)
// u2 = (p2 - s2) / norm(p2 - s2)
// Cdot = -dot(u1, v1 + cross(w1, r1)) - ratio * dot(u2, v2 + cross(w2, r2))
// J = -[u1 cross(r1, u1) ratio * u2  ratio * cross(r2, u2)]
// K = J * invM * JT
//   = invMass1 + invI1 * cross(r1, u1)^2 + ratio^2 * (invMass2 + invI2 * cross(r2, u2)^2)

void b2PulleyJointDef::Initialize(b2Body* bA, b2Body* bB,
				const b2Vec2& groundA, const b2Vec2& groundB,
				const b2Vec2& anchorA, const b2Vec2& anchorB,
				b2Float r)
{
	bodyA = bA;
	bodyB = bB;
	groundAnchorA = groundA;
	groundAnchorB = groundB;
	localAnchorA = bodyA->GetLocalPoint(anchorA);
	localAnchorB = bodyB->GetLocalPoint(anchorB);
	b2Vec2 dA = anchorA - groundA;
	lengthA = dA.Length();
	b2Vec2 dB = anchorB - groundB;
	lengthB = dB.Length();
	ratio = r;
	b2Assert(ratio > b2_epsilon);
}

b2PulleyJoint::b2PulleyJoint(const b2PulleyJointDef* def)
: b2Joint(def)
{
	m_groundAnchorA = def->groundAnchorA;
	m_groundAnchorB = def->groundAnchorB;
	m_localAnchorA = def->localAnchorA;
	m_localAnchorB = def->localAnchorB;

	m_lengthA = def->lengthA;
	m_lengthB = def->lengthB;

	b2Assert(def->ratio != b2Float{0});
	m_ratio = def->ratio;

	m_constant = def->lengthA + m_ratio * def->lengthB;

	m_impulse = b2Float{0};
}

void b2PulleyJoint::InitVelocityConstraints(const b2SolverData& data)
{
	m_indexA = m_bodyA->m_islandIndex;
	m_indexB = m_bodyB->m_islandIndex;
	m_localCenterA = m_bodyA->m_sweep.localCenter;
	m_localCenterB = m_bodyB->m_sweep.localCenter;
	m_invMassA = m_bodyA->m_invMass;
	m_invMassB = m_bodyB->m_invMass;
	m_invIA = m_bodyA->m_invI;
	m_invIB = m_bodyB->m_invI;

	const auto cA = data.positions[m_indexA].c;
	const auto aA = data.positions[m_indexA].a;
	auto vA = data.velocities[m_indexA].v;
	auto wA = data.velocities[m_indexA].w;

	const auto cB = data.positions[m_indexB].c;
	const auto aB = data.positions[m_indexB].a;
	auto vB = data.velocities[m_indexB].v;
	auto wB = data.velocities[m_indexB].w;

	const b2Rot qA(aA), qB(aB);

	m_rA = b2Mul(qA, m_localAnchorA - m_localCenterA);
	m_rB = b2Mul(qB, m_localAnchorB - m_localCenterB);

	// Get the pulley axes.
	m_uA = cA + m_rA - m_groundAnchorA;
	m_uB = cB + m_rB - m_groundAnchorB;

	const auto lengthA = m_uA.Length();
	const auto lengthB = m_uB.Length();

	if (lengthA > (b2Float(10) * b2_linearSlop))
	{
		m_uA *= b2Float(1) / lengthA;
	}
	else
	{
		m_uA = b2Vec2_zero;
	}

	if (lengthB > (b2Float(10) * b2_linearSlop))
	{
		m_uB *= b2Float(1) / lengthB;
	}
	else
	{
		m_uB = b2Vec2_zero;
	}

	// Compute effective mass.
	const auto ruA = b2Cross(m_rA, m_uA);
	const auto ruB = b2Cross(m_rB, m_uB);

	const auto mA = m_invMassA + m_invIA * ruA * ruA;
	const auto mB = m_invMassB + m_invIB * ruB * ruB;

	m_mass = mA + m_ratio * m_ratio * mB;

	if (m_mass > b2Float{0})
	{
		m_mass = b2Float(1) / m_mass;
	}

	if (data.step.warmStarting)
	{
		// Scale impulses to support variable time steps.
		m_impulse *= data.step.dtRatio;

		// Warm starting.
		const auto PA = -(m_impulse) * m_uA;
		const auto PB = (-m_ratio * m_impulse) * m_uB;

		vA += m_invMassA * PA;
		wA += m_invIA * b2Cross(m_rA, PA);
		vB += m_invMassB * PB;
		wB += m_invIB * b2Cross(m_rB, PB);
	}
	else
	{
		m_impulse = b2Float{0};
	}

	data.velocities[m_indexA].v = vA;
	data.velocities[m_indexA].w = wA;
	data.velocities[m_indexB].v = vB;
	data.velocities[m_indexB].w = wB;
}

void b2PulleyJoint::SolveVelocityConstraints(const b2SolverData& data)
{
	auto vA = data.velocities[m_indexA].v;
	auto wA = data.velocities[m_indexA].w;
	auto vB = data.velocities[m_indexB].v;
	auto wB = data.velocities[m_indexB].w;

	const auto vpA = vA + b2Cross(wA, m_rA);
	const auto vpB = vB + b2Cross(wB, m_rB);

	const auto Cdot = -b2Dot(m_uA, vpA) - m_ratio * b2Dot(m_uB, vpB);
	const auto impulse = -m_mass * Cdot;
	m_impulse += impulse;

	const auto PA = -impulse * m_uA;
	const auto PB = -m_ratio * impulse * m_uB;
	vA += m_invMassA * PA;
	wA += m_invIA * b2Cross(m_rA, PA);
	vB += m_invMassB * PB;
	wB += m_invIB * b2Cross(m_rB, PB);

	data.velocities[m_indexA].v = vA;
	data.velocities[m_indexA].w = wA;
	data.velocities[m_indexB].v = vB;
	data.velocities[m_indexB].w = wB;
}

bool b2PulleyJoint::SolvePositionConstraints(const b2SolverData& data)
{
	auto cA = data.positions[m_indexA].c;
	auto aA = data.positions[m_indexA].a;
	auto cB = data.positions[m_indexB].c;
	auto aB = data.positions[m_indexB].a;

	const b2Rot qA(aA), qB(aB);

	const auto rA = b2Mul(qA, m_localAnchorA - m_localCenterA);
	const auto rB = b2Mul(qB, m_localAnchorB - m_localCenterB);

	// Get the pulley axes.
	auto uA = cA + rA - m_groundAnchorA;
	auto uB = cB + rB - m_groundAnchorB;

	const auto lengthA = uA.Length();
	const auto lengthB = uB.Length();

	if (lengthA > (b2Float(10) * b2_linearSlop))
	{
		uA *= b2Float(1) / lengthA;
	}
	else
	{
		uA = b2Vec2_zero;
	}

	if (lengthB > (b2Float(10) * b2_linearSlop))
	{
		uB *= b2Float(1) / lengthB;
	}
	else
	{
		uB = b2Vec2_zero;
	}

	// Compute effective mass.
	const auto ruA = b2Cross(rA, uA);
	const auto ruB = b2Cross(rB, uB);

	const auto mA = m_invMassA + m_invIA * ruA * ruA;
	const auto mB = m_invMassB + m_invIB * ruB * ruB;

	auto mass = mA + m_ratio * m_ratio * mB;

	if (mass > b2Float{0})
	{
		mass = b2Float(1) / mass;
	}

	const auto C = m_constant - lengthA - (m_ratio * lengthB);
	const auto linearError = b2Abs(C);

	const auto impulse = -mass * C;

	const auto PA = -impulse * uA;
	const auto PB = -m_ratio * impulse * uB;

	cA += m_invMassA * PA;
	aA += m_invIA * b2Cross(rA, PA);
	cB += m_invMassB * PB;
	aB += m_invIB * b2Cross(rB, PB);

	data.positions[m_indexA].c = cA;
	data.positions[m_indexA].a = aA;
	data.positions[m_indexB].c = cB;
	data.positions[m_indexB].a = aB;

	return linearError < b2_linearSlop;
}

b2Vec2 b2PulleyJoint::GetAnchorA() const
{
	return m_bodyA->GetWorldPoint(m_localAnchorA);
}

b2Vec2 b2PulleyJoint::GetAnchorB() const
{
	return m_bodyB->GetWorldPoint(m_localAnchorB);
}

b2Vec2 b2PulleyJoint::GetReactionForce(b2Float inv_dt) const
{
	return inv_dt * m_impulse * m_uB;
}

b2Float b2PulleyJoint::GetReactionTorque(b2Float inv_dt) const
{
	B2_NOT_USED(inv_dt);
	return b2Float{0};
}

b2Vec2 b2PulleyJoint::GetGroundAnchorA() const
{
	return m_groundAnchorA;
}

b2Vec2 b2PulleyJoint::GetGroundAnchorB() const
{
	return m_groundAnchorB;
}

b2Float b2PulleyJoint::GetLengthA() const
{
	return m_lengthA;
}

b2Float b2PulleyJoint::GetLengthB() const
{
	return m_lengthB;
}

b2Float b2PulleyJoint::GetRatio() const
{
	return m_ratio;
}

b2Float b2PulleyJoint::GetCurrentLengthA() const
{
	b2Vec2 p = m_bodyA->GetWorldPoint(m_localAnchorA);
	b2Vec2 s = m_groundAnchorA;
	b2Vec2 d = p - s;
	return d.Length();
}

b2Float b2PulleyJoint::GetCurrentLengthB() const
{
	const auto p = m_bodyB->GetWorldPoint(m_localAnchorB);
	const auto s = m_groundAnchorB;
	const auto d = p - s;
	return d.Length();
}

void b2PulleyJoint::Dump()
{
	const auto indexA = m_bodyA->m_islandIndex;
	const auto indexB = m_bodyB->m_islandIndex;

	b2Log("  b2PulleyJointDef jd;\n");
	b2Log("  jd.bodyA = bodies[%d];\n", indexA);
	b2Log("  jd.bodyB = bodies[%d];\n", indexB);
	b2Log("  jd.collideConnected = bool(%d);\n", m_collideConnected);
	b2Log("  jd.groundAnchorA = b2Vec2(%.15lef, %.15lef);\n", m_groundAnchorA.x, m_groundAnchorA.y);
	b2Log("  jd.groundAnchorB = b2Vec2(%.15lef, %.15lef);\n", m_groundAnchorB.x, m_groundAnchorB.y);
	b2Log("  jd.localAnchorA = b2Vec2(%.15lef, %.15lef);\n", m_localAnchorA.x, m_localAnchorA.y);
	b2Log("  jd.localAnchorB = b2Vec2(%.15lef, %.15lef);\n", m_localAnchorB.x, m_localAnchorB.y);
	b2Log("  jd.lengthA = %.15lef;\n", m_lengthA);
	b2Log("  jd.lengthB = %.15lef;\n", m_lengthB);
	b2Log("  jd.ratio = %.15lef;\n", m_ratio);
	b2Log("  joints[%d] = m_world->CreateJoint(&jd);\n", m_index);
}

void b2PulleyJoint::ShiftOrigin(const b2Vec2& newOrigin)
{
	m_groundAnchorA -= newOrigin;
	m_groundAnchorB -= newOrigin;
}
