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

#include <Box2D/Dynamics/Joints/b2RopeJoint.h>
#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Dynamics/b2TimeStep.h>

using namespace box2d;

// Limit:
// C = norm(pB - pA) - L
// u = (pB - pA) / norm(pB - pA)
// Cdot = dot(u, vB + cross(wB, rB) - vA - cross(wA, rA))
// J = [-u -cross(rA, u) u cross(rB, u)]
// K = J * invM * JT
//   = invMassA + invIA * cross(rA, u)^2 + invMassB + invIB * cross(rB, u)^2

b2RopeJoint::b2RopeJoint(const b2RopeJointDef* def)
: b2Joint(def)
{
	m_localAnchorA = def->localAnchorA;
	m_localAnchorB = def->localAnchorB;

	m_maxLength = def->maxLength;

	m_mass = float_t{0};
	m_impulse = float_t{0};
	m_state = e_inactiveLimit;
	m_length = float_t{0};
}

void b2RopeJoint::InitVelocityConstraints(const b2SolverData& data)
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
	m_u = cB + m_rB - cA - m_rA;

	m_length = m_u.Length();

	const auto C = m_length - m_maxLength;
	if (C > float_t{0})
	{
		m_state = e_atUpperLimit;
	}
	else
	{
		m_state = e_inactiveLimit;
	}

	if (m_length > b2_linearSlop)
	{
		m_u *= float_t(1) / m_length;
	}
	else
	{
		m_u = b2Vec2_zero;
		m_mass = float_t{0};
		m_impulse = float_t{0};
		return;
	}

	// Compute effective mass.
	const auto crA = b2Cross(m_rA, m_u);
	const auto crB = b2Cross(m_rB, m_u);
	const auto invMass = m_invMassA + m_invIA * crA * crA + m_invMassB + m_invIB * crB * crB;

	m_mass = (invMass != float_t{0}) ? float_t(1) / invMass : float_t{0};

	if (data.step.warmStarting)
	{
		// Scale the impulse to support a variable time step.
		m_impulse *= data.step.dtRatio;

		const auto P = m_impulse * m_u;
		vA -= m_invMassA * P;
		wA -= m_invIA * b2Cross(m_rA, P);
		vB += m_invMassB * P;
		wB += m_invIB * b2Cross(m_rB, P);
	}
	else
	{
		m_impulse = float_t{0};
	}

	data.velocities[m_indexA].v = vA;
	data.velocities[m_indexA].w = wA;
	data.velocities[m_indexB].v = vB;
	data.velocities[m_indexB].w = wB;
}

void b2RopeJoint::SolveVelocityConstraints(const b2SolverData& data)
{
	auto vA = data.velocities[m_indexA].v;
	auto wA = data.velocities[m_indexA].w;
	auto vB = data.velocities[m_indexB].v;
	auto wB = data.velocities[m_indexB].w;

	// Cdot = dot(u, v + cross(w, r))
	const auto vpA = vA + b2Cross(wA, m_rA);
	const auto vpB = vB + b2Cross(wB, m_rB);
	const auto C = m_length - m_maxLength;
	auto Cdot = b2Dot(m_u, vpB - vpA);

	// Predictive constraint.
	if (C < float_t{0})
	{
		Cdot += data.step.get_inv_dt() * C;
	}

	auto impulse = -m_mass * Cdot;
	const auto oldImpulse = m_impulse;
	m_impulse = b2Min(float_t{0}, m_impulse + impulse);
	impulse = m_impulse - oldImpulse;

	const auto P = impulse * m_u;
	vA -= m_invMassA * P;
	wA -= m_invIA * b2Cross(m_rA, P);
	vB += m_invMassB * P;
	wB += m_invIB * b2Cross(m_rB, P);

	data.velocities[m_indexA].v = vA;
	data.velocities[m_indexA].w = wA;
	data.velocities[m_indexB].v = vB;
	data.velocities[m_indexB].w = wB;
}

bool b2RopeJoint::SolvePositionConstraints(const b2SolverData& data)
{
	auto cA = data.positions[m_indexA].c;
	auto aA = data.positions[m_indexA].a;
	auto cB = data.positions[m_indexB].c;
	auto aB = data.positions[m_indexB].a;

	const b2Rot qA(aA), qB(aB);

	const auto rA = b2Mul(qA, m_localAnchorA - m_localCenterA);
	const auto rB = b2Mul(qB, m_localAnchorB - m_localCenterB);
	auto u = cB + rB - cA - rA;

	const auto length = u.Normalize();
	auto C = length - m_maxLength;

	C = b2Clamp(C, float_t{0}, b2_maxLinearCorrection);

	const auto impulse = -m_mass * C;
	const auto P = impulse * u;

	cA -= m_invMassA * P;
	aA -= m_invIA * b2Cross(rA, P);
	cB += m_invMassB * P;
	aB += m_invIB * b2Cross(rB, P);

	data.positions[m_indexA].c = cA;
	data.positions[m_indexA].a = aA;
	data.positions[m_indexB].c = cB;
	data.positions[m_indexB].a = aB;

	return (length - m_maxLength) < b2_linearSlop;
}

b2Vec2 b2RopeJoint::GetAnchorA() const
{
	return m_bodyA->GetWorldPoint(m_localAnchorA);
}

b2Vec2 b2RopeJoint::GetAnchorB() const
{
	return m_bodyB->GetWorldPoint(m_localAnchorB);
}

b2Vec2 b2RopeJoint::GetReactionForce(float_t inv_dt) const
{
	b2Vec2 F = (inv_dt * m_impulse) * m_u;
	return F;
}

float_t b2RopeJoint::GetReactionTorque(float_t inv_dt) const
{
	BOX2D_NOT_USED(inv_dt);
	return float_t{0};
}

float_t b2RopeJoint::GetMaxLength() const
{
	return m_maxLength;
}

b2LimitState b2RopeJoint::GetLimitState() const
{
	return m_state;
}

void b2RopeJoint::Dump()
{
	const auto indexA = m_bodyA->m_islandIndex;
	const auto indexB = m_bodyB->m_islandIndex;

	b2Log("  b2RopeJointDef jd;\n");
	b2Log("  jd.bodyA = bodies[%d];\n", indexA);
	b2Log("  jd.bodyB = bodies[%d];\n", indexB);
	b2Log("  jd.collideConnected = bool(%d);\n", m_collideConnected);
	b2Log("  jd.localAnchorA = b2Vec2(%.15lef, %.15lef);\n", m_localAnchorA.x, m_localAnchorA.y);
	b2Log("  jd.localAnchorB = b2Vec2(%.15lef, %.15lef);\n", m_localAnchorB.x, m_localAnchorB.y);
	b2Log("  jd.maxLength = %.15lef;\n", m_maxLength);
	b2Log("  joints[%d] = m_world->CreateJoint(&jd);\n", m_index);
}
