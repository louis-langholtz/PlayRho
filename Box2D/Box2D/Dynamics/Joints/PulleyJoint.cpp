/*
* Original work Copyright (c) 2007 Erin Catto http://www.box2d.org
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

#include <Box2D/Dynamics/Joints/PulleyJoint.h>
#include <Box2D/Dynamics/Body.hpp>
#include <Box2D/Dynamics/TimeStep.hpp>
#include <Box2D/Dynamics/Contacts/ContactSolver.hpp>

using namespace box2d;

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

void PulleyJointDef::Initialize(Body* bA, Body* bB,
				const Vec2& groundA, const Vec2& groundB,
				const Vec2& anchorA, const Vec2& anchorB,
				float_t r)
{
	assert((r > 0) && !almost_zero(r));
	
	bodyA = bA;
	bodyB = bB;
	groundAnchorA = groundA;
	groundAnchorB = groundB;
	localAnchorA = GetLocalPoint(*bodyA, anchorA);
	localAnchorB = GetLocalPoint(*bodyB, anchorB);
	lengthA = GetLength(anchorA - groundA);
	lengthB = GetLength(anchorB - groundB);
	ratio = r;
}

PulleyJoint::PulleyJoint(const PulleyJointDef& def)
: Joint(def)
{
	assert(!almost_zero(def.ratio));

	m_groundAnchorA = def.groundAnchorA;
	m_groundAnchorB = def.groundAnchorB;
	m_localAnchorA = def.localAnchorA;
	m_localAnchorB = def.localAnchorB;
	m_lengthA = def.lengthA;
	m_lengthB = def.lengthB;
	m_ratio = def.ratio;
	m_constant = def.lengthA + m_ratio * def.lengthB;
	m_impulse = float_t{0};
}

void PulleyJoint::InitVelocityConstraints(Span<Velocity> velocities,
										  Span<const Position> positions,
										  const TimeStep& step,
										  const ConstraintSolverConf& conf)
{
	m_indexA = GetBodyA()->GetIslandIndex();
	m_indexB = GetBodyB()->GetIslandIndex();
	m_localCenterA = GetBodyA()->GetLocalCenter();
	m_localCenterB = GetBodyB()->GetLocalCenter();
	m_invMassA = GetBodyA()->GetInverseMass();
	m_invMassB = GetBodyB()->GetInverseMass();
	m_invIA = GetBodyA()->GetInverseInertia();
	m_invIB = GetBodyB()->GetInverseInertia();

	const auto cA = positions[m_indexA].c;
	const auto aA = positions[m_indexA].a;
	auto vA = velocities[m_indexA].v;
	auto wA = velocities[m_indexA].w;

	const auto cB = positions[m_indexB].c;
	const auto aB = positions[m_indexB].a;
	auto vB = velocities[m_indexB].v;
	auto wB = velocities[m_indexB].w;

	const UnitVec2 qA(aA), qB(aB);

	m_rA = Rotate(m_localAnchorA - m_localCenterA, qA);
	m_rB = Rotate(m_localAnchorB - m_localCenterB, qB);

	// Get the pulley axes.
	m_uA = cA + m_rA - m_groundAnchorA;
	m_uB = cB + m_rB - m_groundAnchorB;

	const auto lengthA = GetLength(m_uA);
	const auto lengthB = GetLength(m_uB);

	if (lengthA > (float_t(10) * conf.linearSlop))
	{
		m_uA *= float_t{1} / lengthA;
	}
	else
	{
		m_uA = Vec2_zero;
	}

	if (lengthB > (float_t(10) * conf.linearSlop))
	{
		m_uB *= float_t{1} / lengthB;
	}
	else
	{
		m_uB = Vec2_zero;
	}

	// Compute effective mass.
	const auto ruA = Cross(m_rA, m_uA);
	const auto ruB = Cross(m_rB, m_uB);

	const auto mA = m_invMassA + m_invIA * ruA * ruA;
	const auto mB = m_invMassB + m_invIB * ruB * ruB;

	m_mass = mA + m_ratio * m_ratio * mB;

	if (m_mass > float_t{0})
	{
		m_mass = float_t{1} / m_mass;
	}

	if (step.warmStarting)
	{
		// Scale impulses to support variable time steps.
		m_impulse *= step.dtRatio;

		// Warm starting.
		const auto PA = -(m_impulse) * m_uA;
		const auto PB = (-m_ratio * m_impulse) * m_uB;

		vA += m_invMassA * PA;
		wA += 1_rad * m_invIA * Cross(m_rA, PA);
		vB += m_invMassB * PB;
		wB += 1_rad * m_invIB * Cross(m_rB, PB);
	}
	else
	{
		m_impulse = float_t{0};
	}

	velocities[m_indexA].v = vA;
	velocities[m_indexA].w = wA;
	velocities[m_indexB].v = vB;
	velocities[m_indexB].w = wB;
}

void PulleyJoint::SolveVelocityConstraints(Span<Velocity> velocities, const TimeStep& step)
{
	auto vA = velocities[m_indexA].v;
	auto wA = velocities[m_indexA].w;
	auto vB = velocities[m_indexB].v;
	auto wB = velocities[m_indexB].w;

	const auto vpA = vA + GetRevPerpendicular(m_rA) * wA.ToRadians();
	const auto vpB = vB + GetRevPerpendicular(m_rB) * wB.ToRadians();

	const auto Cdot = -Dot(m_uA, vpA) - m_ratio * Dot(m_uB, vpB);
	const auto impulse = -m_mass * Cdot;
	m_impulse += impulse;

	const auto PA = -impulse * m_uA;
	const auto PB = -m_ratio * impulse * m_uB;
	vA += m_invMassA * PA;
	wA += 1_rad * m_invIA * Cross(m_rA, PA);
	vB += m_invMassB * PB;
	wB += 1_rad * m_invIB * Cross(m_rB, PB);

	velocities[m_indexA].v = vA;
	velocities[m_indexA].w = wA;
	velocities[m_indexB].v = vB;
	velocities[m_indexB].w = wB;
}

bool PulleyJoint::SolvePositionConstraints(Span<Position> positions, const ConstraintSolverConf& conf)
{
	auto cA = positions[m_indexA].c;
	auto aA = positions[m_indexA].a;
	auto cB = positions[m_indexB].c;
	auto aB = positions[m_indexB].a;

	const auto rA = Rotate(m_localAnchorA - m_localCenterA, UnitVec2{aA});
	const auto rB = Rotate(m_localAnchorB - m_localCenterB, UnitVec2{aB});

	// Get the pulley axes.
	const auto pA = cA + rA - m_groundAnchorA;
	const auto lengthA = GetLength(pA);
	const auto uA = (lengthA > (float_t(10) * conf.linearSlop))? pA / lengthA: Vec2_zero;

	const auto pB = cB + rB - m_groundAnchorB;
	const auto lengthB = GetLength(pB);
	const auto uB = (lengthB > (float_t(10) * conf.linearSlop))? pB / lengthB: Vec2_zero;

	// Compute effective mass.
	const auto ruA = Cross(rA, uA);
	const auto ruB = Cross(rB, uB);

	const auto mA = m_invMassA + m_invIA * ruA * ruA;
	const auto mB = m_invMassB + m_invIB * ruB * ruB;

	auto mass = mA + m_ratio * m_ratio * mB;
	if (mass > float_t{0})
	{
		mass = float_t{1} / mass;
	}

	const auto C = m_constant - lengthA - (m_ratio * lengthB);
	const auto linearError = Abs(C);

	const auto impulse = -mass * C;

	const auto PA = -impulse * uA;
	const auto PB = -m_ratio * impulse * uB;

	cA += m_invMassA * PA;
	aA += 1_rad * m_invIA * Cross(rA, PA);
	cB += m_invMassB * PB;
	aB += 1_rad * m_invIB * Cross(rB, PB);

	positions[m_indexA].c = cA;
	positions[m_indexA].a = aA;
	positions[m_indexB].c = cB;
	positions[m_indexB].a = aB;

	return linearError < conf.linearSlop;
}

Vec2 PulleyJoint::GetAnchorA() const
{
	return GetWorldPoint(*GetBodyA(), GetLocalAnchorA());
}

Vec2 PulleyJoint::GetAnchorB() const
{
	return GetWorldPoint(*GetBodyB(), GetLocalAnchorB());
}

Vec2 PulleyJoint::GetReactionForce(float_t inv_dt) const
{
	return inv_dt * m_impulse * m_uB;
}

float_t PulleyJoint::GetReactionTorque(float_t inv_dt) const
{
	BOX2D_NOT_USED(inv_dt);
	return float_t{0};
}

Vec2 PulleyJoint::GetGroundAnchorA() const
{
	return m_groundAnchorA;
}

Vec2 PulleyJoint::GetGroundAnchorB() const
{
	return m_groundAnchorB;
}

float_t PulleyJoint::GetCurrentLengthA() const
{
	return GetLength(GetWorldPoint(*GetBodyA(), m_localAnchorA) - m_groundAnchorA);
}

float_t PulleyJoint::GetCurrentLengthB() const
{
	return GetLength(GetWorldPoint(*GetBodyB(), m_localAnchorB) - m_groundAnchorB);
}

void PulleyJoint::ShiftOrigin(const Vec2& newOrigin)
{
	m_groundAnchorA -= newOrigin;
	m_groundAnchorB -= newOrigin;
}

void box2d::Dump(const PulleyJoint& joint, size_t index)
{
	log("  PulleyJointDef jd;\n");
	log("  jd.bodyA = bodies[%d];\n", GetWorldIndex(joint.GetBodyA()));
	log("  jd.bodyB = bodies[%d];\n", GetWorldIndex(joint.GetBodyB()));
	log("  jd.collideConnected = bool(%d);\n", joint.GetCollideConnected());
	log("  jd.groundAnchorA = Vec2(%.15lef, %.15lef);\n", joint.GetGroundAnchorA().x, joint.GetGroundAnchorA().y);
	log("  jd.groundAnchorB = Vec2(%.15lef, %.15lef);\n", joint.GetGroundAnchorB().x, joint.GetGroundAnchorB().y);
	log("  jd.localAnchorA = Vec2(%.15lef, %.15lef);\n", joint.GetLocalAnchorA().x,  joint.GetLocalAnchorA().y);
	log("  jd.localAnchorB = Vec2(%.15lef, %.15lef);\n",  joint.GetLocalAnchorB().x,  joint.GetLocalAnchorB().y);
	log("  jd.lengthA = %.15lef;\n", joint.GetLengthA());
	log("  jd.lengthB = %.15lef;\n", joint.GetLengthB());
	log("  jd.ratio = %.15lef;\n", joint.GetRatio());
	log("  joints[%d] = m_world->CreateJoint(jd);\n", index);
}
