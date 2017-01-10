/*
* Original work Copyright (c) 2007-2011 Erin Catto http://www.box2d.org
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

#include <Box2D/Dynamics/Joints/RopeJoint.hpp>
#include <Box2D/Dynamics/Body.hpp>
#include <Box2D/Dynamics/TimeStep.hpp>
#include <Box2D/Dynamics/Contacts/ContactSolver.hpp>

using namespace box2d;

// Limit:
// C = norm(pB - pA) - L
// u = (pB - pA) / norm(pB - pA)
// Cdot = dot(u, vB + cross(wB, rB) - vA - cross(wA, rA))
// J = [-u -cross(rA, u) u cross(rB, u)]
// K = J * invM * JT
//   = invMassA + invIA * cross(rA, u)^2 + invMassB + invIB * cross(rB, u)^2

RopeJoint::RopeJoint(const RopeJointDef& def)
: Joint(def)
{
	m_localAnchorA = def.localAnchorA;
	m_localAnchorB = def.localAnchorB;

	m_maxLength = def.maxLength;

	m_mass = float_t{0};
	m_impulse = float_t{0};
	m_state = e_inactiveLimit;
	m_length = float_t{0};
}

void RopeJoint::InitVelocityConstraints(Span<Velocity> velocities,
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
	m_u = cB + m_rB - cA - m_rA;

	m_length = GetLength(m_u);

	const auto C = m_length - m_maxLength;
	m_state = (C > float_t{0})? e_atUpperLimit: e_inactiveLimit;

	if (m_length > conf.linearSlop)
	{
		m_u *= float_t{1} / m_length;
	}
	else
	{
		m_u = Vec2_zero;
		m_mass = float_t{0};
		m_impulse = float_t{0};
		return;
	}

	// Compute effective mass.
	const auto crA = Cross(m_rA, m_u);
	const auto crB = Cross(m_rB, m_u);
	const auto invMass = m_invMassA + m_invIA * crA * crA + m_invMassB + m_invIB * crB * crB;

	m_mass = (invMass != float_t{0}) ? float_t{1} / invMass : float_t{0};

	if (step.doWarmStart)
	{
		// Scale the impulse to support a variable time step.
		m_impulse *= step.dtRatio;

		const auto P = m_impulse * m_u;
		vA -= m_invMassA * P;
		wA -= 1_rad * m_invIA * Cross(m_rA, P);
		vB += m_invMassB * P;
		wB += 1_rad * m_invIB * Cross(m_rB, P);
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

void RopeJoint::SolveVelocityConstraints(Span<Velocity> velocities, const TimeStep& step)
{
	auto vA = velocities[m_indexA].v;
	auto wA = velocities[m_indexA].w;
	auto vB = velocities[m_indexB].v;
	auto wB = velocities[m_indexB].w;

	// Cdot = dot(u, v + cross(w, r))
	const auto vpA = vA + GetRevPerpendicular(m_rA) * wA.ToRadians();
	const auto vpB = vB + GetRevPerpendicular(m_rB) * wB.ToRadians();
	const auto C = m_length - m_maxLength;
	auto Cdot = Dot(m_u, vpB - vpA);

	// Predictive constraint.
	if (C < float_t{0})
	{
		Cdot += step.get_inv_dt() * C;
	}

	auto impulse = -m_mass * Cdot;
	const auto oldImpulse = m_impulse;
	m_impulse = Min(float_t{0}, m_impulse + impulse);
	impulse = m_impulse - oldImpulse;

	const auto P = impulse * m_u;
	vA -= m_invMassA * P;
	wA -= 1_rad * m_invIA * Cross(m_rA, P);
	vB += m_invMassB * P;
	wB += 1_rad * m_invIB * Cross(m_rB, P);

	velocities[m_indexA].v = vA;
	velocities[m_indexA].w = wA;
	velocities[m_indexB].v = vB;
	velocities[m_indexB].w = wB;
}

bool RopeJoint::SolvePositionConstraints(Span<Position> positions, const ConstraintSolverConf& conf)
{
	auto cA = positions[m_indexA].c;
	auto aA = positions[m_indexA].a;
	auto cB = positions[m_indexB].c;
	auto aB = positions[m_indexB].a;

	const UnitVec2 qA(aA), qB(aB);

	const auto rA = Rotate(m_localAnchorA - m_localCenterA, qA);
	const auto rB = Rotate(m_localAnchorB - m_localCenterB, qB);
	auto u = cB + rB - cA - rA;

	const auto length = Normalize(u);
	auto C = length - m_maxLength;

	C = Clamp(C, float_t{0}, conf.maxLinearCorrection);

	const auto impulse = -m_mass * C;
	const auto P = impulse * u;

	cA -= m_invMassA * P;
	aA -= 1_rad * m_invIA * Cross(rA, P);
	cB += m_invMassB * P;
	aB += 1_rad * m_invIB * Cross(rB, P);

	positions[m_indexA].c = cA;
	positions[m_indexA].a = aA;
	positions[m_indexB].c = cB;
	positions[m_indexB].a = aB;

	return (length - m_maxLength) < conf.linearSlop;
}

Vec2 RopeJoint::GetAnchorA() const
{
	return GetWorldPoint(*GetBodyA(), m_localAnchorA);
}

Vec2 RopeJoint::GetAnchorB() const
{
	return GetWorldPoint(*GetBodyB(), m_localAnchorB);
}

Vec2 RopeJoint::GetReactionForce(float_t inv_dt) const
{
	return (inv_dt * m_impulse) * m_u;
}

float_t RopeJoint::GetReactionTorque(float_t inv_dt) const
{
	BOX2D_NOT_USED(inv_dt);
	return float_t{0};
}

float_t RopeJoint::GetMaxLength() const
{
	return m_maxLength;
}

Joint::LimitState RopeJoint::GetLimitState() const
{
	return m_state;
}
