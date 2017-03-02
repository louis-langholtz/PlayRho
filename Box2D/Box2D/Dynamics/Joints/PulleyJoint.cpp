/*
 * Original work Copyright (c) 2007 Erin Catto http://www.box2d.org
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

#include <Box2D/Dynamics/Joints/PulleyJoint.hpp>
#include <Box2D/Dynamics/Body.hpp>
#include <Box2D/Dynamics/StepConf.hpp>
#include <Box2D/Dynamics/Contacts/ContactSolver.hpp>
#include <Box2D/Dynamics/Contacts/BodyConstraint.hpp>

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
				const Vec2 groundA, const Vec2 groundB,
				const Vec2 anchorA, const Vec2 anchorB,
				RealNum r)
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

PulleyJoint::PulleyJoint(const PulleyJointDef& def):
	Joint(def),
	m_groundAnchorA(def.groundAnchorA),
	m_groundAnchorB(def.groundAnchorB),
	m_localAnchorA(def.localAnchorA),
	m_localAnchorB(def.localAnchorB),
	m_lengthA(def.lengthA),
	m_lengthB(def.lengthB),
	m_ratio(def.ratio),
	m_constant(def.lengthA + def.ratio * def.lengthB)
{
	assert(!almost_zero(def.ratio));
}

void PulleyJoint::InitVelocityConstraints(BodyConstraints& bodies,
										  const StepConf& step,
										  const ConstraintSolverConf& conf)
{
	auto& bodiesA = bodies.at(GetBodyA());
	auto& bodiesB = bodies.at(GetBodyB());

	m_localCenterA = bodiesA.GetLocalCenter();
	m_invMassA = bodiesA.GetInvMass();
	m_invIA = bodiesA.GetInvRotInertia();
	const auto posA = bodiesA.GetPosition();
	auto velA = bodiesA.GetVelocity();

	m_localCenterB = bodiesB.GetLocalCenter();
	m_invMassB = bodiesB.GetInvMass();
	m_invIB = bodiesB.GetInvRotInertia();
	const auto posB = bodiesB.GetPosition();
	auto velB = bodiesB.GetVelocity();

	const UnitVec2 qA(posA.angular), qB(posB.angular);

	m_rA = Rotate(m_localAnchorA - m_localCenterA, qA);
	m_rB = Rotate(m_localAnchorB - m_localCenterB, qB);

	// Get the pulley axes.
	m_uA = posA.linear + m_rA - m_groundAnchorA;
	m_uB = posB.linear + m_rB - m_groundAnchorB;

	const auto lengthA = GetLength(m_uA);
	const auto lengthB = GetLength(m_uB);

	if (lengthA > (conf.linearSlop * 10))
	{
		m_uA *= RealNum{1} / lengthA;
	}
	else
	{
		m_uA = Vec2_zero;
	}

	if (lengthB > (conf.linearSlop * 10))
	{
		m_uB *= RealNum{1} / lengthB;
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

	if (m_mass > 0)
	{
		m_mass = RealNum{1} / m_mass;
	}

	if (step.doWarmStart)
	{
		// Scale impulses to support variable time steps.
		m_impulse *= step.dtRatio;

		// Warm starting.
		const auto PA = -(m_impulse) * m_uA;
		const auto PB = (-m_ratio * m_impulse) * m_uB;

		velA += Velocity{m_invMassA * PA, 1_rad * m_invIA * Cross(m_rA, PA)};
		velB += Velocity{m_invMassB * PB, 1_rad * m_invIB * Cross(m_rB, PB)};
	}
	else
	{
		m_impulse = 0;
	}

	bodiesA.SetVelocity(velA);
	bodiesB.SetVelocity(velB);
}

RealNum PulleyJoint::SolveVelocityConstraints(BodyConstraints& bodies, const StepConf&)
{
	auto& bodiesA = bodies.at(GetBodyA());
	auto& bodiesB = bodies.at(GetBodyB());

	auto velA = bodiesA.GetVelocity();
	auto velB = bodiesB.GetVelocity();

	const auto vpA = velA.linear + GetRevPerpendicular(m_rA) * velA.angular.ToRadians();
	const auto vpB = velB.linear + GetRevPerpendicular(m_rB) * velB.angular.ToRadians();

	const auto Cdot = -Dot(m_uA, vpA) - m_ratio * Dot(m_uB, vpB);
	const auto impulse = -m_mass * Cdot;
	m_impulse += impulse;

	const auto PA = -impulse * m_uA;
	const auto PB = -m_ratio * impulse * m_uB;
	velA += Velocity{m_invMassA * PA, 1_rad * m_invIA * Cross(m_rA, PA)};
	velB += Velocity{m_invMassB * PB, 1_rad * m_invIB * Cross(m_rB, PB)};

	bodiesA.SetVelocity(velA);
	bodiesB.SetVelocity(velB);
	
	return impulse;
}

bool PulleyJoint::SolvePositionConstraints(BodyConstraints& bodies, const ConstraintSolverConf& conf) const
{
	auto& bodiesA = bodies.at(GetBodyA());
	auto& bodiesB = bodies.at(GetBodyB());

	auto posA = bodiesA.GetPosition();
	auto posB = bodiesB.GetPosition();

	const auto rA = Rotate(m_localAnchorA - m_localCenterA, UnitVec2{posA.angular});
	const auto rB = Rotate(m_localAnchorB - m_localCenterB, UnitVec2{posB.angular});

	// Get the pulley axes.
	const auto pA = posA.linear + rA - m_groundAnchorA;
	const auto lengthA = GetLength(pA);
	const auto uA = (lengthA > (conf.linearSlop * 10))? pA / lengthA: Vec2_zero;

	const auto pB = posB.linear + rB - m_groundAnchorB;
	const auto lengthB = GetLength(pB);
	const auto uB = (lengthB > (conf.linearSlop * 10))? pB / lengthB: Vec2_zero;

	// Compute effective mass.
	const auto ruA = Cross(rA, uA);
	const auto ruB = Cross(rB, uB);

	const auto mA = m_invMassA + m_invIA * ruA * ruA;
	const auto mB = m_invMassB + m_invIB * ruB * ruB;

	auto mass = mA + m_ratio * m_ratio * mB;
	if (mass > 0)
	{
		mass = RealNum{1} / mass;
	}

	const auto C = m_constant - lengthA - (m_ratio * lengthB);
	const auto linearError = Abs(C);

	const auto impulse = -mass * C;

	const auto PA = -impulse * uA;
	const auto PB = -m_ratio * impulse * uB;

	posA += Position{m_invMassA * PA, 1_rad * m_invIA * Cross(rA, PA)};
	posB += Position{m_invMassB * PB, 1_rad * m_invIB * Cross(rB, PB)};

	bodiesA.SetPosition(posA);
	bodiesB.SetPosition(posB);

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

Vec2 PulleyJoint::GetReactionForce(RealNum inv_dt) const
{
	return inv_dt * m_impulse * m_uB;
}

RealNum PulleyJoint::GetReactionTorque(RealNum inv_dt) const
{
	NOT_USED(inv_dt);
	return RealNum{0};
}

RealNum box2d::GetCurrentLengthA(const PulleyJoint& joint)
{
	return GetLength(GetWorldPoint(*joint.GetBodyA(), joint.GetLocalAnchorA()) - joint.GetGroundAnchorA());
}

RealNum box2d::GetCurrentLengthB(const PulleyJoint& joint)
{
	return GetLength(GetWorldPoint(*joint.GetBodyB(), joint.GetLocalAnchorB()) - joint.GetGroundAnchorB());
}

void PulleyJoint::ShiftOrigin(const Vec2 newOrigin)
{
	m_groundAnchorA -= newOrigin;
	m_groundAnchorB -= newOrigin;
}
