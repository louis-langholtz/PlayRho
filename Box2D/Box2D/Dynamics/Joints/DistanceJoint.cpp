/*
* Original work Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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

#include <Box2D/Dynamics/Joints/DistanceJoint.h>
#include <Box2D/Dynamics/Body.hpp>
#include <Box2D/Dynamics/TimeStep.hpp>
#include <Box2D/Dynamics/Contacts/ContactSolver.h>

using namespace box2d;

// 1-D constrained system
// m (v2 - v1) = lambda
// v2 + (beta/h) * x1 + gamma * lambda = 0, gamma has units of inverse mass.
// x2 = x1 + h * v2

// 1-D mass-damper-spring system
// m (v2 - v1) + h * d * v2 + h * k * 

// C = norm(p2 - p1) - L
// u = (p2 - p1) / norm(p2 - p1)
// Cdot = dot(u, v2 + cross(w2, r2) - v1 - cross(w1, r1))
// J = [-u -cross(r1, u) u cross(r2, u)]
// K = J * invM * JT
//   = invMass1 + invI1 * cross(r1, u)^2 + invMass2 + invI2 * cross(r2, u)^2

DistanceJointDef::DistanceJointDef(Body* bA, Body* bB,
								   const Vec2& anchor1, const Vec2& anchor2,
								   float_t freq, float_t damp) noexcept:
	JointDef{JointType::Distance, bA, bB},
	localAnchorA{GetLocalPoint(*bA, anchor1)}, localAnchorB{GetLocalPoint(*bB, anchor2)},
	length{GetLength(anchor2 - anchor1)},
	frequencyHz{freq}, dampingRatio{damp}
{
}

DistanceJoint::DistanceJoint(const DistanceJointDef& def)
: Joint(def)
{
	m_localAnchorA = def.localAnchorA;
	m_localAnchorB = def.localAnchorB;
	m_length = def.length;
	m_frequencyHz = def.frequencyHz;
	m_dampingRatio = def.dampingRatio;
}

void DistanceJoint::InitVelocityConstraints(Span<Velocity> velocities,
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
	m_u = (cB + m_rB) - (cA + m_rA);

	// Handle singularity.
	const auto length = box2d::GetLength(m_u);
	if (length > conf.linearSlop)
	{
		m_u *= float_t(1) / length;
	}
	else
	{
		m_u = Vec2_zero;
	}

	const auto crAu = Cross(m_rA, m_u);
	const auto crBu = Cross(m_rB, m_u);
	auto invMass = m_invMassA + m_invIA * Square(crAu) + m_invMassB + m_invIB * Square(crBu);

	// Compute the effective mass matrix.
	m_mass = (invMass != float_t{0}) ? float_t{1} / invMass : float_t{0};

	if (m_frequencyHz > float_t{0})
	{
		const auto C = length - m_length;

		// Frequency
		const auto omega = float_t(2) * Pi * m_frequencyHz;

		// Damping coefficient
		const auto d = float_t(2) * m_mass * m_dampingRatio * omega;

		// Spring stiffness
		const auto k = m_mass * Square(omega);

		// magic formulas
		const auto h = step.get_dt();
		m_gamma = h * (d + h * k);
		m_gamma = (m_gamma != float_t{0}) ? float_t{1} / m_gamma : float_t{0};
		m_bias = C * h * k * m_gamma;

		invMass += m_gamma;
		m_mass = (invMass != float_t{0}) ? float_t{1} / invMass : float_t{0};
	}
	else
	{
		m_gamma = float_t{0};
		m_bias = float_t{0};
	}

	if (step.warmStarting)
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

void DistanceJoint::SolveVelocityConstraints(Span<Velocity> velocities, const TimeStep& step)
{
	auto vA = velocities[m_indexA].v;
	auto wA = velocities[m_indexA].w;
	auto vB = velocities[m_indexB].v;
	auto wB = velocities[m_indexB].w;

	// Cdot = dot(u, v + cross(w, r))
	const auto vpA = vA + GetRevPerpendicular(m_rA) * wA.ToRadians();
	const auto vpB = vB + GetRevPerpendicular(m_rB) * wB.ToRadians();
	const auto Cdot = Dot(m_u, vpB - vpA);

	const auto impulse = -m_mass * (Cdot + m_bias + m_gamma * m_impulse);
	m_impulse += impulse;

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

bool DistanceJoint::SolvePositionConstraints(Span<Position> positions, const ConstraintSolverConf& conf)
{
	if (m_frequencyHz > float_t{0})
	{
		// There is no position correction for soft distance constraints.
		return true;
	}

	auto cA = positions[m_indexA].c;
	auto aA = positions[m_indexA].a;
	auto cB = positions[m_indexB].c;
	auto aB = positions[m_indexB].a;

	const auto qA = UnitVec2(aA);
	const auto qB = UnitVec2(aB);

	const auto rA = Rotate(m_localAnchorA - m_localCenterA, qA);
	const auto rB = Rotate(m_localAnchorB - m_localCenterB, qB);
	auto u = cB + rB - cA - rA;

	const auto length = Normalize(u);
	const auto deltaLength = length - m_length;
	const auto C = Clamp(deltaLength, -conf.maxLinearCorrection, conf.maxLinearCorrection);

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

	return Abs(C) < conf.linearSlop;
}

Vec2 DistanceJoint::GetAnchorA() const
{
	return GetWorldPoint(*GetBodyA(), m_localAnchorA);
}

Vec2 DistanceJoint::GetAnchorB() const
{
	return GetWorldPoint(*GetBodyB(), m_localAnchorB);
}

Vec2 DistanceJoint::GetReactionForce(float_t inv_dt) const
{
	return (inv_dt * m_impulse) * m_u;
}

float_t DistanceJoint::GetReactionTorque(float_t inv_dt) const
{
	BOX2D_NOT_USED(inv_dt);
	return float_t{0};
}

void box2d::Dump(const DistanceJoint& joint, size_t index)
{
	log("  DistanceJointDef jd;\n");
	log("  jd.bodyA = bodies[%d];\n", GetWorldIndex(joint.GetBodyA()));
	log("  jd.bodyB = bodies[%d];\n", GetWorldIndex(joint.GetBodyB()));
	log("  jd.collideConnected = bool(%d);\n", joint.GetCollideConnected());
	log("  jd.localAnchorA = Vec2(%.15lef, %.15lef);\n", joint.GetLocalAnchorA().x, joint.GetLocalAnchorA().y);
	log("  jd.localAnchorB = Vec2(%.15lef, %.15lef);\n", joint.GetLocalAnchorB().x, joint.GetLocalAnchorB().y);
	log("  jd.length = %.15lef;\n", joint.GetLength());
	log("  jd.frequencyHz = %.15lef;\n", joint.GetFrequency());
	log("  jd.dampingRatio = %.15lef;\n", joint.GetDampingRatio());
	log("  joints[%d] = m_world->CreateJoint(jd);\n", index);
}
