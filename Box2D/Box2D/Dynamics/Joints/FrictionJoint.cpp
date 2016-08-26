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

#include <Box2D/Dynamics/Joints/FrictionJoint.h>
#include <Box2D/Dynamics/Body.h>
#include <Box2D/Dynamics/TimeStep.h>

using namespace box2d;

// Point-to-point constraint
// Cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-I -r1_skew I r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

// Angle constraint
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2

void FrictionJointDef::Initialize(Body* bA, Body* bB, const Vec2& anchor)
{
	bodyA = bA;
	bodyB = bB;
	localAnchorA = GetLocalPoint(*bodyA, anchor);
	localAnchorB = GetLocalPoint(*bodyB, anchor);
}

FrictionJoint::FrictionJoint(const FrictionJointDef& def)
: Joint(def)
{
	m_localAnchorA = def.localAnchorA;
	m_localAnchorB = def.localAnchorB;
	m_maxForce = def.maxForce;
	m_maxTorque = def.maxTorque;
}

void FrictionJoint::InitVelocityConstraints(const SolverData& data)
{
	m_indexA = m_bodyA->GetIslandIndex();
	m_indexB = m_bodyB->GetIslandIndex();
	m_localCenterA = m_bodyA->GetLocalCenter();
	m_localCenterB = m_bodyB->GetLocalCenter();
	m_invMassA = m_bodyA->GetInverseMass();
	m_invMassB = m_bodyB->GetInverseMass();
	m_invIA = m_bodyA->GetInverseInertia();
	m_invIB = m_bodyB->GetInverseInertia();

	const auto aA = data.positions[m_indexA].a;
	auto vA = data.velocities[m_indexA].v;
	auto wA = data.velocities[m_indexA].w;

	const auto aB = data.positions[m_indexB].a;
	auto vB = data.velocities[m_indexB].v;
	auto wB = data.velocities[m_indexB].w;

	const auto qA = Rot(aA);
	const auto qB = Rot(aB);

	// Compute the effective mass matrix.
	m_rA = Rotate(m_localAnchorA - m_localCenterA, qA);
	m_rB = Rotate(m_localAnchorB - m_localCenterB, qB);

	// J = [-I -r1_skew I r2_skew]
	//     [ 0       -1 0       1]
	// r_skew = [-ry; rx]

	// Matlab
	// K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
	//     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
	//     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

	const auto mA = m_invMassA, mB = m_invMassB;
	const auto iA = m_invIA, iB = m_invIB;

	Mat22 K;
	K.ex.x = mA + mB + iA * Square(m_rA.y) + iB * Square(m_rB.y);
	K.ex.y = -iA * m_rA.x * m_rA.y - iB * m_rB.x * m_rB.y;
	K.ey.x = K.ex.y;
	K.ey.y = mA + mB + iA * Square(m_rA.x) + iB * Square(m_rB.x);

	m_linearMass = Invert(K);

	m_angularMass = iA + iB;
	if (m_angularMass > float_t{0})
	{
		m_angularMass = float_t{1} / m_angularMass;
	}

	if (data.step.warmStarting)
	{
		// Scale impulses to support a variable time step.
		m_linearImpulse *= data.step.dtRatio;
		m_angularImpulse *= data.step.dtRatio;

		const auto P = Vec2{m_linearImpulse.x, m_linearImpulse.y};
		vA -= mA * P;
		wA -= iA * (Cross(m_rA, P) + m_angularImpulse);
		vB += mB * P;
		wB += iB * (Cross(m_rB, P) + m_angularImpulse);
	}
	else
	{
		m_linearImpulse = Vec2_zero;
		m_angularImpulse = float_t{0};
	}

	data.velocities[m_indexA].v = vA;
	data.velocities[m_indexA].w = wA;
	data.velocities[m_indexB].v = vB;
	data.velocities[m_indexB].w = wB;
}

void FrictionJoint::SolveVelocityConstraints(const SolverData& data)
{
	auto vA = data.velocities[m_indexA].v;
	auto wA = data.velocities[m_indexA].w;
	auto vB = data.velocities[m_indexB].v;
	auto wB = data.velocities[m_indexB].w;

	const auto mA = m_invMassA, mB = m_invMassB;
	const auto iA = m_invIA, iB = m_invIB;

	const auto h = data.step.get_dt();

	// Solve angular friction
	{
		const auto Cdot = wB - wA;
		auto impulse = -m_angularMass * Cdot;

		const auto oldImpulse = m_angularImpulse;
		const auto maxImpulse = h * m_maxTorque;
		m_angularImpulse = Clamp(m_angularImpulse + impulse, -maxImpulse, maxImpulse);
		impulse = m_angularImpulse - oldImpulse;

		wA -= iA * impulse;
		wB += iB * impulse;
	}

	// Solve linear friction
	{
		const auto Cdot = vB + (GetReversePerpendicular(m_rB) * wB) - vA - (GetReversePerpendicular(m_rA) * wA);

		auto impulse = -Transform(Cdot, m_linearMass);
		const auto oldImpulse = m_linearImpulse;
		m_linearImpulse += impulse;

		const auto maxImpulse = h * m_maxForce;

		if (LengthSquared(m_linearImpulse) > Square(maxImpulse))
		{
			m_linearImpulse = GetUnitVector(m_linearImpulse) * maxImpulse;
		}

		impulse = m_linearImpulse - oldImpulse;

		vA -= mA * impulse;
		wA -= iA * Cross(m_rA, impulse);

		vB += mB * impulse;
		wB += iB * Cross(m_rB, impulse);
	}

	data.velocities[m_indexA].v = vA;
	data.velocities[m_indexA].w = wA;
	data.velocities[m_indexB].v = vB;
	data.velocities[m_indexB].w = wB;
}

bool FrictionJoint::SolvePositionConstraints(const SolverData& data)
{
	BOX2D_NOT_USED(data);

	return true;
}

Vec2 FrictionJoint::GetAnchorA() const
{
	return GetWorldPoint(*m_bodyA, m_localAnchorA);
}

Vec2 FrictionJoint::GetAnchorB() const
{
	return GetWorldPoint(*m_bodyB, m_localAnchorB);
}

Vec2 FrictionJoint::GetReactionForce(float_t inv_dt) const
{
	return inv_dt * m_linearImpulse;
}

float_t FrictionJoint::GetReactionTorque(float_t inv_dt) const
{
	return inv_dt * m_angularImpulse;
}

void FrictionJoint::SetMaxForce(float_t force)
{
	assert(IsValid(force) && (force >= float_t{0}));
	m_maxForce = force;
}

float_t FrictionJoint::GetMaxForce() const
{
	return m_maxForce;
}

void FrictionJoint::SetMaxTorque(float_t torque)
{
	assert(IsValid(torque) && (torque >= float_t{0}));
	m_maxTorque = torque;
}

float_t FrictionJoint::GetMaxTorque() const
{
	return m_maxTorque;
}

void box2d::Dump(const FrictionJoint& joint, size_t index)
{
	log("  FrictionJointDef jd;\n");
	log("  jd.bodyA = bodies[%d];\n", GetWorldIndex(joint.GetBodyA()));
	log("  jd.bodyB = bodies[%d];\n", GetWorldIndex(joint.GetBodyB()));
	log("  jd.collideConnected = bool(%d);\n", joint.GetCollideConnected());
	log("  jd.localAnchorA = Vec2(%.15lef, %.15lef);\n", joint.GetLocalAnchorA().x, joint.GetLocalAnchorA().y);
	log("  jd.localAnchorB = Vec2(%.15lef, %.15lef);\n", joint.GetLocalAnchorB().x, joint.GetLocalAnchorB().y);
	log("  jd.maxForce = %.15lef;\n", joint.GetMaxForce());
	log("  jd.maxTorque = %.15lef;\n", joint.GetMaxTorque());
	log("  joints[%d] = m_world->CreateJoint(jd);\n", index);
}
