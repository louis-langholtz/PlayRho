/*
* Copyright (c) 2006-2012 Erin Catto http://www.box2d.org
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

#include <Box2D/Dynamics/Joints/MotorJoint.h>
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

void MotorJointDef::Initialize(Body* bA, Body* bB)
{
	bodyA = bA;
	bodyB = bB;
	linearOffset = bodyA->GetLocalPoint(bodyB->GetPosition());
	angularOffset = bodyB->GetAngle() - bodyA->GetAngle();
}

MotorJoint::MotorJoint(const MotorJointDef& def)
: Joint(def)
{
	m_linearOffset = def.linearOffset;
	m_angularOffset = def.angularOffset;

	m_linearImpulse = Vec2_zero;
	m_angularImpulse = float_t{0};

	m_maxForce = def.maxForce;
	m_maxTorque = def.maxTorque;
	m_correctionFactor = def.correctionFactor;
}

void MotorJoint::InitVelocityConstraints(const SolverData& data)
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

	const auto qA = Rot(aA);
	const auto qB = Rot(aB);

	// Compute the effective mass matrix.
	m_rA = Mul(qA, -m_localCenterA);
	m_rB = Mul(qB, -m_localCenterB);

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
	K.ex.x = mA + mB + iA * m_rA.y * m_rA.y + iB * m_rB.y * m_rB.y;
	K.ex.y = -iA * m_rA.x * m_rA.y - iB * m_rB.x * m_rB.y;
	K.ey.x = K.ex.y;
	K.ey.y = mA + mB + iA * m_rA.x * m_rA.x + iB * m_rB.x * m_rB.x;

	m_linearMass = K.GetInverse();

	m_angularMass = iA + iB;
	if (m_angularMass > float_t{0})
	{
		m_angularMass = float_t{1} / m_angularMass;
	}

	m_linearError = cB + m_rB - cA - m_rA - Mul(qA, m_linearOffset);
	m_angularError = aB - aA - m_angularOffset;

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

void MotorJoint::SolveVelocityConstraints(const SolverData& data)
{
	auto vA = data.velocities[m_indexA].v;
	auto wA = data.velocities[m_indexA].w;
	auto vB = data.velocities[m_indexB].v;
	auto wB = data.velocities[m_indexB].w;

	const auto mA = m_invMassA, mB = m_invMassB;
	const auto iA = m_invIA, iB = m_invIB;

	const auto h = data.step.get_dt();
	const auto inv_h = data.step.get_inv_dt();

	// Solve angular friction
	{
		const auto Cdot = wB - wA + inv_h * m_correctionFactor * m_angularError;
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
		const auto Cdot = vB + Cross(wB, m_rB) - vA - Cross(wA, m_rA) + inv_h * m_correctionFactor * m_linearError;

		auto impulse = -Mul(m_linearMass, Cdot);
		const auto oldImpulse = m_linearImpulse;
		m_linearImpulse += impulse;

		const auto maxImpulse = h * m_maxForce;

		if (m_linearImpulse.LengthSquared() > maxImpulse * maxImpulse)
		{
			m_linearImpulse.Normalize();
			m_linearImpulse *= maxImpulse;
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

bool MotorJoint::SolvePositionConstraints(const SolverData& data)
{
	BOX2D_NOT_USED(data);

	return true;
}

Vec2 MotorJoint::GetAnchorA() const
{
	return m_bodyA->GetPosition();
}

Vec2 MotorJoint::GetAnchorB() const
{
	return m_bodyB->GetPosition();
}

Vec2 MotorJoint::GetReactionForce(float_t inv_dt) const
{
	return inv_dt * m_linearImpulse;
}

float_t MotorJoint::GetReactionTorque(float_t inv_dt) const
{
	return inv_dt * m_angularImpulse;
}

void MotorJoint::SetMaxForce(float_t force)
{
	assert(IsValid(force) && (force >= float_t{0}));
	m_maxForce = force;
}

float_t MotorJoint::GetMaxForce() const
{
	return m_maxForce;
}

void MotorJoint::SetMaxTorque(float_t torque)
{
	assert(IsValid(torque) && (torque >= float_t{0}));
	m_maxTorque = torque;
}

float_t MotorJoint::GetMaxTorque() const
{
	return m_maxTorque;
}

void MotorJoint::SetCorrectionFactor(float_t factor)
{
	assert(IsValid(factor) && (float_t{0} <= factor) && (factor <= float_t{1}));
	m_correctionFactor = factor;
}

float_t MotorJoint::GetCorrectionFactor() const
{
	return m_correctionFactor;
}

void MotorJoint::SetLinearOffset(const Vec2& linearOffset)
{
	if ((linearOffset.x != m_linearOffset.x) || (linearOffset.y != m_linearOffset.y))
	{
		m_bodyA->SetAwake();
		m_bodyB->SetAwake();
		m_linearOffset = linearOffset;
	}
}

const Vec2& MotorJoint::GetLinearOffset() const
{
	return m_linearOffset;
}

void MotorJoint::SetAngularOffset(float_t angularOffset)
{
	if (angularOffset != m_angularOffset)
	{
		m_bodyA->SetAwake();
		m_bodyB->SetAwake();
		m_angularOffset = angularOffset;
	}
}

float_t MotorJoint::GetAngularOffset() const
{
	return m_angularOffset;
}

void MotorJoint::Dump()
{
	const auto indexA = m_bodyA->m_islandIndex;
	const auto indexB = m_bodyB->m_islandIndex;

	log("  MotorJointDef jd;\n");
	log("  jd.bodyA = bodies[%d];\n", indexA);
	log("  jd.bodyB = bodies[%d];\n", indexB);
	log("  jd.collideConnected = bool(%d);\n", m_collideConnected);
	log("  jd.linearOffset = Vec2(%.15lef, %.15lef);\n", m_linearOffset.x, m_linearOffset.y);
	log("  jd.angularOffset = %.15lef;\n", m_angularOffset);
	log("  jd.maxForce = %.15lef;\n", m_maxForce);
	log("  jd.maxTorque = %.15lef;\n", m_maxTorque);
	log("  jd.correctionFactor = %.15lef;\n", m_correctionFactor);
	log("  joints[%d] = m_world->CreateJoint(&jd);\n", m_index);
}
