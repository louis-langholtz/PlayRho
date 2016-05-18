/*
* Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
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

#include <Box2D/Dynamics/Joints/b2WheelJoint.h>
#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Dynamics/b2TimeStep.h>

using namespace box2d;

// Linear constraint (point-to-line)
// d = pB - pA = xB + rB - xA - rA
// C = dot(ay, d)
// Cdot = dot(d, cross(wA, ay)) + dot(ay, vB + cross(wB, rB) - vA - cross(wA, rA))
//      = -dot(ay, vA) - dot(cross(d + rA, ay), wA) + dot(ay, vB) + dot(cross(rB, ay), vB)
// J = [-ay, -cross(d + rA, ay), ay, cross(rB, ay)]

// Spring linear constraint
// C = dot(ax, d)
// Cdot = = -dot(ax, vA) - dot(cross(d + rA, ax), wA) + dot(ax, vB) + dot(cross(rB, ax), vB)
// J = [-ax -cross(d+rA, ax) ax cross(rB, ax)]

// Motor rotational constraint
// Cdot = wB - wA
// J = [0 0 -1 0 0 1]

void b2WheelJointDef::Initialize(b2Body* bA, b2Body* bB, const b2Vec2& anchor, const b2Vec2& axis)
{
	bodyA = bA;
	bodyB = bB;
	localAnchorA = bodyA->GetLocalPoint(anchor);
	localAnchorB = bodyB->GetLocalPoint(anchor);
	localAxisA = bodyA->GetLocalVector(axis);
}

b2WheelJoint::b2WheelJoint(const b2WheelJointDef* def)
: b2Joint(def)
{
	m_localAnchorA = def->localAnchorA;
	m_localAnchorB = def->localAnchorB;
	m_localXAxisA = def->localAxisA;
	m_localYAxisA = b2Cross(b2Float(1), m_localXAxisA);

	m_mass = b2Float{0};
	m_impulse = b2Float{0};
	m_motorMass = b2Float{0};
	m_motorImpulse = b2Float{0};
	m_springMass = b2Float{0};
	m_springImpulse = b2Float{0};

	m_maxMotorTorque = def->maxMotorTorque;
	m_motorSpeed = def->motorSpeed;
	m_enableMotor = def->enableMotor;

	m_frequencyHz = def->frequencyHz;
	m_dampingRatio = def->dampingRatio;

	m_bias = b2Float{0};
	m_gamma = b2Float{0};

	m_ax = b2Vec2_zero;
	m_ay = b2Vec2_zero;
}

void b2WheelJoint::InitVelocityConstraints(const b2SolverData& data)
{
	m_indexA = m_bodyA->m_islandIndex;
	m_indexB = m_bodyB->m_islandIndex;
	m_localCenterA = m_bodyA->m_sweep.localCenter;
	m_localCenterB = m_bodyB->m_sweep.localCenter;
	m_invMassA = m_bodyA->m_invMass;
	m_invMassB = m_bodyB->m_invMass;
	m_invIA = m_bodyA->m_invI;
	m_invIB = m_bodyB->m_invI;

	const auto mA = m_invMassA, mB = m_invMassB;
	const auto iA = m_invIA, iB = m_invIB;

	const auto cA = data.positions[m_indexA].c;
	const auto aA = data.positions[m_indexA].a;
	auto vA = data.velocities[m_indexA].v;
	auto wA = data.velocities[m_indexA].w;

	const auto cB = data.positions[m_indexB].c;
	const auto aB = data.positions[m_indexB].a;
	auto vB = data.velocities[m_indexB].v;
	auto wB = data.velocities[m_indexB].w;

	const b2Rot qA(aA), qB(aB);

	// Compute the effective masses.
	const auto rA = b2Mul(qA, m_localAnchorA - m_localCenterA);
	const auto rB = b2Mul(qB, m_localAnchorB - m_localCenterB);
	const auto dd = cB + rB - cA - rA;

	// Point to line constraint
	{
		m_ay = b2Mul(qA, m_localYAxisA);
		m_sAy = b2Cross(dd + rA, m_ay);
		m_sBy = b2Cross(rB, m_ay);

		m_mass = mA + mB + iA * m_sAy * m_sAy + iB * m_sBy * m_sBy;

		if (m_mass > b2Float{0})
		{
			m_mass = b2Float(1) / m_mass;
		}
	}

	// Spring constraint
	m_springMass = b2Float{0};
	m_bias = b2Float{0};
	m_gamma = b2Float{0};
	if (m_frequencyHz > b2Float{0})
	{
		m_ax = b2Mul(qA, m_localXAxisA);
		m_sAx = b2Cross(dd + rA, m_ax);
		m_sBx = b2Cross(rB, m_ax);

		const auto invMass = mA + mB + iA * m_sAx * m_sAx + iB * m_sBx * m_sBx;

		if (invMass > b2Float{0})
		{
			m_springMass = b2Float(1) / invMass;

			const auto C = b2Dot(dd, m_ax);

			// Frequency
			const auto omega = b2Float(2) * b2_pi * m_frequencyHz;

			// Damping coefficient
			const auto d = b2Float(2) * m_springMass * m_dampingRatio * omega;

			// Spring stiffness
			const auto k = m_springMass * omega * omega;

			// magic formulas
			const auto h = data.step.get_dt();
			m_gamma = h * (d + h * k);
			if (m_gamma > b2Float{0})
			{
				m_gamma = b2Float(1) / m_gamma;
			}

			m_bias = C * h * k * m_gamma;

			m_springMass = invMass + m_gamma;
			if (m_springMass > b2Float{0})
			{
				m_springMass = b2Float(1) / m_springMass;
			}
		}
	}
	else
	{
		m_springImpulse = b2Float{0};
	}

	// Rotational motor
	if (m_enableMotor)
	{
		m_motorMass = iA + iB;
		if (m_motorMass > b2Float{0})
		{
			m_motorMass = b2Float(1) / m_motorMass;
		}
	}
	else
	{
		m_motorMass = b2Float{0};
		m_motorImpulse = b2Float{0};
	}

	if (data.step.warmStarting)
	{
		// Account for variable time step.
		m_impulse *= data.step.dtRatio;
		m_springImpulse *= data.step.dtRatio;
		m_motorImpulse *= data.step.dtRatio;

		const auto P = m_impulse * m_ay + m_springImpulse * m_ax;
		const auto LA = m_impulse * m_sAy + m_springImpulse * m_sAx + m_motorImpulse;
		const auto LB = m_impulse * m_sBy + m_springImpulse * m_sBx + m_motorImpulse;

		vA -= m_invMassA * P;
		wA -= m_invIA * LA;

		vB += m_invMassB * P;
		wB += m_invIB * LB;
	}
	else
	{
		m_impulse = b2Float{0};
		m_springImpulse = b2Float{0};
		m_motorImpulse = b2Float{0};
	}

	data.velocities[m_indexA].v = vA;
	data.velocities[m_indexA].w = wA;
	data.velocities[m_indexB].v = vB;
	data.velocities[m_indexB].w = wB;
}

void b2WheelJoint::SolveVelocityConstraints(const b2SolverData& data)
{
	const auto mA = m_invMassA, mB = m_invMassB;
	const auto iA = m_invIA, iB = m_invIB;

	auto vA = data.velocities[m_indexA].v;
	auto wA = data.velocities[m_indexA].w;
	auto vB = data.velocities[m_indexB].v;
	auto wB = data.velocities[m_indexB].w;

	// Solve spring constraint
	{
		const auto Cdot = b2Dot(m_ax, vB - vA) + m_sBx * wB - m_sAx * wA;
		const auto impulse = -m_springMass * (Cdot + m_bias + m_gamma * m_springImpulse);
		m_springImpulse += impulse;

		const auto P = impulse * m_ax;
		const auto LA = impulse * m_sAx;
		const auto LB = impulse * m_sBx;

		vA -= mA * P;
		wA -= iA * LA;

		vB += mB * P;
		wB += iB * LB;
	}

	// Solve rotational motor constraint
	{
		const auto Cdot = wB - wA - m_motorSpeed;
		auto impulse = -m_motorMass * Cdot;

		const auto oldImpulse = m_motorImpulse;
		const auto maxImpulse = data.step.get_dt() * m_maxMotorTorque;
		m_motorImpulse = b2Clamp(m_motorImpulse + impulse, -maxImpulse, maxImpulse);
		impulse = m_motorImpulse - oldImpulse;

		wA -= iA * impulse;
		wB += iB * impulse;
	}

	// Solve point to line constraint
	{
		const auto Cdot = b2Dot(m_ay, vB - vA) + m_sBy * wB - m_sAy * wA;
		const auto impulse = -m_mass * Cdot;
		m_impulse += impulse;

		const auto P = impulse * m_ay;
		const auto LA = impulse * m_sAy;
		const auto LB = impulse * m_sBy;

		vA -= mA * P;
		wA -= iA * LA;

		vB += mB * P;
		wB += iB * LB;
	}

	data.velocities[m_indexA].v = vA;
	data.velocities[m_indexA].w = wA;
	data.velocities[m_indexB].v = vB;
	data.velocities[m_indexB].w = wB;
}

bool b2WheelJoint::SolvePositionConstraints(const b2SolverData& data)
{
	auto cA = data.positions[m_indexA].c;
	auto aA = data.positions[m_indexA].a;
	auto cB = data.positions[m_indexB].c;
	auto aB = data.positions[m_indexB].a;

	const b2Rot qA{aA}, qB{aB};

	const auto rA = b2Mul(qA, m_localAnchorA - m_localCenterA);
	const auto rB = b2Mul(qB, m_localAnchorB - m_localCenterB);
	const auto d = (cB - cA) + rB - rA;

	const auto ay = b2Mul(qA, m_localYAxisA);

	const auto sAy = b2Cross(d + rA, ay);
	const auto sBy = b2Cross(rB, ay);

	const auto C = b2Dot(d, ay);

	const auto k = m_invMassA + m_invMassB + m_invIA * m_sAy * m_sAy + m_invIB * m_sBy * m_sBy;

	const auto impulse = (k != b2Float{0})? - C / k: b2Float{0};

	const auto P = impulse * ay;
	const auto LA = impulse * sAy;
	const auto LB = impulse * sBy;

	cA -= m_invMassA * P;
	aA -= m_invIA * LA;
	cB += m_invMassB * P;
	aB += m_invIB * LB;

	data.positions[m_indexA].c = cA;
	data.positions[m_indexA].a = aA;
	data.positions[m_indexB].c = cB;
	data.positions[m_indexB].a = aB;

	return b2Abs(C) <= b2_linearSlop;
}

b2Vec2 b2WheelJoint::GetAnchorA() const
{
	return m_bodyA->GetWorldPoint(m_localAnchorA);
}

b2Vec2 b2WheelJoint::GetAnchorB() const
{
	return m_bodyB->GetWorldPoint(m_localAnchorB);
}

b2Vec2 b2WheelJoint::GetReactionForce(b2Float inv_dt) const
{
	return inv_dt * (m_impulse * m_ay + m_springImpulse * m_ax);
}

b2Float b2WheelJoint::GetReactionTorque(b2Float inv_dt) const
{
	return inv_dt * m_motorImpulse;
}

b2Float b2WheelJoint::GetJointTranslation() const
{
	const auto pA = m_bodyA->GetWorldPoint(m_localAnchorA);
	const auto pB = m_bodyB->GetWorldPoint(m_localAnchorB);
	const auto d = pB - pA;
	const auto axis = m_bodyA->GetWorldVector(m_localXAxisA);
	return b2Dot(d, axis);
}

b2Float b2WheelJoint::GetJointSpeed() const
{
	return m_bodyB->m_angularVelocity - m_bodyA->m_angularVelocity;
}

bool b2WheelJoint::IsMotorEnabled() const
{
	return m_enableMotor;
}

void b2WheelJoint::EnableMotor(bool flag)
{
	m_bodyA->SetAwake();
	m_bodyB->SetAwake();
	m_enableMotor = flag;
}

void b2WheelJoint::SetMotorSpeed(b2Float speed)
{
	m_bodyA->SetAwake();
	m_bodyB->SetAwake();
	m_motorSpeed = speed;
}

void b2WheelJoint::SetMaxMotorTorque(b2Float torque)
{
	m_bodyA->SetAwake();
	m_bodyB->SetAwake();
	m_maxMotorTorque = torque;
}

b2Float b2WheelJoint::GetMotorTorque(b2Float inv_dt) const
{
	return inv_dt * m_motorImpulse;
}

void b2WheelJoint::Dump()
{
	const auto indexA = m_bodyA->m_islandIndex;
	const auto indexB = m_bodyB->m_islandIndex;

	b2Log("  b2WheelJointDef jd;\n");
	b2Log("  jd.bodyA = bodies[%d];\n", indexA);
	b2Log("  jd.bodyB = bodies[%d];\n", indexB);
	b2Log("  jd.collideConnected = bool(%d);\n", m_collideConnected);
	b2Log("  jd.localAnchorA = b2Vec2(%.15lef, %.15lef);\n", m_localAnchorA.x, m_localAnchorA.y);
	b2Log("  jd.localAnchorB = b2Vec2(%.15lef, %.15lef);\n", m_localAnchorB.x, m_localAnchorB.y);
	b2Log("  jd.localAxisA = b2Vec2(%.15lef, %.15lef);\n", m_localXAxisA.x, m_localXAxisA.y);
	b2Log("  jd.enableMotor = bool(%d);\n", m_enableMotor);
	b2Log("  jd.motorSpeed = %.15lef;\n", m_motorSpeed);
	b2Log("  jd.maxMotorTorque = %.15lef;\n", m_maxMotorTorque);
	b2Log("  jd.frequencyHz = %.15lef;\n", m_frequencyHz);
	b2Log("  jd.dampingRatio = %.15lef;\n", m_dampingRatio);
	b2Log("  joints[%d] = m_world->CreateJoint(&jd);\n", m_index);
}
