/*
* Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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

#include <Box2D/Dynamics/Joints/b2RevoluteJoint.h>
#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Dynamics/b2TimeStep.h>

using namespace box2d;

// Point-to-point constraint
// C = p2 - p1
// Cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-I -r1_skew I r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

// Motor constraint
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2

void b2RevoluteJointDef::Initialize(b2Body* bA, b2Body* bB, const b2Vec2& anchor)
{
	bodyA = bA;
	bodyB = bB;
	localAnchorA = bodyA->GetLocalPoint(anchor);
	localAnchorB = bodyB->GetLocalPoint(anchor);
	referenceAngle = bodyB->GetAngle() - bodyA->GetAngle();
}

b2RevoluteJoint::b2RevoluteJoint(const b2RevoluteJointDef* def)
: b2Joint(def)
{
	m_localAnchorA = def->localAnchorA;
	m_localAnchorB = def->localAnchorB;
	m_referenceAngle = def->referenceAngle;

	m_impulse = b2Vec3_zero;
	m_motorImpulse = float_t{0};

	m_lowerAngle = def->lowerAngle;
	m_upperAngle = def->upperAngle;
	m_maxMotorTorque = def->maxMotorTorque;
	m_motorSpeed = def->motorSpeed;
	m_enableLimit = def->enableLimit;
	m_enableMotor = def->enableMotor;
	m_limitState = e_inactiveLimit;
}

void b2RevoluteJoint::InitVelocityConstraints(const b2SolverData& data)
{
	m_indexA = m_bodyA->m_islandIndex;
	m_indexB = m_bodyB->m_islandIndex;
	m_localCenterA = m_bodyA->m_sweep.localCenter;
	m_localCenterB = m_bodyB->m_sweep.localCenter;
	m_invMassA = m_bodyA->m_invMass;
	m_invMassB = m_bodyB->m_invMass;
	m_invIA = m_bodyA->m_invI;
	m_invIB = m_bodyB->m_invI;

	const auto aA = data.positions[m_indexA].a;
	auto vA = data.velocities[m_indexA].v;
	auto wA = data.velocities[m_indexA].w;

	const auto aB = data.positions[m_indexB].a;
	auto vB = data.velocities[m_indexB].v;
	auto wB = data.velocities[m_indexB].w;

	const b2Rot qA(aA), qB(aB);

	m_rA = b2Mul(qA, m_localAnchorA - m_localCenterA);
	m_rB = b2Mul(qB, m_localAnchorB - m_localCenterB);

	// J = [-I -r1_skew I r2_skew]
	//     [ 0       -1 0       1]
	// r_skew = [-ry; rx]

	// Matlab
	// K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
	//     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
	//     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

	const auto mA = m_invMassA, mB = m_invMassB;
	const auto iA = m_invIA, iB = m_invIB;

	const auto fixedRotation = ((iA + iB) == float_t{0});

	m_mass.ex.x = mA + mB + m_rA.y * m_rA.y * iA + m_rB.y * m_rB.y * iB;
	m_mass.ey.x = -m_rA.y * m_rA.x * iA - m_rB.y * m_rB.x * iB;
	m_mass.ez.x = -m_rA.y * iA - m_rB.y * iB;
	m_mass.ex.y = m_mass.ey.x;
	m_mass.ey.y = mA + mB + m_rA.x * m_rA.x * iA + m_rB.x * m_rB.x * iB;
	m_mass.ez.y = m_rA.x * iA + m_rB.x * iB;
	m_mass.ex.z = m_mass.ez.x;
	m_mass.ey.z = m_mass.ez.y;
	m_mass.ez.z = iA + iB;

	m_motorMass = iA + iB;
	if (m_motorMass > float_t{0})
	{
		m_motorMass = float_t{1} / m_motorMass;
	}

	if (m_enableMotor == false || fixedRotation)
	{
		m_motorImpulse = float_t{0};
	}

	if (m_enableLimit && fixedRotation == false)
	{
		const auto jointAngle = aB - aA - m_referenceAngle;
		if (b2Abs(m_upperAngle - m_lowerAngle) < (float_t{2} * b2_angularSlop))
		{
			m_limitState = e_equalLimits;
		}
		else if (jointAngle <= m_lowerAngle)
		{
			if (m_limitState != e_atLowerLimit)
			{
				m_impulse.z = float_t{0};
			}
			m_limitState = e_atLowerLimit;
		}
		else if (jointAngle >= m_upperAngle)
		{
			if (m_limitState != e_atUpperLimit)
			{
				m_impulse.z = float_t{0};
			}
			m_limitState = e_atUpperLimit;
		}
		else
		{
			m_limitState = e_inactiveLimit;
			m_impulse.z = float_t{0};
		}
	}
	else
	{
		m_limitState = e_inactiveLimit;
	}

	if (data.step.warmStarting)
	{
		// Scale impulses to support a variable time step.
		m_impulse *= data.step.dtRatio;
		m_motorImpulse *= data.step.dtRatio;

		const auto P = b2Vec2(m_impulse.x, m_impulse.y);

		vA -= mA * P;
		wA -= iA * (b2Cross(m_rA, P) + m_motorImpulse + m_impulse.z);

		vB += mB * P;
		wB += iB * (b2Cross(m_rB, P) + m_motorImpulse + m_impulse.z);
	}
	else
	{
		m_impulse = b2Vec3_zero;
		m_motorImpulse = float_t{0};
	}

	data.velocities[m_indexA].v = vA;
	data.velocities[m_indexA].w = wA;
	data.velocities[m_indexB].v = vB;
	data.velocities[m_indexB].w = wB;
}

void b2RevoluteJoint::SolveVelocityConstraints(const b2SolverData& data)
{
	auto vA = data.velocities[m_indexA].v;
	auto wA = data.velocities[m_indexA].w;
	auto vB = data.velocities[m_indexB].v;
	auto wB = data.velocities[m_indexB].w;

	const auto mA = m_invMassA, mB = m_invMassB;
	const auto iA = m_invIA, iB = m_invIB;

	const auto fixedRotation = (iA + iB == float_t{0});

	// Solve motor constraint.
	if (m_enableMotor && (m_limitState != e_equalLimits) && !fixedRotation)
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

	// Solve limit constraint.
	if (m_enableLimit && (m_limitState != e_inactiveLimit) && !fixedRotation)
	{
		const auto Cdot1 = vB + b2Cross(wB, m_rB) - vA - b2Cross(wA, m_rA);
		const auto Cdot2 = wB - wA;
		const auto Cdot = b2Vec3(Cdot1.x, Cdot1.y, Cdot2);

		auto impulse = -m_mass.Solve33(Cdot);

		if (m_limitState == e_equalLimits)
		{
			m_impulse += impulse;
		}
		else if (m_limitState == e_atLowerLimit)
		{
			const auto newImpulse = m_impulse.z + impulse.z;
			if (newImpulse < float_t{0})
			{
				const auto rhs = -Cdot1 + m_impulse.z * b2Vec2(m_mass.ez.x, m_mass.ez.y);
				const auto reduced = m_mass.Solve22(rhs);
				impulse.x = reduced.x;
				impulse.y = reduced.y;
				impulse.z = -m_impulse.z;
				m_impulse.x += reduced.x;
				m_impulse.y += reduced.y;
				m_impulse.z = float_t{0};
			}
			else
			{
				m_impulse += impulse;
			}
		}
		else if (m_limitState == e_atUpperLimit)
		{
			const auto newImpulse = m_impulse.z + impulse.z;
			if (newImpulse > float_t{0})
			{
				const auto rhs = -Cdot1 + m_impulse.z * b2Vec2(m_mass.ez.x, m_mass.ez.y);
				const auto reduced = m_mass.Solve22(rhs);
				impulse.x = reduced.x;
				impulse.y = reduced.y;
				impulse.z = -m_impulse.z;
				m_impulse.x += reduced.x;
				m_impulse.y += reduced.y;
				m_impulse.z = float_t{0};
			}
			else
			{
				m_impulse += impulse;
			}
		}

		const auto P = b2Vec2(impulse.x, impulse.y);

		vA -= mA * P;
		wA -= iA * (b2Cross(m_rA, P) + impulse.z);

		vB += mB * P;
		wB += iB * (b2Cross(m_rB, P) + impulse.z);
	}
	else
	{
		// Solve point-to-point constraint
		const auto Cdot = vB + b2Cross(wB, m_rB) - vA - b2Cross(wA, m_rA);
		const auto impulse = m_mass.Solve22(-Cdot);

		m_impulse.x += impulse.x;
		m_impulse.y += impulse.y;

		vA -= mA * impulse;
		wA -= iA * b2Cross(m_rA, impulse);

		vB += mB * impulse;
		wB += iB * b2Cross(m_rB, impulse);
	}

	data.velocities[m_indexA].v = vA;
	data.velocities[m_indexA].w = wA;
	data.velocities[m_indexB].v = vB;
	data.velocities[m_indexB].w = wB;
}

bool b2RevoluteJoint::SolvePositionConstraints(const b2SolverData& data)
{
	auto cA = data.positions[m_indexA].c;
	auto aA = data.positions[m_indexA].a;
	auto cB = data.positions[m_indexB].c;
	auto aB = data.positions[m_indexB].a;

	auto qA = b2Rot(aA);
	auto qB = b2Rot(aB);

	auto angularError = float_t{0};
	auto positionError = float_t{0};

	const auto fixedRotation = ((m_invIA + m_invIB) == float_t{0});

	// Solve angular limit constraint.
	if (m_enableLimit && m_limitState != e_inactiveLimit && fixedRotation == false)
	{
		const auto angle = aB - aA - m_referenceAngle;
		auto limitImpulse = float_t{0};

		if (m_limitState == e_equalLimits)
		{
			// Prevent large angular corrections
			const auto C = b2Clamp(angle - m_lowerAngle, -b2_maxAngularCorrection, b2_maxAngularCorrection);
			limitImpulse = -m_motorMass * C;
			angularError = b2Abs(C);
		}
		else if (m_limitState == e_atLowerLimit)
		{
			auto C = angle - m_lowerAngle;
			angularError = -C;

			// Prevent large angular corrections and allow some slop.
			C = b2Clamp(C + b2_angularSlop, -b2_maxAngularCorrection, float_t{0});
			limitImpulse = -m_motorMass * C;
		}
		else if (m_limitState == e_atUpperLimit)
		{
			auto C = angle - m_upperAngle;
			angularError = C;

			// Prevent large angular corrections and allow some slop.
			C = b2Clamp(C - b2_angularSlop, float_t{0}, b2_maxAngularCorrection);
			limitImpulse = -m_motorMass * C;
		}

		aA -= m_invIA * limitImpulse;
		aB += m_invIB * limitImpulse;
	}

	// Solve point-to-point constraint.
	{
		qA = b2Rot(aA);
		qB = b2Rot(aB);
		const auto rA = b2Mul(qA, m_localAnchorA - m_localCenterA);
		const auto rB = b2Mul(qB, m_localAnchorB - m_localCenterB);

		const auto C = cB + rB - cA - rA;
		positionError = C.Length();

		const auto mA = m_invMassA, mB = m_invMassB;
		const auto iA = m_invIA, iB = m_invIB;

		b2Mat22 K;
		K.ex.x = mA + mB + iA * rA.y * rA.y + iB * rB.y * rB.y;
		K.ex.y = -iA * rA.x * rA.y - iB * rB.x * rB.y;
		K.ey.x = K.ex.y;
		K.ey.y = mA + mB + iA * rA.x * rA.x + iB * rB.x * rB.x;

		const auto impulse = -K.Solve(C);

		cA -= mA * impulse;
		aA -= iA * b2Cross(rA, impulse);

		cB += mB * impulse;
		aB += iB * b2Cross(rB, impulse);
	}

	data.positions[m_indexA].c = cA;
	data.positions[m_indexA].a = aA;
	data.positions[m_indexB].c = cB;
	data.positions[m_indexB].a = aB;
	
	return (positionError <= b2_linearSlop) && (angularError <= b2_angularSlop);
}

b2Vec2 b2RevoluteJoint::GetAnchorA() const
{
	return m_bodyA->GetWorldPoint(m_localAnchorA);
}

b2Vec2 b2RevoluteJoint::GetAnchorB() const
{
	return m_bodyB->GetWorldPoint(m_localAnchorB);
}

b2Vec2 b2RevoluteJoint::GetReactionForce(float_t inv_dt) const
{
	const auto P = b2Vec2(m_impulse.x, m_impulse.y);
	return inv_dt * P;
}

float_t b2RevoluteJoint::GetReactionTorque(float_t inv_dt) const
{
	return inv_dt * m_impulse.z;
}

float_t b2RevoluteJoint::GetJointAngle() const
{
	return m_bodyB->m_sweep.a - m_bodyA->m_sweep.a - m_referenceAngle;
}

float_t b2RevoluteJoint::GetJointSpeed() const
{
	return m_bodyB->m_angularVelocity - m_bodyA->m_angularVelocity;
}

bool b2RevoluteJoint::IsMotorEnabled() const
{
	return m_enableMotor;
}

void b2RevoluteJoint::EnableMotor(bool flag)
{
	m_bodyA->SetAwake();
	m_bodyB->SetAwake();
	m_enableMotor = flag;
}

float_t b2RevoluteJoint::GetMotorTorque(float_t inv_dt) const
{
	return inv_dt * m_motorImpulse;
}

void b2RevoluteJoint::SetMotorSpeed(float_t speed)
{
	m_bodyA->SetAwake();
	m_bodyB->SetAwake();
	m_motorSpeed = speed;
}

void b2RevoluteJoint::SetMaxMotorTorque(float_t torque)
{
	m_bodyA->SetAwake();
	m_bodyB->SetAwake();
	m_maxMotorTorque = torque;
}

bool b2RevoluteJoint::IsLimitEnabled() const
{
	return m_enableLimit;
}

void b2RevoluteJoint::EnableLimit(bool flag)
{
	if (flag != m_enableLimit)
	{
		m_bodyA->SetAwake();
		m_bodyB->SetAwake();
		m_enableLimit = flag;
		m_impulse.z = float_t{0};
	}
}

float_t b2RevoluteJoint::GetLowerLimit() const
{
	return m_lowerAngle;
}

float_t b2RevoluteJoint::GetUpperLimit() const
{
	return m_upperAngle;
}

void b2RevoluteJoint::SetLimits(float_t lower, float_t upper)
{
	assert(lower <= upper);
	
	if ((lower != m_lowerAngle) || (upper != m_upperAngle))
	{
		m_bodyA->SetAwake();
		m_bodyB->SetAwake();
		m_impulse.z = float_t{0};
		m_lowerAngle = lower;
		m_upperAngle = upper;
	}
}

void b2RevoluteJoint::Dump()
{
	const auto indexA = m_bodyA->m_islandIndex;
	const auto indexB = m_bodyB->m_islandIndex;

	b2Log("  b2RevoluteJointDef jd;\n");
	b2Log("  jd.bodyA = bodies[%d];\n", indexA);
	b2Log("  jd.bodyB = bodies[%d];\n", indexB);
	b2Log("  jd.collideConnected = bool(%d);\n", m_collideConnected);
	b2Log("  jd.localAnchorA = b2Vec2(%.15lef, %.15lef);\n", m_localAnchorA.x, m_localAnchorA.y);
	b2Log("  jd.localAnchorB = b2Vec2(%.15lef, %.15lef);\n", m_localAnchorB.x, m_localAnchorB.y);
	b2Log("  jd.referenceAngle = %.15lef;\n", m_referenceAngle);
	b2Log("  jd.enableLimit = bool(%d);\n", m_enableLimit);
	b2Log("  jd.lowerAngle = %.15lef;\n", m_lowerAngle);
	b2Log("  jd.upperAngle = %.15lef;\n", m_upperAngle);
	b2Log("  jd.enableMotor = bool(%d);\n", m_enableMotor);
	b2Log("  jd.motorSpeed = %.15lef;\n", m_motorSpeed);
	b2Log("  jd.maxMotorTorque = %.15lef;\n", m_maxMotorTorque);
	b2Log("  joints[%d] = m_world->CreateJoint(&jd);\n", m_index);
}
