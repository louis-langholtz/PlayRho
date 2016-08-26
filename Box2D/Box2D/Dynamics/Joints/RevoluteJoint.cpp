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

#include <Box2D/Dynamics/Joints/RevoluteJoint.h>
#include <Box2D/Dynamics/Body.h>
#include <Box2D/Dynamics/TimeStep.h>

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

RevoluteJointDef::RevoluteJointDef(Body* bA, Body* bB, const Vec2& anchor, bool cc):
	JointDef{JointType::Revolute, bA, bB, cc},
	localAnchorA{GetLocalPoint(*bA, anchor)},
	localAnchorB{GetLocalPoint(*bB, anchor)},
	referenceAngle{bB->GetAngle() - bA->GetAngle()}
{
}

RevoluteJoint::RevoluteJoint(const RevoluteJointDef& def)
: Joint(def)
{
	m_localAnchorA = def.localAnchorA;
	m_localAnchorB = def.localAnchorB;
	m_referenceAngle = def.referenceAngle;

	m_impulse = Vec3_zero;
	m_motorImpulse = float_t{0};

	m_lowerAngle = def.lowerAngle;
	m_upperAngle = def.upperAngle;
	m_maxMotorTorque = def.maxMotorTorque;
	m_motorSpeed = def.motorSpeed;
	m_enableLimit = def.enableLimit;
	m_enableMotor = def.enableMotor;
	m_limitState = e_inactiveLimit;
}

void RevoluteJoint::InitVelocityConstraints(const SolverData& data)
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

	const Rot qA(aA), qB(aB);

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
		if (Abs(m_upperAngle - m_lowerAngle) < (float_t{2} * AngularSlop))
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

		const auto P = Vec2{m_impulse.x, m_impulse.y};

		vA -= mA * P;
		wA -= iA * (Cross(m_rA, P) + m_motorImpulse + m_impulse.z);

		vB += mB * P;
		wB += iB * (Cross(m_rB, P) + m_motorImpulse + m_impulse.z);
	}
	else
	{
		m_impulse = Vec3_zero;
		m_motorImpulse = float_t{0};
	}

	data.velocities[m_indexA].v = vA;
	data.velocities[m_indexA].w = wA;
	data.velocities[m_indexB].v = vB;
	data.velocities[m_indexB].w = wB;
}

void RevoluteJoint::SolveVelocityConstraints(const SolverData& data)
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
		m_motorImpulse = Clamp(m_motorImpulse + impulse, -maxImpulse, maxImpulse);
		impulse = m_motorImpulse - oldImpulse;

		wA -= iA * impulse;
		wB += iB * impulse;
	}

	// Solve limit constraint.
	if (m_enableLimit && (m_limitState != e_inactiveLimit) && !fixedRotation)
	{
		const auto Cdot1 = vB + (GetReversePerpendicular(m_rB) * wB) - vA - (GetReversePerpendicular(m_rA) * wA);
		const auto Cdot2 = wB - wA;
		const auto Cdot = Vec3(Cdot1.x, Cdot1.y, Cdot2);

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
				const auto rhs = -Cdot1 + m_impulse.z * Vec2{m_mass.ez.x, m_mass.ez.y};
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
				const auto rhs = -Cdot1 + m_impulse.z * Vec2{m_mass.ez.x, m_mass.ez.y};
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

		const auto P = Vec2{impulse.x, impulse.y};

		vA -= mA * P;
		wA -= iA * (Cross(m_rA, P) + impulse.z);

		vB += mB * P;
		wB += iB * (Cross(m_rB, P) + impulse.z);
	}
	else
	{
		// Solve point-to-point constraint
		const auto Cdot = vB + (GetReversePerpendicular(m_rB) * wB) - vA - (GetReversePerpendicular(m_rA) * wA);
		const auto impulse = m_mass.Solve22(-Cdot);

		m_impulse.x += impulse.x;
		m_impulse.y += impulse.y;

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

bool RevoluteJoint::SolvePositionConstraints(const SolverData& data)
{
	auto cA = data.positions[m_indexA].c;
	auto aA = data.positions[m_indexA].a;
	auto cB = data.positions[m_indexB].c;
	auto aB = data.positions[m_indexB].a;

	auto qA = Rot(aA);
	auto qB = Rot(aB);

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
			const auto C = Clamp(angle - m_lowerAngle, -MaxAngularCorrection, MaxAngularCorrection);
			limitImpulse = -m_motorMass * C;
			angularError = Abs(C);
		}
		else if (m_limitState == e_atLowerLimit)
		{
			auto C = angle - m_lowerAngle;
			angularError = -C;

			// Prevent large angular corrections and allow some slop.
			C = Clamp(C + AngularSlop, -MaxAngularCorrection, float_t{0});
			limitImpulse = -m_motorMass * C;
		}
		else if (m_limitState == e_atUpperLimit)
		{
			auto C = angle - m_upperAngle;
			angularError = C;

			// Prevent large angular corrections and allow some slop.
			C = Clamp(C - AngularSlop, float_t{0}, MaxAngularCorrection);
			limitImpulse = -m_motorMass * C;
		}

		aA -= m_invIA * limitImpulse;
		aB += m_invIB * limitImpulse;
	}

	// Solve point-to-point constraint.
	{
		qA = Rot(aA);
		qB = Rot(aB);
		const auto rA = Rotate(m_localAnchorA - m_localCenterA, qA);
		const auto rB = Rotate(m_localAnchorB - m_localCenterB, qB);

		const auto C = cB + rB - cA - rA;
		positionError = Length(C);

		const auto mA = m_invMassA, mB = m_invMassB;
		const auto iA = m_invIA, iB = m_invIB;

		Mat22 K;
		K.ex.x = mA + mB + iA * rA.y * rA.y + iB * rB.y * rB.y;
		K.ex.y = -iA * rA.x * rA.y - iB * rB.x * rB.y;
		K.ey.x = K.ex.y;
		K.ey.y = mA + mB + iA * rA.x * rA.x + iB * rB.x * rB.x;

		const auto impulse = -K.Solve(C);

		cA -= mA * impulse;
		aA -= iA * Cross(rA, impulse);

		cB += mB * impulse;
		aB += iB * Cross(rB, impulse);
	}

	data.positions[m_indexA].c = cA;
	data.positions[m_indexA].a = aA;
	data.positions[m_indexB].c = cB;
	data.positions[m_indexB].a = aB;
	
	return (positionError <= LinearSlop) && (angularError <= AngularSlop);
}

Vec2 RevoluteJoint::GetAnchorA() const
{
	return GetWorldPoint(*m_bodyA, m_localAnchorA);
}

Vec2 RevoluteJoint::GetAnchorB() const
{
	return GetWorldPoint(*m_bodyB, m_localAnchorB);
}

Vec2 RevoluteJoint::GetReactionForce(float_t inv_dt) const
{
	return inv_dt * Vec2{m_impulse.x, m_impulse.y};
}

float_t RevoluteJoint::GetReactionTorque(float_t inv_dt) const
{
	return inv_dt * m_impulse.z;
}

float_t RevoluteJoint::GetJointAngle() const
{
	return m_bodyB->GetAngle() - m_bodyA->GetAngle() - m_referenceAngle;
}

float_t RevoluteJoint::GetJointSpeed() const
{
	return m_bodyB->GetVelocity().w - m_bodyA->GetVelocity().w;
}

bool RevoluteJoint::IsMotorEnabled() const
{
	return m_enableMotor;
}

void RevoluteJoint::EnableMotor(bool flag)
{
	m_bodyA->SetAwake();
	m_bodyB->SetAwake();
	m_enableMotor = flag;
}

float_t RevoluteJoint::GetMotorTorque(float_t inv_dt) const
{
	return inv_dt * m_motorImpulse;
}

void RevoluteJoint::SetMotorSpeed(float_t speed)
{
	m_bodyA->SetAwake();
	m_bodyB->SetAwake();
	m_motorSpeed = speed;
}

void RevoluteJoint::SetMaxMotorTorque(float_t torque)
{
	m_bodyA->SetAwake();
	m_bodyB->SetAwake();
	m_maxMotorTorque = torque;
}

bool RevoluteJoint::IsLimitEnabled() const
{
	return m_enableLimit;
}

void RevoluteJoint::EnableLimit(bool flag)
{
	if (flag != m_enableLimit)
	{
		m_bodyA->SetAwake();
		m_bodyB->SetAwake();
		m_enableLimit = flag;
		m_impulse.z = float_t{0};
	}
}

float_t RevoluteJoint::GetLowerLimit() const
{
	return m_lowerAngle;
}

float_t RevoluteJoint::GetUpperLimit() const
{
	return m_upperAngle;
}

void RevoluteJoint::SetLimits(float_t lower, float_t upper)
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

void box2d::Dump(const RevoluteJoint& joint, size_t index)
{
	log("  RevoluteJointDef jd;\n");
	log("  jd.bodyA = bodies[%d];\n", GetWorldIndex(joint.GetBodyA()));
	log("  jd.bodyB = bodies[%d];\n", GetWorldIndex(joint.GetBodyB()));
	log("  jd.collideConnected = bool(%d);\n", joint.GetCollideConnected());
	log("  jd.localAnchorA = Vec2(%.15lef, %.15lef);\n", joint.GetLocalAnchorA().x, joint.GetLocalAnchorA().y);
	log("  jd.localAnchorB = Vec2(%.15lef, %.15lef);\n", joint.GetLocalAnchorB().x, joint.GetLocalAnchorB().y);
	log("  jd.referenceAngle = %.15lef;\n", joint.GetReferenceAngle());
	log("  jd.enableLimit = bool(%d);\n", joint.IsLimitEnabled());
	log("  jd.lowerAngle = %.15lef;\n", joint.GetLowerLimit());
	log("  jd.upperAngle = %.15lef;\n", joint.GetUpperLimit());
	log("  jd.enableMotor = bool(%d);\n", joint.IsMotorEnabled());
	log("  jd.motorSpeed = %.15lef;\n", joint.GetMotorSpeed());
	log("  jd.maxMotorTorque = %.15lef;\n", joint.GetMaxMotorTorque());
	log("  joints[%d] = m_world->CreateJoint(jd);\n", index);
}
