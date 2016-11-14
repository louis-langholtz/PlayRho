/*
* Original work Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
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

#include <Box2D/Dynamics/Joints/WheelJoint.h>
#include <Box2D/Dynamics/Body.h>
#include <Box2D/Dynamics/TimeStep.h>
#include <Box2D/Dynamics/Contacts/ContactSolver.h>

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

void WheelJointDef::Initialize(Body* bA, Body* bB, const Vec2& anchor, const Vec2& axis)
{
	bodyA = bA;
	bodyB = bB;
	localAnchorA = GetLocalPoint(*bodyA, anchor);
	localAnchorB = GetLocalPoint(*bodyB, anchor);
	localAxisA = GetLocalVector(*bodyA, axis);
}

WheelJoint::WheelJoint(const WheelJointDef& def)
: Joint(def)
{
	m_localAnchorA = def.localAnchorA;
	m_localAnchorB = def.localAnchorB;
	m_localXAxisA = def.localAxisA;
	m_localYAxisA = GetRevPerpendicular(m_localXAxisA);

	m_mass = float_t{0};
	m_impulse = float_t{0};
	m_motorMass = float_t{0};
	m_motorImpulse = float_t{0};
	m_springMass = float_t{0};
	m_springImpulse = float_t{0};

	m_maxMotorTorque = def.maxMotorTorque;
	m_motorSpeed = def.motorSpeed;
	m_enableMotor = def.enableMotor;

	m_frequencyHz = def.frequencyHz;
	m_dampingRatio = def.dampingRatio;

	m_bias = float_t{0};
	m_gamma = float_t{0};

	m_ax = Vec2_zero;
	m_ay = Vec2_zero;
}

void WheelJoint::InitVelocityConstraints(Span<Velocity> velocities, Span<const Position> positions, const TimeStep& step, const ConstraintSolverConf& conf)
{
	m_indexA = GetBodyA()->GetIslandIndex();
	m_indexB = GetBodyB()->GetIslandIndex();
	m_localCenterA = GetBodyA()->GetLocalCenter();
	m_localCenterB = GetBodyB()->GetLocalCenter();
	m_invMassA = GetBodyA()->GetInverseMass();
	m_invMassB = GetBodyB()->GetInverseMass();
	m_invIA = GetBodyA()->GetInverseInertia();
	m_invIB = GetBodyB()->GetInverseInertia();

	const auto mA = m_invMassA, mB = m_invMassB;
	const auto iA = m_invIA, iB = m_invIB;

	const auto cA = positions[m_indexA].c;
	const auto aA = positions[m_indexA].a;
	auto vA = velocities[m_indexA].v;
	auto wA = velocities[m_indexA].w;

	const auto cB = positions[m_indexB].c;
	const auto aB = positions[m_indexB].a;
	auto vB = velocities[m_indexB].v;
	auto wB = velocities[m_indexB].w;

	const UnitVec2 qA(aA), qB(aB);

	// Compute the effective masses.
	const auto rA = Rotate(m_localAnchorA - m_localCenterA, qA);
	const auto rB = Rotate(m_localAnchorB - m_localCenterB, qB);
	const auto dd = cB + rB - cA - rA;

	// Point to line constraint
	{
		m_ay = Rotate(m_localYAxisA, qA);
		m_sAy = Cross(dd + rA, m_ay);
		m_sBy = Cross(rB, m_ay);

		m_mass = mA + mB + iA * m_sAy * m_sAy + iB * m_sBy * m_sBy;

		if (m_mass > float_t{0})
		{
			m_mass = float_t{1} / m_mass;
		}
	}

	// Spring constraint
	m_springMass = float_t{0};
	m_bias = float_t{0};
	m_gamma = float_t{0};
	if (m_frequencyHz > float_t{0})
	{
		m_ax = Rotate(m_localXAxisA, qA);
		m_sAx = Cross(dd + rA, m_ax);
		m_sBx = Cross(rB, m_ax);

		const auto invMass = mA + mB + iA * m_sAx * m_sAx + iB * m_sBx * m_sBx;

		if (invMass > float_t{0})
		{
			m_springMass = float_t{1} / invMass;

			const auto C = Dot(dd, m_ax);

			// Frequency
			const auto omega = float_t(2) * Pi * m_frequencyHz;

			// Damping coefficient
			const auto d = float_t(2) * m_springMass * m_dampingRatio * omega;

			// Spring stiffness
			const auto k = m_springMass * omega * omega;

			// magic formulas
			const auto h = step.get_dt();
			m_gamma = h * (d + h * k);
			if (m_gamma > float_t{0})
			{
				m_gamma = float_t{1} / m_gamma;
			}

			m_bias = C * h * k * m_gamma;

			m_springMass = invMass + m_gamma;
			if (m_springMass > float_t{0})
			{
				m_springMass = float_t{1} / m_springMass;
			}
		}
	}
	else
	{
		m_springImpulse = float_t{0};
	}

	// Rotational motor
	if (m_enableMotor)
	{
		m_motorMass = iA + iB;
		if (m_motorMass > float_t{0})
		{
			m_motorMass = float_t{1} / m_motorMass;
		}
	}
	else
	{
		m_motorMass = float_t{0};
		m_motorImpulse = float_t{0};
	}

	if (step.warmStarting)
	{
		// Account for variable time step.
		m_impulse *= step.dtRatio;
		m_springImpulse *= step.dtRatio;
		m_motorImpulse *= step.dtRatio;

		const auto P = m_impulse * m_ay + m_springImpulse * m_ax;
		const auto LA = m_impulse * m_sAy + m_springImpulse * m_sAx + m_motorImpulse;
		const auto LB = m_impulse * m_sBy + m_springImpulse * m_sBx + m_motorImpulse;

		vA -= m_invMassA * P;
		wA -= 1_rad * m_invIA * LA;

		vB += m_invMassB * P;
		wB += 1_rad * m_invIB * LB;
	}
	else
	{
		m_impulse = float_t{0};
		m_springImpulse = float_t{0};
		m_motorImpulse = float_t{0};
	}

	velocities[m_indexA].v = vA;
	velocities[m_indexA].w = wA;
	velocities[m_indexB].v = vB;
	velocities[m_indexB].w = wB;
}

void WheelJoint::SolveVelocityConstraints(Span<Velocity> velocities, const TimeStep& step)
{
	const auto mA = m_invMassA, mB = m_invMassB;
	const auto iA = m_invIA, iB = m_invIB;

	auto vA = velocities[m_indexA].v;
	auto wA = velocities[m_indexA].w;
	auto vB = velocities[m_indexB].v;
	auto wB = velocities[m_indexB].w;

	// Solve spring constraint
	{
		const auto Cdot = Dot(m_ax, vB - vA) + m_sBx * wB.ToRadians() - m_sAx * wA.ToRadians();
		const auto impulse = -m_springMass * (Cdot + m_bias + m_gamma * m_springImpulse);
		m_springImpulse += impulse;

		const auto P = impulse * m_ax;
		const auto LA = impulse * m_sAx;
		const auto LB = impulse * m_sBx;

		vA -= mA * P;
		wA -= 1_rad * iA * LA;

		vB += mB * P;
		wB += 1_rad * iB * LB;
	}

	// Solve rotational motor constraint
	{
		const auto Cdot = wB - wA - m_motorSpeed;
		auto impulse = -m_motorMass * Cdot.ToRadians();

		const auto oldImpulse = m_motorImpulse;
		const auto maxImpulse = step.get_dt() * m_maxMotorTorque;
		m_motorImpulse = Clamp(m_motorImpulse + impulse, -maxImpulse, maxImpulse);
		impulse = m_motorImpulse - oldImpulse;

		wA -= 1_rad * iA * impulse;
		wB += 1_rad * iB * impulse;
	}

	// Solve point to line constraint
	{
		const auto Cdot = Dot(m_ay, vB - vA) + m_sBy * wB.ToRadians() - m_sAy * wA.ToRadians();
		const auto impulse = -m_mass * Cdot;
		m_impulse += impulse;

		const auto P = impulse * m_ay;
		const auto LA = impulse * m_sAy;
		const auto LB = impulse * m_sBy;

		vA -= mA * P;
		wA -= 1_rad * iA * LA;

		vB += mB * P;
		wB += 1_rad * iB * LB;
	}

	velocities[m_indexA].v = vA;
	velocities[m_indexA].w = wA;
	velocities[m_indexB].v = vB;
	velocities[m_indexB].w = wB;
}

bool WheelJoint::SolvePositionConstraints(Span<Position> positions, const ConstraintSolverConf& conf)
{
	auto cA = positions[m_indexA].c;
	auto aA = positions[m_indexA].a;
	auto cB = positions[m_indexB].c;
	auto aB = positions[m_indexB].a;

	const auto qA = UnitVec2{aA};
	const auto qB = UnitVec2{aB};

	const auto rA = Rotate(m_localAnchorA - m_localCenterA, qA);
	const auto rB = Rotate(m_localAnchorB - m_localCenterB, qB);
	const auto d = (cB - cA) + rB - rA;

	const auto ay = Rotate(m_localYAxisA, qA);

	const auto sAy = Cross(d + rA, ay);
	const auto sBy = Cross(rB, ay);

	const auto C = Dot(d, ay);

	const auto k = m_invMassA + m_invMassB + m_invIA * m_sAy * m_sAy + m_invIB * m_sBy * m_sBy;

	const auto impulse = (k != float_t{0})? - C / k: float_t{0};

	const auto P = impulse * ay;
	const auto LA = impulse * sAy;
	const auto LB = impulse * sBy;

	cA -= m_invMassA * P;
	aA -= 1_rad * m_invIA * LA;
	cB += m_invMassB * P;
	aB += 1_rad * m_invIB * LB;

	positions[m_indexA].c = cA;
	positions[m_indexA].a = aA;
	positions[m_indexB].c = cB;
	positions[m_indexB].a = aB;

	return Abs(C) <= conf.linearSlop;
}

Vec2 WheelJoint::GetAnchorA() const
{
	return GetWorldPoint(*GetBodyA(), m_localAnchorA);
}

Vec2 WheelJoint::GetAnchorB() const
{
	return GetWorldPoint(*GetBodyB(), m_localAnchorB);
}

Vec2 WheelJoint::GetReactionForce(float_t inv_dt) const
{
	return inv_dt * (m_impulse * m_ay + m_springImpulse * m_ax);
}

float_t WheelJoint::GetReactionTorque(float_t inv_dt) const
{
	return inv_dt * m_motorImpulse;
}

float_t WheelJoint::GetJointTranslation() const
{
	const auto pA = GetWorldPoint(*GetBodyA(), m_localAnchorA);
	const auto pB = GetWorldPoint(*GetBodyB(), m_localAnchorB);
	const auto d = pB - pA;
	const auto axis = GetWorldVector(*GetBodyA(), m_localXAxisA);
	return Dot(d, axis);
}

Angle WheelJoint::GetJointSpeed() const
{
	return GetBodyB()->GetVelocity().w - GetBodyA()->GetVelocity().w;
}

void WheelJoint::EnableMotor(bool flag)
{
	GetBodyA()->SetAwake();
	GetBodyB()->SetAwake();
	m_enableMotor = flag;
}

void WheelJoint::SetMotorSpeed(Angle speed)
{
	GetBodyA()->SetAwake();
	GetBodyB()->SetAwake();
	m_motorSpeed = speed;
}

void WheelJoint::SetMaxMotorTorque(float_t torque)
{
	GetBodyA()->SetAwake();
	GetBodyB()->SetAwake();
	m_maxMotorTorque = torque;
}

float_t WheelJoint::GetMotorTorque(float_t inv_dt) const
{
	return inv_dt * m_motorImpulse;
}

void box2d::Dump(const WheelJoint& joint, size_t index)
{
	log("  WheelJointDef jd;\n");
	log("  jd.bodyA = bodies[%d];\n", GetWorldIndex(joint.GetBodyA()));
	log("  jd.bodyB = bodies[%d];\n", GetWorldIndex(joint.GetBodyB()));
	log("  jd.collideConnected = bool(%d);\n", joint.GetCollideConnected());
	log("  jd.localAnchorA = Vec2(%.15lef, %.15lef);\n", joint.GetLocalAnchorA().x, joint.GetLocalAnchorA().y);
	log("  jd.localAnchorB = Vec2(%.15lef, %.15lef);\n", joint.GetLocalAnchorB().x, joint.GetLocalAnchorB().y);
	log("  jd.localAxisA = Vec2(%.15lef, %.15lef);\n", joint.GetLocalAxisA().x, joint.GetLocalAxisA().y);
	log("  jd.enableMotor = bool(%d);\n", joint.IsMotorEnabled());
	log("  jd.motorSpeed = %.15lef;\n", joint.GetMotorSpeed());
	log("  jd.maxMotorTorque = %.15lef;\n", joint.GetMaxMotorTorque());
	log("  jd.frequencyHz = %.15lef;\n", joint.GetSpringFrequencyHz());
	log("  jd.dampingRatio = %.15lef;\n", joint.GetSpringDampingRatio());
	log("  joints[%d] = m_world->CreateJoint(jd);\n", index);
}
