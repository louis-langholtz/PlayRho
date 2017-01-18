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

#include <Box2D/Dynamics/Joints/WheelJoint.hpp>
#include <Box2D/Dynamics/Body.hpp>
#include <Box2D/Dynamics/StepConf.hpp>
#include <Box2D/Dynamics/Contacts/ContactSolver.hpp>

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

void WheelJointDef::Initialize(Body* bA, Body* bB, const Vec2 anchor, const Vec2 axis)
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

	m_mass = realnum{0};
	m_impulse = realnum{0};
	m_motorMass = realnum{0};
	m_motorImpulse = realnum{0};
	m_springMass = realnum{0};
	m_springImpulse = realnum{0};

	m_maxMotorTorque = def.maxMotorTorque;
	m_motorSpeed = def.motorSpeed;
	m_enableMotor = def.enableMotor;

	m_frequencyHz = def.frequencyHz;
	m_dampingRatio = def.dampingRatio;

	m_bias = realnum{0};
	m_gamma = realnum{0};

	m_ax = Vec2_zero;
	m_ay = Vec2_zero;
}

void WheelJoint::InitVelocityConstraints(Span<Velocity> velocities, Span<const Position> positions, const StepConf& step, const ConstraintSolverConf& conf)
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

	const auto cA = positions[m_indexA].linear;
	const auto aA = positions[m_indexA].angular;
	auto vA = velocities[m_indexA].linear;
	auto wA = velocities[m_indexA].angular;

	const auto cB = positions[m_indexB].linear;
	const auto aB = positions[m_indexB].angular;
	auto vB = velocities[m_indexB].linear;
	auto wB = velocities[m_indexB].angular;

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

		if (m_mass > realnum{0})
		{
			m_mass = realnum{1} / m_mass;
		}
	}

	// Spring constraint
	m_springMass = realnum{0};
	m_bias = realnum{0};
	m_gamma = realnum{0};
	if (m_frequencyHz > realnum{0})
	{
		m_ax = Rotate(m_localXAxisA, qA);
		m_sAx = Cross(dd + rA, m_ax);
		m_sBx = Cross(rB, m_ax);

		const auto invMass = mA + mB + iA * m_sAx * m_sAx + iB * m_sBx * m_sBx;

		if (invMass > realnum{0})
		{
			m_springMass = realnum{1} / invMass;

			const auto C = Dot(dd, m_ax);

			// Frequency
			const auto omega = realnum(2) * Pi * m_frequencyHz;

			// Damping coefficient
			const auto d = realnum(2) * m_springMass * m_dampingRatio * omega;

			// Spring stiffness
			const auto k = m_springMass * omega * omega;

			// magic formulas
			const auto h = step.get_dt();
			m_gamma = h * (d + h * k);
			if (m_gamma > realnum{0})
			{
				m_gamma = realnum{1} / m_gamma;
			}

			m_bias = C * h * k * m_gamma;

			m_springMass = invMass + m_gamma;
			if (m_springMass > realnum{0})
			{
				m_springMass = realnum{1} / m_springMass;
			}
		}
	}
	else
	{
		m_springImpulse = realnum{0};
	}

	// Rotational motor
	if (m_enableMotor)
	{
		m_motorMass = iA + iB;
		if (m_motorMass > realnum{0})
		{
			m_motorMass = realnum{1} / m_motorMass;
		}
	}
	else
	{
		m_motorMass = realnum{0};
		m_motorImpulse = realnum{0};
	}

	if (step.doWarmStart)
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
		m_impulse = realnum{0};
		m_springImpulse = realnum{0};
		m_motorImpulse = realnum{0};
	}

	velocities[m_indexA].linear = vA;
	velocities[m_indexA].angular = wA;
	velocities[m_indexB].linear = vB;
	velocities[m_indexB].angular = wB;
}

void WheelJoint::SolveVelocityConstraints(Span<Velocity> velocities, const StepConf& step)
{
	const auto mA = m_invMassA, mB = m_invMassB;
	const auto iA = m_invIA, iB = m_invIB;

	auto vA = velocities[m_indexA].linear;
	auto wA = velocities[m_indexA].angular;
	auto vB = velocities[m_indexB].linear;
	auto wB = velocities[m_indexB].angular;

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

	velocities[m_indexA].linear = vA;
	velocities[m_indexA].angular = wA;
	velocities[m_indexB].linear = vB;
	velocities[m_indexB].angular = wB;
}

bool WheelJoint::SolvePositionConstraints(Span<Position> positions, const ConstraintSolverConf& conf)
{
	auto cA = positions[m_indexA].linear;
	auto aA = positions[m_indexA].angular;
	auto cB = positions[m_indexB].linear;
	auto aB = positions[m_indexB].angular;

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

	const auto impulse = (k != realnum{0})? - C / k: realnum{0};

	const auto P = impulse * ay;
	const auto LA = impulse * sAy;
	const auto LB = impulse * sBy;

	cA -= m_invMassA * P;
	aA -= 1_rad * m_invIA * LA;
	cB += m_invMassB * P;
	aB += 1_rad * m_invIB * LB;

	positions[m_indexA].linear = cA;
	positions[m_indexA].angular = aA;
	positions[m_indexB].linear = cB;
	positions[m_indexB].angular = aB;

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

Vec2 WheelJoint::GetReactionForce(realnum inv_dt) const
{
	return inv_dt * (m_impulse * m_ay + m_springImpulse * m_ax);
}

realnum WheelJoint::GetReactionTorque(realnum inv_dt) const
{
	return inv_dt * m_motorImpulse;
}

realnum WheelJoint::GetJointTranslation() const
{
	const auto pA = GetWorldPoint(*GetBodyA(), m_localAnchorA);
	const auto pB = GetWorldPoint(*GetBodyB(), m_localAnchorB);
	const auto d = pB - pA;
	const auto axis = GetWorldVector(*GetBodyA(), m_localXAxisA);
	return Dot(d, axis);
}

Angle WheelJoint::GetJointSpeed() const
{
	return GetBodyB()->GetVelocity().angular - GetBodyA()->GetVelocity().angular;
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

void WheelJoint::SetMaxMotorTorque(realnum torque)
{
	GetBodyA()->SetAwake();
	GetBodyB()->SetAwake();
	m_maxMotorTorque = torque;
}

realnum WheelJoint::GetMotorTorque(realnum inv_dt) const
{
	return inv_dt * m_motorImpulse;
}
