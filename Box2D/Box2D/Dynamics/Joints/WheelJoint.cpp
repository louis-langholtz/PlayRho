/*
* Original work Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
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

#include <Box2D/Dynamics/Joints/WheelJoint.hpp>
#include <Box2D/Dynamics/Body.hpp>
#include <Box2D/Dynamics/StepConf.hpp>
#include <Box2D/Dynamics/Contacts/ContactSolver.hpp>
#include <Box2D/Dynamics/Contacts/BodyConstraint.hpp>

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

WheelJoint::WheelJoint(const WheelJointDef& def):
	Joint(def),
	m_localAnchorA(def.localAnchorA),
	m_localAnchorB(def.localAnchorB),
	m_localXAxisA(def.localAxisA),
	m_localYAxisA(GetRevPerpendicular(m_localXAxisA)),
	m_maxMotorTorque(def.maxMotorTorque),
	m_motorSpeed(def.motorSpeed),
	m_enableMotor(def.enableMotor),
	m_frequencyHz(def.frequencyHz),
	m_dampingRatio(def.dampingRatio)
{
	// Intentionally empty.
}

void WheelJoint::InitVelocityConstraints(BodyConstraints& bodies, const StepConf& step, const ConstraintSolverConf&)
{
	auto& bodiesA = bodies.at(GetBodyA());
	auto& bodiesB = bodies.at(GetBodyB());

	m_localCenterA = bodiesA.GetLocalCenter();
	m_invMassA = RealNum{bodiesA.GetInvMass() * Kilogram};
	m_invIA = bodiesA.GetInvRotInertia() * (SquareMeter * Kilogram / SquareRadian);
	const auto posA = bodiesA.GetPosition();
	auto velA = bodiesA.GetVelocity();
	const auto invMassA = m_invMassA;
	const auto iA = m_invIA;

	m_localCenterB = bodiesB.GetLocalCenter();
	m_invMassB = RealNum{bodiesB.GetInvMass() * Kilogram};
	m_invIB = bodiesB.GetInvRotInertia() * (SquareMeter * Kilogram / SquareRadian);
	const auto posB = bodiesB.GetPosition();
	auto velB = bodiesB.GetVelocity();
	const auto invMassB = m_invMassB;
	const auto iB = m_invIB;

	const auto qA = UnitVec2{posA.angular};
	const auto qB = UnitVec2{posB.angular};

	// Compute the effective masses.
	const auto rA = Rotate(m_localAnchorA - m_localCenterA, qA);
	const auto rB = Rotate(m_localAnchorB - m_localCenterB, qB);
	const auto dd = posB.linear + rB - posA.linear - rA;

	// Point to line constraint
	{
		m_ay = Rotate(m_localYAxisA, qA);
		m_sAy = Cross(dd + rA, m_ay);
		m_sBy = Cross(rB, m_ay);

		const auto invMass = invMassA + invMassB + iA * m_sAy * m_sAy + iB * m_sBy * m_sBy;

		m_mass = (invMass > 0)? 1 / invMass: 0;
	}

	// Spring constraint
	m_springMass = 0;
	m_bias = 0;
	m_gamma = 0;
	if (m_frequencyHz > 0)
	{
		m_ax = Rotate(m_localXAxisA, qA);
		m_sAx = Cross(dd + rA, m_ax);
		m_sBx = Cross(rB, m_ax);

		const auto invMass = invMassA + invMassB + iA * m_sAx * m_sAx + iB * m_sBx * m_sBx;

		if (invMass > 0)
		{
			m_springMass = RealNum{1} / invMass;

			const auto C = Dot(dd, m_ax);

			// Frequency
			const auto omega = 2 * Pi * m_frequencyHz;

			// Damping coefficient
			const auto d = 2 * m_springMass * m_dampingRatio * omega;

			// Spring stiffness
			const auto k = m_springMass * omega * omega;

			// magic formulas
			const auto h = RealNum{step.GetTime() / Second};
			m_gamma = h * (d + h * k);
			if (m_gamma > 0)
			{
				m_gamma = 1 / m_gamma;
			}

			m_bias = C * h * k * m_gamma;

			m_springMass = invMass + m_gamma;
			if (m_springMass > 0)
			{
				m_springMass = 1 / m_springMass;
			}
		}
	}
	else
	{
		m_springImpulse = 0;
	}

	// Rotational motor
	if (m_enableMotor)
	{
		m_motorMass = iA + iB;
		if (m_motorMass > 0)
		{
			m_motorMass = 1 / m_motorMass;
		}
	}
	else
	{
		m_motorMass = 0;
		m_motorImpulse = 0;
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

		velA -= Velocity{m_invMassA * P, RadianPerSecond * m_invIA * LA};
		velB += Velocity{m_invMassB * P, RadianPerSecond * m_invIB * LB};
	}
	else
	{
		m_impulse = 0;
		m_springImpulse = 0;
		m_motorImpulse = 0;
	}

	bodiesA.SetVelocity(velA);
	bodiesB.SetVelocity(velB);
}

RealNum WheelJoint::SolveVelocityConstraints(BodyConstraints& bodies, const StepConf& step)
{
	auto& bodiesA = bodies.at(GetBodyA());
	auto& bodiesB = bodies.at(GetBodyB());

	auto velA = bodiesA.GetVelocity();
	const auto invMassA = m_invMassA;
	const auto iA = m_invIA;

	auto velB = bodiesB.GetVelocity();
	const auto invMassB = m_invMassB;
	const auto iB = m_invIB;

	// Solve spring constraint
	{
		const auto Cdot = Dot(m_ax, velB.linear - velA.linear) + m_sBx * RealNum{velB.angular / RadianPerSecond} - m_sAx * RealNum{velA.angular / RadianPerSecond};
		const auto impulse = -m_springMass * (Cdot + m_bias + m_gamma * m_springImpulse);
		m_springImpulse += impulse;

		const auto P = impulse * m_ax;
		const auto LA = impulse * m_sAx;
		const auto LB = impulse * m_sBx;

		velA -= Velocity{invMassA * P, RadianPerSecond * iA * LA};
		velB += Velocity{invMassB * P, RadianPerSecond * iB * LB};
	}

	// Solve rotational motor constraint
	{
		const auto Cdot = RealNum{(velB.angular - velA.angular - m_motorSpeed) / RadianPerSecond};
		auto impulse = -m_motorMass * Cdot;

		const auto oldImpulse = m_motorImpulse;
		const auto maxImpulse = RealNum{step.GetTime() / Second} * m_maxMotorTorque;
		m_motorImpulse = Clamp(m_motorImpulse + impulse, -maxImpulse, maxImpulse);
		impulse = m_motorImpulse - oldImpulse;

		velA.angular -= RadianPerSecond * iA * impulse;
		velB.angular += RadianPerSecond * iB * impulse;
	}

	// Solve point to line constraint
	{
		const auto Cdot = Dot(m_ay, velB.linear - velA.linear) + m_sBy * RealNum{velB.angular / RadianPerSecond} - m_sAy * RealNum{velA.angular / RadianPerSecond};
		const auto impulse = -m_mass * Cdot;
		m_impulse += impulse;

		const auto P = impulse * m_ay;
		const auto LA = impulse * m_sAy;
		const auto LB = impulse * m_sBy;

		velA -= Velocity{invMassA * P, RadianPerSecond * iA * LA};
		velB += Velocity{invMassB * P, RadianPerSecond * iB * LB};
	}

	bodiesA.SetVelocity(velA);
	bodiesB.SetVelocity(velB);
	
	return GetInvalid<RealNum>();
}

bool WheelJoint::SolvePositionConstraints(BodyConstraints& bodies, const ConstraintSolverConf& conf) const
{
	auto& bodiesA = bodies.at(GetBodyA());
	auto& bodiesB = bodies.at(GetBodyB());

	auto posA = bodiesA.GetPosition();
	auto posB = bodiesB.GetPosition();

	const auto qA = UnitVec2{posA.angular};
	const auto qB = UnitVec2{posB.angular};

	const auto rA = Rotate(m_localAnchorA - m_localCenterA, qA);
	const auto rB = Rotate(m_localAnchorB - m_localCenterB, qB);
	const auto d = (posB.linear - posA.linear) + rB - rA;

	const auto ay = Rotate(m_localYAxisA, qA);

	const auto sAy = Cross(d + rA, ay);
	const auto sBy = Cross(rB, ay);

	const auto C = Dot(d, ay);

	const auto k = m_invMassA + m_invMassB + m_invIA * m_sAy * m_sAy + m_invIB * m_sBy * m_sBy;

	const auto impulse = (k != 0)? - C / k: RealNum{0};

	const auto P = impulse * ay;
	const auto LA = impulse * sAy;
	const auto LB = impulse * sBy;

	posA -= Position{m_invMassA * P, Radian * m_invIA * LA};
	posB += Position{m_invMassB * P, Radian * m_invIB * LB};

	bodiesA.SetPosition(posA);
	bodiesB.SetPosition(posB);

	return Abs(C) <= conf.linearSlop;
}

Vec2 WheelJoint::GetAnchorA() const
{
	return GetWorldPoint(*GetBodyA(), GetLocalAnchorA());
}

Vec2 WheelJoint::GetAnchorB() const
{
	return GetWorldPoint(*GetBodyB(), GetLocalAnchorB());
}

Vec2 WheelJoint::GetReactionForce(Frequency inv_dt) const
{
	return RealNum{inv_dt / Hertz} * (m_impulse * m_ay + m_springImpulse * m_ax);
}

RealNum WheelJoint::GetReactionTorque(Frequency inv_dt) const
{
	return RealNum{inv_dt / Hertz} * m_motorImpulse;
}

RealNum WheelJoint::GetJointTranslation() const
{
	const auto pA = GetWorldPoint(*GetBodyA(), GetLocalAnchorA());
	const auto pB = GetWorldPoint(*GetBodyB(), GetLocalAnchorB());
	const auto d = pB - pA;
	const auto axis = GetWorldVector(*GetBodyA(), m_localXAxisA);
	return Dot(d, axis);
}

AngularVelocity WheelJoint::GetJointSpeed() const
{
	return GetBodyB()->GetVelocity().angular - GetBodyA()->GetVelocity().angular;
}

void WheelJoint::EnableMotor(bool flag)
{
	GetBodyA()->SetAwake();
	GetBodyB()->SetAwake();
	m_enableMotor = flag;
}

void WheelJoint::SetMotorSpeed(AngularVelocity speed)
{
	GetBodyA()->SetAwake();
	GetBodyB()->SetAwake();
	m_motorSpeed = speed;
}

void WheelJoint::SetMaxMotorTorque(RealNum torque)
{
	GetBodyA()->SetAwake();
	GetBodyB()->SetAwake();
	m_maxMotorTorque = torque;
}

RealNum WheelJoint::GetMotorTorque(RealNum inv_dt) const
{
	return inv_dt * m_motorImpulse;
}
