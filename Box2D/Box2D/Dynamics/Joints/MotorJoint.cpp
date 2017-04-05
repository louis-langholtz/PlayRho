/*
* Original work Copyright (c) 2006-2012 Erin Catto http://www.box2d.org
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

#include <Box2D/Dynamics/Joints/MotorJoint.hpp>
#include <Box2D/Dynamics/Body.hpp>
#include <Box2D/Dynamics/StepConf.hpp>
#include <Box2D/Dynamics/Contacts/BodyConstraint.hpp>

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
	linearOffset = GetLocalPoint(*bodyA, bodyB->GetLocation());
	angularOffset = bodyB->GetAngle() - bodyA->GetAngle();
}

MotorJoint::MotorJoint(const MotorJointDef& def):
	Joint(def),
	m_linearOffset(def.linearOffset),
	m_angularOffset(def.angularOffset),
	m_maxForce(def.maxForce),
	m_maxTorque(def.maxTorque),
	m_correctionFactor(def.correctionFactor)
{
	// Intentionally empty.
}

void MotorJoint::InitVelocityConstraints(BodyConstraints& bodies, const StepConf& step, const ConstraintSolverConf&)
{
	auto& bodiesA = bodies.at(GetBodyA());
	auto& bodiesB = bodies.at(GetBodyB());

	m_localCenterA = bodiesA.GetLocalCenter();
	m_invMassA = RealNum{bodiesA.GetInvMass() * Kilogram};
	m_invIA = bodiesA.GetInvRotInertia();

	m_localCenterB = bodiesB.GetLocalCenter();
	m_invMassB = RealNum{bodiesB.GetInvMass() * Kilogram};
	m_invIB = bodiesB.GetInvRotInertia();

	const auto posA = bodiesA.GetPosition();
	auto velA = bodiesA.GetVelocity();

	const auto posB = bodiesB.GetPosition();
	auto velB = bodiesB.GetVelocity();

	const auto qA = UnitVec2(posA.angular);
	const auto qB = UnitVec2(posB.angular);

	// Compute the effective mass matrix.
	m_rA = Rotate(-m_localCenterA, qA);
	m_rB = Rotate(-m_localCenterB, qB);

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

	m_linearMass = Invert(K);

	m_angularMass = iA + iB;
	if (m_angularMass > 0)
	{
		m_angularMass = RealNum{1} / m_angularMass;
	}

	m_linearError = posB.linear + m_rB - posA.linear - m_rA - Rotate(m_linearOffset, qA);
	m_angularError = posB.angular - posA.angular - m_angularOffset;

	if (step.doWarmStart)
	{
		// Scale impulses to support a variable time step.
		m_linearImpulse *= step.dtRatio;
		m_angularImpulse *= step.dtRatio;

		const auto P = Vec2{m_linearImpulse.x, m_linearImpulse.y};
		velA -= Velocity{mA * P, Radian * iA * (Cross(m_rA, P) + m_angularImpulse)};
		velB += Velocity{mB * P, Radian * iB * (Cross(m_rB, P) + m_angularImpulse)};
	}
	else
	{
		m_linearImpulse = Vec2_zero;
		m_angularImpulse = 0;
	}

	bodiesA.SetVelocity(velA);
	bodiesB.SetVelocity(velB);
}

RealNum MotorJoint::SolveVelocityConstraints(BodyConstraints& bodies, const StepConf& step)
{
	auto& bodiesA = bodies.at(GetBodyA());
	auto& bodiesB = bodies.at(GetBodyB());

	auto velA = bodiesA.GetVelocity();
	auto velB = bodiesB.GetVelocity();

	const auto mA = m_invMassA;
	const auto mB = m_invMassB;
	const auto iA = m_invIA;
	const auto iB = m_invIB;

	const auto h = RealNum{step.get_dt() / Second};
	const auto inv_h = RealNum{step.get_inv_dt() / Hertz};

	// Solve angular friction
	auto angularIncImpulse = RealNum(0);
	{
		const auto Cdot = RealNum{(velB.angular - velA.angular + inv_h * m_correctionFactor * m_angularError) / Radian};
		const auto impulse = -m_angularMass * Cdot;

		const auto oldImpulse = m_angularImpulse;
		const auto maxImpulse = h * m_maxTorque;
		m_angularImpulse = Clamp(m_angularImpulse + impulse, -maxImpulse, maxImpulse);
		angularIncImpulse = m_angularImpulse - oldImpulse;

		velA.angular -= Radian * iA * angularIncImpulse;
		velB.angular += Radian * iB * angularIncImpulse;
	}

	// Solve linear friction
	{
		const auto vb = velB.linear + (GetRevPerpendicular(m_rB) * RealNum{velB.angular / Radian});
		const auto va = velA.linear - (GetRevPerpendicular(m_rA) * RealNum{velA.angular / Radian});
		const auto Cdot = vb - va + inv_h * m_correctionFactor * m_linearError;

		auto impulse = -Transform(Cdot, m_linearMass);
		const auto oldImpulse = m_linearImpulse;
		m_linearImpulse += impulse;

		const auto maxImpulse = h * m_maxForce;

		if (GetLengthSquared(m_linearImpulse) > Square(maxImpulse))
		{
			m_linearImpulse = GetUnitVector(m_linearImpulse, UnitVec2::GetZero()) * maxImpulse;
		}

		impulse = m_linearImpulse - oldImpulse;

		velA -= Velocity{mA * impulse, Radian * iA * Cross(m_rA, impulse)};
		velB += Velocity{mB * impulse, Radian * iB * Cross(m_rB, impulse)};
	}

	bodiesA.SetVelocity(velA);
	bodiesB.SetVelocity(velB);
	
	return GetInvalid<RealNum>();
}

bool MotorJoint::SolvePositionConstraints(BodyConstraints& bodies, const ConstraintSolverConf& conf) const
{
	NOT_USED(bodies);
	NOT_USED(conf);

	return true;
}

Vec2 MotorJoint::GetAnchorA() const
{
	return GetBodyA()->GetLocation();
}

Vec2 MotorJoint::GetAnchorB() const
{
	return GetBodyB()->GetLocation();
}

Vec2 MotorJoint::GetReactionForce(RealNum inv_dt) const
{
	return inv_dt * m_linearImpulse;
}

RealNum MotorJoint::GetReactionTorque(RealNum inv_dt) const
{
	return inv_dt * m_angularImpulse;
}

void MotorJoint::SetMaxForce(RealNum force)
{
	assert(IsValid(force) && (force >= 0));
	m_maxForce = force;
}

RealNum MotorJoint::GetMaxForce() const
{
	return m_maxForce;
}

void MotorJoint::SetMaxTorque(RealNum torque)
{
	assert(IsValid(torque) && (torque >= 0));
	m_maxTorque = torque;
}

RealNum MotorJoint::GetMaxTorque() const
{
	return m_maxTorque;
}

void MotorJoint::SetCorrectionFactor(RealNum factor)
{
	assert(IsValid(factor) && (0 <= factor) && (factor <= RealNum{1}));
	m_correctionFactor = factor;
}

RealNum MotorJoint::GetCorrectionFactor() const
{
	return m_correctionFactor;
}

void MotorJoint::SetLinearOffset(const Vec2 linearOffset)
{
	if ((linearOffset.x != m_linearOffset.x) || (linearOffset.y != m_linearOffset.y))
	{
		GetBodyA()->SetAwake();
		GetBodyB()->SetAwake();
		m_linearOffset = linearOffset;
	}
}

const Vec2 MotorJoint::GetLinearOffset() const
{
	return m_linearOffset;
}

void MotorJoint::SetAngularOffset(Angle angularOffset)
{
	if (angularOffset != m_angularOffset)
	{
		GetBodyA()->SetAwake();
		GetBodyB()->SetAwake();
		m_angularOffset = angularOffset;
	}
}

Angle MotorJoint::GetAngularOffset() const
{
	return m_angularOffset;
}
