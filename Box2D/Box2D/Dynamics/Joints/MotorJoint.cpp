/*
* Original work Copyright (c) 2006-2012 Erin Catto http://www.box2d.org
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

#include <Box2D/Dynamics/Joints/MotorJoint.hpp>
#include <Box2D/Dynamics/Body.hpp>
#include <Box2D/Dynamics/StepConf.hpp>

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

MotorJoint::MotorJoint(const MotorJointDef& def)
: Joint{def}
{
	m_linearOffset = def.linearOffset;
	m_angularOffset = def.angularOffset;

	m_linearImpulse = Vec2_zero;
	m_angularImpulse = RealNum{0};

	m_maxForce = def.maxForce;
	m_maxTorque = def.maxTorque;
	m_correctionFactor = def.correctionFactor;
}

void MotorJoint::InitVelocityConstraints(Span<Velocity> velocities, Span<const Position> positions, const StepConf& step, const ConstraintSolverConf&)
{
	m_indexA = GetBodyA()->GetIslandIndex();
	m_localCenterA = GetBodyA()->GetLocalCenter();
	m_invMassA = GetBodyA()->GetInverseMass();
	m_invIA = GetBodyA()->GetInverseInertia();

	m_indexB = GetBodyB()->GetIslandIndex();
	m_localCenterB = GetBodyB()->GetLocalCenter();
	m_invMassB = GetBodyB()->GetInverseMass();
	m_invIB = GetBodyB()->GetInverseInertia();

	const auto cA = positions[m_indexA].linear;
	const auto aA = positions[m_indexA].angular;
	auto vA = velocities[m_indexA].linear;
	auto wA = velocities[m_indexA].angular;

	const auto cB = positions[m_indexB].linear;
	const auto aB = positions[m_indexB].angular;
	auto vB = velocities[m_indexB].linear;
	auto wB = velocities[m_indexB].angular;

	const auto qA = UnitVec2(aA);
	const auto qB = UnitVec2(aB);

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
	if (m_angularMass > RealNum{0})
	{
		m_angularMass = RealNum{1} / m_angularMass;
	}

	m_linearError = cB + m_rB - cA - m_rA - Rotate(m_linearOffset, qA);
	m_angularError = aB - aA - m_angularOffset;

	if (step.doWarmStart)
	{
		// Scale impulses to support a variable time step.
		m_linearImpulse *= step.dtRatio;
		m_angularImpulse *= step.dtRatio;

		const auto P = Vec2{m_linearImpulse.x, m_linearImpulse.y};
		vA -= mA * P;
		wA -= 1_rad * iA * (Cross(m_rA, P) + m_angularImpulse);
		vB += mB * P;
		wB += 1_rad * iB * (Cross(m_rB, P) + m_angularImpulse);
	}
	else
	{
		m_linearImpulse = Vec2_zero;
		m_angularImpulse = RealNum{0};
	}

	velocities[m_indexA].linear = vA;
	velocities[m_indexA].angular = wA;
	velocities[m_indexB].linear = vB;
	velocities[m_indexB].angular = wB;
}

void MotorJoint::SolveVelocityConstraints(Span<Velocity> velocities, const StepConf& step)
{
	auto vA = velocities[m_indexA].linear;
	auto wA = velocities[m_indexA].angular;
	auto vB = velocities[m_indexB].linear;
	auto wB = velocities[m_indexB].angular;

	const auto mA = m_invMassA, mB = m_invMassB;
	const auto iA = m_invIA, iB = m_invIB;

	const auto h = step.get_dt();
	const auto inv_h = step.get_inv_dt();

	// Solve angular friction
	{
		const auto Cdot = wB - wA + inv_h * m_correctionFactor * m_angularError;
		auto impulse = -m_angularMass * Cdot.ToRadians();

		const auto oldImpulse = m_angularImpulse;
		const auto maxImpulse = h * m_maxTorque;
		m_angularImpulse = Clamp(m_angularImpulse + impulse, -maxImpulse, maxImpulse);
		impulse = m_angularImpulse - oldImpulse;

		wA -= 1_rad * iA * impulse;
		wB += 1_rad * iB * impulse;
	}

	// Solve linear friction
	{
		const auto Cdot = vB + (GetRevPerpendicular(m_rB) * wB.ToRadians()) - vA - (GetRevPerpendicular(m_rA) * wA.ToRadians()) + inv_h * m_correctionFactor * m_linearError;

		auto impulse = -Transform(Cdot, m_linearMass);
		const auto oldImpulse = m_linearImpulse;
		m_linearImpulse += impulse;

		const auto maxImpulse = h * m_maxForce;

		if (GetLengthSquared(m_linearImpulse) > Square(maxImpulse))
		{
			m_linearImpulse = GetUnitVector(m_linearImpulse, UnitVec2::GetZero()) * maxImpulse;
		}

		impulse = m_linearImpulse - oldImpulse;

		vA -= mA * impulse;
		wA -= 1_rad * iA * Cross(m_rA, impulse);

		vB += mB * impulse;
		wB += 1_rad * iB * Cross(m_rB, impulse);
	}

	velocities[m_indexA].linear = vA;
	velocities[m_indexA].angular = wA;
	velocities[m_indexB].linear = vB;
	velocities[m_indexB].angular = wB;
}

bool MotorJoint::SolvePositionConstraints(Span<Position> positions, const ConstraintSolverConf& conf) const
{
	NOT_USED(positions);
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
	assert(IsValid(force) && (force >= RealNum{0}));
	m_maxForce = force;
}

RealNum MotorJoint::GetMaxForce() const
{
	return m_maxForce;
}

void MotorJoint::SetMaxTorque(RealNum torque)
{
	assert(IsValid(torque) && (torque >= RealNum{0}));
	m_maxTorque = torque;
}

RealNum MotorJoint::GetMaxTorque() const
{
	return m_maxTorque;
}

void MotorJoint::SetCorrectionFactor(RealNum factor)
{
	assert(IsValid(factor) && (RealNum{0} <= factor) && (factor <= RealNum{1}));
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
