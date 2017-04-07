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

#include <Box2D/Dynamics/Joints/MouseJoint.hpp>
#include <Box2D/Dynamics/Body.hpp>
#include <Box2D/Dynamics/StepConf.hpp>
#include <Box2D/Dynamics/Contacts/BodyConstraint.hpp>

using namespace box2d;

// p = attached point, m = mouse point
// C = p - m
// Cdot = v
//      = v + cross(w, r)
// J = [I r_skew]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)


bool MouseJoint::IsOkay(const MouseJointDef& def) noexcept
{
	if (!Joint::IsOkay(def))
	{
		return false;
	}
	if (!IsValid(def.target))
	{
		return false;
	}
	if (!(def.maxForce >= 0) || !(def.frequencyHz >= 0) || !(def.dampingRatio >= 0))
	{
		return false;
	}
	return true;
}

MouseJoint::MouseJoint(const MouseJointDef& def):
	Joint{def},
	m_localAnchorB{InverseTransform(def.target, GetBodyB()->GetTransformation())},
	m_targetA{def.target}, m_maxForce{def.maxForce}, m_frequencyHz{def.frequencyHz}, m_dampingRatio{def.dampingRatio}
{
	assert(IsValid(def.target));
	assert(IsValid(def.maxForce) && (def.maxForce >= 0));
	assert(IsValid(def.frequencyHz) && (def.frequencyHz >= 0));
	assert(IsValid(def.dampingRatio) && (def.dampingRatio >= 0));
}

void MouseJoint::SetTarget(const Vec2 target) noexcept
{
	assert(IsValid(target));
	if (!GetBodyB()->IsAwake())
	{
		GetBodyB()->SetAwake();
	}
	m_targetA = target;
}

Mat22 MouseJoint::GetEffectiveMassMatrix() const noexcept
{
	// K    = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
	//      = [1/m1+1/m2     0    ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
	//        [    0     1/m1+1/m2]           [-r1.x*r1.y r1.x*r1.x]           [-r1.x*r1.y r1.x*r1.x]
	Mat22 K;
	K.ex.x = m_invMassB + (m_invIB * m_rB.y * m_rB.y) + m_gamma;
	K.ex.y = -m_invIB * m_rB.x * m_rB.y;
	K.ey.x = K.ex.y;
	K.ey.y = m_invMassB + (m_invIB * m_rB.x * m_rB.x) + m_gamma;
	return K;
}

void MouseJoint::InitVelocityConstraints(BodyConstraints& bodies, const StepConf& step, const ConstraintSolverConf&)
{
	auto& bodiesB = bodies.at(GetBodyB());

	m_localCenterB = bodiesB.GetLocalCenter();
	m_invMassB = RealNum{bodiesB.GetInvMass() * Kilogram};
	m_invIB = bodiesB.GetInvRotInertia() * (SquareMeter * Kilogram / SquareRadian);
	const auto posB = bodiesB.GetPosition();
	auto velB = bodiesB.GetVelocity();

	const UnitVec2 qB(posB.angular);

	const auto mass = GetMass(*GetBodyB());

	// Frequency
	const auto omega = 2 * Pi * m_frequencyHz;

	// Damping coefficient
	const auto d = 2 * RealNum{mass / Kilogram} * m_dampingRatio * omega;

	// Spring stiffness
	const auto k = RealNum{mass / Kilogram} * Square(omega);

	// magic formulas
	// gamma has units of inverse mass.
	// beta has units of inverse time.
	const auto h = RealNum{step.GetTime() / Second};
	const auto tmp = d + h * k;
	assert(IsValid(tmp));
	assert((tmp > 0) && !almost_zero(tmp));
	m_gamma = h * tmp;
	assert(IsValid(m_gamma));
	if (m_gamma != 0)
	{
		m_gamma = 1 / m_gamma;
	}
	const auto beta = h * k * m_gamma;

	// Compute the effective mass matrix.
	m_rB = Rotate(m_localAnchorB - m_localCenterB, qB);

	const auto K = GetEffectiveMassMatrix();

	m_mass = Invert(K);

	m_C = ((posB.linear + m_rB) - m_targetA) * beta;
	assert(IsValid(m_C));

	// Cheat with some damping
	velB.angular *= 0.98f;

	if (step.doWarmStart)
	{
		m_impulse *= step.dtRatio;
		velB += Velocity{m_invMassB * m_impulse * MeterPerSecond, RadianPerSecond * m_invIB * Cross(m_rB, m_impulse)};
	}
	else
	{
		m_impulse = Vec2_zero;
	}

	bodiesB.SetVelocity(velB);
}

RealNum MouseJoint::SolveVelocityConstraints(BodyConstraints& bodies, const StepConf& step)
{
	auto& bodiesB = bodies.at(GetBodyB());

	auto velB = bodiesB.GetVelocity();
	assert(IsValid(velB));

	const auto Cdot = velB.linear + (GetRevPerpendicular(m_rB) * RealNum{velB.angular / RadianPerSecond}) * MeterPerSecond;

	const auto oldImpulse = m_impulse;
	const auto addImpulse = Transform(-(Vec2{Cdot.x / MeterPerSecond, Cdot.y / MeterPerSecond} + m_C + m_gamma * m_impulse), m_mass);
	assert(IsValid(addImpulse));
	m_impulse += addImpulse;
	const auto maxImpulse = RealNum{step.GetTime() / Second} * m_maxForce;
	if (GetLengthSquared(m_impulse) > Square(maxImpulse))
	{
		m_impulse *= maxImpulse / GetLength(m_impulse);
	}

	const auto deltaImpulse = m_impulse - oldImpulse;

	velB += Velocity{m_invMassB * deltaImpulse * MeterPerSecond, RadianPerSecond * m_invIB * Cross(m_rB, deltaImpulse)};

	bodiesB.SetVelocity(velB);
	
	return GetInvalid<RealNum>(); // TODO
}

bool MouseJoint::SolvePositionConstraints(BodyConstraints& bodies, const ConstraintSolverConf& conf) const
{
	NOT_USED(bodies);
	NOT_USED(conf);
	return true;
}

Vec2 MouseJoint::GetAnchorB() const
{
	return GetWorldPoint(*GetBodyB(), GetLocalAnchorB());
}

Vec2 MouseJoint::GetReactionForce(Frequency inv_dt) const
{
	return RealNum{inv_dt / Hertz} * m_impulse;
}

RealNum MouseJoint::GetReactionTorque(Frequency inv_dt) const
{
	return RealNum{inv_dt / Hertz} * 0;
}

void MouseJoint::ShiftOrigin(const Vec2 newOrigin)
{
	m_targetA -= newOrigin;
}

