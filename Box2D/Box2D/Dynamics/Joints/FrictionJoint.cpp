/*
* Original work Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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

#include <Box2D/Dynamics/Joints/FrictionJoint.hpp>
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

void FrictionJointDef::Initialize(Body* bA, Body* bB, const Vec2 anchor)
{
	bodyA = bA;
	bodyB = bB;
	localAnchorA = GetLocalPoint(*bA, anchor);
	localAnchorB = GetLocalPoint(*bB, anchor);
}

FrictionJoint::FrictionJoint(const FrictionJointDef& def):
	Joint(def),
	m_localAnchorA(def.localAnchorA),
	m_localAnchorB(def.localAnchorB),
	m_maxForce(def.maxForce),
	m_maxTorque(def.maxTorque)
{
	// Intentionally empty.
}

void FrictionJoint::InitVelocityConstraints(BodyConstraints& bodies, const StepConf& step, const ConstraintSolverConf&)
{
	auto& bodiesA = bodies.at(GetBodyA());
	auto& bodiesB = bodies.at(GetBodyB());

	m_localCenterA = bodiesA.GetLocalCenter();
	m_invMassA = bodiesA.GetInvMass();
	m_invIA = bodiesA.GetInvRotInertia();
	const auto posA = bodiesA.GetPosition();
	auto velA = bodiesA.GetVelocity();

	m_localCenterB = bodiesB.GetLocalCenter();
	m_invMassB = bodiesB.GetInvMass();
	m_invIB = bodiesB.GetInvRotInertia();
	const auto posB = bodiesB.GetPosition();
	auto velB = bodiesB.GetVelocity();

	// Compute the effective mass matrix.
	m_rA = Rotate(m_localAnchorA - m_localCenterA, UnitVec2{posA.angular});
	m_rB = Rotate(m_localAnchorB - m_localCenterB, UnitVec2{posB.angular});

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
	K.ex.x = mA + mB + iA * Square(m_rA.y) + iB * Square(m_rB.y);
	K.ex.y = -iA * m_rA.x * m_rA.y - iB * m_rB.x * m_rB.y;
	K.ey.x = K.ex.y;
	K.ey.y = mA + mB + iA * Square(m_rA.x) + iB * Square(m_rB.x);

	m_linearMass = Invert(K);

	m_angularMass = iA + iB;
	if (m_angularMass > 0)
	{
		m_angularMass = RealNum{1} / m_angularMass;
	}

	if (step.doWarmStart)
	{
		// Scale impulses to support a variable time step.
		m_linearImpulse *= step.dtRatio;
		m_angularImpulse *= step.dtRatio;

		const auto P = Vec2{m_linearImpulse.x, m_linearImpulse.y};
		velA -= Velocity{mA * P, 1_rad * iA * (Cross(m_rA, P) + m_angularImpulse)};
		velB += Velocity{mB * P, 1_rad * iB * (Cross(m_rB, P) + m_angularImpulse)};
	}
	else
	{
		m_linearImpulse = Vec2_zero;
		m_angularImpulse = 0;
	}

	bodiesA.SetVelocity(velA);
	bodiesB.SetVelocity(velB);
}

RealNum FrictionJoint::SolveVelocityConstraints(BodyConstraints& bodies, const StepConf& step)
{
	auto& bodiesA = bodies.at(GetBodyA());
	auto& bodiesB = bodies.at(GetBodyB());

	auto velA = bodiesA.GetVelocity();
	auto velB = bodiesB.GetVelocity();

	const auto iA = m_invIA;
	const auto iB = m_invIB;

	const auto h = RealNum{step.get_dt() / second};

	// Solve angular friction
	auto angularImpulse = RealNum{0};
	{
		const auto Cdot = velB.angular.ToRadians() - velA.angular.ToRadians();
		const auto impulse = -m_angularMass * Cdot;

		const auto oldImpulse = m_angularImpulse;
		const auto maxImpulse = h * m_maxTorque;
		m_angularImpulse = Clamp(m_angularImpulse + impulse, -maxImpulse, maxImpulse);
		angularImpulse = m_angularImpulse - oldImpulse;

		velA.angular -= 1_rad * iA * angularImpulse;
		velB.angular += 1_rad * iB * angularImpulse;
	}

	// Solve linear friction
	{
		const auto Cdot = velB.linear + (GetRevPerpendicular(m_rB) * velB.angular.ToRadians()) - velA.linear - (GetRevPerpendicular(m_rA) * velA.angular.ToRadians());

		auto impulse = -Transform(Cdot, m_linearMass);
		const auto oldImpulse = m_linearImpulse;
		m_linearImpulse += impulse;

		const auto maxImpulse = h * m_maxForce;

		if (GetLengthSquared(m_linearImpulse) > Square(maxImpulse))
		{
			m_linearImpulse = GetUnitVector(m_linearImpulse, UnitVec2::GetZero()) * maxImpulse;
		}

		impulse = m_linearImpulse - oldImpulse;

		velA -= Velocity{m_invMassA * impulse, 1_rad * iA * Cross(m_rA, impulse)};
		velB += Velocity{m_invMassB * impulse, 1_rad * iB * Cross(m_rB, impulse)};
	}

	bodiesA.SetVelocity(velA);
	bodiesB.SetVelocity(velB);
	
	return GetInvalid<RealNum>(); // TODO
}

bool FrictionJoint::SolvePositionConstraints(BodyConstraints& bodies, const ConstraintSolverConf& conf) const
{
	NOT_USED(bodies);
	NOT_USED(conf);

	return true;
}

Vec2 FrictionJoint::GetAnchorA() const
{
	return GetWorldPoint(*GetBodyA(), GetLocalAnchorA());
}

Vec2 FrictionJoint::GetAnchorB() const
{
	return GetWorldPoint(*GetBodyB(), GetLocalAnchorB());
}

Vec2 FrictionJoint::GetReactionForce(RealNum inv_dt) const
{
	return inv_dt * m_linearImpulse;
}

RealNum FrictionJoint::GetReactionTorque(RealNum inv_dt) const
{
	return inv_dt * m_angularImpulse;
}

void FrictionJoint::SetMaxForce(RealNum force)
{
	assert(IsValid(force) && (force >= 0));
	m_maxForce = force;
}

RealNum FrictionJoint::GetMaxForce() const
{
	return m_maxForce;
}

void FrictionJoint::SetMaxTorque(RealNum torque)
{
	assert(IsValid(torque) && (torque >= 0));
	m_maxTorque = torque;
}

RealNum FrictionJoint::GetMaxTorque() const
{
	return m_maxTorque;
}
