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

#include <Box2D/Dynamics/Joints/WeldJoint.hpp>
#include <Box2D/Dynamics/Body.hpp>
#include <Box2D/Dynamics/StepConf.hpp>
#include <Box2D/Dynamics/Contacts/ContactSolver.hpp>
#include <Box2D/Dynamics/Contacts/BodyConstraint.hpp>

using namespace box2d;

// Point-to-point constraint
// C = p2 - p1
// Cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-I -r1_skew I r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

// Angle constraint
// C = angle2 - angle1 - referenceAngle
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2

void WeldJointDef::Initialize(Body* bA, Body* bB, const Vec2 anchor)
{
	bodyA = bA;
	bodyB = bB;
	localAnchorA = GetLocalPoint(*bodyA, anchor);
	localAnchorB = GetLocalPoint(*bodyB, anchor);
	referenceAngle = bodyB->GetAngle() - bodyA->GetAngle();
}

WeldJoint::WeldJoint(const WeldJointDef& def)
: Joint(def)
{
	m_localAnchorA = def.localAnchorA;
	m_localAnchorB = def.localAnchorB;
	m_referenceAngle = def.referenceAngle;
	m_frequencyHz = def.frequencyHz;
	m_dampingRatio = def.dampingRatio;

	m_impulse = Vec3_zero;
}

void WeldJoint::InitVelocityConstraints(Span<BodyConstraint> bodies, const StepConf& step, const ConstraintSolverConf&)
{
	m_indexA = GetBodyA()->GetIslandIndex();
	m_indexB = GetBodyB()->GetIslandIndex();
	m_localCenterA = GetBodyA()->GetLocalCenter();
	m_localCenterB = GetBodyB()->GetLocalCenter();
	m_invMassA = GetBodyA()->GetInverseMass();
	m_invMassB = GetBodyB()->GetInverseMass();
	m_invIA = GetBodyA()->GetInverseInertia();
	m_invIB = GetBodyB()->GetInverseInertia();

	const auto aA = bodies[m_indexA].GetPosition().angular;
	auto vA = bodies[m_indexA].GetVelocity().linear;
	auto wA = bodies[m_indexA].GetVelocity().angular;

	const auto aB = bodies[m_indexB].GetPosition().angular;
	auto vB = bodies[m_indexB].GetVelocity().linear;
	auto wB = bodies[m_indexB].GetVelocity().angular;

	const UnitVec2 qA(aA), qB(aB);

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

	Mat33 K;
	K.ex.x = mA + mB + m_rA.y * m_rA.y * iA + m_rB.y * m_rB.y * iB;
	K.ey.x = -m_rA.y * m_rA.x * iA - m_rB.y * m_rB.x * iB;
	K.ez.x = -m_rA.y * iA - m_rB.y * iB;
	K.ex.y = K.ey.x;
	K.ey.y = mA + mB + m_rA.x * m_rA.x * iA + m_rB.x * m_rB.x * iB;
	K.ez.y = m_rA.x * iA + m_rB.x * iB;
	K.ex.z = K.ez.x;
	K.ey.z = K.ez.y;
	K.ez.z = iA + iB;

	if (m_frequencyHz > RealNum{0})
	{
		m_mass = GetInverse22(K);

		auto invM = iA + iB;
		const auto m = (invM > RealNum{0}) ? RealNum{1} / invM : RealNum{0};

		const auto C = aB - aA - m_referenceAngle;

		// Frequency
		const auto omega = RealNum(2) * Pi * m_frequencyHz;

		// Damping coefficient
		const auto d = RealNum(2) * m * m_dampingRatio * omega;

		// Spring stiffness
		const auto k = m * omega * omega;

		// magic formulas
		const auto h = step.get_dt();
		m_gamma = h * (d + h * k);
		m_gamma = (m_gamma != RealNum{0}) ? RealNum{1} / m_gamma : RealNum{0};
		m_bias = C.ToRadians() * h * k * m_gamma;

		invM += m_gamma;
		m_mass.ez.z = (invM != RealNum{0}) ? RealNum{1} / invM : RealNum{0};
	}
	else if (K.ez.z == RealNum{0})
	{
		m_mass = GetInverse22(K);
		m_gamma = RealNum{0};
		m_bias = RealNum{0};
	}
	else
	{
		m_mass = GetSymInverse33(K);
		m_gamma = RealNum{0};
		m_bias = RealNum{0};
	}

	if (step.doWarmStart)
	{
		// Scale impulses to support a variable time step.
		m_impulse *= step.dtRatio;

		const auto P = Vec2{m_impulse.x, m_impulse.y};

		vA -= mA * P;
		wA -= 1_rad * iA * (Cross(m_rA, P) + m_impulse.z);

		vB += mB * P;
		wB += 1_rad * iB * (Cross(m_rB, P) + m_impulse.z);
	}
	else
	{
		m_impulse = Vec3_zero;
	}

	bodies[m_indexA].SetVelocity(Velocity{vA, wA});
	bodies[m_indexB].SetVelocity(Velocity{vB, wB});
}

void WeldJoint::SolveVelocityConstraints(Span<BodyConstraint> bodies, const StepConf&)
{
	auto vA = bodies[m_indexA].GetVelocity().linear;
	auto wA = bodies[m_indexA].GetVelocity().angular;
	auto vB = bodies[m_indexB].GetVelocity().linear;
	auto wB = bodies[m_indexB].GetVelocity().angular;
	
	const auto mA = m_invMassA, mB = m_invMassB;
	const auto iA = m_invIA, iB = m_invIB;

	if (m_frequencyHz > RealNum{0})
	{
		const auto Cdot2 = (wB - wA).ToRadians();

		const auto impulse2 = -m_mass.ez.z * (Cdot2 + m_bias + m_gamma * m_impulse.z);
		m_impulse.z += impulse2;

		wA -= 1_rad * iA * impulse2;
		wB += 1_rad * iB * impulse2;

		const auto Cdot1 = vB + (GetRevPerpendicular(m_rB) * wB.ToRadians()) - vA - (GetRevPerpendicular(m_rA) * wA.ToRadians());

		const auto impulse1 = -Transform(Cdot1, m_mass);
		m_impulse.x += impulse1.x;
		m_impulse.y += impulse1.y;

		const auto P = impulse1;

		vA -= mA * P;
		wA -= 1_rad * iA * Cross(m_rA, P);

		vB += mB * P;
		wB += 1_rad * iB * Cross(m_rB, P);
	}
	else
	{
		const auto Cdot1 = vB + (GetRevPerpendicular(m_rB) * wB.ToRadians()) - vA - (GetRevPerpendicular(m_rA) * wA.ToRadians());
		const auto Cdot2 = (wB - wA).ToRadians();
		const auto Cdot = Vec3(Cdot1.x, Cdot1.y, Cdot2);

		const auto impulse = -Transform(Cdot, m_mass);
		m_impulse += impulse;

		const auto P = Vec2{impulse.x, impulse.y};

		vA -= mA * P;
		wA -= 1_rad * iA * (Cross(m_rA, P) + impulse.z);

		vB += mB * P;
		wB += 1_rad * iB * (Cross(m_rB, P) + impulse.z);
	}

	bodies[m_indexA].SetVelocity(Velocity{vA, wA});
	bodies[m_indexB].SetVelocity(Velocity{vB, wB});
}

bool WeldJoint::SolvePositionConstraints(Span<BodyConstraint> bodies, const ConstraintSolverConf& conf) const
{
	auto cA = bodies[m_indexA].GetPosition().linear;
	auto aA = bodies[m_indexA].GetPosition().angular;
	auto cB = bodies[m_indexB].GetPosition().linear;
	auto aB = bodies[m_indexB].GetPosition().angular;

	const auto qA = UnitVec2{aA};
	const auto qB = UnitVec2{aB};

	const auto mA = m_invMassA, mB = m_invMassB;
	const auto iA = m_invIA, iB = m_invIB;

	const auto rA = Rotate(m_localAnchorA - m_localCenterA, qA);
	const auto rB = Rotate(m_localAnchorB - m_localCenterB, qB);

	RealNum positionError, angularError;

	Mat33 K;
	K.ex.x = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB;
	K.ey.x = -rA.y * rA.x * iA - rB.y * rB.x * iB;
	K.ez.x = -rA.y * iA - rB.y * iB;
	K.ex.y = K.ey.x;
	K.ey.y = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;
	K.ez.y = rA.x * iA + rB.x * iB;
	K.ex.z = K.ez.x;
	K.ey.z = K.ez.y;
	K.ez.z = iA + iB;

	if (m_frequencyHz > RealNum{0})
	{
		const auto C1 =  cB + rB - cA - rA;

		positionError = GetLength(C1);
		angularError = RealNum{0};

		const auto P = -Solve22(K, C1);

		cA -= mA * P;
		aA -= 1_rad * iA * Cross(rA, P);

		cB += mB * P;
		aB += 1_rad * iB * Cross(rB, P);
	}
	else
	{
		const auto C1 = cB + rB - cA - rA;
		const auto C2 = (aB - aA - m_referenceAngle).ToRadians();

		positionError = GetLength(C1);
		angularError = Abs(C2);

		const auto C = Vec3(C1.x, C1.y, C2);
	
		Vec3 impulse;
		if (K.ez.z > RealNum{0})
		{
			impulse = -Solve33(K, C);
		}
		else
		{
			const auto impulse2 = -Solve22(K, C1);
			impulse = Vec3(impulse2.x, impulse2.y, RealNum{0});
		}

		const auto P = Vec2{impulse.x, impulse.y};

		cA -= mA * P;
		aA -= 1_rad * iA * (Cross(rA, P) + impulse.z);

		cB += mB * P;
		aB += 1_rad * iB * (Cross(rB, P) + impulse.z);
	}

	bodies[m_indexA].SetPosition(Position{cA, aA});
	bodies[m_indexB].SetPosition(Position{cB, aB});

	return (positionError <= conf.linearSlop) && (angularError <= conf.angularSlop);
}

Vec2 WeldJoint::GetAnchorA() const
{
	return GetWorldPoint(*GetBodyA(), GetLocalAnchorA());
}

Vec2 WeldJoint::GetAnchorB() const
{
	return GetWorldPoint(*GetBodyB(), GetLocalAnchorB());
}

Vec2 WeldJoint::GetReactionForce(RealNum inv_dt) const
{
	const auto P = Vec2{m_impulse.x, m_impulse.y};
	return inv_dt * P;
}

RealNum WeldJoint::GetReactionTorque(RealNum inv_dt) const
{
	return inv_dt * m_impulse.z;
}
