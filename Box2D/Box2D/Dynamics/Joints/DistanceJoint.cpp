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

#include <Box2D/Dynamics/Joints/DistanceJoint.hpp>
#include <Box2D/Dynamics/Body.hpp>
#include <Box2D/Dynamics/StepConf.hpp>
#include <Box2D/Dynamics/Contacts/ContactSolver.hpp>
#include <Box2D/Dynamics/Contacts/BodyConstraint.hpp>

using namespace box2d;

// 1-D constrained system
// m (v2 - v1) = lambda
// v2 + (beta/h) * x1 + gamma * lambda = 0, gamma has units of inverse mass.
// x2 = x1 + h * v2

// 1-D mass-damper-spring system
// m (v2 - v1) + h * d * v2 + h * k * 

// C = norm(p2 - p1) - L
// u = (p2 - p1) / norm(p2 - p1)
// Cdot = dot(u, v2 + cross(w2, r2) - v1 - cross(w1, r1))
// J = [-u -cross(r1, u) u cross(r2, u)]
// K = J * invM * JT
//   = invMass1 + invI1 * cross(r1, u)^2 + invMass2 + invI2 * cross(r2, u)^2

DistanceJointDef::DistanceJointDef(Body* bA, Body* bB,
								   const Vec2 anchor1, const Vec2 anchor2,
								   RealNum freq, RealNum damp) noexcept:
	JointDef{JointType::Distance, bA, bB},
	localAnchorA{GetLocalPoint(*bA, anchor1)}, localAnchorB{GetLocalPoint(*bB, anchor2)},
	length{GetLength(anchor2 - anchor1)},
	frequencyHz{freq}, dampingRatio{damp}
{
}

bool DistanceJoint::IsOkay(const DistanceJointDef& def) noexcept
{
	if (!Joint::IsOkay(def))
	{
		return false;
	}
	if (!(def.frequencyHz >= 0))
	{
		return false;
	}
	return true;
}

DistanceJoint::DistanceJoint(const DistanceJointDef& def):
	Joint(def),
	m_localAnchorA(def.localAnchorA),
	m_localAnchorB(def.localAnchorB),
	m_length(def.length),
	m_frequencyHz(def.frequencyHz),
	m_dampingRatio(def.dampingRatio)
{
	assert(def.frequencyHz >= 0);
}

void DistanceJoint::InitVelocityConstraints(BodyConstraints& bodies,
											const StepConf& step,
											const ConstraintSolverConf& conf)
{
	auto& bodiesA = bodies.at(GetBodyA());
	auto& bodiesB = bodies.at(GetBodyB());

	m_localCenterA = bodiesA.GetLocalCenter();
	m_invMassA = RealNum{bodiesA.GetInvMass() * Kilogram};
	
	m_invIA = bodiesA.GetInvRotInertia() * (SquareMeter * Kilogram / SquareRadian);

	m_localCenterB = bodiesB.GetLocalCenter();
	m_invMassB = RealNum{bodiesB.GetInvMass() * Kilogram};
	m_invIB = bodiesB.GetInvRotInertia() * (SquareMeter * Kilogram / SquareRadian);

	const auto posA = bodiesA.GetPosition();
	auto velA = bodiesA.GetVelocity();

	const auto posB = bodiesB.GetPosition();
	auto velB = bodiesB.GetVelocity();

	const auto qA = UnitVec2{posA.angular};
	const auto qB = UnitVec2{posB.angular};

	m_rA = Rotate(m_localAnchorA - m_localCenterA, qA);
	m_rB = Rotate(m_localAnchorB - m_localCenterB, qB);
	m_u = (posB.linear + m_rB) - (posA.linear + m_rA);

	// Handle singularity.
	const auto length = box2d::GetLength(m_u);
	if (length > conf.linearSlop)
	{
		m_u *= RealNum(1) / length;
	}
	else
	{
		m_u = Vec2_zero;
	}

	const auto crAu = Cross(m_rA, m_u);
	const auto crBu = Cross(m_rB, m_u);
	auto invMass = m_invMassA + m_invIA * Square(crAu) + m_invMassB + m_invIB * Square(crBu);

	// Compute the effective mass matrix.
	m_mass = (invMass != 0) ? RealNum{1} / invMass : RealNum{0};

	if (m_frequencyHz > 0)
	{
		const auto C = length - m_length;

		// Frequency
		const auto omega = 2 * Pi * m_frequencyHz;

		// Damping coefficient
		const auto d = 2 * m_mass * m_dampingRatio * omega;

		// Spring stiffness
		const auto k = m_mass * Square(omega);

		// magic formulas
		const auto h = RealNum{step.GetTime() / Second};
		const auto gamma = h * (d + h * k);
		m_invGamma = (gamma != 0) ? 1 / gamma: 0;
		m_bias = C * h * k * m_invGamma;

		invMass += m_invGamma;
		m_mass = (invMass != 0) ? 1 / invMass: 0;
	}
	else
	{
		m_invGamma = 0;
		m_bias = 0;
	}

	if (step.doWarmStart)
	{
		// Scale the impulse to support a variable time step.
		m_impulse *= step.dtRatio;

		const auto P = m_impulse * m_u;
		velA -= Velocity{m_invMassA * P, RadianPerSecond * m_invIA * Cross(m_rA, P)};
		velB += Velocity{m_invMassB * P, RadianPerSecond * m_invIB * Cross(m_rB, P)};
	}
	else
	{
		m_impulse = 0;
	}

	bodiesA.SetVelocity(velA);
	bodiesB.SetVelocity(velB);
}

RealNum DistanceJoint::SolveVelocityConstraints(BodyConstraints& bodies, const StepConf&)
{
	auto& bodiesA = bodies.at(GetBodyA());
	auto& bodiesB = bodies.at(GetBodyB());

	auto velA = bodiesA.GetVelocity();
	auto velB = bodiesB.GetVelocity();

	// Cdot = dot(u, v + cross(w, r))
	const auto vpA = velA.linear + GetRevPerpendicular(m_rA) * RealNum{velA.angular / RadianPerSecond};
	const auto vpB = velB.linear + GetRevPerpendicular(m_rB) * RealNum{velB.angular / RadianPerSecond};
	const auto Cdot = Dot(m_u, vpB - vpA);

	const auto impulse = -m_mass * (Cdot + m_bias + m_invGamma * m_impulse);
	m_impulse += impulse;

	const auto P = impulse * m_u;
	velA -= Velocity{m_invMassA * P, RadianPerSecond * m_invIA * Cross(m_rA, P)};
	velB += Velocity{m_invMassB * P, RadianPerSecond * m_invIB * Cross(m_rB, P)};

	bodiesA.SetVelocity(velA);
	bodiesB.SetVelocity(velB);
	
	return impulse;
}

bool DistanceJoint::SolvePositionConstraints(BodyConstraints& bodies, const ConstraintSolverConf& conf) const
{
	if (m_frequencyHz > 0)
	{
		// There is no position correction for soft distance constraints.
		return true;
	}

	auto& bodiesA = bodies.at(GetBodyA());
	auto& bodiesB = bodies.at(GetBodyB());

	auto posA = bodiesA.GetPosition();
	auto posB = bodiesB.GetPosition();

	const auto qA = UnitVec2(posA.angular);
	const auto qB = UnitVec2(posB.angular);

	const auto rA = Rotate(m_localAnchorA - m_localCenterA, qA);
	const auto rB = Rotate(m_localAnchorB - m_localCenterB, qB);
	auto u = posB.linear + rB - posA.linear - rA;

	const auto length = Normalize(u);
	const auto deltaLength = length - m_length;
	const auto C = Clamp(deltaLength, -conf.maxLinearCorrection, conf.maxLinearCorrection);

	const auto impulse = -m_mass * C;
	const auto P = impulse * u;

	posA -= Position{m_invMassA * P, Radian * m_invIA * Cross(rA, P)};
	posB += Position{m_invMassB * P, Radian * m_invIB * Cross(rB, P)};

	bodiesA.SetPosition(posA);
	bodiesB.SetPosition(posB);

	return Abs(C) < conf.linearSlop;
}

Vec2 DistanceJoint::GetAnchorA() const
{
	return GetWorldPoint(*GetBodyA(), GetLocalAnchorA());
}

Vec2 DistanceJoint::GetAnchorB() const
{
	return GetWorldPoint(*GetBodyB(), GetLocalAnchorB());
}

Vec2 DistanceJoint::GetReactionForce(Frequency inv_dt) const
{
	return (RealNum{inv_dt / Hertz} * m_impulse) * m_u;
}

RealNum DistanceJoint::GetReactionTorque(Frequency inv_dt) const
{
	NOT_USED(inv_dt);
	return 0;
}
