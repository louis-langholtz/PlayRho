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

#include <Box2D/Dynamics/Joints/MouseJoint.h>
#include <Box2D/Dynamics/Body.h>
#include <Box2D/Dynamics/SolverData.hpp>

using namespace box2d;

// p = attached point, m = mouse point
// C = p - m
// Cdot = v
//      = v + cross(w, r)
// J = [I r_skew]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

MouseJoint::MouseJoint(const MouseJointDef& def):
	Joint{def},
	m_localAnchorB{InverseTransform(def.target, GetBodyB()->GetTransformation())},
	m_targetA{def.target}, m_maxForce{def.maxForce}, m_frequencyHz{def.frequencyHz}, m_dampingRatio{def.dampingRatio}
{
	assert(IsValid(def.target));
	assert(IsValid(def.maxForce) && (def.maxForce >= float_t{0}));
	assert(IsValid(def.frequencyHz) && (def.frequencyHz >= float_t{0}));
	assert(IsValid(def.dampingRatio) && (def.dampingRatio >= float_t{0}));
}

void MouseJoint::SetTarget(const Vec2& target)
{
	assert(IsValid(target));
	if (!GetBodyB()->IsAwake())
	{
		GetBodyB()->SetAwake();
	}
	m_targetA = target;
}

void MouseJoint::InitVelocityConstraints(const SolverData& data)
{
	m_indexB = GetBodyB()->GetIslandIndex();
	m_localCenterB = GetBodyB()->GetLocalCenter();
	m_invMassB = GetBodyB()->GetInverseMass();
	m_invIB = GetBodyB()->GetInverseInertia();

	const auto positionB = data.positions[m_indexB];
	assert(IsValid(positionB));
	const auto cB = positionB.c;
	const auto aB = positionB.a;

	const auto velocityB = data.velocities[m_indexB];
	assert(IsValid(velocityB));
	auto vB = velocityB.v;
	auto wB = velocityB.w;

	const Rot qB(aB);

	const auto mass = GetMass(*GetBodyB());

	// Frequency
	const auto omega = float_t(2) * Pi * m_frequencyHz;

	// Damping coefficient
	const auto d = float_t(2) * mass * m_dampingRatio * omega;

	// Spring stiffness
	const auto k = mass * Square(omega);

	// magic formulas
	// gamma has units of inverse mass.
	// beta has units of inverse time.
	const auto h = data.step.get_dt();
	const auto tmp = d + h * k;
	assert(IsValid(tmp));
	assert((tmp > 0) && !almost_zero(tmp));
	m_gamma = h * tmp;
	assert(IsValid(m_gamma));
	if (m_gamma != float_t{0})
	{
		m_gamma = float_t{1} / m_gamma;
	}
	const auto beta = h * k * m_gamma;

	// Compute the effective mass matrix.
	m_rB = Rotate(m_localAnchorB - m_localCenterB, qB);

	// K    = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
	//      = [1/m1+1/m2     0    ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
	//        [    0     1/m1+1/m2]           [-r1.x*r1.y r1.x*r1.x]           [-r1.x*r1.y r1.x*r1.x]
	Mat22 K;
	K.ex.x = m_invMassB + m_invIB * m_rB.y * m_rB.y + m_gamma;
	K.ex.y = -m_invIB * m_rB.x * m_rB.y;
	K.ey.x = K.ex.y;
	K.ey.y = m_invMassB + m_invIB * m_rB.x * m_rB.x + m_gamma;

	m_mass = Invert(K);

	m_C = ((cB + m_rB) - m_targetA) * beta;
	assert(IsValid(m_C));

	// Cheat with some damping
	wB *= float_t(0.98);

	if (data.step.warmStarting)
	{
		m_impulse *= data.step.dtRatio;
		vB += m_invMassB * m_impulse;
		wB += m_invIB * Cross(m_rB, m_impulse);
	}
	else
	{
		m_impulse = Vec2_zero;
	}

	data.velocities[m_indexB].v = vB;
	data.velocities[m_indexB].w = wB;
}

void MouseJoint::SolveVelocityConstraints(const SolverData& data)
{
	const auto velocityB = data.velocities[m_indexB];
	assert(IsValid(velocityB));
	auto vB = velocityB.v;
	auto wB = velocityB.w;

	const auto Cdot = vB + (GetRevPerpendicular(m_rB) * wB);

	const auto oldImpulse = m_impulse;
	const auto addImpulse = Transform(-(Cdot + m_C + m_gamma * m_impulse), m_mass);
	assert(IsValid(addImpulse));
	m_impulse += addImpulse;
	const auto maxImpulse = data.step.get_dt() * m_maxForce;
	if (LengthSquared(m_impulse) > Square(maxImpulse))
	{
		m_impulse *= maxImpulse / Length(m_impulse);
	}

	const auto deltaImpulse = m_impulse - oldImpulse;

	vB += m_invMassB * deltaImpulse;
	wB += m_invIB * Cross(m_rB, deltaImpulse);

	data.velocities[m_indexB].v = vB;
	data.velocities[m_indexB].w = wB;
}

bool MouseJoint::SolvePositionConstraints(const SolverData& data)
{
	BOX2D_NOT_USED(data);
	return true;
}

Vec2 MouseJoint::GetAnchorB() const
{
	return GetWorldPoint(*GetBodyB(), m_localAnchorB);
}

Vec2 MouseJoint::GetReactionForce(float_t inv_dt) const
{
	return inv_dt * m_impulse;
}

float_t MouseJoint::GetReactionTorque(float_t inv_dt) const
{
	return inv_dt * float_t{0};
}

void MouseJoint::ShiftOrigin(const Vec2& newOrigin)
{
	m_targetA -= newOrigin;
}

void box2d::Dump(const MouseJoint& joint, size_t index)
{
	log("  MouseJoint jd;\n");
	log("  jd.bodyA = bodies[%d];\n", GetWorldIndex(joint.GetBodyA()));
	log("  jd.bodyB = bodies[%d];\n", GetWorldIndex(joint.GetBodyB()));
	log("  jd.collideConnected = bool(%d);\n", joint.GetCollideConnected());
	log("  jd.localAnchorB = Vec2(%.15lef, %.15lef);\n", joint.GetLocalAnchorB().x, joint.GetLocalAnchorB().y);
	log("  jd.frequencyHz = %.15lef;\n", joint.GetFrequency());
	log("  jd.dampingRatio = %.15lef;\n", joint.GetDampingRatio());
	log("  jd.maxForce = %.15lef;\n", joint.GetMaxForce());
	log("  joints[%d] = m_world->CreateJoint(jd);\n", index);
}

