/*
* Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
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

#include <Box2D/Dynamics/Joints/b2MouseJoint.h>
#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Dynamics/b2TimeStep.h>

using namespace box2d;

// p = attached point, m = mouse point
// C = p - m
// Cdot = v
//      = v + cross(w, r)
// J = [I r_skew]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

b2MouseJoint::b2MouseJoint(const b2MouseJointDef* def)
: b2Joint(def)
{
	b2Assert(def->target.IsValid());
	b2Assert(b2IsValid(def->maxForce) && (def->maxForce >= float_t{0}));
	b2Assert(b2IsValid(def->frequencyHz) && (def->frequencyHz >= float_t{0}));
	b2Assert(b2IsValid(def->dampingRatio) && (def->dampingRatio >= float_t{0}));

	m_targetA = def->target;
	m_localAnchorB = b2MulT(m_bodyB->GetTransform(), def->target);
	m_maxForce = def->maxForce;
	m_frequencyHz = def->frequencyHz;
	m_dampingRatio = def->dampingRatio;
}

void b2MouseJoint::SetTarget(const b2Vec2& target)
{
	if (!m_bodyB->IsAwake())
	{
		m_bodyB->SetAwake();
	}
	m_targetA = target;
}

const b2Vec2& b2MouseJoint::GetTarget() const
{
	return m_targetA;
}

void b2MouseJoint::SetMaxForce(float_t force)
{
	m_maxForce = force;
}

float_t b2MouseJoint::GetMaxForce() const
{
	return m_maxForce;
}

void b2MouseJoint::SetFrequency(float_t hz)
{
	m_frequencyHz = hz;
}

float_t b2MouseJoint::GetFrequency() const
{
	return m_frequencyHz;
}

void b2MouseJoint::SetDampingRatio(float_t ratio)
{
	m_dampingRatio = ratio;
}

float_t b2MouseJoint::GetDampingRatio() const
{
	return m_dampingRatio;
}

void b2MouseJoint::InitVelocityConstraints(const b2SolverData& data)
{
	m_indexB = m_bodyB->m_islandIndex;
	m_localCenterB = m_bodyB->m_sweep.localCenter;
	m_invMassB = m_bodyB->m_invMass;
	m_invIB = m_bodyB->m_invI;

	const auto cB = data.positions[m_indexB].c;
	const auto aB = data.positions[m_indexB].a;
	auto vB = data.velocities[m_indexB].v;
	auto wB = data.velocities[m_indexB].w;

	const b2Rot qB(aB);

	const auto mass = m_bodyB->GetMass();

	// Frequency
	const auto omega = float_t(2) * b2_pi * m_frequencyHz;

	// Damping coefficient
	const auto d = float_t(2) * mass * m_dampingRatio * omega;

	// Spring stiffness
	const auto k = mass * (omega * omega);

	// magic formulas
	// gamma has units of inverse mass.
	// beta has units of inverse time.
	const auto h = data.step.get_dt();
	b2Assert(d + h * k > b2_epsilon);
	m_gamma = h * (d + h * k);
	if (m_gamma != float_t{0})
	{
		m_gamma = float_t(1) / m_gamma;
	}
	m_beta = h * k * m_gamma;

	// Compute the effective mass matrix.
	m_rB = b2Mul(qB, m_localAnchorB - m_localCenterB);

	// K    = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
	//      = [1/m1+1/m2     0    ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
	//        [    0     1/m1+1/m2]           [-r1.x*r1.y r1.x*r1.x]           [-r1.x*r1.y r1.x*r1.x]
	b2Mat22 K;
	K.ex.x = m_invMassB + m_invIB * m_rB.y * m_rB.y + m_gamma;
	K.ex.y = -m_invIB * m_rB.x * m_rB.y;
	K.ey.x = K.ex.y;
	K.ey.y = m_invMassB + m_invIB * m_rB.x * m_rB.x + m_gamma;

	m_mass = K.GetInverse();

	m_C = cB + m_rB - m_targetA;
	m_C *= m_beta;

	// Cheat with some damping
	wB *= float_t(0.98);

	if (data.step.warmStarting)
	{
		m_impulse *= data.step.dtRatio;
		vB += m_invMassB * m_impulse;
		wB += m_invIB * b2Cross(m_rB, m_impulse);
	}
	else
	{
		m_impulse = b2Vec2_zero;
	}

	data.velocities[m_indexB].v = vB;
	data.velocities[m_indexB].w = wB;
}

void b2MouseJoint::SolveVelocityConstraints(const b2SolverData& data)
{
	auto vB = data.velocities[m_indexB].v;
	auto wB = data.velocities[m_indexB].w;

	// Cdot = v + cross(w, r)
	const auto Cdot = vB + b2Cross(wB, m_rB);
	auto impulse = b2Mul(m_mass, -(Cdot + m_C + m_gamma * m_impulse));

	const auto oldImpulse = m_impulse;
	m_impulse += impulse;
	const auto maxImpulse = data.step.get_dt() * m_maxForce;
	if (m_impulse.LengthSquared() > maxImpulse * maxImpulse)
	{
		m_impulse *= maxImpulse / m_impulse.Length();
	}
	impulse = m_impulse - oldImpulse;

	vB += m_invMassB * impulse;
	wB += m_invIB * b2Cross(m_rB, impulse);

	data.velocities[m_indexB].v = vB;
	data.velocities[m_indexB].w = wB;
}

bool b2MouseJoint::SolvePositionConstraints(const b2SolverData& data)
{
	B2_NOT_USED(data);
	return true;
}

b2Vec2 b2MouseJoint::GetAnchorA() const
{
	return m_targetA;
}

b2Vec2 b2MouseJoint::GetAnchorB() const
{
	return m_bodyB->GetWorldPoint(m_localAnchorB);
}

b2Vec2 b2MouseJoint::GetReactionForce(float_t inv_dt) const
{
	return inv_dt * m_impulse;
}

float_t b2MouseJoint::GetReactionTorque(float_t inv_dt) const
{
	return inv_dt * float_t{0};
}

void b2MouseJoint::ShiftOrigin(const b2Vec2& newOrigin)
{
	m_targetA -= newOrigin;
}

void b2MouseJoint::Dump()
{
	const auto indexA = m_bodyA->m_islandIndex;
	const auto indexB = m_bodyB->m_islandIndex;
	
	b2Log("  b2MouseJoint jd;\n");
	b2Log("  jd.bodyA = bodies[%d];\n", indexA);
	b2Log("  jd.bodyB = bodies[%d];\n", indexB);
	b2Log("  jd.collideConnected = bool(%d);\n", m_collideConnected);
	b2Log("  jd.localAnchorB = b2Vec2(%.15lef, %.15lef);\n", m_localAnchorB.x, m_localAnchorB.y);
	b2Log("  jd.frequencyHz = %.15lef;\n", m_frequencyHz);
	b2Log("  jd.dampingRatio = %.15lef;\n", m_dampingRatio);
	b2Log("  jd.maxForce = %.15lef;\n", m_maxForce);
	b2Log("  joints[%d] = m_world->CreateJoint(&jd);\n", m_index);
}

