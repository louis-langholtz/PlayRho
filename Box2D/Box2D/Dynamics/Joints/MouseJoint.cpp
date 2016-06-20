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
#include <Box2D/Dynamics/TimeStep.h>

using namespace box2d;

// p = attached point, m = mouse point
// C = p - m
// Cdot = v
//      = v + cross(w, r)
// J = [I r_skew]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

MouseJoint::MouseJoint(const MouseJointDef& def)
: Joint(def)
{
	assert(def.target.IsValid());
	assert(IsValid(def.maxForce) && (def.maxForce >= float_t{0}));
	assert(IsValid(def.frequencyHz) && (def.frequencyHz >= float_t{0}));
	assert(IsValid(def.dampingRatio) && (def.dampingRatio >= float_t{0}));

	m_targetA = def.target;
	m_localAnchorB = MulT(m_bodyB->GetTransform(), def.target);
	m_maxForce = def.maxForce;
	m_frequencyHz = def.frequencyHz;
	m_dampingRatio = def.dampingRatio;
}

void MouseJoint::SetTarget(const Vec2& target)
{
	if (!m_bodyB->IsAwake())
	{
		m_bodyB->SetAwake();
	}
	m_targetA = target;
}

const Vec2& MouseJoint::GetTarget() const
{
	return m_targetA;
}

void MouseJoint::SetMaxForce(float_t force)
{
	m_maxForce = force;
}

float_t MouseJoint::GetMaxForce() const
{
	return m_maxForce;
}

void MouseJoint::SetFrequency(float_t hz)
{
	m_frequencyHz = hz;
}

float_t MouseJoint::GetFrequency() const
{
	return m_frequencyHz;
}

void MouseJoint::SetDampingRatio(float_t ratio)
{
	m_dampingRatio = ratio;
}

float_t MouseJoint::GetDampingRatio() const
{
	return m_dampingRatio;
}

void MouseJoint::InitVelocityConstraints(const SolverData& data)
{
	m_indexB = m_bodyB->m_islandIndex;
	m_localCenterB = m_bodyB->m_sweep.localCenter;
	m_invMassB = m_bodyB->m_invMass;
	m_invIB = m_bodyB->m_invI;

	const auto cB = data.positions[m_indexB].c;
	const auto aB = data.positions[m_indexB].a;
	auto vB = data.velocities[m_indexB].v;
	auto wB = data.velocities[m_indexB].w;

	const Rot qB(aB);

	const auto mass = m_bodyB->GetMass();

	// Frequency
	const auto omega = float_t(2) * Pi * m_frequencyHz;

	// Damping coefficient
	const auto d = float_t(2) * mass * m_dampingRatio * omega;

	// Spring stiffness
	const auto k = mass * (omega * omega);

	// magic formulas
	// gamma has units of inverse mass.
	// beta has units of inverse time.
	const auto h = data.step.get_dt();
	assert(d + h * k > Epsilon);
	m_gamma = h * (d + h * k);
	if (m_gamma != float_t{0})
	{
		m_gamma = float_t{1} / m_gamma;
	}
	m_beta = h * k * m_gamma;

	// Compute the effective mass matrix.
	m_rB = Mul(qB, m_localAnchorB - m_localCenterB);

	// K    = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
	//      = [1/m1+1/m2     0    ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
	//        [    0     1/m1+1/m2]           [-r1.x*r1.y r1.x*r1.x]           [-r1.x*r1.y r1.x*r1.x]
	Mat22 K;
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
	auto vB = data.velocities[m_indexB].v;
	auto wB = data.velocities[m_indexB].w;

	// Cdot = v + cross(w, r)
	const auto Cdot = vB + Cross(wB, m_rB);
	auto impulse = Mul(m_mass, -(Cdot + m_C + m_gamma * m_impulse));

	const auto oldImpulse = m_impulse;
	m_impulse += impulse;
	const auto maxImpulse = data.step.get_dt() * m_maxForce;
	if (m_impulse.LengthSquared() > maxImpulse * maxImpulse)
	{
		m_impulse *= maxImpulse / m_impulse.Length();
	}
	impulse = m_impulse - oldImpulse;

	vB += m_invMassB * impulse;
	wB += m_invIB * Cross(m_rB, impulse);

	data.velocities[m_indexB].v = vB;
	data.velocities[m_indexB].w = wB;
}

bool MouseJoint::SolvePositionConstraints(const SolverData& data)
{
	BOX2D_NOT_USED(data);
	return true;
}

Vec2 MouseJoint::GetAnchorA() const
{
	return m_targetA;
}

Vec2 MouseJoint::GetAnchorB() const
{
	return m_bodyB->GetWorldPoint(m_localAnchorB);
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

void MouseJoint::Dump()
{
	const auto indexA = m_bodyA->m_islandIndex;
	const auto indexB = m_bodyB->m_islandIndex;
	
	log("  MouseJoint jd;\n");
	log("  jd.bodyA = bodies[%d];\n", indexA);
	log("  jd.bodyB = bodies[%d];\n", indexB);
	log("  jd.collideConnected = bool(%d);\n", m_collideConnected);
	log("  jd.localAnchorB = Vec2(%.15lef, %.15lef);\n", m_localAnchorB.x, m_localAnchorB.y);
	log("  jd.frequencyHz = %.15lef;\n", m_frequencyHz);
	log("  jd.dampingRatio = %.15lef;\n", m_dampingRatio);
	log("  jd.maxForce = %.15lef;\n", m_maxForce);
	log("  joints[%d] = m_world->CreateJoint(&jd);\n", m_index);
}

