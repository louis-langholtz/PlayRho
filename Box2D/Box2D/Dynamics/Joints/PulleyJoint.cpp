/*
 * Original work Copyright (c) 2007 Erin Catto http://www.box2d.org
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

#include <Box2D/Dynamics/Joints/PulleyJoint.hpp>
#include <Box2D/Dynamics/Body.hpp>
#include <Box2D/Dynamics/StepConf.hpp>
#include <Box2D/Dynamics/Contacts/ContactSolver.hpp>
#include <Box2D/Dynamics/Contacts/BodyConstraint.hpp>

using namespace box2d;

// Pulley:
// length1 = norm(p1 - s1)
// length2 = norm(p2 - s2)
// C0 = (length1 + ratio * length2)_initial
// C = C0 - (length1 + ratio * length2)
// u1 = (p1 - s1) / norm(p1 - s1)
// u2 = (p2 - s2) / norm(p2 - s2)
// Cdot = -dot(u1, v1 + cross(w1, r1)) - ratio * dot(u2, v2 + cross(w2, r2))
// J = -[u1 cross(r1, u1) ratio * u2  ratio * cross(r2, u2)]
// K = J * invM * JT
//   = invMass1 + invI1 * cross(r1, u1)^2 + ratio^2 * (invMass2 + invI2 * cross(r2, u2)^2)

void PulleyJointDef::Initialize(Body* bA, Body* bB,
                const Length2D groundA, const Length2D groundB,
                const Length2D anchorA, const Length2D anchorB,
                RealNum r)
{
    assert((r > 0) && !almost_zero(r));
    
    bodyA = bA;
    bodyB = bB;
    groundAnchorA = groundA;
    groundAnchorB = groundB;
    localAnchorA = GetLocalPoint(*bodyA, anchorA);
    localAnchorB = GetLocalPoint(*bodyB, anchorB);
    lengthA = GetLength(anchorA - groundA);
    lengthB = GetLength(anchorB - groundB);
    ratio = r;
}

PulleyJoint::PulleyJoint(const PulleyJointDef& def):
    Joint(def),
    m_groundAnchorA(def.groundAnchorA),
    m_groundAnchorB(def.groundAnchorB),
    m_localAnchorA(def.localAnchorA),
    m_localAnchorB(def.localAnchorB),
    m_lengthA(def.lengthA),
    m_lengthB(def.lengthB),
    m_ratio(def.ratio),
    m_constant(def.lengthA + def.ratio * def.lengthB)
{
    assert(!almost_zero(def.ratio));
}

void PulleyJoint::InitVelocityConstraints(BodyConstraints& bodies,
                                          const StepConf& step,
                                          const ConstraintSolverConf& conf)
{
    auto& bodiesA = bodies.at(GetBodyA());
    auto& bodiesB = bodies.at(GetBodyB());
    
    const auto posA = bodiesA.GetPosition();
    auto velA = bodiesA.GetVelocity();
    const auto posB = bodiesB.GetPosition();
    auto velB = bodiesB.GetVelocity();

    const UnitVec2 qA(posA.angular), qB(posB.angular);

    m_rA = Rotate(m_localAnchorA - bodiesA.GetLocalCenter(), qA);
    m_rB = Rotate(m_localAnchorB - bodiesB.GetLocalCenter(), qB);

    // Get the pulley axes.
    const auto pulleyAxisA = Length2D{posA.linear + m_rA - m_groundAnchorA};
    const auto pulleyAxisB = Length2D{posB.linear + m_rB - m_groundAnchorB};

    const auto lengthA = GetLength(pulleyAxisA);
    if (lengthA > (conf.linearSlop * RealNum{10}))
    {
        const auto uv = pulleyAxisA / lengthA;
        m_uA = Vec2{RealNum{uv.x}, RealNum{uv.y}};
    }
    else
    {
        m_uA = Vec2_zero;
    }

    const auto lengthB = GetLength(pulleyAxisB);
    if (lengthB > (conf.linearSlop * RealNum{10}))
    {
        const auto uv = pulleyAxisB / lengthB;
        m_uB = Vec2{RealNum{uv.x}, RealNum{uv.y}};
    }
    else
    {
        m_uB = Vec2_zero;
    }

    // Compute effective mass.
    const auto ruA = Cross(m_rA, m_uA);
    const auto ruB = Cross(m_rB, m_uB);

    const auto invMassA = bodiesA.GetInvMass() + (bodiesA.GetInvRotInertia() * Square(ruA)) / SquareRadian;
    const auto invMassB = bodiesB.GetInvMass() + (bodiesB.GetInvRotInertia() * Square(ruB)) / SquareRadian;

    const auto totalInvMass = invMassA + m_ratio * m_ratio * invMassB;

    m_mass = (totalInvMass > InvMass{0})? RealNum{1} / totalInvMass: Mass{0};

    if (step.doWarmStart)
    {
        // Scale impulses to support variable time steps.
        m_impulse *= step.dtRatio;

        // Warm starting.
        const auto PA = -(m_impulse) * m_uA;
        const auto PB = (-m_ratio * m_impulse) * m_uB;

        velA += Velocity{bodiesA.GetInvMass() * PA, bodiesA.GetInvRotInertia() * Cross(m_rA, PA) / Radian};
        velB += Velocity{bodiesB.GetInvMass() * PB, bodiesB.GetInvRotInertia() * Cross(m_rB, PB) / Radian};
    }
    else
    {
        m_impulse = 0;
    }

    bodiesA.SetVelocity(velA);
    bodiesB.SetVelocity(velB);
}

RealNum PulleyJoint::SolveVelocityConstraints(BodyConstraints& bodies, const StepConf&)
{
    auto& bodiesA = bodies.at(GetBodyA());
    auto& bodiesB = bodies.at(GetBodyB());

    auto velA = bodiesA.GetVelocity();
    auto velB = bodiesB.GetVelocity();

    const auto vpA = LinearVelocity2D{velA.linear + GetRevPerpendicular(m_rA) * (velA.angular / Radian)};
    const auto vpB = LinearVelocity2D{velB.linear + GetRevPerpendicular(m_rB) * (velB.angular / Radian)};

    const auto Cdot = LinearVelocity{-Dot(m_uA, vpA) - m_ratio * Dot(m_uB, vpB)};
    const auto impulse = -m_mass * Cdot;
    m_impulse += impulse;

    const auto PA = -impulse * m_uA;
    const auto PB = -m_ratio * impulse * m_uB;
    velA += Velocity{bodiesA.GetInvMass() * PA, bodiesA.GetInvRotInertia() * Cross(m_rA, PA) / Radian};
    velB += Velocity{bodiesB.GetInvMass() * PB, bodiesB.GetInvRotInertia() * Cross(m_rB, PB) / Radian};

    bodiesA.SetVelocity(velA);
    bodiesB.SetVelocity(velB);
    
    return StripUnit(impulse);
}

bool PulleyJoint::SolvePositionConstraints(BodyConstraints& bodies, const ConstraintSolverConf& conf) const
{
    auto& bodiesA = bodies.at(GetBodyA());
    auto& bodiesB = bodies.at(GetBodyB());

    auto posA = bodiesA.GetPosition();
    auto posB = bodiesB.GetPosition();

    const auto rA = Rotate(m_localAnchorA - bodiesA.GetLocalCenter(), UnitVec2{posA.angular});
    const auto rB = Rotate(m_localAnchorB - bodiesB.GetLocalCenter(), UnitVec2{posB.angular});

    // Get the pulley axes.
    const auto pA = Length2D{posA.linear + rA - m_groundAnchorA};
    auto lengthA = Length{0};
    auto uA = GetUnitVector(pA, lengthA);
    if (lengthA <= (conf.linearSlop * RealNum{10}))
    {
        uA = UnitVec2::GetZero();
    }

    const auto pB = Length2D{posB.linear + rB - m_groundAnchorB};
    auto lengthB = Length{0};
    auto uB = GetUnitVector(pB, lengthB);
    if (lengthB <= (conf.linearSlop * RealNum{10}))
    {
        uB = UnitVec2::GetZero();
    }

    // Compute effective mass.
    const auto ruA = Length{Cross(rA, uA)};
    const auto ruB = Length{Cross(rB, uB)};

    const auto totalInvMassA = bodiesA.GetInvMass() + bodiesA.GetInvRotInertia() * ruA * ruA / SquareRadian;
    const auto totalInvMassB = bodiesB.GetInvMass() + bodiesB.GetInvRotInertia() * ruB * ruB / SquareRadian;

    const auto totalInvMass = totalInvMassA + m_ratio * m_ratio * totalInvMassB;
    const auto mass = (totalInvMass > InvMass{0})? RealNum{1} / totalInvMass: Mass{0};

    const auto C = Length{m_constant - lengthA - (m_ratio * lengthB)};
    const auto linearError = Abs(C);

    const auto impulse = -mass * C;

    const auto PA = -impulse * uA;
    const auto PB = -m_ratio * impulse * uB;

    posA += Position{bodiesA.GetInvMass() * PA, bodiesA.GetInvRotInertia() * Cross(rA, PA) / Radian};
    posB += Position{bodiesB.GetInvMass() * PB, bodiesB.GetInvRotInertia() * Cross(rB, PB) / Radian};

    bodiesA.SetPosition(posA);
    bodiesB.SetPosition(posB);

    return linearError < conf.linearSlop;
}

Length2D PulleyJoint::GetAnchorA() const
{
    return GetWorldPoint(*GetBodyA(), GetLocalAnchorA());
}

Length2D PulleyJoint::GetAnchorB() const
{
    return GetWorldPoint(*GetBodyB(), GetLocalAnchorB());
}

Force2D PulleyJoint::GetReactionForce(Frequency inv_dt) const
{
    return inv_dt * m_impulse * m_uB;
}

Torque PulleyJoint::GetReactionTorque(Frequency inv_dt) const
{
    NOT_USED(inv_dt);
    return Torque{0};
}

Length box2d::GetCurrentLengthA(const PulleyJoint& joint)
{
    return GetLength(GetWorldPoint(*joint.GetBodyA(), joint.GetLocalAnchorA()) - joint.GetGroundAnchorA());
}

Length box2d::GetCurrentLengthB(const PulleyJoint& joint)
{
    return GetLength(GetWorldPoint(*joint.GetBodyB(), joint.GetLocalAnchorB()) - joint.GetGroundAnchorB());
}

void PulleyJoint::ShiftOrigin(const Length2D newOrigin)
{
    m_groundAnchorA -= newOrigin;
    m_groundAnchorB -= newOrigin;
}
