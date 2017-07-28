/*
 * Original work Copyright (c) 2007 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#include <PlayRho/Dynamics/Joints/PulleyJoint.hpp>
#include <PlayRho/Dynamics/Body.hpp>
#include <PlayRho/Dynamics/StepConf.hpp>
#include <PlayRho/Dynamics/Contacts/ContactSolver.hpp>
#include <PlayRho/Dynamics/Contacts/BodyConstraint.hpp>

using namespace playrho;

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

PulleyJointDef::PulleyJointDef(NonNull<Body*> bA, NonNull<Body*> bB,
                               const Length2D groundA, const Length2D groundB,
                               const Length2D anchorA, const Length2D anchorB,
                               Real r):
    JointDef{JointType::Pulley, bA, bB, true},
    groundAnchorA{groundA},
    groundAnchorB{groundB},
    localAnchorA{GetLocalPoint(*bA, anchorA)},
    localAnchorB{GetLocalPoint(*bB, anchorB)},
    lengthA{GetLength(anchorA - groundA)},
    lengthB{GetLength(anchorB - groundB)},
    ratio{r}
{
    assert((r > 0) && !almost_zero(r));
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

void PulleyJoint::InitVelocityConstraints(BodyConstraintsMap& bodies,
                                          const StepConf& step,
                                          const ConstraintSolverConf& conf)
{
    auto& bodyConstraintA = At(bodies, GetBodyA());
    auto& bodyConstraintB = At(bodies, GetBodyB());
    
    const auto posA = bodyConstraintA->GetPosition();
    auto velA = bodyConstraintA->GetVelocity();
    const auto posB = bodyConstraintB->GetPosition();
    auto velB = bodyConstraintB->GetVelocity();

    const auto qA = UnitVec2::Get(posA.angular);
    const auto qB = UnitVec2::Get(posB.angular);

    m_rA = Rotate(m_localAnchorA - bodyConstraintA->GetLocalCenter(), qA);
    m_rB = Rotate(m_localAnchorB - bodyConstraintB->GetLocalCenter(), qB);

    // Get the pulley axes.
    const auto pulleyAxisA = Length2D{posA.linear + m_rA - m_groundAnchorA};
    const auto pulleyAxisB = Length2D{posB.linear + m_rB - m_groundAnchorB};

    const auto lengthA = GetLength(pulleyAxisA);
    if (lengthA > (conf.linearSlop * Real{10}))
    {
        const auto uv = pulleyAxisA / lengthA;
        m_uA = Vec2{Real{uv.x}, Real{uv.y}};
    }
    else
    {
        m_uA = Vec2_zero;
    }

    const auto lengthB = GetLength(pulleyAxisB);
    if (lengthB > (conf.linearSlop * Real{10}))
    {
        const auto uv = pulleyAxisB / lengthB;
        m_uB = Vec2{Real{uv.x}, Real{uv.y}};
    }
    else
    {
        m_uB = Vec2_zero;
    }

    // Compute effective mass.
    const auto ruA = Cross(m_rA, m_uA);
    const auto ruB = Cross(m_rB, m_uB);

    const auto invMassA = bodyConstraintA->GetInvMass() + (bodyConstraintA->GetInvRotInertia() * Square(ruA)) / SquareRadian;
    const auto invMassB = bodyConstraintB->GetInvMass() + (bodyConstraintB->GetInvRotInertia() * Square(ruB)) / SquareRadian;

    const auto totalInvMass = invMassA + m_ratio * m_ratio * invMassB;

    m_mass = (totalInvMass > InvMass{0})? Real{1} / totalInvMass: Mass{0};

    if (step.doWarmStart)
    {
        // Scale impulses to support variable time steps.
        m_impulse *= step.dtRatio;

        // Warm starting.
        const auto PA = -(m_impulse) * m_uA;
        const auto PB = (-m_ratio * m_impulse) * m_uB;

        velA += Velocity{bodyConstraintA->GetInvMass() * PA, bodyConstraintA->GetInvRotInertia() * Cross(m_rA, PA) / Radian};
        velB += Velocity{bodyConstraintB->GetInvMass() * PB, bodyConstraintB->GetInvRotInertia() * Cross(m_rB, PB) / Radian};
    }
    else
    {
        m_impulse = 0;
    }

    bodyConstraintA->SetVelocity(velA);
    bodyConstraintB->SetVelocity(velB);
}

bool PulleyJoint::SolveVelocityConstraints(BodyConstraintsMap& bodies, const StepConf&)
{
    auto& bodyConstraintA = At(bodies, GetBodyA());
    auto& bodyConstraintB = At(bodies, GetBodyB());

    auto velA = bodyConstraintA->GetVelocity();
    auto velB = bodyConstraintB->GetVelocity();

    const auto vpA = LinearVelocity2D{velA.linear + GetRevPerpendicular(m_rA) * (velA.angular / Radian)};
    const auto vpB = LinearVelocity2D{velB.linear + GetRevPerpendicular(m_rB) * (velB.angular / Radian)};

    const auto Cdot = LinearVelocity{-Dot(m_uA, vpA) - m_ratio * Dot(m_uB, vpB)};
    const auto impulse = -m_mass * Cdot;
    m_impulse += impulse;

    const auto PA = -impulse * m_uA;
    const auto PB = -m_ratio * impulse * m_uB;
    velA += Velocity{bodyConstraintA->GetInvMass() * PA, bodyConstraintA->GetInvRotInertia() * Cross(m_rA, PA) / Radian};
    velB += Velocity{bodyConstraintB->GetInvMass() * PB, bodyConstraintB->GetInvRotInertia() * Cross(m_rB, PB) / Radian};

    bodyConstraintA->SetVelocity(velA);
    bodyConstraintB->SetVelocity(velB);
    
    return impulse == Momentum(0);
}

bool PulleyJoint::SolvePositionConstraints(BodyConstraintsMap& bodies, const ConstraintSolverConf& conf) const
{
    auto& bodyConstraintA = At(bodies, GetBodyA());
    auto& bodyConstraintB = At(bodies, GetBodyB());

    auto posA = bodyConstraintA->GetPosition();
    auto posB = bodyConstraintB->GetPosition();

    const auto rA = Rotate(m_localAnchorA - bodyConstraintA->GetLocalCenter(), UnitVec2::Get(posA.angular));
    const auto rB = Rotate(m_localAnchorB - bodyConstraintB->GetLocalCenter(), UnitVec2::Get(posB.angular));

    // Get the pulley axes.
    const auto pA = Length2D{posA.linear + rA - m_groundAnchorA};
    auto lengthA = Length{0};
    auto uA = GetUnitVector(pA, lengthA);
    if (lengthA <= (conf.linearSlop * Real{10}))
    {
        uA = UnitVec2::GetZero();
    }

    const auto pB = Length2D{posB.linear + rB - m_groundAnchorB};
    auto lengthB = Length{0};
    auto uB = GetUnitVector(pB, lengthB);
    if (lengthB <= (conf.linearSlop * Real{10}))
    {
        uB = UnitVec2::GetZero();
    }

    // Compute effective mass.
    const auto ruA = Length{Cross(rA, uA)};
    const auto ruB = Length{Cross(rB, uB)};

    const auto totalInvMassA = bodyConstraintA->GetInvMass() + bodyConstraintA->GetInvRotInertia() * ruA * ruA / SquareRadian;
    const auto totalInvMassB = bodyConstraintB->GetInvMass() + bodyConstraintB->GetInvRotInertia() * ruB * ruB / SquareRadian;

    const auto totalInvMass = totalInvMassA + m_ratio * m_ratio * totalInvMassB;
    const auto mass = (totalInvMass > InvMass{0})? Real{1} / totalInvMass: Mass{0};

    const auto C = Length{m_constant - lengthA - (m_ratio * lengthB)};
    const auto linearError = Abs(C);

    const auto impulse = -mass * C;

    const auto PA = -impulse * uA;
    const auto PB = -m_ratio * impulse * uB;

    posA += Position{bodyConstraintA->GetInvMass() * PA, bodyConstraintA->GetInvRotInertia() * Cross(rA, PA) / Radian};
    posB += Position{bodyConstraintB->GetInvMass() * PB, bodyConstraintB->GetInvRotInertia() * Cross(rB, PB) / Radian};

    bodyConstraintA->SetPosition(posA);
    bodyConstraintB->SetPosition(posB);

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

Length playrho::GetCurrentLengthA(const PulleyJoint& joint)
{
    return GetLength(GetWorldPoint(*joint.GetBodyA(), joint.GetLocalAnchorA()) - joint.GetGroundAnchorA());
}

Length playrho::GetCurrentLengthB(const PulleyJoint& joint)
{
    return GetLength(GetWorldPoint(*joint.GetBodyB(), joint.GetLocalAnchorB()) - joint.GetGroundAnchorB());
}

void PulleyJoint::ShiftOrigin(const Length2D newOrigin)
{
    m_groundAnchorA -= newOrigin;
    m_groundAnchorB -= newOrigin;
}

PulleyJointDef playrho::GetPulleyJointDef(const PulleyJoint& joint) noexcept
{
    auto def = PulleyJointDef{};
    
    Set(def, joint);
    
    def.groundAnchorA = joint.GetGroundAnchorA();
    def.groundAnchorB = joint.GetGroundAnchorB();
    def.localAnchorA = joint.GetLocalAnchorA();
    def.localAnchorB = joint.GetGroundAnchorB();
    def.lengthA = joint.GetLengthA();
    def.lengthB = joint.GetLengthB();
    def.ratio = joint.GetRatio();
    
    return def;
}
