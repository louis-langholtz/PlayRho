/*
 * Original work Copyright (c) 2007 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
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
    assert(!AlmostZero(def.ratio));
}

void PulleyJoint::InitVelocityConstraints(BodyConstraintsMap& bodies,
                                          const StepConf& step,
                                          const ConstraintSolverConf&)
{
    auto& bodyConstraintA = At(bodies, GetBodyA());
    auto& bodyConstraintB = At(bodies, GetBodyB());
    
    const auto posA = bodyConstraintA->GetPosition();
    const auto invMassA = bodyConstraintA->GetInvMass();
    const auto invRotInertiaA = bodyConstraintA->GetInvRotInertia();
    auto velA = bodyConstraintA->GetVelocity();

    const auto posB = bodyConstraintB->GetPosition();
    const auto invMassB = bodyConstraintB->GetInvMass();
    const auto invRotInertiaB = bodyConstraintB->GetInvRotInertia();
    auto velB = bodyConstraintB->GetVelocity();

    const auto qA = UnitVec2::Get(posA.angular);
    const auto qB = UnitVec2::Get(posB.angular);

    m_rA = Rotate(m_localAnchorA - bodyConstraintA->GetLocalCenter(), qA);
    m_rB = Rotate(m_localAnchorB - bodyConstraintB->GetLocalCenter(), qB);

    // Get the pulley axes.
    const auto pulleyAxisA = Length2D{posA.linear + m_rA - m_groundAnchorA};
    const auto pulleyAxisB = Length2D{posB.linear + m_rB - m_groundAnchorB};

    m_uA = GetUnitVector(pulleyAxisA, UnitVec2::GetZero());
    m_uB = GetUnitVector(pulleyAxisB, UnitVec2::GetZero());

    // Compute effective mass.
    const auto ruA = Cross(m_rA, m_uA);
    const auto ruB = Cross(m_rB, m_uB);

    const auto totInvMassA = invMassA + (invRotInertiaA * Square(ruA)) / SquareRadian;
    const auto totInvMassB = invMassB + (invRotInertiaB * Square(ruB)) / SquareRadian;

    const auto totalInvMass = totInvMassA + m_ratio * m_ratio * totInvMassB;

    m_mass = (totalInvMass > InvMass{0})? Real{1} / totalInvMass: Mass{0};

    if (step.doWarmStart)
    {
        // Scale impulses to support variable time steps.
        m_impulse *= step.dtRatio;

        // Warm starting.
        const auto PA = -(m_impulse) * m_uA;
        const auto PB = (-m_ratio * m_impulse) * m_uB;

        velA += Velocity{invMassA * PA, invRotInertiaA * Cross(m_rA, PA) / Radian};
        velB += Velocity{invMassB * PB, invRotInertiaB * Cross(m_rB, PB) / Radian};
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

    const auto invMassA = bodyConstraintA->GetInvMass();
    const auto invRotInertiaA = bodyConstraintA->GetInvRotInertia();
    auto velA = bodyConstraintA->GetVelocity();

    const auto invMassB = bodyConstraintB->GetInvMass();
    const auto invRotInertiaB = bodyConstraintB->GetInvRotInertia();
    auto velB = bodyConstraintB->GetVelocity();

    const auto vpA = LinearVelocity2D{velA.linear + GetRevPerpendicular(m_rA) * (velA.angular / Radian)};
    const auto vpB = LinearVelocity2D{velB.linear + GetRevPerpendicular(m_rB) * (velB.angular / Radian)};

    const auto Cdot = LinearVelocity{-Dot(m_uA, vpA) - m_ratio * Dot(m_uB, vpB)};
    const auto impulse = -m_mass * Cdot;
    m_impulse += impulse;

    const auto PA = -impulse * m_uA;
    const auto PB = -m_ratio * impulse * m_uB;
    velA += Velocity{invMassA * PA, invRotInertiaA * Cross(m_rA, PA) / Radian};
    velB += Velocity{invMassB * PB, invRotInertiaB * Cross(m_rB, PB) / Radian};

    bodyConstraintA->SetVelocity(velA);
    bodyConstraintB->SetVelocity(velB);
    
    return impulse == Momentum(0);
}

bool PulleyJoint::SolvePositionConstraints(BodyConstraintsMap& bodies,
                                           const ConstraintSolverConf& conf) const
{
    auto& bodyConstraintA = At(bodies, GetBodyA());
    auto& bodyConstraintB = At(bodies, GetBodyB());

    const auto invMassA = bodyConstraintA->GetInvMass();
    const auto invRotInertiaA = bodyConstraintA->GetInvRotInertia();
    auto posA = bodyConstraintA->GetPosition();

    const auto invMassB = bodyConstraintB->GetInvMass();
    const auto invRotInertiaB = bodyConstraintB->GetInvRotInertia();
    auto posB = bodyConstraintB->GetPosition();

    const auto rA = Rotate(m_localAnchorA - bodyConstraintA->GetLocalCenter(),
                           UnitVec2::Get(posA.angular));
    const auto rB = Rotate(m_localAnchorB - bodyConstraintB->GetLocalCenter(),
                           UnitVec2::Get(posB.angular));

    // Get the pulley axes.
    const auto pA = Length2D{posA.linear + rA - m_groundAnchorA};
    auto lengthA = Length{0};
    const auto uA = GetUnitVector(pA, lengthA, UnitVec2::GetZero());

    const auto pB = Length2D{posB.linear + rB - m_groundAnchorB};
    auto lengthB = Length{0};
    const auto uB = GetUnitVector(pB, lengthB, UnitVec2::GetZero());

    // Compute effective mass.
    const auto ruA = Length{Cross(rA, uA)};
    const auto ruB = Length{Cross(rB, uB)};

    const auto totalInvMassA = invMassA + invRotInertiaA * ruA * ruA / SquareRadian;
    const auto totalInvMassB = invMassB + invRotInertiaB * ruB * ruB / SquareRadian;

    const auto totalInvMass = totalInvMassA + m_ratio * m_ratio * totalInvMassB;
    const auto mass = (totalInvMass > InvMass{0})? Real{1} / totalInvMass: Mass{0};

    const auto C = Length{m_constant - lengthA - (m_ratio * lengthB)};
    const auto linearError = Abs(C);

    const auto impulse = -mass * C;

    const auto PA = -impulse * uA;
    const auto PB = -m_ratio * impulse * uB;

    posA += Position{invMassA * PA, invRotInertiaA * Cross(rA, PA) / Radian};
    posB += Position{invMassB * PB, invRotInertiaB * Cross(rB, PB) / Radian};

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

Momentum2D PulleyJoint::GetLinearReaction() const
{
    return m_impulse * m_uB;
}

AngularMomentum PulleyJoint::GetAngularReaction() const
{
    return AngularMomentum{0};
}

Length playrho::GetCurrentLengthA(const PulleyJoint& joint)
{
    return GetLength(GetWorldPoint(*joint.GetBodyA(),
                                   joint.GetLocalAnchorA()) - joint.GetGroundAnchorA());
}

Length playrho::GetCurrentLengthB(const PulleyJoint& joint)
{
    return GetLength(GetWorldPoint(*joint.GetBodyB(),
                                   joint.GetLocalAnchorB()) - joint.GetGroundAnchorB());
}

void PulleyJoint::ShiftOrigin(const Length2D newOrigin)
{
    m_groundAnchorA -= newOrigin;
    m_groundAnchorB -= newOrigin;
}
