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
                                   const Length2D anchor1, const Length2D anchor2,
                                   Frequency freq, RealNum damp) noexcept:
    JointDef{JointType::Distance, bA, bB},
    localAnchorA{GetLocalPoint(*bA, anchor1)}, localAnchorB{GetLocalPoint(*bB, anchor2)},
    length{GetLength(anchor2 - anchor1)},
    frequency{freq}, dampingRatio{damp}
{
}

bool DistanceJoint::IsOkay(const DistanceJointDef& def) noexcept
{
    if (!Joint::IsOkay(def))
    {
        return false;
    }
    if (!(def.frequency >= Frequency{0}))
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
    m_frequency(def.frequency),
    m_dampingRatio(def.dampingRatio)
{
    assert(def.frequency >= Frequency{0});
}

void DistanceJoint::InitVelocityConstraints(BodyConstraintsMap& bodies,
                                            const StepConf& step,
                                            const ConstraintSolverConf& conf)
{
    auto& bodyConstraintA = bodies.at(GetBodyA());
    auto& bodyConstraintB = bodies.at(GetBodyB());

    const auto invMassA = bodyConstraintA->GetInvMass();
    const auto invRotInertiaA = bodyConstraintA->GetInvRotInertia(); // L^-2 M^-1 QP^2
    const auto invMassB = bodyConstraintB->GetInvMass();
    const auto invRotInertiaB = bodyConstraintB->GetInvRotInertia(); // L^-2 M^-1 QP^2

    const auto posA = bodyConstraintA->GetPosition();
    auto velA = bodyConstraintA->GetVelocity();

    const auto posB = bodyConstraintB->GetPosition();
    auto velB = bodyConstraintB->GetVelocity();

    const auto qA = UnitVec2{posA.angular};
    const auto qB = UnitVec2{posB.angular};

    m_rA = Rotate(m_localAnchorA - bodyConstraintA->GetLocalCenter(), qA);
    m_rB = Rotate(m_localAnchorB - bodyConstraintB->GetLocalCenter(), qB);
    const auto deltaLocation = Length2D{(posB.linear + m_rB) - (posA.linear + m_rA)};

    // Handle singularity.
    Length length = Length{0};
    m_u = GetUnitVector(deltaLocation, length);
    if (length <= conf.linearSlop)
    {
        m_u = UnitVec2::GetZero();
    }

    const auto crAu = Length{Cross(m_rA, m_u)} / Radian;
    const auto crBu = Length{Cross(m_rB, m_u)} / Radian;
    const auto invRotMassA = invRotInertiaA * Square(crAu);
    const auto invRotMassB = invRotInertiaB * Square(crBu);
    auto invMass = InvMass{invMassA + invRotMassA + invMassB + invRotMassB};

    // Compute the effective mass matrix.
    m_mass = (invMass != InvMass{0}) ? RealNum{1} / invMass: Mass{0};

    if (m_frequency > Frequency{0})
    {
        const auto C = length - m_length; // L

        // Frequency
        const auto omega = RealNum{2} * Pi * m_frequency;

        // Damping coefficient
        const auto d = RealNum{2} * m_mass * m_dampingRatio * omega; // M T^-1

        // Spring stiffness
        const auto k = m_mass * Square(omega); // M T^-2

        // magic formulas
        const auto h = step.GetTime();
        const auto gamma = Mass{h * (d + h * k)}; // T (M T^-1 + T M T^-2) = M
        m_invGamma = (gamma != Mass{0})? RealNum{1} / gamma: 0;
        m_bias = C * h * k * m_invGamma; // L T M T^-2 M^-1 = L T^-1

        invMass += m_invGamma;
        m_mass = (invMass != InvMass{0}) ? RealNum{1} / invMass: 0;
    }
    else
    {
        m_invGamma = InvMass{0};
        m_bias = LinearVelocity{0};
    }

    if (step.doWarmStart)
    {
        // Scale the impulse to support a variable time step.
        m_impulse *= step.dtRatio;

        const auto P = m_impulse * m_u;

        // P is M L T^-2
        // Cross(Length2D, P) is: M L^2 T^-1
        // inv rotational inertia is: L^-2 M^-1 QP^2
        // Product is: L^-2 M^-1 QP^2 M L^2 T^-1 = QP^2 T^-1
        const auto LA = Cross(m_rA, P) / Radian;
        const auto LB = Cross(m_rB, P) / Radian;
        velA -= Velocity{invMassA * P, invRotInertiaA * LA};
        velB += Velocity{invMassB * P, invRotInertiaB * LB};
    }
    else
    {
        m_impulse = 0;
    }

    bodyConstraintA->SetVelocity(velA);
    bodyConstraintB->SetVelocity(velB);
}

bool DistanceJoint::SolveVelocityConstraints(BodyConstraintsMap& bodies, const StepConf&)
{
    auto& bodyConstraintA = bodies.at(GetBodyA());
    auto& bodyConstraintB = bodies.at(GetBodyB());

    const auto invMassA = bodyConstraintA->GetInvMass();
    const auto invRotInertiaA = bodyConstraintA->GetInvRotInertia();
    const auto invMassB = bodyConstraintB->GetInvMass();
    const auto invRotInertiaB = bodyConstraintB->GetInvRotInertia();

    auto velA = bodyConstraintA->GetVelocity();
    auto velB = bodyConstraintB->GetVelocity();

    // Cdot = dot(u, v + cross(w, r))
    const auto vpA = velA.linear + GetRevPerpendicular(m_rA) * (velA.angular / Radian);
    const auto vpB = velB.linear + GetRevPerpendicular(m_rB) * (velB.angular / Radian);
    const auto Cdot = LinearVelocity{Dot(m_u, vpB - vpA)};

    const auto impulse = Momentum{-m_mass * (Cdot + m_bias + m_invGamma * m_impulse)};
    m_impulse += impulse;

    const auto P = impulse * m_u;
    const auto LA = Cross(m_rA, P) / Radian;
    const auto LB = Cross(m_rB, P) / Radian;
    velA -= Velocity{invMassA * P, invRotInertiaA * LA};
    velB += Velocity{invMassB * P, invRotInertiaB * LB};

    bodyConstraintA->SetVelocity(velA);
    bodyConstraintB->SetVelocity(velB);
    
    return impulse == Momentum{0};
}

bool DistanceJoint::SolvePositionConstraints(BodyConstraintsMap& bodies, const ConstraintSolverConf& conf) const
{
    if (m_frequency > Frequency{0})
    {
        // There is no position correction for soft distance constraints.
        return true;
    }

    auto& bodyConstraintA = bodies.at(GetBodyA());
    auto& bodyConstraintB = bodies.at(GetBodyB());

    const auto invMassA = bodyConstraintA->GetInvMass();
    const auto invRotInertiaA = bodyConstraintA->GetInvRotInertia();
    const auto invMassB = bodyConstraintB->GetInvMass();
    const auto invRotInertiaB = bodyConstraintB->GetInvRotInertia();

    auto posA = bodyConstraintA->GetPosition();
    auto posB = bodyConstraintB->GetPosition();

    const auto qA = UnitVec2(posA.angular);
    const auto qB = UnitVec2(posB.angular);

    const auto rA = Length2D{Rotate(m_localAnchorA - bodyConstraintA->GetLocalCenter(), qA)};
    const auto rB = Length2D{Rotate(m_localAnchorB - bodyConstraintB->GetLocalCenter(), qB)};
    const auto relLoc = Length2D{(posB.linear + rB) - (posA.linear + rA)};
    
    auto length = Length{0};
    const auto u = GetUnitVector(relLoc, length);
    const auto deltaLength = length - m_length;
    const auto C = Clamp(deltaLength, -conf.maxLinearCorrection, conf.maxLinearCorrection);

    const auto impulse = -m_mass * C;
    const auto P = impulse * u;

    posA -= Position{invMassA * P, invRotInertiaA * Cross(rA, P) / Radian};
    posB += Position{invMassB * P, invRotInertiaB * Cross(rB, P) / Radian};

    bodyConstraintA->SetPosition(posA);
    bodyConstraintB->SetPosition(posB);

    return Abs(C) < conf.linearSlop;
}

Length2D DistanceJoint::GetAnchorA() const
{
    return GetWorldPoint(*GetBodyA(), GetLocalAnchorA());
}

Length2D DistanceJoint::GetAnchorB() const
{
    return GetWorldPoint(*GetBodyB(), GetLocalAnchorB());
}

Force2D DistanceJoint::GetReactionForce(Frequency inv_dt) const
{
    return inv_dt * m_impulse * m_u;
}

Torque DistanceJoint::GetReactionTorque(Frequency inv_dt) const
{
    NOT_USED(inv_dt);
    return Torque{0};
}

DistanceJointDef box2d::GetDistanceJointDef(const DistanceJoint& joint) noexcept
{
    auto def = DistanceJointDef{};
    
    Set(def, joint);

    def.localAnchorA = joint.GetLocalAnchorA();
    def.localAnchorB = joint.GetLocalAnchorB();
    def.length = joint.GetLength();
    def.frequency = joint.GetFrequency();
    def.dampingRatio = joint.GetDampingRatio();

    return def;
}
