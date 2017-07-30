/*
* Original work Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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

#include <PlayRho/Dynamics/Joints/FrictionJoint.hpp>
#include <PlayRho/Dynamics/Body.hpp>
#include <PlayRho/Dynamics/StepConf.hpp>
#include <PlayRho/Dynamics/Contacts/BodyConstraint.hpp>

using namespace playrho;

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

FrictionJointDef::FrictionJointDef(Body* bA, Body* bB, const Length2D anchor) noexcept:
    super{super{JointType::Friction}.UseBodyA(bA).UseBodyB(bB)},
    localAnchorA{GetLocalPoint(*bA, anchor)},
    localAnchorB{GetLocalPoint(*bB, anchor)}
{
    // Intentionally empty.
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

void FrictionJoint::InitVelocityConstraints(BodyConstraintsMap& bodies, const StepConf& step,
                                            const ConstraintSolverConf&)
{
    auto& bodyConstraintA = At(bodies, GetBodyA());
    auto& bodyConstraintB = At(bodies, GetBodyB());
    const auto posA = bodyConstraintA->GetPosition();
    auto velA = bodyConstraintA->GetVelocity();
    const auto posB = bodyConstraintB->GetPosition();
    auto velB = bodyConstraintB->GetVelocity();

    // Compute the effective mass matrix.
    m_rA = Rotate(m_localAnchorA - bodyConstraintA->GetLocalCenter(), UnitVec2::Get(posA.angular));
    m_rB = Rotate(m_localAnchorB - bodyConstraintB->GetLocalCenter(), UnitVec2::Get(posB.angular));

    // J = [-I -r1_skew I r2_skew]
    //     [ 0       -1 0       1]
    // r_skew = [-ry; rx]

    // Matlab
    // K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
    //     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
    //     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

    const auto invMassA = bodyConstraintA->GetInvMass();
    const auto invMassB = bodyConstraintB->GetInvMass();
    const auto invRotInertiaA = bodyConstraintA->GetInvRotInertia();
    const auto invRotInertiaB = bodyConstraintB->GetInvRotInertia();

    {
        Mat22 K;
        const auto exx = InvMass{
            invMassA + invRotInertiaA * Square(m_rA.y) / SquareRadian +
            invMassB + invRotInertiaB * Square(m_rB.y) / SquareRadian
        };
        const auto exy = InvMass{
            -invRotInertiaA * m_rA.x * m_rA.y / SquareRadian +
            -invRotInertiaB * m_rB.x * m_rB.y / SquareRadian
        };
        const auto eyy = InvMass{
            invMassA + invRotInertiaA * Square(m_rA.x) / SquareRadian +
            invMassB + invRotInertiaB * Square(m_rB.x) / SquareRadian
        };
        K.ex.x = StripUnit(exx);
        K.ex.y = StripUnit(exy);
        K.ey.x = K.ex.y;
        K.ey.y = StripUnit(eyy);
        m_linearMass = Invert(K);
    }

    const auto invRotInertia = invRotInertiaA + invRotInertiaB;
    m_angularMass = (invRotInertia > InvRotInertia{0})? RotInertia{Real{1} / invRotInertia}: RotInertia{0};
    
    if (step.doWarmStart)
    {
        // Scale impulses to support a variable time step.
        m_linearImpulse *= step.dtRatio;
        m_angularImpulse *= step.dtRatio;

        const auto P = m_linearImpulse;

        // L * M * L T^-1 / QP is: L^2 M T^-1 QP^-1 which is: AngularMomentum.
        const auto crossAP = AngularMomentum{Cross(m_rA, P) / Radian};
        const auto crossBP = AngularMomentum{Cross(m_rB, P) / Radian}; // L * M * L T^-1 is: L^2 M T^-1
        
        velA -= Velocity{invMassA * P, invRotInertiaA * (crossAP + m_angularImpulse)};
        velB += Velocity{invMassB * P, invRotInertiaB * (crossBP + m_angularImpulse)};
    }
    else
    {
        m_linearImpulse = Momentum2D{0, 0};
        m_angularImpulse = AngularMomentum{0};
    }

    bodyConstraintA->SetVelocity(velA);
    bodyConstraintB->SetVelocity(velB);
}

bool FrictionJoint::SolveVelocityConstraints(BodyConstraintsMap& bodies, const StepConf& step)
{
    auto& bodyConstraintA = At(bodies, GetBodyA());
    auto& bodyConstraintB = At(bodies, GetBodyB());

    auto velA = bodyConstraintA->GetVelocity();
    const auto invRotInertiaA = bodyConstraintA->GetInvRotInertia();

    auto velB = bodyConstraintB->GetVelocity();
    const auto invRotInertiaB = bodyConstraintB->GetInvRotInertia();

    const auto h = step.GetTime();

    auto solved = true;

    // Solve angular friction
    {
        // L^2 M QP^-2 * QP T^-1 is: L^2 M QP^-1 T^-1 (SquareMeter * Kilogram / Second) / Radian
        //                           L^2 M QP^-1 T^-1
        const auto angularImpulse = AngularMomentum{-m_angularMass * (velB.angular - velA.angular)};

        const auto oldAngularImpulse = m_angularImpulse;
        const auto maxAngularImpulse = h * m_maxTorque;
        m_angularImpulse = Clamp(m_angularImpulse + angularImpulse, -maxAngularImpulse, maxAngularImpulse);
        const auto incAngularImpulse = m_angularImpulse - oldAngularImpulse;

        if (incAngularImpulse != AngularMomentum(0))
        {
            solved = false;
        }

        velA.angular -= invRotInertiaA * incAngularImpulse;
        velB.angular += invRotInertiaB * incAngularImpulse;
    }

    // Solve linear friction
    {
        const auto vb = LinearVelocity2D{velB.linear + (GetRevPerpendicular(m_rB) * (velB.angular / Radian))};
        const auto va = LinearVelocity2D{velA.linear + (GetRevPerpendicular(m_rA) * (velA.angular / Radian))};

        const auto unitlessImpulse = -Transform(GetVec2(vb - va), m_linearMass);
        const auto impulse = Momentum2D{
            unitlessImpulse.GetX() * Kilogram * MeterPerSecond,
            unitlessImpulse.GetY() * Kilogram * MeterPerSecond
        };
        const auto oldImpulse = m_linearImpulse;
        m_linearImpulse += impulse;

        const auto maxImpulse = h * m_maxForce;

        if (GetLengthSquared(m_linearImpulse) > Square(maxImpulse))
        {
            m_linearImpulse = GetUnitVector(m_linearImpulse, UnitVec2::GetZero()) * maxImpulse;
        }

        const auto incImpulse = Momentum2D{m_linearImpulse - oldImpulse};
        const auto angImpulseA = AngularMomentum{Cross(m_rA, incImpulse) / Radian};
        const auto angImpulseB = AngularMomentum{Cross(m_rB, incImpulse) / Radian};

        if (incImpulse != Momentum2D{0, 0})
        {
            solved = false;
        }

        velA -= Velocity{bodyConstraintA->GetInvMass() * incImpulse, invRotInertiaA * angImpulseA};
        velB += Velocity{bodyConstraintB->GetInvMass() * incImpulse, invRotInertiaB * angImpulseB};
    }

    bodyConstraintA->SetVelocity(velA);
    bodyConstraintB->SetVelocity(velB);
    
    return solved;
}

bool FrictionJoint::SolvePositionConstraints(BodyConstraintsMap& bodies, const ConstraintSolverConf& conf) const
{
    NOT_USED(bodies);
    NOT_USED(conf);

    return true;
}

Length2D FrictionJoint::GetAnchorA() const
{
    return GetWorldPoint(*GetBodyA(), GetLocalAnchorA());
}

Length2D FrictionJoint::GetAnchorB() const
{
    return GetWorldPoint(*GetBodyB(), GetLocalAnchorB());
}

Force2D FrictionJoint::GetReactionForce(Frequency inv_dt) const
{
    return inv_dt * m_linearImpulse;
}

Torque FrictionJoint::GetReactionTorque(Frequency inv_dt) const
{
    return inv_dt * m_angularImpulse;
}

void FrictionJoint::SetMaxForce(Force force)
{
    assert(IsValid(force) && (force >= Force{0}));
    m_maxForce = force;
}

Force FrictionJoint::GetMaxForce() const
{
    return m_maxForce;
}

void FrictionJoint::SetMaxTorque(Torque torque)
{
    assert(IsValid(torque) && (torque >= Torque{0}));
    m_maxTorque = torque;
}

Torque FrictionJoint::GetMaxTorque() const
{
    return m_maxTorque;
}

FrictionJointDef playrho::GetFrictionJointDef(const FrictionJoint& joint) noexcept
{
    auto def = FrictionJointDef{};
    
    Set(def, joint);
    
    def.localAnchorA = joint.GetLocalAnchorA();
    def.localAnchorB = joint.GetLocalAnchorB();
    def.maxForce = joint.GetMaxForce();
    def.maxTorque = joint.GetMaxTorque();
    
    return def;
}
