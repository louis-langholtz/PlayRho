/*
 * Original work Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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

#include <PlayRho/Dynamics/Joints/RevoluteJoint.hpp>
#include <PlayRho/Dynamics/Body.hpp>
#include <PlayRho/Dynamics/StepConf.hpp>
#include <PlayRho/Dynamics/Contacts/ContactSolver.hpp>
#include <PlayRho/Dynamics/Contacts/BodyConstraint.hpp>

using namespace playrho;

// Point-to-point constraint
// C = p2 - p1
// Cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-I -r1_skew I r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

// Motor constraint
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2

RevoluteJoint::RevoluteJoint(const RevoluteJointDef& def):
    Joint{def},
    m_localAnchorA{def.localAnchorA},
    m_localAnchorB{def.localAnchorB},
    m_enableMotor{def.enableMotor},
    m_maxMotorTorque{def.maxMotorTorque},
    m_motorSpeed{def.motorSpeed},
    m_enableLimit{def.enableLimit},
    m_referenceAngle{def.referenceAngle},
    m_lowerAngle{def.lowerAngle},
    m_upperAngle{def.upperAngle}
{
    // Intentionally empty.
}

void RevoluteJoint::InitVelocityConstraints(BodyConstraintsMap& bodies,
                                            const StepConf& step,
                                            const ConstraintSolverConf& conf)
{
    auto& bodyConstraintA = At(bodies, GetBodyA());
    auto& bodyConstraintB = At(bodies, GetBodyB());

    const auto invMassA = bodyConstraintA->GetInvMass();
    const auto invRotInertiaA = bodyConstraintA->GetInvRotInertia();
    const auto aA = bodyConstraintA->GetPosition().angular;
    auto velA = bodyConstraintA->GetVelocity();

    const auto invMassB = bodyConstraintB->GetInvMass();
    const auto invRotInertiaB = bodyConstraintB->GetInvRotInertia();
    const auto aB = bodyConstraintB->GetPosition().angular;
    auto velB = bodyConstraintB->GetVelocity();

    const auto qA = UnitVec2::Get(aA);
    const auto qB = UnitVec2::Get(aB);

    m_rA = Rotate(m_localAnchorA - bodyConstraintA->GetLocalCenter(), qA);
    m_rB = Rotate(m_localAnchorB - bodyConstraintB->GetLocalCenter(), qB);

    // J = [-I -r1_skew I r2_skew]
    //     [ 0       -1 0       1]
    // r_skew = [-ry; rx]

    // Matlab
    // K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
    //     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
    //     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]
    
    const auto totInvI = invRotInertiaA + invRotInertiaB;

    const auto fixedRotation = (totInvI == InvRotInertia{0});

    const auto exx = InvMass{
        invMassA + (Square(GetY(m_rA)) * invRotInertiaA / SquareRadian) +
        invMassB + (Square(GetY(m_rB)) * invRotInertiaB / SquareRadian)
    };
    const auto eyx = InvMass{
        (-GetY(m_rA) * GetX(m_rA) * invRotInertiaA / SquareRadian) +
        (-GetY(m_rB) * GetX(m_rB) * invRotInertiaB / SquareRadian)
    };
    const auto ezx = InvMass{
        (-GetY(m_rA) * invRotInertiaA * Meter / SquareRadian) +
        (-GetY(m_rB) * invRotInertiaB * Meter / SquareRadian)
    };
    const auto eyy = InvMass{
        invMassA + (Square(GetX(m_rA)) * invRotInertiaA / SquareRadian) +
        invMassB + (Square(GetX(m_rB)) * invRotInertiaB / SquareRadian)
    };
    const auto ezy = InvMass{
        (GetX(m_rA) * invRotInertiaA * Meter / SquareRadian) +
        (GetX(m_rB) * invRotInertiaB * Meter / SquareRadian)
    };
    GetX(GetX(m_mass)) = StripUnit(exx);
    GetX(GetY(m_mass)) = StripUnit(eyx);
    GetX(GetZ(m_mass)) = StripUnit(ezx);
    GetY(GetX(m_mass)) = GetX(GetY(m_mass));
    GetY(GetY(m_mass)) = StripUnit(eyy);
    GetY(GetZ(m_mass)) = StripUnit(ezy);
    GetZ(GetX(m_mass)) = GetX(GetZ(m_mass));
    GetZ(GetY(m_mass)) = GetY(GetZ(m_mass));
    GetZ(GetZ(m_mass)) = StripUnit(totInvI);

    m_motorMass = (totInvI > InvRotInertia{0})? RotInertia{Real{1} / totInvI}: RotInertia{0};

    if (!m_enableMotor || fixedRotation)
    {
        m_motorImpulse = 0;
    }

    if (m_enableLimit && !fixedRotation)
    {
        const auto jointAngle = aB - aA - GetReferenceAngle();
        if (Abs(m_upperAngle - m_lowerAngle) < (Real{2} * conf.angularSlop))
        {
            m_limitState = e_equalLimits;
        }
        else if (jointAngle <= m_lowerAngle)
        {
            if (m_limitState != e_atLowerLimit)
            {
                m_limitState = e_atLowerLimit;
                GetZ(m_impulse) = 0;
            }
        }
        else if (jointAngle >= m_upperAngle)
        {
            if (m_limitState != e_atUpperLimit)
            {
                m_limitState = e_atUpperLimit;
                GetZ(m_impulse) = 0;
            }
        }
        else
        {
            m_limitState = e_inactiveLimit;
            GetZ(m_impulse) = 0;
        }
    }
    else
    {
        m_limitState = e_inactiveLimit;
    }

    if (step.doWarmStart)
    {
        // Scale impulses to support a variable time step.
        m_impulse *= step.dtRatio;
        m_motorImpulse *= step.dtRatio;

        const auto P = Momentum2D{
            GetX(m_impulse) * Kilogram * MeterPerSecond,
            GetY(m_impulse) * Kilogram * MeterPerSecond
        };
        
        // AngularMomentum is L^2 M T^-1 QP^-1.
        const auto L = AngularMomentum{
            m_motorImpulse + (GetZ(m_impulse) * SquareMeter * Kilogram / (Second * Radian))
        };
        const auto LA = AngularMomentum{Cross(m_rA, P) / Radian} + L;
        const auto LB = AngularMomentum{Cross(m_rB, P) / Radian} + L;

        velA -= Velocity{invMassA * P, invRotInertiaA * LA};
        velB += Velocity{invMassB * P, invRotInertiaB * LB};
    }
    else
    {
        m_impulse = Vec3_zero;
        m_motorImpulse = 0;
    }

    bodyConstraintA->SetVelocity(velA);
    bodyConstraintB->SetVelocity(velB);
}

bool RevoluteJoint::SolveVelocityConstraints(BodyConstraintsMap& bodies, const StepConf& step)
{
    auto& bodyConstraintA = At(bodies, GetBodyA());
    auto& bodyConstraintB = At(bodies, GetBodyB());

    const auto oldVelA = bodyConstraintA->GetVelocity();
    auto velA = oldVelA;
    const auto invMassA = bodyConstraintA->GetInvMass();
    const auto invRotInertiaA = bodyConstraintA->GetInvRotInertia();

    const auto oldVelB = bodyConstraintB->GetVelocity();
    auto velB = oldVelB;
    const auto invMassB = bodyConstraintB->GetInvMass();
    const auto invRotInertiaB = bodyConstraintB->GetInvRotInertia();

    const auto fixedRotation = (invRotInertiaA + invRotInertiaB == InvRotInertia{0});

    // Solve motor constraint.
    if (m_enableMotor && (m_limitState != e_equalLimits) && !fixedRotation)
    {
        const auto impulse = AngularMomentum{-m_motorMass * (velB.angular - velA.angular - m_motorSpeed)};
        const auto oldImpulse = m_motorImpulse;
        const auto maxImpulse = step.GetTime() * m_maxMotorTorque;
        m_motorImpulse = Clamp(m_motorImpulse + impulse, -maxImpulse, maxImpulse);
        const auto incImpulse = m_motorImpulse - oldImpulse;

        velA.angular -= invRotInertiaA * incImpulse;
        velB.angular += invRotInertiaB * incImpulse;
    }

    const auto vb = velB.linear + GetRevPerpendicular(m_rB) * (velB.angular / Radian);
    const auto va = velA.linear + GetRevPerpendicular(m_rA) * (velA.angular / Radian);
    const auto vDelta = vb - va;

    // Solve limit constraint.
    if (m_enableLimit && (m_limitState != e_inactiveLimit) && !fixedRotation)
    {
        const auto Cdot = Vec3{
            GetX(vDelta) / MeterPerSecond,
            GetY(vDelta) / MeterPerSecond,
            (velB.angular - velA.angular) / RadianPerSecond
        };
        auto impulse = -Solve33(m_mass, Cdot);

        auto UpdateImpulseProc = [&]() {
            const auto rhs = -Vec2{
                GetX(vDelta) / MeterPerSecond,
                GetY(vDelta) / MeterPerSecond
            } + GetZ(m_impulse) * Vec2{GetX(GetZ(m_mass)), GetY(GetZ(m_mass))};
            const auto reduced = Solve22(m_mass, rhs);
            GetX(impulse) = GetX(reduced);
            GetY(impulse) = GetY(reduced);
            GetZ(impulse) = -GetZ(m_impulse);
            GetX(m_impulse) += GetX(reduced);
            GetY(m_impulse) += GetY(reduced);
            GetZ(m_impulse) = 0;
        };
        
        if (m_limitState == e_equalLimits)
        {
            m_impulse += impulse;
        }
        else if (m_limitState == e_atLowerLimit)
        {
            const auto newImpulse = GetZ(m_impulse) + GetZ(impulse);
            if (newImpulse < 0)
            {
                UpdateImpulseProc();
            }
            else
            {
                m_impulse += impulse;
            }
        }
        else if (m_limitState == e_atUpperLimit)
        {
            const auto newImpulse = GetZ(m_impulse) + GetZ(impulse);
            if (newImpulse > 0)
            {
                UpdateImpulseProc();
            }
            else
            {
                m_impulse += impulse;
            }
        }

        const auto P = Momentum2D{
            GetX(impulse) * Kilogram * MeterPerSecond,
            GetY(impulse) * Kilogram * MeterPerSecond
        };
        const auto L = AngularMomentum{GetZ(impulse) * SquareMeter * Kilogram / (Second * Radian)};
        const auto LA = AngularMomentum{Cross(m_rA, P) / Radian} + L;
        const auto LB = AngularMomentum{Cross(m_rB, P) / Radian} + L;

        velA -= Velocity{invMassA * P, invRotInertiaA * LA};
        velB += Velocity{invMassB * P, invRotInertiaB * LB};
    }
    else
    {
        // Solve point-to-point constraint
        const auto impulse = Solve22(m_mass, -Vec2{
            Get<0>(vDelta) / MeterPerSecond, Get<1>(vDelta) / MeterPerSecond
        });

        GetX(m_impulse) += GetX(impulse);
        GetY(m_impulse) += GetY(impulse);

        const auto P = Momentum2D{
            GetX(impulse) * Kilogram * MeterPerSecond,
            GetY(impulse) * Kilogram * MeterPerSecond
        };
        const auto LA = AngularMomentum{Cross(m_rA, P) / Radian};
        const auto LB = AngularMomentum{Cross(m_rB, P) / Radian};

        velA -= Velocity{invMassA * P, invRotInertiaA * LA};
        velB += Velocity{invMassB * P, invRotInertiaB * LB};
    }

    if ((velA != oldVelA) || (velB != oldVelB))
    {
	    bodyConstraintA->SetVelocity(velA);
    	bodyConstraintB->SetVelocity(velB);
        return false;
    }
    return true;
}

bool RevoluteJoint::SolvePositionConstraints(BodyConstraintsMap& bodies, const ConstraintSolverConf& conf) const
{
    auto& bodyConstraintA = At(bodies, GetBodyA());
    auto& bodyConstraintB = At(bodies, GetBodyB());

    auto posA = bodyConstraintA->GetPosition();
    const auto invRotInertiaA = bodyConstraintA->GetInvRotInertia();

    auto posB = bodyConstraintB->GetPosition();
    const auto invRotInertiaB = bodyConstraintB->GetInvRotInertia();

    const auto fixedRotation = ((invRotInertiaA + invRotInertiaB) == InvRotInertia{0});

    // Solve angular limit constraint.
    auto angularError = Angle{0};
    if (m_enableLimit && m_limitState != e_inactiveLimit && !fixedRotation)
    {
        const auto angle = posB.angular - posA.angular - GetReferenceAngle();

        // RotInertia is L^2 M QP^-2, Angle is QP, so RotInertia * Angle is L^2 M QP^-1.
        auto limitImpulse = Real{0} * SquareMeter * Kilogram / Radian;

        if (m_limitState == e_equalLimits)
        {
            // Prevent large angular corrections
            const auto C = Clamp(angle - m_lowerAngle, -conf.maxAngularCorrection, conf.maxAngularCorrection);
            limitImpulse = -m_motorMass * C;
            angularError = Abs(C);
        }
        else if (m_limitState == e_atLowerLimit)
        {
            auto C = angle - m_lowerAngle;
            angularError = -C;

            // Prevent large angular corrections and allow some slop.
            C = Clamp(C + conf.angularSlop, -conf.maxAngularCorrection, Real{0} * Radian);
            limitImpulse = -m_motorMass * C;
        }
        else if (m_limitState == e_atUpperLimit)
        {
            auto C = angle - m_upperAngle;
            angularError = C;

            // Prevent large angular corrections and allow some slop.
            C = Clamp(C - conf.angularSlop, Real{0} * Radian, conf.maxAngularCorrection);
            limitImpulse = -m_motorMass * C;
        }

        // InvRotInertia is L^-2 M^-1 QP^2, limitImpulse is L^2 M QP^-1, so product is QP.
        posA.angular -= invRotInertiaA * limitImpulse;
        posB.angular += invRotInertiaB * limitImpulse;
    }

    // Solve point-to-point constraint.
    auto positionError = Area{0};
    {
        const auto qA = UnitVec2::Get(posA.angular);
        const auto qB = UnitVec2::Get(posB.angular);

        const auto rA = Length2D{Rotate(m_localAnchorA - bodyConstraintA->GetLocalCenter(), qA)};
        const auto rB = Length2D{Rotate(m_localAnchorB - bodyConstraintB->GetLocalCenter(), qB)};

        const auto C = (posB.linear + rB) - (posA.linear + rA);
        positionError = GetLengthSquared(C);

        const auto invMassA = bodyConstraintA->GetInvMass();
        const auto invMassB = bodyConstraintB->GetInvMass();

        const auto exx = InvMass{
            invMassA + (invRotInertiaA * Square(GetY(rA)) / SquareRadian) +
            invMassB + (invRotInertiaB * Square(GetY(rB)) / SquareRadian)
        };
        const auto exy = InvMass{
            (-invRotInertiaA * GetX(rA) * GetY(rA) / SquareRadian) +
            (-invRotInertiaB * GetX(rB) * GetY(rB) / SquareRadian)
        };
        const auto eyy = InvMass{
            invMassA + (invRotInertiaA * Square(GetX(rA)) / SquareRadian) +
            invMassB + (invRotInertiaB * Square(GetX(rB)) / SquareRadian)
        };
        
        InvMass22 K;
        GetX(GetX(K)) = exx;
        GetY(GetX(K)) = exy;
        GetX(GetY(K)) = exy;
        GetY(GetY(K)) = eyy;
        const auto P = -Solve(K, C);

        posA -= Position{invMassA * P, invRotInertiaA * Cross(rA, P) / Radian};
        posB += Position{invMassB * P, invRotInertiaB * Cross(rB, P) / Radian};
    }

    bodyConstraintA->SetPosition(posA);
    bodyConstraintB->SetPosition(posB);
    
    return (positionError <= Square(conf.linearSlop)) && (angularError <= conf.angularSlop);
}

Length2D RevoluteJoint::GetAnchorA() const
{
    return GetWorldPoint(*GetBodyA(), GetLocalAnchorA());
}

Length2D RevoluteJoint::GetAnchorB() const
{
    return GetWorldPoint(*GetBodyB(), GetLocalAnchorB());
}

Momentum2D RevoluteJoint::GetLinearReaction() const
{
    return Momentum2D{
        GetX(m_impulse) * Kilogram * MeterPerSecond,
        GetY(m_impulse) * Kilogram * MeterPerSecond
    };
}

AngularMomentum RevoluteJoint::GetAngularReaction() const
{
    // AngularMomentum is L^2 M T^-1 QP^-1.
    return GetZ(m_impulse) * SquareMeter * Kilogram / (Second * Radian);
}

void RevoluteJoint::EnableMotor(bool flag)
{
    if (m_enableMotor != flag)
    {
    	m_enableMotor = flag;
        
        // XXX Should these be called regardless of whether the state changed?
	    GetBodyA()->SetAwake();
    	GetBodyB()->SetAwake();
    }
}

Torque RevoluteJoint::GetMotorTorque(Frequency inv_dt) const
{
    return inv_dt * m_motorImpulse;
}

void RevoluteJoint::SetMotorSpeed(AngularVelocity speed)
{
    if (m_motorSpeed != speed)
    {
	    m_motorSpeed = speed;

        // XXX Should these be called regardless of whether the state changed?
    	GetBodyA()->SetAwake();
    	GetBodyB()->SetAwake();
    }
}

void RevoluteJoint::SetMaxMotorTorque(Torque torque)
{
    if (m_maxMotorTorque != torque)
    {
	    m_maxMotorTorque = torque;

        // XXX Should these be called regardless of whether the state changed?
    	GetBodyA()->SetAwake();
    	GetBodyB()->SetAwake();
    }
}

void RevoluteJoint::EnableLimit(bool flag)
{
    if (flag != m_enableLimit)
    {
        m_enableLimit = flag;
        GetZ(m_impulse) = 0;

        GetBodyA()->SetAwake();
        GetBodyB()->SetAwake();
    }
}

void RevoluteJoint::SetLimits(Angle lower, Angle upper)
{
    assert(lower <= upper);
    
    if ((lower != m_lowerAngle) || (upper != m_upperAngle))
    {
        GetZ(m_impulse) = 0;
        m_lowerAngle = lower;
        m_upperAngle = upper;

        GetBodyA()->SetAwake();
        GetBodyB()->SetAwake();
    }
}

Angle playrho::GetJointAngle(const RevoluteJoint& joint)
{
    return joint.GetBodyB()->GetAngle() - joint.GetBodyA()->GetAngle() - joint.GetReferenceAngle();
}

AngularVelocity playrho::GetAngularVelocity(const RevoluteJoint& joint)
{
    return joint.GetBodyB()->GetVelocity().angular - joint.GetBodyA()->GetVelocity().angular;
}
