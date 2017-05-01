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

#include <Box2D/Dynamics/Joints/RevoluteJoint.hpp>
#include <Box2D/Dynamics/Body.hpp>
#include <Box2D/Dynamics/StepConf.hpp>
#include <Box2D/Dynamics/Contacts/ContactSolver.hpp>
#include <Box2D/Dynamics/Contacts/BodyConstraint.hpp>

using namespace box2d;

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

RevoluteJointDef::RevoluteJointDef(Body* bA, Body* bB, const Length2D anchor, bool cc):
    JointDef{JointType::Revolute, bA, bB, cc},
    localAnchorA{GetLocalPoint(*bA, anchor)},
    localAnchorB{GetLocalPoint(*bB, anchor)},
    referenceAngle{bB->GetAngle() - bA->GetAngle()}
{
    // Intentionally empty.
}

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

void RevoluteJoint::InitVelocityConstraints(BodyConstraints& bodies,
                                            const StepConf& step,
                                            const ConstraintSolverConf& conf)
{
    auto& bodiesA = bodies.at(GetBodyA());
    auto& bodiesB = bodies.at(GetBodyB());

    const auto aA = bodiesA.GetPosition().angular;
    auto velA = bodiesA.GetVelocity();

    const auto aB = bodiesB.GetPosition().angular;
    auto velB = bodiesB.GetVelocity();

    const auto qA = UnitVec2(aA);
    const auto qB = UnitVec2(aB);

    m_rA = Rotate(m_localAnchorA - bodiesA.GetLocalCenter(), qA);
    m_rB = Rotate(m_localAnchorB - bodiesB.GetLocalCenter(), qB);

    // J = [-I -r1_skew I r2_skew]
    //     [ 0       -1 0       1]
    // r_skew = [-ry; rx]

    // Matlab
    // K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
    //     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
    //     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

    const auto invMassA = bodiesA.GetInvMass();
    const auto invRotInertiaA = bodiesA.GetInvRotInertia();

    const auto invMassB = bodiesB.GetInvMass();
    const auto invRotInertiaB = bodiesB.GetInvRotInertia();
    
    const auto totInvI = invRotInertiaA + invRotInertiaB;

    const auto fixedRotation = (totInvI == InvRotInertia{0});

    const auto exx = InvMass{
        invMassA + (Square(m_rA.y) * invRotInertiaA / SquareRadian) +
        invMassB + (Square(m_rB.y) * invRotInertiaB / SquareRadian)
    };
    const auto eyx = InvMass{
        (-m_rA.y * m_rA.x * invRotInertiaA / SquareRadian) +
        (-m_rB.y * m_rB.x * invRotInertiaB / SquareRadian)
    };
    const auto ezx = InvMass{
        (-m_rA.y * invRotInertiaA * Meter / SquareRadian) +
        (-m_rB.y * invRotInertiaB * Meter / SquareRadian)
    };
    const auto eyy = InvMass{
        invMassA + (Square(m_rA.x) * invRotInertiaA / SquareRadian) +
        invMassB + (Square(m_rB.x) * invRotInertiaB / SquareRadian)
    };
    const auto ezy = InvMass{
        (m_rA.x * invRotInertiaA * Meter / SquareRadian) + (m_rB.x * invRotInertiaB * Meter / SquareRadian)
    };
    m_mass.ex.x = StripUnit(exx);
    m_mass.ey.x = StripUnit(eyx);
    m_mass.ez.x = StripUnit(ezx);
    m_mass.ex.y = m_mass.ey.x;
    m_mass.ey.y = StripUnit(eyy);
    m_mass.ez.y = StripUnit(ezy);
    m_mass.ex.z = m_mass.ez.x;
    m_mass.ey.z = m_mass.ez.y;
    m_mass.ez.z = StripUnit(totInvI);

    m_motorMass = (totInvI > InvRotInertia{0})? RotInertia{RealNum{1} / totInvI}: RotInertia{0};

    if (!m_enableMotor || fixedRotation)
    {
        m_motorImpulse = 0;
    }

    if (m_enableLimit && !fixedRotation)
    {
        const auto jointAngle = aB - aA - GetReferenceAngle();
        if (Abs(m_upperAngle - m_lowerAngle) < (RealNum{2} * conf.angularSlop))
        {
            m_limitState = e_equalLimits;
        }
        else if (jointAngle <= m_lowerAngle)
        {
            if (m_limitState != e_atLowerLimit)
            {
                m_impulse.z = 0;
            }
            m_limitState = e_atLowerLimit;
        }
        else if (jointAngle >= m_upperAngle)
        {
            if (m_limitState != e_atUpperLimit)
            {
                m_impulse.z = 0;
            }
            m_limitState = e_atUpperLimit;
        }
        else
        {
            m_limitState = e_inactiveLimit;
            m_impulse.z = 0;
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

        const auto P = Momentum2D{Vec2{m_impulse.x, m_impulse.y} * Kilogram * MeterPerSecond};
        
        // AngularMomentum is L^2 M T^-1 QP^-1.
        const auto L = AngularMomentum{
            m_motorImpulse + (m_impulse.z * SquareMeter * Kilogram / (Second * Radian))
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

    bodiesA.SetVelocity(velA);
    bodiesB.SetVelocity(velB);
}

RealNum RevoluteJoint::SolveVelocityConstraints(BodyConstraints& bodies, const StepConf& step)
{
    auto& bodiesA = bodies.at(GetBodyA());
    auto& bodiesB = bodies.at(GetBodyB());

    auto velA = bodiesA.GetVelocity();
    const auto invMassA = bodiesA.GetInvMass();
    const auto invRotInertiaA = bodiesA.GetInvRotInertia();

    auto velB = bodiesB.GetVelocity();
    const auto invMassB = bodiesB.GetInvMass();
    const auto invRotInertiaB = bodiesB.GetInvRotInertia();

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

    const auto vb = velB.linear + GetRevPerpendicular(m_rB) * velB.angular / Radian;
    const auto va = velA.linear + GetRevPerpendicular(m_rA) * velA.angular / Radian;

    // Solve limit constraint.
    if (m_enableLimit && (m_limitState != e_inactiveLimit) && !fixedRotation)
    {
        const auto Cdot1 = vb - va;
        const auto Cdot = Vec3{Cdot1.x / MeterPerSecond, Cdot1.y / MeterPerSecond, (velB.angular - velA.angular) / RadianPerSecond};
        auto impulse = -Solve33(m_mass, Cdot);

        auto UpdateImpulseProc = [&]() {
            const auto rhs = -Vec2{Cdot1.x / MeterPerSecond, Cdot1.y / MeterPerSecond} + m_impulse.z * Vec2{m_mass.ez.x, m_mass.ez.y};
            const auto reduced = Solve22(m_mass, rhs);
            impulse.x = reduced.x;
            impulse.y = reduced.y;
            impulse.z = -m_impulse.z;
            m_impulse.x += reduced.x;
            m_impulse.y += reduced.y;
            m_impulse.z = 0;
        };
        
        if (m_limitState == e_equalLimits)
        {
            m_impulse += impulse;
        }
        else if (m_limitState == e_atLowerLimit)
        {
            const auto newImpulse = m_impulse.z + impulse.z;
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
            const auto newImpulse = m_impulse.z + impulse.z;
            if (newImpulse > 0)
            {
                UpdateImpulseProc();
            }
            else
            {
                m_impulse += impulse;
            }
        }

        const auto P = Momentum2D{Vec2{impulse.x, impulse.y} * Kilogram * MeterPerSecond};
        const auto L = AngularMomentum{impulse.z * SquareMeter * Kilogram / (Second * Radian)};
        const auto LA = AngularMomentum{Cross(m_rA, P) / Radian} + L;
        const auto LB = AngularMomentum{Cross(m_rB, P) / Radian} + L;

        velA -= Velocity{invMassA * P, invRotInertiaA * LA};
        velB += Velocity{invMassB * P, invRotInertiaB * LB};
    }
    else
    {
        // Solve point-to-point constraint
        const auto Cdot = vb - va;
        const auto impulse = Solve22(m_mass, -Vec2{Cdot.x / MeterPerSecond, Cdot.y / MeterPerSecond});

        m_impulse.x += impulse.x;
        m_impulse.y += impulse.y;

        const auto P = Momentum2D{Vec2{impulse.x, impulse.y} * Kilogram * MeterPerSecond};
        const auto LA = AngularMomentum{Cross(m_rA, P) / Radian};
        const auto LB = AngularMomentum{Cross(m_rB, P) / Radian};

        velA -= Velocity{invMassA * P, invRotInertiaA * LA};
        velB += Velocity{invMassB * P, invRotInertiaB * LB};
    }

    bodiesA.SetVelocity(velA);
    bodiesB.SetVelocity(velB);

    return GetInvalid<RealNum>(); // TODO
}

bool RevoluteJoint::SolvePositionConstraints(BodyConstraints& bodies, const ConstraintSolverConf& conf) const
{
    auto& bodiesA = bodies.at(GetBodyA());
    auto& bodiesB = bodies.at(GetBodyB());

    auto posA = bodiesA.GetPosition();
    const auto invRotInertiaA = bodiesA.GetInvRotInertia();

    auto posB = bodiesB.GetPosition();
    const auto invRotInertiaB = bodiesB.GetInvRotInertia();

    const auto fixedRotation = ((invRotInertiaA + invRotInertiaB) == InvRotInertia{0});

    // Solve angular limit constraint.
    auto angularError = Angle{0};
    if (m_enableLimit && m_limitState != e_inactiveLimit && !fixedRotation)
    {
        const auto angle = posB.angular - posA.angular - GetReferenceAngle();

        // RotInertia is L^2 M QP^-2, Angle is QP, so RotInertia * Angle is L^2 M QP^-1.
        auto limitImpulse = RealNum{0} * SquareMeter * Kilogram / Radian;

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
            C = Clamp(C + conf.angularSlop, -conf.maxAngularCorrection, RealNum{0} * Radian);
            limitImpulse = -m_motorMass * C;
        }
        else if (m_limitState == e_atUpperLimit)
        {
            auto C = angle - m_upperAngle;
            angularError = C;

            // Prevent large angular corrections and allow some slop.
            C = Clamp(C - conf.angularSlop, RealNum{0} * Radian, conf.maxAngularCorrection);
            limitImpulse = -m_motorMass * C;
        }

        // InvRotInertia is L^-2 M^-1 QP^2, limitImpulse is L^2 M QP^-1, so product is QP.
        posA.angular -= invRotInertiaA * limitImpulse;
        posB.angular += invRotInertiaB * limitImpulse;
    }

    // Solve point-to-point constraint.
    auto positionError = Length{0};
    {
        const auto qA = UnitVec2(posA.angular);
        const auto qB = UnitVec2(posB.angular);

        const auto rA = Length2D{Rotate(m_localAnchorA - bodiesA.GetLocalCenter(), qA)};
        const auto rB = Length2D{Rotate(m_localAnchorB - bodiesB.GetLocalCenter(), qB)};

        const auto C = (posB.linear + rB) - (posA.linear + rA);
        positionError = GetLength(C);

        const auto invMassA = bodiesA.GetInvMass();
        const auto invMassB = bodiesB.GetInvMass();

        const auto exx = InvMass{
            invMassA + (invRotInertiaA * Square(rA.y) / SquareRadian) +
            invMassB + (invRotInertiaB * Square(rB.y) / SquareRadian)
        };
        const auto exy = InvMass{
            (-invRotInertiaA * rA.x * rA.y / SquareRadian) +
            (-invRotInertiaB * rB.x * rB.y / SquareRadian)
        };
        const auto eyy = InvMass{
            invMassA + (invRotInertiaA * Square(rA.x) / SquareRadian) +
            invMassB + (invRotInertiaB * Square(rB.x) / SquareRadian)
        };
        
        Mat22 K;
        K.ex.x = StripUnit(exx);
        K.ex.y = StripUnit(exy);
        K.ey.x = K.ex.y;
        K.ey.y = StripUnit(eyy);
        const auto P = -Solve(K, StripUnits(C)) * Kilogram * Meter;

        posA -= Position{invMassA * P, invRotInertiaA * Cross(rA, P) / Radian};
        posB += Position{invMassB * P, invRotInertiaB * Cross(rB, P) / Radian};
    }

    bodiesA.SetPosition(posA);
    bodiesB.SetPosition(posB);
    
    return (positionError <= conf.linearSlop) && (angularError <= conf.angularSlop);
}

Length2D RevoluteJoint::GetAnchorA() const
{
    return GetWorldPoint(*GetBodyA(), GetLocalAnchorA());
}

Length2D RevoluteJoint::GetAnchorB() const
{
    return GetWorldPoint(*GetBodyB(), GetLocalAnchorB());
}

Force2D RevoluteJoint::GetReactionForce(Frequency inv_dt) const
{
    return inv_dt * Vec2{m_impulse.x, m_impulse.y} * Kilogram * MeterPerSecond;
}

Torque RevoluteJoint::GetReactionTorque(Frequency inv_dt) const
{
    // AngularMomentum is L^2 M T^-1 QP^-1.
    return inv_dt * m_impulse.z * SquareMeter * Kilogram / (Second * Radian);
}

void RevoluteJoint::EnableMotor(bool flag)
{
    GetBodyA()->SetAwake();
    GetBodyB()->SetAwake();
    m_enableMotor = flag;
}

Torque RevoluteJoint::GetMotorTorque(Frequency inv_dt) const
{
    return inv_dt * m_motorImpulse;
}

void RevoluteJoint::SetMotorSpeed(AngularVelocity speed)
{
    GetBodyA()->SetAwake();
    GetBodyB()->SetAwake();
    m_motorSpeed = speed;
}

void RevoluteJoint::SetMaxMotorTorque(Torque torque)
{
    GetBodyA()->SetAwake();
    GetBodyB()->SetAwake();
    m_maxMotorTorque = torque;
}

void RevoluteJoint::EnableLimit(bool flag)
{
    if (flag != m_enableLimit)
    {
        GetBodyA()->SetAwake();
        GetBodyB()->SetAwake();
        m_enableLimit = flag;
        m_impulse.z = 0;
    }
}

void RevoluteJoint::SetLimits(Angle lower, Angle upper)
{
    assert(lower <= upper);
    
    if ((lower != m_lowerAngle) || (upper != m_upperAngle))
    {
        GetBodyA()->SetAwake();
        GetBodyB()->SetAwake();
        m_impulse.z = 0;
        m_lowerAngle = lower;
        m_upperAngle = upper;
    }
}

Angle box2d::GetJointAngle(const RevoluteJoint& joint)
{
    return joint.GetBodyB()->GetAngle() - joint.GetBodyA()->GetAngle() - joint.GetReferenceAngle();
}

AngularVelocity box2d::GetJointSpeed(const RevoluteJoint& joint)
{
    return joint.GetBodyB()->GetVelocity().angular - joint.GetBodyA()->GetVelocity().angular;
}
