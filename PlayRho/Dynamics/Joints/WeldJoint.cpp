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

#include <PlayRho/Dynamics/Joints/WeldJoint.hpp>
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

// Angle constraint
// C = angle2 - angle1 - referenceAngle
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2

WeldJoint::WeldJoint(const WeldJointDef& def):
    Joint(def),
    m_localAnchorA(def.localAnchorA),
    m_localAnchorB(def.localAnchorB),
    m_referenceAngle(def.referenceAngle),
    m_frequency(def.frequency),
    m_dampingRatio(def.dampingRatio)
{
    // Intentionally empty.
}

void WeldJoint::InitVelocityConstraints(BodyConstraintsMap& bodies, const StepConf& step,
                                        const ConstraintSolverConf&)
{
    auto& bodyConstraintA = At(bodies, GetBodyA());
    auto& bodyConstraintB = At(bodies, GetBodyB());

    auto velA = bodyConstraintA->GetVelocity();
    const auto posA = bodyConstraintA->GetPosition();
    const auto invMassA = bodyConstraintA->GetInvMass();
    const auto invRotInertiaA = bodyConstraintA->GetInvRotInertia();

    auto velB = bodyConstraintB->GetVelocity();
    const auto posB = bodyConstraintB->GetPosition();
    const auto invMassB = bodyConstraintB->GetInvMass();
    const auto invRotInertiaB = bodyConstraintB->GetInvRotInertia();

    const auto qA = UnitVec2::Get(posA.angular);
    const auto qB = UnitVec2::Get(posB.angular);

    m_rA = Rotate(m_localAnchorA - bodyConstraintA->GetLocalCenter(), qA);
    m_rB = Rotate(m_localAnchorB - bodyConstraintB->GetLocalCenter(), qB);

    // J = [-I -r1_skew I r2_skew]
    //     [ 0       -1 0       1]
    // r_skew = [-ry; rx]

    // Matlab
    // K = [ invMassA+r1y^2*invRotInertiaA+invMassB+r2y^2*invRotInertiaB,  -r1y*invRotInertiaA*r1x-r2y*invRotInertiaB*r2x,          -r1y*invRotInertiaA-r2y*invRotInertiaB]
    //     [  -r1y*invRotInertiaA*r1x-r2y*invRotInertiaB*r2x, invMassA+r1x^2*invRotInertiaA+invMassB+r2x^2*invRotInertiaB,           r1x*invRotInertiaA+r2x*invRotInertiaB]
    //     [          -r1y*invRotInertiaA-r2y*invRotInertiaB,           r1x*invRotInertiaA+r2x*invRotInertiaB,                   invRotInertiaA+invRotInertiaB]

    Mat33 K;
    const auto exx = InvMass{
        invMassA + Square(m_rA.y) * invRotInertiaA / SquareRadian +
        invMassB + Square(m_rB.y) * invRotInertiaB / SquareRadian
    };
    const auto eyx = InvMass{
        -m_rA.y * m_rA.x * invRotInertiaA / SquareRadian +
        -m_rB.y * m_rB.x * invRotInertiaB / SquareRadian
    };
    const auto ezx = InvMass{
        -m_rA.y * invRotInertiaA * Meter / SquareRadian +
        -m_rB.y * invRotInertiaB * Meter / SquareRadian
    };
    const auto eyy = InvMass{
        invMassA + Square(m_rA.x) * invRotInertiaA / SquareRadian +
        invMassB + Square(m_rB.x) * invRotInertiaB / SquareRadian
    };
    const auto ezy = InvMass{
        m_rA.x * invRotInertiaA * Meter / SquareRadian +
        m_rB.x * invRotInertiaB * Meter / SquareRadian
    };
    const auto ezz = InvMass{(invRotInertiaA + invRotInertiaB) * SquareMeter / SquareRadian};

    K.ex.x = StripUnit(exx);
    K.ey.x = StripUnit(eyx);
    K.ez.x = StripUnit(ezx);
    K.ex.y = K.ey.x;
    K.ey.y = StripUnit(eyy);
    K.ez.y = StripUnit(ezy);
    K.ex.z = K.ez.x;
    K.ey.z = K.ez.y;
    K.ez.z = StripUnit(ezz);

    if (m_frequency > Frequency{0})
    {
        m_mass = GetInverse22(K);

        // InvRotInertia is L^-2 M^-1 QP^2
        //    RotInertia is L^2  M    QP^-2
        auto invRotInertia = InvRotInertia{invRotInertiaA + invRotInertiaB};
        const auto rotInertia = (invRotInertia > InvRotInertia{0})? Real{1} / invRotInertia: RotInertia{0};

        const auto C = Angle{posB.angular - posA.angular - m_referenceAngle};
        const auto omega = Real(2) * Pi * m_frequency; // T^-1
        const auto d = Real(2) * rotInertia * m_dampingRatio * omega;

        // Spring stiffness: L^2 M QP^-2 T^-2
        const auto k = rotInertia * omega * omega;

        // magic formulas
        const auto h = step.GetTime();
        const auto invGamma = RotInertia{h * (d + h * k)};
        m_gamma = (invGamma != RotInertia{0})? Real{1} / invGamma: InvRotInertia{0};
        // QP * T * L^2 M QP^-2 T^-2 * L^-2 M^-1 QP^2 is: QP T^-1
        m_bias = AngularVelocity{C * h * k * m_gamma};

        invRotInertia += m_gamma;
        m_mass.ez.z = StripUnit((invRotInertia != InvRotInertia{0}) ? Real{1} / invRotInertia : RotInertia{0});
    }
    else if (K.ez.z == 0)
    {
        m_mass = GetInverse22(K);
        m_gamma = InvRotInertia{0};
        m_bias = AngularVelocity{0};
    }
    else
    {
        m_mass = GetSymInverse33(K);
        m_gamma = InvRotInertia{0};
        m_bias = AngularVelocity{0};
    }

    if (step.doWarmStart)
    {
        // Scale impulses to support a variable time step.
        m_impulse *= step.dtRatio;

        const auto P = Momentum2D{
            m_impulse.x * Kilogram * MeterPerSecond,
            m_impulse.y * Kilogram * MeterPerSecond
        };

        // AngularMomentum is L^2 M T^-1 QP^-1.
        const auto L = AngularMomentum{m_impulse.z * SquareMeter * Kilogram / (Second * Radian)};
        const auto LA = L + AngularMomentum{Cross(m_rA, P) / Radian};
        const auto LB = L + AngularMomentum{Cross(m_rB, P) / Radian};

        velA -= Velocity{invMassA * P, invRotInertiaA * LA};
        velB += Velocity{invMassB * P, invRotInertiaB * LB};
    }
    else
    {
        m_impulse = Vec3_zero;
    }

    bodyConstraintA->SetVelocity(velA);
    bodyConstraintB->SetVelocity(velB);
}

bool WeldJoint::SolveVelocityConstraints(BodyConstraintsMap& bodies, const StepConf&)
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

    if (m_frequency > Frequency{0})
    {
        const auto Cdot2 = velB.angular - velA.angular;
        
        // InvRotInertia is L^-2 M^-1 QP^2. Angular velocity is QP T^-1
        const auto gamma = AngularVelocity{m_gamma * m_impulse.z * SquareMeter * Kilogram / (Radian * Second)};

        // AngularMomentum is L^2 M T^-1 QP^-1.
        const auto impulse2 = -m_mass.ez.z * StripUnit(Cdot2 + m_bias + gamma);
        m_impulse.z += impulse2;

        velA.angular -= AngularVelocity{invRotInertiaA * impulse2 * SquareMeter * Kilogram / (Second * Radian)};
        velB.angular += AngularVelocity{invRotInertiaB * impulse2 * SquareMeter * Kilogram / (Second * Radian)};

        const auto vb = velB.linear + LinearVelocity2D{(GetRevPerpendicular(m_rB) * (velB.angular / Radian))};
        const auto va = velA.linear + LinearVelocity2D{(GetRevPerpendicular(m_rA) * (velA.angular / Radian))};

        const auto Cdot1 = vb - va;

        const auto impulse1 = -Transform(Vec2{Cdot1.x / MeterPerSecond, Cdot1.y / MeterPerSecond}, m_mass);
        m_impulse.x += impulse1.x;
        m_impulse.y += impulse1.y;

        const auto P = Momentum2D{
            impulse1.x * Kilogram * MeterPerSecond,
            impulse1.y * Kilogram * MeterPerSecond
        };
        const auto LA = AngularMomentum{Cross(m_rA, P) / Radian};
        const auto LB = AngularMomentum{Cross(m_rB, P) / Radian};

        velA -= Velocity{invMassA * P, invRotInertiaA * LA};
        velB += Velocity{invMassB * P, invRotInertiaB * LB};
    }
    else
    {
        const auto vb = velB.linear + LinearVelocity2D{(GetRevPerpendicular(m_rB) * (velB.angular / Radian))};
        const auto va = velA.linear + LinearVelocity2D{(GetRevPerpendicular(m_rA) * (velA.angular / Radian))};

        const auto Cdot1 = vb - va;
        const auto Cdot2 = Real{(velB.angular - velA.angular) / RadianPerSecond};
        const auto Cdot = Vec3{Cdot1.x / MeterPerSecond, Cdot1.y / MeterPerSecond, Cdot2};

        const auto impulse = -Transform(Cdot, m_mass);
        m_impulse += impulse;

        const auto P = Momentum2D{
            impulse.x * Kilogram * MeterPerSecond,
            impulse.y * Kilogram * MeterPerSecond
        };
        
        // AngularMomentum is L^2 M T^-1 QP^-1.
        const auto L = AngularMomentum{impulse.z * SquareMeter * Kilogram / (Second * Radian)};
        const auto LA = L + AngularMomentum{Cross(m_rA, P) / Radian};
        const auto LB = L + AngularMomentum{Cross(m_rB, P) / Radian};

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

bool WeldJoint::SolvePositionConstraints(BodyConstraintsMap& bodies, const ConstraintSolverConf& conf) const
{
    auto& bodyConstraintA = At(bodies, GetBodyA());
    auto& bodyConstraintB = At(bodies, GetBodyB());

    auto posA = bodyConstraintA->GetPosition();
    auto posB = bodyConstraintB->GetPosition();

    const auto qA = UnitVec2::Get(posA.angular);
    const auto qB = UnitVec2::Get(posB.angular);

    const auto invMassA = bodyConstraintA->GetInvMass();
    const auto invRotInertiaA = bodyConstraintA->GetInvRotInertia();
    const auto invMassB = bodyConstraintB->GetInvMass();
    const auto invRotInertiaB = bodyConstraintB->GetInvRotInertia();

    const auto rA = Rotate(m_localAnchorA - bodyConstraintA->GetLocalCenter(), qA);
    const auto rB = Rotate(m_localAnchorB - bodyConstraintB->GetLocalCenter(), qB);

    auto positionError = Length{0};
    auto angularError = Angle{0};

    const auto exx = InvMass{
        invMassA + Square(rA.y) * invRotInertiaA / SquareRadian +
        invMassB + Square(rB.y) * invRotInertiaB / SquareRadian
    };
    const auto eyx = InvMass{
        -rA.y * rA.x * invRotInertiaA / SquareRadian +
        -rB.y * rB.x * invRotInertiaB / SquareRadian
    };
    const auto ezx = InvMass{
        -rA.y * invRotInertiaA * Meter / SquareRadian +
        -rB.y * invRotInertiaB * Meter / SquareRadian
    };
    const auto eyy = InvMass{
        invMassA + Square(rA.x) * invRotInertiaA / SquareRadian +
        invMassB + Square(rB.x) * invRotInertiaB / SquareRadian
    };
    const auto ezy = InvMass{
        rA.x * invRotInertiaA * Meter / SquareRadian +
        rB.x * invRotInertiaB * Meter / SquareRadian
    };
    const auto ezz = InvMass{(invRotInertiaA + invRotInertiaB) * SquareMeter / SquareRadian};

    Mat33 K;
    K.ex.x = StripUnit(exx);
    K.ey.x = StripUnit(eyx);
    K.ez.x = StripUnit(ezx);
    K.ex.y = K.ey.x;
    K.ey.y = StripUnit(eyy);
    K.ez.y = StripUnit(ezy);
    K.ex.z = K.ez.x;
    K.ey.z = K.ez.y;
    K.ez.z = StripUnit(ezz);

    if (m_frequency > Frequency{0})
    {
        const auto C1 = Length2D{(posB.linear + rB) - (posA.linear + rA)};

        positionError = GetLength(C1);
        angularError = Angle{0};

        const auto P = -Solve22(K, C1) * (Real(1) * Kilogram);
        const auto LA = Cross(rA, P) / Radian;
        const auto LB = Cross(rB, P) / Radian;

        posA -= Position{invMassA * P, invRotInertiaA * LA};
        posB += Position{invMassB * P, invRotInertiaB * LB};
    }
    else
    {
        const auto C1 = Length2D{(posB.linear + rB) - (posA.linear + rA)};
        const auto C2 = Angle{posB.angular - posA.angular - m_referenceAngle};

        positionError = GetLength(C1);
        angularError = Abs(C2);

        const auto C = Vec3{StripUnit(C1.x), StripUnit(C1.y), StripUnit(C2)};
    
        Vec3 impulse;
        if (K.ez.z > 0)
        {
            impulse = -Solve33(K, C);
        }
        else
        {
            const auto impulse2 = -Solve22(K, GetVec2(C1));
            impulse = Vec3{impulse2.x, impulse2.y, 0};
        }

        const auto P = Length2D{impulse.x * Meter, impulse.y * Meter} * (Real(1) * Kilogram);
        const auto L = impulse.z * Kilogram * SquareMeter / Radian;
        const auto LA = L + Cross(rA, P) / Radian;
        const auto LB = L + Cross(rB, P) / Radian;

        posA -= Position{invMassA * P, invRotInertiaA * LA};
        posB += Position{invMassB * P, invRotInertiaB * LB};
    }

    bodyConstraintA->SetPosition(posA);
    bodyConstraintB->SetPosition(posB);

    return (positionError <= conf.linearSlop) && (angularError <= conf.angularSlop);
}

Length2D WeldJoint::GetAnchorA() const
{
    return GetWorldPoint(*GetBodyA(), GetLocalAnchorA());
}

Length2D WeldJoint::GetAnchorB() const
{
    return GetWorldPoint(*GetBodyB(), GetLocalAnchorB());
}

Force2D WeldJoint::GetReactionForce(Frequency inv_dt) const
{
    const auto P = Momentum2D{
        m_impulse.x * Kilogram * MeterPerSecond,
        m_impulse.y * Kilogram * MeterPerSecond
    };
    return inv_dt * P;
}

Torque WeldJoint::GetReactionTorque(Frequency inv_dt) const
{
    // AngularMomentum is L^2 M T^-1 QP^-1
    const auto angMomentum = AngularMomentum{m_impulse.z * SquareMeter * Kilogram / (Second * Radian)};
    return inv_dt * angMomentum;
}
