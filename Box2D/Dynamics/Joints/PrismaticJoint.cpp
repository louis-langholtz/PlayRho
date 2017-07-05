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

#include <Box2D/Dynamics/Joints/PrismaticJoint.hpp>
#include <Box2D/Dynamics/Body.hpp>
#include <Box2D/Dynamics/StepConf.hpp>
#include <Box2D/Dynamics/Contacts/ContactSolver.hpp>
#include <Box2D/Dynamics/Contacts/BodyConstraint.hpp>

using namespace box2d;

// Linear constraint (point-to-line)
// d = p2 - p1 = x2 + r2 - x1 - r1
// C = dot(perp, d)
// Cdot = dot(d, cross(w1, perp)) + dot(perp, v2 + cross(w2, r2) - v1 - cross(w1, r1))
//      = -dot(perp, v1) - dot(cross(d + r1, perp), w1) + dot(perp, v2) + dot(cross(r2, perp), v2)
// J = [-perp, -cross(d + r1, perp), perp, cross(r2,perp)]
//
// Angular constraint
// C = a2 - a1 + a_initial
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
//
// K = J * invM * JT
//
// J = [-a -s1 a s2]
//     [0  -1  0  1]
// a = perp
// s1 = cross(d + r1, a) = cross(p2 - x1, a)
// s2 = cross(r2, a) = cross(p2 - x2, a)


// Motor/Limit linear constraint
// C = dot(ax1, d)
// Cdot = = -dot(ax1, v1) - dot(cross(d + r1, ax1), w1) + dot(ax1, v2) + dot(cross(r2, ax1), v2)
// J = [-ax1 -cross(d+r1,ax1) ax1 cross(r2,ax1)]

// Block Solver
// We develop a block solver that includes the joint limit. This makes the limit stiff (inelastic) even
// when the mass has poor distribution (leading to large torques about the joint anchor points).
//
// The Jacobian has 3 rows:
// J = [-uT -s1 uT s2] // linear
//     [0   -1   0  1] // angular
//     [-vT -a1 vT a2] // limit
//
// u = perp
// v = axis
// s1 = cross(d + r1, u), s2 = cross(r2, u)
// a1 = cross(d + r1, v), a2 = cross(r2, v)

// M * (v2 - v1) = JT * df
// J * v2 = bias
//
// v2 = v1 + invM * JT * df
// J * (v1 + invM * JT * df) = bias
// K * df = bias - J * v1 = -Cdot
// K = J * invM * JT
// Cdot = J * v1 - bias
//
// Now solve for f2.
// df = f2 - f1
// K * (f2 - f1) = -Cdot
// f2 = invK * (-Cdot) + f1
//
// Clamp accumulated limit impulse.
// lower: f2(3) = max(f2(3), 0)
// upper: f2(3) = min(f2(3), 0)
//
// Solve for correct f2(1:2)
// K(1:2, 1:2) * f2(1:2) = -Cdot(1:2) - K(1:2,3) * f2(3) + K(1:2,1:3) * f1
//                       = -Cdot(1:2) - K(1:2,3) * f2(3) + K(1:2,1:2) * f1(1:2) + K(1:2,3) * f1(3)
// K(1:2, 1:2) * f2(1:2) = -Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3)) + K(1:2,1:2) * f1(1:2)
// f2(1:2) = invK(1:2,1:2) * (-Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3))) + f1(1:2)
//
// Now compute impulse to be applied:
// df = f2 - f1

PrismaticJointDef::PrismaticJointDef(Body* bA, Body* bB, const Length2D anchor, const UnitVec2 axis) noexcept:
    JointDef{JointType::Prismatic, bA, bB},
    localAnchorA{GetLocalPoint(*bA, anchor)},
    localAnchorB{GetLocalPoint(*bB, anchor)},
    localAxisA{GetLocalVector(*bA, axis)},
    referenceAngle{bB->GetAngle() - bA->GetAngle()}    
{
    // Intentionally empty.
}

PrismaticJoint::PrismaticJoint(const PrismaticJointDef& def):
    Joint(def),
    m_localAnchorA{def.localAnchorA},
    m_localAnchorB{def.localAnchorB},
    m_localXAxisA{def.localAxisA},
    m_localYAxisA{GetRevPerpendicular(m_localXAxisA)},
    m_referenceAngle{def.referenceAngle},
    m_lowerTranslation{def.lowerTranslation},
    m_upperTranslation{def.upperTranslation},
    m_maxMotorForce{Force{StripUnit(def.maxMotorTorque) * Newton}}, // TODO check this!!
    m_motorSpeed{def.motorSpeed},
    m_enableLimit{def.enableLimit},
    m_enableMotor{def.enableMotor}
{
    // Intentionally empty.
}

void PrismaticJoint::InitVelocityConstraints(BodyConstraintsMap& bodies,
                                             const StepConf& step,
                                             const ConstraintSolverConf& conf)
{
    auto& bodyConstraintA = bodies.at(GetBodyA());
    auto& bodyConstraintB = bodies.at(GetBodyB());

    const auto posA = bodyConstraintA->GetPosition();
    auto velA = bodyConstraintA->GetVelocity();

    const auto posB = bodyConstraintB->GetPosition();
    auto velB = bodyConstraintB->GetVelocity();

    const auto qA = UnitVec2(posA.angular);
    const auto qB = UnitVec2(posB.angular);

    // Compute the effective masses.
    const auto rA = Rotate(m_localAnchorA - bodyConstraintA->GetLocalCenter(), qA); // Length2D
    const auto rB = Rotate(m_localAnchorB - bodyConstraintB->GetLocalCenter(), qB); // Length2D
    const auto d = (posB.linear - posA.linear) + rB - rA; // Length2D

    const auto invMassA = bodyConstraintA->GetInvMass();
    const auto invMassB = bodyConstraintB->GetInvMass();
    const auto invRotInertiaA = bodyConstraintA->GetInvRotInertia();
    const auto invRotInertiaB = bodyConstraintB->GetInvRotInertia();

    // Compute motor Jacobian and effective mass.
    m_axis = Rotate(m_localXAxisA, qA);
    m_a1 = Cross(d + rA, m_axis); // Length
    m_a2 = Cross(rB, m_axis); // Length

    const auto invRotMassA = InvMass{invRotInertiaA * m_a1 * m_a1 / SquareRadian};
    const auto invRotMassB = InvMass{invRotInertiaB * m_a2 * m_a2 / SquareRadian};
    const auto totalInvMass = invMassA + invMassB + invRotMassA + invRotMassB;
    m_motorMass = (totalInvMass > InvMass{0})? RealNum{1} / totalInvMass: Mass{0};

    // Prismatic constraint.
    {
        m_perp = Rotate(m_localYAxisA, qA);

        m_s1 = Cross(d + rA, m_perp);
        m_s2 = Cross(rB, m_perp);

        const auto invRotMassA2 = InvMass{invRotInertiaA * m_s1 * m_s1 / SquareRadian};
        const auto invRotMassB2 = InvMass{invRotInertiaB * m_s2 * m_s2 / SquareRadian};
        const auto k11 = StripUnit(invMassA + invMassB + invRotMassA2 + invRotMassB2);
        
        // L^-2 M^-1 QP^2 * L is: L^-1 M^-1 QP^2.
        const auto k12 = (invRotInertiaA * m_s1 + invRotInertiaB * m_s2) * Meter * Kilogram / SquareRadian;
        const auto k13 = StripUnit(InvMass{(invRotInertiaA * m_s1 * m_a1 + invRotInertiaB * m_s2 * m_a2) / SquareRadian});
        const auto totalInvRotInertia = invRotInertiaA + invRotInertiaB;
        
        const auto k22 = (totalInvRotInertia == InvRotInertia{0})? RealNum{1}: StripUnit(totalInvRotInertia);
        const auto k23 = (invRotInertiaA * m_a1 + invRotInertiaB * m_a2) * Meter * Kilogram / SquareRadian;
        const auto k33 = StripUnit(totalInvMass);

        m_K.ex = Vec3{k11, k12, k13};
        m_K.ey = Vec3{k12, k22, k23};
        m_K.ez = Vec3{k13, k23, k33};
    }

    // Compute motor and limit terms.
    if (m_enableLimit)
    {
        const auto jointTranslation = Length{Dot(m_axis, d)};
        if (Abs(m_upperTranslation - m_lowerTranslation) < (conf.linearSlop * RealNum{2}))
        {
            m_limitState = e_equalLimits;
        }
        else if (jointTranslation <= m_lowerTranslation)
        {
            if (m_limitState != e_atLowerLimit)
            {
                m_limitState = e_atLowerLimit;
                m_impulse.z = 0;
            }
        }
        else if (jointTranslation >= m_upperTranslation)
        {
            if (m_limitState != e_atUpperLimit)
            {
                m_limitState = e_atUpperLimit;
                m_impulse.z = 0;
            }
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
        m_impulse.z = 0;
    }

    if (!m_enableMotor)
    {
        m_motorImpulse = 0;
    }

    if (step.doWarmStart)
    {
        // Account for variable time step.
        m_impulse *= step.dtRatio;
        m_motorImpulse *= step.dtRatio;

        const auto ulImpulseX = m_impulse.x * m_perp;
        const auto Px = Momentum2D{
            ulImpulseX.GetX() * Kilogram * MeterPerSecond,
            ulImpulseX.GetY() * Kilogram * MeterPerSecond
        };
        const auto Pxs1 = Momentum{m_impulse.x * m_s1 * Kilogram / Second};
        const auto Pxs2 = Momentum{m_impulse.x * m_s2 * Kilogram / Second};
        const auto PzLength = Momentum{m_motorImpulse + m_impulse.z * Kilogram * MeterPerSecond};
        const auto Pz = Momentum2D{PzLength * m_axis};
        const auto P = Px + Pz;
        
        // AngularMomentum is L^2 M T^-1 QP^-1.
        const auto L = AngularMomentum{m_impulse.y * SquareMeter * Kilogram / (Second * Radian)};
        const auto LA = L + (Pxs1 * Meter + PzLength * m_a1) / Radian;
        const auto LB = L + (Pxs2 * Meter + PzLength * m_a2) / Radian;

        // InvRotInertia is L^-2 M^-1 QP^2
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

bool PrismaticJoint::SolveVelocityConstraints(BodyConstraintsMap& bodies, const StepConf& step)
{
    auto& bodyConstraintA = bodies.at(GetBodyA());
    auto& bodyConstraintB = bodies.at(GetBodyB());

    const auto oldVelA = bodyConstraintA->GetVelocity();
    auto velA = oldVelA;
    const auto invMassA = bodyConstraintA->GetInvMass();
    const auto invRotInertiaA = bodyConstraintA->GetInvRotInertia();

    const auto oldVelB = bodyConstraintB->GetVelocity();
    auto velB = oldVelB;
    const auto invMassB = bodyConstraintB->GetInvMass();
    const auto invRotInertiaB = bodyConstraintB->GetInvRotInertia();

    // Solve linear motor constraint.
    if (m_enableMotor && m_limitState != e_equalLimits)
    {
        const auto vDot = LinearVelocity{Dot(m_axis, velB.linear - velA.linear)};
        const auto Cdot = vDot + (m_a2 * velB.angular - m_a1 * velA.angular) / Radian;
        auto impulse = Momentum{m_motorMass * (m_motorSpeed * Meter / Radian - Cdot)};
        const auto oldImpulse = m_motorImpulse;
        const auto maxImpulse = step.GetTime() * m_maxMotorForce;
        m_motorImpulse = Clamp(m_motorImpulse + impulse, -maxImpulse, maxImpulse);
        impulse = m_motorImpulse - oldImpulse;

        const auto P = Momentum2D{impulse * m_axis};
        
        // Momentum is L^2 M T^-1. AngularMomentum is L^2 M T^-1 QP^-1.
        const auto LA = impulse * m_a1 / Radian;
        const auto LB = impulse * m_a2 / Radian;

        velA -= Velocity{invMassA * P, invRotInertiaA * LA};
        velB += Velocity{invMassB * P, invRotInertiaB * LB};
    }

    const auto velDelta = velB.linear - velA.linear;
    const auto sRotSpeed = LinearVelocity{(m_s2 * velB.angular - m_s1 * velA.angular) / Radian};
    const auto Cdot1 = Vec2{
        StripUnit(Dot(m_perp, velDelta) + sRotSpeed),
        StripUnit(velB.angular - velA.angular)
    };

    if (m_enableLimit && (m_limitState != e_inactiveLimit))
    {
        // Solve prismatic and limit constraint in block form.
        const auto deltaDot = LinearVelocity{Dot(m_axis, velDelta)};
        const auto aRotSpeed = LinearVelocity{(m_a2 * velB.angular - m_a1 * velA.angular) / Radian};
        const auto Cdot2 = StripUnit(deltaDot + aRotSpeed);
        const auto Cdot = Vec3{Cdot1.x, Cdot1.y, Cdot2};

        const auto f1 = m_impulse;
        m_impulse += Solve33(m_K, -Cdot);

        if (m_limitState == e_atLowerLimit)
        {
            m_impulse.z = Max(m_impulse.z, RealNum{0});
        }
        else if (m_limitState == e_atUpperLimit)
        {
            m_impulse.z = Min(m_impulse.z, RealNum{0});
        }

        // f2(1:2) = invK(1:2,1:2) * (-Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3))) + f1(1:2)
        const auto b = -Cdot1 - (m_impulse.z - f1.z) * Vec2{m_K.ez.x, m_K.ez.y};
        const auto f2r = Solve22(m_K, b) + Vec2{f1.x, f1.y};
        m_impulse.x = f2r.x;
        m_impulse.y = f2r.y;

        const auto df = m_impulse - f1;

        const auto ulP = df.x * m_perp + df.z * m_axis;
        const auto P = Momentum2D{
            ulP.GetX() * Kilogram * MeterPerSecond,
            ulP.GetY() * Kilogram * MeterPerSecond
        };
        const auto LA = AngularMomentum{
            (df.x * m_s1 + df.y * Meter + df.z * m_a1) * Kilogram * MeterPerSecond / Radian
        };
        const auto LB = AngularMomentum{
            (df.x * m_s2 + df.y * Meter + df.z * m_a2) * Kilogram * MeterPerSecond / Radian
        };

        velA -= Velocity{invMassA * P, invRotInertiaA * LA};
        velB += Velocity{invMassB * P, invRotInertiaB * LB};
    }
    else
    {
        // Limit is inactive, just solve the prismatic constraint in block form.
        const auto df = Solve22(m_K, -Cdot1);
        m_impulse.x += df.x;
        m_impulse.y += df.y;

        const auto ulP = df.x * m_perp;
        const auto P = Momentum2D{
            ulP.GetX() * Kilogram * MeterPerSecond,
            ulP.GetY() * Kilogram * MeterPerSecond
        };
        const auto LA = AngularMomentum{
            (df.x * m_s1 + df.y * Meter) * Kilogram * MeterPerSecond / Radian
        };
        const auto LB = AngularMomentum{
            (df.x * m_s2 + df.y * Meter) * Kilogram * MeterPerSecond / Radian
        };

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

// A velocity based solver computes reaction forces(impulses) using the velocity constraint solver.Under this context,
// the position solver is not there to resolve forces.It is only there to cope with integration error.
//
// Therefore, the pseudo impulses in the position solver do not have any physical meaning.Thus it is okay if they suck.
//
// We could take the active state from the velocity solver.However, the joint might push past the limit when the velocity
// solver indicates the limit is inactive.
bool PrismaticJoint::SolvePositionConstraints(BodyConstraintsMap& bodies, const ConstraintSolverConf& conf) const
{
    auto& bodyConstraintA = bodies.at(GetBodyA());
    auto& bodyConstraintB = bodies.at(GetBodyB());

    auto posA = bodyConstraintA->GetPosition();
    const auto invMassA = bodyConstraintA->GetInvMass();
    const auto invRotInertiaA = bodyConstraintA->GetInvRotInertia();

    auto posB = bodyConstraintB->GetPosition();
    const auto invMassB = bodyConstraintB->GetInvMass();
    const auto invRotInertiaB = bodyConstraintB->GetInvRotInertia();

    const auto qA = UnitVec2{posA.angular};
    const auto qB = UnitVec2{posB.angular};

    // Compute fresh Jacobians
    const auto rA = Rotate(m_localAnchorA - bodyConstraintA->GetLocalCenter(), qA);
    const auto rB = Rotate(m_localAnchorB - bodyConstraintB->GetLocalCenter(), qB);
    const auto d = Length2D{(posB.linear + rB) - (posA.linear + rA)};

    const auto axis = Rotate(m_localXAxisA, qA);
    const auto a1 = Length{Cross(d + rA, axis)};
    const auto a2 = Length{Cross(rB, axis)};
    const auto perp = Rotate(m_localYAxisA, qA);

    const auto s1 = Length{Cross(d + rA, perp)};
    const auto s2 = Length{Cross(rB, perp)};

    const auto C1 = Vec2{
        Dot(perp, d) / Meter,
        (posB.angular - posA.angular - m_referenceAngle) / Radian
    };

    auto linearError = Length{Abs(C1.x) * Meter};
    const auto angularError = Angle{Abs(C1.y) * Radian};

    auto active = false;
    auto C2 = RealNum{0};
    if (m_enableLimit)
    {
        const auto translation = Length{Dot(axis, d)};
        if (Abs(m_upperTranslation - m_lowerTranslation) < (RealNum{2} * conf.linearSlop))
        {
            // Prevent large angular corrections
            C2 = StripUnit(Clamp(translation, -conf.maxLinearCorrection, conf.maxLinearCorrection));
            linearError = Max(linearError, Abs(translation));
            active = true;
        }
        else if (translation <= m_lowerTranslation)
        {
            // Prevent large linear corrections and allow some slop.
            C2 = StripUnit(Clamp(translation - m_lowerTranslation + conf.linearSlop, -conf.maxLinearCorrection, Length{0}));
            linearError = Max(linearError, m_lowerTranslation - translation);
            active = true;
        }
        else if (translation >= m_upperTranslation)
        {
            // Prevent large linear corrections and allow some slop.
            C2 = StripUnit(Clamp(translation - m_upperTranslation - conf.linearSlop, Length{0}, conf.maxLinearCorrection));
            linearError = Max(linearError, translation - m_upperTranslation);
            active = true;
        }
    }

    Vec3 impulse;
    if (active)
    {
        const auto k11 = StripUnit(InvMass{
            invMassA + invRotInertiaA * s1 * s1 / SquareRadian +
            invMassB + invRotInertiaB * s2 * s2 / SquareRadian
        });
        const auto k12 = StripUnit(InvMass{
            invRotInertiaA * s1 * Meter / SquareRadian +
            invRotInertiaB * s2 * Meter / SquareRadian
        });
        const auto k13 = StripUnit(InvMass{
            invRotInertiaA * s1 * a1 / SquareRadian +
            invRotInertiaB * s2 * a2 / SquareRadian
        });
    
        // InvRotInertia is L^-2 M^-1 QP^2
        auto k22 = StripUnit(invRotInertiaA + invRotInertiaB);
        if (k22 == RealNum{0})
        {
            // For fixed rotation
            k22 = StripUnit(RealNum{1} * SquareRadian / (Kilogram * SquareMeter));
        }
        const auto k23 = StripUnit(InvMass{
            invRotInertiaA * a1 * Meter / SquareRadian +
            invRotInertiaB * a2 * Meter / SquareRadian
        });
        const auto k33 = StripUnit(InvMass{
            invMassA + invRotInertiaA * Square(a1) / SquareRadian +
            invMassB + invRotInertiaB * Square(a2) / SquareRadian
        });

        const auto K = Mat33{Vec3{k11, k12, k13}, Vec3{k12, k22, k23}, Vec3{k13, k23, k33}};
        const auto C = Vec3{C1.x, C1.y, C2};

        impulse = Solve33(K, -C);
    }
    else
    {
        const auto k11 = StripUnit(InvMass{
            invMassA + invRotInertiaA * s1 * s1 / SquareRadian +
            invMassB + invRotInertiaB * s2 * s2 / SquareRadian
        });
        const auto k12 = StripUnit(InvMass{
            invRotInertiaA * s1 * Meter / SquareRadian +
            invRotInertiaB * s2 * Meter / SquareRadian
        });
        auto k22 = StripUnit(invRotInertiaA + invRotInertiaB);
        if (k22 == 0)
        {
            k22 = 1;
        }

        const auto K = Mat22{Vec2{k11, k12}, Vec2{k12, k22}};

        const auto impulse1 = Solve(K, -C1);
        impulse.x = impulse1.x;
        impulse.y = impulse1.y;
        impulse.z = 0;
    }

    const auto P = (impulse.x * perp + impulse.z * axis) * (RealNum(1) * Kilogram * Meter);
    const auto LA = (impulse.x * s1 + impulse.y * Meter + impulse.z * a1) * Kilogram * Meter / Radian;
    const auto LB = (impulse.x * s2 + impulse.y * Meter + impulse.z * a2) * Kilogram * Meter / Radian;

    posA -= Position{Length2D{invMassA * P}, invRotInertiaA * LA};
    posB += Position{invMassB * P, invRotInertiaB * LB};

    bodyConstraintA->SetPosition(posA);
    bodyConstraintB->SetPosition(posB);

    return (linearError <= conf.linearSlop) && (angularError <= conf.angularSlop);
}

Length2D PrismaticJoint::GetAnchorA() const
{
    return GetWorldPoint(*GetBodyA(), GetLocalAnchorA());
}

Length2D PrismaticJoint::GetAnchorB() const
{
    return GetWorldPoint(*GetBodyB(), GetLocalAnchorB());
}

Force2D PrismaticJoint::GetReactionForce(Frequency inv_dt) const
{
    const auto ulImpulse = m_impulse.x * m_perp;
    const auto impulse = Momentum2D{
        ulImpulse.GetX() * Kilogram * MeterPerSecond,
        ulImpulse.GetY() * Kilogram * MeterPerSecond
    };
    const auto P = Momentum2D{
        impulse + (m_motorImpulse + m_impulse.z * Kilogram * MeterPerSecond) * m_axis
    };
    return inv_dt * P;
}

Torque PrismaticJoint::GetReactionTorque(Frequency inv_dt) const
{
    // Torque is L^2 M T^-2 QP^-1.
    return inv_dt * m_impulse.y * SquareMeter * Kilogram / (Second * Radian);
}

Length PrismaticJoint::GetJointTranslation() const
{
    const auto pA = GetWorldPoint(*GetBodyA(), m_localAnchorA);
    const auto pB = GetWorldPoint(*GetBodyB(), m_localAnchorB);
    return Dot(pB - pA, GetWorldVector(*GetBodyA(), m_localXAxisA));
}

LinearVelocity PrismaticJoint::GetJointSpeed() const
{
    const auto bA = GetBodyA();
    const auto bB = GetBodyB();

    const auto rA = Rotate(m_localAnchorA - bA->GetLocalCenter(), bA->GetTransformation().q);
    const auto rB = Rotate(m_localAnchorB - bB->GetLocalCenter(), bB->GetTransformation().q);
    const auto p1 = bA->GetWorldCenter() + rA;
    const auto p2 = bB->GetWorldCenter() + rB;
    const auto d = p2 - p1;
    const auto axis = Rotate(m_localXAxisA, bA->GetTransformation().q);

    const auto vA = bA->GetVelocity().linear;
    const auto vB = bB->GetVelocity().linear;
    const auto wA = bA->GetVelocity().angular;
    const auto wB = bB->GetVelocity().angular;

    const auto vel = vB + (GetRevPerpendicular(rB) * (wB / Radian)) - (vA + (GetRevPerpendicular(rA) * (wA / Radian)));
    return Dot(d, (GetRevPerpendicular(axis) * (wA / Radian))) + Dot(axis, vel);
}

bool PrismaticJoint::IsLimitEnabled() const noexcept
{
    return m_enableLimit;
}

void PrismaticJoint::EnableLimit(bool flag) noexcept
{
    if (m_enableLimit != flag)
    {
        m_enableLimit = flag;
        m_impulse.z = 0;

        GetBodyA()->SetAwake();
        GetBodyB()->SetAwake();
    }
}

Length PrismaticJoint::GetLowerLimit() const noexcept
{
    return m_lowerTranslation;
}

Length PrismaticJoint::GetUpperLimit() const noexcept
{
    return m_upperTranslation;
}

void PrismaticJoint::SetLimits(Length lower, Length upper)
{
    assert(lower <= upper);
    if ((lower != m_lowerTranslation) || (upper != m_upperTranslation))
    {
        m_lowerTranslation = lower;
        m_upperTranslation = upper;
        m_impulse.z = 0;
        
        GetBodyA()->SetAwake();
        GetBodyB()->SetAwake();
    }
}

bool PrismaticJoint::IsMotorEnabled() const noexcept
{
    return m_enableMotor;
}

void PrismaticJoint::EnableMotor(bool flag) noexcept
{
    if (m_enableMotor != flag)
    {
        m_enableMotor = flag;

        // XXX Should these be called regardless of whether the state changed?
        GetBodyA()->SetAwake();
        GetBodyB()->SetAwake();
    }
}

void PrismaticJoint::SetMotorSpeed(AngularVelocity speed) noexcept
{
    if (m_motorSpeed != speed)
    {
        m_motorSpeed = speed;

        // XXX Should these be called regardless of whether the state changed?
	    GetBodyA()->SetAwake();
    	GetBodyB()->SetAwake();
    }
}

void PrismaticJoint::SetMaxMotorForce(Force force) noexcept
{
    if (m_maxMotorForce != force)
    {
        m_maxMotorForce = force;

        // XXX Should these be called regardless of whether the state changed?
        GetBodyA()->SetAwake();
        GetBodyB()->SetAwake();
    }
}

Force PrismaticJoint::GetMotorForce(Frequency inv_dt) const noexcept
{
    return inv_dt * m_motorImpulse;
}

PrismaticJointDef box2d::GetPrismaticJointDef(const PrismaticJoint& joint) noexcept
{
    auto def = PrismaticJointDef{};
    
    Set(def, joint);
    
    def.localAnchorA = joint.GetLocalAnchorA();
    def.localAnchorB = joint.GetLocalAnchorB();
    def.localAxisA = joint.GetLocalAxisA();
    def.referenceAngle = joint.GetReferenceAngle();
    def.enableLimit = joint.IsLimitEnabled();
    def.lowerTranslation = joint.GetLowerLimit();
    def.upperTranslation = joint.GetUpperLimit();
    def.enableMotor = joint.IsMotorEnabled();
    def.motorSpeed = joint.GetMotorSpeed();
    def.maxMotorTorque = Torque{StripUnit(joint.GetMaxMotorForce()) * NewtonMeter};
    
    return def;
}
