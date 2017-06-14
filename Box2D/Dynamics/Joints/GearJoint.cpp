/*
* Original work Copyright (c) 2007-2011 Erin Catto http://www.box2d.org
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

#include <Box2D/Dynamics/Joints/GearJoint.hpp>
#include <Box2D/Dynamics/Joints/RevoluteJoint.hpp>
#include <Box2D/Dynamics/Joints/PrismaticJoint.hpp>
#include <Box2D/Dynamics/Body.hpp>
#include <Box2D/Dynamics/StepConf.hpp>
#include <Box2D/Dynamics/Contacts/ContactSolver.hpp>
#include <Box2D/Dynamics/Contacts/BodyConstraint.hpp>

using namespace box2d;

// Gear Joint:
// C0 = (coordinate1 + ratio * coordinate2)_initial
// C = (coordinate1 + ratio * coordinate2) - C0 = 0
// J = [J1 ratio * J2]
// K = J * invM * JT
//   = J1 * invM1 * J1T + ratio * ratio * J2 * invM2 * J2T
//
// Revolute:
// coordinate = rotation
// Cdot = angularVelocity
// J = [0 0 1]
// K = J * invM * JT = invI
//
// Prismatic:
// coordinate = dot(p - pg, ug)
// Cdot = dot(v + cross(w, r), ug)
// J = [ug cross(r, ug)]
// K = J * invM * JT = invMass + invI * cross(r, ug)^2

GearJoint::GearJoint(const GearJointDef& def):
    Joint(JointDef(def).UseBodyA(def.joint1->GetBodyB()).UseBodyB(def.joint2->GetBodyB())),
    m_joint1(def.joint1),
    m_joint2(def.joint2),
    m_typeA(def.joint1->GetType()),
    m_typeB(def.joint2->GetType()),
    m_ratio(def.ratio)
{
    assert(m_typeA == JointType::Revolute || m_typeA == JointType::Prismatic);
    assert(m_typeB == JointType::Revolute || m_typeB == JointType::Prismatic);

    // TODO_ERIN there might be some problem with the joint edges in Joint.

    m_bodyC = m_joint1->GetBodyA();

    // Get geometry of joint1
    const auto xfA = GetBodyA()->GetTransformation();
    const auto aA = GetBodyA()->GetAngle();
    const auto xfC = m_bodyC->GetTransformation();
    const auto aC = m_bodyC->GetAngle();

    RealNum coordinateA; // Duck-typed to handle m_typeA's type.
    if (m_typeA == JointType::Revolute)
    {
        const auto revolute = static_cast<const RevoluteJoint*>(def.joint1);
        m_localAnchorC = revolute->GetLocalAnchorA();
        m_localAnchorA = revolute->GetLocalAnchorB();
        m_referenceAngleA = revolute->GetReferenceAngle();
        m_localAxisC = UnitVec2::GetZero();
        coordinateA = (aA - aC - m_referenceAngleA) / Radian;
    }
    else // if (m_typeA != JointType::Revolute)
    {
        const auto prismatic = static_cast<const PrismaticJoint*>(def.joint1);
        m_localAnchorC = prismatic->GetLocalAnchorA();
        m_localAnchorA = prismatic->GetLocalAnchorB();
        m_referenceAngleA = prismatic->GetReferenceAngle();
        m_localAxisC = prismatic->GetLocalAxisA();

        const auto pC = m_localAnchorC;
        const auto pA = InverseRotate(Rotate(m_localAnchorA, xfA.q) + (xfA.p - xfC.p), xfC.q);
        coordinateA = Dot(pA - pC, m_localAxisC) / Meter;
    }

    m_bodyD = m_joint2->GetBodyA();

    // Get geometry of joint2
    const auto xfB = GetBodyB()->GetTransformation();
    const auto aB = GetBodyB()->GetAngle();
    const auto xfD = m_bodyD->GetTransformation();
    const auto aD = m_bodyD->GetAngle();

    RealNum coordinateB; // Duck-typed to handle m_typeB's type.
    if (m_typeB == JointType::Revolute)
    {
        const auto revolute = static_cast<const RevoluteJoint*>(def.joint2);
        m_localAnchorD = revolute->GetLocalAnchorA();
        m_localAnchorB = revolute->GetLocalAnchorB();
        m_referenceAngleB = revolute->GetReferenceAngle();
        m_localAxisD = UnitVec2::GetZero();
        coordinateB = (aB - aD - m_referenceAngleB) / Radian;
    }
    else
    {
        const auto prismatic = static_cast<const PrismaticJoint*>(def.joint2);
        m_localAnchorD = prismatic->GetLocalAnchorA();
        m_localAnchorB = prismatic->GetLocalAnchorB();
        m_referenceAngleB = prismatic->GetReferenceAngle();
        m_localAxisD = prismatic->GetLocalAxisA();

        const auto pD = m_localAnchorD;
        const auto pB = InverseRotate(Rotate(m_localAnchorB, xfB.q) + (xfB.p - xfD.p), xfD.q);
        coordinateB = Dot(pB - pD, m_localAxisD) / Meter;
    }

    m_constant = coordinateA + m_ratio * coordinateB;
}

void GearJoint::InitVelocityConstraints(BodyConstraints& bodies, const StepConf& step, const ConstraintSolverConf&)
{
    auto& bodiesA = bodies.at(GetBodyA());
    auto& bodiesB = bodies.at(GetBodyB());
    auto& bodiesC = bodies.at(m_bodyC);
    auto& bodiesD = bodies.at(m_bodyD);

    auto velA = bodiesA.GetVelocity();
    const auto aA = bodiesA.GetPosition().angular;

    auto velB = bodiesB.GetVelocity();
    const auto aB = bodiesB.GetPosition().angular;

    auto velC = bodiesC.GetVelocity();
    const auto aC = bodiesC.GetPosition().angular;

    auto velD = bodiesD.GetVelocity();
    const auto aD = bodiesD.GetPosition().angular;

    const auto qA = UnitVec2(aA);
    const auto qB = UnitVec2(aB);
    const auto qC = UnitVec2(aC);
    const auto qD = UnitVec2(aD);

    auto invMass = RealNum{0}; // Unitless to double for either linear mass or angular mass.

    if (m_typeA == JointType::Revolute)
    {
        m_JvAC = Vec2_zero;
        m_JwA = RealNum{1} * Meter;
        m_JwC = RealNum{1} * Meter;
        const auto invAngMass = bodiesA.GetInvRotInertia() + bodiesC.GetInvRotInertia();
        invMass += StripUnit(invAngMass);
    }
    else
    {
        const auto u = Rotate(m_localAxisC, qC);
        const auto rC = Length2D{Rotate(m_localAnchorC - bodiesC.GetLocalCenter(), qC)};
        const auto rA = Length2D{Rotate(m_localAnchorA - bodiesA.GetLocalCenter(), qA)};
        m_JvAC = RealNum{1} * u;
        m_JwC = Cross(rC, u);
        m_JwA = Cross(rA, u);
        const auto invRotMassC = InvMass{bodiesC.GetInvRotInertia() * Square(m_JwC) / SquareRadian};
        const auto invRotMassA = InvMass{bodiesA.GetInvRotInertia() * Square(m_JwA) / SquareRadian};
        const auto invLinMass = InvMass{bodiesC.GetInvMass() + bodiesA.GetInvMass() + invRotMassC + invRotMassA};
        invMass += StripUnit(invLinMass);
    }

    if (m_typeB == JointType::Revolute)
    {
        m_JvBD = Vec2_zero;
        m_JwB = m_ratio * Meter;
        m_JwD = m_ratio * Meter;
        const auto invAngMass = InvRotInertia{Square(m_ratio) * (bodiesB.GetInvRotInertia() + bodiesD.GetInvRotInertia())};
        invMass += StripUnit(invAngMass);
    }
    else
    {
        const auto u = Rotate(m_localAxisD, qD);
        const auto rD = Rotate(m_localAnchorD - bodiesD.GetLocalCenter(), qD);
        const auto rB = Rotate(m_localAnchorB - bodiesB.GetLocalCenter(), qB);
        m_JvBD = m_ratio * u;
        m_JwD = m_ratio * Cross(rD, u);
        m_JwB = m_ratio * Cross(rB, u);
        const auto invRotMassD = InvMass{bodiesD.GetInvRotInertia() * Square(m_JwD) / SquareRadian};
        const auto invRotMassB = InvMass{bodiesB.GetInvRotInertia() * Square(m_JwB) / SquareRadian};
        const auto invLinMass = InvMass{
            Square(m_ratio) * (bodiesD.GetInvMass() + bodiesB.GetInvMass()) +
            invRotMassD + invRotMassB
        };
        invMass += StripUnit(invLinMass);
    }

    // Compute effective mass.
    m_mass = (invMass > RealNum{0})? RealNum{1} / invMass: RealNum{0};

    if (step.doWarmStart)
    {
        velA += Velocity{
            (bodiesA.GetInvMass() * m_impulse) * m_JvAC,
            bodiesA.GetInvRotInertia() * m_impulse * m_JwA / Radian
        };
        velB += Velocity{
            (bodiesB.GetInvMass() * m_impulse) * m_JvBD,
            bodiesB.GetInvRotInertia() * m_impulse * m_JwB / Radian
        };
        velC -= Velocity{
            (bodiesC.GetInvMass() * m_impulse) * m_JvAC,
            bodiesC.GetInvRotInertia() * m_impulse * m_JwC / Radian
        };
        velD -= Velocity{
            (bodiesD.GetInvMass() * m_impulse) * m_JvBD,
            bodiesD.GetInvRotInertia() * m_impulse * m_JwD / Radian
        };
    }
    else
    {
        m_impulse = Momentum{0};
    }

    bodiesA.SetVelocity(velA);
    bodiesB.SetVelocity(velB);
    bodiesC.SetVelocity(velC);
    bodiesD.SetVelocity(velD);
}

bool GearJoint::SolveVelocityConstraints(BodyConstraints& bodies, const StepConf&)
{
    auto& bodiesA = bodies.at(GetBodyA());
    auto& bodiesB = bodies.at(GetBodyB());
    auto& bodiesC = bodies.at(m_bodyC);
    auto& bodiesD = bodies.at(m_bodyD);

    auto velA = bodiesA.GetVelocity();
    auto velB = bodiesB.GetVelocity();
    auto velC = bodiesC.GetVelocity();
    auto velD = bodiesD.GetVelocity();

    const auto acDot = LinearVelocity{Dot(m_JvAC, velA.linear - velC.linear)};
    const auto bdDot = LinearVelocity{Dot(m_JvBD, velB.linear - velD.linear)};
    const auto Cdot = acDot + bdDot
        + (m_JwA * velA.angular - m_JwC * velC.angular) / Radian
        + (m_JwB * velB.angular - m_JwD * velD.angular) / Radian;

    const auto impulse = Momentum{-m_mass * Kilogram * Cdot};
    m_impulse += impulse;

    velA += Velocity{
        (bodiesA.GetInvMass() * impulse) * m_JvAC,
        bodiesA.GetInvRotInertia() * impulse * m_JwA / Radian
    };
    velB += Velocity{
        (bodiesB.GetInvMass() * impulse) * m_JvBD,
        bodiesB.GetInvRotInertia() * impulse * m_JwB / Radian
    };
    velC -= Velocity{
        (bodiesC.GetInvMass() * impulse) * m_JvAC,
        bodiesC.GetInvRotInertia() * impulse * m_JwC / Radian
    };
    velD -= Velocity{
        (bodiesD.GetInvMass() * impulse) * m_JvBD,
        bodiesD.GetInvRotInertia() * impulse * m_JwD / Radian
    };

    bodiesA.SetVelocity(velA);
    bodiesB.SetVelocity(velB);
    bodiesC.SetVelocity(velC);
    bodiesD.SetVelocity(velD);
    
    return impulse == Momentum(0);
}

bool GearJoint::SolvePositionConstraints(BodyConstraints& bodies, const ConstraintSolverConf& conf) const
{
    auto& bodiesA = bodies.at(GetBodyA());
    auto& bodiesB = bodies.at(GetBodyB());
    auto& bodiesC = bodies.at(m_bodyC);
    auto& bodiesD = bodies.at(m_bodyD);

    auto posA = bodiesA.GetPosition();
    auto posB = bodiesB.GetPosition();
    auto posC = bodiesC.GetPosition();
    auto posD = bodiesD.GetPosition();

    const UnitVec2 qA(posA.angular), qB(posB.angular), qC(posC.angular), qD(posD.angular);

    const auto linearError = Length{0};


    Vec2 JvAC, JvBD;
    RealNum JwA, JwB, JwC, JwD;

    auto coordinateA = RealNum{0}; // Angle or length.
    auto coordinateB = RealNum{0};
    auto invMass = RealNum{0}; // Inverse linear mass or inverse angular mass.

    if (m_typeA == JointType::Revolute)
    {
        JvAC = Vec2_zero;
        JwA = 1;
        JwC = 1;
        const auto invAngMass = bodiesA.GetInvRotInertia() + bodiesC.GetInvRotInertia();
        invMass += StripUnit(invAngMass);
        coordinateA = (posA.angular - posC.angular - m_referenceAngleA) / Radian;
    }
    else
    {
        const auto u = Rotate(m_localAxisC, qC);
        const auto rC = Rotate(m_localAnchorC - bodiesC.GetLocalCenter(), qC);
        const auto rA = Rotate(m_localAnchorA - bodiesA.GetLocalCenter(), qA);
        JvAC = u * RealNum{1};
        JwC = StripUnit(Length{Cross(rC, u)});
        JwA = StripUnit(Length{Cross(rA, u)});
        const auto invLinMass = InvMass{bodiesC.GetInvMass() + bodiesA.GetInvMass()};
        const auto invRotMassC = InvMass{bodiesC.GetInvRotInertia() * Square(JwC * Meter / Radian)};
        const auto invRotMassA = InvMass{bodiesA.GetInvRotInertia() * Square(JwA * Meter / Radian)};
        invMass += StripUnit(invLinMass + invRotMassC + invRotMassA);
        const auto pC = m_localAnchorC - bodiesC.GetLocalCenter();
        const auto pA = InverseRotate(rA + (posA.linear - posC.linear), qC);
        coordinateA = Dot(pA - pC, m_localAxisC) / Meter;
    }

    if (m_typeB == JointType::Revolute)
    {
        JvBD = Vec2_zero;
        JwB = m_ratio;
        JwD = m_ratio;
        const auto invAngMass = InvRotInertia{
            Square(m_ratio) * (bodiesB.GetInvRotInertia() + bodiesD.GetInvRotInertia())
        };
        invMass += StripUnit(invAngMass);
        coordinateB = (posB.angular - posD.angular - m_referenceAngleB) / Radian;
    }
    else
    {
        const auto u = Rotate(m_localAxisD, qD);
        const auto rD = Rotate(m_localAnchorD - bodiesD.GetLocalCenter(), qD);
        const auto rB = Rotate(m_localAnchorB - bodiesB.GetLocalCenter(), qB);
        JvBD = m_ratio * u;
        JwD = m_ratio * StripUnit(Length{Cross(rD, u)});
        JwB = m_ratio * StripUnit(Length{Cross(rB, u)});
        const auto invLinMass = InvMass{Square(m_ratio) * (bodiesD.GetInvMass() + bodiesB.GetInvMass())};
        const auto invRotMassD = InvMass{bodiesD.GetInvRotInertia() * Square(JwD * Meter / Radian)};
        const auto invRotMassB = InvMass{bodiesB.GetInvRotInertia() * Square(JwB * Meter / Radian)};
        invMass += StripUnit(invLinMass + invRotMassD + invRotMassB);
        const auto pD = m_localAnchorD - bodiesD.GetLocalCenter();
        const auto pB = InverseRotate(rB + (posB.linear - posD.linear), qD);
        coordinateB = Dot(pB - pD, m_localAxisD) / Meter;
    }

    const auto C = ((coordinateA + m_ratio * coordinateB) - m_constant);

    const auto impulse = ((invMass > 0)? -C / invMass: 0) * Kilogram * Meter;

    posA += Position{
        bodiesA.GetInvMass() * impulse * JvAC,
        bodiesA.GetInvRotInertia() * impulse * JwA * Meter / Radian
    };
    posB += Position{
        bodiesB.GetInvMass() * impulse * JvBD,
        bodiesB.GetInvRotInertia() * impulse * JwB * Meter / Radian
    };
    posC -= Position{
        bodiesC.GetInvMass() * impulse * JvAC,
        bodiesC.GetInvRotInertia() * impulse * JwC * Meter / Radian
    };
    posD -= Position{
        bodiesD.GetInvMass() * impulse * JvBD,
        bodiesD.GetInvRotInertia() * impulse * JwD * Meter / Radian
    };

    bodiesA.SetPosition(posA);
    bodiesB.SetPosition(posB);
    bodiesC.SetPosition(posC);
    bodiesD.SetPosition(posD);

    // TODO_ERIN not implemented
    return linearError < conf.linearSlop;
}

Length2D GearJoint::GetAnchorA() const
{
    return GetWorldPoint(*GetBodyA(), GetLocalAnchorA());
}

Length2D GearJoint::GetAnchorB() const
{
    return GetWorldPoint(*GetBodyB(), GetLocalAnchorB());
}

Force2D GearJoint::GetReactionForce(Frequency inv_dt) const
{
    return inv_dt * m_impulse * m_JvAC;
}

Torque GearJoint::GetReactionTorque(Frequency inv_dt) const
{
    return inv_dt * m_impulse * m_JwA / Radian;
}

void GearJoint::SetRatio(RealNum ratio)
{
    assert(IsValid(ratio));
    m_ratio = ratio;
}

RealNum GearJoint::GetRatio() const
{
    return m_ratio;
}

GearJointDef box2d::GetGearJointDef(const GearJoint& joint) noexcept
{
    auto def = GearJointDef{};
    
    Set(def, joint);

    def.joint1 = joint.GetJoint1();
    def.joint2 = joint.GetJoint2();
    def.ratio = joint.GetRatio();

    return def;
}
