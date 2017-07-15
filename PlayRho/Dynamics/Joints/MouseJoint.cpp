/*
* Original work Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
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

#include <PlayRho/Dynamics/Joints/MouseJoint.hpp>
#include <PlayRho/Dynamics/Body.hpp>
#include <PlayRho/Dynamics/StepConf.hpp>
#include <PlayRho/Dynamics/Contacts/BodyConstraint.hpp>

using namespace playrho;

// p = attached point, m = mouse point
// C = p - m
// Cdot = v
//      = v + cross(w, r)
// J = [I r_skew]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)


bool MouseJoint::IsOkay(const MouseJointDef& def) noexcept
{
    if (!Joint::IsOkay(def))
    {
        return false;
    }
    if (!IsValid(def.target))
    {
        return false;
    }
    if (!(def.dampingRatio >= 0))
    {
        return false;
    }
    return true;
}

MouseJoint::MouseJoint(const MouseJointDef& def):
    Joint{def},
    m_localAnchorB{def.bodyB?
        InverseTransform(def.target, def.bodyB->GetTransformation()):
        GetInvalid<decltype(m_localAnchorB)>()},
    m_targetA{def.target},
    m_maxForce{def.maxForce},
    m_frequency{def.frequency},
    m_dampingRatio{def.dampingRatio}
{
    assert(IsValid(def.target));
    assert(IsValid(def.dampingRatio) && (def.dampingRatio >= 0));
}

void MouseJoint::SetTarget(const Length2D target) noexcept
{
    assert(IsValid(target));
    if (target != m_targetA)
    {
	    m_targetA = target;

        GetBodyB()->SetAwake();
    }
}

Mat22 MouseJoint::GetEffectiveMassMatrix(const BodyConstraint& body) const noexcept
{
    // K    = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
    //      = [1/m1+1/m2     0    ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
    //        [    0     1/m1+1/m2]           [-r1.x*r1.y r1.x*r1.x]           [-r1.x*r1.y r1.x*r1.x]
    
    const auto invMass = body.GetInvMass();
    const auto invRotInertia = body.GetInvRotInertia();

    const auto exx = InvMass{invMass + (invRotInertia * Square(m_rB.y) / SquareRadian) + m_gamma};
    const auto exy = InvMass{-invRotInertia * m_rB.x * m_rB.y / SquareRadian};
    const auto eyy = InvMass{invMass + (invRotInertia * Square(m_rB.x) / SquareRadian) + m_gamma};

    Mat22 K;
    K.ex.x = StripUnit(exx);
    K.ex.y = StripUnit(exy);
    K.ey.x = K.ex.y;
    K.ey.y = StripUnit(eyy);
    return K;
}

void MouseJoint::InitVelocityConstraints(BodyConstraintsMap& bodies, const StepConf& step,
                                         const ConstraintSolverConf&)
{
    auto& bodyConstraintB = At(bodies, GetBodyB());

    const auto posB = bodyConstraintB->GetPosition();
    auto velB = bodyConstraintB->GetVelocity();

    const auto qB = UnitVec2::Get(posB.angular);

    const auto mass = GetMass(*GetBodyB());

    // Frequency
    const auto omega = Real{2} * Pi * Frequency{m_frequency}; // T^-1

    // Damping coefficient
    const auto d = Real{2} * mass * m_dampingRatio * omega; // M T^-1

    // Spring stiffness
    const auto k = mass * Square(omega); // M T^-2

    // magic formulas
    // gamma has units of inverse mass.
    // beta has units of inverse time.
    const auto h = step.GetTime();
    const auto tmp = d + h * k; // M T^-1
    assert(IsValid(Real{tmp * Second / Kilogram}));
    assert((tmp > Real{0} * Kilogram / Second) && !almost_zero(tmp * Second / Kilogram));
    const auto invGamma = Mass{h * tmp}; // M T^-1 * T is simply M.
    m_gamma = (invGamma != Mass{0})? Real{1} / invGamma: InvMass{0};
    const auto beta = Frequency{h * k * m_gamma}; // T * M T^-2 * M^-1 is T^-1

    // Compute the effective mass matrix.
    m_rB = Rotate(m_localAnchorB - bodyConstraintB->GetLocalCenter(), qB);

    m_mass = Invert(GetEffectiveMassMatrix(*bodyConstraintB));

    m_C = LinearVelocity2D{((posB.linear + m_rB) - m_targetA) * beta};
    assert(IsValid(m_C));

    // Cheat with some damping
    velB.angular *= 0.98f;

    if (step.doWarmStart)
    {
        m_impulse *= step.dtRatio;
        const auto P = m_impulse;
        const auto crossBP = AngularMomentum{Cross(m_rB, P) / Radian}; // L * M * L T^-1 is: L^2 M T^-1
        velB += Velocity{bodyConstraintB->GetInvMass() * P, bodyConstraintB->GetInvRotInertia() * crossBP};
    }
    else
    {
        m_impulse = Momentum2D{0, 0};
    }

    bodyConstraintB->SetVelocity(velB);
}

bool MouseJoint::SolveVelocityConstraints(BodyConstraintsMap& bodies, const StepConf& step)
{
    auto& bodyConstraintB = At(bodies, GetBodyB());

    auto velB = bodyConstraintB->GetVelocity();
    assert(IsValid(velB));

    const auto Cdot = LinearVelocity2D{velB.linear + (GetRevPerpendicular(m_rB) * (velB.angular / Radian))};
    const auto ev = Cdot + LinearVelocity2D{m_C + (m_gamma * m_impulse)};
    const auto oldImpulse = m_impulse;
    const auto unitlessImpulse = Transform(GetVec2(-ev), m_mass);
    const auto addImpulse = Momentum2D{
        unitlessImpulse.GetX() * Kilogram * MeterPerSecond,
        unitlessImpulse.GetY() * Kilogram * MeterPerSecond
    };
    assert(IsValid(addImpulse));
    m_impulse += addImpulse;
    const auto maxImpulse = step.GetTime() * Force{m_maxForce};
    if (GetLengthSquared(m_impulse) > Square(maxImpulse))
    {
        m_impulse = GetUnitVector(m_impulse, UnitVec2::GetZero()) * maxImpulse;
    }

    const auto incImpulse = (m_impulse - oldImpulse);
    const auto angImpulseB = AngularMomentum{Cross(m_rB, incImpulse) / Radian};

    velB += Velocity{bodyConstraintB->GetInvMass() * incImpulse, bodyConstraintB->GetInvRotInertia() * angImpulseB};

    bodyConstraintB->SetVelocity(velB);
    
    return incImpulse == Momentum2D{0, 0};
}

bool MouseJoint::SolvePositionConstraints(BodyConstraintsMap& bodies, const ConstraintSolverConf& conf) const
{
    NOT_USED(bodies);
    NOT_USED(conf);
    return true;
}

Length2D MouseJoint::GetAnchorB() const
{
    return GetBodyB()? GetWorldPoint(*GetBodyB(), GetLocalAnchorB()): GetInvalid<Length2D>();
}

Force2D MouseJoint::GetReactionForce(Frequency inv_dt) const
{
    return inv_dt * m_impulse;
}

Torque MouseJoint::GetReactionTorque(Frequency inv_dt) const
{
    return inv_dt * AngularMomentum{0};
}

void MouseJoint::ShiftOrigin(const Length2D newOrigin)
{
    m_targetA -= newOrigin;
}

MouseJointDef playrho::GetMouseJointDef(const MouseJoint& joint) noexcept
{
    auto def = MouseJointDef{};

    Set(def, joint);

    def.target = joint.GetTarget();
    def.maxForce = joint.GetMaxForce();
    def.frequency = joint.GetFrequency();
    def.dampingRatio = joint.GetDampingRatio();

    return def;
}
