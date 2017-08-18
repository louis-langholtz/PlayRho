/*
 * Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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

#include <PlayRho/Dynamics/Contacts/VelocityConstraint.hpp>
#include <PlayRho/Collision/WorldManifold.hpp>
#include <PlayRho/Collision/Manifold.hpp>

#define PLAYRHO_MAGIC(x) (x)

using namespace playrho;

namespace {

inline InvMass22 ComputeK(const VelocityConstraint& vc) noexcept
{
    assert(vc.GetPointCount() == 2);

    const auto normal = vc.GetNormal();
    const auto bodyA = vc.GetBodyA();
    const auto bodyB = vc.GetBodyB();
    
    const auto invRotInertiaA = bodyA->GetInvRotInertia();
    const auto invMassA = bodyA->GetInvMass();
    
    const auto invRotInertiaB = bodyB->GetInvRotInertia();
    const auto invMassB = bodyB->GetInvMass();
    
    const auto relA0 = vc.GetPointRelPosA(0);
    const auto relB0 = vc.GetPointRelPosB(0);
    const auto relA1 = vc.GetPointRelPosA(1);
    const auto relB1 = vc.GetPointRelPosB(1);

    const auto rn1A = Length{Cross(relA0, normal)};
    const auto rn1B = Length{Cross(relB0, normal)};
    const auto rn2A = Length{Cross(relA1, normal)};
    const auto rn2B = Length{Cross(relB1, normal)};
    
    const auto invMass = invMassA + invMassB;
    assert(invMass > InvMass{0});
    
    const auto invRotMassA1 = InvMass{(invRotInertiaA * Square(rn1A)) / SquareRadian};
    const auto invRotMassA2 = InvMass{(invRotInertiaA * Square(rn2A)) / SquareRadian};
    const auto invRotMassA = InvMass{(invRotInertiaA * rn1A * rn2A) / SquareRadian};
    const auto invRotMassB1 = InvMass{(invRotInertiaB * Square(rn1B)) / SquareRadian};
    const auto invRotMassB2 = InvMass{(invRotInertiaB * Square(rn2B)) / SquareRadian};
    const auto invRotMassB = InvMass{(invRotInertiaB * rn1B * rn2B) / SquareRadian};
    
    const auto k11 = invMass + invRotMassA1 + invRotMassB1;
    const auto k22 = invMass + invRotMassA2 + invRotMassB2;
    const auto k12 = invMass + invRotMassA + invRotMassB;
    
    return InvMass22{Vector2D<InvMass>{k11, k12}, Vector2D<InvMass>{k12, k22}};
}

} // anonymous namespace

VelocityConstraint::VelocityConstraint(Real friction, Real restitution,
                                       LinearVelocity tangentSpeed,
                                       const WorldManifold& worldManifold,
                                       BodyConstraint& bA,
                                       BodyConstraint& bB,
                                       Conf conf):
    m_friction{friction}, m_restitution{restitution}, m_tangentSpeed{tangentSpeed},
    m_bodyA{&bA}, m_bodyB{&bB},
    m_invMass{bA.GetInvMass() + bB.GetInvMass()},
    m_normal{worldManifold.GetNormal()}
{
    assert(IsValid(friction));
    assert(IsValid(restitution));
    assert(IsValid(tangentSpeed));
    assert(IsValid(m_normal));
    
    const auto pointCount = worldManifold.GetPointCount();
    assert(pointCount > 0);
    for (auto j = decltype(pointCount){0}; j < pointCount; ++j)
    {
        const auto ci = worldManifold.GetImpulses(j);
        const auto worldPoint = worldManifold.GetPoint(j);
        const auto relA = worldPoint - bA.GetPosition().linear;
        const auto relB = worldPoint - bB.GetPosition().linear;
        AddPoint(ci.m_normal, ci.m_tangent, relA, relB, conf);
    }
    
    if (conf.blockSolve && (pointCount == 2))
    {
        const auto k = ComputeK(*this);
        
        // Ensure a reasonable condition number.
        constexpr auto maxCondNum = PLAYRHO_MAGIC(Real(1000));
        const auto scaled_k00_squared = Get<0>(Get<0>(k)) * (Get<0>(Get<0>(k)) / maxCondNum);
        const auto k00_times_k11 = Get<0>(Get<0>(k)) * Get<1>(Get<1>(k));
        const auto k01_squared = Square(Get<1>(Get<0>(k)));
        const auto k_diff = k00_times_k11 - k01_squared;
        if (scaled_k00_squared < k_diff)
        {
            // K is safe to invert.
            // Prepare the block solver.
            m_K = k;
            m_normalMass = Invert(k);
        }
        else
        {
            // The constraints are redundant, just use one.
            // TODO_ERIN use deepest?
            RemovePoint();
        }
    }
}

VelocityConstraint::Point
VelocityConstraint::GetPoint(Momentum normalImpulse, Momentum tangentImpulse,
                             Length2D relA, Length2D relB, Conf conf) const noexcept
{
    assert(IsValid(normalImpulse));
    assert(IsValid(tangentImpulse));
    assert(IsValid(relA));
    assert(IsValid(relB));
    
    const auto bodyA = GetBodyA();
    const auto bodyB = GetBodyB();
    
    const auto invMass = GetInvMass();
    const auto invRotInertiaA = bodyA->GetInvRotInertia();
    const auto invRotInertiaB = bodyB->GetInvRotInertia();
    
    Point point;
    point.normalImpulse = normalImpulse;
    point.tangentImpulse = tangentImpulse;
    point.velocityBias = [&]() {
        // Get the magnitude of the contact relative velocity in direction of the normal.
        // This will be an invalid value if the normal is invalid. The comparison in this
        // case will fail and this lambda will return 0. And that's fine. There's no need
        // to have a check that the normal is valid and possibly incur the overhead of a
        // conditional branch here.
        const auto dv = GetContactRelVelocity(bodyA->GetVelocity(), relA,
                                              bodyB->GetVelocity(), relB);
        const auto vn = LinearVelocity{Dot(dv, GetNormal())};
        return (vn < -conf.velocityThreshold)? -GetRestitution() * vn: LinearVelocity{0};
    }();
    point.relA = relA;
    point.relB = relB;
    point.normalMass = [&](){
        const auto invRotMassA = invRotInertiaA * Square(Cross(relA, GetNormal())) / SquareRadian;
        const auto invRotMassB = invRotInertiaB * Square(Cross(relB, GetNormal())) / SquareRadian;
        const auto value = invMass + invRotMassA + invRotMassB;
        return (value != InvMass{0})? Real{1} / value : Mass{0};
    }();
    point.tangentMass = [&]() {
        const auto invRotMassA = invRotInertiaA * Square(Cross(relA, GetTangent())) / SquareRadian;
        const auto invRotMassB = invRotInertiaB * Square(Cross(relB, GetTangent())) / SquareRadian;
        const auto value = invMass + invRotMassA + invRotMassB;
        return (value != InvMass{0})? Real{1} / value : Mass{0};
    }();

    return point;
}

void VelocityConstraint::AddPoint(Momentum normalImpulse, Momentum tangentImpulse,
                                  Length2D relA, Length2D relB, Conf conf)
{
    assert(m_pointCount < MaxManifoldPoints);
    m_points[m_pointCount] = GetPoint(normalImpulse * conf.dtRatio, tangentImpulse * conf.dtRatio,
                                      relA, relB, conf);
    ++m_pointCount;
}
