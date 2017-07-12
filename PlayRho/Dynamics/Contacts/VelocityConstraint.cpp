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

#define BOX2D_MAGIC(x) (x)

using namespace playrho;

VelocityConstraint::VelocityConstraint(index_type contactIndex,
                                       Real friction, Real restitution,
                                       LinearVelocity tangentSpeed,
                                       const Manifold& manifold,
                                       BodyConstraint& bA, Length radiusA,
                                       BodyConstraint& bB, Length radiusB,
                                       Conf conf):
    m_contactIndex{contactIndex},
    m_friction{friction}, m_restitution{restitution}, m_tangentSpeed{tangentSpeed},
    m_bodyA{&bA}, m_bodyB{&bB},
    m_invMass{bA.GetInvMass() + bB.GetInvMass()}
{
    assert(IsValid(contactIndex));
    assert(IsValid(friction));
    assert(IsValid(restitution));
    assert(IsValid(tangentSpeed));
    
    const auto xfA = GetTransformation(bA.GetPosition(), bA.GetLocalCenter());
    const auto xfB = GetTransformation(bB.GetPosition(), bB.GetLocalCenter());
    const auto worldManifold = GetWorldManifold(manifold, xfA, radiusA, xfB, radiusB);
    m_normal = worldManifold.GetNormal();
    assert(IsValid(m_normal));
    m_tangent = GetFwdPerpendicular(m_normal);

    const auto pointCount = manifold.GetPointCount();
    assert(pointCount > 0);
    for (auto j = decltype(pointCount){0}; j < pointCount; ++j)
    {
        const auto ci = manifold.GetContactImpulses(j);
        
        const auto worldPoint = worldManifold.GetPoint(j);
        const auto relA = worldPoint - bA.GetPosition().linear;
        const auto relB = worldPoint - bB.GetPosition().linear;
        
        AddPoint(ci.m_normal, ci.m_tangent, relA, relB, conf);
    }
    
    if (conf.blockSolve)
    {
        const auto k = ComputeK();
        if (IsValid(k))
        {
            // Ensure a reasonable condition number.
            constexpr auto maxCondNum = BOX2D_MAGIC(Real(1000));
            const auto scaled_k11_squared = k.ex.x * (k.ex.x / maxCondNum);
            const auto k11_times_k22 = k.ex.x * k.ey.y;
            const auto k12_squared = Square(k.ex.y);
            const auto k_diff = k11_times_k22 - k12_squared;
            if (scaled_k11_squared < k_diff)
            {
                // K is safe to invert.
                // Prepare the block solver.
                SetK(k);
            }
            else
            {
                // The constraints are redundant, just use one.
                // TODO_ERIN use deepest?
                RemovePoint();
            }
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
    
    Point point;

    // Get the magnitude of the contact relative velocity in direction of the normal.
    // This will be an invalid value if the normal is invalid. The comparison in this
    // case will fail and this lambda will return 0. And that's fine. There's no need
    // to have a check that the normal is valid and possibly incur the overhead of a
    // conditional branch here.
    const auto dv = GetContactRelVelocity(m_bodyA->GetVelocity(), relA,
                                          m_bodyB->GetVelocity(), relB);
    const auto vn = LinearVelocity{Dot(dv, GetNormal())};

    point.normalImpulse = normalImpulse;
    point.tangentImpulse = tangentImpulse;
    point.relA = relA;
    point.relB = relB;
    point.velocityBias = (vn < -conf.velocityThreshold)? -GetRestitution() * vn: LinearVelocity{0};
    
    const auto invMass = GetInvMass();
    const auto invRotInertiaA = m_bodyA->GetInvRotInertia();
    const auto invRotInertiaB = m_bodyB->GetInvRotInertia();

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

Mat22 VelocityConstraint::ComputeK() const noexcept
{
    const auto pointCount = GetPointCount();
    if (pointCount == 2)
    {
        const auto normal = GetNormal();
        
        const auto rn1A = Cross(GetVec2(GetPointRelPosA(0)), normal);
        const auto rn1B = Cross(GetVec2(GetPointRelPosB(0)), normal);
        
        const auto rn2A = Cross(GetVec2(GetPointRelPosA(1)), normal);
        const auto rn2B = Cross(GetVec2(GetPointRelPosB(1)), normal);
        
        const auto invMass = Real{GetInvMass() * Kilogram};
        const auto invRotInertiaA = m_bodyA->GetInvRotInertia() * (SquareMeter * Kilogram / SquareRadian);
        const auto invRotInertiaB = m_bodyB->GetInvRotInertia() * (SquareMeter * Kilogram / SquareRadian);

        const auto k11 = invMass + (invRotInertiaA * Square(rn1A)) + (invRotInertiaB * Square(rn1B));
        const auto k22 = invMass + (invRotInertiaA * Square(rn2A)) + (invRotInertiaB * Square(rn2B));
        const auto k12 = invMass + (invRotInertiaA * rn1A * rn2A)  + (invRotInertiaB * rn1B * rn2B);

        return Mat22{Vec2{k11, k12}, Vec2{k12, k22}};
    }
    return GetInvalid<Mat22>();
}
