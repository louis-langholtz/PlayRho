/*
 * Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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

#include <Box2D/Dynamics/Contacts/VelocityConstraint.hpp>
#include <Box2D/Collision/WorldManifold.hpp>
#include <Box2D/Collision/Manifold.hpp>

#define BOX2D_MAGIC(x) (x)

using namespace box2d;

VelocityConstraint::VelocityConstraint(index_type contactIndex,
									   RealNum friction, RealNum restitution, RealNum tangentSpeed,
									   const Manifold& manifold,
									   BodyConstraint& bA, RealNum radiusA,
									   BodyConstraint& bB, RealNum radiusB,
									   Conf conf):
	m_contactIndex{contactIndex},
	m_friction{friction}, m_restitution{restitution}, m_tangentSpeed{tangentSpeed},
	bodyA{bA}, bodyB{bB},
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
		const auto vcp_rA = worldPoint - bodyA.GetPosition().linear;
		const auto vcp_rB = worldPoint - bodyB.GetPosition().linear;
		
		AddPoint(ci.m_normal, ci.m_tangent, vcp_rA, vcp_rB, conf);
	}
	
	if (conf.blockSolve)
	{
		const auto k = ComputeK();
		if (IsValid(k))
		{
			// Ensure a reasonable condition number.
			constexpr auto maxCondNum = BOX2D_MAGIC(RealNum(1000));
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

VelocityConstraint::Point VelocityConstraint::GetPoint(RealNum normalImpulse, RealNum tangentImpulse, Vec2 rA, Vec2 rB, Conf conf) const noexcept
{
	assert(IsValid(normalImpulse));
	assert(IsValid(tangentImpulse));
	assert(IsValid(rA));
	assert(IsValid(rB));
	
	auto point = Point{};

	point.normalImpulse = normalImpulse;
	point.tangentImpulse = tangentImpulse;
	point.rA = rA;
	point.rB = rB;
	point.velocityBias = [&]() {
		// Get the magnitude of the contact relative velocity in direction of the normal.
		// This will be an invalid value if the normal is invalid. The comparison in this
		// case will fail and this lambda will return 0. And that's fine. There's no need
		// to have a check that the normal is valid and possibly incur the overhead of a
		// conditional branch here.
		const auto dv = GetContactRelVelocity(bodyA.GetVelocity(), rA, bodyB.GetVelocity(), rB);
		const auto vn = Dot(dv, GetNormal());
		return (vn < -conf.velocityThreshold)? -GetRestitution() * vn: RealNum{0};
	}();
	
	const auto invMass = RealNum{GetInvMass() * Kilogram};
	const auto invRotInertiaA = bodyA.GetInvRotInertia() * (SquareMeter * Kilogram / SquareRadian);
	const auto invRotInertiaB = bodyB.GetInvRotInertia() * (SquareMeter * Kilogram / SquareRadian);

	point.normalMass = [&](){
		const auto value = invMass
			+ (invRotInertiaA * Square(Cross(rA, GetNormal())))
			+ (invRotInertiaB * Square(Cross(rB, GetNormal())));
		return (value != 0)? RealNum{1} / value : RealNum{0};
	}();
	
	point.tangentMass = [&]() {
		const auto value = invMass
			+ (invRotInertiaA * Square(Cross(rA, GetTangent())))
			+ (invRotInertiaB * Square(Cross(rB, GetTangent())));
		return (value != 0)? RealNum{1} / value : RealNum{0};
	}();

	return point;
}

void VelocityConstraint::AddPoint(RealNum normalImpulse, RealNum tangentImpulse, Vec2 rA, Vec2 rB, Conf conf)
{
	assert(m_pointCount < MaxManifoldPoints);
	m_points[m_pointCount] = GetPoint(normalImpulse * conf.dtRatio, tangentImpulse * conf.dtRatio, rA, rB, conf);
	++m_pointCount;
}

Mat22 VelocityConstraint::ComputeK() const noexcept
{
	const auto pointCount = GetPointCount();
	if (pointCount == 2)
	{
		const auto normal = GetNormal();
		
		const auto rn1A = Cross(GetPointRelPosA(0), normal);
		const auto rn1B = Cross(GetPointRelPosB(0), normal);
		
		const auto rn2A = Cross(GetPointRelPosA(1), normal);
		const auto rn2B = Cross(GetPointRelPosB(1), normal);
		
		const auto invMass = RealNum{GetInvMass() * Kilogram};
		const auto invRotInertiaA = bodyA.GetInvRotInertia() * (SquareMeter * Kilogram / SquareRadian);
		const auto invRotInertiaB = bodyB.GetInvRotInertia() * (SquareMeter * Kilogram / SquareRadian);

		const auto k11 = invMass + (invRotInertiaA * Square(rn1A)) + (invRotInertiaB * Square(rn1B));
		const auto k22 = invMass + (invRotInertiaA * Square(rn2A)) + (invRotInertiaB * Square(rn2B));
		const auto k12 = invMass + (invRotInertiaA * rn1A * rn2A)  + (invRotInertiaB * rn1B * rn2B);

		return Mat22{Vec2{k11, k12}, Vec2{k12, k22}};
	}
	return GetInvalid<Mat22>();
}
