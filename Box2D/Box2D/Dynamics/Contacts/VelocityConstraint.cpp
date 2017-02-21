/*
 * Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2016 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

#define BOX2D_MAGIC(x) (x)

using namespace box2d;

VelocityConstraint::VelocityConstraint(index_type contactIndex,
									   RealNum friction, RealNum restitution, RealNum tangentSpeed,
									   BodyData bA, BodyData bB,
									   UnitVec2 normal):
	m_contactIndex{contactIndex},
	m_friction{friction}, m_restitution{restitution}, m_tangentSpeed{tangentSpeed},
	bodyA{bA}, bodyB{bB}, m_normal{normal},
	m_tangent{GetFwdPerpendicular(normal)},
	m_invMass{bA.GetInvMass() + bB.GetInvMass()}
{
	assert(IsValid(contactIndex));
	assert(IsValid(friction));
	assert(IsValid(restitution));
	assert(IsValid(tangentSpeed));
	assert(IsValid(normal));
}

VelocityConstraint::Point VelocityConstraint::GetPoint(RealNum normalImpulse, RealNum tangentImpulse, Vec2 rA, Vec2 rB,  Velocity velA, Velocity velB, Conf conf) const noexcept
{
	auto point = Point{};
	
	// Get the magnitude of the contact relative velocity in direction of the normal.
	// This will be an invalid value if the normal is invalid. The comparison in this
	// case will fail and this lambda will return 0. And that's fine. There's no need
	// to have a check that the normal is valid and possibly incur the overhead of a
	// conditional branch here.
	const auto vn = Dot(GetContactRelVelocity(velA, rA, velB, rB), GetNormal());
	const auto velocityBias = (vn < -conf.velocityThreshold)? -GetRestitution() * vn: RealNum{0};

	point.normalImpulse = normalImpulse;
	point.tangentImpulse = tangentImpulse;
	point.rA = rA;
	point.rB = rB;
	point.velocityBias = velocityBias;
	
	point.normalMass = [&](){
		const auto value = GetInverseMass()
		+ (bodyA.GetInvRotI() * Square(Cross(rA, GetNormal())))
		+ (bodyB.GetInvRotI() * Square(Cross(rB, GetNormal())));
		return (value != 0)? RealNum{1} / value : RealNum{0};
	}();
	
	point.tangentMass = [&]() {
		const auto value = GetInverseMass()
		+ (bodyA.GetInvRotI() * Square(Cross(rA, GetTangent())))
		+ (bodyB.GetInvRotI() * Square(Cross(rB, GetTangent())));
		return (value != 0)? RealNum{1} / value : RealNum{0};
	}();

	return point;
}

void VelocityConstraint::AddPoint(RealNum normalImpulse, RealNum tangentImpulse, Vec2 rA, Vec2 rB, Velocity velA, Velocity velB, Conf conf)
{
	assert(m_pointCount < MaxManifoldPoints);
	m_points[m_pointCount] = GetPoint(normalImpulse * conf.dtRatio, tangentImpulse * conf.dtRatio, rA, rB, velA, velB, conf);
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
		
		const auto totalInvMass = GetInverseMass();
		const auto k11 = totalInvMass + (bodyA.GetInvRotI() * Square(rn1A)) + (bodyB.GetInvRotI() * Square(rn1B));
		const auto k22 = totalInvMass + (bodyA.GetInvRotI() * Square(rn2A)) + (bodyB.GetInvRotI() * Square(rn2B));
		const auto k12 = totalInvMass + (bodyA.GetInvRotI() * rn1A * rn2A)  + (bodyB.GetInvRotI() * rn1B * rn2B);

		return Mat22{Vec2{k11, k12}, Vec2{k12, k22}};
	}
	return GetInvalid<Mat22>();
}
