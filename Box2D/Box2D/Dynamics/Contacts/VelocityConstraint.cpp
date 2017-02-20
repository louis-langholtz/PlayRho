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

void VelocityConstraint::AddPoint(RealNum normalImpulse, RealNum tangentImpulse, Vec2 rA, Vec2 rB, RealNum velocityBias)
{
	assert(m_pointCount < MaxManifoldPoints);
	
	auto point = Point{};
	
	point.normalImpulse = normalImpulse;
	point.tangentImpulse = tangentImpulse;
	point.rA = rA;
	point.rB = rB;
	point.velocityBias = velocityBias;

#if !defined(BOX2D_NOCACHE_VC_POINT_MASSES)
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
#endif

	m_points[m_pointCount] = point;
	++m_pointCount;
}

void VelocityConstraint::Update(const Conf conf)
{
	SetK(GetInvalid<Mat22>());

	const auto normal = GetNormal();
	const auto pointCount = GetPointCount();

	// If we have two points, then prepare the block solver.
	if ((pointCount == 2) && conf.blockSolve)
	{
		const auto rn1A = Cross(GetPointRelPosA(0), normal);
		const auto rn1B = Cross(GetPointRelPosB(0), normal);
		
		const auto rn2A = Cross(GetPointRelPosA(1), normal);
		const auto rn2B = Cross(GetPointRelPosB(1), normal);
		
		const auto totalInvMass = GetInverseMass();
		const auto k11 = totalInvMass + (bodyA.GetInvRotI() * Square(rn1A)) + (bodyB.GetInvRotI() * Square(rn1B));
		const auto k22 = totalInvMass + (bodyA.GetInvRotI() * Square(rn2A)) + (bodyB.GetInvRotI() * Square(rn2B));
		const auto k12 = totalInvMass + (bodyA.GetInvRotI() * rn1A * rn2A)  + (bodyB.GetInvRotI() * rn1B * rn2B);
		
		// Ensure a reasonable condition number.
		constexpr auto maxCondNum = BOX2D_MAGIC(RealNum(1000));
		//const auto k11_squared = Square(k11);
		const auto scaled_k11_squared = k11 * (k11 / maxCondNum);
		const auto k11_times_k22 = k11 * k22;
		if (scaled_k11_squared < (k11_times_k22 - Square(k12))) 
		{
			// K is safe to invert.
			SetK(Mat22{Vec2{k11, k12}, Vec2{k12, k22}});
		}
		else
		{
			// The constraints are redundant, just use one.
			// TODO_ERIN use deepest?
			RemovePoint();
		}
	}
}
