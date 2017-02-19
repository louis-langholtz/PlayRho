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

void VelocityConstraint::Update(const WorldManifold& worldManifold,
								const Vec2 posA, const Vec2 posB,
								Span<const Velocity> velocities,
								const UpdateConf conf)
{
	assert(IsValid(bodyA.GetIndex()));
	assert(IsValid(bodyB.GetIndex()));
	assert(GetPointCount() == worldManifold.GetPointCount());
	
	const auto normal = worldManifold.GetNormal();
	
	SetNormal(normal);
	
	const auto pointCount = GetPointCount();
	
	{
		const auto velA = velocities[bodyA.GetIndex()];
		const auto velB = velocities[bodyB.GetIndex()];
		
		auto restitutionFunc = [&](decltype(pointCount) j)
		{
			const auto worldPoint = worldManifold.GetPoint(j);
			const auto vcp_rA = worldPoint - posA;
			const auto vcp_rB = worldPoint - posB;
			SetPointRelPositions(j, vcp_rA, vcp_rB);
			SetVelocityBiasAtPoint(j, [&]() {
				// Get the magnitude of the contact relative velocity in direction of the normal.
				// This will be an invalid value if the normal is invalid. The comparison in this
				// case will fail and this lambda will return 0. And that's fine. There's no need
				// to have a check that the normal is valid and possibly incur the overhead of a
				// conditional branch here.
				const auto vn = Dot(GetContactRelVelocity(velA, vcp_rA, velB, vcp_rB), normal);
				return (vn < -conf.velocityThreshold)? -GetRestitution() * vn: RealNum{0};
			}());
		};
		switch (pointCount)
		{
			case 2: restitutionFunc(1);
			case 1: restitutionFunc(0);
			default: break;
		}
	}
	
	SetK(GetInvalid<Mat22>());
	
	// If we have two points, then prepare the block solver.
	if ((pointCount == 2) && conf.blockSolve)
	{
		const auto rn1A = Cross(GetPointRelPosA(0), normal);
		const auto rn1B = Cross(GetPointRelPosB(0), normal);
		
		const auto rn2A = Cross(GetPointRelPosA(1), normal);
		const auto rn2B = Cross(GetPointRelPosB(1), normal);
		
		const auto totalInvMass = GetInverseMass(*this);
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
