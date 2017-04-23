/*
 * Original work Copyright (c) 2007-2009 Erin Catto http://www.box2d.org
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

#include <Box2D/Collision/RayCastOutput.hpp>
#include <Box2D/Collision/RayCastInput.hpp>
#include <Box2D/Collision/AABB.hpp>
#include <Box2D/Collision/Shapes/CircleShape.hpp>
#include <Box2D/Collision/Shapes/EdgeShape.hpp>
#include <Box2D/Collision/Shapes/PolygonShape.hpp>
#include <Box2D/Collision/Shapes/ChainShape.hpp>
#include <Box2D/Collision/Shapes/Shape.hpp>
#include <Box2D/Dynamics/Fixture.hpp>

using namespace box2d;
	
// From Real-time Collision Detection, p179.
RayCastOutput box2d::RayCast(const AABB& aabb, const RayCastInput& input)
{
	auto tmin = -MaxFloat;
	auto tmax = MaxFloat;
	
	const auto p1 = input.p1;
	const auto pDelta = input.p2 - input.p1;
	
	UnitVec2 normal;
	
	for (auto i = decltype(pDelta.max_size()){0}; i < pDelta.max_size(); ++i)
	{
		const auto p1i = p1[i];
		const auto pdi = pDelta[i];
		const auto lbi = aabb.GetLowerBound()[i];
		const auto ubi = aabb.GetUpperBound()[i];

		if (almost_zero(pdi / Meter))
		{
			// Parallel.
			if ((p1i < lbi) || (ubi < p1i))
			{
				return RayCastOutput{};
			}
		}
		else
		{
			const auto inv_d = RealNum{1} / pdi;
			auto t1 = RealNum{(lbi - p1i) * inv_d};
			auto t2 = RealNum{(ubi - p1i) * inv_d};
			
			// Sign of the normal vector.
			auto s = -1;
			
			if (t1 > t2)
			{
				Swap(t1, t2);
				s = 1;
			}
			
			// Push the min up
			if (tmin < t1)
			{
				normal = (i == 0)? ((s < 0)? UnitVec2::GetLeft(): UnitVec2::GetRight()): ((s < 0)? UnitVec2::GetBottom(): UnitVec2::GetTop());
				tmin = t1;
			}
			
			// Pull the max down
			tmax = Min(tmax, t2);
			
			if (tmin > tmax)
			{
				return RayCastOutput{};
			}
		}
	};
	
	// Does the ray start inside the box?
	// Does the ray intersect beyond the max fraction?
	if ((tmin < 0) || (tmin > input.maxFraction))
	{
		return RayCastOutput{};
	}
	
	// Intersection.
	return RayCastOutput{normal, tmin};
}

RayCastOutput box2d::RayCast(const Fixture& f, const RayCastInput& input, child_count_t childIndex)
{
	return f.GetShape()->RayCast(input, GetTransformation(f), childIndex);
}
