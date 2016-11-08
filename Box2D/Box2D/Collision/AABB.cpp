/*
 * Original work Copyright (c) 2007-2009 Erin Catto http://www.box2d.org
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

#include <Box2D/Collision/AABB.hpp>

namespace box2d
{
	
	// From Real-time Collision Detection, p179.
	RayCastOutput RayCast(const AABB& aabb, const RayCastInput& input)
	{
		auto tmin = -MaxFloat;
		auto tmax = MaxFloat;
		
		const auto p = input.p1;
		const auto d = input.p2 - input.p1;
		
		UnitVec2 normal;
		
		for (auto i = decltype(d.max_size()){0}; i < d.max_size(); ++i)
		{
			if (almost_zero(d[i]))
			{
				// Parallel.
				if ((p[i] < aabb.GetLowerBound()[i]) || (aabb.GetUpperBound()[i] < p[i]))
				{
					return RayCastOutput{};
				}
			}
			else
			{
				const auto inv_d = float_t{1} / d[i];
				auto t1 = (aabb.GetLowerBound()[i] - p[i]) * inv_d;
				auto t2 = (aabb.GetUpperBound()[i] - p[i]) * inv_d;
				
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
	
} // namespace box2d
