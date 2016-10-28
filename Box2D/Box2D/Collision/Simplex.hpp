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

#ifndef Simplex_hpp
#define Simplex_hpp

#include <Box2D/Common/ArrayList.hpp>
#include <Box2D/Collision/SimplexVertex.hpp>

namespace box2d
{
	/// Simplex.
	/// @detail
	/// An encapsulation of a point, line segment, or triangle.
	/// These are defined respectively as: a 0-simplex, a 1-simplex, and a 2-simplex.
	/// Used in doing GJK collision detection.
	/// @sa https://en.wikipedia.org/wiki/Simplex
	/// @sa https://en.wikipedia.org/wiki/Gilbert%2DJohnson%2DKeerthi_distance_algorithm
	using Simplex = ArrayList<SimplexVertex, MaxSimplexVertices>;
	
	/// Gets the "search direction" for the given simplex.
	/// @param simplex A one or two vertex simplex.
	/// @warning Behavior is undefined if the given simplex has zero vertices.
	/// @return "search direction" vector.
	constexpr inline Vec2 GetSearchDirection(const Simplex& simplex) noexcept
	{
		static_assert(std::tuple_size<Simplex>::value == 3,
					  "Invalid maximum # of elements of Simplex");

		assert((simplex.size() == 1) || (simplex.size() == 2));
		switch (simplex.size())
		{
			case 1:
				return -GetW(simplex[0]);
				
			case 2:
			{
				const auto e12 = GetW(simplex[1]) - GetW(simplex[0]);
				const auto sgn = Cross(e12, -GetW(simplex[0]));
				// If sgn > 0, then origin is left of e12, else origin is right of e12.
				return (sgn > float_t{0})? GetRevPerpendicular(e12): GetFwdPerpendicular(e12);
			}
				
			default:
				return Vec2_zero;
		}
	}
	
	/// Gets the "closest point".
	/// @note This uses the vertices "a" values when count is 2.
	constexpr inline Vec2 GetClosestPoint(const Simplex& simplex)
	{
		static_assert(std::tuple_size<Simplex>::value == 3,
					  "Invalid maximum # of elements of Simplex");

		assert(simplex.size() < 4);
		switch (simplex.size())
		{
			case 1: return GetW(simplex[0]);
			case 2: return GetScaledDelta(simplex[0]) + GetScaledDelta(simplex[1]);
			case 3: return Vec2_zero;
			default: return Vec2_zero;
		}
	}
	
	inline float_t GetMetric(const Simplex& simplex)
	{
		static_assert(std::tuple_size<Simplex>::value == 3,
					  "Invalid maximum # of elements of Simplex");

		assert(simplex.size() < 4);
		switch (simplex.size())
		{
			case 0: return float_t{0};
			case 1: return float_t{0};
			case 2:	return Sqrt(LengthSquared(GetW(simplex[0]) - GetW(simplex[1])));
			case 3:	return Cross(GetW(simplex[1]) - GetW(simplex[0]),
								 GetW(simplex[2]) - GetW(simplex[0]));
			default: break; // should not be reached
		}
		return float_t{0};
	}

}	

#endif /* Simplex_hpp */
