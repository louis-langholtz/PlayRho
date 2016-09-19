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

#ifndef WorldManifold_hpp
#define WorldManifold_hpp

#include <Box2D/Common/Math.h>

namespace box2d
{
	class Manifold;
	class Contact;

	/// This is used to compute the current state of a contact manifold.
	class WorldManifold
	{
	public:
		using size_type = std::remove_const<decltype(MaxPolygonVertices)>::type;
		
		struct PointSeparation
		{
			PointSeparation() noexcept = default;
			PointSeparation(const PointSeparation& copy) noexcept = default;
			
			constexpr PointSeparation(Vec2 point, float_t separation) noexcept: p{point}, s{separation} {}
			
			Vec2 p; ///< Point.
			float_t s; ///< Separation.
		};
		
		WorldManifold() noexcept = default;
		
		constexpr explicit WorldManifold(Vec2 n) noexcept:
			normal{n}, count{0}, points{}, separations{} {}
		
		constexpr explicit WorldManifold(Vec2 n, PointSeparation ps0) noexcept:
			normal{n}, count{1}, points{ps0.p}, separations{ps0.s} {}
		
		constexpr explicit WorldManifold(Vec2 n, PointSeparation ps0, PointSeparation ps1) noexcept:
			normal{n}, count{2}, points{ps0.p, ps1.p}, separations{ps0.s, ps1.s} {}
		
		size_type GetPointCount() const noexcept { return count; }
		
		Vec2 GetNormal() const { return normal; }
		
		Vec2 GetPoint(size_type index) const
		{
			assert(index < MaxManifoldPoints);
			return points[index];
		}
		
		float_t GetSeparation(size_type index) const
		{
			assert(index < MaxManifoldPoints);
			return separations[index];
		}
		
	private:	
		Vec2 normal; ///< world vector pointing from A to B
		size_type count = 0;
		Vec2 points[MaxManifoldPoints]; ///< world contact point (point of intersection)
		float_t separations[MaxManifoldPoints]; ///< a negative value indicates overlap, in meters
	};
	
	/// Gets the world manifold for the given data.
	///
	/// @pre The given manifold input has between 0 and 2 points.
	///
	/// @param manifold Manifold to use.
	///   Uses the manifold's type, local point, local normal, point-count,
	///   and the indexed-points' local point data.
	/// @param xfA Transformation A.
	/// @param radiusA Radius of shape A.
	/// @param xfB Transformation B.
	/// @param radiusB Radius of shape B.
	///
	/// @return World manifold value for the given inputs which will have the same number of points as
	///   the given manifold has.
	///
	WorldManifold GetWorldManifold(const Manifold& manifold,
								   const Transformation& xfA, const float_t radiusA,
								   const Transformation& xfB, const float_t radiusB);
	
	WorldManifold GetWorldManifold(const Contact& contact);	
}

#endif /* WorldManifold_hpp */
