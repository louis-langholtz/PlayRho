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

#include <Box2D/Common/Math.hpp>

namespace box2d
{
	class Manifold;
	class Contact;
	class PositionConstraint;

	/// World manifold.
	/// @details
	/// This is used to recoginze the current state of a contact manifold in world coordinates.
	/// @note This data structure is 36-bytes large (on at least one 64-bit platform).
	class WorldManifold
	{
	public:
		using size_type = std::remove_const<decltype(MaxManifoldPoints)>::type;
		
		struct PointSeparation
		{
			PointSeparation() noexcept = default;
			PointSeparation(const PointSeparation& copy) noexcept = default;
			
			constexpr PointSeparation(Vec2 point, RealNum separation) noexcept: p{point}, s{separation} {}
			
			Vec2 p; ///< Point.
			RealNum s; ///< Separation.
		};
		
		/// Default constructor.
		/// @detail
		/// A default constructed world manifold will gave a point count of zero, an invalid
		/// normal, invalid points, and invalid separations.
		WorldManifold() noexcept = default;
		
		constexpr explicit WorldManifold(UnitVec2 normal) noexcept:
			m_normal{normal}, m_count{0},
			m_points{GetInvalid<Vec2>(), GetInvalid<Vec2>()},
			m_separations{GetInvalid<RealNum>(), GetInvalid<RealNum>()}
		{
			// Intentionally empty.
		}
		
		constexpr explicit WorldManifold(UnitVec2 normal, PointSeparation ps0) noexcept:
			m_normal{normal}, m_count{1},
			m_points{ps0.p, GetInvalid<Vec2>()},
			m_separations{ps0.s, GetInvalid<RealNum>()}
		{
			// Intentionally empty.
		}
		
		constexpr explicit WorldManifold(UnitVec2 normal, PointSeparation ps0, PointSeparation ps1) noexcept:
			m_normal{normal}, m_count{2}, m_points{ps0.p, ps1.p}, m_separations{ps0.s, ps1.s}
		{
			// Intentionally empty.
		}
		
		/// Gets the point count.
		///
		/// @detail This is the maximum index value that can be used to access valid point or
		///   separation information.
		///
		/// @return Value between 0 and 2.
		///
		size_type GetPointCount() const noexcept { return m_count; }
		
		/// Gets the normal of the contact.
		/// @detail This is a directional unit-vector.
		/// @return Normal of the contact or an invalid value.
		UnitVec2 GetNormal() const noexcept { return m_normal; }
		
		/// Gets the indexed point's location in world coordinates.
		///
		/// @note Behavior is undefined if the index value is not less than
		///   <code>MaxManifoldPoints</code>
		///
		/// @param index Index to return point for. This must be between 0 and
		///   <code>GetPointCount()</code> to get a valid point from this method.
		///
		/// @return Point or an invalid value if the given index was invalid.
		///
		Vec2 GetPoint(size_type index) const noexcept
		{
			assert(index < MaxManifoldPoints);
			return m_points[index];
		}
		
		/// Gets the amount of separation at the given indexed point.
		///
		/// @note Behavior is undefined if the index value is not less than
		///   <code>MaxManifoldPoints</code>
		/// @param index Index to return separation for. This must be between 0 and
		///   <code>GetPointCount()</code>.
		///
		/// @return Separation amount (a negative value), or an invalid value if the given index
		///   was invalid.
		///
		RealNum GetSeparation(size_type index) const noexcept
		{
			assert(index < MaxManifoldPoints);
			return m_separations[index];
		}
		
	private:	
		UnitVec2 m_normal = GetInvalid<UnitVec2>(); ///< world vector pointing from A to B
		
		size_type m_count = 0;

		/// Points.
		/// @detail Manifold's contact points in world coordinates (mid-point of intersection)
		Vec2 m_points[MaxManifoldPoints] = {GetInvalid<Vec2>(), GetInvalid<Vec2>()};
		
		/// Separations (in meters).
		/// @detail A negative value indicates overlap.
		RealNum m_separations[MaxManifoldPoints] = {GetInvalid<RealNum>(), GetInvalid<RealNum>()};
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
	///   the given manifold has. The returned world manifold points will be the mid-points of the
	///   manifold intersection.
	///
	WorldManifold GetWorldManifold(const Manifold& manifold,
								   const Transformation& xfA, const RealNum radiusA,
								   const Transformation& xfB, const RealNum radiusB);
	
	/// Gets the world manifold for the given data.
	///
	/// @param contact Contact to return a world manifold for.
	///
	/// @return World manifold value for the given inputs which will have the same number of points as
	///   the given manifold has. The returned world manifold points will be the mid-points of the
	///   contact's intersection.
	///
	WorldManifold GetWorldManifold(const Contact& contact);
	
	WorldManifold GetWorldManifold(const PositionConstraint& pc, Position posA, Position posB);
}

#endif /* WorldManifold_hpp */
