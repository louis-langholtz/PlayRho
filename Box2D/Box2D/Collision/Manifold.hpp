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

#ifndef Manifold_hpp
#define Manifold_hpp

#include <Box2D/Common/Math.h>
#include <Box2D/Collision/ContactFeature.hpp>

#include <array>

namespace box2d
{	
	/// Manifold for two touching convex shapes.
	/// @detail
	/// Multiple types of contact are supported:
	/// - clip point versus plane with radius
	/// - point versus point with radius (circles)
	/// The local point usage depends on the manifold type:
	/// -e_circles: the local center of circleA
	/// -e_faceA: the center of faceA
	/// -e_faceB: the center of faceB
	/// Similarly the local normal usage:
	/// -e_circles: not used
	/// -e_faceA: the normal on polygonA
	/// -e_faceB: the normal on polygonB
	/// We store contacts in this way so that position correction can
	/// account for movement, which is critical for continuous physics.
	/// All contact scenarios must be expressed in one of these types.
	/// This structure is stored across time steps, so we keep it small.
	/// @note This data structure is at least 59-bytes large.
	class Manifold
	{
	public:
		using size_type = std::remove_const<decltype(MaxManifoldPoints)>::type;
		
		enum Type: uint8
		{
			/// Unset type.
			/// @detail Manifold is unset.
			/// Point count is zero, point data is undefined, and all other properties are invalid.
			e_unset,
			
			/// Circles type.
			/// @detail Manifold is for circle-to-circle like collisions.
			/// For manifolds of this type:
			///   the local point is local center of "circle-A" (where shape A wasn't necessarily
			///     a circle but treating it as such is useful),
			///   the local normal is invalid, and
			///   the point count will be zero or one.
			e_circles,

			/// Face-A type.
			/// @detail Indicates local point is center of face A and local normal is normal on shape A.
			e_faceA,

			/// Face-B type.
			/// @detail Indicates local point is center of face B and local normal is normal on shape B.
			e_faceB
		};
		
		/// Manifold point data.
		/// @detail A manifold point is a contact point belonging to a contact
		/// manifold. It holds details related to the geometry and dynamics
		/// of the contact points.
		/// This structure is stored across time steps, so we keep it small.
		/// @note The impulses are used for internal caching and may not
		///   provide reliable contact forces especially for high speed collisions.
		/// @note This structure is at least 20-bytes large.
		struct Point
		{
			Point() noexcept = default;
			Point(const Point& copy) = default;
			
			constexpr explicit Point(Vec2 lp, ContactFeature cf = DefaultContactFeature,
									 float_t ni = float_t{0}, float_t ti = float_t{0}) noexcept:
				localPoint{lp}, contactFeature{cf}, normalImpulse{ni}, tangentImpulse{ti}
			{}
			
			/// Local point.
			/// @detail Usage depends on manifold type.
			/// For circles type manifolds, this is the local center of circle B.
			/// For face-A type manifolds, this is the local center of cirlce B or the clip point of polygon B.
			/// For face-B type manifolds, this is the clip point of polygon A.
			/// @note 8-bytes.
			Vec2 localPoint;

			float_t normalImpulse; ///< the non-penetration impulse (4-bytes).
			
			float_t tangentImpulse; ///< the friction impulse (4-bytes).
			
			ContactFeature contactFeature; ///< uniquely identifies a contact point between two shapes (4-bytes).
		};
		
		// For Circles type manifolds...
		
		/// Gets a circle-typed manifold.
		/// @param lp Local center of "circle" A.
		/// @param mp1 Manifold point 1 whose local point is the center of circle B.
		static constexpr Manifold GetForCircles(Vec2 lp, const Point& mp1) noexcept
		{
			return Manifold{e_circles, GetInvalid<Vec2>(), lp, 1, {{mp1}}};
		}
		
		// For Face A type manifolds...
		
		/// Gets a face A typed manifold.
		/// @param ln Normal on polygon A.
		/// @param lp Center of face A.
		static constexpr Manifold GetForFaceA(Vec2 ln, Vec2 lp) noexcept
		{
			return Manifold{e_faceA, ln, lp, 0, {{}}};
		}
		
		/// Gets a face A typed manifold.
		/// @param ln Normal on polygon A.
		/// @param lp Center of face A.
		/// @param mp1 Manifold point 1 (of 1).
		static constexpr Manifold GetForFaceA(Vec2 ln, Vec2 lp, const Point& mp1) noexcept
		{
			return Manifold{e_faceA, ln, lp, 1, {{mp1}}};
		}
		
		/// Gets a face A typed manifold.
		/// @param ln Normal on polygon A.
		/// @param lp Center of face A.
		/// @param mp1 Manifold point 1 (of 2).
		/// @param mp2 Manifold point 2 (of 2).
		static constexpr Manifold GetForFaceA(Vec2 ln, Vec2 lp, const Point& mp1, const Point& mp2) noexcept
		{
			return Manifold{e_faceA, ln, lp, 2, {{mp1, mp2}}};
		}
		
		// For Face B...
		
		/// Gets a face B typed manifold.
		/// @param ln Normal on polygon B.
		/// @param lp Center of face B.
		static constexpr Manifold GetForFaceB(Vec2 ln, Vec2 lp) noexcept
		{
			return Manifold{e_faceB, ln, lp, 0, {{}}};
		}
		
		/// Gets a face B typed manifold.
		/// @param ln Normal on polygon B.
		/// @param lp Center of face B.
		/// @param mp1 Manifold point 1.
		static constexpr Manifold GetForFaceB(Vec2 ln, Vec2 lp, const Point& mp1) noexcept
		{
			return Manifold{e_faceB, ln, lp, 1, {{mp1}}};
		}
		
		/// Gets a face B typed manifold.
		/// @param ln Normal on polygon B.
		/// @param lp Center of face B.
		/// @param mp1 Manifold point 1 (of 2).
		/// @param mp2 Manifold point 2 (of 2).
		static constexpr Manifold GetForFaceB(Vec2 ln, Vec2 lp, const Point& mp1, const Point& mp2) noexcept
		{
			return Manifold{e_faceB, ln, lp, 2, {{mp1, mp2}}};
		}
		
		/// Default constructor.
		/// @detail
		/// Constructs an unset-type manifold.
		/// For an unset-type manifold:
		/// point count is zero, point data is undefined, and all other properties are invalid.
		Manifold() noexcept = default;
		
		Manifold(const Manifold& copy) noexcept = default;
		
		/// Gets the type of this manifold.
		Type GetType() const noexcept { return type; }
		
		/// Gets the manifold point count.
		/// @detail This is the count of contact points for this manifold.
		///   Only up to this many points can be validly accessed using the GetPoint() method.
		/// @note Non-zero values indicate that the two shapes are touching.
		/// @return Value between 0 and MaxManifoldPoints.
		/// @sa MaxManifoldPoints.
		/// @sa AddPoint().
		/// @sa GetPoint().
		size_type GetPointCount() const noexcept { return pointCount; }
		
		const Point& GetPoint(size_type index) const
		{
			assert((0 <= index) && (index < pointCount));
			return points[index];
		}
		
		Point& GetPoint(size_type index)
		{
			assert((0 <= index) && (index < pointCount));
			return points[index];
		}
		
		/// Adds a new point.
		/// @detail This can be called once for circle type manifolds,
		///   and up to MaxManifoldPoints times for face-A or face-B type manifolds.
		/// GetPointCount() can be called to find out how many points have already been added.
		/// @note Behavior is undefined if this object's type is e_unset.
		/// @note Behavior is undefined if this is called more than MaxManifoldPoints times. 
		void AddPoint(const Point& mp)
		{
			assert(type != e_unset);
			assert(type != e_circles || pointCount == 0);
			assert(pointCount < MaxManifoldPoints);
			points[pointCount] = mp;
			++pointCount;
		}
		
		/// Gets the local normal for a face-type manifold.
		/// @return Local normal if the manifold type is face A or face B, else invalid value.
		/// @sa SetLocalNormal.
		Vec2 GetLocalNormal() const noexcept
		{
			return localNormal;
		}
		
		/// Gets the local point.
		/// @detail
		/// This is the:
		/// local center of "circle" A for circles-type manifolds,
		/// the center of face A for face-A-type manifolds, and
		/// the center of face B for face-B-type manifolds.
		/// @note Value invalid for unset (e_unset) type manifolds.
		/// @return Local point.
		/// @sa SetLocalPoint.
		Vec2 GetLocalPoint() const noexcept
		{
			return localPoint;
		}
		
	private:
		using PointArray = std::array<Point, MaxManifoldPoints>;
		
		/// Constructs manifold with array of points using the given values.
		/// @param t Manifold type.
		/// @param ln Local normal.
		/// @param lp Local point.
		/// @param n number of points defined in arary.
		/// @param mpa Manifold point array.
		constexpr Manifold(Type t, Vec2 ln, Vec2 lp, size_type n, const PointArray& mpa) noexcept:
			type{t}, localNormal{ln}, localPoint{lp}, pointCount{n}, points{mpa}
		{
			assert(t != e_unset || n == 0);
			assert(t != e_circles || (n == 1 && !IsValid(ln)));
		}
		
		Type type = e_unset; ///< Type of collision this manifold is associated with (1-byte).
		size_type pointCount = 0; ///< Number of defined manifold points (2-bytes).
		
		/// Local normal.
		/// @detail Exact usage depends on manifold type (8-bytes).
		/// @note Invalid for the unset and circle manifold types.
		Vec2 localNormal = GetInvalid<Vec2>();

		/// Local point.
		/// @detail Exact usage depends on manifold type (8-bytes).
		/// @note Invalid for the unset manifold type.
		Vec2 localPoint = GetInvalid<Vec2>();
		
		PointArray points; ///< Points of contact (at least 40-bytes). @sa pointCount.
	};
	
} // namespace box2d

#endif /* Manifold_hpp */
