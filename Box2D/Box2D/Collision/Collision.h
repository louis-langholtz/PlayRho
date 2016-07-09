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

#ifndef B2_COLLISION_H
#define B2_COLLISION_H

#include <Box2D/Common/Math.h>
#include <Box2D/Collision/ContactFeature.hpp>
#include <Box2D/Collision/AABB.hpp>

#include <climits>
#include <array>
#include <type_traits>

namespace box2d
{

/// @file
/// Structures and functions used for computing contact points, distance
/// queries, and TOI queries.

class Shape;
class CircleShape;
class EdgeShape;
class PolygonShape;

/// Manifold point data.
/// @detail A manifold point is a contact point belonging to a contact
/// manifold. It holds details related to the geometry and dynamics
/// of the contact points.
/// The local point usage depends on the manifold type:
///   1. e_circles: The local center of circle B;
///   2. e_faceA: The local center of cirlce B or the clip point of polygon B; or,
///   3. e_faceB: The clip point of polygon A.
/// This structure is stored across time steps, so we keep it small.
/// @note The impulses are used for internal caching and may not
///   provide reliable contact forces especially for high speed collisions.
struct ManifoldPoint
{
	ManifoldPoint() noexcept = default;
	ManifoldPoint(const ManifoldPoint& copy) noexcept = default;

	constexpr explicit ManifoldPoint(Vec2 lp, ContactFeature cf = DefaultContactFeature,
									 float_t ni = float_t{0}, float_t ti = float_t{0}) noexcept:
		localPoint{lp}, contactFeature{cf}, normalImpulse{ni}, tangentImpulse{ti}
	{}

	Vec2 localPoint;		///< usage depends on manifold type
	float_t normalImpulse;	///< the non-penetration impulse
	float_t tangentImpulse;	///< the friction impulse
	ContactFeature contactFeature; ///< uniquely identifies a contact point between two shapes
};

/// A manifold for two touching convex shapes.
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
class Manifold
{
public:
	using size_type = std::remove_const<decltype(MaxManifoldPoints)>::type;
	
	enum Type: uint8
	{
		e_unset, ///< Manifold is unset. All other properties are undefined.
		e_circles, ///< Indicates local point is local center of circle A and local normal is not used.
		e_faceA, ///< Indicates local point is center of face A and local normal is normal on shape A.
		e_faceB ///< Indicates local point is center of face B and local normal is normal on shape B.
	};
	
	// For Circles...

	/// Gets a circle-typed manifold.
	/// @param lp Local center of circle A.
	/// @param mp1 Manifold point 1.
	static constexpr Manifold GetForCircles(Vec2 lp, const ManifoldPoint& mp1) noexcept
	{
		return Manifold{e_circles, Vec2_zero, lp, 1, {{mp1}}};
	}

	// For Face A...

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
	static constexpr Manifold GetForFaceA(Vec2 ln, Vec2 lp, const ManifoldPoint& mp1) noexcept
	{
		return Manifold{e_faceA, ln, lp, 1, {{mp1}}};
	}

	/// Gets a face A typed manifold.
	/// @param ln Normal on polygon A.
	/// @param lp Center of face A.
	/// @param mp1 Manifold point 1 (of 2).
	/// @param mp2 Manifold point 2 (of 2).
	static constexpr Manifold GetForFaceA(Vec2 ln, Vec2 lp, const ManifoldPoint& mp1, const ManifoldPoint& mp2) noexcept
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
	static constexpr Manifold GetForFaceB(Vec2 ln, Vec2 lp, const ManifoldPoint& mp1) noexcept
	{
		return Manifold{e_faceB, ln, lp, 1, {{mp1}}};
	}

	/// Gets a face B typed manifold.
	/// @param ln Normal on polygon B.
	/// @param lp Center of face B.
	/// @param mp1 Manifold point 1 (of 2).
	/// @param mp2 Manifold point 2 (of 2).
	static constexpr Manifold GetForFaceB(Vec2 ln, Vec2 lp, const ManifoldPoint& mp1, const ManifoldPoint& mp2) noexcept
	{
		return Manifold{e_faceB, ln, lp, 2, {{mp1, mp2}}};
	}
	
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

	const ManifoldPoint& GetPoint(size_type index) const
	{
		assert((0 <= index) && (index < pointCount));
		return points[index];
	}

	ManifoldPoint& GetPoint(size_type index)
	{
		assert((0 <= index) && (index < pointCount));
		return points[index];
	}
	
	/// Adds a new point.
	/// @detail This can be called up to MaxManifoldPoints times.
	/// GetPointCount() can be called to find out how many points have already been added.
	/// @note Behavior is undefined if this object's type is e_unset.
	/// @note Behavior is undefined if this is called more than MaxManifoldPoints times. 
	void AddPoint(const ManifoldPoint& mp)
	{
		assert(type != e_unset);
		assert(type != e_circles || pointCount == 0);
		assert(pointCount < MaxManifoldPoints);
		points[pointCount] = mp;
		++pointCount;
	}

	/// Gets the local normal for a face typed manifold.
	/// @warning Behavior is undefined if the manifold type is other than face A or face B.
	/// @return Local normal.
	/// @sa SetLocalNormal.
	Vec2 GetLocalNormal() const noexcept
	{
		assert(type == e_faceA || type == e_faceB);
		return localNormal;
	}

	/// Gets the local point.
	/// @detail
	/// This is the:
	/// local center of circle A for circle type manifolds,
	/// the center of face A for face A type manifolds, and
	/// the center of face B for face B type manifolds.
	/// @note Value undefined for unset (e_unset) type manifolds.
	/// @return Local point.
	/// @sa SetLocalPoint.
	Vec2 GetLocalPoint() const noexcept
	{
		assert(type != e_unset);
		return localPoint;
	}

private:
	using ManifoldPointArray = std::array<ManifoldPoint, MaxManifoldPoints>;

	/// Constructs manifold with array of points using the given values.
	/// @param t Manifold type.
	/// @param ln Local normal.
	/// @param lp Local point.
	/// @param n number of points defined in arary.
	/// @param mpa Manifold point array.
	constexpr Manifold(Type t, Vec2 ln, Vec2 lp, size_type n, const ManifoldPointArray& mpa) noexcept:
		type{t}, localNormal{ln}, localPoint{lp}, pointCount{n}, points{mpa} {}
	
	Type type = e_unset; ///< Type of collision this manifold is associated with.
	size_type pointCount = 0; ///< Number of defined manifold points.
	
	Vec2 localNormal; ///< Local normal. @detail Exact usage depends on manifold type. @note Undefined for Type::e_circles.
	Vec2 localPoint; ///< Local point. @detail Exact usage depends on manifold type.
	
	ManifoldPointArray points; ///< Points of contact. @sa pointCount.
};

struct PointSeparation
{
	PointSeparation() noexcept = default;
	PointSeparation(const PointSeparation& copy) noexcept = default;
	
	constexpr PointSeparation(Vec2 point, float_t separation) noexcept: p{point}, s{separation} {}
	
	Vec2 p; ///< Point.
	float_t s; ///< Separation.
};

/// This is used to compute the current state of a contact manifold.
class WorldManifold
{
public:
	using size_type = std::remove_const<decltype(MaxPolygonVertices)>::type;

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
	Vec2 normal;								///< world vector pointing from A to B
	size_type count = 0;
	Vec2 points[MaxManifoldPoints];		///< world contact point (point of intersection)
	float_t separations[MaxManifoldPoints];	///< a negative value indicates overlap, in meters
};

/// Gets the world manifold for the given data.
/// @param manifold Manifold to use.
///   Specifically this uses the manifold's type, local point, local normal, point-count,
///   and the indexed-points' local point data.
/// @param xfA Transformation A.
/// @param radiusA Radius of shape A.
/// @param xfB Transformation B.
/// @param radiusB Radius of shape B.
WorldManifold GetWorldManifold(const Manifold& manifold,
							   const Transformation& xfA, const float_t radiusA,
							   const Transformation& xfB, const float_t radiusB);
	
/// This is used for determining the state of contact points.
enum class PointState
{
	NullState,		///< point does not exist
	AddState,		///< point was added in the update
	PersistState,	///< point persisted across the update
	RemoveState		///< point was removed in the update
};

/// Compute the point states given two manifolds. The states pertain to the transition from manifold1
/// to manifold2. So state1 is either persist or remove while state2 is either add or persist.
using PointStateArray = std::array<PointState,MaxManifoldPoints>;
void GetPointStates(PointStateArray& state1, PointStateArray& state2,
					  const Manifold& manifold1, const Manifold& manifold2);

/// Used for computing contact manifolds.
struct ClipVertex
{
	Vec2 v; ///< Vertex of edge or polygon.
	ContactFeature cf; ///< Contact feature information.
};

/// Ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
struct RayCastInput
{
	Vec2 p1; ///< Point 1.
	Vec2 p2; ///< Point 2.
	float_t maxFraction; ///< Max fraction.
};

/// Ray-cast output data. The ray hits at p1 + fraction * (p2 - p1), where p1 and p2
/// come from RayCastInput.
struct RayCastOutput
{
	Vec2 normal;
	float_t fraction;
};

/// Computes the collision manifold between two circles.
/// @param shapeA Shape A.
/// @param xfA Transformation for shape A.
/// @param shapeB Shape B.
/// @param xfB Transformation for shape B.
/// @return Manifold value with one or more points if the shapes are touching.
Manifold CollideShapes(const CircleShape& shapeA, const Transformation& xfA, const CircleShape& shapeB, const Transformation& xfB);

/// Computes the collision manifold between a polygon and a circle.
/// @param shapeA Shape A.
/// @param xfA Transformation for shape A.
/// @param shapeB Shape B.
/// @param xfB Transformation for shape B.
/// @return Manifold value with one or more points if the shapes are touching.
Manifold CollideShapes(const PolygonShape& shapeA, const Transformation& xfA, const CircleShape& shapeB, const Transformation& xfB);

/// Computes the collision manifold between two polygons.
/// @param shapeA Shape A.
/// @param xfA Transformation for shape A.
/// @param shapeB Shape B.
/// @param xfB Transformation for shape B.
/// @return Manifold value with one or more points if the shapes are touching.
Manifold CollideShapes(const PolygonShape& shapeA, const Transformation& xfA, const PolygonShape& shapeB, const Transformation& xfB);

/// Computes the collision manifold between an edge and a circle.
/// @param shapeA Shape A.
/// @param xfA Transformation for shape A.
/// @param shapeB Shape B.
/// @param xfB Transformation for shape B.
/// @return Manifold value with one or more points if the shapes are touching.
Manifold CollideShapes(const EdgeShape& shapeA, const Transformation& xfA, const CircleShape& shapeB, const Transformation& xfB);

/// Computes the collision manifold between an edge and a circle.
/// @param shapeA Shape A.
/// @param xfA Transformation for shape A.
/// @param shapeB Shape B.
/// @param xfB Transformation for shape B.
/// @return Manifold value with one or more points if the shapes are touching.
Manifold CollideShapes(const EdgeShape& shapeA, const Transformation& xfA, const PolygonShape& shapeB, const Transformation& xfB);

/// Clip array for ClipSegmentToLine.
/// @see ClipSegmentToLine.
using ClipArray = std::array<ClipVertex, MaxManifoldPoints>;

/// Clipping for contact manifolds.
/// @detail This returns an array of points from the given line that are inside of the plane as
///   defined by a given normal and offset.
/// @param vOut Output clip array returning the positions and contact features of points within the plane.
/// @param vIn Input clip array of points defining the line.
/// @param normal Normal of the plane with which to determine intersection.
/// @param offset Offset of the plane with which to determine intersection.
/// @param indexA Index of vertex A.
/// @return Number of valid elements of the output array being returned (# of points of the line found within the plane).
ClipArray::size_type ClipSegmentToLine(ClipArray& vOut, const ClipArray& vIn,
										   const Vec2& normal, float_t offset, ContactFeature::index_t indexA);

/// Determine if two generic shapes overlap.
bool TestOverlap(const Shape& shapeA, child_count_t indexA,
				   const Shape& shapeB, child_count_t indexB,
				   const Transformation& xfA, const Transformation& xfB);

// ---------------- Inline Functions ------------------------------------------

inline bool TestOverlap(const AABB& a, const AABB& b) noexcept
{
	const auto d1 = b.GetLowerBound() - a.GetUpperBound();
	if ((d1.x > float_t{0}) || (d1.y > float_t{0}))
		return false;

	const auto d2 = a.GetLowerBound() - b.GetUpperBound();
	if ((d2.x > float_t{0}) || (d2.y > float_t{0}))
		return false;

	return true;
}

} /* namespace box2d */

#endif
