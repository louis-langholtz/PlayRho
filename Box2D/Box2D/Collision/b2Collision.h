/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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

#include <Box2D/Common/b2Math.h>

#include <climits>
#include <array>
#include <type_traits>

namespace box2d
{

/// @file
/// Structures and functions used for computing contact points, distance
/// queries, and TOI queries.

class Shape;
class b2CircleShape;
class EdgeShape;
class b2PolygonShape;

/// The features that intersect to form the contact point
struct ContactFeature
{
	using index_t = unsigned int;

	enum Type
	{
		e_vertex = 0,
		e_face = 1
	};

	ContactFeature() = default;

	constexpr ContactFeature(Type ta, index_t ia, Type tb, index_t ib):
		typeA(ta), indexA(ia), typeB(tb), indexB(ib) {}

	Type typeA; ///< The feature type on shape A
	index_t indexA; ///< Feature index on shape A
	Type typeB; ///< The feature type on shape B
	index_t indexB; ///< Feature index on shape B
};

constexpr ContactFeature Flip(const ContactFeature& val)
{
	// Swap features
	return ContactFeature(val.typeB, val.indexB, val.typeA, val.indexA);
}

constexpr bool operator==(const ContactFeature& lhs, const ContactFeature& rhs)
{
	return lhs.typeA == rhs.typeA && lhs.typeB == rhs.typeB && lhs.indexA == rhs.indexA && lhs.indexB == rhs.indexB;
}

/// Manifold point data.
/// @detail A manifold point is a contact point belonging to a contact
/// manifold. It holds details related to the geometry and dynamics
/// of the contact points.
/// The local point usage depends on the manifold type:
/// -e_circles: the local center of circleB
/// -e_faceA: the local center of cirlceB or the clip point of polygonB
/// -e_faceB: the clip point of polygonA
/// This structure is stored across time steps, so we keep it small.
/// @note The impulses are used for internal caching and may not
/// provide reliable contact forces, especially for high speed collisions.
struct ManifoldPoint
{
	Vec2 localPoint;		///< usage depends on manifold type
	float_t normalImpulse;	///< the non-penetration impulse
	float_t tangentImpulse;	///< the friction impulse
	ContactFeature cf;    ///< uniquely identifies a contact point between two shapes
};

/// A manifold for two touching convex shapes.
/// Box2D supports multiple types of contact:
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
	using size_type = std::remove_cv<decltype(MaxPolygonVertices)>::type;

	enum Type
	{
		e_unset,
		e_circles,
		e_faceA,
		e_faceB
	};

	Manifold() = default;

	/// Constructs a manifold with the given values.
	/// @param t Manifold type.
	/// @param ln Local normal.
	/// @param lp Local point.
	Manifold(Type t, Vec2 ln = Vec2_zero, Vec2 lp = Vec2_zero): type(t), localNormal(ln), localPoint(lp) {}

	Type GetType() const noexcept { return type; }

	/// Sets the type of this manifold object.
	/// @note This also results in the manifold's point count being reset to zero.
	void SetType(Type val) noexcept
	{
		type = val;
		pointCount = 0;
	}

	/// Gets the manifold point count.
	/// @detail This is the count of points added using the AddPoint() method.
	///   Only up to this many points can be validly accessed using the GetPoint() method.
	///   Non-zero values indicate that the two shapes are touching.
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
	/// @note Behavior is undefined if this is called more than MaxManifoldPoints times. 
	void AddPoint(const Vec2& lp, ContactFeature cf = ContactFeature{ContactFeature::e_vertex, 0, ContactFeature::e_vertex, 0})
	{
		assert(pointCount < MaxManifoldPoints);
		points[pointCount].localPoint = lp;
		points[pointCount].cf = cf;
		points[pointCount].normalImpulse = 0.f;
		points[pointCount].tangentImpulse = 0.f;
		++pointCount;
	}

	Vec2 GetLocalNormal() const noexcept { return localNormal; }
	void SetLocalNormal(const Vec2& val) noexcept { localNormal = val; }
	
	Vec2 GetLocalPoint() const noexcept { return localPoint; }
	void SetLocalPoint(const Vec2& val) noexcept { localPoint = val; }

private:
	Type type = e_unset; ///< Type of collision this manifold is associated with.
	Vec2 localNormal;								///< not use for Type::e_points
	Vec2 localPoint;								///< usage depends on manifold type
	size_type pointCount = 0;							///< the number of manifold points
	ManifoldPoint points[MaxManifoldPoints];	///< the points of contact
};

/// This is used to compute the current state of a contact manifold.
class WorldManifold
{
public:
	using size_type = std::remove_cv<decltype(MaxPolygonVertices)>::type;

	WorldManifold() = default;

	WorldManifold(const Manifold& manifold,
					const Transform& xfA, float_t radiusA,
					const Transform& xfB, float_t radiusB);

	/// Evaluate the manifold with supplied transforms. This assumes
	/// modest motion from the original state. This does not change the
	/// point count, impulses, etc. The radii must come from the shapes
	/// that generated the manifold.
	void Assign(const Manifold& manifold,
					const Transform& xfA, float_t radiusA,
					const Transform& xfB, float_t radiusB);

	size_type GetPointCount() const noexcept { return pointCount; }

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
	size_type pointCount = 0;
	Vec2 points[MaxManifoldPoints];		///< world contact point (point of intersection)
	float_t separations[MaxManifoldPoints];	///< a negative value indicates overlap, in meters
};

/// This is used for determining the state of contact points.
enum class b2PointState
{
	NullState,		///< point does not exist
	AddState,		///< point was added in the update
	PersistState,	///< point persisted across the update
	RemoveState		///< point was removed in the update
};

/// Compute the point states given two manifolds. The states pertain to the transition from manifold1
/// to manifold2. So state1 is either persist or remove while state2 is either add or persist.
using b2PointStateArray = std::array<b2PointState,MaxManifoldPoints>;
void b2GetPointStates(b2PointStateArray& state1, b2PointStateArray& state2,
					  const Manifold& manifold1, const Manifold& manifold2);

/// Used for computing contact manifolds.
struct b2ClipVertex
{
	Vec2 v; ///< Vertex of edge or polygon.
	ContactFeature cf; ///< Contact feature information.
};

/// Ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
struct b2RayCastInput
{
	Vec2 p1, p2;
	float_t maxFraction;
};

/// Ray-cast output data. The ray hits at p1 + fraction * (p2 - p1), where p1 and p2
/// come from b2RayCastInput.
struct b2RayCastOutput
{
	Vec2 normal;
	float_t fraction;
};

/// An axis aligned bounding box.
class AABB
{
public:
	AABB() = default;

	constexpr AABB(Vec2 a, Vec2 b) noexcept:
		lowerBound(Vec2(Min(a.x, b.x), Min(a.y, b.y))), upperBound(Vec2(Max(a.x, b.x), Max(a.y, b.y))) {}

	/// Get the center of the AABB.
	constexpr Vec2 GetCenter() const noexcept
	{
		return (lowerBound + upperBound) / float_t(2);
	}

	/// Get the extents of the AABB (half-widths).
	constexpr Vec2 GetExtents() const noexcept
	{
		return (upperBound - lowerBound) / float_t(2);
	}

	/// Get the perimeter length
	constexpr float_t GetPerimeter() const noexcept
	{
		const auto wx = upperBound.x - lowerBound.x;
		const auto wy = upperBound.y - lowerBound.y;
		return float_t(2) * (wx + wy);
	}

	/// Combine an AABB into this one.
	constexpr AABB& operator += (const AABB& aabb)
	{
		lowerBound = Min(lowerBound, aabb.lowerBound);
		upperBound = Max(upperBound, aabb.upperBound);
		return *this;
	}

	/// Does this aabb contain the provided AABB.
	constexpr bool Contains(const AABB& aabb) const noexcept
	{
		return
			(lowerBound.x <= aabb.lowerBound.x) && (lowerBound.y <= aabb.lowerBound.y) &&
			(aabb.upperBound.x <= upperBound.x) && (aabb.upperBound.y <= upperBound.y);
	}

	bool RayCast(b2RayCastOutput* output, const b2RayCastInput& input) const;

	Vec2 GetLowerBound() const noexcept { return lowerBound; }
	Vec2 GetUpperBound() const noexcept { return upperBound; }

	AABB& Move(Vec2 value) noexcept
	{
		lowerBound += value;
		upperBound += value;
		return *this;
	}

private:
	Vec2 lowerBound;	///< the lower vertex
	Vec2 upperBound;	///< the upper vertex
};

constexpr inline AABB operator + (const AABB& aabb1, const AABB& aabb2)
{
	return AABB{Min(aabb1.GetLowerBound(), aabb2.GetLowerBound()), Max(aabb1.GetUpperBound(), aabb2.GetUpperBound())};
}

constexpr inline AABB operator + (Vec2 lhs, const AABB& rhs)
{
	return AABB{rhs.GetLowerBound() - lhs, rhs.GetUpperBound() + lhs};
}

constexpr inline AABB operator + (const AABB& lhs, Vec2 rhs)
{
	return AABB{lhs.GetLowerBound() - rhs, lhs.GetUpperBound() + rhs};
}

/// Computes the collision manifold between two circles.
/// @param shapeA Shape A.
/// @param xfA Transform for shape A.
/// @param shapeB Shape B.
/// @param xfB Transform for shape B.
/// @return Manifold value with one or more points if the shapes are touching.
Manifold CollideShapes(const b2CircleShape& shapeA, const Transform& xfA, const b2CircleShape& shapeB, const Transform& xfB);

/// Computes the collision manifold between a polygon and a circle.
/// @param shapeA Shape A.
/// @param xfA Transform for shape A.
/// @param shapeB Shape B.
/// @param xfB Transform for shape B.
/// @return Manifold value with one or more points if the shapes are touching.
Manifold CollideShapes(const b2PolygonShape& shapeA, const Transform& xfA, const b2CircleShape& shapeB, const Transform& xfB);

/// Computes the collision manifold between two polygons.
/// @param shapeA Shape A.
/// @param xfA Transform for shape A.
/// @param shapeB Shape B.
/// @param xfB Transform for shape B.
/// @return Manifold value with one or more points if the shapes are touching.
Manifold CollideShapes(const b2PolygonShape& shapeA, const Transform& xfA, const b2PolygonShape& shapeB, const Transform& xfB);

/// Computes the collision manifold between an edge and a circle.
/// @param shapeA Shape A.
/// @param xfA Transform for shape A.
/// @param shapeB Shape B.
/// @param xfB Transform for shape B.
/// @return Manifold value with one or more points if the shapes are touching.
Manifold CollideShapes(const EdgeShape& shapeA, const Transform& xfA, const b2CircleShape& shapeB, const Transform& xfB);

/// Computes the collision manifold between an edge and a circle.
/// @param shapeA Shape A.
/// @param xfA Transform for shape A.
/// @param shapeB Shape B.
/// @param xfB Transform for shape B.
/// @return Manifold value with one or more points if the shapes are touching.
Manifold CollideShapes(const EdgeShape& shapeA, const Transform& xfA, const b2PolygonShape& shapeB, const Transform& xfB);

/// Clip array for ClipSegmentToLine.
/// @see ClipSegmentToLine.
using ClipArray = std::array<b2ClipVertex, MaxManifoldPoints>;

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
bool b2TestOverlap(const Shape& shapeA, child_count_t indexA,
				   const Shape& shapeB, child_count_t indexB,
				   const Transform& xfA, const Transform& xfB);

// ---------------- Inline Functions ------------------------------------------

inline bool b2TestOverlap(const AABB& a, const AABB& b) noexcept
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
