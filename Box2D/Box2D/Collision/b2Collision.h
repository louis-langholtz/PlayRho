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

/// @file
/// Structures and functions used for computing contact points, distance
/// queries, and TOI queries.

class b2Shape;
class b2CircleShape;
class b2EdgeShape;
class b2PolygonShape;

const uint8 b2_nullFeature = UCHAR_MAX;

/// The features that intersect to form the contact point
struct b2ContactFeature
{
	using index_t = unsigned int;

	enum Type
	{
		e_vertex = 0,
		e_face = 1
	};

	b2ContactFeature() = default;

	constexpr b2ContactFeature(Type ta, index_t ia, Type tb, index_t ib):
		typeA(ta), indexA(ia), typeB(tb), indexB(ib) {}

	Type typeA; ///< The feature type on shapeA
	index_t indexA; ///< Feature index on shapeA
	Type typeB; ///< The feature type on shapeB
	index_t indexB; ///< Feature index on shapeB
};

constexpr b2ContactFeature b2Flip(const b2ContactFeature& val)
{
	// Swap features
	return b2ContactFeature(val.typeB, val.indexB, val.typeA, val.indexA);
}

constexpr bool operator==(const b2ContactFeature& lhs, const b2ContactFeature& rhs)
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
struct b2ManifoldPoint
{
	b2Vec2 localPoint;		///< usage depends on manifold type
	b2Float normalImpulse;	///< the non-penetration impulse
	b2Float tangentImpulse;	///< the friction impulse
	b2ContactFeature cf;    ///< uniquely identifies a contact point between two shapes
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
class b2Manifold
{
public:
	using size_type = std::remove_cv<decltype(b2_maxPolygonVertices)>::type;

	enum Type
	{
		e_unset,
		e_circles,
		e_faceA,
		e_faceB
	};

	b2Manifold() = default;

	b2Manifold(Type t): type(t) {}

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
	/// @return Value between 0 and b2_maxManifoldPoints.
	/// @sa b2_maxManifoldPoints.
	/// @sa AddPoint().
	/// @sa GetPoint().
	size_type GetPointCount() const noexcept { return pointCount; }

	const b2ManifoldPoint& GetPoint(size_type index) const
	{
		b2Assert((0 <= index) && (index < pointCount));
		return points[index];
	}

	b2ManifoldPoint& GetPoint(size_type index)
	{
		b2Assert((0 <= index) && (index < pointCount));
		return points[index];
	}
	
	/// Adds a new point.
	/// @detail This can be called up to b2_maxManifoldPoints times.
	/// GetPointCount() can be called to find out how many points have already been added.
	/// @note Behavior is undefined if this is called more than b2_maxManifoldPoints times. 
	void AddPoint(const b2Vec2& lp, b2ContactFeature cf = b2ContactFeature{b2ContactFeature::e_vertex, 0, b2ContactFeature::e_vertex, 0})
	{
		b2Assert(pointCount < b2_maxManifoldPoints);
		points[pointCount].localPoint = lp;
		points[pointCount].cf = cf;
		points[pointCount].normalImpulse = 0.f;
		points[pointCount].tangentImpulse = 0.f;
		++pointCount;
	}

	b2Vec2 GetLocalNormal() const noexcept { return localNormal; }
	void SetLocalNormal(const b2Vec2& val) noexcept { localNormal = val; }
	
	b2Vec2 GetLocalPoint() const noexcept { return localPoint; }
	void SetLocalPoint(const b2Vec2& val) noexcept { localPoint = val; }

private:
	Type type = e_unset; ///< Type of collision this manifold is associated with.
	b2Vec2 localNormal;								///< not use for Type::e_points
	b2Vec2 localPoint;								///< usage depends on manifold type
	size_type pointCount = 0;							///< the number of manifold points
	b2ManifoldPoint points[b2_maxManifoldPoints];	///< the points of contact
};

/// This is used to compute the current state of a contact manifold.
class b2WorldManifold
{
public:
	using size_type = std::remove_cv<decltype(b2_maxPolygonVertices)>::type;

	b2WorldManifold() = default;

	b2WorldManifold(const b2Manifold& manifold,
					const b2Transform& xfA, b2Float radiusA,
					const b2Transform& xfB, b2Float radiusB);

	/// Evaluate the manifold with supplied transforms. This assumes
	/// modest motion from the original state. This does not change the
	/// point count, impulses, etc. The radii must come from the shapes
	/// that generated the manifold.
	void Assign(const b2Manifold& manifold,
					const b2Transform& xfA, b2Float radiusA,
					const b2Transform& xfB, b2Float radiusB);

	size_type GetPointCount() const noexcept { return pointCount; }

	b2Vec2 GetNormal() const { return normal; }

	b2Vec2 GetPoint(size_type index) const
	{
		b2Assert(index < b2_maxManifoldPoints);
		return points[index];
	}

	b2Float GetSeparation(size_type index) const
	{
		b2Assert(index < b2_maxManifoldPoints);
		return separations[index];
	}

private:
	b2Vec2 normal;								///< world vector pointing from A to B
	size_type pointCount = 0;
	b2Vec2 points[b2_maxManifoldPoints];		///< world contact point (point of intersection)
	b2Float separations[b2_maxManifoldPoints];	///< a negative value indicates overlap, in meters
};

/// This is used for determining the state of contact points.
enum b2PointState
{
	b2_nullState,		///< point does not exist
	b2_addState,		///< point was added in the update
	b2_persistState,	///< point persisted across the update
	b2_removeState		///< point was removed in the update
};

/// Compute the point states given two manifolds. The states pertain to the transition from manifold1
/// to manifold2. So state1 is either persist or remove while state2 is either add or persist.
using b2PointStateArray = std::array<b2PointState,b2_maxManifoldPoints>;
void b2GetPointStates(b2PointStateArray& state1, b2PointStateArray& state2,
					  const b2Manifold& manifold1, const b2Manifold& manifold2);

/// Used for computing contact manifolds.
struct b2ClipVertex
{
	b2Vec2 v;
	b2ContactFeature cf;
};

/// Ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
struct b2RayCastInput
{
	b2Vec2 p1, p2;
	b2Float maxFraction;
};

/// Ray-cast output data. The ray hits at p1 + fraction * (p2 - p1), where p1 and p2
/// come from b2RayCastInput.
struct b2RayCastOutput
{
	b2Vec2 normal;
	b2Float fraction;
};

/// An axis aligned bounding box.
class b2AABB
{
public:
	b2AABB() = default;

	constexpr b2AABB(b2Vec2 a, b2Vec2 b) noexcept:
		lowerBound(b2Vec2(b2Min(a.x, b.x), b2Min(a.y, b.y))), upperBound(b2Vec2(b2Max(a.x, b.x), b2Max(a.y, b.y))) {}

	/// Get the center of the AABB.
	constexpr b2Vec2 GetCenter() const noexcept
	{
		return (lowerBound + upperBound) / b2Float(2);
	}

	/// Get the extents of the AABB (half-widths).
	constexpr b2Vec2 GetExtents() const noexcept
	{
		return (upperBound - lowerBound) / b2Float(2);
	}

	/// Get the perimeter length
	constexpr b2Float GetPerimeter() const noexcept
	{
		const auto wx = upperBound.x - lowerBound.x;
		const auto wy = upperBound.y - lowerBound.y;
		return b2Float(2) * (wx + wy);
	}

	/// Combine an AABB into this one.
	constexpr b2AABB& operator += (const b2AABB& aabb)
	{
		lowerBound = b2Min(lowerBound, aabb.lowerBound);
		upperBound = b2Max(upperBound, aabb.upperBound);
		return *this;
	}

	/// Does this aabb contain the provided AABB.
	constexpr bool Contains(const b2AABB& aabb) const noexcept
	{
		return
			(lowerBound.x <= aabb.lowerBound.x) && (lowerBound.y <= aabb.lowerBound.y) &&
			(aabb.upperBound.x <= upperBound.x) && (aabb.upperBound.y <= upperBound.y);
	}

	bool RayCast(b2RayCastOutput* output, const b2RayCastInput& input) const;

	b2Vec2 GetLowerBound() const noexcept { return lowerBound; }
	b2Vec2 GetUpperBound() const noexcept { return upperBound; }

	b2AABB& Move(b2Vec2 value) noexcept
	{
		lowerBound += value;
		upperBound += value;
		return *this;
	}

private:
	b2Vec2 lowerBound;	///< the lower vertex
	b2Vec2 upperBound;	///< the upper vertex
};

constexpr inline b2AABB operator + (const b2AABB& aabb1, const b2AABB& aabb2)
{
	return b2AABB{b2Min(aabb1.GetLowerBound(), aabb2.GetLowerBound()), b2Max(aabb1.GetUpperBound(), aabb2.GetUpperBound())};
}

constexpr inline b2AABB operator + (b2Vec2 lhs, const b2AABB& rhs)
{
	return b2AABB{rhs.GetLowerBound() - lhs, rhs.GetUpperBound() + lhs};
}

constexpr inline b2AABB operator + (const b2AABB& lhs, b2Vec2 rhs)
{
	return b2AABB{lhs.GetLowerBound() - rhs, lhs.GetUpperBound() + rhs};
}

/// Computes the collision manifold between two circles.
/// @param shapeA Shape A.
/// @param xfA Transform for shape A.
/// @param shapeB Shape B.
/// @param xfB Transform for shape B.
/// @return Manifold value with one or more points if the shapes are touching.
b2Manifold b2CollideShapes(const b2CircleShape& shapeA, const b2Transform& xfA, const b2CircleShape& shapeB, const b2Transform& xfB);

/// Computes the collision manifold between a polygon and a circle.
/// @param shapeA Shape A.
/// @param xfA Transform for shape A.
/// @param shapeB Shape B.
/// @param xfB Transform for shape B.
/// @return Manifold value with one or more points if the shapes are touching.
b2Manifold b2CollideShapes(const b2PolygonShape& shapeA, const b2Transform& xfA, const b2CircleShape& shapeB, const b2Transform& xfB);

/// Computes the collision manifold between two polygons.
/// @param shapeA Shape A.
/// @param xfA Transform for shape A.
/// @param shapeB Shape B.
/// @param xfB Transform for shape B.
/// @return Manifold value with one or more points if the shapes are touching.
b2Manifold b2CollideShapes(const b2PolygonShape& shapeA, const b2Transform& xfA, const b2PolygonShape& shapeB, const b2Transform& xfB);

/// Computes the collision manifold between an edge and a circle.
/// @param shapeA Shape A.
/// @param xfA Transform for shape A.
/// @param shapeB Shape B.
/// @param xfB Transform for shape B.
/// @return Manifold value with one or more points if the shapes are touching.
b2Manifold b2CollideShapes(const b2EdgeShape& shapeA, const b2Transform& xfA, const b2CircleShape& shapeB, const b2Transform& xfB);

/// Computes the collision manifold between an edge and a circle.
/// @param shapeA Shape A.
/// @param xfA Transform for shape A.
/// @param shapeB Shape B.
/// @param xfB Transform for shape B.
/// @return Manifold value with one or more points if the shapes are touching.
b2Manifold b2CollideShapes(const b2EdgeShape& shapeA, const b2Transform& xfA, const b2PolygonShape& shapeB, const b2Transform& xfB);

/// Clip array for b2ClipSegmentToLine.
/// @see b2ClipSegmentToLine.
using b2ClipArray = std::array<b2ClipVertex, b2_maxManifoldPoints>;

/// Clipping for contact manifolds.
/// Uses Sutherland-Hodgman clipping.
b2ClipArray::size_type b2ClipSegmentToLine(b2ClipArray& vOut, const b2ClipArray& vIn,
										   const b2Vec2& normal, b2Float offset, b2ContactFeature::index_t vertexIndexA);

/// Determine if two generic shapes overlap.
bool b2TestOverlap(const b2Shape& shapeA, child_count_t indexA,
				   const b2Shape& shapeB, child_count_t indexB,
				   const b2Transform& xfA, const b2Transform& xfB);

// ---------------- Inline Functions ------------------------------------------

inline bool b2TestOverlap(const b2AABB& a, const b2AABB& b) noexcept
{
	const auto d1 = b.GetLowerBound() - a.GetUpperBound();
	if ((d1.x > b2Float{0}) || (d1.y > b2Float{0}))
		return false;

	const auto d2 = a.GetLowerBound() - b.GetUpperBound();
	if ((d2.x > b2Float{0}) || (d2.y > b2Float{0}))
		return false;

	return true;
}

#endif
