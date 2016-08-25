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
#include <Box2D/Collision/Manifold.hpp>

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
	Vec2 normal; ///< world vector pointing from A to B
	size_type count = 0;
	Vec2 points[MaxManifoldPoints]; ///< world contact point (point of intersection)
	float_t separations[MaxManifoldPoints]; ///< a negative value indicates overlap, in meters
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

// ---------------- Inline Functions ------------------------------------------

} /* namespace box2d */

#endif
