//
//  PositionSolverManifold.cpp
//  Box2D
//
//  Created by Louis D. Langholtz on 6/23/16.
//
//

#include <Box2D/Dynamics/Contacts/PositionSolverManifold.hpp>

namespace box2d
{

/// Gets the position solver manifold in world coordinates for a circles-type manifold.
/// @param lp Local point. Location of shape A in local coordinates.
/// @param plp Point's local point. Location of shape B in local coordinates.
/// @param xfA Transformation for body A.
/// @param xfB Transformation for body B.
/// @note The returned separation is the magnitude of the positional difference of the two points.
///   This is always a non-negative amount.
static inline PositionSolverManifold GetForCircles(Vec2 lp, Vec2 plp,
												   const Transformation& xfA, const Transformation& xfB)
{
	const auto pointA = Transform(lp, xfA);
	const auto pointB = Transform(plp, xfB);
	const auto delta = pointB - pointA; // The edge from pointA to pointB
	const auto normal = GetUnitVector(delta, UnitVec2::GetZero()); // The direction of the edge.
	const auto midpoint = (pointA + pointB) / 2;
	const auto separation = Dot(delta, normal); // The length of edge without doing sqrt again.
	return PositionSolverManifold{normal, midpoint, separation};
}

/// Gets the position solver manifold in world coordinates for a face-a-type manifold.
/// @param lp Local point. Location for shape A in local coordinates.
/// @param plp Point's local point. Location for shape B in local coordinates.
/// @param xfA Transformation for shape A.
/// @param xfB Transformation for shape B.
/// @param ln Local normal for shape A to be transformed into a world normal based on the
///   transformation for shape A.
/// @return Separation is the dot-product of the positional difference between the two points in
///   the direction of the world normal.
static inline PositionSolverManifold GetForFaceA(Vec2 lp, Vec2 plp,
												 const Transformation& xfA, const Transformation& xfB,
												 UnitVec2 ln)
{
	const auto planePoint = Transform(lp, xfA);
	const auto clipPoint = Transform(plp, xfB);
	const auto normal = Rotate(ln, xfA.q);
	const auto separation = Dot(clipPoint - planePoint, normal);
	return PositionSolverManifold{normal, clipPoint, separation};
}

/// Gets the position solver manifold in world coordinates for a face-b-type manifold.
/// @param lp Local point.
/// @param plp Point's local point.
/// @param xfA Transformation for body A.
/// @param xfB Transformation for body B.
/// @param ln Local normal for shape B to be transformed into a world normal based on the
///   transformation for shape B.
static inline PositionSolverManifold GetForFaceB(Vec2 lp, Vec2 plp,
												 const Transformation& xfA, const Transformation& xfB,
												 UnitVec2 ln)
{
	const auto planePoint = Transform(lp, xfB);
	const auto clipPoint = Transform(plp, xfA);
	const auto normal = Rotate(ln, xfB.q);
	const auto separation = Dot(clipPoint - planePoint, normal);
	// Negate normal to ensure the PSM normal points from A to B
	return PositionSolverManifold{-normal, clipPoint, separation};
}

PositionSolverManifold GetPSM(const Manifold& manifold, Manifold::size_type index,
							  const Transformation& xfA, const Transformation& xfB)
{
	assert(manifold.GetType() != Manifold::e_unset);
	assert(manifold.GetPointCount() > 0);
	
	switch (manifold.GetType())
	{
		case Manifold::e_circles:
			return GetForCircles(manifold.GetLocalPoint(), manifold.GetPoint(index).localPoint,
								 xfA, xfB);
		case Manifold::e_faceA:
			return GetForFaceA(manifold.GetLocalPoint(), manifold.GetPoint(index).localPoint,
							   xfA, xfB, manifold.GetLocalNormal());
		case Manifold::e_faceB:
			return GetForFaceB(manifold.GetLocalPoint(), manifold.GetPoint(index).localPoint,
							   xfA, xfB, manifold.GetLocalNormal());
		case Manifold::e_unset:
			break;
	}
	// should not be reached
	return PositionSolverManifold{GetInvalid<UnitVec2>(), GetInvalid<Vec2>(), GetInvalid<float_t>()};
}

}; // namespace box2d
