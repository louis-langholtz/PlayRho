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

namespace
{

/// Gets the position solver manifold in world coordinates for a circles-type manifold.
/// @param xfA Transformation for body A.
/// @param lp Local point. Location of shape A in local coordinates.
/// @param xfB Transformation for body B.
/// @param plp Point's local point. Location of shape B in local coordinates.
/// @note The returned separation is the magnitude of the positional difference of the two points.
///   This is always a non-negative amount.
inline PositionSolverManifold GetForCircles(const Transformation& xfA, Vec2 lp,
											const Transformation& xfB, Vec2 plp)
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
/// @param xfA Transformation for shape A.
/// @param lp Local point. Location for shape A in local coordinates.
/// @param ln Local normal for shape A to be transformed into a world normal based on the
///   transformation for shape A.
/// @param xfB Transformation for shape B.
/// @param plp Point's local point. Location for shape B in local coordinates.
/// @return Separation is the dot-product of the positional difference between the two points in
///   the direction of the world normal.
inline PositionSolverManifold GetForFaceA(const Transformation& xfA, Vec2 lp, UnitVec2 ln,
										  const Transformation& xfB, Vec2 plp)
{
	const auto planePoint = Transform(lp, xfA);
	const auto normal = Rotate(ln, xfA.q);
	const auto clipPoint = Transform(plp, xfB);
	const auto separation = Dot(clipPoint - planePoint, normal);
	return PositionSolverManifold{normal, clipPoint, separation};
}

/// Gets the position solver manifold in world coordinates for a face-b-type manifold.
/// @param xfB Transformation for body B.
/// @param lp Local point.
/// @param ln Local normal for shape B to be transformed into a world normal based on the
///   transformation for shape B.
/// @param xfA Transformation for body A.
/// @param plp Point's local point. Location for shape A in local coordinates.
/// @return Separation is the dot-product of the positional difference between the two points in
///   the direction of the world normal.
inline PositionSolverManifold GetForFaceB(const Transformation& xfB, Vec2 lp, UnitVec2 ln,
										  const Transformation& xfA, Vec2 plp)
{
	const auto planePoint = Transform(lp, xfB);
	const auto normal = Rotate(ln, xfB.q);
	const auto clipPoint = Transform(plp, xfA);
	const auto separation = Dot(clipPoint - planePoint, normal);
	// Negate normal to ensure the PSM normal points from A to B
	return PositionSolverManifold{-normal, clipPoint, separation};
}

} // unnamed namespace

PositionSolverManifold GetPSM(const Manifold& m, Manifold::size_type idx,
							  const Transformation& xfA, const Transformation& xfB)
{
	assert(m.GetType() != Manifold::e_unset);
	assert(m.GetPointCount() > 0);
	
	switch (m.GetType())
	{
	case Manifold::e_circles:
		return GetForCircles(xfA, m.GetLocalPoint(), xfB, m.GetPoint(idx).localPoint);
	case Manifold::e_faceA:
		return GetForFaceA(xfA, m.GetLocalPoint(), m.GetLocalNormal(), xfB, m.GetPoint(idx).localPoint);
	case Manifold::e_faceB:
		return GetForFaceB(xfB, m.GetLocalPoint(), m.GetLocalNormal(), xfA, m.GetPoint(idx).localPoint);
	case Manifold::e_unset:
		break;
	}

	// should not be reached
	return PositionSolverManifold{GetInvalid<UnitVec2>(), GetInvalid<Vec2>(), GetInvalid<float_t>()};
}

}; // namespace box2d
