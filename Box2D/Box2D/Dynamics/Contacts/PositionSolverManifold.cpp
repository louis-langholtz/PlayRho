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

/// Gets the position solver manifold for a circles-type manifold.
/// @param lp Local point.
/// @param plp Point's local point.
/// @param xfA Transformation for body A.
/// @param xfB Transformation for body B.
/// @return Separation is the dot-product of:
///   the positional difference of the two bodies, and
///   the unit vector of that positional difference.
static inline PositionSolverManifold GetForCircles(Vec2 lp, Vec2 plp,
												   const Transformation& xfA, const Transformation& xfB)
{
	const auto pointA = Transform(lp, xfA);
	const auto pointB = Transform(plp, xfB);
	const auto delta = pointB - pointA;
	const auto normal = GetUnitVector(delta, UnitVec2::GetZero());
	const auto point = (pointA + pointB) / float_t{2};
	const auto separation = Dot(delta, normal);
	return PositionSolverManifold{normal, point, separation};
}

/// Gets the position solver manifold for a face-a-type manifold.
/// @param lp Local point.
/// @param plp Point's local point.
/// @param xfA Transformation for body A.
/// @param xfB Transformation for body B.
/// @param ln Local normal.
/// @return Separation is the dot-product of:
///   the positional difference between the two bodies, and
///   the local normal rotated by the rotational component of xfA.
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

/// Gets the position solver manifold for a face-b-type manifold.
/// @param lp Local point.
/// @param plp Point's local point.
/// @param xfA Transformation for body A.
/// @param xfB Transformation for body B.
/// @param ln Local normal.
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
