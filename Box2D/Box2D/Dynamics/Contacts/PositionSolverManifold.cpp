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

static inline PositionSolverManifold GetForCircles(Vec2 lp, Vec2 plp,
												   const Transformation& xfA, const Transformation& xfB)
{
	const auto pointA = Transform(lp, xfA);
	const auto pointB = Transform(plp, xfB);
	const auto delta = pointB - pointA;
	const auto normal = GetUnitVector(delta);
	const auto point = (pointA + pointB) / float_t{2};
	const auto separation = Dot(delta, normal);
	return PositionSolverManifold{normal, point, separation};
}

static inline PositionSolverManifold GetForFaceA(Vec2 lp, Vec2 plp,
												 const Transformation& xfA, const Transformation& xfB,
												 Vec2 ln)
{
	const auto planePoint = Transform(lp, xfA);
	const auto clipPoint = Transform(plp, xfB);
	const auto normal = Rotate(ln, xfA.q);
	const auto separation = Dot(clipPoint - planePoint, normal);
	return PositionSolverManifold{normal, clipPoint, separation};
}

static inline PositionSolverManifold GetForFaceB(Vec2 lp, Vec2 plp,
												 const Transformation& xfA, const Transformation& xfB,
												 Vec2 ln)
{
	const auto planePoint = Transform(lp, xfB);
	const auto clipPoint = Transform(plp, xfA);
	const auto normal = Rotate(ln, xfB.q);
	const auto separation = Dot(clipPoint - planePoint, normal);
	// Negate normal to ensure the PSM normal points from A to B
	return PositionSolverManifold{-normal, clipPoint, separation};
}

PositionSolverManifold GetPSM(const Manifold& manifold,
							  const Transformation& xfA, const Transformation& xfB,
							  Manifold::size_type index)
{
	assert(manifold.GetType() != Manifold::e_unset);
	assert(manifold.GetPointCount() > 0);
	
	// Note for valid manifold types:
	//   Sum the radius values and subtract this sum to reduce FP losses in cases where the radius
	//   values would otherwise be insignificant compared to the values being subtracted from.
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
	return PositionSolverManifold{Vec2_zero, Vec2_zero, 0};
}

}; // namespace box2d