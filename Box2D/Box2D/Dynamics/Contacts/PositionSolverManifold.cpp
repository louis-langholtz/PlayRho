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

static inline PositionSolverManifold GetPSM_ForCircles(Vec2 lp, Vec2 plp, float_t totalRadius,
													   const Transform& xfA, const Transform& xfB)
{
	const auto pointA = Mul(xfA, lp);
	const auto pointB = Mul(xfB, plp);
	const auto delta = pointB - pointA;
	const auto normal = GetUnitVector(delta);
	const auto point = (pointA + pointB) / float_t{2};
	const auto separation = Dot(delta, normal) - totalRadius;
	return PositionSolverManifold{normal, point, separation};
}

static inline PositionSolverManifold GetPSM_ForFaceA(Vec2 lp, Vec2 plp, float_t totalRadius,
													 const Transform& xfA, const Transform& xfB,
													 Vec2 ln)
{
	const auto planePoint = Mul(xfA, lp);
	const auto clipPoint = Mul(xfB, plp);
	const auto normal = Rotate(xfA.q, ln);
	const auto separation = Dot(clipPoint - planePoint, normal) - totalRadius;
	return PositionSolverManifold{normal, clipPoint, separation};
}

static inline PositionSolverManifold GetPSM_ForFaceB(Vec2 lp, Vec2 plp, float_t totalRadius,
													 const Transform& xfA, const Transform& xfB,
													 Vec2 ln)
{
	const auto planePoint = Mul(xfB, lp);
	const auto clipPoint = Mul(xfA, plp);
	const auto normal = Rotate(xfB.q, ln);
	const auto separation = Dot(clipPoint - planePoint, normal) - totalRadius;
	// Negate normal to ensure the PSM normal points from A to B
	return PositionSolverManifold{-normal, clipPoint, separation};
}

PositionSolverManifold GetPSM(const Manifold& manifold, float_t totalRadius,
							  const Transform& xfA, const Transform& xfB,
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
			return GetPSM_ForCircles(manifold.GetLocalPoint(), manifold.GetPoint(index).localPoint,
									 totalRadius, xfA, xfB);
		case Manifold::e_faceA:
			return GetPSM_ForFaceA(manifold.GetLocalPoint(), manifold.GetPoint(index).localPoint,
								   totalRadius, xfA, xfB, manifold.GetLocalNormal());
		case Manifold::e_faceB:
			return GetPSM_ForFaceB(manifold.GetLocalPoint(), manifold.GetPoint(index).localPoint,
								   totalRadius, xfA, xfB, manifold.GetLocalNormal());
		case Manifold::e_unset:
			break;
	}
	// should not be reached
	return PositionSolverManifold{Vec2_zero, Vec2_zero, 0};
}

}; // namespace box2d