//
//  PositionSolverManifold.cpp
//  Box2D
//
//  Created by Louis D. Langholtz on 6/23/16.
//
//

#include <Box2D/Dynamics/Contacts/PositionSolverManifold.hpp>
#include <Box2D/Dynamics/Contacts/ContactSolver.h>

namespace box2d
{

static inline PositionSolverManifold GetPSM_ForCircles(const ContactPositionConstraint& pc,
													   const Transform& xfA, const Transform& xfB,
													   PositionSolverManifold::index_t index)
{
	assert(index == 0);
	const auto pointA = Mul(xfA, pc.manifold.GetLocalPoint());
	const auto pointB = Mul(xfB, pc.manifold.GetPoint(index).localPoint);
	const auto delta = pointB - pointA;
	const auto normal = Normalize(delta);
	const auto point = (pointA + pointB) / float_t{2};
	const auto totalRadius = pc.radiusA + pc.radiusB;
	const auto separation = Dot(delta, normal) - totalRadius;
	return PositionSolverManifold{normal, point, separation};
}

static inline PositionSolverManifold GetPSM_ForFaceA(const ContactPositionConstraint& pc,
													 const Transform& xfA, const Transform& xfB,
													 PositionSolverManifold::index_t index)
{
	const auto planePoint = Mul(xfA, pc.manifold.GetLocalPoint());
	const auto clipPoint = Mul(xfB, pc.manifold.GetPoint(index).localPoint);
	const auto normal = Mul(xfA.q, pc.manifold.GetLocalNormal());
	const auto totalRadius = pc.radiusA + pc.radiusB;
	const auto separation = Dot(clipPoint - planePoint, normal) - totalRadius;
	return PositionSolverManifold{normal, clipPoint, separation};
}

static inline PositionSolverManifold GetPSM_ForFaceB(const ContactPositionConstraint& pc,
											   const Transform& xfA, const Transform& xfB,
											   PositionSolverManifold::index_t index)
{
	const auto planePoint = Mul(xfB, pc.manifold.GetLocalPoint());
	const auto clipPoint = Mul(xfA, pc.manifold.GetPoint(index).localPoint);
	const auto normal = Mul(xfB.q, pc.manifold.GetLocalNormal());
	const auto totalRadius = pc.radiusA + pc.radiusB;
	const auto separation = Dot(clipPoint - planePoint, normal) - totalRadius;
	// Negate normal to ensure the PSM normal points from A to B
	return PositionSolverManifold{-normal, clipPoint, separation};
}

PositionSolverManifold GetPSM(const ContactPositionConstraint& pc,
							  const Transform& xfA, const Transform& xfB,
							  PositionSolverManifold::index_t index)
{
	assert(pc.manifold.GetType() != Manifold::e_unset);
	assert(pc.manifold.GetPointCount() > 0);
	
	// Note for valid manifold types:
	//   Sum the radius values and subtract this sum to reduce FP losses in cases where the radius
	//   values would otherwise be insignificant compared to the values being subtracted from.
	switch (pc.manifold.GetType())
	{
		case Manifold::e_circles: return GetPSM_ForCircles(pc, xfA, xfB, index);
		case Manifold::e_faceA: return GetPSM_ForFaceA(pc, xfA, xfB, index);
		case Manifold::e_faceB: return GetPSM_ForFaceB(pc, xfA, xfB, index);
		case Manifold::e_unset: break;
	}
	// should not be reached
	return PositionSolverManifold{Vec2_zero, Vec2_zero, 0};
}

}; // namespace box2d