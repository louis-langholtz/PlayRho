//
//  PositionSolverManifold.hpp
//  Box2D
//
//  Created by Louis D. Langholtz on 6/23/16.
//
//

#ifndef PositionSolverManifold_hpp
#define PositionSolverManifold_hpp

#include <Box2D/Common/Settings.h>
#include <Box2D/Common/Math.h>

#include <Box2D/Dynamics/Contacts/ContactSolver.h>

namespace box2d
{

struct PositionSolverManifold
{
	PositionSolverManifold() noexcept = default;
	PositionSolverManifold(const PositionSolverManifold& copy) noexcept = default;
	
	constexpr PositionSolverManifold(Vec2 n, Vec2 p, float_t s) noexcept: normal{n}, point{p}, separation{s} {}
	
	Vec2 normal; ///< Normal.
	Vec2 point; ///< Point.
	float_t separation; ///< "separation" between two points (of a contact position constraint).
};

PositionSolverManifold GetPSM(const Manifold& manifold, float_t totalRadius,
							  const Transformation& xfA, const Transformation& xfB, Manifold::size_type index);

}; // namespace box2d

#endif /* PositionSolverManifold_hpp */
