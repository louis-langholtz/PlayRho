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

namespace box2d
{

struct PositionSolverManifold
{
	using index_t = std::remove_const<decltype(MaxManifoldPoints)>::type;
	
	PositionSolverManifold() noexcept = default;
	PositionSolverManifold(const PositionSolverManifold& copy) noexcept = default;
	
	constexpr PositionSolverManifold(Vec2 n, Vec2 p, float_t s) noexcept: normal{n}, point{p}, separation{s} {}
	
	Vec2 normal; ///< Normal.
	Vec2 point; ///< Point.
	float_t separation; ///< "separation" between two points (of a contact position constraint).
};

class Manifold;

PositionSolverManifold GetPSM(const Manifold& manifold, float_t totalRadius,
							  const Transform& xfA, const Transform& xfB, PositionSolverManifold::index_t index);

}; // namespace box2d

#endif /* PositionSolverManifold_hpp */
