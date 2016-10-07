/*
 * Original work Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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

#ifndef PositionSolverManifold_hpp
#define PositionSolverManifold_hpp

#include <Box2D/Common/Settings.h>
#include <Box2D/Common/Math.h>

#include <Box2D/Collision/Manifold.hpp>

namespace box2d
{
	/// Position solver manifold.
	/// @detail
	/// This is a normal-point-separation composition of data.
	struct PositionSolverManifold
	{
		PositionSolverManifold() noexcept = default;
		PositionSolverManifold(const PositionSolverManifold& copy) noexcept = default;
		
		constexpr PositionSolverManifold(Vec2 n, Vec2 p, float_t s) noexcept:
			normal{n}, point{p}, separation{s} {}
		
		Vec2 normal; ///< Normal.
		Vec2 point; ///< Point.
		float_t separation; ///< "separation" between two points (of a contact position constraint).
	};

	/// Gets the normal-point-separation data for the given inputs.
	PositionSolverManifold GetPSM(const Manifold& manifold, Manifold::size_type index,
								  const Transformation& xfA, const Transformation& xfB);
	
	inline PositionSolverManifold GetPSM(const Manifold& manifold, Manifold::size_type index,
										 Position pos_a, Vec2 lc_ctr_a,
										 Position pos_b, Vec2 lc_ctr_b)
	{
		const auto xfA = GetTransformation(pos_a, lc_ctr_a);
		const auto xfB = GetTransformation(pos_b, lc_ctr_b);
		return GetPSM(manifold, index, xfA, xfB);
	}
	
}; // namespace box2d

#endif /* PositionSolverManifold_hpp */
