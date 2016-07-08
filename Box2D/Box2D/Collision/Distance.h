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

#ifndef B2_DISTANCE_H
#define B2_DISTANCE_H

#include <Box2D/Common/Math.h>
#include <Box2D/Collision/DistanceProxy.hpp>
#include <Box2D/Collision/SimplexCache.hpp>

namespace box2d
{

struct WitnessPoints
{
	Vec2 a;
	Vec2 b;
};
	
/// Output for Distance.
struct DistanceOutput
{
	DistanceOutput() = default;
	DistanceOutput(const DistanceOutput& copy) = default;

	constexpr DistanceOutput(const WitnessPoints& wp, unsigned it) noexcept: witnessPoints{wp}, iterations{it} {}

	WitnessPoints witnessPoints; ///< closest point on shapeA and closest point on shapeB
	unsigned iterations;	///< number of GJK iterations used
};

/// Determines the closest points between two shapes.
/// @detail
/// Supports any combination of:
/// CircleShape, PolygonShape, EdgeShape. The simplex cache is input/output.
/// @note On the first call, the SimplexCache.count should be set to zero.
/// @param cache Simplex cache for assisting the determination.
/// @param proxyA Proxy A.
/// @param transformA Transoform of A.
/// @param proxyB Proxy B.
/// @param transformB Transoform of B.
/// @return Closest points between the two shapes and the count of iterations it took to determine them.
DistanceOutput Distance(SimplexCache& cache,
						const DistanceProxy& proxyA, const Transformation& transformA,
						const DistanceProxy& proxyB, const Transformation& transformB);

} /* namespace box2d */

#endif
