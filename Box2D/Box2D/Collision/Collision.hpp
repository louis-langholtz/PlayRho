/*
 * Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

#ifndef B2_COLLISION_H
#define B2_COLLISION_H

#include <Box2D/Common/Math.hpp>
#include <Box2D/Common/ArrayList.hpp>
#include <Box2D/Collision/ContactFeature.hpp>

#include <array>
#include <type_traits>

namespace box2d
{

/// @file
/// Structures and functions used for computing contact points, distance
/// queries, and TOI queries.

class Manifold;
	
/// This is used for determining the state of contact points.
enum class PointState
{
	NullState,		///< point does not exist
	AddState,		///< point was added in the update
	PersistState,	///< point persisted across the update
	RemoveState		///< point was removed in the update
};

using PointStateArray = std::array<PointState,MaxManifoldPoints>;
	
/// Computes the point states given two manifolds. The states pertain to the transition from manifold1
/// to manifold2. So state1 is either persist or remove while state2 is either add or persist.
void GetPointStates(PointStateArray& state1, PointStateArray& state2,
					const Manifold& manifold1, const Manifold& manifold2);

/// Used for computing contact manifolds.
/// @note This data structure is 12-bytes large (on at least one 64-bit platform).
struct ClipVertex
{
	Length2D v; ///< Vertex of edge or polygon. 8-bytes.
	ContactFeature cf; ///< Contact feature information. 4-bytes.
};

/// Clip list for ClipSegmentToLine.
/// @sa ClipSegmentToLine.
/// @note This data structure is at least 24-bytes large.
using ClipList = ArrayList<ClipVertex, MaxManifoldPoints>;

/// Clipping for contact manifolds.
/// @details This returns an array of points from the given line that are inside of the plane as
///   defined by a given normal and offset.
/// @param vIn Clip list of two points defining the line.
/// @param normal Normal of the plane with which to determine intersection.
/// @param offset Offset of the plane with which to determine intersection.
/// @param indexA Index of vertex A.
/// @return List of zero one or two clip points.
ClipList ClipSegmentToLine(const ClipList& vIn, const UnitVec2& normal, Length offset,
						   ContactFeature::index_t indexA);

// ---------------- Inline Functions ------------------------------------------

} /* namespace box2d */

#endif
