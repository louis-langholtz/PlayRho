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

#ifndef B2_COLLISION_H
#define B2_COLLISION_H

#include <Box2D/Common/Math.h>
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

/// Compute the point states given two manifolds. The states pertain to the transition from manifold1
/// to manifold2. So state1 is either persist or remove while state2 is either add or persist.
using PointStateArray = std::array<PointState,MaxManifoldPoints>;
void GetPointStates(PointStateArray& state1, PointStateArray& state2,
					  const Manifold& manifold1, const Manifold& manifold2);

/// Used for computing contact manifolds.
struct ClipVertex
{
	Vec2 v; ///< Vertex of edge or polygon.
	ContactFeature cf; ///< Contact feature information.
};

/// Clip array for ClipSegmentToLine.
/// @see ClipSegmentToLine.
using ClipArray = std::array<ClipVertex, MaxManifoldPoints>;

class ClipList
{
public:
	using size_type = std::remove_const<decltype(MaxManifoldPoints)>::type;
	using data_type = ClipVertex;
	using pointer = data_type*;
	using const_pointer = const data_type*;

	bool add(data_type value)
	{
		if (m_size < MaxManifoldPoints)
		{
			m_clips[m_size] = value;
			++m_size;
			return true;
		}
		return false;
	}
	
	data_type& operator[](size_t index)
	{
		assert(index < MaxManifoldPoints);
		return m_clips[index];
	}

	data_type operator[](size_t index) const
	{
		assert(index < MaxManifoldPoints);
		return m_clips[index];
	}

	size_type size() const noexcept { return m_size; }
	
	pointer begin() noexcept { return &m_clips[0]; }
	pointer end() noexcept { return &m_clips[0] + m_size; }

	const_pointer begin() const noexcept { return &m_clips[0]; }
	const_pointer end() const noexcept { return &m_clips[0] + m_size; }

private:
	ClipArray m_clips;
	size_type m_size = size_type{0};
};
	
/// Clipping for contact manifolds.
/// @detail This returns an array of points from the given line that are inside of the plane as
///   defined by a given normal and offset.
/// @param vIn Clip list of two points defining the line.
/// @param normal Normal of the plane with which to determine intersection.
/// @param offset Offset of the plane with which to determine intersection.
/// @param indexA Index of vertex A.
/// @return List of zero one or two clip points.
ClipList ClipSegmentToLine(const ClipList& vIn, const Vec2& normal, float_t offset,
						   ContactFeature::index_t indexA);

// ---------------- Inline Functions ------------------------------------------

} /* namespace box2d */

#endif
