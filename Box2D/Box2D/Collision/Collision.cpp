/*
* Original work Copyright (c) 2007-2009 Erin Catto http://www.box2d.org
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

#include <Box2D/Collision/Collision.hpp>
#include <Box2D/Collision/Manifold.hpp>

namespace box2d {

void GetPointStates(PointStateArray& state1, PointStateArray& state2,
					const Manifold& manifold1, const Manifold& manifold2)
{
	for (auto i = decltype(MaxManifoldPoints){0}; i < MaxManifoldPoints; ++i)
	{
		state1[i] = PointState::NullState;
		state2[i] = PointState::NullState;
	}

	// Detect persists and removes.
	for (auto i = decltype(manifold1.GetPointCount()){0}; i < manifold1.GetPointCount(); ++i)
	{
		const auto cf = manifold1.GetContactFeature(i);

		state1[i] = PointState::RemoveState;

		for (auto j = decltype(manifold2.GetPointCount()){0}; j < manifold2.GetPointCount(); ++j)
		{
			if (manifold2.GetContactFeature(j) == cf)
			{
				state1[i] = PointState::PersistState;
				break;
			}
		}
	}

	// Detect persists and adds.
	for (auto i = decltype(manifold2.GetPointCount()){0}; i < manifold2.GetPointCount(); ++i)
	{
		const auto cf = manifold2.GetContactFeature(i);

		state2[i] = PointState::AddState;

		for (auto j = decltype(manifold1.GetPointCount()){0}; j < manifold1.GetPointCount(); ++j)
		{
			if (manifold1.GetContactFeature(j) == cf)
			{
				state2[i] = PointState::PersistState;
				break;
			}
		}
	}
}

ClipList ClipSegmentToLine(const ClipList& vIn, const UnitVec2& normal, float_t offset,
						   ContactFeature::index_t indexA)
{
	ClipList vOut;

	if (vIn.size() == 2) // must have two points (for a segment)
	{
		// Use Sutherland-Hodgman clipping (https://en.wikipedia.org/wiki/Sutherland%E2%80%93Hodgman_algorithm ).
		
		// Calculate the distance of end points to the line
		const auto distance0 = Dot(normal, vIn[0].v) - offset; ///< Magnitude of vIn[0].v in direction of normal minus offset.
		const auto distance1 = Dot(normal, vIn[1].v) - offset; ///< Magnitude of vIn[1].v in direction of normal minus offset.

		// If the points are behind the plane
		if (distance0 <= 0)
		{
			vOut.add(vIn[0]);
		}
		if (distance1 <= 0)
		{
			vOut.add(vIn[1]);
		}

		// If the points are on different sides of the plane
		if ((distance0 * distance1) < 0)
		{
			// Neither distance0 nor distance1 is 0 and either distance0 or distance1 is negative (but not both).
			// Find intersection point of edge and plane
			// Vertex A is hitting edge B.
			const auto interp = distance0 / (distance0 - distance1);
			const auto vertex = vIn[0].v + (vIn[1].v - vIn[0].v) * interp;
			vOut.add(ClipVertex{vertex, GetVertexFaceContactFeature(indexA, vIn[0].cf.indexB)});
			// XXX: Maybe one of the alternatives would be more appropriate??
			//vOut.add(ClipVertex{vertex, GetFaceFaceContactFeature(indexA, vIn[0].cf.indexB)});
			//vOut.add(ClipVertex{vertex, ContactFeature{ContactFeature::e_face, indexA, vIn[0].cf.typeB, vIn[0].cf.indexB}});
		}
	}

	return vOut;
}

} // namespace box2d
