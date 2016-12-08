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

#ifndef ShapeSeparation_hpp
#define ShapeSeparation_hpp

#include <Box2D/Common/Math.hpp>
#include <Box2D/Common/Span.hpp>

namespace box2d
{
	// This structure is used to keep track of the best separating axis.
	struct ShapeSeparation
	{
		using index_type = std::remove_const<decltype(MaxShapeVertices)>::type;
		using distance_type = float_t;
		
		static constexpr index_type InvalidIndex = static_cast<index_type>(-1);
		static constexpr distance_type InvalidDistance = GetInvalid<distance_type>();
		
		index_type index = InvalidIndex;
		distance_type separation = InvalidDistance;
	};
	
	/// Gets the shape separation information for the most opposite vector.
	/// @param vectors Collection of 0 or more vectors to find the most anti-parallel vector from and
	///    its magnitude from the reference vector.
	/// @param refvec Reference vector.
	template <typename T>
	static inline ShapeSeparation GetMostOppositeSeparation(Span<const T> vectors, const T refvec, const T offset)
	{
		// Search for the vector that is most anti-parallel to the reference vector.
		// See: https://en.wikipedia.org/wiki/Antiparallel_(mathematics)#Antiparallel_vectors
		auto bestIndex = ShapeSeparation::InvalidIndex;
		auto minValue = MaxFloat;
		const auto count = vectors.size();
		for (auto i = decltype(count){0}; i < count; ++i)
		{
			// Get cosine of angle between refvec and vectors[i] multiplied by their
			// magnitudes (which will essentially be 1 for any two unit vectors).
			// Get distance from offset to vectors[i] in direction of refvec.
			const auto s = Dot(refvec, vectors[i] - offset);
			if (minValue > s)
			{
				minValue = s;
				bestIndex = static_cast<ShapeSeparation::index_type>(i);
			}
		}
		return ShapeSeparation{bestIndex, minValue};
	}

	/// Gets the max separation information.
	/// @return The index of the vertex and normal from vertices1 and normals1
	///   that had the maximum separation distance from any vertex in vertices2 in the
	///   direction of that normal and that maximal distance.
	ShapeSeparation	GetMaxSeparation(Span<const Vec2> verts1, Span<const UnitVec2> norms1, const Transformation& xf1,
									 Span<const Vec2> verts2, const Transformation& xf2);

} // namespace box2d

#endif /* ShapeSeparation_hpp */
