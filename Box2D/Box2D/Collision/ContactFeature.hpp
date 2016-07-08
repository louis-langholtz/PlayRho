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

#ifndef ContactFeature_hpp
#define ContactFeature_hpp

#include <Box2D/Common/Math.h>

namespace box2d
{
	/// Contact Feature.
	/// @detail The features that intersect to form the contact point.
	/// @note This structure is designed to be compact and passed-by-value.
	struct ContactFeature
	{
		using index_t = uint8; ///< Index type.
		
		enum Type: uint8
		{
			e_vertex = 0,
			e_face = 1
		};
		
		ContactFeature() noexcept = default;
		ContactFeature(const ContactFeature& copy) noexcept = default;
		
		constexpr ContactFeature(Type ta, index_t ia, Type tb, index_t ib) noexcept:
			typeA{ta}, indexA{ia}, typeB{tb}, indexB{ib}
		{
			static_assert(sizeof(struct ContactFeature) == 4, "bad size");
		}
		
		// Fit data into 4-byte large structure...
		
		Type typeA; ///< The feature type on shape A
		Type typeB; ///< The feature type on shape B
		index_t indexA; ///< Feature index on shape A
		index_t indexB; ///< Feature index on shape B
	};
	
	/// Default contact feature value.
	constexpr auto DefaultContactFeature = ContactFeature{ContactFeature::e_vertex, 0, ContactFeature::e_vertex, 0};
	
	/// Flips contact features information.
	constexpr ContactFeature Flip(ContactFeature val)
	{
		return ContactFeature{val.typeB, val.indexB, val.typeA, val.indexA};
	}
	
	constexpr bool operator==(ContactFeature lhs, ContactFeature rhs)
	{
		return (lhs.typeA == rhs.typeA) && (lhs.typeB == rhs.typeB) && (lhs.indexA == rhs.indexA) && (lhs.indexB == rhs.indexB);
	}

	constexpr bool operator!=(ContactFeature lhs, ContactFeature rhs)
	{
		return (lhs.typeA != rhs.typeA) || (lhs.typeB != rhs.typeB) || (lhs.indexA != rhs.indexA) || (lhs.indexB != rhs.indexB);
	}
}; // namespace box2d

#endif /* ContactFeature_hpp */
