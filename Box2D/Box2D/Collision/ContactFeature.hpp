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

#ifndef ContactFeature_hpp
#define ContactFeature_hpp

#include <Box2D/Common/Math.hpp>

namespace box2d
{
	/// Contact Feature.
	/// @detail The features that intersect to form the contact point.
	/// @note This structure is designed to be compact and passed-by-value.
	/// @note This data structure is 4-bytes large.
	struct ContactFeature
	{
		using index_t = uint8; ///< Index type.
		
		enum Type: uint8
		{
			e_vertex = 0,
			e_face = 1
		};
				
		// Fit data into 4-byte large structure...
		
		Type typeA; ///< The feature type on shape A
		index_t indexA; ///< Feature index on shape A
		Type typeB; ///< The feature type on shape B
		index_t indexB; ///< Feature index on shape B
	};
	
	constexpr ContactFeature GetVertexVertexContactFeature(ContactFeature::index_t a, ContactFeature::index_t b) noexcept
	{
		return ContactFeature{ContactFeature::e_vertex, a, ContactFeature::e_vertex, b};
	}

	constexpr ContactFeature GetVertexFaceContactFeature(ContactFeature::index_t a, ContactFeature::index_t b) noexcept
	{
		return ContactFeature{ContactFeature::e_vertex, a, ContactFeature::e_face, b};
	}
	
	constexpr ContactFeature GetFaceVertexContactFeature(ContactFeature::index_t a, ContactFeature::index_t b) noexcept
	{
		return ContactFeature{ContactFeature::e_face, a, ContactFeature::e_vertex, b};
	}
	
	constexpr ContactFeature GetFaceFaceContactFeature(ContactFeature::index_t a, ContactFeature::index_t b) noexcept
	{
		return ContactFeature{ContactFeature::e_face, a, ContactFeature::e_face, b};
	}
		
	/// Flips contact features information.
	constexpr ContactFeature Flip(ContactFeature val) noexcept
	{
		return ContactFeature{val.typeB, val.indexB, val.typeA, val.indexA};
	}
	
	constexpr bool operator==(ContactFeature lhs, ContactFeature rhs) noexcept
	{
		return (lhs.typeA == rhs.typeA) && (lhs.typeB == rhs.typeB) && (lhs.indexA == rhs.indexA) && (lhs.indexB == rhs.indexB);
	}

	constexpr bool operator!=(ContactFeature lhs, ContactFeature rhs) noexcept
	{
		return (lhs.typeA != rhs.typeA) || (lhs.typeB != rhs.typeB) || (lhs.indexA != rhs.indexA) || (lhs.indexB != rhs.indexB);
	}
	
}; // namespace box2d

#endif /* ContactFeature_hpp */
