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

#ifndef IndexPair_hpp
#define IndexPair_hpp

#include <Box2D/Common/Settings.h>

namespace box2d
{
	
	/// Index pair.
	/// @note This data structure is at least 2-bytes large.
	struct IndexPair
	{
		/// Size type.
		/// @detail Must be big enough to hold max posible count of vertices.
		using size_type = std::remove_const<decltype(MaxShapeVertices)>::type;
		
		static constexpr size_type InvalidIndex = static_cast<size_type>(-1);
		
		size_type a; ///< Index of vertex from shape A.
		size_type b; ///< Index of vertex from shape B.
	};
	
	constexpr inline bool operator == (IndexPair lhs, IndexPair rhs)
	{
		return (lhs.a == rhs.a) && (lhs.b == rhs.b);
	}
	
	constexpr inline bool operator != (IndexPair lhs, IndexPair rhs)
	{
		return (lhs.a != rhs.a) || (lhs.b != rhs.b);
	}

};

#endif /* IndexPair_hpp */
