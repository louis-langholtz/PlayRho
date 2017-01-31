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

#ifndef Vec2_hpp
#define Vec2_hpp

#include <Box2D/Common/UnitVec2.hpp>

namespace box2d
{
	/// Vector 2D.
	/// @note This data structure is two-times the size of the <code>RealNum</code> type.
	/// This is two times 4-bytes for a total of 8-bytes (on at least one 64-bit platform).
	struct Vec2
	{
		using size_type = size_t;
		using data_type = RealNum;
		
		/// Default constructor does nothing (for performance).
		Vec2() noexcept = default;
		
		Vec2(const Vec2& copy) noexcept = default;
		
		/// Construct using coordinates.
		constexpr Vec2(data_type x_, data_type y_) noexcept : x{x_}, y{y_} {}
		
		constexpr explicit Vec2(const UnitVec2 unitvector) noexcept: x{unitvector.GetX()}, y{unitvector.GetY()} {}
		
		/// Negate this vector.
		constexpr auto operator- () const noexcept { return Vec2{-x, -y}; }
		
		/// Maximum size.
		/// @detail This is this vector type's dimensionality.
		constexpr size_type max_size() const noexcept { return 2; }
		
		/// Accesses element by index.
		/// @param i Index (0 for x, 1 for y).
		auto operator[] (size_type i) const
		{
			assert(i < max_size());
			switch (i)
			{
				case 0: return x;
				case 1: return y;
				default: break;
			}
			return x;
		}
		
		/// Accesses element by index.
		/// @param i Index (0 for x, 1 for y).
		auto& operator[] (size_type i)
		{
			assert(i < max_size());
			switch (i)
			{
				case 0: return x;
				case 1: return y;
				default: break;
			}
			return x;
		}
		
		data_type x, y;
	};
	
	constexpr inline Vec2::data_type GetX(const Vec2 value)
	{
		return value.x;
	}
	
	constexpr inline Vec2::data_type GetY(const Vec2 value)
	{
		return value.y;
	}

} // namespace box2d

#endif /* Vec2_hpp */
